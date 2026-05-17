#![no_std]
#![no_main]

use defmt_rtt as _;
use embedded_hal::i2c::I2c;
use hal::uart::{DataBits, StopBits, UartConfig};
use panic_probe as _;
use rp_pico::entry;
use rp_pico::hal::fugit::RateExtU32;
use rp_pico::hal::{self, clocks::Clock, pac};

//-------------------------
// BNO055 Constants
//-------------------------
const BNO055_ADDR: u8 = 0x28;
const CHIP_ID_REG: u8 = 0x00;
const OPR_MODE_REG: u8 = 0x3D;

// Raw data registers (available in all modes)
const ACCEL_REG: u8 = 0x08;
const MAG_REG: u8 = 0x0E;
const GYRO_REG: u8 = 0x14;

// Fusion output registers (NDOF mode only)
const EULER_REG: u8 = 0x1A; // 6 bytes: H_LSB, H_MSB, R_LSB, R_MSB, P_LSB, P_MSB
const TEMP_REG: u8 = 0x34; // 1 byte, °C
const CALIB_REG: u8 = 0x35; // 1 byte, calibration status

//-------------------------
// BNO055 Operating Modes
//-------------------------
const CONFIG_MODE: u8 = 0x00;
const NDOF_MODE: u8 = 0x0C; // Full fusion — raw + euler + temp

//-------------------------
// Sampling
//-------------------------
const SAMPLE_PERIOD_US: u64 = 10_000; // 100Hz

//-------------------------
// UART Writer
//-------------------------
struct UartWriter<'a> {
    buf: &'a mut [u8],
    pos: usize,
}

impl<'a> UartWriter<'a> {
    fn new(buf: &'a mut [u8]) -> Self {
        UartWriter { buf, pos: 0 }
    }
    fn as_bytes(&self) -> &[u8] {
        &self.buf[..self.pos]
    }
}

impl<'a> core::fmt::Write for UartWriter<'a> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let bytes = s.as_bytes();
        let len = bytes.len().min(self.buf.len() - self.pos);
        self.buf[self.pos..self.pos + len].copy_from_slice(&bytes[..len]);
        self.pos += len;
        Ok(())
    }
}

macro_rules! uart_println {
    ($uart:expr, $buf:expr, $($arg:tt)*) => {{
        let mut w = UartWriter::new($buf);
        core::fmt::write(&mut w, format_args!($($arg)*)).ok();
        $uart.write_full_blocking(w.as_bytes());
    }};
}

//-------------------------
// Helper: Two bytes → i16 (little endian)
//-------------------------
fn to_i16(low: u8, high: u8) -> i16 {
    i16::from_le_bytes([low, high])
}

//-------------------------
// Entry Point
//-------------------------
#[entry]
fn main() -> ! {
    //-------------------------
    // Hardware Init
    //-------------------------
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let sio = hal::Sio::new(pac.SIO);

    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    //-------------------------
    // I2C Init — GP4: SDA, GP5: SCL
    //-------------------------
    let sda_pin = pins
        .gpio4
        .reconfigure::<hal::gpio::FunctionI2C, hal::gpio::PullUp>();
    let scl_pin = pins
        .gpio5
        .reconfigure::<hal::gpio::FunctionI2C, hal::gpio::PullUp>();

    let mut i2c = hal::I2C::i2c0(
        pac.I2C0,
        sda_pin,
        scl_pin,
        400_u32.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    //-------------------------
    // UART Init — GP0: TX, GP1: RX
    //-------------------------
    let uart_pins = (
        pins.gpio0.into_function::<hal::gpio::FunctionUart>(),
        pins.gpio1.into_function::<hal::gpio::FunctionUart>(),
    );

    let uart = hal::uart::UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS)
        .enable(
            UartConfig::new(115200_u32.Hz(), DataBits::Eight, None, StopBits::One),
            clocks.peripheral_clock.freq(),
        )
        .unwrap();

    //-------------------------
    // Timer Init
    //-------------------------
    let timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);

    let mut buf = [0u8; 128];

    //-------------------------
    // BNO055 Init
    //-------------------------
    uart_println!(uart, &mut buf, "Starting...\r\n");
    delay.delay_ms(800); // Boot time

    // Chip ID check — must be 0xA0
    let mut chip_id = [0u8; 1];
    i2c.write_read(BNO055_ADDR, &[CHIP_ID_REG], &mut chip_id)
        .unwrap();
    uart_println!(uart, &mut buf, "Chip ID: {:#X}\r\n", chip_id[0]);

    // Config mode
    i2c.write(BNO055_ADDR, &[OPR_MODE_REG, CONFIG_MODE])
        .unwrap();
    delay.delay_ms(25);

    // NDOF mode — full fusion, raw + euler + temp available
    i2c.write(BNO055_ADDR, &[OPR_MODE_REG, NDOF_MODE]).unwrap();
    delay.delay_ms(800);

    // CSV Header
    uart_println!(
        uart,
        &mut buf,
        "ts_us,ax,ay,az,gx,gy,gz,mx,my,mz,temp,roll,pitch,yaw,sys_cal,gyro_cal,acc_cal,mag_cal\r\n"
    );

    //-------------------------
    // Main Loop — 100Hz
    //-------------------------
    loop {
        let start = timer.get_counter().ticks();

        // Read Accelerometer (6 bytes) — 1 m/s² = 100 LSB
        let mut accel = [0u8; 6];
        i2c.write_read(BNO055_ADDR, &[ACCEL_REG], &mut accel)
            .unwrap();
        let ax = to_i16(accel[0], accel[1]);
        let ay = to_i16(accel[2], accel[3]);
        let az = to_i16(accel[4], accel[5]);

        // Read Gyroscope (6 bytes) — 1 °/s = 16 LSB
        let mut gyro = [0u8; 6];
        i2c.write_read(BNO055_ADDR, &[GYRO_REG], &mut gyro).unwrap();
        let gx = to_i16(gyro[0], gyro[1]);
        let gy = to_i16(gyro[2], gyro[3]);
        let gz = to_i16(gyro[4], gyro[5]);

        // Read Magnetometer (6 bytes) — 1 µT = 16 LSB
        let mut mag = [0u8; 6];
        i2c.write_read(BNO055_ADDR, &[MAG_REG], &mut mag).unwrap();
        let mx = to_i16(mag[0], mag[1]);
        let my = to_i16(mag[2], mag[3]);
        let mz = to_i16(mag[4], mag[5]);

        // Read Temperature (1 byte) — °C
        let mut temp_buf = [0u8; 1];
        i2c.write_read(BNO055_ADDR, &[TEMP_REG], &mut temp_buf)
            .unwrap();
        let temp = temp_buf[0] as i8;

        // Read Euler Angles (6 bytes) — 1° = 16 LSB
        // Order: Heading(Yaw), Roll, Pitch
        let mut euler = [0u8; 6];
        i2c.write_read(BNO055_ADDR, &[EULER_REG], &mut euler)
            .unwrap();
        let yaw = to_i16(euler[0], euler[1]); // 0 to 5760 (0° to 360°)
        let roll = to_i16(euler[2], euler[3]); // -2880 to +2880 (-180° to +180°)
        let pitch = to_i16(euler[4], euler[5]); // -1440 to +1440 (-90° to +90°)

        // Read Calibration Status (1 byte)
        // Bits [7:6] = System, [5:4] = Gyro, [3:2] = Accel, [1:0] = Mag
        // 3 = fully calibrated, 0 = not calibrated
        let mut calib = [0u8; 1];
        i2c.write_read(BNO055_ADDR, &[CALIB_REG], &mut calib)
            .unwrap();
        let sys_cal = (calib[0] >> 6) & 0x03;
        let gyro_cal = (calib[0] >> 4) & 0x03;
        let acc_cal = (calib[0] >> 2) & 0x03;
        let mag_cal = (calib[0] >> 0) & 0x03;

        // Send CSV
        uart_println!(
            uart,
            &mut buf,
            "{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}\r\n",
            start,
            ax,
            ay,
            az,
            gx,
            gy,
            gz,
            mx,
            my,
            mz,
            temp,
            roll,
            pitch,
            yaw,
            sys_cal,
            gyro_cal,
            acc_cal,
            mag_cal
        );

        // Fixed 100Hz sampling
        while timer.get_counter().ticks() - start < SAMPLE_PERIOD_US {}
    }
}
