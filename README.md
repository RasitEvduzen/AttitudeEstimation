# Embedded Rust IMU Data Logger

This project implements a 9-DOF IMU data acquisition system using a Raspberry Pi Pico H and a BNO055 sensor. Raw sensor data and onboard fusion output are streamed over UART at 100 Hz to a PC, where MATLAB tools handle calibration, logging, and attitude estimation.

The goal is to implement every attitude estimation algorithm from the AHRS library in MATLAB from scratch, and compare them against the BNO055 onboard fusion output.


## What You Need

Raspberry Pi Pico H

BNO055 breakout board (GY-BNO055 or similar)

USB to UART adapter (CH340G, FT232RL, or similar)

Jumper wires and a breadboard

A Windows, macOS, or Linux computer


## Rust Toolchain Setup

Install Rust by going to https://rustup.rs and running the installer. On Windows, it will ask you to install Visual Studio C++ build tools first. Choose the default options.

After installation, open a terminal and verify:

```
rustc --version
cargo --version
```

Add the ARM Cortex-M0+ compile target that the Pico uses:

```
rustup target add thumbv6m-none-eabi
```

Install the flashing and debug tool:

```
cargo install probe-rs-tools --locked
```

Install the UF2 converter for drag-and-drop flashing:

```
cargo install elf2uf2-rs
```


## Getting the Code

```
git clone https://github.com/your-username/pico-bno055.git
cd pico-bno055
```


## Building

```
cargo build
```

For a release build:

```
cargo build --release
```


## Flashing the Pico

Hold the BOOTSEL button on the Pico, then plug it into your computer via USB. Release the button. The Pico will appear as a USB drive called RPI-RP2.

Convert the compiled binary to UF2 format:

```
elf2uf2-rs target/thumbv6m-none-eabi/debug/pico-bno055 pico-bno055.uf2
```

Copy the file to the Pico drive. On Windows:

```
copy pico-bno055.uf2 E:\
```

Replace E with the actual drive letter. The Pico will reboot and start running the firmware.


## Wiring

Connect the BNO055 to the Pico H as shown below.

```
Pico H          BNO055
3V3 (pin 36)    VCC
GND (pin 38)    GND
GP4 (pin 6)     SDA
GP5 (pin 7)     SCL
GND             PS0
GND             PS1
GND             ADR
```

PS0, PS1, and ADR pulled to GND selects I2C mode at address 0x28.

Connect the UART adapter to receive serial data on your PC.

```
Pico H          UART Adapter
GP0 (pin 1)     RX
GP1 (pin 2)     TX
GND             GND
```

Do not connect the VCC pin of the UART adapter to the Pico. The Pico is powered through its own USB connection.


## Serial Output

Open a serial terminal (Termite, PuTTY, or similar) at 115200 baud on the COM port assigned to your UART adapter. You will see output like this:

```
Starting...
Chip ID: 0xA0
ts_us,ax,ay,az,gx,gy,gz,mx,my,mz,temp,roll,pitch,yaw,sys_cal,gyro_cal,acc_cal,mag_cal
1628370,-72,139,943,0,2,-1,-2454,-1280,758,23,-68,-134,0,3,3,0,3
```

The firmware outputs 18 comma-separated values at 100 Hz. The calibration columns at the end go from 0 (not calibrated) to 3 (fully calibrated). Move the sensor slowly in all orientations until all four values reach 3 before collecting data.


## Sensor Units

```
Accelerometer    m/s squared    1 m/s2 = 100 LSB
Gyroscope        degrees/s      1 deg/s = 16 LSB
Magnetometer     microtesla     1 uT = 16 LSB
Euler angles     degrees        1 degree = 16 LSB
Temperature      Celsius        direct
```


## MATLAB Tools

Open MATLAB and navigate to the matlab folder in this repository.

Run bno055_calibration.m first to calibrate the sensor. The tool will guide you through gyroscope, accelerometer, and magnetometer calibration step by step and save the results to bno055_calib.mat.

Run bno055_logger.m to start real-time data logging. If bno055_calib.mat exists, calibration offsets are applied automatically. Data is saved to a timestamped CSV file.

The attitude estimation scripts in the filters folder implement each algorithm from scratch. Each filter reads from a logged CSV file and outputs Euler angles for comparison against the BNO055 onboard fusion.


## Project Structure

```
pico-bno055/
    src/
        main.rs         firmware source
    Cargo.toml          project dependencies
    matlab/
        bno055_calibration.m
        bno055_logger.m
        filters/
            complementary.m
            ekf.m
            madgwick.m
            ...
```


## Attitude Estimation Filters

All filters are written from scratch in MATLAB, based on the AHRS reference library. None use any external attitude estimation library.

Static estimators use only accelerometer and magnetometer data at a single point in time. Dynamic estimators additionally integrate gyroscope data over time.

```
Tilt                static
TRIAD               static
QUEST               static
Davenport           static
OLEQ                static
ROLEQ               static
FAMC                static
FLAE                static
FQA                 static
SAAM                static
Complementary       dynamic
EKF                 dynamic
Madgwick            dynamic
Mahony              dynamic
Fourati             dynamic
FKF                 dynamic
AQUA                dynamic
UKF                 dynamic
Angular rate        dynamic
```


## License

MIT
