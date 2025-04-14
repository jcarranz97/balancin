# Balancin Robot

## Project Structure

```
balancin/
├── docs/                     # Documentation (markdown, images, diagrams, etc.)
│   └── assets/               # Images or other media for docs
├── firmware/                 # Folder for all firmware components
│   ├── robot/           # Each firmware module/subproject
│   ├── component2/
│   └── common/               # Shared firmware code or utilities
├── hardware/                 # KiCad or other hardware design files
│   └── pcb1/      # Specific board/project (supports expansion)
│       ├── schematics/
│       └── pcb/
├── scripts/                  # Utility scripts (build, flash, analysis, etc.)
├── LICENSE.txt
├── README.md
└── .gitignore
```

### Firmware folders

```
component1/
├── CMakeLists.txt           # CMake config for this component
├── include/                 # Public headers
│   └── component1/          # Namespaced headers
│       └── component1.h
├── src/                     # Source files
│   ├── component1.c
│   └── main.c
│   └── utils.c
├── lib/                     # Optional: Local copies of libraries or third-party code
│   └── some-lib/
├── README.md                # Description of what this firmware does
└── pico_sdk_import.cmake    # Only if building standalone; usually imported from root
```

## Other Links

https://wired.chillibasket.com/2015/01/calibrating-mpu6050/
https://github.com/jrowberg/i2cdevlib/tree/master/RP2040/MPU6050

## Setup

## Compilation

```bash
cd firmware/
mkdir build
cd build
cmake ..
make -j$(nproc)
```

## Flashing

### Calibration

This is a program taken from [mpu6050_calibration.cpp](https://github.com/jrowberg/i2cdevlib/blob/master/RP2040/MPU6050/examples/mpu6050_calibration/mpu6050_calibration.cpp) which
performs calibration in your MPU6050.

```bash
cp calibration/mpu6050_calibration.uf2 /media/jcarranz/RPI-RP2/
```

### See Yaw, Pitch and Roll

This is a program taken from [mpu6050_DMP_port.cpp](https://github.com/jrowberg/i2cdevlib/blob/master/RP2040/MPU6050/examples/mpu6050_DMP_V6.12/mpu6050_DMP_port.cpp) which
uses the DMP of the MPU6050 to calculate the yaw, pitch and roll of the robot.

```bash
cp mpu6050_DMP/mpu6050_DMP_port.uf2 /media/jcarranz/RPI-RP2/
```

### Robot Program

This is the main program of the robot. It uses the DMP of the MPU6050 to calculate the yaw, pitch and roll of the robot.
It also uses the PID controller to control the motors of the robot.

```bash
cp robot/robot.uf2 /media/jcarranz/RPI-RP2/
```
