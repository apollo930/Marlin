# Auto-Syringe Project

Automatic syringe filling machine controller based on Marlin firmware.

## Project Overview

This project uses modified Marlin firmware to control a machine that automatically fills syringes with precise volumes of liquid. The system uses stepper motors for precise movement control and limit switches for position detection.

## Hardware

- **MCU**: STM32F103RET6 (Creality 4.2.2 Board)
- **Display**: LCD with encoder interface
- **Stepper Motors**: Multiple axes for syringe control
- **Limit Switch**: Hardware interrupt-based detection on X_STOP_PIN (PA11)
- **Sensors**: ADC inputs for thermistor readings and position control

### Pin Configuration

- **X_STEP_PIN**: PB0
- **X_DIR_PIN**: PB1
- **Y_STEP_PIN**: PB8
- **Y_DIR_PIN**: PB7
- **Z_STEP_PIN**: PB6
- **Z_DIR_PIN**: PB5
- **E0_STEP_PIN**: PB9
- **E0_DIR_PIN**: PB10
- **X_ENABLE_PIN**: PC3 (shared enable for all steppers)
- **X_STOP_PIN**: PA11 (limit switch with hardware interrupt)
- **TEMP_0_PIN**: PC5 (ADC input)
- **TEMP_BED_PIN**: PC4 (ADC input)

## Features

### Manual Control Mode

Custom manual control system accessible via:
- **Serial Commands**: Direct control through serial terminal
- **LCD Menu**: User-friendly interface for field operation

#### Serial Commands

- `h` - Read hotend thermistor
- `b` - Read bed thermistor
- `x+[steps]` - Move X axis forward (default: 500 steps)
- `x-[steps]` - Move X axis backward
- `y+[steps]` - Move Y axis forward
- `y-[steps]` - Move Y axis backward
- `z+[steps]` - Move Z axis up
- `z-[steps]` - Move Z axis down
- `e+[steps]` - Move extruder forward
- `e-[steps]` - Move extruder backward
- `on` - Enable stepper motors
- `off` - Disable stepper motors
- `tz` - Test Z-axis limit switch
- `ts` - Monitor limit switch state
- `adc_on` - Enable ADC position control
- `adc_off` - Disable ADC position control
- `adc_zero` - Reset ADC position to zero
- `adc_range[value]` - Set ADC position range
- `help` - Show all commands

### LCD Menu System

The LCD menu provides easy access to:
- Stepper motor enable/disable
- Axis movement controls (X, Y, Z, E)
- Configurable step counts
- Thermistor readings
- ADC position control
- Limit switch testing

### Limit Switch System

- Hardware interrupt on RISING edge (LOW to HIGH transition)
- Non-blocking operation using ISR flags
- Integrated into Z-axis test function
- Returns motor to original position after trigger

### ADC Position Control

- Median filtering (8-sample buffer) for noise reduction
- Proportional control with configurable range
- Configurable position range (default: ±33000 steps, ~2×10^4 steps/V)
- Real-time feedback via serial output

## Building

1. Install [Visual Studio Code](https://code.visualstudio.com/)
2. Install [PlatformIO IDE](https://platformio.org/) extension
3. Open this project folder in VS Code
4. Build with PlatformIO for target: `STM32F103RE_creality`

## Configuration

Key configuration options in `Configuration_adv.h`:
- `MANUAL_CONTROL_MODE` - Enables manual control features (line 4788)

## Custom Files

- `Marlin/src/feature/manual_control.cpp` - Manual control implementation
- `Marlin/src/feature/manual_control.h` - Manual control header
- `Marlin/src/lcd/menu/menu_manual_control.cpp` - LCD menu integration
- `Marlin/src/pins/stm32f4/pins_BLACKPILL_CUSTOM.h` - Custom pin definitions

## Based on Marlin Firmware

This project is based on Marlin 2.1.x firmware. See [LICENSE](/LICENSE) for details.

Original Marlin documentation: [marlinfw.org](https://marlinfw.org/)
