/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2024 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Manual Control Mode
 * Manual stepper control and thermistor reading for auto-syringe project
 */
#pragma once

#include "../inc/MarlinConfigPre.h"

#if ENABLED(MANUAL_CONTROL_MODE)

#include "../inc/MarlinConfig.h"

// Function declarations
void manual_control_init();
void manual_control_task();
void manual_read_hotend_thermistor();
void manual_read_bed_thermistor();
void manual_move_axis(pin_t step_pin, pin_t dir_pin, bool direction, uint16_t steps);
void manual_enable_steppers();
void manual_disable_steppers();
void manual_adc_control_y();
void process_manual_command(const char* command);

// Utility functions
float calculate_resistance(float voltage, float pullup_resistance = 4700.0f);
uint16_t parse_steps(const char* command, uint16_t defaultSteps);

#endif // MANUAL_CONTROL_MODE
