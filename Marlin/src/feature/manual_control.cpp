/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2024 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Manual Control Mode Implementation
 * Manual stepper control and thermistor reading for auto-syringe project
 */

#include "../inc/MarlinConfig.h"

#if ENABLED(MANUAL_CONTROL_MODE)

#include "manual_control.h"
#include "../HAL/shared/Delay.h"
#include "../core/serial.h"
#include "../pins/pins.h"

// ADC position control variables
static bool adc_control_active = false;
static int32_t adc_current_position = 0;  // Current Y position in steps
static int32_t adc_target_position = 0;   // Target Y position in steps
static uint16_t adc_range = 4095;         // Full ADC range (0-4095)
static int32_t position_range = 6400;     // Position range in steps (±3200 = ±1 full revolution)
static uint32_t last_adc_move = 0;        // Timing control

// ADC averaging variables
#define ADC_SAMPLES 8                     // Number of samples to average
static uint16_t adc_buffer[ADC_SAMPLES];  // Circular buffer for ADC readings
static uint8_t adc_buffer_index = 0;     // Current buffer position
static bool adc_buffer_filled = false;   // Whether buffer is fully populated

// Calculate thermistor resistance from voltage divider
float calculate_resistance(float voltage, float pullup_resistance) {
  if (voltage >= 3.3f) return 0.0f; // Short circuit
  if (voltage <= 0.0f) return 999999.0f; // Open circuit
  
  // Voltage divider: V_thermistor = V_supply * R_thermistor / (R_pullup + R_thermistor)
  // Solving for R_thermistor: R_thermistor = (V_thermistor * R_pullup) / (V_supply - V_thermistor)
  return (voltage * pullup_resistance) / (3.3f - voltage);
}

void manual_read_hotend_thermistor() {
  uint16_t adcValue = analogRead(TEMP_0_PIN);  // PC5
  float voltage = (adcValue * 3.3f) / 4095.0f;
  float resistance = calculate_resistance(voltage);
  
  SERIAL_ECHO("Hotend ADC Input - ADC: ");
  SERIAL_ECHO(adcValue);
  SERIAL_ECHO(", Voltage: ");
  SERIAL_ECHO(voltage);
  SERIAL_ECHO("V, Calculated R: ");
  if (resistance > 999999.0f) {
    SERIAL_ECHOLNPGM("OPEN");
  } else if (resistance < 1.0f) {
    SERIAL_ECHOLNPGM("SHORT");  
  } else if (resistance >= 1000.0f) {
    SERIAL_ECHO(resistance / 1000.0f);
    SERIAL_ECHOLNPGM("kΩ");
  } else {
    SERIAL_ECHO(resistance);
    SERIAL_ECHOLNPGM("Ω");
  }
}

void manual_read_bed_thermistor() {
  uint16_t adcValue = analogRead(TEMP_BED_PIN);  // PC4
  float voltage = (adcValue * 3.3f) / 4095.0f;
  float resistance = calculate_resistance(voltage);
  
  SERIAL_ECHO("Bed ADC Input - ADC: ");
  SERIAL_ECHO(adcValue);
  SERIAL_ECHO(", Voltage: ");
  SERIAL_ECHO(voltage);
  SERIAL_ECHO("V, Calculated R: ");
  if (resistance > 999999.0f) {
    SERIAL_ECHOLNPGM("OPEN");
  } else if (resistance < 1.0f) {
    SERIAL_ECHOLNPGM("SHORT");
  } else if (resistance >= 1000.0f) {
    SERIAL_ECHO(resistance / 1000.0f);
    SERIAL_ECHOLNPGM("kΩ");
  } else {
    SERIAL_ECHO(resistance);
    SERIAL_ECHOLNPGM("Ω");
  }
}

void manual_move_axis(pin_t step_pin, pin_t dir_pin, bool direction, uint16_t steps) {
  // Enable steppers
  WRITE(X_ENABLE_PIN, LOW);  // PC3 - Active LOW
  
  // Set direction
  WRITE(dir_pin, direction ? HIGH : LOW);
  DELAY_US(10);
  
  SERIAL_ECHO("Moving ");
  SERIAL_ECHO(steps);
  SERIAL_ECHOLNPGM(direction ? " steps forward" : " steps backward");
  
  // Step the motor
  for (uint16_t i = 0; i < steps; i++) {
    WRITE(step_pin, HIGH);
    DELAY_US(500);
    WRITE(step_pin, LOW);
    DELAY_US(1500);
    
    // Feed watchdog every 10 steps to prevent reset
    if (i % 10 == 0) hal.watchdog_refresh();
  }
  
  SERIAL_ECHOLNPGM("Move complete");
}

void manual_enable_steppers() {
  WRITE(X_ENABLE_PIN, LOW);  // All steppers share PC3
  SERIAL_ECHOLNPGM("Steppers ENABLED");
}

void manual_disable_steppers() {
  WRITE(X_ENABLE_PIN, HIGH);
  SERIAL_ECHOLNPGM("Steppers DISABLED - Manual movement allowed");
}

void manual_adc_control_y() {
  if (!adc_control_active) return;
  
  const millis_t now = millis();
  if (now - last_adc_move < 10) return;  // Update every 10ms
  
  // Read new ADC value and add to circular buffer
  uint16_t raw_adc = analogRead(TEMP_BED_PIN);  // Using bed ADC (PC4)
  adc_buffer[adc_buffer_index] = raw_adc;
  adc_buffer_index = (adc_buffer_index + 1) % ADC_SAMPLES;
  
  // Mark buffer as filled once we've wrapped around
  if (adc_buffer_index == 0 && !adc_buffer_filled) {
    adc_buffer_filled = true;
  }
  
  // Calculate median ADC value
  uint8_t samples_to_use = adc_buffer_filled ? ADC_SAMPLES : adc_buffer_index;
  
  // Copy buffer for sorting (don't modify original)
  uint16_t sorted_buffer[ADC_SAMPLES];
  for (uint8_t i = 0; i < samples_to_use; i++) {
    sorted_buffer[i] = adc_buffer[i];
  }
  
  // Simple bubble sort for median
  for (uint8_t i = 0; i < samples_to_use - 1; i++) {
    for (uint8_t j = 0; j < samples_to_use - i - 1; j++) {
      if (sorted_buffer[j] > sorted_buffer[j + 1]) {
        uint16_t temp = sorted_buffer[j];
        sorted_buffer[j] = sorted_buffer[j + 1];
        sorted_buffer[j + 1] = temp;
      }
    }
  }
  
  uint16_t adcValue = sorted_buffer[samples_to_use / 2];  // Median ADC value
  
  // Map averaged ADC value to target position (-3200 to +3200 steps)
  adc_target_position = map(adcValue, 0, adc_range, -position_range/2, position_range/2);
  
  // Calculate position error
  int32_t position_error = adc_target_position - adc_current_position;
  
  // Only move if there's a significant error (deadzone of 5 steps)
  if (abs(position_error) > 5) {
    // Enable steppers
    WRITE(X_ENABLE_PIN, LOW);
    
    // Determine direction
    bool direction = position_error > 0;
    
    // Calculate steps to move this update (proportional control)
    uint16_t steps_to_move = min((uint16_t)abs(position_error), (uint16_t)10);  // Max 10 steps per update
    
    // Set direction
    WRITE(Y_DIR_PIN, direction ? HIGH : LOW);
    DELAY_US(10);
    
    // Move steps
    for (uint16_t i = 0; i < steps_to_move; i++) {
      WRITE(Y_STEP_PIN, HIGH);
      DELAY_US(500);
      WRITE(Y_STEP_PIN, LOW);
      DELAY_US(500);
      
      // Update current position
      adc_current_position += direction ? 1 : -1;
      
      // Feed watchdog every few steps to prevent reset
      if (i % 5 == 0) hal.watchdog_refresh();
    }
    
    last_adc_move = now;
    
    // Print feedback every 25 updates (250ms)
    static uint8_t update_counter = 0;
    if (++update_counter >= 25) {
      update_counter = 0;
      SERIAL_ECHO("ADC Position Control - Raw: ");
      SERIAL_ECHO(raw_adc);
      SERIAL_ECHO(", Median: ");
      SERIAL_ECHO(adcValue);
      SERIAL_ECHO(", Target: ");
      SERIAL_ECHO(adc_target_position);
      SERIAL_ECHO(", Current: ");
      SERIAL_ECHO(adc_current_position);
      SERIAL_ECHO(", Error: ");
      SERIAL_ECHOLN(position_error);
    }
  }
}

// Parse number from command (e.g., "x+100" returns 100)
uint16_t parse_steps(const char* command, uint16_t defaultSteps) {
  // Find the number after the axis command (e.g., "x+100")
  const char* numStart = command + 2; // Skip "x+"
  if (*numStart == '\0') return defaultSteps; // No number, use default
  
  uint16_t steps = 0;
  while (*numStart >= '0' && *numStart <= '9') {
    steps = steps * 10 + (*numStart - '0');
    numStart++;
  }
  
  return (steps > 0 && steps <= 10000) ? steps : defaultSteps; // Limit to 10000 steps max
}

// Command processor
void process_manual_command(const char* command) {
  if (strcmp(command, "h") == 0) {
    manual_read_hotend_thermistor();
  }
  else if (strcmp(command, "b") == 0) {
    manual_read_bed_thermistor();
  }
  else if (strncmp(command, "x+", 2) == 0) {
    uint16_t steps = parse_steps(command, 100);
    manual_move_axis(X_STEP_PIN, X_DIR_PIN, true, steps);  // PC2, PB9
  }
  else if (strncmp(command, "x-", 2) == 0) {
    uint16_t steps = parse_steps(command, 100);
    manual_move_axis(X_STEP_PIN, X_DIR_PIN, false, steps);
  }
  else if (strncmp(command, "y+", 2) == 0) {
    uint16_t steps = parse_steps(command, 100);
    manual_move_axis(Y_STEP_PIN, Y_DIR_PIN, true, steps);  // PB8, PB7
  }
  else if (strncmp(command, "y-", 2) == 0) {
    uint16_t steps = parse_steps(command, 100);
    manual_move_axis(Y_STEP_PIN, Y_DIR_PIN, false, steps);
  }
  else if (strncmp(command, "z+", 2) == 0) {
    uint16_t steps = parse_steps(command, 10);
    manual_move_axis(Z_STEP_PIN, Z_DIR_PIN, true, steps);   // PB6, PB5
  }
  else if (strncmp(command, "z-", 2) == 0) {
    uint16_t steps = parse_steps(command, 10);
    manual_move_axis(Z_STEP_PIN, Z_DIR_PIN, false, steps);
  }
  else if (strncmp(command, "e+", 2) == 0) {
    uint16_t steps = parse_steps(command, 50);
    manual_move_axis(E0_STEP_PIN, E0_DIR_PIN, true, steps); // PB4, PB3
  }
  else if (strncmp(command, "e-", 2) == 0) {
    uint16_t steps = parse_steps(command, 50);
    manual_move_axis(E0_STEP_PIN, E0_DIR_PIN, false, steps);
  }
  else if (strcmp(command, "on") == 0) {
    manual_enable_steppers();
  }
  else if (strcmp(command, "off") == 0) {
    manual_disable_steppers();
  }
  else if (strcmp(command, "adc_on") == 0) {
    adc_control_active = true;
    manual_enable_steppers();
    SERIAL_ECHOLNPGM("ADC Position Control ENABLED - ADC controls Y position");
    SERIAL_ECHO("Range: ");
    SERIAL_ECHO(-position_range/2);
    SERIAL_ECHO(" to +");
    SERIAL_ECHO(position_range/2);
    SERIAL_ECHOLNPGM(" steps");
  }
  else if (strcmp(command, "adc_off") == 0) {
    adc_control_active = false;
    SERIAL_ECHOLNPGM("ADC Position Control DISABLED");
  }
  else if (strcmp(command, "adc_zero") == 0) {
    adc_current_position = 0;
    SERIAL_ECHOLNPGM("Current position reset to zero");
  }
  else if (strncmp(command, "adc_range", 9) == 0) {
    const char* numStart = command + 9;
    if (*numStart != '\0') {
      int32_t new_range = 0;
      while (*numStart >= '0' && *numStart <= '9') {
        new_range = new_range * 10 + (*numStart - '0');
        numStart++;
      }
      if (new_range > 0 && new_range <= 50000) {
        position_range = new_range;
        SERIAL_ECHO("Position range set to ±");
        SERIAL_ECHOLN(position_range/2);
      }
    } else {
      SERIAL_ECHO("Current position range: ±");
      SERIAL_ECHOLN(position_range/2);
    }
  }
  else if (strcmp(command, "help") == 0) {
    SERIAL_ECHOLNPGM("Commands:");
    SERIAL_ECHOLNPGM("h - Read hotend thermistor");
    SERIAL_ECHOLNPGM("b - Read bed thermistor");
    SERIAL_ECHOLNPGM("x+[steps] - Move X positive (e.g., x+50)");
    SERIAL_ECHOLNPGM("x-[steps] - Move X negative");
    SERIAL_ECHOLNPGM("y+[steps] - Move Y positive");
    SERIAL_ECHOLNPGM("y-[steps] - Move Y negative");
    SERIAL_ECHOLNPGM("z+[steps] - Move Z up (default 10)");
    SERIAL_ECHOLNPGM("z-[steps] - Move Z down");
    SERIAL_ECHOLNPGM("e+[steps] - Extrude (default 50)");
    SERIAL_ECHOLNPGM("e-[steps] - Retract");
    SERIAL_ECHOLNPGM("on - Enable steppers");
    SERIAL_ECHOLNPGM("off - Disable steppers");
    SERIAL_ECHOLNPGM("adc_on - Enable ADC position control");
    SERIAL_ECHOLNPGM("adc_off - Disable ADC position control");
    SERIAL_ECHOLNPGM("adc_zero - Reset current position to zero");
    SERIAL_ECHOLNPGM("adc_range[value] - Set position range");
  }
  else if (strlen(command) > 0) {
    SERIAL_ECHO("Unknown command: ");
    SERIAL_ECHO(command);
    SERIAL_ECHOLNPGM(" (type 'help' for commands)");
  }
}

// Task to check for manual commands
void manual_control_task() {
  static char command_buffer[32];
  static uint8_t buffer_pos = 0;
  
  // Handle ADC-controlled Y movement
  manual_adc_control_y();
  
  while (MYSERIAL1.available() > 0) {
    char c = MYSERIAL1.read();
    
    if (c == '\n' || c == '\r') {
      if (buffer_pos > 0) {
        command_buffer[buffer_pos] = '\0';
        process_manual_command(command_buffer);
        buffer_pos = 0;
      }
    }
    else if (c >= 32 && c <= 126 && buffer_pos < 31) { // Printable characters
      command_buffer[buffer_pos++] = c;
    }
  }
}

void manual_control_init() {
  // Initialize pins
  SET_OUTPUT(X_STEP_PIN);   // PC2
  SET_OUTPUT(X_DIR_PIN);    // PB9
  SET_OUTPUT(Y_STEP_PIN);   // PB8
  SET_OUTPUT(Y_DIR_PIN);    // PB7
  SET_OUTPUT(Z_STEP_PIN);   // PB6
  SET_OUTPUT(Z_DIR_PIN);    // PB5
  SET_OUTPUT(E0_STEP_PIN);  // PB4
  SET_OUTPUT(E0_DIR_PIN);   // PB3
  SET_OUTPUT(X_ENABLE_PIN); // PC3
  
  SET_INPUT_PULLUP(TEMP_0_PIN);   // PC5
  SET_INPUT_PULLUP(TEMP_BED_PIN); // PC4
  
  // Disable steppers initially
  manual_disable_steppers();
  
  SERIAL_ECHOLNPGM("Manual Control Initialized");
  SERIAL_ECHOLNPGM("Commands: h, b, x+[steps], y+[steps], z+[steps], e+[steps], on, off");
  SERIAL_ECHOLNPGM("ADC Control: adc_on, adc_off, adc_zero, adc_range[value]");
  SERIAL_ECHOLNPGM("Examples: x+200, y-50, z+5, e+100 (type 'help' for full list)");
}

#endif // MANUAL_CONTROL_MODE
