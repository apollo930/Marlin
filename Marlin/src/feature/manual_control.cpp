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

#if HAS_MARLINUI_MENU
  #include "../lcd/marlinui.h"
#endif

// Limit switch interrupt variables
volatile bool limit_switch_triggered = false;
static volatile uint32_t limit_switch_timestamp = 0;

// Interrupt handler for limit switch
void limit_switch_isr() {
  // Trigger on rising edge (LOW to HIGH transition)
  limit_switch_triggered = true;
  limit_switch_timestamp = millis();
}

// ADC position control variables
static bool adc_control_active = false;
static int32_t adc_current_position = 0;  // Current Y position in steps
static int32_t adc_target_position = 0;   // Target Y position in steps
static uint16_t adc_range = 4095;         // Full ADC range (0-4095)
// Position range in steps; 66000 => ~2e4 steps per volt over 3.3V range
static int32_t position_range = 66000;     // ±33000 default range
static uint32_t last_adc_move = 0;        // Timing control

// Live monitoring variables
static bool live_monitor_active = false;  // Live ADC monitoring enabled
static uint8_t monitor_pin = 0;           // 0=hotend, 1=bed

// ADC averaging variables
#define ADC_SAMPLES 10                     // Number of samples to average
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

// Lookup table for non-linear potentiometer: resistance (kΩ) -> distance (mm)
// Calibrated data: 10mm to 80mm range
const struct {
  float resistance;  // in kΩ
  float distance;    // in mm
} pot_lookup_table[] = {
  {0.79, 10},
  {0.86, 11},
  {0.98, 12},
  {1.09, 13},
  {1.21, 14},
  {1.32, 15},
  {1.45, 16},
  {1.56, 17},
  {1.68, 18},
  {1.79, 19},
  {1.91, 20},
  {2.03, 21},
  {2.15, 22},
  {2.27, 23},
  {2.39, 24},
  {2.51, 25},
  {2.63, 26},
  {2.75, 27},
  {2.87, 28},
  {2.99, 29},
  {3.11, 30},
  {3.22, 31},
  {3.35, 32},
  {3.47, 33},
  {3.59, 34},
  {3.71, 35},
  {3.82, 36},
  {3.94, 37},
  {4.05, 38},
  {4.18, 39},
  {4.30, 40},
  {4.41, 41},
  {4.53, 42},
  {4.64, 43},
  {4.77, 44},
  {4.88, 45},
  {5.01, 46},
  {5.13, 47},
  {5.25, 48},
  {5.37, 49},
  {5.49, 50},
  {5.61, 51},
  {5.73, 52},
  {5.85, 53},
  {5.96, 54},
  {6.08, 55},
  {6.19, 56},
  {6.32, 57},
  {6.44, 58},
  {6.56, 59},
  {6.68, 60},
  {6.80, 61},
  {6.93, 62},
  {7.04, 63},
  {7.15, 64},
  {7.28, 65},
  {7.40, 66},
  {7.51, 67},
  {7.64, 68},
  {7.76, 69},
  {7.88, 70},
  {8.00, 71},
  {8.12, 72},
  {8.24, 73},
  {8.36, 74},
  {8.47, 75},
  {8.60, 76},
  {8.72, 77},
  {8.83, 78},
  {8.95, 79},
  {9.00, 80}
};
const uint8_t pot_lookup_size = sizeof(pot_lookup_table) / sizeof(pot_lookup_table[0]);

// Convert resistance to distance using lookup table with linear interpolation
float resistance_to_distance(float resistance_ohms) {
  float resistance_kohms = resistance_ohms / 1000.0f;

  // Clamp to lookup table range
  if (resistance_kohms <= pot_lookup_table[0].resistance) {
    return pot_lookup_table[0].distance;
  }
  if (resistance_kohms >= pot_lookup_table[pot_lookup_size - 1].resistance) {
    return pot_lookup_table[pot_lookup_size - 1].distance;
  }
  
  // Find surrounding points for interpolation
  for (uint8_t i = 0; i < pot_lookup_size - 1; i++) {
    if (resistance_kohms >= pot_lookup_table[i].resistance && 
        resistance_kohms <= pot_lookup_table[i + 1].resistance) {
      // Linear interpolation
      float r1 = pot_lookup_table[i].resistance;
      float r2 = pot_lookup_table[i + 1].resistance;
      float d1 = pot_lookup_table[i].distance;
      float d2 = pot_lookup_table[i + 1].distance;
      
      float t = (resistance_kohms - r1) / (r2 - r1);  // 0 to 1
      return d1 + t * (d2 - d1);
    }
  }
  
  return 0.0f;  // Should not reach here
}

// Convert distance to resistance using lookup table with linear interpolation (reverse lookup)
float distance_to_resistance(float distance_mm) {
  // Clamp to lookup table range
  if (distance_mm <= pot_lookup_table[0].distance) {
    return pot_lookup_table[0].resistance * 1000.0f;
  }
  if (distance_mm >= pot_lookup_table[pot_lookup_size - 1].distance) {
    return pot_lookup_table[pot_lookup_size - 1].resistance * 1000.0f;
  }
  
  // Find surrounding points for interpolation
  for (uint8_t i = 0; i < pot_lookup_size - 1; i++) {
    if (distance_mm >= pot_lookup_table[i].distance && 
        distance_mm <= pot_lookup_table[i + 1].distance) {
      // Linear interpolation
      float d1 = pot_lookup_table[i].distance;
      float d2 = pot_lookup_table[i + 1].distance;
      float r1 = pot_lookup_table[i].resistance;
      float r2 = pot_lookup_table[i + 1].resistance;
      
      float t = (distance_mm - d1) / (d2 - d1);  // 0 to 1
      return (r1 + t * (r2 - r1)) * 1000.0f;  // Convert back to ohms
    }
  }
  
  return 0.0f;  // Should not reach here
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
  #if HAS_MARLINUI_MENU
    char status[20];
    snprintf(status, 20, "HOT:%u", adcValue);
    ui.set_status(status);
  #endif
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
  #if HAS_MARLINUI_MENU
    char status[20];
    snprintf(status, 20, "BED:%u", adcValue);
    ui.set_status(status);
  #endif
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
  
  #if HAS_MARLINUI_MENU
    ui.set_status(F("Move complete"));
  #endif
}

void manual_enable_steppers() {
  WRITE(X_ENABLE_PIN, LOW);  // All steppers share PC3
  SERIAL_ECHOLNPGM("Steppers ENABLED");
}

void manual_disable_steppers() {
  WRITE(X_ENABLE_PIN, HIGH);
  SERIAL_ECHOLNPGM("Steppers DISABLED - Manual movement allowed");
}

void test_limit_switch_only() {
  SERIAL_ECHOLNPGM("Testing limit switch on X_STOP_PIN (PA11)...");
  SERIAL_ECHO("Current state: ");
  
  if (READ(X_STOP_PIN) == HIGH) {
    SERIAL_ECHOLNPGM("HIGH (not pressed)");
  } else {
    SERIAL_ECHOLNPGM("LOW (pressed/triggered)");
  }
  
  SERIAL_ECHOLNPGM("Monitoring for 10 seconds (press Ctrl+C to stop)...");
  
  for (uint8_t i = 0; i < 100; i++) {
    static bool last_state = HIGH;
    bool current_state = READ(X_STOP_PIN);
    
    if (current_state != last_state) {
      SERIAL_ECHO("State changed to: ");
      SERIAL_ECHOLNPGM(current_state == HIGH ? "HIGH (released)" : "LOW (pressed)");
      last_state = current_state;
    }
    
    delay(100);
    hal.watchdog_refresh();
  }
  
  SERIAL_ECHOLNPGM("Limit switch test complete!");
}

void test_z_limit_switch() {
  SERIAL_ECHOLNPGM("Starting Z limit switch test with interrupt...");
  
  // Reset interrupt flag
  limit_switch_triggered = false;
  
  // Enable steppers
  WRITE(X_ENABLE_PIN, LOW);
  DELAY_US(100);
  
  // Set direction - anticlockwise (false = reverse direction)
  WRITE(Z_DIR_PIN, LOW);
  DELAY_US(100);
  
  // Move until limit switch interrupt is triggered
  uint16_t step_count = 0;
  const uint16_t max_steps = 50000; // Safety limit
  
  SERIAL_ECHOLNPGM("Moving Z anticlockwise until limit switch interrupt...");
  
  while (!limit_switch_triggered && step_count < max_steps) {
    WRITE(Z_STEP_PIN, HIGH);
    DELAY_US(100);
    WRITE(Z_STEP_PIN, LOW);
    DELAY_US(400);
    
    step_count++;
    
    // Watchdog refresh and status update every 100 steps
    if (step_count % 100 == 0) {
      hal.watchdog_refresh();
      if (step_count % 500 == 0) {
        SERIAL_ECHO("Steps: ");
        SERIAL_ECHOLN(step_count);
      }
    }
  }
  
  if (limit_switch_triggered) {
    SERIAL_ECHO("Limit switch triggered after ");
    SERIAL_ECHO(step_count);
    SERIAL_ECHOLNPGM(" steps!");
    SERIAL_ECHO("Interrupt timestamp: ");
    SERIAL_ECHOLN(limit_switch_timestamp);
  } else {
    SERIAL_ECHOLNPGM("WARNING: Max steps reached without hitting limit switch!");
  }
  
  // Wait 1 second
  SERIAL_ECHOLNPGM("Waiting 1 second...");
  delay(1000);
  
  // Move back (clockwise)
  WRITE(Z_DIR_PIN, HIGH);
  DELAY_US(100);
  
  SERIAL_ECHO("Moving back ");
  SERIAL_ECHO(step_count);
  SERIAL_ECHOLNPGM(" steps...");
  
  for (uint16_t i = 0; i < step_count; i++) {
    WRITE(Z_STEP_PIN, HIGH);
    DELAY_US(100);
    WRITE(Z_STEP_PIN, LOW);
    DELAY_US(400);
    
    if (i % 100 == 0) {
      hal.watchdog_refresh();
    }
  }
  
  SERIAL_ECHOLNPGM("Z limit switch test complete!");
  
  // Reset interrupt flag
  limit_switch_triggered = false;
  
  #if HAS_MARLINUI_MENU
    ui.set_status(F("Z test complete!"));
  #endif
}

void calibrate_potentiometer() {
  SERIAL_ECHOLNPGM("=== Potentiometer Calibration ===");
  SERIAL_ECHOLNPGM("This will move Y axis and record resistance values");
  SERIAL_ECHOLNPGM("Make sure Y axis is at 10mm position before starting!");
  SERIAL_ECHOLNPGM("Press any key to start or 'q' to quit...");
  
  // Wait for user confirmation
  while (!MYSERIAL1.available()) {
    hal.watchdog_refresh();
    delay(10);
  }
  
  char confirm = MYSERIAL1.read();
  if (confirm == 'q' || confirm == 'Q') {
    SERIAL_ECHOLNPGM("Calibration cancelled");
    return;
  }
  
  // Clear any remaining serial data
  while (MYSERIAL1.available()) MYSERIAL1.read();
  
  // Enable steppers
  WRITE(X_ENABLE_PIN, LOW);
  DELAY_US(100);
  
  SERIAL_ECHOLNPGM("\nStarting calibration...");
  SERIAL_ECHOLNPGM("=== COPY BELOW INTO pot_lookup_table[] ===");
  SERIAL_ECHOLNPGM("");
  
  const uint16_t steps_per_mm = 400;
  const uint8_t start_distance_mm = 10;
  const uint8_t end_distance_mm = 80;
  const uint8_t measurement_interval_mm = 1; // Take measurement every 1mm
  const uint8_t total_measurements = (end_distance_mm - start_distance_mm) / measurement_interval_mm + 1;
  
  // Set direction to positive (reversed)
  WRITE(Y_DIR_PIN, LOW);
  DELAY_US(100);
  
  uint8_t measurement_count = 0;
  
  for (uint8_t distance_mm = start_distance_mm; distance_mm <= end_distance_mm; distance_mm += measurement_interval_mm) {
    // Take multiple ADC readings and average
    const uint8_t num_samples = 20;
    uint32_t adc_sum = 0;
    
    delay(100); // Let motor settle
    
    for (uint8_t i = 0; i < num_samples; i++) {
      uint16_t adc_val = analogRead(TEMP_BED_PIN);
      adc_sum += adc_val;
      delay(10);
      hal.watchdog_refresh();
    }
    
    uint16_t adc_avg = adc_sum / num_samples;
    
    // Clamp to valid range
    adc_avg = constrain(adc_avg, 0, 2690);
    
    // Convert to voltage and resistance
    float voltage = (adc_avg * 3.3f) / 4095.0f;
    float resistance = calculate_resistance(voltage, 4700.0f);
    float resistance_kohms = resistance / 1000.0f;
    
    // Output in format ready for lookup table
    SERIAL_ECHO("  {");
    SERIAL_ECHO(resistance_kohms);
    SERIAL_ECHO(", ");
    SERIAL_ECHO(distance_mm);
    SERIAL_ECHO("}");
    
    measurement_count++;
    if (measurement_count < total_measurements) {
      SERIAL_ECHO(",");
    }
    
    // Also output raw data for reference
    SERIAL_ECHO("  // ADC: ");
    SERIAL_ECHO(adc_avg);
    SERIAL_ECHO(", V: ");
    SERIAL_ECHO(voltage);
    SERIAL_ECHOLNPGM("V");
    
    // Move to next position (except at last measurement)
    if (distance_mm < end_distance_mm) {
      uint16_t steps_to_move = steps_per_mm * measurement_interval_mm;
      
      for (uint16_t i = 0; i < steps_to_move; i++) {
        WRITE(Y_STEP_PIN, HIGH);
        DELAY_US(500);
        WRITE(Y_STEP_PIN, LOW);
        DELAY_US(1500);
        
        if (i % 10 == 0) hal.watchdog_refresh();
      }
    }
  }
  
  SERIAL_ECHOLNPGM("");
  SERIAL_ECHOLNPGM("=== END LOOKUP TABLE DATA ===");
  SERIAL_ECHO("Total measurements: ");
  SERIAL_ECHOLN(measurement_count);
  SERIAL_ECHOLNPGM("\nUpdate code:");
  SERIAL_ECHO("const uint8_t pot_lookup_size = ");
  SERIAL_ECHO(measurement_count);
  SERIAL_ECHOLNPGM(";");
  SERIAL_ECHOLNPGM("\nReturning to 10mm start position...");
  
  // Return to start position (10mm) (reversed)
  WRITE(Y_DIR_PIN, HIGH);
  DELAY_US(100);
  
  uint32_t total_steps = steps_per_mm * (end_distance_mm - start_distance_mm);
  for (uint32_t i = 0; i < total_steps; i++) {
    WRITE(Y_STEP_PIN, HIGH);
    DELAY_US(500);
    WRITE(Y_STEP_PIN, LOW);
    DELAY_US(1500);
    
    if (i % 100 == 0) {
      hal.watchdog_refresh();
      if (i % 2000 == 0) {
        SERIAL_ECHO("Returning... ");
        SERIAL_ECHO((total_steps - i) / steps_per_mm);
        SERIAL_ECHOLNPGM("mm remaining");
      }
    }
  }
  
  SERIAL_ECHOLNPGM("Calibration routine complete!");
  SERIAL_ECHOLNPGM("Back at 10mm position");
  
  #if HAS_MARLINUI_MENU
    ui.set_status(F("Cal complete!"));
  #endif
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
  
  // Clamp ADC to physical range (0..2690)
  adcValue = constrain(adcValue, 0, 2690);
  
  // Convert ADC to voltage
  float voltage = (adcValue * 3.3f) / 4095.0f;
  
  // Convert voltage to resistance using voltage divider with 4.7k pullup
  float resistance = calculate_resistance(voltage, 4700.0f);
  
  // Convert resistance to distance using lookup table
  float target_mm_f = resistance_to_distance(resistance);
  int32_t target_steps = (int32_t)(target_mm_f * 400.0f);    // steps
  
  // Calculate position error
  int32_t position_error = target_steps - adc_current_position;
  
  // Deadzone to prevent jitter (20 steps = 0.05mm)
  if (abs(position_error) > 20) {
    // Enable steppers
    WRITE(X_ENABLE_PIN, LOW);
    
    // Determine direction
    bool direction = position_error > 0;
    
    // Proportional control: larger error = more steps (max 50 steps per update)
    uint16_t steps_to_move = min((uint16_t)abs(position_error), (uint16_t)50);
    
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
      float current_mm = adc_current_position / 400.0f;
      float target_mm = target_steps / 400.0f;
      SERIAL_ECHO("ADC: ");
      SERIAL_ECHO(adcValue);
      SERIAL_ECHO(" | Target: ");
      SERIAL_ECHO(target_mm);
      SERIAL_ECHO("mm | Current: ");
      SERIAL_ECHO(current_mm);
      SERIAL_ECHO("mm | Error: ");
      SERIAL_ECHO(position_error);
      SERIAL_ECHOLNPGM(" steps");
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

// Parse millimeters and convert to steps (400 steps/mm)
uint16_t parse_mm_to_steps(const char* command) {
  // Find the number after the command (e.g., "xm+1.5")
  const char* numStart = command + 3; // Skip "xm+"
  if (*numStart == '\0') return 0;
  
  // Parse integer and decimal parts
  float mm = 0.0f;
  float decimal = 0.0f;
  float decimal_place = 0.1f;
  bool in_decimal = false;
  
  while ((*numStart >= '0' && *numStart <= '9') || *numStart == '.') {
    if (*numStart == '.') {
      in_decimal = true;
    } else if (in_decimal) {
      decimal += (*numStart - '0') * decimal_place;
      decimal_place *= 0.1f;
    } else {
      mm = mm * 10.0f + (*numStart - '0');
    }
    numStart++;
  }
  
  mm += decimal;
  
  // Convert mm to steps (400 steps/mm)
  uint16_t steps = (uint16_t)(mm * 400.0f);
  
  // Limit to reasonable range
  return (steps > 0 && steps <= 10000) ? steps : 0;
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
    uint16_t steps = parse_steps(command, 500);
    manual_move_axis(X_STEP_PIN, X_DIR_PIN, true, steps);  // PC2, PB9
  }
  else if (strncmp(command, "x-", 2) == 0) {
    uint16_t steps = parse_steps(command, 500);
    manual_move_axis(X_STEP_PIN, X_DIR_PIN, false, steps);
  }
  else if (strncmp(command, "y+", 2) == 0) {
    uint16_t steps = parse_steps(command, 500);
    manual_move_axis(Y_STEP_PIN, Y_DIR_PIN, true, steps);  // PB8, PB7
  }
  else if (strncmp(command, "y-", 2) == 0) {
    uint16_t steps = parse_steps(command, 500);
    manual_move_axis(Y_STEP_PIN, Y_DIR_PIN, false, steps);
  }
  else if (strncmp(command, "ym+", 3) == 0) {
    uint16_t steps = parse_mm_to_steps(command);
    if (steps > 0) {
      manual_move_axis(Y_STEP_PIN, Y_DIR_PIN, true, steps);
      SERIAL_ECHO("Y+ ");
      SERIAL_ECHO(steps);
      SERIAL_ECHOLNPGM(" steps");
    }
  }
  else if (strncmp(command, "ym-", 3) == 0) {
    uint16_t steps = parse_mm_to_steps(command);
    if (steps > 0) {
      manual_move_axis(Y_STEP_PIN, Y_DIR_PIN, false, steps);
      SERIAL_ECHO("Y- ");
      SERIAL_ECHO(steps);
      SERIAL_ECHOLNPGM(" steps");
    }
  }
  else if (strncmp(command, "z+", 2) == 0) {
    uint16_t steps = parse_steps(command, 500);
    manual_move_axis(Z_STEP_PIN, Z_DIR_PIN, true, steps);   // PB6, PB5
  }
  else if (strncmp(command, "z-", 2) == 0) {
    uint16_t steps = parse_steps(command, 500);
    manual_move_axis(Z_STEP_PIN, Z_DIR_PIN, false, steps);
  }
  else if (strncmp(command, "e+", 2) == 0) {
    uint16_t steps = parse_steps(command, 500);
    manual_move_axis(E0_STEP_PIN, E0_DIR_PIN, true, steps); // PB4, PB3
  }
  else if (strncmp(command, "e-", 2) == 0) {
    uint16_t steps = parse_steps(command, 500);
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
  else if (strcmp(command, "monitor_on") == 0) {
    live_monitor_active = true;
    SERIAL_ECHOLNPGM("Live ADC monitoring ENABLED");
  }
  else if (strcmp(command, "monitor_off") == 0) {
    live_monitor_active = false;
    SERIAL_ECHOLNPGM("Live ADC monitoring DISABLED");
  }
  else if (strcmp(command, "monitor_hotend") == 0) {
    live_monitor_active = true;
    monitor_pin = 0;
    SERIAL_ECHOLNPGM("Live monitoring HOTEND (PC5)");
  }
  else if (strcmp(command, "monitor_bed") == 0) {
    live_monitor_active = true;
    monitor_pin = 1;
    SERIAL_ECHOLNPGM("Live monitoring BED (PC4)");
  }
  else if (strcmp(command, "tz") == 0) {
    test_z_limit_switch();
  }
  else if (strcmp(command, "ts") == 0) {
    test_limit_switch_only();
  }
  else if (strcmp(command, "cal") == 0 || strcmp(command, "calibrate") == 0) {
    calibrate_potentiometer();
  }
  else if (strncmp(command, "goto", 4) == 0) {
    // Parse mm value from command (e.g., "goto50" or "goto12.5")
    const char* numStart = command + 4;
    if (*numStart != '\0') {
      float target_mm = 0.0f;
      float decimal = 0.0f;
      float decimal_place = 0.1f;
      bool in_decimal = false;
      
      while ((*numStart >= '0' && *numStart <= '9') || *numStart == '.') {
        if (*numStart == '.') {
          in_decimal = true;
        } else if (in_decimal) {
          decimal += (*numStart - '0') * decimal_place;
          decimal_place *= 0.1f;
        } else {
          target_mm = target_mm * 10.0f + (*numStart - '0');
        }
        numStart++;
      }
      
      target_mm += decimal;
      
      // Validate range (0 to 100mm)
      if (target_mm >= 0.0f && target_mm <= 100.0f) {
        // Convert target mm to target resistance using lookup table
        float target_resistance = distance_to_resistance(target_mm);
        
        SERIAL_ECHO("Moving to ");
        SERIAL_ECHO(target_mm);
        SERIAL_ECHO("mm (Target R: ");
        SERIAL_ECHO(target_resistance);
        SERIAL_ECHOLNPGM("Ω)");
        
        // Enable steppers
        WRITE(X_ENABLE_PIN, LOW);
        
        // Closed-loop control with resistance feedback
        const uint16_t max_iterations = 5000; // Timeout after 5000 iterations
        uint16_t iteration = 0;
        const float resistance_tolerance = 56.5f; // Ohms tolerance (~0.5mm with 113 ohms/mm)
        
        while (iteration < max_iterations) {
          // Read current ADC position with median filter
          uint16_t adc_readings[8];
          for (uint8_t i = 0; i < 8; i++) {
            adc_readings[i] = analogRead(TEMP_BED_PIN);
            DELAY_US(1000);
          }
          
          // Sort for median
          for (uint8_t i = 0; i < 7; i++) {
            for (uint8_t j = 0; j < 7 - i; j++) {
              if (adc_readings[j] > adc_readings[j + 1]) {
                uint16_t temp = adc_readings[j];
                adc_readings[j] = adc_readings[j + 1];
                adc_readings[j + 1] = temp;
              }
            }
          }
          uint16_t current_adc = adc_readings[4]; // Median
          
          // Clamp to valid range
          current_adc = constrain(current_adc, 0, 2690);
          
          // Convert ADC to voltage, then to resistance with 4.7k pullup
          float voltage = (current_adc * 3.3f) / 4095.0f;
          float current_resistance = calculate_resistance(voltage, 4700.0f);
          
          // Calculate error in resistance
          float resistance_error = target_resistance - current_resistance;
          
          // Check if we've reached target
          if (abs(resistance_error) <= resistance_tolerance) {
            float final_mm = resistance_to_distance(current_resistance);
            SERIAL_ECHO("Target reached! R: ");
            SERIAL_ECHO(current_resistance);
            SERIAL_ECHO("Ω Position: ");
            SERIAL_ECHO(final_mm);
            SERIAL_ECHOLNPGM("mm");
            
            // Update position tracking
            adc_current_position = (int32_t)(final_mm * 400.0f);
            break;
          }
          
          // Proportional control: move based on error magnitude
          bool direction = resistance_error < 0; // Negative error means move forward
          uint16_t steps_to_move = min((uint16_t)abs((int32_t)resistance_error / 6), (uint16_t)20); // Scale error, max 20 steps
          steps_to_move = max(steps_to_move, (uint16_t)1); // Minimum 1 step
          
          // Set direction
          WRITE(Y_DIR_PIN, direction ? HIGH : LOW);
          DELAY_US(10);
          
          // Move steps
          for (uint16_t i = 0; i < steps_to_move; i++) {
            WRITE(Y_STEP_PIN, HIGH);
            DELAY_US(500);
            WRITE(Y_STEP_PIN, LOW);
            DELAY_US(500);
          }
          
          // Progress feedback every 100 iterations
          if (iteration % 100 == 0) {
            float current_mm = constrain(current_resistance / 113.0f, 0.0f, 100.0f);
            SERIAL_ECHO("R: ");
            SERIAL_ECHO(current_resistance);
            SERIAL_ECHO("Ω Pos: ");
            SERIAL_ECHO(current_mm);
            SERIAL_ECHO("mm Error: ");
            SERIAL_ECHOLN(resistance_error);
          }
          
          iteration++;
          hal.watchdog_refresh();
        }
        
        if (iteration >= max_iterations) {
          SERIAL_ECHOLNPGM("Warning: Max iterations reached");
        }
      } else {
        SERIAL_ECHOLNPGM("Error: Position must be 0-100mm");
      }
    } else {
      // Read current resistance position
      uint16_t current_adc = analogRead(TEMP_BED_PIN);
      current_adc = constrain(current_adc, 0, 2690);
      float voltage = (current_adc * 3.3f) / 4095.0f;
      float current_resistance = calculate_resistance(voltage, 4700.0f);
      float current_mm = constrain(current_resistance / 113.0f, 0.0f, 100.0f);
      SERIAL_ECHO("Current R: ");
      SERIAL_ECHO(current_resistance);
      SERIAL_ECHO("Ω Position: ");
      SERIAL_ECHO(current_mm);
      SERIAL_ECHOLNPGM("mm");
    }
  }
  else if (strncmp(command, "adc_range", 9) == 0) {
    const char* numStart = command + 9;
    if (*numStart != '\0') {
      int32_t new_range = 0;
      while (*numStart >= '0' && *numStart <= '9') {
        new_range = new_range * 10 + (*numStart - '0');
        numStart++;
      }
      if (new_range > 0 && new_range <= 100000) {
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
    SERIAL_ECHOLNPGM("ym+[mm] - Move Y positive in mm (e.g., ym+1.5)");
    SERIAL_ECHOLNPGM("ym-[mm] - Move Y negative in mm (400 steps/mm)");
    SERIAL_ECHOLNPGM("z+[steps] - Move Z up (default 10)");
    SERIAL_ECHOLNPGM("z-[steps] - Move Z down");
    SERIAL_ECHOLNPGM("e+[steps] - Extrude (default 50)");
    SERIAL_ECHOLNPGM("e-[steps] - Retract");
    SERIAL_ECHOLNPGM("on - Enable steppers");
    SERIAL_ECHOLNPGM("off - Disable steppers");
    SERIAL_ECHOLNPGM("ts - Test limit switch only");
    SERIAL_ECHOLNPGM("tz - Test Z limit switch");
    SERIAL_ECHOLNPGM("cal - Calibrate potentiometer (records R values every 1mm)");
    SERIAL_ECHOLNPGM("goto[mm] - Move to absolute position (e.g., goto50 or goto12.5)");
    SERIAL_ECHOLNPGM("           Range: 0-100mm, 400 steps/mm");
    SERIAL_ECHOLNPGM("adc_on - Enable ADC position control");
    SERIAL_ECHOLNPGM("adc_off - Disable ADC position control");
    SERIAL_ECHOLNPGM("adc_zero - Reset current position to zero");
    SERIAL_ECHOLNPGM("adc_range[value] - Set position range (default 66000 => ±33000)");
    SERIAL_ECHOLNPGM("monitor_on - Start live ADC monitoring");
    SERIAL_ECHOLNPGM("monitor_off - Stop live ADC monitoring");
    SERIAL_ECHOLNPGM("monitor_hotend - Monitor hotend sensor (PC5)");
    SERIAL_ECHOLNPGM("monitor_bed - Monitor bed sensor (PC4)");
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
  
  // Process serial first - don't let ADC blocking delays drop keypresses
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
  
  // Handle ADC-controlled Y movement only when serial is clear
  if (!MYSERIAL1.available()) {
    manual_adc_control_y();
  }
}

// Get current Y position from potentiometer (non-blocking)
float get_current_position_mm() {
  // Read ADC with median filter
  uint16_t adc_readings[8];
  for (uint8_t i = 0; i < 8; i++) {
    adc_readings[i] = analogRead(TEMP_BED_PIN);
    DELAY_US(1000);
  }
  
  // Sort for median
  for (uint8_t i = 0; i < 7; i++) {
    for (uint8_t j = 0; j < 7 - i; j++) {
      if (adc_readings[j] > adc_readings[j + 1]) {
        uint16_t temp = adc_readings[j];
        adc_readings[j] = adc_readings[j + 1];
        adc_readings[j + 1] = temp;
      }
    }
  }
  uint16_t current_adc = adc_readings[4]; // Median
  
  // Clamp to valid range
  current_adc = constrain(current_adc, 0, 2690);
  
  // Convert ADC to voltage, then to resistance with 4.7k pullup
  float voltage = (current_adc * 3.3f) / 4095.0f;
  float current_resistance = calculate_resistance(voltage, 4700.0f);
  
  // Convert resistance to distance
  return resistance_to_distance(current_resistance);
}

// Non-blocking absolute position movement
MoveStatus move_to_absolute_mm(float target_mm) {
  // State variables for non-blocking operation
  static bool move_active = false;
  static float target_resistance = 0.0f;
  static uint16_t iteration = 0;
  static const uint16_t max_iterations = 5000;
  static const float resistance_tolerance = 56.5f; // ~0.5mm
  
  // Validate target range
  if (!move_active && (target_mm < 10.0f || target_mm > 80.0f)) {
    return MOVE_ERROR_BOUNDS;
  }
  
  // Initialize movement
  if (!move_active) {
    target_resistance = distance_to_resistance(target_mm);
    iteration = 0;
    move_active = true;
    WRITE(X_ENABLE_PIN, LOW); // Enable steppers
    return MOVE_IN_PROGRESS;
  }
  
  // Read current position
  uint16_t adc_readings[8];
  for (uint8_t i = 0; i < 8; i++) {
    adc_readings[i] = analogRead(TEMP_BED_PIN);
    DELAY_US(1000);
  }
  
  // Sort for median
  for (uint8_t i = 0; i < 7; i++) {
    for (uint8_t j = 0; j < 7 - i; j++) {
      if (adc_readings[j] > adc_readings[j + 1]) {
        uint16_t temp = adc_readings[j];
        adc_readings[j] = adc_readings[j + 1];
        adc_readings[j + 1] = temp;
      }
    }
  }
  uint16_t current_adc = adc_readings[4];
  current_adc = constrain(current_adc, 0, 2690);
  
  float voltage = (current_adc * 3.3f) / 4095.0f;
  float current_resistance = calculate_resistance(voltage, 4700.0f);
  float resistance_error = target_resistance - current_resistance;
  
  // Check if reached target
  if (abs(resistance_error) <= resistance_tolerance) {
    float final_mm = resistance_to_distance(current_resistance);
    adc_current_position = (int32_t)(final_mm * 400.0f);
    move_active = false;
    return MOVE_COMPLETE;
  }
  
  // Check timeout
  if (iteration >= max_iterations) {
    move_active = false;
    return MOVE_ERROR_TIMEOUT;
  }
  
  // Proportional control: move 10-20 steps per call
  bool direction = resistance_error < 0;
  uint16_t steps_to_move = min((uint16_t)abs((int32_t)resistance_error / 6), (uint16_t)20);
  steps_to_move = max(steps_to_move, (uint16_t)1);
  
  WRITE(Y_DIR_PIN, direction ? HIGH : LOW);
  DELAY_US(10);
  
  for (uint16_t i = 0; i < steps_to_move; i++) {
    WRITE(Y_STEP_PIN, HIGH);
    DELAY_US(500);
    WRITE(Y_STEP_PIN, LOW);
    DELAY_US(500);
    
    if (i % 10 == 0) hal.watchdog_refresh();
  }
  
  iteration++;
  hal.watchdog_refresh();
  
  return MOVE_IN_PROGRESS;
}

// Non-blocking relative position movement
MoveStatus move_relative_mm(float delta_mm) {
  static bool initialized = false;
  static float target_mm = 0.0f;
  
  if (!initialized) {
    float current_mm = get_current_position_mm();
    target_mm = current_mm + delta_mm;
    
    // Validate target
    if (target_mm < 10.0f || target_mm > 80.0f) {
      initialized = false;
      return MOVE_ERROR_BOUNDS;
    }
    
    initialized = true;
  }
  
  MoveStatus status = move_to_absolute_mm(target_mm);
  
  if (status != MOVE_IN_PROGRESS) {
    initialized = false; // Reset for next call
  }
  
  return status;
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
  
  // Set up limit switch pin with pullup
  SET_INPUT_PULLUP(X_STOP_PIN);   // PA11
  
  // Attach interrupt to limit switch on rising edge (LOW to HIGH)
  attachInterrupt(digitalPinToInterrupt(X_STOP_PIN), limit_switch_isr, RISING);
  
  // Disable steppers initially
  manual_disable_steppers();
  
  SERIAL_ECHOLNPGM("Manual Control Initialized");
  SERIAL_ECHOLNPGM("Limit switch interrupt enabled on X_STOP_PIN (PA11)");
  SERIAL_ECHOLNPGM("Commands: h, b, x+[steps], y+[steps], z+[steps], e+[steps], on, off");
  SERIAL_ECHOLNPGM("ADC Control: adc_on, adc_off, adc_zero, adc_range[value]");
  SERIAL_ECHOLNPGM("Examples: x+200, y-50, z+5, e+100 (type 'help' for full list)");
}

// Access functions for menu integration
bool get_adc_control_active() {
  return adc_control_active;
}

void set_adc_control_active(bool active) {
  adc_control_active = active;
  if (active) {
    manual_enable_steppers();
    SERIAL_ECHOLNPGM("ADC Position Control ENABLED via LCD");
  } else {
    SERIAL_ECHOLNPGM("ADC Position Control DISABLED via LCD");
  }
}

void reset_adc_position() {
  adc_current_position = 0;
  SERIAL_ECHOLNPGM("Current position reset to zero via LCD");
}



#endif // MANUAL_CONTROL_MODE
