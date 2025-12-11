/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2024 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Manual Control Menu Implementation
 */

#include "../../inc/MarlinConfigPre.h"

#if HAS_MARLINUI_MENU

#include "menu_item.h"

#if ENABLED(MANUAL_CONTROL_MODE)

#include "../../feature/manual_control.h"
#include "../../core/serial.h"

// Menu variables for stepper movement
static uint16_t manual_move_steps = 500;
static uint16_t manual_z_steps = 500;
static uint16_t manual_e_steps = 500;
static uint16_t manual_goto_mm = 0; // 0-100 mm target for Y goto

// Manual Control Menu
void menu_manual_control() {
  START_MENU();
  BACK_ITEM(MSG_MAIN_MENU);
  
  // Test item
  ACTION_ITEM_F(F("TEST"), []() {
    SERIAL_ECHOLNPGM("LCD: Test");
    ui.set_status(F("Test!"));
    ui.completion_feedback();
  });
  
  // Stepper control
  STATIC_ITEM_F(F("=== Steppers ==="));
  ACTION_ITEM_F(F("Enable Steppers"), []() {
    ui.set_status(F("Steppers ON"));
    process_manual_command("on");
  });
  ACTION_ITEM_F(F("Disable Steppers"), []() {
    ui.set_status(F("Steppers OFF"));
    process_manual_command("off");
  });
  
  // ADC position control
  STATIC_ITEM_F(F("=== ADC Control ==="));
  if (get_adc_control_active()) {
    ACTION_ITEM_F(F("Disable ADC"), []() {
      ui.set_status(F("ADC OFF"));
      process_manual_command("adc_off");
    });
  } else {
    ACTION_ITEM_F(F("Enable ADC"), []() {
      ui.set_status(F("ADC ON"));
      process_manual_command("adc_on");
    });
  }
  ACTION_ITEM_F(F("Reset Position"), []() {
    ui.set_status(F("Position reset"));
    process_manual_command("adc_zero");
  });
  
  // Z Limit Switch Test
  STATIC_ITEM_F(F("=== Tests ==="));
  ACTION_ITEM_F(F("Test Z Limit"), []() {
    ui.set_status(F("Testing Z limit..."));
    process_manual_command("tz");
  });
  
  // Movement settings
  STATIC_ITEM_F(F("=== Movement ==="));
  EDIT_ITEM_F(uint16_3, F("Z Steps"), &manual_z_steps, 1, 100);
  EDIT_ITEM_F(uint16_3, F("Goto Y (mm)"), &manual_goto_mm, 0, 100);
  ACTION_ITEM_F(F("Run Goto"), []() {
    ui.set_status(F("Goto Y..."));
    char cmd[16];
    sprintf(cmd, "goto%u", (unsigned)manual_goto_mm);
    process_manual_command(cmd);
  });
  
  // X axis movement
  STATIC_ITEM_F(F("--- X Axis ---"));
  ACTION_ITEM_F(F("X +"), []() {
    ui.set_status(F("Moving X+..."));
    char cmd[16];
    sprintf(cmd, "x+%d", manual_move_steps);
    process_manual_command(cmd);
  });
  ACTION_ITEM_F(F("X -"), []() {
    ui.set_status(F("Moving X-..."));
    char cmd[16];
    sprintf(cmd, "x-%d", manual_move_steps);
    process_manual_command(cmd);
  });
  
  // Y axis movement  
  STATIC_ITEM_F(F("--- Y Axis ---"));
  ACTION_ITEM_F(F("Y +"), []() {
    ui.set_status(F("Moving Y+..."));
    char cmd[16];
    sprintf(cmd, "y+%d", manual_move_steps);
    process_manual_command(cmd);
  });
  ACTION_ITEM_F(F("Y -"), []() {
    ui.set_status(F("Moving Y-..."));
    char cmd[16];
    sprintf(cmd, "y-%d", manual_move_steps);
    process_manual_command(cmd);
  });
  
  // Z axis movement
  STATIC_ITEM_F(F("--- Z Axis ---"));
  ACTION_ITEM_F(F("Z +"), []() {
    ui.set_status(F("Moving Z+..."));
    char cmd[16];
    sprintf(cmd, "z+%d", manual_z_steps);
    process_manual_command(cmd);
  });
  ACTION_ITEM_F(F("Z -"), []() {
    ui.set_status(F("Moving Z-..."));
    char cmd[16];
    sprintf(cmd, "z-%d", manual_z_steps);
    process_manual_command(cmd);
  });
  
  // E axis movement
  STATIC_ITEM_F(F("--- Extruder ---"));
  ACTION_ITEM_F(F("E +"), []() {
    ui.set_status(F("Moving E+..."));
    char cmd[16];
    sprintf(cmd, "e+%d", manual_e_steps);
    process_manual_command(cmd);
  });
  ACTION_ITEM_F(F("E -"), []() {
    ui.set_status(F("Moving E-..."));
    char cmd[16];
    sprintf(cmd, "e-%d", manual_e_steps);
    process_manual_command(cmd);
  });
  
  END_MENU();
}

#endif // ENABLED(MANUAL_CONTROL_MODE)

#endif // HAS_MARLINUI_MENU
