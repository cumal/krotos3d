/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2019 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "../../inc/MarlinConfig.h"

#include "../gcode.h"
#include "../../feature/bedlevel/bedlevel.h"
#include "../../module/planner.h"
#include "../../module/probe.h"
#include "../../module/stepper.h"
#include "../../core/serial.h"
#include "../../HAL/shared/Delay.h"

#if ENABLED(EEPROM_SETTINGS)
  #include "../../module/settings.h"
#endif

#if ENABLED(EXTENSIBLE_UI)
  #include "../../lcd/extui/ui_api.h"
#endif

// Custom parameters
#define MAXREPETITIONS 5
#define MAXOFFSET 0.05f // Tolerance in mm (0.05mm)
#define Z_MOTORS_POS { { 10, Y_BED_SIZE-30 } , { 10, 20 } , { X_BED_SIZE-60 , Y_BED_SIZE-30 } , { X_BED_SIZE-60 , 20 } }

/**
 * M777: Hardware bed leveling
 */

void set_aux_motor_enable(const uint8_t motor_index, const bool enable) {
  // Logic: 0 = Enable (LOW), 255 = Disable (HIGH)
  const int val = enable ? 0 : 255;
  switch (motor_index) {
    case 0: analogWrite(AUX2_03, val); break;
    case 1: analogWrite(AUX2_05, val); break;
    case 2: analogWrite(AUX2_10, val); break;
    case 3: analogWrite(AUX2_09, val); break;
  }
}

void moveMotorsParallel(const float deviations[4]) {
  const float steps_per_mm = planner.settings.axis_steps_per_mm[Z_AXIS];
  uint32_t steps[4];
  bool dir[4];
  bool has_steps = false;

  for (uint8_t i = 0; i < 4; i++) {
    steps[i] = lround(abs(deviations[i]) * steps_per_mm);
    dir[i] = deviations[i] > 0; // True = UP (HIGH), False = DOWN (LOW)
    if (steps[i] > 0) has_steps = true;
  }

  if (!has_steps) return;

  // 1. Disable global Z enable to prevent conflicts
  analogWrite(Z_ENABLE_PIN, 255);

  // Process one direction at a time (UP then DOWN)
  for (uint8_t d = 0; d < 2; d++) {
    bool current_dir = (d == 0); // First UP (true), then DOWN (false)
    
    // Find max steps for this direction group
    uint32_t max_steps = 0;
    for (uint8_t i = 0; i < 4; i++) {
      if (dir[i] == current_dir && steps[i] > 0) {
        max_steps = _MAX(max_steps, steps[i]);
      }
    }

    if (max_steps == 0) continue;

    // Set Direction and Enable relevant motors
    digitalWrite(Z_DIR_PIN, current_dir ? HIGH : LOW);
    for (uint8_t i = 0; i < 4; i++) {
      set_aux_motor_enable(i, (dir[i] == current_dir && steps[i] > 0));
    }

    // Step loop
    for (uint32_t s = 0; s < max_steps; s++) {
      // Disable motors that have finished their specific step count
      for (uint8_t i = 0; i < 4; i++) {
        if (dir[i] == current_dir && steps[i] > 0 && s == steps[i]) {
          set_aux_motor_enable(i, false);
        }
      }
      digitalWrite(Z_STEP_PIN, HIGH);
      DELAY_US(300); // ~1.6kHz stepping (~4mm/s)
      digitalWrite(Z_STEP_PIN, LOW);
      DELAY_US(300);
      idle();
    }
    
    // Ensure all in this group are disabled
    for (uint8_t i = 0; i < 4; i++) {
      if (dir[i] == current_dir) set_aux_motor_enable(i, false);
    }
  }

  // Re-enable global Z to hold position
  analogWrite(Z_ENABLE_PIN, 0);
}

float getMin(float array[]){
  float minimum = array[0];
  for (int i = 0; i < 4; i++) {
    if (array[i] < minimum) { minimum = array[i]; }
  }
  return minimum;
}

float getMax(float array[]){
  float maximun = array[0];
  for (int i = 0; i < 4; i++) {
    if (array[i] > maximun) { maximun = array[i]; }
  }
  return maximun;
}

void printDesviationSummary(float items[], float diff) {
  SERIAL_ECHOLN("Deviation summary: ", items[0], ", ", items[1], ", ", items[2], ", ", items[3]);
  SERIAL_ECHOLN("Diff: ", diff);
}
 
void GcodeSuite::M777() {
  int iter;
  if (parser.seenval('R')) {
    iter = parser.value_int();
  } else {
    iter = MAXREPETITIONS;
  }
  SERIAL_ECHOLN("Starting HW bed leveling. R:", iter);
  //float probe_z_offset = probe.offset.z;
  //probe.offset.z = 0;
  
  // Ensure probe is ready
  if (probe.deploy()) return;

  gcode.process_subcommands_now(F("G28")); // Home XY
  planner.synchronize();

  int repTimes = 1;
  bool run = true;
  xy_pos_t motPosition[4] = Z_MOTORS_POS;
  float motDesv[4];
  float heightDiff;

  while (run){
    for (int i = 0; i < 4; i++) {
      // Move to measurement position
      // do_blocking_move_to_xy(motPosition[i]);
      
      // Probe the point. probe_at_point returns the bed Z height.
      // We want the bed to be at Z=0 (or consistent).
      // If bed is at -1.0, we need to move it UP by 1.0.
      // Correction = -measured_z.
      const float measured_z = probe.probe_at_point(motPosition[i], PROBE_PT_NONE, 0);
      
      if (isnan(measured_z)) {
        SERIAL_ECHOLN("Probe failed at point ", i);
        probe.stow();
        return;
      }

      motDesv[i] = -measured_z; // Amount to move motor to reach Z=0
    }

    // Apply corrections
    planner.synchronize(); // Ensure no moves are active
    moveMotorsParallel(motDesv);
    
    heightDiff = (getMax(motDesv) - getMin(motDesv));
    printDesviationSummary(motDesv, heightDiff);

    if ( (abs(heightDiff) <= MAXOFFSET) || (repTimes >= iter) ) {
      run = false;
    } else {
      gcode.process_subcommands_now(F("G28 Z")); // Re-home Z after adjustment
      planner.synchronize();
      repTimes++;
    }
  }
  probe.stow();
  //probe.offset.z = probe_z_offset;
  SERIAL_ECHOLN("Ended HW bed leveling. Diff:", heightDiff, " Reps:", repTimes);
}