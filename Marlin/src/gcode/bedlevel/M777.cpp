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
#include "../../core/serial.h"

#if ENABLED(EEPROM_SETTINGS)
  #include "../../module/settings.h"
#endif

#if ENABLED(EXTENSIBLE_UI)
  #include "../../lcd/extui/ui_api.h"
#endif

// Custom parameters
#define MAXREPETITIONS 5
#define MAXOFFSET 2 // correspond to 0.2 see (1)
#define Z_MOTORS_POS { { 20, 200 } , { 20, 40 } , { 160 , 200 } , { 160 , 40 } }

/**
 * M777: Hardware bed leveling
 */

 void moveMotor(const int mot, int loo) {
  if (loo == 0) {
    return;
  } else if (loo > 0) {
    digitalWrite(Z_DIR_PIN, HIGH); //High=up dir
  } else {
    digitalWrite(Z_DIR_PIN, LOW); //LOW=down dir
  }
  analogWrite(Z_ENABLE_PIN, 255); //Deactivate all z stepper
  if (mot==0){ // Activate motor 0 and disable rest
    analogWrite(AUX2_03, 0);
    analogWrite(AUX2_05, 255);
    analogWrite(AUX2_10, 255);
    analogWrite(AUX2_09, 255);
  } else if (mot==1) { // Activate motor 1 and disable rest
    analogWrite(AUX2_05, 0);
    analogWrite(AUX2_03, 255);
    analogWrite(AUX2_10, 255);
    analogWrite(AUX2_09, 255);
  } else if (mot==2) { // Activate motor 2 and disable rest
    analogWrite(AUX2_10, 0);
    analogWrite(AUX2_03, 255);
    analogWrite(AUX2_05, 255);
    analogWrite(AUX2_09, 255);
  } else if (mot==3) { // Activate motor 3 and disable rest
    analogWrite(AUX2_09, 0);
    analogWrite(AUX2_03, 255);
    analogWrite(AUX2_05, 255);
    analogWrite(AUX2_10, 255);
  }
  for (int x = 0; x < abs(loo*40); x++) { // Move steps. Normally 400 steps per mm, reduced to 40 see (1)
    digitalWrite(Z_STEP_PIN, HIGH);
    delay (2.5);
    digitalWrite(Z_STEP_PIN, LOW);
    delay (2.5);
    idle();
  }
  // Deactivate motor
  if (mot==0){ analogWrite(AUX2_03, 255); }
  else if (mot==1){ analogWrite(AUX2_05, 255); }
  else if (mot==2){ analogWrite(AUX2_10, 255); }
  else if (mot==3){ analogWrite(AUX2_09, 255); }
  analogWrite(Z_ENABLE_PIN, 0); // Activate Z steppers to keep height
}

int getDesviation(){
  int measuredDesv = 0;
  gcode.process_subcommands_now("G91"); // Relative positioning
  if (digitalRead(Z_MIN_PIN) == LOW) { // Endpoint triggered. Go down
    while (digitalRead(Z_MIN_PIN) == LOW){
      gcode.process_subcommands_now("G1 Z0.1 F500");
      planner.synchronize();
      measuredDesv = measuredDesv - 1; // Use 1 instead of 0.1 to avoid float errors (1)
    }
    measuredDesv = measuredDesv + 1; // last move goes out height
  } else { // Endpoint not triggered. Go up
    while (digitalRead(Z_MIN_PIN) != LOW){
      gcode.process_subcommands_now("G1 Z-0.1 F500");
      planner.synchronize();
      measuredDesv = measuredDesv + 1; // Use 1 instead of 0.1 to avoid float errors (1)
    }
  }
  gcode.process_subcommands_now("G90"); // Absolute positioning
  return measuredDesv;
}

float getMin(int array[]){
  float minimum = array[0];
  for (int i = 0; i < 4; i++) {
    if (array[i] < minimum) { minimum = array[i]; }
  }
  return minimum;
}

float getMax(int array[]){
  float maximun = array[0];
  for (int i = 0; i < 4; i++) {
    if (array[i] > maximun) { maximun = array[i]; }
  }
  return maximun;
}

void aBitDown(){
  gcode.process_subcommands_now("G91"); // Relative positioning
  gcode.process_subcommands_now("G1 Z5 F500");
  planner.synchronize();
  gcode.process_subcommands_now("G90"); // Absolute positioning
}

void printDesviationSummary(int items[]) {
  SERIAL_ECHO("Desviation summary: ");
  SERIAL_ECHO(items[0]);
  SERIAL_ECHO(", ");
  SERIAL_ECHO(items[1]);
  SERIAL_ECHO(", ");
  SERIAL_ECHO(items[2]);
  SERIAL_ECHO(", ");
  SERIAL_ECHOLN(items[3]);
}
 
void GcodeSuite::M777() {
  SERIAL_ECHOLN("Starting HW bed leveling...");
  float probe_z_offset = probe.offset.z; // Gets z probe offset
  gcode.process_subcommands_now("M851 Z0"); // Removes Z probe offset
  gcode.process_subcommands_now("G90"); // Absolute positioning
  gcode.process_subcommands_now("G28 X Y"); // Home XY
  planner.synchronize();
  int repTimes = 0;
  bool run = true;
  xy_pos_t motPosition[4] = Z_MOTORS_POS;
  char cmd[20], str_1[16], str_2[16];
  int motDesv[4];
  int heightDiff;
  while (run){
    gcode.process_subcommands_now("G28 Z"); // Center
    planner.synchronize(); // Wait move to finish
    for (int i = 0; i < 4; i++) {
      gcode.process_subcommands_now("G90"); // Absolute positioning
      sprintf_P(cmd, PSTR("G1X%sY%sZ0F1300"), dtostrf(motPosition[i].x, 1, 3, str_1), dtostrf(motPosition[i].y, 1, 3, str_2));
      gcode.process_subcommands_now(cmd); // Move to measure position
      planner.synchronize();
      motDesv[i] = getDesviation(); // Gets the height difference
    }
    for (int i = 0; i < 4; i++) {
      moveMotor(i, motDesv[i]); // fix height
    }
    printDesviationSummary(motDesv);
    heightDiff = (getMax(motDesv) - getMin(motDesv));
    if ( (heightDiff <= MAXOFFSET) || (repTimes == MAXREPETITIONS) ) {
      run = false;
      SERIAL_ECHO("Maximun: ");
      SERIAL_ECHO(heightDiff);
      SERIAL_ECHO("/");
      SERIAL_ECHOLN(MAXOFFSET);
      SERIAL_ECHO("Repetitions: ");
      SERIAL_ECHO(repTimes);
      SERIAL_ECHO("/");
      SERIAL_ECHOLN(MAXREPETITIONS);
    }
    repTimes=repTimes+1;
  }
  sprintf_P(cmd, PSTR("M851Z%s"), dtostrf(probe_z_offset, 6, 2, str_1)); // Restore Z probe offset
  gcode.process_subcommands_now(cmd); // Move to measure position
  SERIAL_ECHOLN("Ended HW bed leveling.");
}