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
#include "../../core/serial.h"

#if ENABLED(EEPROM_SETTINGS)
  #include "../../module/configuration_store.h"
#endif

#if ENABLED(EXTENSIBLE_UI)
  #include "../../lcd/extensible_ui/ui_api.h"
#endif

//#define M420_C_USE_MEAN

/**
 * M777: Hardware bed leveling
 */
 
 void moveup(const int mot, float loo) {
  idle();
  SERIAL_ECHO("Adjusting...");
  SERIAL_ECHOLN(mot);
  if (loo>800){
    loo=800;  }
  SERIAL_ECHO("Correction...");
  SERIAL_ECHOLN(loo);              
  //Motors UP
  digitalWrite(Z_DIR_PIN, HIGH); //High=up dir
  analogWrite(Z_ENABLE_PIN, 255); //Deactivate all z stepper
  
  // Deactivate all motors
  analogWrite(M1_ENABLE_PIN, 255);
  analogWrite(M2_ENABLE_PIN, 255);
  analogWrite(M3_ENABLE_PIN, 255);
  analogWrite(M4_ENABLE_PIN, 255);
  
  if (mot==1){ analogWrite(M1_ENABLE_PIN, 0); }
  else if (mot==2){ analogWrite(M2_ENABLE_PIN, 0); }
  else if (mot==3){ analogWrite(M3_ENABLE_PIN, 0); }
  else if (mot==4){ analogWrite(M4_ENABLE_PIN, 0); }
  
  // Move one step
  for (float x = 0; x < loo; x++) {
    digitalWrite(Z_STEP_PIN, HIGH);
    delay (5/2);
    digitalWrite(Z_STEP_PIN, LOW);
    delay (5/2);
  }

  // Deactivate all motors
  analogWrite(M1_ENABLE_PIN, 255);
  analogWrite(M2_ENABLE_PIN, 255);
  analogWrite(M3_ENABLE_PIN, 255);
  analogWrite(M4_ENABLE_PIN, 255);
  // Activate Z steppers to keep height
  analogWrite(Z_ENABLE_PIN, 0);
  idle();
}

void movedown(const int mot, float loo) {
  idle();
  SERIAL_ECHO("Adjusting...");
  SERIAL_ECHOLN(mot);
  if (loo>800){
    loo=800;  }
  SERIAL_ECHO("Correction...");
  SERIAL_ECHOLN(loo);              
  //Motors UP
  digitalWrite(Z_DIR_PIN, LOW); //High=up dir
  analogWrite(Z_ENABLE_PIN, 255); //Deactivate all z stepper
  
  // Deactivate all motors
  analogWrite(M1_ENABLE_PIN, 255);
  analogWrite(M2_ENABLE_PIN, 255);
  analogWrite(M3_ENABLE_PIN, 255);
  analogWrite(M4_ENABLE_PIN, 255);
  
  if (mot==1){ analogWrite(M1_ENABLE_PIN, 0); }
  else if (mot==2){ analogWrite(M2_ENABLE_PIN, 0); }
  else if (mot==3){ analogWrite(M3_ENABLE_PIN, 0); }
  else if (mot==4){ analogWrite(M4_ENABLE_PIN, 0); }
  
  // Move one step
  for (float x = 0; x < loo; x++) {
    digitalWrite(Z_STEP_PIN, HIGH);
    delay (5/2);
    digitalWrite(Z_STEP_PIN, LOW);
    delay (5/2);
  }

  // Deactivate all motors
  analogWrite(M1_ENABLE_PIN, 255);
  analogWrite(M2_ENABLE_PIN, 255);
  analogWrite(M3_ENABLE_PIN, 255);
  analogWrite(M4_ENABLE_PIN, 255);
  // Activate Z steppers to keep height
  analogWrite(Z_ENABLE_PIN, 0);
  idle();
}

float get_desviation (){
  idle();
  SERIAL_ECHOLN("Getting desviation");
  float desviation=0;
  const float precission = 0.1;
  relative_mode = true; //G91

  
  parser.parse("G1 Z-0.1 F1000");

  while (digitalRead(Z_MIN_PIN) != LOW){
	  GcodeSuite::process_parsed_command();
    planner.synchronize();
    desviation=desviation+precission;
  }
  
  relative_mode = false; //G90
  idle();
  return desviation;
}

float getMin(float* array, int size){
  idle();
  SERIAL_ECHOLN("Getting minimun");
  float minimum = array[0];
  for (int i = 0; i < size; i++)
  {
    if (array[i] < minimum) { minimum = array[i]; }
  }
  idle();
  return minimum;
}

float getMax(float* array, int size){
  idle();
  SERIAL_ECHOLN("Getting maximun");
  float maximun = array[0];
  for (int i = 0; i < size; i++)
  {
    if (array[i] > maximun) { maximun = array[i]; }
  }
  idle();
  return maximun;
}

float get_steps(float desviation){
  idle();
  // 400 steps per mm
  SERIAL_ECHOLN("Getting steps");
  float toapply = 400*desviation;
  idle();
  return toapply;
}

void a_bit_down(){
  idle();
  relative_mode = false; //G90
  parser.parse("G1 Z5 F1000");
  GcodeSuite::process_parsed_command();
  planner.synchronize();
}
 
 
void GcodeSuite::M777() {
  float maxim=100;
  int rep_times=0;
  
  relative_mode = false; //G90
  
  //parser.parse("M113");
  //process_parsed_command();
  
  //parser.parse("M106");
  //GcodeSuite::process_parsed_command();

  //parser.parse("M104 S120");
  //GcodeSuite::process_parsed_command();
  
  parser.parse("M206 X0 Y0 Z0");
  GcodeSuite::process_parsed_command();
  
  parser.parse("G28");
  GcodeSuite::process_parsed_command();
  planner.synchronize();

  
  while ( (maxim > 1) && (rep_times < 3) ){
	  
	  a_bit_down();

	  parser.parse("G1 X145 Y110 F2500");
	  GcodeSuite::process_parsed_command();
	  planner.synchronize();
	  float desv0 = get_desviation();

	  a_bit_down();

	  parser.parse("G1 X210 Y30 F2500");
	  GcodeSuite::process_parsed_command();
	  planner.synchronize();
	  float desv2 = get_desviation();

	  a_bit_down();

	  parser.parse("G1 X210 Y210 F2500");
	  GcodeSuite::process_parsed_command();
	  planner.synchronize();
	  float desv1 = get_desviation();

	  a_bit_down();

	  parser.parse("G1 X80 Y210 F2500");
	  GcodeSuite::process_parsed_command();
	  planner.synchronize();
	  float desv3 = get_desviation();

	  a_bit_down();

	  parser.parse("G1 X80 Y30 F2500");
	  GcodeSuite::process_parsed_command();
	  planner.synchronize();
	  float desv4 = get_desviation();
	  
	  /* float angles[]={desv1,desv2,desv3,desv4};
	  float minim = getMin(angles, 4);  // pass the array and its size */

	  float corrected1 = get_steps(desv1-desv0);
	  float corrected2 = get_steps(desv2-desv0);
	  float corrected3 = get_steps(desv3-desv0);
	  float corrected4 = get_steps(desv4-desv0);
	  SERIAL_ECHOLN("Desviation:");
	  SERIAL_ECHO(corrected1);
	  SERIAL_ECHO(",");
	  SERIAL_ECHO(corrected2);
	  SERIAL_ECHO(",");
	  SERIAL_ECHO(corrected3);
	  SERIAL_ECHO(",");
	  SERIAL_ECHOLN(corrected4);
	  
	  a_bit_down();
	  
	  float abscorrected1;
	  float abscorrected2;
	  float abscorrected3;
	  float abscorrected4;
	  
	  if (corrected1 > 0) { moveup(1, corrected1); abscorrected1=corrected1; }
	  else { movedown(1, -corrected1); abscorrected1=-corrected1; }
	  planner.synchronize();
	  
	  if (corrected2 > 0) { moveup(2, corrected2); abscorrected2=corrected2; }
	  else { movedown(2, -corrected2); abscorrected2=-corrected2; }
	  planner.synchronize();
	  
	  if (corrected3 > 0) { moveup(3, corrected3); abscorrected3=corrected3; }
	  else { movedown(3, -corrected3); abscorrected3=-corrected3; }
	  planner.synchronize();
	  
	  if (corrected4 > 0) { moveup(4, corrected4); abscorrected4=corrected4; }
	  else { movedown(4, -corrected4); abscorrected4=-corrected4; }
	  planner.synchronize();

	  float finalangles[]={abscorrected1,abscorrected2,abscorrected3,abscorrected4};
	  maxim = getMax(finalangles, 4);
	  if ( (corrected1==corrected2) && (corrected1==corrected3) && (corrected1==corrected4) ) { maxim=0; }
	  SERIAL_ECHO("Maxim: ");
	  SERIAL_ECHOLN(maxim); 
	  rep_times=rep_times+1;
  }
  
  //parser.parse("M104 S0");
  //GcodeSuite::process_parsed_command();
  
  //parser.parse("M501");
  //GcodeSuite::process_parsed_command();

  parser.parse("G28 X Y");
  GcodeSuite::process_parsed_command();
  planner.synchronize();

  parser.parse("G28 Z");
  GcodeSuite::process_parsed_command();
  planner.synchronize();
  
  //parser.parse("M501");
  //GcodeSuite::process_parsed_command();
  
}