# Krotos3D

Custom [Marlin](https://github.com/MarlinFirmware/Marlin) firmware for [Krotos3d](https://kustomizit.com/build-your-own-3d-printer/) printer

![img](pic.png)

## New Gcode
```
M777 - HW bed leveling
```
New custom gcode to use the 4-motor Z axis for bed leveling
By default will run 5 times until the bed is leveled. Can be modified with R parameter
```
M777 R1
```

## Remarkable commands
```
G28 - Auto Home
G29 - Bed Leveling
M80 - Power On PSU
M81 - Power Off PSU
M114 - Get Current Position
M420 - Bed Leveling State
M500 - Save Settings
M503 - Report Settings
M851 - XYZ Probe Offset
```

## Tips
```
M420 S1 - before printing enables bed leveling
M851 Z-X.XX - sets nozzle diff
```

## Configure nozzle height
```
M777 R1 - HW bed leveling
G29 - Bed leveling
G28 Z - Home z at center
G91 - Relative positioning
G1 Z-0.1 - Move up the bed until it touches the nozzle
M114 - Get the Z height X.XX
M851 Z-X.XX - Set the nozzle height to X.XX
```

## Updated files

1. M777.cpp
   - Added custom gcode
2. pins_RAMPS.h
   - Added z axis motors pins
3. gcode.h
   - Added M777
4. gcode.cpp
   - Added 777
5. configuration.h
   - Updated e driver to drv8825
   - Z_MIN_ENDSTOP_INVERTING to true
   - Z_MIN_PROBE_ENDSTOP_INVERTING to true
   - DEFAULT_AXIS_STEPS_PER_UNIT to { 400, 400, 400, 97 }
   - DEFAULT_MAX_ACCELERATION
   - Update ACCELERATION
   - define USE_PROBE_FOR_Z_HOMING
   - define FIX_MOUNTED_PROBE
