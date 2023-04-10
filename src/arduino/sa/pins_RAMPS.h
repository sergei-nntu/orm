#define X_STEP_PIN         54
#define X_DIR_PIN          55
#define X_ENABLE_PIN       38
#define X_MIN_PIN           3
#define X_MAX_PIN           2

#define Y_STEP_PIN         60
#define Y_DIR_PIN          61
#define Y_ENABLE_PIN       56
#define Y_MIN_PIN          14
#define Y_MAX_PIN          15

#define Z_STEP_PIN         46
#define Z_DIR_PIN          48
#define Z_ENABLE_PIN       62
#define Z_MIN_PIN          18
#define Z_MAX_PIN          19

#define E_STEP_PIN         26
#define E_DIR_PIN          28
#define E_ENABLE_PIN       24

#define Q_STEP_PIN         36
#define Q_DIR_PIN          34
#define Q_ENABLE_PIN       30

#define SDPOWER            -1
#define SDSS               53
#define LED_PIN            13

#define FAN_PIN            9

#define PS_ON_PIN          12
#define KILL_PIN           -1

#define HEATER_0_PIN       10
#define HEATER_1_PIN       8
#define TEMP_0_PIN          13   // ANALOG NUMBERING
#define TEMP_1_PIN          14   // ANALOG NUMBERING

// AS5600 Encoder Input
#define X_ENCODER_IN      A3
#define Y_ENCODER_IN      A4
#define Z_ENCODER_IN      A5
#define E0_ENCODER_IN     A9
#define E1_ENCODER_IN     A10

//
// Misc. Functions
//
#define SDSS               53
#define LED_PIN            13

#ifndef FILWIDTH_PIN
  #define FILWIDTH_PIN      5   // Analog Input on AUX2
#endif

// define digital pin 4 for the filament runout sensor. Use the RAMPS 1.4 digital input 4 on the servos connector
#ifndef FIL_RUNOUT_PIN
  #define FIL_RUNOUT_PIN    4
#endif

#ifndef PS_ON_PIN
  #define PS_ON_PIN        12
#endif

//
// Průša i3 MK2 Multiplexer Support
//
#ifndef E_MUX0_PIN
  #define E_MUX0_PIN 40   // Z_CS_PIN
#endif
#ifndef E_MUX1_PIN
  #define E_MUX1_PIN 42   // E0_CS_PIN
#endif
#ifndef E_MUX2_PIN
  #define E_MUX2_PIN 44   // E1_CS_PIN
#endif
