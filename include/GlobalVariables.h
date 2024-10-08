#ifndef __GLOBALVARIABLES_H__
#define __GLOBALVARIABLES_H__

#include <Arduino.h>


#define BLACK_COLOR_THRESHOLD 0.50f

#define TOTAL_LINE_SENSORS 8

#define LINE_SENSOR_1_PIN A2
#define LINE_SENSOR_2_PIN A3
#define LINE_SENSOR_3_PIN A4
#define LINE_SENSOR_4_PIN A5
#define LINE_SENSOR_5_PIN A6
#define LINE_SENSOR_6_PIN A7
#define LINE_SENSOR_7_PIN A8
#define LINE_SENSOR_8_PIN A9

#define SERIAL_PORT_BAUDRATE 230400  //230400

static int linesensors_pins[TOTAL_LINE_SENSORS] = {
  LINE_SENSOR_1_PIN,
  LINE_SENSOR_2_PIN,
  LINE_SENSOR_3_PIN,
  LINE_SENSOR_4_PIN,
  LINE_SENSOR_5_PIN,
  LINE_SENSOR_6_PIN,
  LINE_SENSOR_7_PIN,
  LINE_SENSOR_8_PIN
  };

  float BackgroundColorOnlyCalibrationAvarages[TOTAL_LINE_SENSORS] = {
    286.0f,
    194.0f,
    267.0f,
    368.0f,
    332.0f,
    0.0f,
    0.0f,
    0.0f
  };

float LineColorOlyCalibrationAvarages[TOTAL_LINE_SENSORS] = {
    3211.0f,
    3013.0f,
    3040.0f,
    3265.0f,
    3358.0f,
    0.0f,
    0.0f,
    0.0f
  };



#endif