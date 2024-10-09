#include <Arduino.h>
#include <LineSensors.h>
#include <geometry2D.h>
#include <SteeringController.h>
#include "GlobalVariables.h"
#include "PID.h"

const float PID_Kp = 1.0f;

LineSensors lineSensors(TOTAL_LINE_SENSORS);
float sensorsReadings[TOTAL_LINE_SENSORS];
Point2D linePosition;
SteeringController steeringController(1.0f, 0.0f, -1.0f);
PID steering_pid(1.0, 0.0, 0.0, -1.0, 1.0, 0.0f);


void setup()
{
  Serial.begin(SERIAL_PORT_BAUDRATE);
  /*
  while (!Serial) {
    delay(100);
  }*/

  for (size_t i = 0; i < TOTAL_LINE_SENSORS; i++) {
    pinMode(linesensors_pins[i], INPUT);
  }

  steeringController.attach(-1, -1, -1, -1);

  lineSensors.setPins(linesensors_pins, TOTAL_LINE_SENSORS);
  lineSensors.SetBackgroundColorOnlyCalibrationAvarages(BackgroundColorOnlyCalibrationAvarages);
  lineSensors.SetLineColorOlyCalibrationAvarages(LineColorOlyCalibrationAvarages);
}

float rotateTreshold = 0.5f;
float maxSpeed = 0.25f;
float speed = maxSpeed;
float right_track_speed_percentage = 1.0f;
float left_track_speed_percentage = 1.0f;
float PID_out_right, PID_out_left;
float steering_correction_pid_out;
Point2D middleLineMax, middleLineMin;
float blackLinePositionX, blackLinePositionY;

void loop()
{
  lineSensors.read();
  middleLineMax = lineSensors.getMaxValue();
  middleLineMin = lineSensors.getMinValue();


  blackLinePositionX = middleLineMax.x;
  blackLinePositionY = middleLineMax.y;
  Serial.print("Max Posx:" + String(blackLinePositionX));
  Serial.print('\t');
  Serial.print("Max Posy:" + String(blackLinePositionY));
  Serial.print('\t');
  Serial.print("Min Posx:" + String(middleLineMin.x));
  Serial.print('\t');
  Serial.print("Min Posy:" + String(middleLineMin.y));
  Serial.print('\t');

  /*
  Pos_x:   Left: -1    Right: +1
  */

  /*
  Black line is on the right sensor
  Go full right
  Stop right wheel and accelerate left wheel

  left_wheel = 1;
  right_wheel = 0;

  blackLinePositionX = 1;
  steering_correction_pid_out = -1;

  left_wheel = 1 - steering_correction_pid_out; =  1 -(-1) = 2
  right_wheel = 1 + steering_correction_pid_out; = 1 +(-1) = 0



  Black line is on the left sensor
  Go full left
  Accelerate right wheel and stop left wheel

  left_wheel = 0;
  right_wheel = 1;

  blackLinePositionX = -1;
  steering_correction_pid_out = 1;

  left_wheel = 1 - steering_correction_pid_out; =  1 -(1) = 0
  right_wheel = 1 + steering_correction_pid_out; = 1 +(1) = 2
  */
 
  steering_correction_pid_out = steering_pid.calculate(0, blackLinePositionX, 1.0);

  left_track_speed_percentage = 1 - steering_correction_pid_out;
  right_track_speed_percentage = 1 + steering_correction_pid_out;


  left_track_speed_percentage = MIN(left_track_speed_percentage, 1.0f);
  left_track_speed_percentage = MAX(left_track_speed_percentage, -1.0f);
  right_track_speed_percentage = MIN(right_track_speed_percentage, 1.0f);
  right_track_speed_percentage = MAX(right_track_speed_percentage, -1.0f);


  Serial.print("left_track:" + String(left_track_speed_percentage));
  Serial.print('\t');
  Serial.print("right_track:" + String(right_track_speed_percentage));
  Serial.println();

  //toArduino = String(speed) + ";"+ String(left_track_speed_percentage) + ";" + String(right_track_speed_percentage);
  //serial_soft1.print(toArduino);
  steeringController.write(speed, left_track_speed_percentage, right_track_speed_percentage);
  delay(10);
  //steeringController.write(1.0f, 1.0f, 1.0f);
}