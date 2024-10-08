#include <Arduino.h>
#include <LineSensors.h>
#include <geometry2D.h>
#include <SteeringController.h>
#include "GlobalVariables.h"




const float PID_Kp = 1.0f;



LineSensors lineSensors(TOTAL_LINE_SENSORS);
float sensorsReadings[TOTAL_LINE_SENSORS];
Point2D linePosition;
SteeringController steeringController(1.0f, 0.0f, -1.0f);





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
float right_track_speed_cercentage = 1.0f;
float left_track_speed_cercentage = 1.0f;
float PID_out_right, PID_out_left;
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


  if (blackLinePositionX < 0.0f) {
    PID_out_right = 1.0f;
    if (blackLinePositionX <= (-rotateTreshold)) {
      PID_out_left = (blackLinePositionX + rotateTreshold) * 2.0f;
    }
    else{
      PID_out_left = ((rotateTreshold) + blackLinePositionX) * 2.0f;
    }
  }
  else{
    PID_out_left = 1.0f;
    if (blackLinePositionX <= (rotateTreshold)) {
      PID_out_right = (rotateTreshold - blackLinePositionX) * 2.0f;
    }
    else{
      PID_out_right = ((-blackLinePositionX) + rotateTreshold) * 2.0f;
    }
  }
  
  PID_out_right = PID_out_right * PID_Kp;
  PID_out_left = PID_out_left * PID_Kp;

  right_track_speed_cercentage = PID_out_right;
  left_track_speed_cercentage = PID_out_left;


  left_track_speed_cercentage = MIN(left_track_speed_cercentage, 1.0f);
  left_track_speed_cercentage = MAX(left_track_speed_cercentage, -1.0f);
  right_track_speed_cercentage = MIN(right_track_speed_cercentage, 1.0f);
  right_track_speed_cercentage = MAX(right_track_speed_cercentage, -1.0f);


  Serial.print("left_track:" + String(left_track_speed_cercentage));
  Serial.print('\t');
  Serial.print("right_track:" + String(right_track_speed_cercentage));

  Serial.println();

  //toArduino = String(speed) + ";"+ String(left_track_speed_cercentage) + ";" + String(right_track_speed_cercentage);
  //serial_soft1.print(toArduino);
  steeringController.write(speed, left_track_speed_cercentage, right_track_speed_cercentage);
  delay(10);
  //steeringController.write(1.0f, 1.0f, 1.0f);
}