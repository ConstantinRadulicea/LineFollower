#include "GlobalVariables.h"

void setup(){
    Serial.begin(SERIAL_PORT_BAUDRATE);

    for (size_t i = 0; i < TOTAL_LINE_SENSORS; i++) {
      pinMode(linesensors_pins[i], INPUT);
    }

}


void loop(){

  for (size_t i = 0; i < TOTAL_LINE_SENSORS; i++) {
      Serial.print(analogRead(linesensors_pins[i]));
      Serial.print('\t');
  }
  Serial.print('\n');
  delay(10);
}