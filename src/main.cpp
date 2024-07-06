#include <Arduino.h>
#include <iostream>
#include <memory>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include <atomic>
#include "ESP32Servo.h"

#define LEFT 6
#define RIGHT 7

Servo left_servo;
Servo right_servo;

struct flapping{
  uint8_t amplitude;
  uint8_t offset;
  uint8_t roll;
  double freq;
};

TaskHandle_t Task1;
TaskHandle_t Task2;

int pos;

std::atomic<std::int_fast8_t> wing_position;

uint16_t min_pulse_width = 1000; // Minimum pulse width in microseconds
uint16_t max_pulse_width = 2000; // Maximum pulse width in microseconds

int min_angle = -60;       // Minimum angle in degrees
int max_angle = 60;        // Maximum angle in degrees

uint16_t degreeToPulse(int8_t degree) {

    // Ensure the input degree is within the valid range
    if (degree < min_angle) degree = min_angle;
    if (degree > max_angle) degree = max_angle;
    
    // Perform linear mapping
    uint16_t pulse = (uint16_t)((degree - min_angle) * (max_pulse_width - min_pulse_width) / (max_angle - min_angle) + min_pulse_width);
    
    return pulse;
}

flapping *flapping_param;

void paramUpdate( void * pvParameters ){
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());
  int time_ = 0;
  for(;;){

      uint16_t periode_ = 1000 / flapping_param->freq;
      wing_position = flapping_param->amplitude * sin((2 * M_PI * time_) / periode_) + flapping_param->offset;

      if (time_ < periode_) {
          time_++;
      } else {
          time_ = 0;
      }
        
      delay(1);
    }
}
void motorUpdate( void * pvParameters ){
  Serial.print("Task2 running on core ");
  Serial.println(xPortGetCoreID());
  for(;;){
    
    left_servo.writeMicroseconds(degreeToPulse(wing_position));
    right_servo.writeMicroseconds(degreeToPulse(wing_position) * -1); // to adjust for a different angle between left and right

    Serial.println(degreeToPulse(wing_position));

    delay(1);

  }
}

void setup() {
  flapping_param = (flapping *) malloc(sizeof(flapping));

  Serial.begin(115200);

  left_servo.setPeriodHertz(300);
  left_servo.attach(LEFT, min_pulse_width, max_pulse_width);

  right_servo.setPeriodHertz(300);
  right_servo.attach(RIGHT, min_pulse_width, max_pulse_width);

  xTaskCreatePinnedToCore(
                    paramUpdate,   /* Task function. */
                    "Task1",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task1,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */
  delay(500);
  xTaskCreatePinnedToCore(
                    motorUpdate,   /* Task function. */
                    "Task2",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task2,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 0 */
  delay(500);
}

void loop() {
    flapping_param->amplitude = 30;
    flapping_param->offset = 0;
    flapping_param->freq = 4;
}

