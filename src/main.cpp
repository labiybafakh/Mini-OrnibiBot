#include <Arduino.h>
#include <iostream>
#include <memory>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include "ESP32Servo.h"
#include "OrnibibBot.hpp"

OrnibiBot robot;

#define LEFT 21
#define RIGHT 22

Servo left_servo;
Servo right_servo;


TaskHandle_t Task1;
TaskHandle_t Task2;

int pos;

uint32_t current_time, last_time, dif_time;

void paramUpdate( void * pvParameters ){
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());

  for(;;){

        delay(1);
    }
}
void motorUpdate( void * pvParameters ){
  Serial.print("Task2 running on core ");
  Serial.println(xPortGetCoreID());
  for(;;){
  
  robot._flappingParam->frequency = 0.5;
  uint16_t timing_signal = abs((int) (1000/robot._flappingParam->frequency));
  // Serial.print(timing_signal);
  // Serial.print("\t");
  current_time = millis();

  if((current_time - last_time) >= (timing_signal) ){
    // A full period has passed, reset the timer
    last_time = current_time;
  }

  // Create a square wave by alternating between two states
  if ((current_time - last_time) < (int)(timing_signal*0.5)) {
    // We are in the first half of the period
    left_servo.write(1500 + (int)(50*7.778f));
    right_servo.write(1500 - (int)(50*7.778f));
    // Serial.println(1);
  } 
  else{
    // We are in the second half of the period
    left_servo.write(1500 + (int)(-10*7.778f));
    right_servo.write(1500 - (int)(-10*7.778f));
    // Serial.println(2);
  }

  // Serial.println((timing_signal*0.5));

  delay(1);

  }
}

void setup() {
  Serial.begin(115200);
  while(!Serial);
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);

  left_servo.setPeriodHertz(50);
  left_servo.attach(LEFT, 800, 2200);

  right_servo.setPeriodHertz(50);
  right_servo.attach(RIGHT, 800, 2200);

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


}

// void serialEvent(){
//   if(Serial.available())
// }