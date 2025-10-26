#include <Arduino.h>
#include <iostream>
#include <memory>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include <atomic>
#include "ESP32Servo.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <HardwareSerial.h>

enum ServoType{
  SBUS = 0,
  PWM = 1
};

const ServoType SERVO_TYPE = PWM; 

// PWM servo pins 
const int LEFT_WING_PIN = 1;
const int RIGHT_WING_PIN = 2;
const int TAIL_ROLL_PIN = 5;
const int TAIL_PITCH_PIN = 18;

WiFiUDP udp;

const char* ssid = "ornibibot";
const char* password = "ornibibot5208";
const int udpPort = 4210;

// Define the static IP address for the AP
IPAddress local_ip(192,168,3,15);
IPAddress gateway(192,168,3,15);
IPAddress subnet(255,255,255,0);


HardwareSerial SerialPort(0); // use UART1


struct flapping{
  uint8_t amplitude;
  uint8_t offset;
  uint8_t roll;
  double freq;
};

struct ornibibot_param{
  std::atomic<float> frequency;
  std::atomic<int8_t> roll;
  std::atomic<int8_t> pitch;
};

TaskHandle_t Task1;
TaskHandle_t Task2;

int pos;
int payload=0;

std::atomic<std::int_fast8_t> wing_position;

ornibibot_param ornibibot_parameter;

// PWM servo objects (only used when SERVO_TYPE = PWM)
Servo leftWingServo;
Servo rightWingServo; 
Servo tailRollServo;
Servo tailPitchServo;

uint16_t degToSignal(int8_t pos){
    // Limit position to safe range
    if(pos > 70)       pos = 70;
    else if(pos < -70) pos = -70;

    if(SERVO_TYPE == SBUS) {
        // SBUS: Mid = 1023, range for ±70 degrees
        if(pos > 90)       pos = 90;
        else if(pos < -90) pos = -90;
        return (uint16_t)(1023 - (-pos * 11.36));
    } else {
        // PWM: 900-2100μs range for ±70 degrees, center = 1500μs
        return (uint16_t)(1500 + (pos * (1200.0/140.0))); // 1200μs range / 140° total = 8.57μs/degree
    }
}

uint16_t degToSignalTail(int8_t pos){
    // Limit position to safe range
    if(pos > 70)       pos = 70;
    else if(pos < -70) pos = -70;

    if(SERVO_TYPE == SBUS) {
        // SBUS: Mid = 1023, range for ±80 degrees
        if(pos > 80)       pos = 80;
        else if(pos < -80) pos = -80;
        return (uint16_t)(1023 - (-pos * 12.79));
    } else {
        // PWM: 900-2100μs range for ±70 degrees, center = 1500μs
        return (uint16_t)(1500 + (pos * (1200.0/140.0))); // Same calculation as wing servos
    }
}

void setPosition(uint16_t pos_left, uint16_t pos_right, uint16_t pos_tail_roll, uint16_t pos_tail_pitch) {
    if(SERVO_TYPE == SBUS) {
        // SBUS mode: Send SBUS packet
        const size_t SBUS_BUFFER = 25;
        uint8_t packet_sbus[SBUS_BUFFER];
        memset(packet_sbus, 0x00, SBUS_BUFFER);

        uint16_t zeroing = 0;

        packet_sbus[0] = 0x0f;
        packet_sbus[1] = (uint8_t)(pos_right & 0xff);
        packet_sbus[2] = (uint8_t)((pos_right >> 8) & 0x07 ) | ((pos_left  << 3 ) );
        packet_sbus[3] = (uint8_t)((pos_left >> 5) & 0x3f ) | (pos_tail_roll  << 6);
        packet_sbus[4] = (uint8_t)((pos_tail_roll >> 2) & 0xFF);
        packet_sbus[5] = (uint8_t)((pos_tail_roll >> 10) & 0x01) | (pos_tail_pitch << 1);
        packet_sbus[6] = (uint8_t)(pos_tail_pitch >> 7) & 0x0f | (zeroing << 4);

        packet_sbus[23] = 0x00;
        packet_sbus[24] = 0x00;

        SerialPort.write(packet_sbus, sizeof(packet_sbus));
    } else {
        // PWM mode: Control servos directly (333Hz frequency)
        leftWingServo.writeMicroseconds(pos_left);
        rightWingServo.writeMicroseconds(pos_right);
        tailRollServo.writeMicroseconds(pos_tail_roll);
        tailPitchServo.writeMicroseconds(pos_tail_pitch);
    }
}


flapping *flapping_param;

void paramUpdate( void * pvParameters ){
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());
  int time_ = 0;
  const TickType_t xDelay = 1 / portTICK_PERIOD_MS;
  for(;;){

      uint16_t periode_ = 1000 / ornibibot_parameter.frequency;
      wing_position = (flapping_param->amplitude * sin((2 * M_PI * time_) / periode_)) ;

      if(wing_position > 0) wing_position = flapping_param->amplitude + flapping_param->offset;
      else wing_position = (flapping_param->amplitude + flapping_param->offset) * -1;

      if (time_ < periode_) {
          time_++;
      } else {
          time_ = 0;
      }
        
      delay(xDelay);
    }
}

void motorUpdate( void * pvParameters ){
  Serial.print("Task2 running on core ");
  Serial.println(xPortGetCoreID());
  const TickType_t xDelay = 5 / portTICK_PERIOD_MS;
  for(;;){
    const int adjustment = 0;
    const int minimum_pitch_tail = 20;
    
    if(ornibibot_parameter.frequency < 0.5){
        int8_t tail = minimum_pitch_tail + ornibibot_parameter.pitch;

        setPosition(
          degToSignal((25+ornibibot_parameter.roll-adjustment)),
          degToSignal((25-ornibibot_parameter.roll)*-1),
          degToSignalTail(tail*-1),
          degToSignalTail(tail)
        );
    }

    else{
        int8_t tail = minimum_pitch_tail + ornibibot_parameter.pitch;
        
          setPosition(
          degToSignal((wing_position+ornibibot_parameter.roll-adjustment)),
          degToSignal((wing_position-ornibibot_parameter.roll)*-1),
          degToSignalTail(tail*-1),
          degToSignalTail(tail)
        );
        // }
    }

    vTaskDelay(xDelay);
  }
}

void deserializeUDP(){
    uint8_t buffer[3] = {0, 0, 0};
    int packetSize = udp.parsePacket();

    if(packetSize>2){
        udp.read(buffer, sizeof(buffer));

        ornibibot_parameter.frequency = (float)buffer[0]*0.1f;
        ornibibot_parameter.roll = (int8_t) buffer[1];
        ornibibot_parameter.pitch = (int8_t) buffer[2];
    }
}

void setup() {
  flapping_param = (flapping *) malloc(sizeof(flapping));

  // Initialize servo system based on type
  if(SERVO_TYPE == SBUS) {
    // SBUS setup
    SerialPort.begin(100000, SERIAL_8E2, D7, D6);
  } else {
    // PWM setup: Attach servos with 333Hz frequency (3000μs period)
    leftWingServo.attach(LEFT_WING_PIN, 900, 2100);
    rightWingServo.attach(RIGHT_WING_PIN, 900, 2100);
    tailRollServo.attach(TAIL_ROLL_PIN, 900, 2100);
    tailPitchServo.attach(TAIL_PITCH_PIN, 900, 2100);
    
    // Set PWM frequency to 333Hz for all servos
    leftWingServo.setPeriodHertz(333);
    rightWingServo.setPeriodHertz(333);
    tailRollServo.setPeriodHertz(333);
    tailPitchServo.setPeriodHertz(333);
  }

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  WiFi.softAPConfig(local_ip, gateway, subnet);

  udp.begin(udpPort);

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

    // if(payload == 100) flapping_param->amplitude = 70;
    // else flapping_param->amplitude = 60;
    flapping_param->amplitude = 60;
    flapping_param->offset = 0;
    // ornibibot_parameter.frequency = 5.0;

    if(WiFi.status() != WL_DISCONNECTED){
        deserializeUDP();

      // ornibibot_parameter.frequency = 0.0;
      // SerialPort.print(incomingPacket[0]);
      digitalWrite(LED_BUILTIN, HIGH);
    }
    else{
            ornibibot_parameter.frequency = 0.0;

          digitalWrite(LED_BUILTIN, LOW);

    }

    delay(5);

}
