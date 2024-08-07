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

std::atomic<std::int_fast8_t> wing_position;

ornibibot_param ornibibot_parameter;

uint16_t degToSignal(int8_t pos){
    //Rotate Servo from -60 to 60 Degrees
    //Mid Servo using SBUS is 1023
    //Upstroke<1023 - Downstroke>1023
    if(pos>90)         pos=90;
    else if(pos<-90)   pos=-90;

    return (uint16_t)(1023 - (-pos*11.36)); //reversed to adjust upstroke-downstroke
}

uint16_t degToSignalTail(int8_t pos){
    //Rotate Servo from -60 to 60 Degrees
    //Mid Servo using SBUS is 1023
    //Upstroke<1023 - Downstroke>1023
    if(pos>80)         pos=80;
    else if(pos<-80)   pos=-80;

    return (uint16_t)(1023 - (-pos*12.79)); //reversed to adjust upstroke-downstroke
}

void setPosition(uint16_t pos_left, uint16_t pos_right, uint16_t pos_tail_left, uint16_t pos_tail_right){
    const size_t SBUS_BUFFER = 25;
    uint8_t packet_sbus[SBUS_BUFFER];
    memset(packet_sbus, 0x00, SBUS_BUFFER);

    uint16_t zeroing = 0;

    packet_sbus[0] = 0x0f;
    packet_sbus[1] = (uint8_t)(pos_left & 0xff);
    packet_sbus[2] = (uint8_t)((pos_left >> 8) & 0x07 ) | ((pos_right  << 3 ) );
    packet_sbus[3] = (uint8_t)((pos_right >> 5) & 0x3f ) | (pos_tail_left  << 6);
    packet_sbus[4] = (uint8_t)((pos_tail_left >> 2) & 0xFF);
    packet_sbus[5] = (uint8_t)((pos_tail_left >> 10) & 0x01) | (pos_tail_right << 1);
    packet_sbus[6] = (uint8_t)(pos_tail_right >> 7) & 0x0f | (zeroing << 4);

    // // Fill the rest of the packet with zeros (assuming no other channels are used)
    // for (int i = 5; i < 23; i++) {
    //     packet_sbus[i] = 0x00;
    // }

    // Stop byte(s)
    packet_sbus[23] = 0x00;
    packet_sbus[24] = 0x00;

    SerialPort.write(packet_sbus, sizeof(packet_sbus));
}


flapping *flapping_param;

void paramUpdate( void * pvParameters ){
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());
  int time_ = 0;
  const TickType_t xDelay = 1 / portTICK_PERIOD_MS;
  for(;;){

      uint16_t periode_ = 1000 / ornibibot_parameter.frequency;
      wing_position = (flapping_param->amplitude * sin((2 * M_PI * time_) / periode_)) + flapping_param->offset;

      if(wing_position > 0) wing_position = flapping_param->amplitude;
      else wing_position = flapping_param->amplitude * -1;

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

    const int adjustment = 8;
    const int minimum_pitch_tail = 30;
    
    if(ornibibot_parameter.frequency < 0.5){

        if(ornibibot_parameter.pitch < -20) ornibibot_parameter.pitch = -20;

        if(ornibibot_parameter.roll > 25) ornibibot_parameter.roll = 25;
        else if(ornibibot_parameter.roll < -25) ornibibot_parameter.roll = -25;

        int8_t left_tail = minimum_pitch_tail - ornibibot_parameter.roll;
        int8_t right_tail = minimum_pitch_tail + ornibibot_parameter.roll;

        setPosition(
          degToSignal(25),
          degToSignal((25+adjustment)*-1),
          degToSignalTail(left_tail),
          degToSignalTail(right_tail*-1)
        );
    }

    else{
        if(ornibibot_parameter.pitch < -20) ornibibot_parameter.pitch = -20;

        if(ornibibot_parameter.roll > 25) ornibibot_parameter.roll = 25;
        else if(ornibibot_parameter.roll < -25) ornibibot_parameter.roll = -25;

        int8_t left_tail = minimum_pitch_tail + ornibibot_parameter.pitch - ornibibot_parameter.roll;
        int8_t right_tail = minimum_pitch_tail + ornibibot_parameter.pitch + ornibibot_parameter.roll;
        setPosition(
          degToSignal(wing_position),
          degToSignal((wing_position+adjustment)*-1),
          degToSignalTail(left_tail),
          degToSignalTail(right_tail*-1)
        );
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

  SerialPort.begin(100000, SERIAL_8E2, D7, D6);  // 1000000 baud, 8E2 config, TX on GPIO7 (D6), RX pin not used (-1)

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

    flapping_param->amplitude = 65;
    flapping_param->offset = 0;
    deserializeUDP();

    delay(5);

}
