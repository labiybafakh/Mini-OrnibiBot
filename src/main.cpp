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

const int servoPins[] = {D0, D1, D2, D3};  // Adjust these pins as needed

Servo left_servo;
Servo right_servo;
Servo pitch_servo;
Servo roll_servo;

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

uint16_t min_pulse_width = 920; // Minimum pulse width in microseconds
uint16_t max_pulse_width = 2120; // Maximum pulse width in microseconds

int min_angle = -60;       // Minimum angle in degrees
int max_angle = 60;        // Maximum angle in degrees

ornibibot_param ornibibot_parameter;

uint16_t degreeToPulse(int8_t degree) {

    // Ensure the input degree is within the valid range
    if (degree < min_angle) degree = min_angle;
    if (degree > max_angle) degree = max_angle;
    
    // Perform linear mapping
    uint16_t pulse = (uint16_t)((degree - min_angle) * (max_pulse_width - min_pulse_width) / (max_angle - min_angle) + min_pulse_width);
    
    return pulse;
}

uint16_t degToSignal(int8_t pos){
    //Rotate Servo from -60 to 60 Degrees
    //Mid Servo using SBUS is 1023
    //Upstroke<1023 - Downstroke>1023
    if(pos>60)         pos=60;
    else if(pos<-60)   pos=-60;

    return (uint16_t)(1023 - (-pos*17)); //reversed to adjust upstroke-downstroke
}

void setPosition(uint16_t pos_left, uint16_t pos_right){
    const size_t SBUS_BUFFER = 25;
    uint8_t packet_sbus[SBUS_BUFFER];
    memset(packet_sbus, 0x00, SBUS_BUFFER);

    uint16_t zeroing = 0;

    packet_sbus[0] = 0x0f;
    packet_sbus[1] = (uint8_t)(pos_left & 0xff);
    packet_sbus[2] = (uint8_t)((pos_left >> 8) & 0x07 ) | ((pos_right  << 3 ) );
    packet_sbus[3] = (uint8_t)((pos_right >> 5) & 0x3f ) | (zeroing  << 6);
    packet_sbus[4] = (uint8_t)((zeroing >> 2) & 0xFF);

    // // Fill the rest of the packet with zeros (assuming no other channels are used)
    // for (int i = 5; i < 23; i++) {
    //     packet_sbus[i] = 0x00;
    // }

    // Stop byte(s)
    packet_sbus[23] = 0x00;
    packet_sbus[24] = 0x00;

    SerialPort.write(packet_sbus, sizeof(packet_sbus));
}

uint16_t degreeToPulseTail(int8_t degree) {
    uint16_t min_pulse_width_ = 1100; // Minimum pulse width in microseconds
    uint16_t max_pulse_width_ = 1900; // Maximum pulse width in microseconds

    int min_angle_ = -45;       // Minimum angle in degrees
    int max_angle_ = 45;        // Maximum angle in degrees

    // Ensure the input degree is within the valid range
    if (degree < min_angle_) degree = min_angle_;
    if (degree > max_angle_) degree = max_angle_;
    
    // Perform linear mapping
    uint16_t pulse = (uint16_t)((degree - min_angle_) * (max_pulse_width_ - min_pulse_width_) / (max_angle_ - min_angle_) + min_pulse_width_);
    
    return pulse;
}

flapping *flapping_param;

void paramUpdate( void * pvParameters ){
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());
  int time_ = 0;
  const TickType_t xDelay = 1 / portTICK_PERIOD_MS;
  for(;;){

      uint16_t periode_ = 1000 / ornibibot_parameter.frequency;
      wing_position = flapping_param->amplitude * sin((2 * M_PI * time_) / periode_) + flapping_param->offset;

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
    
    if(ornibibot_parameter.frequency < 0.5){

        setPosition(
          degToSignal(25),
          degToSignal(25*-1)
        );
        // left_servo.writeMicroseconds(degreeToPulse(0));
        // right_servo.writeMicroseconds(degreeToPulse(-30)); // to adjust for a different angle between left and right
    }

    else{
        setPosition(
          degToSignal(wing_position),
          degToSignal(wing_position*-1)
        );
        // left_servo.writeMicroseconds(degreeToPulse(wing_position));
        // right_servo.writeMicroseconds(degreeToPulse(-wing_position)); // to adjust for a different angle between left and right
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

  // left_servo.setPeriodHertz(300);
  // left_servo.attach(servoPins[0], min_pulse_width, max_pulse_width);

  // right_servo.setPeriodHertz(300);
  // right_servo.attach(servoPins[1], min_pulse_width, max_pulse_width);

  pitch_servo.setPeriodHertz(250);
  pitch_servo.attach(servoPins[2], 1100, 1900);

  roll_servo.setPeriodHertz(250);
  roll_servo.attach(servoPins[3], 1100, 1900);

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

    flapping_param->amplitude = 60;
    flapping_param->offset = 0;
    deserializeUDP();

    if(ornibibot_parameter.frequency < 0.5){
      roll_servo.writeMicroseconds(degreeToPulseTail(0));
      pitch_servo.writeMicroseconds(degreeToPulseTail(30*-1));
    }
    else{
      roll_servo.writeMicroseconds(degreeToPulseTail(ornibibot_parameter.roll));
      pitch_servo.writeMicroseconds(degreeToPulseTail(ornibibot_parameter.pitch*-1));
    }

    delay(5);

}
