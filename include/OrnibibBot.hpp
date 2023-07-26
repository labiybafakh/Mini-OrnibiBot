#ifndef ORNIBIBOT_HPP
#define ORNIBIBOT_HPP

#include <Arduino.h>
#include <iostream>
#include <math.h>
#include <string.h>

#define sine 1
#define square 2
#define saw 3
#define rev_saw 4
#define triangle 5

typedef struct{
  volatile uint16_t total_time;
  volatile uint16_t counter;
  volatile uint8_t flag;
} wing_raw_data;

enum wing{
  left = 0,
  right = 1
};

class OrnibiBot
{
private:
  double scalar = 1;

  struct tailPosition
  {
    unsigned int roll;
    unsigned int pitch;
  };

  typedef struct {
    uint8_t offset;
    uint16_t time;
    uint8_t amplitude;
    volatile int8_t signal;
    float frequency;
  } flappingParameter;

  typedef struct{
    volatile float actual_right;
    volatile float actual_left;
    volatile float desired_right;
    volatile float desired_left;
    volatile float power_left;
    volatile float power_right;
  } wingData;

  volatile int8_t sineFlap();
  volatile int8_t squareFlap();
  volatile int8_t sawFlap();
  volatile int8_t triangleFlap();
  volatile int8_t reverse_sawFlap();
  uint32_t getRawPosition();
  double getPositioninRadians();
  int8_t getPositioninDegrees();

  volatile uint16_t _periode;
  volatile uint16_t _flapping;

public:
  flappingParameter *_flappingParam;
  wingData *p_wing_data;

  tailPosition tail_position;

  OrnibiBot();
  ~OrnibiBot();
  volatile uint16_t getFlapMs();
  volatile int8_t flappingPattern(uint8_t pattern);


};

#endif