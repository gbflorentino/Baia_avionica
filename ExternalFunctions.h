#pragma once

#include "Constants.h"

class ServoMotor
{
public:
  Servo main_servo;
  Servo redundant_servo;
  short int servo_signal; 

public:
  ServoMotor();
  void Init(ServoMotor& s_object);
  void ParachuteSignal(ServoMotor& s);
  void SetMotor(Servo& s_object, unsigned short sp);
};

void buzzerAsaBranca();

void GpsModule();

typedef enum{RUNNING, HIBERNATING, APOGEE1, MAIN, GROUND} RocketStates;
