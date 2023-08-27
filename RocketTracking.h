#pragma once
#include "Constants.h"
#include "Parameter.h"

/*
LoRa pin configuration (Mega)

GND -> GND
VIN -> 3.3V
DI0O -> 3                                                                                                                                                                                                                                                                                                      
RESET -> 9
MOSI -> 51
MISO -> 50
SCLK -> 52
NSS -> 53

*/

//Main Rocket class 
  
class RocketTracking : public Parameter
{
public:
  char data_buffer[DATA_SIZE];  
  int time_stamp;
  Adafruit_BNO055 bno;
  Adafruit_MPL3115A2 baro;
  sensors_event_t gyroData, linearAccelData, orientationData, event;

public:
  RocketTracking();
  int Init(RocketTracking& rocket);
  void GetBNOData(RocketTracking& rocket);
  void GetMPLData(RocketTracking& rocket);
  void SendBNOData(RocketTracking& rocket);
  void SaveData(RocketTracking& rocket);
  void SaveApogeeData(RocketTracking& rocket);
  void SendApogeeData(RocketTracking& rocket);
};
