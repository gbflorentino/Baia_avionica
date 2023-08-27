#pragma once 

#include "Constants.h"

class Parameter
{
public:
  int rssi_value;
  int acceleration_x, acceleration_y, acceleration_z;
  int gyro_x, gyro_y, gyro_z;
  int temp_celsius;
  int orientation_x, orientation_y, orientation_z; //sketch
  double lf_temp_celsius;
  double lf_acceleration_x, lf_acceleration_y, lf_acceleration_z;
  double lf_gyro_x, lf_gyro_y, lf_gyro_z;
  double lf_orientation_x, lf_orientation_y, lf_orientation_z; //sketch
  double altitude, initial_altitude, pressure;
  double altitude_diff;
  double previous_altitude;
  double altitude_offset;
  double max_altitude;
};
