#include "Constants.h"
#include "RocketTracking.h"


// RocketTracking class Constructor
RocketTracking::RocketTracking()
{

}

void InitBno(RocketTracking& rocket)
{
  rocket.bno = Adafruit_BNO055(55, 0x28, &Wire);
  LOGN("BNO");
  if (!rocket.bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  rocket.bno.setExtCrystalUse(true);
}

void InitMPL(RocketTracking& rocket)
{
  rocket.altitude_offset = 0;
  if (!rocket.baro.begin()) {
    Serial.println("Could not find sensor. Check wiring.");
    while(1);
  }
  else
    LOGN("\n:::::The MPL sensor was succefuly initialized:::::");

    
    rocket.baro.setSeaPressure(1017.0);
    for (int i = 0; i < 10; i++)
    {
      rocket.altitude_offset += rocket.baro.getAltitude();  
    }
    rocket.altitude_offset /= 10;
    LOGN("\n:::::The MPL sensor was succefuly calibrated:::::");
 
    rocket.initial_altitude = rocket.baro.getAltitude() - rocket.altitude_offset;
    rocket.max_altitude = 0;
}

void InitLoRa(RocketTracking& rocket)
{

  LoRa.setPins(NSS, NRESET, DI0);


  //LoRa.setSignalBandwidth(BANDWIDTH);

  if (!LoRa.begin(433E6))
  {
    LOGN("Falha em iniciar o LoRa");
    while (1);
    delay(500);
  }

  LOGN("LoRa Inicializado");
}

//Function to initialize the electronic components of the rocket
int RocketTracking::Init(RocketTracking& rocket)
{
  unsigned int rocket_init_status = 0;
  LOGN("Init");

  //InitBno(rocket);

  //InitLoRa(rocket);

  InitMPL(rocket);
  InitBno(rocket);
  LOGN("\n:::::The setup was succefuly initialized:::::");
  rocket_init_status = 1;
  delay(2000);

  return rocket_init_status;
}

//Print some data, just for debugging
//Auxiliar Functions

void GetBNOAccelData(RocketTracking& rocket_data)
{
  rocket_data.bno.getEvent(&rocket_data.linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  rocket_data.acceleration_x = rocket_data.linearAccelData.acceleration.x;
  rocket_data.acceleration_y = rocket_data.linearAccelData.acceleration.y;
  rocket_data.acceleration_z = rocket_data.linearAccelData.acceleration.z;
  rocket_data.lf_acceleration_x = rocket_data.linearAccelData.acceleration.x;
  rocket_data.lf_acceleration_y = rocket_data.linearAccelData.acceleration.y;
  rocket_data.lf_acceleration_z = rocket_data.linearAccelData.acceleration.z;
}

void GetBNOGyroData(RocketTracking& rocket_data)
{
  rocket_data.bno.getEvent(&rocket_data.gyroData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  rocket_data.gyro_x = rocket_data.gyroData.gyro.x;
  rocket_data.gyro_y = rocket_data.gyroData.gyro.y;
  rocket_data.gyro_z = rocket_data.gyroData.gyro.z;
  rocket_data.lf_gyro_x = rocket_data.gyroData.gyro.x;
  rocket_data.lf_gyro_y = rocket_data.gyroData.gyro.y;
  rocket_data.lf_gyro_z = rocket_data.gyroData.gyro.z;
}

void GetBNOOrientationData(RocketTracking& rocket_data)
{
  rocket_data.bno.getEvent(&rocket_data.orientationData, Adafruit_BNO055::VECTOR_EULER);
  rocket_data.orientation_x = rocket_data.orientationData.orientation.x;
  rocket_data.orientation_y = rocket_data.orientationData.orientation.y;
  rocket_data.orientation_z = rocket_data.orientationData.orientation.z;
  rocket_data.lf_orientation_x = rocket_data.orientationData.orientation.x;
  rocket_data.lf_orientation_y = rocket_data.orientationData.orientation.y;
  rocket_data.lf_orientation_z = rocket_data.orientationData.orientation.z;
}

void GetBNOTemp(RocketTracking& rocket)
{
  rocket.temp_celsius = rocket.bno.getTemp();
  rocket.lf_temp_celsius = rocket.bno.getTemp();
}

//Get Acceleration, Gyro and Orientation data of the rocket
void RocketTracking::GetBNOData(RocketTracking& rocket)
{
  //get rocket's accel data
  GetBNOAccelData(rocket);

  //get rocket's gyro data
  GetBNOGyroData(rocket);

  //get rocket's orientatio data
  GetBNOOrientationData(rocket);

  delay(50);
}

void RocketTracking::GetMPLData(RocketTracking& rocket)
{
  rocket.previous_altitude = rocket.altitude;
  rocket.altitude = rocket.baro.getAltitude() - rocket.altitude_offset ;  
  altitude_diff = rocket.altitude - rocket.initial_altitude;
}
void RocketTracking::SendBNOData(RocketTracking& rocket)
{
  //sprintf(rocket.data_buffer, "\tDado /enviado-> Enviado Orientation X %i Orientation Y %i Orientation Z %i\tAccel X %i Accel Y %i Accel Z %i",
  // orientation_x, orientation_y, orientation_z,
  //acceleration_x, acceleration_y, acceleration_z);

  LoRa.beginPacket();
  LoRa.println(rocket.orientation_x);
  LoRa.println(rocket.orientation_y);
  LoRa.println(rocket.orientation_z);
  LoRa.println(rocket.acceleration_x);
  LoRa.println(rocket.acceleration_y);
  LoRa.println(rocket.acceleration_z);
  LoRa.println(rocket.temp_celsius);
  LoRa.endPacket();
  LOGN("");
  LOGN("Dado LoRa Enviado");
  digitalWrite(LED_DEBUG, HIGH);
  delay(500);
  digitalWrite(LED_DEBUG, LOW);
}


void RocketTracking::SaveData(RocketTracking& rocket)
{

  LOGN("SD");
  if (!SD.begin(SD_PIN))
  {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");

  File myFile = SD.open("lasc7.txt", FILE_WRITE);


  // if the file opened okay, write to it:
  if (myFile) {
    Serial.println("Writing to test.txt...");

    myFile.print("LASC 2023 - 27/08/2023\n");
    
    myFile.print("Orientation_x: "); myFile.print(rocket.lf_orientation_x);
    myFile.print("\tOrientation_y: "); myFile.print(rocket.lf_orientation_y);
    myFile.print("\tOrientation_z: "); myFile.println(rocket.lf_orientation_z);

    myFile.print("Acceleration_x: "); myFile.print(rocket.lf_acceleration_x);
    myFile.print("\tAcceleration_y: "); myFile.print(rocket.lf_acceleration_y);
    myFile.print("\tAcceleration_z: "); myFile.println(rocket.lf_acceleration_z);

    myFile.print("Gyro_x: "); myFile.print(rocket.lf_gyro_x);
    myFile.print("\tGyro_y: "); myFile.print(rocket.lf_gyro_y);
    myFile.print("\tGyro_z: "); myFile.println(rocket.lf_gyro_z);

    myFile.print("Altitude :"); myFile.print(rocket.altitude_diff);
    myFile.print("\tMaxima Altitude :"); myFile.println(rocket.max_altitude);
    // close the file:
    myFile.close();
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
  /*
    // re-open the file for reading:
    myFile = SD.open("test2.txt");
    if (myFile)
    {
    Serial.println("AsaBrancaAerospace.txt:");

    // read from the file until there's nothing else in it:
    while (myFile.available()) {
      Serial.write(myFile.read());
    }
    // close the file:
    myFile.close();
    }
    else {
    // if the file didn't open, print an error:class RocketTracking : public Parameter

    Serial.println("error opening test.txt");
    }
  */
}

void RocketTracking::SendApogeeData(RocketTracking& rocket)
{
  LoRa.beginPacket();
  LoRa.print("reached apogee");
  LoRa.print("reached apogee");
  LoRa.print("reached apogee");
  LoRa.endPacket();
}
