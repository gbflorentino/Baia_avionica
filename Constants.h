

#pragma once

#include <Adafruit_MPL3115A2.h>
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>
#include <LoRa.h>
#include <SPI.h>                                                                                                           
#include <SD.h>
#include <string.h>
//#include <Tone.h> 
 
#define BAUD_RATE 115200
#define BNO055_SAMPLERATE_DELAY_MS (100)
#define BANDWIDTH 10.4E3
#define LOG(x) Serial.print(x)
#define LOGN(x) Serial.println(x)
#define SERVO_1 32
#define SERVO_2 33
#define DATA_SIZE 100
#define SD_PIN 53
#define NSS 53
#define NRESET 9
#define DI0 3
#define DT  200
#define DT_SD 200
#define MAIN_PIN 6
#define MAIN_DEPLOY 600
#define APOGEEq 2
#define APOGEE_PIN 7  
#define BUZZER_PIN 4
#define FLOAT_H -5

#define LED_DEBUG 31
