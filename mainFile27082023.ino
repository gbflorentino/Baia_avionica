
/*
  Author: Everton Santos
  Github: esantos_git (?)
  Author: Gabriel <>

  LASC AVIONIC'S SYSTEM

*/

#include "Constants.h"
#include "RocketTracking.h"
#include "ExternalFunctions.h"

static RocketTracking rocket;
static ServoMotor rocket_sm;
//sensors_event_t event;

RocketStates rocket_states;
//Tone rocket_tone;

unsigned short flag_h, flag_i, flag_a, flag_m, flag_g = 0;
static unsigned long tf, ti, t_sd, t_send = 0;
int ttp = 400;
const int buzzer1 = 7;

//Auxiliar variables, just for debug

void GpsModule();

//AUXILIAR FUNCTIONS, JUST FOR DEBUG

void continous()
{
   while(1)
  {
    tf = millis();
    flag_i = 1;
    LOGN("STATE :: RUNNING");
    rocket.GetBNOData(rocket);
    rocket.GetMPLData(rocket);
    delay(50);
/*
    if (tf - t_send > 1000)
    {
      t_send = tf;
      //rocket.SendBNOData(rocket);
      rocket.SaveData(rocket);
    }

    
    
*/      
    if(rocket.max_altitude < rocket.altitude)
      rocket.max_altitude = rocket.altitude;
       
    PrintRocketClass(rocket);

    /*
    LOGN("\t::: Pin 7 :");
    LOGN(digitalRead(APOGEE_PIN));
    LOGN("\t::: Pin  6:");
    LOGN(digitalRead(MAIN_PIN));
    */
    if (tf - t_sd > DT_SD)
    {
      t_sd = tf;
      rocket.SaveData(rocket);
    }
/*
    if (digitalRead(APOGEE_PIN) == 1 && flag_i == 1 && flag_a == 0)
    {
      LOGN("RUNNING => APOGEE");
      rocket_states = APOGEE;
    }

    
    if (digitalRead(MAIN_PIN) == 1 && flag_i == 1 && flag_a == 1 && flag_g == 0)
    {
      LOGN("RUNNING => MAIN");
      rocket_states = MAIN;
    }
    */

   // float h = rocket.initial_altitude - rocket.altitude;
    float test = rocket.altitude - rocket.previous_altitude;

    if (rocket.altitude - rocket.max_altitude < FLOAT_H && flag_i == 1 && flag_a == 0)
    {
      //rocket_states = APOGEE1;
      LOGN("RUNNING => APOGEE");
      flag_a = 1;
      tone(buzzer1, 349, ttp*4);
    }

    
    if (rocket.altitude < MAIN && flag_i == 1 && flag_a == 1 && flag_g == 0)
    {
      LOGN("RUNNING => MAIN");
      flag_m = 1;
      rocket_sm.ParachuteSignal(rocket_sm);
      LOGN("Servo Acionado");
      //rocket_states = MAIN;
    }
    
    while (rocket.lf_acceleration_z <= 1 && flag_m == 1)
    {
      //rocket_states = GROUND;
    LOGN("STATE :: GROUND");
    GpsModule();
    buzzerAsaBranca();
    delay(5000);
      LOGN("RUNNING => GROUND");
    }
  }
            
   
    
  
}

void States()
{
  rocket_states = HIBERNATING; //INITIAL STATE
  while (1)
  {
    tf = millis();
    if (tf - ti > DT)
    {
      ti = tf;

      switch (rocket_states)
      {
        case HIBERNATING:
          flag_h = 1;
          LOGN("STATE :: HIBERNATING");
          rocket.GetBNOData(rocket);
          rocket.GetMPLData(rocket);
          if (tf - t_sd > DT_SD)
          {
            t_sd = tf;
            //rocket.SendBNOData(rocket);
            rocket.SaveData(rocket);

          }
          if (rocket.lf_acceleration_z > 1 && flag_h == 1)
          {
            LOGN("HIBERNATING =>  RUNNING");
              continous();
          }
          break;
        case RUNNING:

        //  if(!flag_i)
          //
          rocket_sm.ParachuteSignal(rocket_sm);
            
          flag_i = 1;
          LOGN("STATE :: RUNNING");
          rocket.GetBNOData(rocket);
          rocket.GetMPLData(rocket);
          delay(50);
/*
          if (tf - t_send > 1000)
          {
            t_send = tf;
            //rocket.SendBNOData(rocket);
            rocket.SaveData(rocket);
          }

          
          
  */      
          if(rocket.max_altitude < rocket.altitude)
            rocket.max_altitude = rocket.altitude;
             
          PrintRocketClass(rocket);

          /*
          LOGN("\t::: Pin 7 :");
          LOGN(digitalRead(APOGEE_PIN));
          LOGN("\t::: Pin  6:");
          LOGN(digitalRead(MAIN_PIN));
          */
          if (tf - t_sd > DT_SD)
          {
            t_sd = tf;
            rocket.SaveData(rocket);
          }
      /*
          if (digitalRead(APOGEE_PIN) == 1 && flag_i == 1 && flag_a == 0)
          {
            LOGN("RUNNING => APOGEE");
            rocket_states = APOGEE;
          }

          
          if (digitalRead(MAIN_PIN) == 1 && flag_i == 1 && flag_a == 1 && flag_g == 0)
          {
            LOGN("RUNNING => MAIN");
            rocket_states = MAIN;
          }
          */

         // float h = rocket.initial_altitude - rocket.altitude;
          float test = rocket.altitude - rocket.previous_altitude;

          if (rocket.altitude - rocket.max_altitude < FLOAT_H && flag_i == 1 && flag_a == 0)
          {
            //rocket_states = APOGEE1;
            LOGN("RUNNING => APOGEE");
            flag_a = 1;
            //tone(buzzer1, 349, ttp*4);
          }

          
          if (rocket.altitude < MAIN && flag_i == 1 && flag_a == 1 && flag_g == 0)
          {
            LOGN("RUNNING => MAIN");
            flag_m = 1;
            tone(buzzer1, 440, ttp*4);
            delay(500);
            tone(buzzer1, 440, ttp*4);
            rocket_sm.ParachuteSignal(rocket_sm);
            LOGN("Servo Acionado");
            //rocket_states = MAIN;
          }
          
          if (rocket.lf_acceleration_z <= 1 & flag_m == 1)
          {
            //rocket_states = GROUND;
          LOGN("STATE :: GROUND");
          GpsModule();
          buzzerAsaBranca();
          delay(5000);
            LOGN("RUNNING => GROUND");
          }
          
          break;
        case APOGEE1:
          flag_a = 1;
          LOGN("STATE :: APOGEE");

          //rocket.SendApogeeData(rocket);

          LOGN("APOGEE => RUNNING");
         // rocket_states = RUNNING;


          break;
        case MAIN:
          flag_m = 1;
          LOGN("STATE :: MAIN");

          //rocket_sm.ParachuteSignal(rocket_sm);

          LOGN("PARACHUTE SIGNAL SEND");
          LOGN("MAIN => RUNNING");
          //rocket_states = RUNNING;

          break;
        case GROUND:
          LOGN("STATE :: GROUND");
          GpsModule();
          buzzerAsaBranca();
          delay(5000);
          break;
        default:
          LOG("ERROR");
          break;
      }
    }
    delay(50);
  }
}


void PrintRocketClass(RocketTracking& rocket_debug)
{
  LOG("Accel X: ");
  LOG(rocket_debug.lf_acceleration_x);
  LOG("\tAccel Y: ");
  LOG(rocket_debug.lf_acceleration_y);
  LOG("\tAccel Z: ");
  LOG(rocket_debug.lf_acceleration_z);
  LOG("\n Gyro X: ");
  LOG(rocket_debug.lf_gyro_x);
  LOG("\tGyro Y: ");
  LOG(rocket_debug.lf_gyro_y);
  LOG("\tGyro Z: ");
  LOG(rocket_debug.lf_gyro_z);
  LOG("\nOrientation X: ");
  LOG(rocket_debug.lf_orientation_x);
  LOG("\tOrientationY: ");
  LOG(rocket_debug.lf_orientation_y);
  LOG("\tOrientation Z: ");
  LOG(rocket_debug.lf_orientation_z);
  LOGN("");
  LOG("Init Altitude :");
  LOG(rocket.initial_altitude);
  LOG("\tAltitude :");
  LOG(rocket.altitude);
  LOG("\tAltitude Diff :");
  LOG(rocket.altitude_diff);
  LOG("\tAtual menos anterior :");
  LOG(rocket.altitude - rocket.previous_altitude);
  LOGN("");
}

void setup()
{

  Serial.begin(BAUD_RATE);

  pinMode(MAIN_PIN, INPUT);
  pinMode(APOGEE_PIN, INPUT);
  pinMode(SD_PIN, OUTPUT);
  pinMode(LED_DEBUG, INPUT);

  digitalWrite(LED_DEBUG, HIGH);

  digitalWrite(SD_PIN, HIGH);
  LOG("SETUP STARTED\n");
  while (!Serial) delay (10);
  if (rocket.Init(rocket))
  {
    buzzerAsaBranca();

    LOGN("BUZZER");
  }

  rocket_sm.Init(rocket_sm);
}

void loop()
{
  States();
  //rocket.GetBNOData(rocket);
  //delay(100);
  //rocket.SaveData(rocket);
  //delay(100);
  //rocket.SendBNOData(rocket);
  // delay(100);
  //PrintRocketClass(rocket);
  //sm.ParachuteSignal(sm);
  //sm.SetMotor(sm.main_servo, 0);
  // LOG("THE MAIN SERVO WAS SUCCEFULLY S");
  //sm.SetMotor(sm.redundant_servo, 90);
  //LOG("THE REDUNDANT SERVO WAS SUCCEFULLY S");
  //delay(3000);
  //sm.SetMotor(sm.main_servo, 90);
  //LOG("MAIN SERVO SPEED = ZERO");

  //delay(1000);


}
