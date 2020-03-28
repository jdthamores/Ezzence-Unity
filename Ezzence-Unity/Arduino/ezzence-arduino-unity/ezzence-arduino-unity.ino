/*********************************************************************
nRF51822 based Bluefruit LE modules
Judith's notes:
Remember to turn on location of the phone.
This code needs to download the following libraries:
Arduino Genuino Zero (Native USB Port)

// Code for sleep study. If toggle is on it will trigger scent after 20 minutes. 
This is different from other versions as the toggle now is needed for the sleep part, not for triggering int/frequency!

*********************************************************************/
#include <Wire.h>
#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
  #include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

/*==============================º===========================================
    APPLICATION SETTINGS

    FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
   
                              Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                              running this at least once is a good idea.
   
                              When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                              Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
       
                              Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    MODE_LED_BEHAVIOUR        LED activity, valid options are
                              "DISABLE" or "MODE" or "BLEUART" or
                              "HWUART"  or "SPI"  or "MANUAL"
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE         0
    #define MINIMUM_FIRMWARE_VERSION    "0.6.6"
    #define MODE_LED_BEHAVIOUR          "DISABLE"
/*=========================================================================*/

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

//-------------- LEDs ---------------------
#define LED_BLUE    A3 
#define LED_RED     A4
#define BOOST_EN    3

//-------------- Piezos ---------------------
//define pins atomizers
//define ATOMIZER A0

#define PWM_PIEZO   6
#define PIEZO_1     11
#define PIEZO_2     0
#define PIEZO_3     1
#define PIEZO_4     9

const bool DEBUG_SERIAL_PRINT = false;

const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

int counterGlobal = 0;
int prev_time = -10;
int cur_time  = 0;
int durationBurst, timeDistanceBurst = 0;

float threshold;
uint8_t frequency = 0, freq = 0; //Default values 1-20-sec release?
uint8_t intensity = 0, intens = 0;
uint8_t triggerHR = 0, triggerBR = 0, switchGV = 0;

//-------------- Toggle Android UI ---------------------
int toggleSleep = 0;
bool HRTrigger, BRTrigger = false;

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
int parseint(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];
uint8_t len;
bool timerIntensityReset, timerFrequencyReset = true;
long   initTimeBurst = 0, initDifferenceBurst = 0, currentTimeBurst = 0, currentDifferenceBurst = 0, timeElapsedBurst = 0, timeElapsedBurstStop = 0;
int piezoFrequency = 108000; //New powerful piezos
bool tSleep = true, timerSleepReset = false;
int sleepOnsetMin = 20; //20 minutes for sleep onset
long initTimeSleep = 0, currentTimeSleep = 0, timeElapsedSleep = 0, sleepOnsetMillis = 0;

void setup(void)
{
  Serial.begin(115200);

  Serial.println(F("BioEssence App Controller"));
  Serial.println(F("-----------------------------------------"));
  
  //LED SETUP
  pinMode(LED_BLUE,OUTPUT);
  pinMode(LED_RED,OUTPUT);
  pinMode(BOOST_EN,OUTPUT);

  //PIEZO SETUP
  pinMode(PIEZO_1, OUTPUT);
  pinMode(PIEZO_2, OUTPUT);
  pinMode(PIEZO_3, OUTPUT);
  pinMode(PIEZO_4, OUTPUT);

  digitalWrite(BOOST_EN,LOW);
  digitalWrite(PIEZO_1, LOW);
  digitalWrite(PIEZO_2, LOW);
  digitalWrite(PIEZO_3, LOW);
  digitalWrite(PIEZO_4, LOW);
  //Serial.begin(0);

  //Setup Frequency of Piezos
  tone(PWM_PIEZO,piezoFrequency); 
 
  //wakeup IMU
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  //Check the bluetooth module and print info data
  printBluetoothInfo();
  ble.setMode(BLUEFRUIT_MODE_DATA);
  //digitalWrite(LED_BLUE, HIGH);
  
}

void loop(void)
{

  counterGlobal++;
  cur_time = millis();

  //IMU DATA
        
  Serial.println( "Time IMU: "+ cur_time-prev_time );
  if ((cur_time-prev_time)>15){

      readAndDisplayIMUdata();
      //checkBLEforData();
      //printBluetoothInfo();
      //ble.info();
      
      ble.flush();
      ble.print("[t:");
      ble.print(millis());
      ble.print(",X:");
      ble.print(AcX);
      ble.print(",Y:");
      ble.print(AcY);
      ble.print(",Z:");
      ble.print(AcZ);
      ble.print("]");
      
      ble.flush();
      prev_time = cur_time;
  }
  
  writeButton(); 
  getToggle(); 
  getSmartphoneSensors(); 
  tSleep = timerSleep();

  // If toggle for HR o BR is activated, use default int/freq.
  
  //--------------- Intensity & Frequency ----------------
 
  if ( switchGV == 1 ){
    
    if ( triggerHR == 1 || triggerBR == 1){
      /*if (intens == 0 && freq == 0){   
        intens = 10; //For 10ms 
        freq = 5000; //Every 5 seconds
      }*/
      intens = intensity;
      freq = frequency;
    }
  
    //If toggle HR & BR is not active, but the user wants to put some int/freq.
    
    if ( (triggerHR == 0 && triggerBR == 0)){
      /*if (intens == 0 && freq == 0){   
        intens = 10; //For 10ms 
        freq = 5000; //Every 5 seconds
      }*/
      intens = 0;
      freq = 0;
  
    }
    
  }else{

    if ( toggleSleep == 0 || ( toggleSleep == 1 && tSleep == true) ){
      if ( toggleSleep == 0 ) timerSleepReset = true;
      intens = intensity;
      freq = frequency;
    }

    // If toggle for sleep is on and the timer is still running, then do not trigger scent
    if ( toggleSleep == 1 && tSleep == false ){ // timerSleep is false when the timer hasn't finished.
      intens = 0;
      freq = 0;
    }

  }
 
  setValuesToPiezo(intens, freq);

  const uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT); //---comment this if we want to make GV work
  //Serial.print("Length: ");
  //Serial.println(len);
  if (len == 0) return; //---comment this if we want to make GV work

}

bool timerSleep(){
  
  if ( timerSleepReset ) initTimeSleep = millis(); //Initialize timer for sleep delay 
  
  currentTimeSleep = millis();

  timeElapsedSleep = (currentTimeSleep - initTimeSleep ); //Time that has passed in milliseconds

  sleepOnsetMillis = sleepOnsetMin*60000; 
  
  if ( ( timeElapsedSleep ) < sleepOnsetMillis ) {
    
    timerSleepReset = false;
    tSleep = false;
    
  }else {
    tSleep = true;
  }
  
  return tSleep;
}

void setValuesToPiezo( uint8_t intensity, uint8_t frequency){
   
   // Release Scent Burst - Piezo 1
   if (frequency!= 0 && intensity!= 0 && frequency > 0 && intensity > 0){ 
    
       if (timerIntensityReset) initTimeBurst = millis(); //Initialize duration of burst

       currentTimeBurst =  millis(); //Current time
              
       timeElapsedBurst = (currentTimeBurst - initTimeBurst );
       
       //If the time elapsed is less than the duration of the burst then keep spraying scent (intensity)
       if ( timeElapsedBurst < ( intensity * 10 ) ) { //if 1000 is 1 second, if 1 = 1 millisecond. 
          timerIntensityReset = false;
          
          Serial.println("TURN ON ");
          //Turn on atomizer
          digitalWrite(LED_BLUE, HIGH);
          digitalWrite(LED_RED, HIGH);
          digitalWrite(PIEZO_1, HIGH); 
          
       } else{ //That means that the time elapsed is more than the duration of the burst of scent.

          if (timerFrequencyReset) initDifferenceBurst = millis();  //Initialize time difference between burst
          currentDifferenceBurst = millis(); //Current time
          timeElapsedBurstStop = (currentDifferenceBurst - initDifferenceBurst );

          //If the time elapsed is less than the time difference between bursts then don't spray scent (frequency)
          if ( timeElapsedBurstStop < frequency * 1000) { 
            timerFrequencyReset = false;
            
            Serial.println("TURN OFF ");
            //Turn off atomizer
            digitalWrite(BOOST_EN,LOW);
            digitalWrite(LED_BLUE,LOW);
            digitalWrite(LED_RED, LOW);
            digitalWrite(PIEZO_1, LOW);
            
          }else{
             //Reset variables 
            //timeElapsedBurst = 0;
            timerIntensityReset = true;
            timerFrequencyReset = true;

          }
       }

   }else{
        //Turn off atomizer
        digitalWrite(BOOST_EN,LOW);
        digitalWrite(LED_BLUE,LOW);
        digitalWrite(LED_RED, LOW);
        digitalWrite(PIEZO_1, LOW);
   }

}
       
void printBluetoothInfo() {
  Serial.print(F("Initialising the Bluefruit LE module: "));
    if ( !ble.begin(VERBOSE_MODE) )
    {
      error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
    }
    Serial.println( F("OK!") );
  
    if ( 1 )
    {
      /* Perform a factory reset to make sure everything is in a known state */
      Serial.println(F("Performing a factory reset: "));
      if ( ! ble.factoryReset() ){
        error(F("Couldn't factory reset"));
      }
    }
  
    /* Disable command echo from Bluefruit */
    ble.echo(false);
  
    Serial.println("Requesting Bluefruit info:");
    /* Print Bluefruit information */
    ble.info();

    /* Change the device name to make it easier to find */
    Serial.println(F("Setting device name to 'BioEssence White Edition': "));
    if (! ble.sendCommandCheckOK(F( "AT+GAPDEVNAME=Ezzence" )) ) {
      error(F("Could not set device name?"));
    }
}

void readAndDisplayIMUdata(){ 
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
 // Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
 // GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
 // GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
 // GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  
  if (DEBUG_SERIAL_PRINT) { 
      Serial.print(" IMU Data Start:");
      Serial.print(millis());
      //Serial.print(",");
      //Serial.print(counterGlobal);
      Serial.print(",");
      Serial.print(AcX);
      Serial.print(","); 
      Serial.print(AcY);
      Serial.print(",");
      Serial.println(AcZ);
      Serial.print(" IMU Data End:");

  }
}

void writeButton ( ) {
  //Manual trigger
  // Buttons 
  if ( packetbuffer[1] == 'B' ) {
    
    uint8_t buttnum = packetbuffer[2] - '0';
    
    boolean pressed = packetbuffer[3] - '0';

    //The user is pressing the button to release scent. Turn on LEDs
    if ( pressed ) {
        
        digitalWrite(LED_BLUE, HIGH); 

        // Release Scent Burst - Piezo 1
        if ( buttnum == 5 ) {
          
            digitalWrite(PIEZO_1, HIGH);
            delay(10); // Release 10ms
            digitalWrite(PIEZO_1, LOW);
            
        }

        // Release Scent Burst - Piezo 2
        if ( buttnum == 6 ) {
          
            digitalWrite(PIEZO_2, HIGH);
            delay(10); // Release 10ms
            digitalWrite(PIEZO_2, LOW);
            
        }
        
        // Release Scent Burst - Piezo 3
        if ( buttnum == 7 ) {
          
            digitalWrite(PIEZO_3, HIGH); 
            delay(10); // Release 10ms
            digitalWrite(PIEZO_3, LOW);
            
        }

    }else{
      
        digitalWrite(LED_BLUE,LOW);
        digitalWrite(LED_RED, LOW);
        
        digitalWrite(BOOST_EN,LOW);
        
        digitalWrite(PIEZO_1, LOW);
        digitalWrite(PIEZO_2, LOW);
        digitalWrite(PIEZO_3, LOW);
      
    }

  }
  
}

void getToggle ( ) {
  
  if (packetbuffer[1] == 'C') {
        //C11300
        Serial.println("C");
        //On/Off toggle Intensity & Frequency
        toggleSleep = packetbuffer[2] - '0';
        intensity = packetbuffer[3] - '0';
    
        if ( packetbuffer[8] - '0' == 0 ) { 
          
          //Morae than 2 digits for frequency
          //!C#### -----> !C(#onoffintfreq)(#intensity)(#frequency)(#onoffHR)(#onoffBR)'/0' !C115800 !C11700
          //!C00010
        
          frequency = ( (packetbuffer[4] - '0')*10 + (packetbuffer[5] - '0'));
          triggerHR = packetbuffer[6] - '0';
          triggerBR = packetbuffer[7] - '0';
          switchGV = packetbuffer[8] - '0';
    
    }else{
      
      frequency = packetbuffer[4] - '0';
      triggerHR = packetbuffer[5] - '0';
      triggerBR = packetbuffer[6] - '0';
      switchGV = packetbuffer[7] - '0';
    }

    Serial.print("Debug: Sleep Toggle: ");
    Serial.println(toggleSleep); 
    Serial.print("Intensity: ");
    Serial.print(intensity); 
    Serial.print(", Frequency: ");
    Serial.println(frequency); 
    Serial.print("Heart Rate Trigger: ");
    Serial.print(triggerHR);
    Serial.print(", Respiration Trigger: ");
    Serial.println(triggerBR);  
    Serial.print(", GV Switch: ");
    Serial.println(switchGV); 
   
  }
  
}

void getSmartphoneSensors(){
  // GPS Location
  if (packetbuffer[1] == 'L') {
    float lat, lon, alt;
    lat = parsefloat(packetbuffer+2);
    lon = parsefloat(packetbuffer+6);
    alt = parsefloat(packetbuffer+10);
    
    Serial.print("GPS Location\t");
    Serial.print("Lat: "); Serial.print(lat, 4); // 4 digits of precision!
    Serial.print('\t');
    Serial.print("Lon: "); Serial.print(lon, 4); // 4 digits of precision!
    Serial.print('\t');
    Serial.print(alt, 4); Serial.println(" meters");
  }

  // Accelerometer
  if (packetbuffer[1] == 'A') {
    float x, y, z;
    x = parsefloat(packetbuffer+2);
    y = parsefloat(packetbuffer+6);
    z = parsefloat(packetbuffer+10);
    Serial.print("Accel\t");
    Serial.print(x);// Serial.print('\t');
    Serial.print(y); //Serial.print('\t');
    Serial.print(z); Serial.println();
  }

  // Magnetometer
  if (packetbuffer[1] == 'M') {
    float x, y, z;
    x = parsefloat(packetbuffer+2);
    y = parsefloat(packetbuffer+6);
    z = parsefloat(packetbuffer+10);
    Serial.print("Mag\t");
    Serial.print(x); Serial.print('\t');
    Serial.print(y); Serial.print('\t');
    Serial.print(z); Serial.println();
  }

  // Gyroscope
  if (packetbuffer[1] == 'G') {
    float x, y, z;
    x = parsefloat(packetbuffer+2);
    y = parsefloat(packetbuffer+6);
    z = parsefloat(packetbuffer+10);
    Serial.print("Gyro\t");
    Serial.print(x); Serial.print('\t');
    Serial.print(y); Serial.print('\t');
    Serial.print(z); Serial.println();
  }
 
  // Quaternions
  if (packetbuffer[1] == 'Q') {
    float x, y, z, w;
    x = parsefloat(packetbuffer+2);
    y = parsefloat(packetbuffer+6);
    z = parsefloat(packetbuffer+10);
    w = parsefloat(packetbuffer+14);
    Serial.print("Quat\t");
    Serial.print(x); Serial.print('\t');
    Serial.print(y); Serial.print('\t');
    Serial.print(z); Serial.print('\t');
    Serial.print(w); Serial.println();
  }
}
