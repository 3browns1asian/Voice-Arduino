#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>


/* Assign a unique base ID for this sensor */   
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);  // Use I2C, ID #1000


/* Or, use Hardware SPI:
  SCK -> SPI CLK
  SDA -> SPI MOSI
  G_SDO + XM_SDO -> tied together to SPI MISO
  then select any two pins for the two CS lines:
*/

#define LSM9DS0_XM_CS 10
#define LSM9DS0_GYRO_CS 9
//Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(LSM9DS0_XM_CS, LSM9DS0_GYRO_CS, 1000);
int SENSOR_PINS[] = {A0,A1,A2,A3,A6};
int LED_PIN = 13;
unsigned long transmissionLength = 0;
int SIGN_LENGTH = 2000; //Sign Length in ms (active transmission)
int WAIT_TIME = 2000; //time between two signs in ms (no transmission)
int DELAY = 50;
int count = 0;
unsigned long startTime = 0;
/* Or, use Software SPI:
  G_SDO + XM_SDO -> tied together to the MISO pin!
  then select any pins for the SPI lines, and the two CS pins above
*/

#define LSM9DS0_SCLK 13
#define LSM9DS0_MISO 12
#define LSM9DS0_MOSI 11

//Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(LSM9DS0_SCLK, LSM9DS0_MISO, LSM9DS0_MOSI, LSM9DS0_XM_CS, LSM9DS0_GYRO_CS, 1000);

/**************************************************************************/
/*
    Configures the gain and integration time for the TSL2561
*/
/**************************************************************************/
void configureSensor(void)
{
  // 1.) Set the accelerometer range
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void) 
{
#ifndef ESP8266
  while (!Serial);     // will pause Zero, Leonardo, etc until serial console opens
#endif
  Serial.begin(9600);
  delay(1000);
  Serial.println(F("LSM9DS0 9DOF Sensor Test")); Serial.println("");
  
  /* Initialise the sensor */
  if(!lsm.begin())
  {
    /* There was a problem detecting the LSM9DS0 ... check your connections */
    Serial.print(F("Ooops, no LSM9DS0 detected ... Check your wiring or I2C ADDR!"));
    while(1);
  }
  Serial.println(F("Found LSM9DS0 9DOF"));
  
  /* Display some basic information on this sensor */
  //displaySensorDetails();
  
  /* Setup the sensor gain and integration time */
  configureSensor();
  pinMode(LED_PIN, OUTPUT);
  /* We're ready to go! */
  Serial.println("");
  startTime = millis();
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void) 
{  
  //count++;
  /* Get a new sensor event */ 
  sensors_event_t accel, mag, gyro, temp;

  lsm.getEvent(&accel, &mag, &gyro, &temp); 
  
  digitalWrite(LED_PIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  if (transmissionLength > SIGN_LENGTH){
    digitalWrite(LED_PIN, LOW);    // turn the LED off by making the voltage LOW
    Serial.println("END");
    delay(WAIT_TIME);              
    transmissionLength = 0;
    startTime = millis();
    count++;
  }
  
  if(count >= 21){
    while(1){ 
    }
  }
    else{
      Serial.print(accel.acceleration.x); Serial.print(",");
      Serial.print(accel.acceleration.y);Serial.print(",");
      Serial.print(accel.acceleration.z);Serial.print(",");
    
      Serial.print(gyro.gyro.x * 0.00875);Serial.print(",");
      Serial.print(gyro.gyro.y * 0.00875);Serial.print(",");
      Serial.print(gyro.gyro.z * 0.00875);Serial.print(",");
    
    
      for (int i = 0; i < sizeof(SENSOR_PINS)/sizeof(int); i = i + 1) {
        int sensorPin = SENSOR_PINS[i];
        int sensorValue = analogRead(sensorPin);
        Serial.print(sensorValue);
        if(i <  sizeof(SENSOR_PINS)/sizeof(int) - 1)
          Serial.print(",");
      }
      Serial.print("\r\n");
      transmissionLength = millis() - startTime;
      delay(DELAY);
   }
}

