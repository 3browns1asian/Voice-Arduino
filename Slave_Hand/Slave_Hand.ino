//  Pins
//  BT VCC to Arduino 5V out. 
//  BT GND to GND
//  BT RX to Arduino pin 2 (through a voltage divider)
//  BT TX to Arduino pin 3 (no need voltage divider)
 
 
#include <SoftwareSerial.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>

SoftwareSerial BTserial(3, 4); // RX | TX
// Connect the HC-05 TX to Arduino pin 2 RX. 
// Connect the HC-05 RX to Arduino pin 3 TX through a voltage divider.



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
int DELAY = 0;
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
void setup() 
{
//  Serial.begin(9600);
  BTserial.begin(9600); 
  delay(1000);
//
  /* Initialise the sensor */
  if(!lsm.begin())
  {
    /* There was a problem detecting the LSM9DS0 ... check your connections */
    BTserial.println("Ooops, no LSM9DS0 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  //Serial.println("Setup");
//  /* Setup the sensor gain and integration time */
  configureSensor();
  pinMode(LED_PIN, OUTPUT);

}
 
void loop()
{ 
    /* Get a new sensor event */ 
    sensors_event_t accel, mag, gyro, temp;
    lsm.getEvent(&accel, &mag, &gyro, &temp); 
    
    
    if (transmissionLength > SIGN_LENGTH){
      digitalWrite(LED_PIN, LOW);    // turn the LED off by making the voltage LOW
      BTserial.write("END\r\n");
//      Serial.write("END\r\n");
      delay(WAIT_TIME);              
      transmissionLength = 0;
      startTime = millis();
    }
    digitalWrite(LED_PIN, HIGH);   // turn the LED on (HIGH is the voltage level)
      //String output = "0,0,0,0,0,0,";
    String output = "";
    output.concat(accel.acceleration.x);
    output.concat(",");
    output.concat(accel.acceleration.y);
    output.concat(",");
    output.concat(accel.acceleration.z);
    output.concat(",");

    output.concat(gyro.gyro.x * 0.00875);
    output.concat(",");
    output.concat(gyro.gyro.y * 0.00875);
    output.concat(",");
    output.concat(gyro.gyro.z * 0.00875);
    output.concat(",");
    
    for (int i = 0; i < sizeof(SENSOR_PINS)/sizeof(int); i = i + 1) {
      int sensorPin = SENSOR_PINS[i]; 
      int sensorValue = analogRead(sensorPin);
      
      output.concat(sensorValue);
      if(i <  sizeof(SENSOR_PINS)/sizeof(int) - 1)
        output.concat(",");
    }
    output.concat("|0,0,0,0,0,0,0,0,0,0,0\r\n");
//    output.concat("\r\n");

    int outputLength = output.length() + 1; 
    char buff[outputLength];

    output.toCharArray(buff, outputLength);
    BTserial.write(buff);
//    Serial.write(buff);
    //Serial.println("test");
    transmissionLength = millis() - startTime;
    delay(DELAY);

 
}
