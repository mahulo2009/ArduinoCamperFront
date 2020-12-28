#include <ESP8266WiFi.h>
#include <Adafruit_BME280.h>
#include "ADXL345.h"

//D3  SDA GREEN
//D2  SCL BLUE
#define SDA_PIN             D3
#define SCL_PIN             D2  

#define BME_ID              0x76

#define SEALEVELPRESSURE    (1013.25)

#define ACCEL_SCALE 1 / 256 // LSB/g
#define G_TO_ACCEL 9.81

/**
 * BM3280 sensor: temperature, humidity, pressure and altitude
 */
Adafruit_BME280       bme;

/**
 * Acceleromter sensor: acceleration x, y and z.
 */
ADXL345 accelerometer;

ADXL345 adxl = ADXL345();

void setup() {

  //Configure Serial port for debugging proposses
  Serial.begin(115200);
  Serial.setTimeout(2000);
  //Wait for the Serial port to be ready
  while(!Serial) {}

  //Configure the I2C pins
  Wire.pins (SDA_PIN, SCL_PIN);

  //Start communication with the BME sensor
  bme.begin(BME_ID);

  //Initialize the accelerometer
  accelerometer.initialize();
  while( !  accelerometer.testConnection()  ) 
  {
      Serial.println("Accelerometer NOT Connected!");
      Serial.println();
      delay(5000);

  }

}

void printBMEData() {

  Serial.println("Temperature:" +String(bme.readTemperature()));
  Serial.println("Humidity:"+String(bme.readHumidity()));
  Serial.println("Pressure:"+String(bme.readPressure()/ 100.0F));
  Serial.println("Altitude:"+String(bme.readAltitude(SEALEVELPRESSURE)));

  Serial.println();

}

void printIMUData() {

  int16_t ax, ay, az;
  
  accelerometer.getAcceleration(&ax, &ay, &az);

  float accX = ax * (double) ACCEL_SCALE * G_TO_ACCEL;
  float accY = ay * (double) ACCEL_SCALE * G_TO_ACCEL;
  float accZ = az * (double) ACCEL_SCALE * G_TO_ACCEL;
  
  Serial.println("Acc: " + 
                  String(accX) + ", " +
                  String(accY) + ", " + 
                  String(accZ) + " ");

  Serial.println();

}

void loop() {
  
  //Print IMU data.
  printIMUData();

  //Print BME data.
  printBMEData();

  delay(2500);

}