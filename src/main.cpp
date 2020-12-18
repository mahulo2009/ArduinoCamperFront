//#include <Arduino.h>
#include <SparkFunMPU9250-DMP.h>

//#include <Wire.h>

#include <Adafruit_BME280.h>


//#include <ESP8266WiFi.h>

//D3  SDA GREEN
//D2  SCL BLUE

#define SDA_PIN             D3
#define SCL_PIN             D2  

#define BME_ID              0x76
#define IMU_ID              0x68

#define SEALEVELPRESSURE    (1013.25)

/**
 * BM3280 sensor: temperature, humidity, pressure and altitude
 */
Adafruit_BME280       bme;

/**
 * TODO
 */
MPU9250_DMP imu;

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
  delay(1000);

  imu.begin();

}

void printData() {
  Serial.println(bme.readTemperature());
  Serial.println(bme.readHumidity());
  Serial.println(bme.readPressure()/ 100.0F);
  Serial.println(bme.readAltitude(SEALEVELPRESSURE));
}

void loop() {

  printData();
  delay(1000);
}