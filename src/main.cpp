#include <ESP8266WiFi.h>
#include <SparkFunMPU9250-DMP-ESP.h>
#include <Adafruit_BME280.h>

//D3  SDA GREEN
//D2  SCL BLUE

#define SDA_PIN             D3
#define SCL_PIN             D2  

#define BME_ID              0x76
#define IMU_ID              0x68

#define SEALEVELPRESSURE    (1013.25)

float accX = 0;
float accY = 0;
float accZ = 0;
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

  delay(100);

  //Configure the I2C pins
  Wire.pins (SDA_PIN, SCL_PIN);

  delay(100);

  //Start communication with the BME sensor
  bme.begin(BME_ID);
  
  delay(100);

  if (imu.begin() != INV_SUCCESS)
  {
    while (1)
    {
      Serial.println("Unable to communicate with MPU-9250");
      Serial.println("Check connections, and try again.");
      Serial.println();
      delay(5000);
    }
  }

  delay(100);

  imu.setSensors(INV_XYZ_ACCEL); 
  imu.setAccelFSR(2); 

  imu.dmpBegin(DMP_FEATURE_SEND_RAW_ACCEL ,
              10);                  
}

void printBMEData() {
  Serial.println("Temperature:" +String(bme.readTemperature()));
  Serial.println("Humidity:"+String(bme.readHumidity()));
  Serial.println("Pressure:"+String(bme.readPressure()/ 100.0F));
  Serial.println("Altitude:"+String(bme.readAltitude(SEALEVELPRESSURE)));
  Serial.println();
}

void printIMUData() {

  accX = imu.calcAccel(imu.ax);
  accY = imu.calcAccel(imu.ay);
  accZ = imu.calcAccel(imu.az);
  
  Serial.println("Acc: " + String(accX) + ", " +
              String(accY) + ", " + String(accZ) + " ");
  Serial.println("Time: " + String(imu.time) + " ms");
  Serial.println();
}

void loop() {

  if ( imu.fifoAvailable() )
  {
    if ( imu.dmpUpdateFifo() == INV_SUCCESS)
    {
      printIMUData();
    }
  }
  
  delay(1000);

}