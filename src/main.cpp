#include <ESP8266WiFi.h>
#include <Adafruit_BME280.h>
#include "ADXL345.h"
#include <LiquidCrystal_I2C.h>

//D3  SDA GREEN
//D2  SCL BLUE
#define BME_ID 0x76
#define BME_SEA_LEVEL_PRESSURE (1013.25)

#define IMU_ACCEL_SCALE 1 / 256 // LSB/g
#define IMU_G_TO_ACCEL 9.81

#define TIP_TILT_DIPLAY_DIMY 8
#define TIP_TILT_DIPLAY_SIZEBOX_MIDDLE 2
  
/**
 * BM3280 sensor: temperature, humidity, pressure and altitude
 */
Adafruit_BME280 bme;

/**
 * Acceleromter sensor: acceleration x, y and z.
 */
ADXL345 accelerometer;

/**
 * Display: 16 columns x 2 rows
 * 
 *  Parameters:
 *    - @param[in] I2C id.
 *    - @param[in] Number of columns
 *    - @param[in] Number of rows
 */
LiquidCrystal_I2C lcd(0x27, 16, 2);

float imu_accX_;
float imu_accY_;
float imu_accZ_;

int tip_;
int tilt_;

void imu_init();
void imu_read();
void imu_normalize();
void imu_print();

float bme_temperature_;
float bme_humidity_;
float bme_pressure_;
float bme_altitude_;

void bme_init();
void bme_read();
void bme_print();

union A
{
  byte a[2];
  int c;
} row;

void displayLCD();

void setup()
{

  //Configure Serial port for debugging proposses
  Serial.begin(115200);
  Serial.setTimeout(2000);
  //Wait for the Serial port to be ready
  while (!Serial)
  {
  }

  //Start communication with the BME sensor
  bme_init();

  //Initialize the imu
  imu_init();

  lcd.init();
  lcd.backlight();
  lcd.display();
}

void loop()
{

  //Print IMU data.
  imu_read();
  imu_print();
  imu_normalize();

  //Print BME data.
  bme_read();
  bme_print();

  displayLCD();

  delay(2000);
}

void imu_init()
{
  //Initialize the accelerometer
  accelerometer.initialize();
  while (!accelerometer.testConnection())
  {
    Serial.println("Accelerometer NOT Connected!");
    Serial.println();
    delay(5000);
  }
}

void imu_read()
{
  int16_t ax, ay, az;

  accelerometer.getAcceleration(&ax, &ay, &az);

  imu_accX_ = ax * (double)IMU_ACCEL_SCALE * IMU_G_TO_ACCEL;
  imu_accY_ = ay * (double)IMU_ACCEL_SCALE * IMU_G_TO_ACCEL;
  imu_accZ_ = az * (double)IMU_ACCEL_SCALE * IMU_G_TO_ACCEL;
}
void imu_print()
{
  Serial.println("Acc: " +
                 String(imu_accX_) + ", " +
                 String(imu_accY_) + ", " +
                 String(imu_accZ_) + " ");

  Serial.println();
}

void bme_init()
{
  bme.begin(BME_ID);
}

void bme_read()
{
  bme_temperature_ = bme.readTemperature();
  bme_humidity_ = bme.readHumidity();
  bme_pressure_ = bme.readPressure() / 100.0F;
  bme_altitude_ = bme.readAltitude(BME_SEA_LEVEL_PRESSURE);
}

void bme_print()
{
  Serial.println("Temperature:" + String(bme_temperature_));
  Serial.println("Humidity:" + String(bme_humidity_));
  Serial.println("Pressure:" + String(bme_pressure_));
  Serial.println("Altitude:" + String(bme_altitude_));

  Serial.println();
}

void displayLCD()
{
  lcd.clear();

  lcd.setCursor(1, 0);
  lcd.print((int)bme_temperature_);
  lcd.setCursor(3, 0);
  lcd.print("C\xDF");

  lcd.setCursor(6, 0);
  lcd.print((int)bme_pressure_);
  lcd.setCursor(10, 0);
  lcd.print("mb");

  lcd.setCursor(1, 1);
  lcd.print((int)bme_humidity_);
  lcd.setCursor(3, 1);
  lcd.print("%");

  lcd.setCursor(6, 1);
  lcd.print((int)bme_altitude_);
  lcd.setCursor(10, 1);
  lcd.print("m");

  lcd.setCursor(13, 0);
  lcd.write(0);
  lcd.setCursor(14, 0);
  lcd.write(1);
  lcd.setCursor(13, 1);
  lcd.write(2);
  lcd.setCursor(14, 1);
  lcd.write(3);
}

void imu_normalize()
{
  tip_ = round(imu_accX_ / 0.2);
  tilt_ = round(imu_accY_ / 0.2);

  if (tilt_ >  3) tilt_ = 3;
  if (tilt_ < -3) tilt_ = -3;
  if (tip_  > 5)   tip_  = 5;
  if (tip_  < -5)  tip_ = -5;

  Serial.println("Tip/Tilt: " +
                 String(tip_) + ", " +
                 String(tilt_));

  Serial.println();

  switch(abs(tilt_)) 
  {
    case 0:
      row.a[1] = B00011;
      row.a[0] = B11000;      
      break;
    case 1:
      if (tilt_ < 0)
      {
        row.a[1] = B00001;
        row.a[0] = B11100;        
      }
      else
      {
        row.a[1] = B00111;        
        row.a[0] = B10000;        
      }      
      break;
    case 2:
      if (tilt_ < 0)
      {
        row.a[1] = B00000;
        row.a[0] = B11110;
        
      }
      else
      {
        row.a[1] = B01111;
        row.a[0] = B00000;        
      }      
      break;    
    case 3:
      if (tilt_ < 0)
      {
        row.a[1] = B00000;
        row.a[0] = B01111;        
      }
      else
      {
        row.a[1] = B11110;
        row.a[0] = B00000;        
      }      
      break;    
  }

  byte box_upper_left[8] =
      {
          B00000,
          B00000,
          B00000,
          B00000,
          B00000,
          B00000,
          B00000,
          B00000};

  for (int i=(TIP_TILT_DIPLAY_DIMY-TIP_TILT_DIPLAY_SIZEBOX_MIDDLE)-tip_;
        i<min((TIP_TILT_DIPLAY_DIMY-TIP_TILT_DIPLAY_SIZEBOX_MIDDLE)+TIP_TILT_DIPLAY_SIZEBOX_MIDDLE*2-tip_,TIP_TILT_DIPLAY_DIMY);
        i++) 
  {    
    box_upper_left[i]=row.a[1];
  }

  byte box_upper_right[8] =
      {
          B00000,
          B00000,
          B00000,
          B00000,
          B00000,
          B00000,
          B00000,
          B00000};

  for (int i=(TIP_TILT_DIPLAY_DIMY-TIP_TILT_DIPLAY_SIZEBOX_MIDDLE)-tip_;
        i<min((TIP_TILT_DIPLAY_DIMY-TIP_TILT_DIPLAY_SIZEBOX_MIDDLE)+TIP_TILT_DIPLAY_SIZEBOX_MIDDLE*2-tip_,TIP_TILT_DIPLAY_DIMY);
        i++) 
  {    
    box_upper_right[i]=row.a[0];
  }

  byte box_bottom_left[8] =
      {
          B00000,
          B00000,
          B00000,
          B00000,
          B00000,
          B00000,
          B00000,
          B00000};

  for (int i=max(-TIP_TILT_DIPLAY_SIZEBOX_MIDDLE-tip_,0);
        i<TIP_TILT_DIPLAY_SIZEBOX_MIDDLE-tip_;
        i++) 
  {    
    box_bottom_left[i]=row.a[1];
  }

  byte box_bottom_right[8] =
      {
          B00000,
          B00000,
          B00000,
          B00000,
          B00000,
          B00000,
          B00000,
          B00000};

  for (int i=max(-TIP_TILT_DIPLAY_SIZEBOX_MIDDLE-tip_,0);
        i<TIP_TILT_DIPLAY_SIZEBOX_MIDDLE-tip_;
        i++) 
  {    
    box_bottom_right[i]=row.a[0];
  }

  lcd.createChar(0, box_upper_left);
  lcd.createChar(1, box_upper_right);
  lcd.createChar(2, box_bottom_left);
  lcd.createChar(3, box_bottom_right);

}
