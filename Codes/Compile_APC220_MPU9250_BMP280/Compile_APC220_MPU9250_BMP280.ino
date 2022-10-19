#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <MPU9250_WE.h>
#include <SoftwareSerial.h>

#define MPU9250_ADDR 0x68
#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)

SoftwareSerial mySerial(4, 5);

MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR);

unsigned long time_now;
unsigned long time_stop = 20000;
double scale_factor_accel = 169.668;
double accel_X,accel_Y, accel_Z;

Adafruit_BMP280 bmp; // I2C

void setup() {
  Serial.begin(38400);
  mySerial.begin(38400);
  while ( !Serial ) delay(100);   // wait for native usb
  Serial.println(F("BMP280 test"));
  unsigned status;
  //status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  status = bmp.begin();
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

    Wire.begin();
  if(!myMPU9250.init()){
    Serial.println("MPU9250 does not respond");
  }
  else{
    Serial.println("MPU9250 is connected");
  }
  if(!myMPU9250.initMagnetometer()){
    Serial.println("Magnetometer does not respond");
  }
  else{
    Serial.println("Magnetometer is connected");
  }

  Serial.println("Position you MPU9250 flat and don't move it - calibrating...");
  delay(1000);
  myMPU9250.autoOffsets();
  Serial.println("Done!");

  myMPU9250.setSampleRateDivider(5);

  myMPU9250.setAccRange(MPU9250_ACC_RANGE_2G);
  
  myMPU9250.enableAccDLPF(true);
  
  myMPU9250.setAccDLPF(MPU9250_DLPF_6);
  
  delay(200);
}

void loop() 
{
    time_now = micros();

    xyzFloat gValue = myMPU9250.getGValues();
//    accel_X = ((double(gValue.x)/scale_factor_accel)-(0.677150538));
//    accel_Y = ((double(gValue.y)/scale_factor_accel)-(0.117553763));
//    accel_Z = ((double(gValue.z)/scale_factor_accel)-(9.870564516));
    accel_X = gValue.x;
    accel_Y = gValue.y;
    accel_Z = gValue.z - 1;
   
    Serial.print("\t");
    Serial.print(accel_X);
    Serial.print(',');
    Serial.print("\t");
    Serial.print(accel_Y);
    Serial.print(',');
    Serial.print("\t");
    Serial.print(accel_Z);
    Serial.print(',');
    
    mySerial.print(accel_X);
    mySerial.print(',');
    mySerial.print(accel_Y);
    mySerial.print(',');
    mySerial.print(accel_Z);
    mySerial.print(',');

    Serial.print('\t');
    Serial.print(F("Temperature,C = "));
    Serial.print(bmp.readTemperature());
    Serial.print('\t');
    Serial.print(F("Pressure,Pa = "));
    Serial.print(bmp.readPressure());
    Serial.print('\t');
    Serial.print(F("Approx altitude,m = "));
    Serial.println(bmp.readAltitude(1013.25)); /* Adjusted to local forecast! */

    mySerial.print(bmp.readTemperature());
    mySerial.print(',');
    mySerial.print(bmp.readPressure());
    mySerial.print(',');
    mySerial.print(bmp.readAltitude(1013.25));
    mySerial.println(',');

    
    while(micros()<= time_now+time_stop);

}
