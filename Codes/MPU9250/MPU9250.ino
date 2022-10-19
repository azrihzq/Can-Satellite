#include <MPU9250_WE.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#define MPU9250_ADDR 0x68

SoftwareSerial mySerial(4, 5);

MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR);

unsigned long time_now;
unsigned long time_stop = 20000;

void setup() {
  Serial.begin(115200);
  mySerial.begin(38400);
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
  float zvalue = gValue.z-0.02;

  Serial.print("\t");
  Serial.print(gValue.x);
  Serial.print("\t");
  Serial.print(gValue.y);
  Serial.print("\t");
  Serial.println(zvalue);

  while(micros()<= time_now+time_stop);
}
