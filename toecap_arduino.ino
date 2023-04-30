#include <BluetoothSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/*
 * BNO1 - Ankle  1: 0x28
 * BNO2 - Thumb  1: 0x29
 * BNO3 - Middle 2: 0x28
 * BNO4 - Pinky  2: 0x29
 * 
 */

#define SDA_1 22    //toe 3 and 5
#define SCL_1 21

#define SDA_2 18    //Thumb and ankle
#define SCL_2 19

#define LED  2

TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);

Adafruit_BNO055 bno1 = Adafruit_BNO055(55, 0x28, &I2Cone);
Adafruit_BNO055 bno2 = Adafruit_BNO055(55, 0x29, &I2Cone);

Adafruit_BNO055 bno3 = Adafruit_BNO055(55, 0x28, &I2Ctwo);
Adafruit_BNO055 bno4 = Adafruit_BNO055(55, 0x29, &I2Ctwo);

BluetoothSerial SerialBT;

void bnoCal(void){
  uint8_t system1, gyro1, accel1, mag1;
  uint8_t system2, gyro2, accel2, mag2;
  uint8_t system3, gyro3, accel3, mag3;
  uint8_t system4, gyro4, accel4, mag4;

  system1 = gyro1 = accel1 = mag1 = 0;
  system2 = gyro2 = accel2 = mag2 = 0;
  system3 = gyro3 = accel3 = mag3 = 0;
  system4 = gyro4 = accel4 = mag4 = 0;
  
  while((system1 <= 1)||(gyro1<=2)||(mag1<=2)||(system2 <= 1)||(gyro2<=2)||(mag2<=2)||(system3 <= 1)||(gyro3<=2)||(mag3<=2)||(system4 <= 1)||(gyro4<=2)||(mag4<=2)){
    bno1.getCalibration(&system1, &gyro1, &accel1, &mag1);
    bno2.getCalibration(&system2, &gyro2, &accel2, &mag2);
    bno3.getCalibration(&system3, &gyro3, &accel3, &mag3);
    bno4.getCalibration(&system4, &gyro4, &accel4, &mag4);
    Serial.print("Sys:");
    Serial.print(system1, DEC);
    Serial.print(" G:");
    Serial.print(gyro1, DEC);
    Serial.print(" A:");
    Serial.print(accel1, DEC);
    Serial.print(" M:");
    Serial.println(mag1, DEC);
  }
}

void setup(void) 
{
  pinMode(LED,OUTPUT);
  Serial.begin(115200);
  SerialBT.begin("ESP ToeCap");
  Serial.println("Bluetooth activated");
  I2Cone.begin(SDA_1, SCL_1, 100000); 
  I2Ctwo.begin(SDA_2, SCL_2, 100000);
  if(!bno1.begin())
  {
    Serial.print("BNO1 not found");
    while(1);
  }
  if(!bno2.begin())
  {
    Serial.print("BNO2 not found");
    while(1);
  }
  if(!bno3.begin())
  {
    Serial.print("BNO3 not found");
    while(1);
  }
  if(!bno4.begin())
  {
    Serial.print("BNO4 not found");
    while(1);
  }
  delay(1000);
  bno1.setExtCrystalUse(true);
  bno2.setExtCrystalUse(true);
  bno3.setExtCrystalUse(true);
  bno4.setExtCrystalUse(true);
  Serial.println("Calibrating...");
  bnoCal();
  digitalWrite(LED, HIGH);
  Serial.println("Calibrated");
}

void loop(void) 
{
  sensors_event_t event; 
  bno1.getEvent(&event);
  bno2.getEvent(&event);
  bno3.getEvent(&event);
  bno4.getEvent(&event);
  imu::Quaternion quat1 = bno1.getQuat();
  imu::Quaternion quat2 = bno2.getQuat();
  imu::Quaternion quat3 = bno3.getQuat();
  imu::Quaternion quat4 = bno4.getQuat();
  /* Display the floating point data */
  Serial.print("qW: ");
  Serial.print(quat1.w(), 4);
  Serial.print(quat2.w(), 4);
  Serial.print(quat3.w(), 4);
  Serial.print(quat4.w(), 4);
  Serial.print(" qX: ");
  Serial.print(quat1.x(), 4);
  Serial.print(quat2.x(), 4);
  Serial.print(quat3.x(), 4);
  Serial.print(quat4.x(), 4);
  Serial.print(" qY: ");
  Serial.print(quat1.y(), 4);
  Serial.print(quat2.y(), 4);
  Serial.print(quat3.y(), 4);
  Serial.print(quat4.y(), 4);
  Serial.print(" qZ: ");
  Serial.print(quat1.z(), 4);
  Serial.print(quat1.z(), 4);
  Serial.print(quat3.z(), 4);
  Serial.print(quat4.z(), 4);
  Serial.println("");
  
  SerialBT.print(String(quat1.w(),4)+","+String(quat1.x(),4)+","+String(quat1.y(),4)+","+String(quat1.z(),4)+",");
  SerialBT.print(String(quat2.w(),4)+","+String(quat2.x(),4)+","+String(quat2.y(),4)+","+String(quat2.z(),4)+",");
  SerialBT.print(String(quat3.w(),4)+","+String(quat3.x(),4)+","+String(quat3.y(),4)+","+String(quat3.z(),4)+",");
  SerialBT.println(String(quat4.w(),4)+","+String(quat4.x(),4)+","+String(quat4.y(),4)+","+String(quat4.z(),4));
  
  delay(10);
}
