//zd75
//DHT Setup
#include "DHT.h"
#define DHTPIN 2 // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11
float h = 0 ;
float c = 0;

// Photoresistor Setup
int photocellPin = A0; //photoresistor module attach to A0
//const int ledPin = 13; //pin 13 built-in led
int outputValue = 0;

// accelerometer Setup (i2c)
#include "Wire.h" // This library allows you to communicate with I2C devices.
const int MPU_ADDR = 0x68; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.

int16_t accelerometer_x, accelerometer_y, accelerometer_z; // variables for accelerometer raw data
int16_t gyro_x, gyro_y, gyro_z; // variables for gyro raw data
int16_t temperature; // variables for temperature data
double delta; // Total displacement
double theta; // Gyroscope displacement
double sum_d;
double sum_t;
double last_d;
double last_t;
//boolean m = false;
boolean safe = true;

char tmp_str[7]; // temporary variable used in convert function
int count = 0; // average counter

char* convert_int16_to_str(int16_t i) { // converts int16 to string. Moreover, resulting strings will have the same length in the debug monitor.
  sprintf(tmp_str, "%6d", i);
  return tmp_str;
}
//BlueTooth
#include <SoftwareSerial.h>
//SoftwareSerial Bluetooth(10, 9); // RX, TX


DHT dht(DHTPIN, DHTTYPE); // Initialize DHT sensor.


void setup() {
  //start DHT 
  Serial.begin(9600);
  dht.begin();

  //start photodiode
  //pinMode(ledPin,OUTPUT); //set ledPin as OUTPUT

  //start accelerometer
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

}

void loop() {
  delay(1000);// Wait between measurements.
  
  // DHT temp & humidity data
  float h = dht.readHumidity();
  float c = dht.readTemperature();
  float f = dht.readTemperature(true);
  if (isnan(h) || isnan(c) || isnan(f)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

  Serial.print(F("Humidity: "));Serial.print(h);
  Serial.print(F(" %  "));Serial.print(f);
  Serial.print(F(" °C "));
  

  
  // Photodiode
  outputValue = analogRead(photocellPin);//read the value of photoresistor
  Serial.print(F("Photodiode Reading: "));
  Serial.println(outputValue); //print it in serial monitor
  
  
  
  // accelerometer
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(MPU_ADDR, 7*2, true); // request a total of 7*2=14 registers

  // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
  accelerometer_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  accelerometer_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  accelerometer_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
  temperature = Wire.read()<<8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
  gyro_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
  gyro_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
  gyro_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)
  
  delta = pow(accelerometer_x,2) + pow(accelerometer_y,2) + pow(accelerometer_z,2);
  delta = sqrt(delta);
  theta = pow(gyro_x,2) + pow(gyro_y,2) + pow(gyro_z,2);
  theta = sqrt(theta);
  
  // print out data
  Serial.print("aX = "); Serial.print(convert_int16_to_str(accelerometer_x));
  Serial.print(" | aY = "); Serial.print(convert_int16_to_str(accelerometer_y));
  Serial.print(" | aZ = "); Serial.print(convert_int16_to_str(accelerometer_z));
  // the following equation was taken from the documentation [MPU-6000/MPU-6050 Register Map and Description, p.30]
  Serial.print(" | tmp = "); Serial.print(temperature/340.00+36.53);
  Serial.print(" | gX = "); Serial.print(convert_int16_to_str(gyro_x));
  Serial.print(" | gY = "); Serial.print(convert_int16_to_str(gyro_y));
  Serial.print(" | gZ = "); Serial.println(convert_int16_to_str(gyro_z));
  Serial.print("Delta "); Serial.print(delta);
  Serial.print("  Theta "); Serial.print(theta);
  Serial.println();
  Serial.println();
  
  
  
}
