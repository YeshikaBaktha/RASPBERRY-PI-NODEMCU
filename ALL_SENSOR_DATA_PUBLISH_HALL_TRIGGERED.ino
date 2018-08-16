#include "Wire.h"
#include <ESP8266WiFi.h>
#include <PubSubClient.h> 
#include <ArduinoJson.h>
#include <stdlib.h>
#include "DHT.h"
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
#define DHTPIN 13 //D7 
DHT dht(DHTPIN, DHTTYPE);

int pin_hall = 12; //D6
int pin_ldr = A0;
const int sleepTimeS = 1*1; //1 second sleep
const char* ssid = "Raspberry_Pi_access_point_name";
const char* wifi_password = "password";
const char* mqtt_server = "172.24.2.2"; // raspberry pi mqtt_server name
const char* mqtt_topic = "ESP2/test_ESP2"; //topic to which you post
const char* mqtt_username = "Raspberry_Pi_1"; //PI user name
const char* mqtt_password = "Raspberry1"; //pi password
const char* clientID = "ESP1";

StaticJsonBuffer<600> jsonBuffer; //data is sent in json format; create buffer for 
StaticJsonBuffer<600> sensorBuffer; //create buffer for sensor data
StaticJsonBuffer<600> idbuffer; //creat buffer to hold ID of the NODEMCU
String jsonString ; 
String sensorString ;
String idString;
String pubString;

int count;
int ldr;
int hall;
float h ;
float t ;


#define your_device_id "5CCF7F3D7682"

const uint8_t MPU_addr=0x68; // I2C address of the MPU-6050
 
const float MPU_GYRO_250_SCALE = 131.0;
const float MPU_GYRO_500_SCALE = 65.5;
const float MPU_GYRO_1000_SCALE = 32.8;
const float MPU_GYRO_2000_SCALE = 16.4;
const float MPU_ACCL_2_SCALE = 16384.0;
const float MPU_ACCL_4_SCALE = 8192.0;
const float MPU_ACCL_8_SCALE = 4096.0;
const float MPU_ACCL_16_SCALE = 2048.0;

struct rawdata {
int16_t AcX;
int16_t AcY;
int16_t AcZ;
int16_t Tmp;
int16_t GyX;
int16_t GyY;
int16_t GyZ;
};
 
struct scaleddata{
float AcX;
float AcY;
float AcZ;
float Tmp;
float GyX;
float GyY;
float GyZ;
};
 
bool checkI2c(byte addr);
void mpu6050Begin(byte addr);
rawdata mpu6050Read(byte addr, bool Debug);
void setMPU6050scales(byte addr,uint8_t Gyro,uint8_t Accl);
void getMPU6050scales(byte addr,uint8_t &Gyro,uint8_t &Accl);
scaleddata convertRawToScaled(byte addr, rawdata data_in,bool Debug);

WiFiClient wifiClient;
PubSubClient client(mqtt_server, 1883, wifiClient); //enabling nodemcu to access WiFi

void setup() {
  Wire.begin();
  Serial.begin(115200);
  mpu6050Begin(MPU_addr);
  Serial.print("Connecting to ");
  Serial.println(ssid);
  // Connect to the WiFi
  WiFi.begin(ssid, wifi_password);
  // Wait until the connection has been confirmed before continuing
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    }
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  dht.begin();
  pinMode(pin_hall, INPUT); 
  pinMode(pin_ldr, INPUT);
  Serial.println("");
   if (client.connect(clientID, mqtt_username, mqtt_password)) {
    Serial.println("Connected to MQTT Broker!");
  }
  else {
    Serial.println("Connection to MQTT Broker failed...");
  }
}

void loop(){
  hall = digitalRead(pin_hall);
  Serial.println("HALL STATE");
  Serial.println(hall);
  if (hall == 0){ //when hall sensor is triggered the device is made to publish its current data to the raspberry pi(mqtt broker)
    read_sensor_data();
    }
}

void read_sensor_data(){
  hall = digitalRead(pin_hall);
  ldr = analogRead(pin_ldr);
  h = dht.readHumidity();
  t = dht.readTemperature();
  gyroscope_accelerometer();
}

void gyroscope_accelerometer(){
rawdata next_sample;
setMPU6050scales(MPU_addr,0b00000000,0b00010000);
next_sample = mpu6050Read(MPU_addr, true);
convertRawToScaled(MPU_addr, next_sample,true);
delay(5000); // Wait 5 seconds and scan again
}

void mpu6050Begin(byte addr){
// This function initializes the MPU-6050 IMU Sensor
// It verifys the address is correct and wakes up the
// MPU.
if (checkI2c(addr)){
Wire.beginTransmission(MPU_addr);
Wire.write(0x6B); // PWR_MGMT_1 register
Wire.write(0); // set to zero (wakes up the MPU-6050)
Wire.endTransmission(true);

delay(30); // Ensure gyro has enough time to power up
}
}

bool checkI2c(byte addr){
// We are using the return value of
// the Write.endTransmisstion to see if
// a device did acknowledge to the address.
Serial.println(" ");
Wire.beginTransmission(addr);
 
if (Wire.endTransmission() == 0)
{
Serial.print(" Device Found at 0x");
Serial.println(addr,HEX);
return true;
}
else
{
Serial.print(" No Device Found at 0x");
Serial.println(addr,HEX);
return false;

}
}

rawdata mpu6050Read(byte addr, bool Debug){
// This function reads the raw 16-bit data values from
// the MPU-6050
 
rawdata values;
 
Wire.beginTransmission(addr);
Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
Wire.endTransmission(false);
Wire.requestFrom(addr,14,true); // request a total of 14 registers
values.AcX=Wire.read()<<8|Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
values.AcY=Wire.read()<<8|Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
values.AcZ=Wire.read()<<8|Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
values.Tmp=Wire.read()<<8|Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
values.GyX=Wire.read()<<8|Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
values.GyY=Wire.read()<<8|Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
values.GyZ=Wire.read()<<8|Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
 
 
if(Debug){
Serial.print(" GyX = "); Serial.print(values.GyX);
Serial.print(" | GyY = "); Serial.print(values.GyY);
Serial.print(" | GyZ = "); Serial.print(values.GyZ);
Serial.print(" | Tmp = "); Serial.print(values.Tmp);
Serial.print(" | AcX = "); Serial.print(values.AcX);
Serial.print(" | AcY = "); Serial.print(values.AcY);
Serial.print(" | AcZ = "); Serial.println(values.AcZ);
}

return values;
}
 
void setMPU6050scales(byte addr,uint8_t Gyro,uint8_t Accl){
Wire.beginTransmission(addr);
Wire.write(0x1B); // write to register starting at 0x1B
Wire.write(Gyro); // Self Tests Off and set Gyro FS to 250
Wire.write(Accl); // Self Tests Off and set Accl FS to 8g
Wire.endTransmission(true);
}
 
void getMPU6050scales(byte addr,uint8_t &Gyro,uint8_t &Accl){
Wire.beginTransmission(addr);
Wire.write(0x1B); // starting with register 0x3B (ACCEL_XOUT_H)
Wire.endTransmission(false);
Wire.requestFrom(addr,2,true); // request a total of 14 registers
Gyro = (Wire.read()&(bit(3)|bit(4)))>>3;
Accl = (Wire.read()&(bit(3)|bit(4)))>>3;

}
 
  
 
scaleddata convertRawToScaled(byte addr, rawdata data_in, bool Debug){
 
scaleddata values;
float scale_value = 0.0;
byte Gyro, Accl;
 
getMPU6050scales(MPU_addr, Gyro, Accl);
 
if(Debug){
//Serial.print("Gyro Full-Scale = ");
}
 
switch (Gyro){
case 0:
//scale_value = MPU_GYRO_250_SCALE;
if(Debug){
//Serial.println("±250 °/s");
}
break;
case 1:
//scale_value = MPU_GYRO_500_SCALE;
if(Debug){
//Serial.println("±500 °/s");
}
break;
case 2:
//scale_value = MPU_GYRO_1000_SCALE;
if(Debug){
//Serial.println("±1000 °/s");
}
break;
case 3:
//scale_value = MPU_GYRO_2000_SCALE;
if(Debug){
//Serial.println("±2000 °/s");
}
break;
default:
break;
}
 
values.GyX = (float) data_in.GyX / scale_value;
values.GyY = (float) data_in.GyY / scale_value;
values.GyZ = (float) data_in.GyZ / scale_value;
 Serial.println(values.GyX);
scale_value = 0.0;
if(Debug){
//Serial.print("Accl Full-Scale = ");
}
switch (Accl){
case 0:
scale_value = MPU_ACCL_2_SCALE;
if(Debug){
//Serial.println("±2 g");
}
break;
case 1:
scale_value = MPU_ACCL_4_SCALE;
if(Debug){
//Serial.println("±4 g");
}
break;
case 2:
scale_value = MPU_ACCL_8_SCALE;
if(Debug){
//Serial.println("±8 g");
}
break;
case 3:
scale_value = MPU_ACCL_16_SCALE;
if(Debug){
//Serial.println("±16 g");
}
break;
default:
break;
}
values.AcX = (float) data_in.AcX / scale_value;
values.AcY = (float) data_in.AcY / scale_value;
values.AcZ = (float) data_in.AcZ / scale_value;
values.Tmp = (float) data_in.Tmp / 340.0 + 36.53;
 
if(Debug){
//Serial.print(" GyX = "); Serial.print(values.GyX);
//Serial.print(" °/s| GyY = "); Serial.print(values.GyY);
//Serial.print(" °/s| GyZ = "); Serial.print(values.GyZ);
//Serial.print(" °/s| Tmp = "); Serial.print(values.Tmp);
//Serial.print(" °C| AcX = "); Serial.print(values.AcX);
//Serial.print(" g| AcY = "); Serial.print(values.AcY);
//Serial.print(" g| AcZ = "); Serial.print(values.AcZ);Serial.println(" g");

}

JsonObject& nested = sensorBuffer.createObject();

nested["hall_sensor"]="DOOR AJAR";
nested["temperature_Degree Celsius"]=t;
nested["humidity"]=h;
nested["ldr"]=ldr;
nested["Gyroscope_X_axis"]=values.GyX;
nested["Gyroscope_Y_axis"]=values.GyY;
nested["Gyroscope_Z_axis"]=values.GyZ;
nested["Gyroscope_Temperature"]=values.Tmp;
nested["Accelerometer_X_axis"]=values.AcX;
nested["Accelerometer_Y_axis"]=values.AcY;
nested["Accelerometer_Z_axis"]=values.AcZ;
nested.printTo(sensorString);
JsonObject& id = idbuffer.createObject();
id["key"]=your_device_id;
id["data"] = sensorString;
id.printTo(idString);
char jsonChar[500];
idString.toCharArray(jsonChar, 600);
Serial.println(jsonChar);

if (client.connect(clientID, mqtt_username, mqtt_password)) {
    Serial.println("Connected to MQTT Broker!");
     client.publish(mqtt_topic, jsonChar); // publish jsonChar to topic ESP2/test_ESP2 
  Serial.println("message sent!");
  }
  else {
    Serial.println("Connection to MQTT Broker failed...");
  }
 //delay(2000);
 while(1){
  check_hall_status();
 }
}

void check_hall_status(){
hall = digitalRead(pin_hall);
count = count + 1 ;
delay(500);
if (count == 20){ //check or 10 seconds if the hall sensor is still active (open door) 
  JsonObject& nested = sensorBuffer.createObject();
  nested["DOOR STATUS"]="DOOR AJAR";
  nested.printTo(sensorString);
  JsonObject& id = idbuffer.createObject();
  id["key"]=your_device_id;
  id["data"] = sensorString;
  id.printTo(idString);
  char jsonChar[500];
  idString.toCharArray(jsonChar, 600);
  Serial.println(jsonChar);
  if (client.connect(clientID, mqtt_username, mqtt_password)) {
    Serial.println("Connected to MQTT Broker!");
    client.publish(mqtt_topic, jsonChar); 
  Serial.println("message sent!");
  }
  else {
    Serial.println("Connection to MQTT Broker failed...");
  }
  }
  else{
    ESP.restart();
  }
 }

