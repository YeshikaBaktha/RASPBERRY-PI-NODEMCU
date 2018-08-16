#include <ESP8266WiFi.h>
#include <PubSubClient.h> 
#include <ArduinoJson.h>

int pin_pir = 12; //D6
int pir_adc = A0;
int pin_led = 2; //D4 inbuilt led
const int sleepTimeS = 1*360; //SLeep time to set 1*number of seconds

const char* ssid = "Raspberry_Pi_1"; //Raspberry pi's accesspoint name
const char* wifi_password = "RBCCPS04"; //password
const char* mqtt_server = "172.24.2.2"; //ip address of raspberry pi
const char* mqtt_topic = "ESP4/test_ESP4"; //mqtt topic
const char* mqtt_username = "Raspberry_Pi_1"; //mqtt user name 
const char* mqtt_password = "Raspberry1"; // mqtt password
const char* clientID = "ESP2"; //our device name 
#define your_device_id "5CCF7F3D7782" // mac address

StaticJsonBuffer<600> jsonBuffer;
StaticJsonBuffer<600> sensorBuffer;
StaticJsonBuffer<600> idbuffer;
String jsonString ;
String sensorString ;
String idString;
String pubString;
int count = 0;
int count_2 = 0;
int count_for_sleep = 0;
WiFiClient wifiClient;
PubSubClient client(mqtt_server, 1883, wifiClient); // 1883 is the listener port for the Broker
 
void setup() {
Serial.begin(115200);
Serial.print("Connecting to ");
Serial.println(ssid);
// Connect to the WiFi
WiFi.begin(ssid, wifi_password);
// Wait until the connection has been confirmed before continuing
while (WiFi.status() != WL_CONNECTED) {
delay(500);
Serial.print(".");
count_for_sleep = count_for_sleep + 1 ;
  if (count_for_sleep == 600) //five minutes 
  {
    Serial.println("ESP8266 in sleep mode");
    ESP.deepSleep(sleepTimeS * 1000000);
    }
    }
Serial.println("WiFi connected");
Serial.print("IP address: ");
Serial.println(WiFi.localIP());
pinMode(pin_pir, INPUT);
pinMode(pir_adc, INPUT);
pinMode(pin_led, OUTPUT);
if (client.connect(clientID, mqtt_username, mqtt_password)) {
    Serial.println("Connected to MQTT Broker!");
  }
else {
    Serial.println("Connection to MQTT Broker failed...");
  } 
}

void loop(){
int adc = analogRead(pir_adc);
Serial.println(adc);
digitalWrite(pin_led, HIGH);
delay(100);
if (adc == 1024){
  count = count + 1;
  digitalWrite(pin_led, HIGH);
  delay(50);
  digitalWrite(pin_led, LOW);
  delay(50);
  Serial.println ("count1");
  Serial.println(count);
  if (count == 2) {
  client.connect(clientID, mqtt_username, mqtt_password);
  client.publish(mqtt_topic, "MOTION DETECTED");
  Serial.println("MOTION DETECTED message sent!");
  delay(50);
  ESP.restart();
  }
  }

if (adc <= 100){
  delay(100);
  count = 0;
  count_2 = count_2 +1;
  Serial.println("count_2  " + String(count_2));
    if (count_2 == 1000){
      client.connect(clientID, mqtt_username, mqtt_password);
      client.publish(mqtt_topic, "NO MOTION DETECTED");
      Serial.println("NO MOTION DETECTED message sent!");
      delay(50);
      ESP.restart();
     }
    }
    else {
 count_2 = 0;
 }

} 
  
 





