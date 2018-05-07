//Define DEBUG to get the Output from DEBUG_PRINTLN
#define DEBUG 1

#include "soc/rtc.h"
#include "rom/uart.h"
#include <Basecamp.hpp>
#include <DHTesp.h>
#include <time.h>
#include <ArduinoJson.h>

// Define the Pin Mapping
#define LEDPIN 16
#define DHTPIN 22
#define SOILPIN 32
#define LIGHTPIN 33
#define BATTERYPIN 34
#define uS_TO_S_FACTOR 1000000
#define uS_TO_MIN_FACTOR 60000000

//Create a new Basecamp instance called iot
//Basecamp wird angewiesen einen verschlüsselten Acess-Point zu öffnen. Das Passwort erscheint in der seriellen Konsole.
Basecamp iot{
  Basecamp::SetupModeWifiEncryption::secured
};

//DHT11 instance
DHTesp dht;

//Variables for the mqtt packages and topics
uint16_t statusPacketIdSub = 0;
String ControlTopic;
String SensorTopic;

//Parameter prvided by the MQTT Broker
RTC_DATA_ATTR int sleepcontrol = 0;

//NTP Server and Time Information
const char* ntpServer = "192.168.5.1";
const long  gmtOffset_sec = 3600;
const int   daylightOffset_sec = 3600;

//Helper function to print the LocalTime
String getLocalTime()
{
  struct tm timeinfo;
  char timestamp[80]; // declare a character variable that contains timestamp

  if (!getLocalTime(&timeinfo)) {
    return String("");
  }

  strftime(timestamp, 80, "%d.%m.%Y %H:%M:%S", &timeinfo);
  return String(timestamp);
}

//setup function invoked after bootip
void setup() {
  //configuration of the pins
  pinMode(LEDPIN, OUTPUT);
  pinMode(DHTPIN, INPUT);
  pinMode(SOILPIN, INPUT);
  pinMode(LIGHTPIN, INPUT);
  pinMode(BATTERYPIN, INPUT);

  digitalWrite(LEDPIN, LOW); //switch on blue LED

  uart_tx_wait_idle(0);
  rtc_clk_cpu_freq_set(RTC_CPU_FREQ_160M);
  
  //setup instances
  dht.setup(DHTPIN, DHTesp::DHT11);
  iot.begin();

  //setup topics using iot.hostname
  ControlTopic = "home/higrow/" + iot.hostname + "/sleepcontrol";
  SensorTopic = "home/higrow/" + iot.hostname + "/sensordata";

  //register MQTT callbacks
  iot.mqtt.onConnect(mqttConnected);
  iot.mqtt.onMessage(mqttMessage);
  iot.mqtt.onPublish(suspendESP);  
}

//MQTT callback function called on connection
void mqttConnected(bool sessionPresent) {
  DEBUG_PRINTLN("MQTT Connection sucessfull!");

  //init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  //subscribe to the parameter topics
  iot.mqtt.subscribe(ControlTopic.c_str(), 2);
  //read and publish sensors
  GetSensors();
}

//MQTT callback function called on subscription messages
void mqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  DEBUG_PRINT("MQTT-Topic received: ");
  DEBUG_PRINT(topic);
  DEBUG_PRINT(", Payload: ");
  DEBUG_PRINTLN(payload);
  if (strcmp(topic, ControlTopic.c_str()) == 0) {
    sleepcontrol = constrain(atoi(payload), 0, 60);
  }
};

void suspendESP(uint16_t packetId) {
  DEBUG_PRINTLN(__func__);

  //Check if the published package is the one of the door sensor
  if (packetId == statusPacketIdSub) {
    if (sleepcontrol == 0) {
      DEBUG_PRINTLN("Delaying Sleep");
      return;
    }
    DEBUG_PRINTLN("Entering deep sleep");
    //properly disconnect from the MQTT broker
    //iot.mqtt.disconnect();
    digitalWrite(LEDPIN, LOW); //switch off blue LED
    //send the ESP into deep sleep
    esp_sleep_enable_timer_wakeup(sleepcontrol * uS_TO_MIN_FACTOR);
    esp_deep_sleep_start();
  }
}

//function to read, log, and publish sensor data
void GetSensors() {
  StaticJsonBuffer<256> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();

  String dtime = getLocalTime();
  String temp = String(dht.getTemperature(), 0);
  String humidity = String(dht.getHumidity(), 0);
  String battery = String(analogRead(BATTERYPIN));

  int waterlevel = analogRead(SOILPIN);
  int lightlevel = analogRead(LIGHTPIN);
  waterlevel = map(waterlevel, 0, 4095, 0, 1023);
  waterlevel = constrain(waterlevel, 0, 1023);
  lightlevel = map(lightlevel, 0, 4095, 0, 1023);
  lightlevel = constrain(lightlevel, 0, 1023);
  String water = String(waterlevel);
  String light = String(lightlevel);

  DEBUG_PRINTLN("");
  DEBUG_PRINTLN("Sensor Data:");
  DEBUG_PRINTLN("Temp = " + temp);
  DEBUG_PRINTLN("Hum = " + humidity);
  DEBUG_PRINTLN("Soil = " + water);
  DEBUG_PRINTLN("Light = " + light);
  DEBUG_PRINTLN("Bat = " + battery);
  DEBUG_PRINTLN("Time = " + dtime);
  DEBUG_PRINTLN("SleepControl = " + String(sleepcontrol));

  root["temperature"] = temp;
  root["humidity"] = humidity;
  root["soil"] = water;
  root["light"] = light;
  root["battery"] = battery;
  root["time"] = dtime;

  String output;
  root.printTo(output);
  statusPacketIdSub = iot.mqtt.publish(SensorTopic.c_str(), 1, true, output.c_str() );
}

//main loop
void loop() {
  delay(10000);
  if (sleepcontrol > 0) {
    DEBUG_PRINTLN("Timeout! Entering deep sleep");
    DEBUG_PRINTLN("SleepControl = " + String(sleepcontrol));
    //properly disconnect from the MQTT broker
    //iot.mqtt.disconnect();
    digitalWrite(LEDPIN, HIGH); //switch off blue LED
    //send the ESP into deep sleep
    esp_sleep_enable_timer_wakeup(sleepcontrol * uS_TO_MIN_FACTOR);
    esp_deep_sleep_start();
  }
  delay(50000); //wait
  digitalWrite(LEDPIN, LOW); //switch on blue LED
  GetSensors();
  digitalWrite(LEDPIN, HIGH);
}
