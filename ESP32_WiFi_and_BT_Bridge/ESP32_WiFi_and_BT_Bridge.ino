// Bluetooth- and WiFi-bridge for Teensysender with ESP32 
// ArduinoOTA
// See config.h for parameter

// Use "Partition Scheme: Minimal SPIFFS (1,9MB APP with OTA/190KB SPIFFS)".
// First upload has to be with a serial cable, because of the changed filesystem.

// Make your settings in config.h
#include "config.h"
//#include <esp_wifi.h>
//#include <WiFi.h>
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager

//ArduinoOTA
//#include <ESPmDNS.h>
//#include <WiFiUdp.h>
#include <ArduinoOTA.h>

// Bluetooth
#include <BluetoothSerial.h>
BluetoothSerial SerialBT;

// WiFi
#include <WiFiClient.h>
WiFiClient TheClient;
WiFiServer TCPserver(8881);
int connectCheck = 0;
uint8_t BTbuf[bufferSize];
uint16_t iBT = 0;
uint8_t WIFIbuf[bufferSize];
uint16_t inWiFI = 0;

void setup() {
  delay(500);
  // Open serial connection for console
  //Serial.begin(115200);
  Serial.begin(115200, SERIAL_8N1, 3, 1);

  // Open serial connection to the Mower
  //Serial2.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 16, 17);
  if (debug) Serial.print("\n\nESP32 BT and WiFi serial bridge ");
  if (debug) Serial.println(VERSION);
  if (debug) Serial.println("Start ESP32 Station mode");
  delay(100);

//NEU
WiFiManager wm;
wm.setSTAStaticIPConfig(ip, gateway, netmask); // set static ip,gw,sn
wm.setShowStaticFields(true); // force show static ip fields
wm.setShowDnsFields(true);    // force show dns field always
bool res = wm.autoConnect("RasenpeterAP","");
if(!res) {
        Serial.println("Failed to connect");
        // ESP.restart();
    } 
    else {
        //if you get here you have connected to the WiFi    
        Serial.println("connected...yeey :)");
    }


/*
  // STATION mode (ESP32 connects to router and use config.h for IP and pass value)
  WiFi.mode(WIFI_STA);
  WiFi.config(ip, gateway, netmask);
  WiFi.begin(ssid, pw);
  if (debug) Serial.print("Try to connect to your wireless network: ");
  if (debug) Serial.println(ssid);

  WiFiconnect();
*/
  if (debug) Serial.println("Start bluetooth server");
  SerialBT.begin("TeensyBT"); //Bluetooth device name

  if (debug) Serial.println("Starting WiFi server on port 8881");
  TCPserver.begin(); // start TCP server
  TCPserver.setNoDelay(true);

  // esp_err_t esp_wifi_set_max_tx_power(50);     // Set lower WiFi Power

  ArduinoOTA.setHostname("Teensy_Bridge_ESP32");         // Hostname in WiFi

  // ArduinoOTA.setPassword("admin");

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();
}

void WiFiconnect() {
  while (WiFi.status() != WL_CONNECTED && connectCheck <= 40) {     // Try max 40 times to connect to the accesspoint
    connectCheck += 1;
    if (debug) Serial.print(".");
    delay(500);
  }
  if (connectCheck <= 40  && connectCheck != 0) {
    if (debug) Serial.print("\nWiFi connected. IP: ");
    if (debug) Serial.println(WiFi.localIP());
    if (debug) Serial.println("Use port 8881");
    connectCheck = 0;
  } else if (connectCheck > 0) {
    if (debug) Serial.println("\nWiFi connection failed.");
    if (debug) Serial.println("connecktCheck=" + connectCheck);
    connectCheck = 0;
  }
}

void loop()
{
  ArduinoOTA.handle();

   WiFiconnect();

  // receive from Bluetooth:
  if (SerialBT.hasClient())
  {
    while (SerialBT.available())
    {
      BTbuf[iBT] = SerialBT.read(); // read char from BT client
      if (iBT < bufferSize - 1) iBT++;
    }
    Serial2.write(BTbuf, iBT); // now send to serial2:
    iBT = 0;
  }

  // recive from WiFi
  if (TCPserver.hasClient())
  {
    //find free/disconnected spot
    if (!TheClient || !TheClient.connected()) {
      if (TheClient) TheClient.stop();
      TheClient = TCPserver.available();
      if (debug) Serial.println("New WIFI client");
    }
    //no free/disconnected spot so reject
    WiFiClient TmpserverClient = TCPserver.available();
    TmpserverClient.stop();
  }

  if (Serial2 != NULL)
  {
    if (TheClient)
    {
      while (TheClient.available())
      {
        WIFIbuf[inWiFI] = TheClient.read(); // read char from client
        if (inWiFI < bufferSize - 1) inWiFI++;
      }
      Serial2.write(WIFIbuf, inWiFI); // now send to UART(2):
      inWiFI = 0;
    }
    if (Serial2.available())
    {
      while (Serial2.available())
      {
        WIFIbuf[inWiFI] = Serial2.read(); // read char from UART(2)
        if (inWiFI < bufferSize - 1) inWiFI++;
      }
      if (TheClient)
        TheClient.write(WIFIbuf, inWiFI);

      // now send to Bluetooth:
      if (SerialBT.hasClient())
        SerialBT.write(WIFIbuf, inWiFI);

      inWiFI = 0;
    }
  }
}
