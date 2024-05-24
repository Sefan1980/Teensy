// Bluetooth- and WiFi-bridge with ArduinoOTA
// For Teensysender with ESP32 
// See config.h for parameter

// Use "Partition Scheme: Minimal SPIFFS (1,9MB APP with OTA/190KB SPIFFS)".
// First upload has to be with a serial cable, because of the changed partition scheme.

// Make your settings in config.h
#include "config.h"
#include <esp_wifi.h>
#include <WiFi.h>

#ifdef OTAupdates
  #ifdef MODE_STA
    // ArduinoOTA
    #include <ESPmDNS.h>
    #include <WiFiUdp.h>
    #include <ArduinoOTA.h>
  #endif
#endif

#ifdef BLUETOOTH
// Bluetooth
#include <BluetoothSerial.h>
BluetoothSerial SerialBT;
#endif

#ifdef WiFiAccess
// WiFi
#include <WiFiClient.h>
WiFiClient TheClient;
WiFiServer TCPserver(8881);
#endif

uint8_t BTbuf[bufferSize];
uint16_t iBT = 0;
uint8_t WIFIbuf[bufferSize];
uint16_t inWiFI = 0;
int connectCheck = 0;

void setup() {

  // Open serial connection for console
  Serial.begin(115200);

  // Open serial connection to the Mower
  Serial2.begin(230400);

  
  if (debug) Serial.println("\n\nESP32 BT and WiFi serial bridge V1.00");

  #ifdef MODE_AP
    if (debug) Serial.println("Open ESP Access Point mode");
    // AP mode (phone connects directly to ESP) (no router)
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid);      // configure ssid and password in config.h
    delay(2000);            // VERY IMPORTANT
    WiFi.softAPConfig(ip, ip, netmask);
    if (debug) Serial.println("ESP Access Point mode started at 192.168.4.1 on port 8881");
  #endif

  #ifdef MODE_STA  // STATION mode (ESP connects to router and use config.h IP and pass value)
    if (debug) Serial.println("Start ESP32 Station mode");
    WiFi.mode(WIFI_STA);
    WiFi.config(ip, gateway, netmask);
    WiFi.begin(ssid, pw);
    if (debug) Serial.print("Try to Connect to your Wireless network: ");
    if (debug) Serial.println(ssid);
    WiFiconnect():
  #endif

  #ifdef BLUETOOTH
    if (debug) Serial.println("Start bluetooth server");
    SerialBT.begin("Teensy2"); // Bluetooth device name
  #endif

  #ifdef WiFiAccess
    if (debug) Serial.println("Starting server on port 8881");
    TCPserver.begin(); // start TCP server
    TCPserver.setNoDelay(true);
    // esp_err_t esp_wifi_set_max_tx_power(50);     // Set lower WiFi power
  #endif


  #ifdef OTAupdates
    #ifdef MODE_STA
      ArduinoOTA.setHostname("Teensy_ESP32");         // Hostname in WiFi

      // ArduinoOTA.setPassword("admin");             // Need a password?

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
    #endif
  #endif
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
  } else {
    if (debug) Serial.println("\nWiFi connection failed.");
  }
}


void loop() {

  #ifdef WiFiaccess
    WiFiconnect();
  #endif
  
  #ifdef OTAupdates
    #ifdef MODE_STA
      ArduinoOTA.handle();
    #endif
  #endif

  #ifdef BLUETOOTH
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
  #endif

  #ifdef WiFiAccess
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
  #endif

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

    #ifdef BLUETOOTH
      // now send to Bluetooth:
      if (SerialBT.hasClient())
        SerialBT.write(WIFIbuf, inWiFI);
    #endif

      inWiFI = 0;
    }
  }
}