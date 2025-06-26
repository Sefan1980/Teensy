/*
  WIFI Communicating sender with 2 possible loops
  Adjust IP according to your ESP32 value 10.0.0.150 in this example
  On your browser type:
  http://10.0.0.150  **************** This page updates every 5 seconds to see the actual state of the sender - you could change the settings by clicking on it like shown below. This Page refreshes every 5 seconds.
  
  Signalcode:     --> click at the number to change the signal
  Sendingspeed:   -->	click at the number to change the signal sending speed
  Sender A:       --> click at on or off to change the state
  Sender B:       --> click at on or off to change the state
  Automaticmode:  --> click at on or off to change the state
  

  ------COLLABORATION FROM BERNARD, SASCHA AND STEFAN------
*/


//********************* includes **********************************
#include <FS.h>                       //this needs to be first, or it all crashes and burns...
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <WiFiManager.h>
WiFiManager wm;
#include "PersonalAccessData.h"       // Here is your phonenumber and the API for Whatsapp-messaging
#include <WiFiClient.h>
#include <HTTPClient.h>
#include <UrlEncode.h>
#include <INA226_WE.h>                // INA226_WE library from Wolfgang Ewald
#include <ArduinoOTA.h>               // ArduinoOTA from Juraj Andrassy


#include <U8x8lib.h>                  // U8g2 from Oliver Kraus
//********************* Display Settings **********************************
// Please UNCOMMENT one of the contructor lines below
// U8x8 Contructor List 
// The complete list is available here: https://github.com/olikraus/u8g2/wiki/u8x8setupcpp
// Please update the pin numbers according to your setup. Use U8X8_PIN_NONE if the reset pin is not connected
 //U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE); 	      
//U8X8_SSD1306_128X64_ALT0_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE); 	      // same as the NONAME variant, but may solve the "every 2nd line skipped" problem
U8X8_SH1106_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);
// End of constructor list


//********************* defines **********************************
// Comment the lines out, you don't need!
#define OTAUpdates 1                  // OTA Updates

#define WhatsApp_messages 1           // Receive Messages when the mower starts from station and returns to station. (Configuration necessary)
                                      // 1) Create the Whatabot contact in your smartphone. The phone number is: +54 9 2364205798
                                      // 2) Send: "I allow whatabot to send me messages"
                                      // 3) Copy the phonenumber and the API that Whatabot sent you and enter it below.
                                      // Type your phone-number (e.g.: +49 170 123456789 --> 4917012345678) and your API-key in PersonalAccessData.h


bool AUTO_START_SIGNAL = 1;           // Use to start sender when mower leave station
#define USE_STATION 1                 // This station is used to charge the Mower. Then show the chargecurrent.
#define USE_PERI_CURRENT 1            // Use Feedback for perimeter current measurements? (set to '0' if not connected!)
//#define USE_BUTTON 0                // Use button to start mowing or send mower to station not finish
//#define SerialOutput 1              // Show serial textmessages for debugging
bool debug = false;                    // for serial output (Wifimanager)
#define Screen 1                      // Screen or not?

#define WORKING_TIMEOUT_MINS 300      // Timeout for perimeter switch-off if robot not in station (minutes) - If AUTO_START_SIGNAL is active, this setting does not work!
#define PERI_CURRENT_MIN 200          // Minimum milliAmpere for cutting wire detection

#define I2C_SDA 21                    // SDA pin
#define I2C_SCL 22                    // SCL pin
#define pinIN1 12                     // M1_IN1  ESP32 GPIO12       ( connect this pin to L298N-IN1)
#define pinIN2 13                     // M1_IN2  ESP32 GPIO13       ( connect this pin to L298N-IN2)
#define pinEnableA 23                 // ENA    ESP32 GPIO23         (connect this pin to L298N-ENA)
#define pinIN3 14                     // M1_IN3  ESP32 GPIO14       ( connect this pin to L298N-IN3)
#define pinIN4 18                     // M1_IN4  ESP32 GPIO18       ( connect this pin to L298N-IN4)
#define pinEnableB 19                 // ENB    ESP32 GPIO19        (connect this pin to L298N-ENA)
//#define pinDoorOpen 34              // Not in use (Magnetic switch)
//#define pinDoorClose 35             // Not in use (Magnetic switch)
//#define pinLDR 32                   // Not in use (Light Sensor)

// At the PCB is a connector for a 2-color LED with common cathode(-). (Attention: The Matrix Mow800 has a LED with common anode(+)!!!)
#define pinGreenLED 25                                  // Station is ready!

// Battery is charging if ChargeCurrent > LoadingThreshold
#define pinRedLED 26                                    // Battery is charging

#define VER "ESP32Sender 3.0 - 26.06.25"                      // code version


//********************* WLAN Settings **********************************                                                       
char static_ip[16] = "192.168.178.124";                 // default IP
char static_gw[16] = "192.168.178.1";                   // default gateway
char static_sn[16] = "255.255.255.0";                   // default subnet
WiFiServer server(80);


//********************* INA226 Settings **********************************
INA226_WE InaPeri = INA226_WE(0x40);                    // 0x40 = without bridge
float resistorPeri = 0.1;                               // for 10mOhm resistor try a value 0.02. For 100mOhm resistor use 0.1 
float rangePeri = 0.8;                                  // Range for 10mOhm Resistor: try 8.0 or 4.0 - For 100mOhm use 0.8

INA226_WE InaCharge = INA226_WE(0x44);                  // 0x44 = bridge between A1 and VSS
float resistorCharge = 0.02;                            
float rangeCharge = 4.0;


//********************* other **********************************
bool WORKING_TIMEOUT = 0;
bool mowerIsWorking = 0;                                // Code for Whatsapp
byte sigCodeInUse = 1;                                  // 1 is the original ardumower sigcode
int sigDuration = 104;                                  // send the bits each 104 microsecond (Also possible 50)
int8_t sigcode_norm[128];
int sigcode_size;
hw_timer_t* timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile int step = 0;
boolean enableSenderA = false;                          //OFF on start to autorise the reset
boolean enableSenderB = false;                          //OFF on start to autorise the reset
int timeSeconds = 0;
unsigned long nextTimeControl = 0;
unsigned long nextTimeSec = 0;
int workTimeMins = 0;
int workTimeChargeMins = 0;
int lastChargeMins = 0;
float PeriCurrent = 0.0;                                // in mA
float PeriBusVoltage = 0.0;                             // voltage at the wire
float PeriShuntVoltage = 0.0;                           // voltage drop across the shunt
float ChargeCurrent = 0.0;
float ChargeCurrentPrint = 0.0;
float ChargeBusVoltage = 0.0;
float ChargeShuntVoltage = 0.0;
bool shouldSaveConfig = false;                          // flag for saving data
bool wm_nonblocking = false;                            // the captive portal blocks the loop
unsigned long previousMillis = millis()-1000;           // WhatsApp
const long interval = 1000;                             // pause interval after sending a WhatsApp-message
String AutoStartSignalPrint;                            // for the web-part
String linktext;                                        // for the web-part
String enableSenderAprint;                              // for the web-part
String enableSenderBprint;                              // for the web-part


/*
  If the Mower is in station and fully charged, the current should be between
  PeriOnOffThreshold(3mA) and ChargeThreshold(10mA)

  If Perimeter starts and mower is in the station, change ChargeThreshold to 0. So you can see the original ChargeCurrent value at http://Your-IP
  If mower is outside, ChargeCurrent should be 0
*/
float ChargeThreshold = 10.0;               // in mA. If the Chargecurrent is below this value, at  the Display shows "0mA" 
float PeriOnOffThreshold = 1.5;             // if ChargeCurrent is below this value, the perimeterloop starts working

//*********************  Sigcode list *********************************************
// must be multiple of 2 !
// http://grauonline.de/alexwww/ardumower/filter/filter.html
// "pseudonoise4_pw" signal (sender)
int8_t sigcode0[] = { 1, -1 };                                                                                //simple square test code
int8_t sigcode1[] = { 1, 1, -1, -1, 1, -1, 1, -1, -1, 1, -1, 1, 1, -1, -1, 1, -1, -1, 1, -1, -1, 1, 1, -1 };  //ardumower signal code
int8_t sigcode2[] = { 1, 1, -1, -1, 1, -1, 1, -1, -1, 1, -1, 1, 1, -1, -1, 1, -1, -1, 1, -1, -1, 1, 1, -1, 1, 1, -1, -1, -1, 1, -1, -1, 1, -1, 1, -1, -1, 1, 1, -1, 1, 1, -1, -1, 1, 1, -1, 1 };
int8_t sigcode3[] = { 1, 1, -1, -1, -1, 1, -1, -1, 1, -1, 1, -1, -1, 1, 1, -1, 1, 1, -1, -1, 1, 1, -1, 1, 1, -1, 1, -1, 1, -1, -1, 1, -1, -1, 1, -1, 1, 1, -1, 1, -1, 1, -1, 1, -1, 1, 1, -1, -1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1, 1, -1, -1, 1, -1,
                      -1, 1, 1, -1, 1, 1, -1, -1, 1, -1, -1, 1, -1, 1, -1, 1, 1, -1, 1, -1, -1, 1, -1, -1, 1, 1, -1, 1, -1, -1, 1, 1, -1, 1, -1, 1, -1, 1, -1, 1, -1, 1, 1, -1, -1, 1, -1, -1, 1, -1, 1, -1, -1, 1, -1, -1, 1, 1, -1, 1, 1, -1, -1, 1 };  // 128 Zahlen from Roland
int8_t sigcode4[] = { 1, 1, 1, -1, -1, -1 };                                                                                                                                                                                                              //extend square test code


//***** END OF CONFIGURATION *****








//********************* Functions **********************************


//********************* Code for Whatsapp **********************************
void sendWhatsappMessage(String message){
  #ifdef WhatsApp_messages
    unsigned long currentMillis = millis();
    if (currentMillis-previousMillis >= interval){
      previousMillis = currentMillis;
      String API_URL = "https://api.whatabot.net/whatsapp/sendMessage?apikey=" + api_key + "&text=" + urlEncode(message) + "&phone=" + mobile_number;
      HTTPClient http;
      http.begin(API_URL);

      http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  
      int http_response_code = http.GET();
      if (http_response_code == 200){
        Serial.print("Whatsapp message sent successfully");
      }
      else{
        Serial.println("Error sending the message");
        Serial.print("HTTP response code: ");
        Serial.println(http_response_code);
      }

      http.end();
    }
  #endif
}
// End of Code for Whatsapp


//********************* SIGNAL MANAGEMENT **********************************
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  if (enableSenderA) {

    if (sigcode_norm[step] == 1) {
      digitalWrite(pinIN1, LOW);
      digitalWrite(pinIN2, HIGH);

    } else if (sigcode_norm[step] == -1) {
      digitalWrite(pinIN1, HIGH);
      digitalWrite(pinIN2, LOW);
    }
    step++;
    if (step == sigcode_size) {
      step = 0;
    }
  }
  if (enableSenderB) {

    if (sigcode_norm[step] == 1) {
      digitalWrite(pinIN3, LOW);
      digitalWrite(pinIN4, HIGH);

    } else if (sigcode_norm[step] == -1) {
      digitalWrite(pinIN3, HIGH);
      digitalWrite(pinIN4, LOW);
    }
    step++;
    if (step == sigcode_size) {
      step = 0;
    }
  }
  portEXIT_CRITICAL_ISR(&timerMux);
}
// End of Signalmanagement


//********************* CHANGE AREA **********************************
void changeArea(byte areaInMowing) {
  step = 0;
  enableSenderA = false;
  enableSenderB = false;
  #ifdef SerialOutput
    Serial.print("Change to Area:");
    Serial.println(areaInMowing);
  #endif

  #ifdef Screen
    u8x8.clear();
    u8x8.setCursor(0,0);  
    u8x8.print("Change Area:");
    u8x8.println(areaInMowing);
  #endif
  for (int uu = 0; uu <= 128; uu++) {  //clear the area
    sigcode_norm[uu] = 0;
  }
  sigcode_size = 0;
  switch (areaInMowing) {
    case 0:
      sigcode_size = sizeof sigcode0;
      for (int uu = 0; uu <= (sigcode_size - 1); uu++) {
        sigcode_norm[uu] = sigcode0[uu];
      }
      break;
    case 1:
      sigcode_size = sizeof sigcode1;
      for (int uu = 0; uu <= (sigcode_size - 1); uu++) {
        sigcode_norm[uu] = sigcode1[uu];
      }
      break;
    case 2:
      sigcode_size = sizeof sigcode2;
      for (int uu = 0; uu <= (sigcode_size - 1); uu++) {
        sigcode_norm[uu] = sigcode2[uu];
      }
      break;
    case 3:
      sigcode_size = sizeof sigcode3;
      for (int uu = 0; uu <= (sigcode_size - 1); uu++) {
        sigcode_norm[uu] = sigcode3[uu];
      }
      break;
    case 4:
      sigcode_size = sizeof sigcode4;
      for (int uu = 0; uu <= (sigcode_size - 1); uu++) {
        sigcode_norm[uu] = sigcode4[uu];
      }
      break;
  }
  #ifdef SerialOutput
    Serial.print("New sigcode in use  : ");
    Serial.println(sigCodeInUse);

    for (int uu = 0; uu <= (sigcode_size - 1); uu++) {
      Serial.print(sigcode_norm[uu]);
      Serial.print(",");
    }
    
    Serial.println();
    Serial.print("New sigcode size  : ");
    Serial.println(sigcode_size);
  #endif

  #ifdef Screen
    u8x8.setCursor(0,2);
    u8x8.print("sigCode in use:");
    u8x8.print(sigCodeInUse);
    Serial.println();
    Serial.print("New sigcode size: ");
    Serial.println(sigcode_size);
    delay(2000);
  #endif
}
// END ChangeArea


//********************* STATICSCREENPARTS **********************************
void StaticScreenParts() {
#ifdef Screen
  //line 0: Title
  u8x8.setCursor(0,0);
  u8x8.inverse();
  u8x8.print(" ESP32 Sender  ");
  u8x8.noInverse();

  //line 1: free
  u8x8.clearLine(1);

  //line 2: Sender ON/OFF
  u8x8.clearLine(2);

  //line 3: free
  u8x8.clearLine(3);

  //line 4: Worktime
  u8x8.setCursor(0, 4);
  u8x8.print("Worktime:");

  //line 5: Perimetercurrent
  u8x8.setCursor(0, 5);
  u8x8.print("Peri mA:");

  //line 6: Chargecurrent
  #ifdef USE_STATION
    u8x8.setCursor(0, 6);
    u8x8.print("Charge mA:");
  #endif

//line 7: Area
  u8x8.setCursor(0, 7);
  u8x8.print("Mowarea:");
#endif  
}
// END StaticScreenParts


//********************* SaveConfigCallback **********************************
void saveConfigCallback () {  //callback notifying us of the need to save config
  if (debug) Serial.println("Should save config");
  shouldSaveConfig = true;
}
// END SaveConfigCallback


//********************* SETUP **********************************
void setup() {
  Serial.begin(115200);                           // Serial interface start
  Wire.begin(I2C_SDA, I2C_SCL);                   // I2C interface start

  //reset settings - for testing
  //wm.resetSettings();
  //SPIFFS.format();

  InaPeri.init();                                 // initialize INA226 for perimetermeasuring 
  InaCharge.init();                               // initialize INA226 for chargemeasuring

  #ifdef Screen
  u8x8.begin();                                   // Screen start
  u8x8.setFont(u8x8_font_5x8_f);                  // Screen font 
  u8x8.clear();
  #endif
  timer = timerBegin(1000000);
  timerAttachInterrupt(timer, &onTimer);
  timerAlarm(timer, 104, true, 0);
  pinMode(pinIN1, OUTPUT);                        // Pinmodes
  pinMode(pinIN2, OUTPUT);
  pinMode(pinEnableA, OUTPUT);
  pinMode(pinIN3, OUTPUT);
  pinMode(pinIN4, OUTPUT);
  pinMode(pinEnableB, OUTPUT);
  pinMode(pinGreenLED, OUTPUT);                   // 2-color LED green
  pinMode(pinRedLED, OUTPUT);                     // 2-color LED red
  digitalWrite(pinGreenLED, LOW);
  digitalWrite(pinRedLED, LOW);
  
  #ifdef SerialOutput
    Serial.println("START");
    Serial.print("ESP32 Sender ");
    Serial.println(VER);
    Serial.print(" USE_PERI_CURRENT=");
    Serial.println(USE_PERI_CURRENT);
  #endif
  #ifdef Screen
    u8x8.println("START");
    u8x8.println("ESP32 Sender");
    u8x8.print("");u8x8.println(VER);
    u8x8.print("USE_PERI_CURR=");
    u8x8.println(USE_PERI_CURRENT);
    delay(2000);  
  #endif

  changeArea(sigCodeInUse);
  if (enableSenderA) {
    digitalWrite(pinEnableA, HIGH);
  }
  if (enableSenderB) {
    digitalWrite(pinEnableB, HIGH);
  }


  //------------------------  WIFI parts  ----------------------------------------
  WiFi.mode(WIFI_STA);
  if (SPIFFS.begin()) {   // read configuration from FS json
    if (SPIFFS.exists("/config.json")) {  //file exists, reading and loading
      File configFile = SPIFFS.open("/config.json", "r");   // open file
      if (configFile) {
        size_t size = configFile.size();  // allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);
        configFile.readBytes(buf.get(), size);
        DynamicJsonDocument json(1024);
        auto deserializeError = deserializeJson(json, buf.get());
        serializeJson(json, Serial);
        if ( ! deserializeError ) {
          if (json["ip"]) {
            if (debug) Serial.println("setting custom ip from config");
            strcpy(static_ip, json["ip"]);
            strcpy(static_gw, json["gateway"]);
            strcpy(static_sn, json["subnet"]);
          } else {
            if (debug) Serial.println("no custom ip in config");
          }
        } else {
          if (debug) Serial.println("failed to load json config");
        }
      }
    }
  } else {
    if (debug) Serial.println("failed to mount FS");
  }   // end read

  wm.setSaveConfigCallback(saveConfigCallback);    //set config save notify callback
  
  IPAddress _ip, _gw, _sn;    //set static ip
  _ip.fromString(static_ip);
  _gw.fromString(static_gw);
  _sn.fromString(static_sn);
  wm.setSTAStaticIPConfig(_ip, _gw, _sn, _gw); // ip, gateway, subnet, dns

  if (!wm.autoConnect("MowerAP","12345678")) { // The password should have at least 8 characters.
    if (debug) Serial.println("Failed to connect - ESP32 restart!");
    delay(3000);
    ESP.restart();
    delay(5000);
  } 
  else {
    if (debug) Serial.println("connected :)");
  }
  
  if (shouldSaveConfig) {   //save the custom parameters to FS
    if (debug) Serial.println("saving config");
    DynamicJsonDocument json(1024);
    json["ip"] = WiFi.localIP().toString();
    json["gateway"] = WiFi.gatewayIP().toString();
    json["subnet"] = WiFi.subnetMask().toString();
    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      if (debug) Serial.println("failed to open config file for writing");
    }
    serializeJson(json, Serial);
    serializeJson(json, configFile);
    configFile.close();
  }   //end save

  sendWhatsappMessage("ESP32 sender is now online.");          // Code for Whatsapp
  server.begin();


  //------------------------  SCREEN parts  ----------------------------------------
  #ifdef Screen            // 16x8 (??16x9??)
    u8x8.clear();
    u8x8.setCursor(0, 0);  // Set cursor position, start of line 0
    u8x8.print("DIY MOWER");  
    u8x8.setCursor(0, 1);  // Set cursor position, start of line 1
    u8x8.print("BB SENDER");
    u8x8.setCursor(0, 2);  // Set cursor position, start of line 2
    u8x8.print(VER);
    u8x8.setCursor(0, 3);  // Set cursor position, line 2 10th character
    u8x8.print("2 LOOPS");
    delay(3000);
  #endif
  #ifdef SerialOutput
    Serial.println("DIY MOWER");
    Serial.println("BB SENDER");
    Serial.println(VER);
    Serial.println("2 LOOPS");
    delay(3000);
  #endif


  //------------------------  current sensor parts  ----------------------------------------
  #ifdef SerialOutput
    Serial.println("Measuring voltage and current using INA226 ...");
  #endif

  InaPeri.setAverage(AVERAGE_4);
  InaPeri.setResistorRange(resistorPeri, rangePeri);
  InaCharge.setAverage(AVERAGE_4);
  InaCharge.setResistorRange(resistorCharge, rangeCharge);
  

  //------------------------  ArduinoOTA  ----------------------------------------
  #ifdef OTAUpdates
  ArduinoOTA.setHostname("ESP32-Sender 3.0");     // Hostname in WiFi
  // ArduinoOTA.setPassword("admin");             // Need a password?
ArduinoOTA.onStart([]() {
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
}
// END SETUP


//********************* LOOP **********************************
void loop() {

  #ifdef OTAUpdates
    ArduinoOTA.handle();
  #endif

  if (millis() >= nextTimeControl) {
    nextTimeControl = millis() + 10000;  //after debug can set this to 10 secondes

    #ifdef Screen
      StaticScreenParts();
    #endif

  
    if (USE_PERI_CURRENT) {
    
      PeriBusVoltage = InaPeri.getBusVoltage_V();
      PeriShuntVoltage = InaPeri.getShuntVoltage_mV();
      PeriCurrent = InaPeri.getCurrent_mA();

      PeriCurrent = PeriCurrent - 80.0;                        //the DC/DC, ESP32, LN298N drain between 80 and 100 mA when nothing is ON and a wifi access point is found (To confirm ????)

      if (PeriCurrent <= PERI_CURRENT_MIN) PeriCurrent = 0;

      if ((enableSenderA) && (PeriCurrent < PERI_CURRENT_MIN)) {
        workTimeMins = 0;
        #ifdef Screen
          u8x8.setCursor(0, 5);
          u8x8.inverse();
          u8x8.print("  Wire is Cut!  ");
          u8x8.noInverse();
        #endif
        #ifdef SerialOutput
          Serial.println("WIRE IS CUT!!!");
        #endif
        
      } else {
        #ifdef Screen
          u8x8.setCursor(8, 5);
          u8x8.print("        ");
          u8x8.setCursor(10, 5);
          u8x8.print(PeriCurrent);
        #endif
        #ifdef SerialOutput
          Serial.print("Pericurr ");
          Serial.println(PeriCurrent);
          Serial.print("PeriVoltage ");
          Serial.println(PeriBusVoltage);
        #endif
      }
    }

    if (workTimeMins >= WORKING_TIMEOUT_MINS) {
      // switch off perimeter
      enableSenderA = false;
      enableSenderB = false;
 
      workTimeMins = 0;
      WORKING_TIMEOUT = 1;
      digitalWrite(pinEnableA, LOW);
      digitalWrite(pinIN1, LOW);
      digitalWrite(pinIN2, LOW);

      digitalWrite(pinEnableB, LOW);
      digitalWrite(pinIN3, LOW);
      digitalWrite(pinIN4, LOW);

      sendWhatsappMessage("TIMEOUT! Mower didn't come back home! Perimeterwire switched off!");          // Code for Whatsapp
 
      Serial.println("********************************   Timeout, so stop Sender  **********************************");
    }
  }

  if (millis() >= nextTimeSec) {          // Do it every second
    nextTimeSec = millis() + 1000;

    #ifdef Screen
      u8x8.setCursor(9, 4);
      u8x8.print("       ");
      u8x8.setCursor(10, 4);
      u8x8.print(workTimeMins);
    
      //line 7: Area
      u8x8.setCursor(10, 7);
      u8x8.print("      ");
      u8x8.setCursor(10, 7);
      u8x8.print(sigCodeInUse);
    #endif
    #ifdef SerialOutput
      Serial.print("Area:");
      Serial.println(sigCodeInUse);
    #endif

    if (USE_STATION) {

      ChargeBusVoltage = InaCharge.getBusVoltage_V();
      ChargeShuntVoltage = InaCharge.getShuntVoltage_mV();
      ChargeCurrent = InaCharge.getCurrent_mA();

      if (ChargeCurrent > ChargeThreshold) {  //Just for charge LED
        digitalWrite(pinGreenLED, LOW);
        digitalWrite(pinRedLED, HIGH);
      } else  {
        digitalWrite(pinRedLED, LOW);
        digitalWrite(pinGreenLED, HIGH);
      }

      ChargeCurrentPrint = ChargeCurrent;                           // just a var to print on the screen
      if (ChargeCurrent < ChargeThreshold) ChargeCurrentPrint = 0;  // shows 0 when the mower is not charging

      #ifdef Screen
        u8x8.setCursor(10, 6);
        u8x8.print("      ");
        u8x8.setCursor(10, 6);
        u8x8.print(ChargeCurrentPrint);
      #endif
      #ifdef SerialOutput
        Serial.print("Charcurr: ");
        Serial.println(ChargeCurrentPrint);
      Serial.print("ChargeVolt: ");
      Serial.println(ChargeBusVoltage);
      #endif

      if (ChargeCurrent > PeriOnOffThreshold) {   // mower is into the station ,in my test 410 ma are drained so possible to stop sender - When ist fully loaded the current is about 4mA.
                                                  // So keep keep the value small to avoid an activation from the perimeterwire before the mower starts.
        enableSenderA = false;
        enableSenderB = false;

        if(mowerIsWorking == 1)  {
          mowerIsWorking = 0;
          WORKING_TIMEOUT = 0;
          sendWhatsappMessage("Mower is back at Home!");          // Code for Whatsapp
        }

        workTimeMins = 0;
        digitalWrite(pinEnableA, LOW);
        digitalWrite(pinIN1, LOW);
        digitalWrite(pinIN2, LOW);

        digitalWrite(pinEnableB, LOW);
        digitalWrite(pinIN3, LOW);
        digitalWrite(pinIN4, LOW);
        delay(200);
      } else {
        if (AUTO_START_SIGNAL && !WORKING_TIMEOUT ) {
          //always start to send a signal when mower leave station
           if(mowerIsWorking == 0)  {
            mowerIsWorking = 1;
            sendWhatsappMessage("Mower is going to work!");          // Code for Whatsapp
          }
          if (!enableSenderB) {
            enableSenderA = true;
            digitalWrite(pinEnableA, HIGH);
            digitalWrite(pinIN1, LOW);
            digitalWrite(pinIN2, LOW);
          } else {
            enableSenderB = true;
            digitalWrite(pinEnableB, HIGH);
            digitalWrite(pinIN3, LOW);
            digitalWrite(pinIN4, LOW);
          }
        }
      }
    }
    
     
    timeSeconds++;
    if (((enableSenderA) || (enableSenderB)) && (timeSeconds >= 60)) {                    // If Sender is ON & 60 seconds are left
      if (workTimeMins < 1440)  {                                                         // avoid overflow
        workTimeMins++;                                                                   // count up a minute
        timeSeconds = 0;                                                                  // set seconds back to 0
        if (workTimeChargeMins > 0) {                                                     // If workTimeCharge > 0 (last state was CHARGING)
          lastChargeMins = workTimeChargeMins;                                            // save this Time in lastChargeMins
          workTimeChargeMins = 0;                                                         // and set it back to 0
        }
      }
    } else  {
      if ((workTimeChargeMins < 1440) && (ChargeCurrent > ChargeThreshold) && (timeSeconds >= 60)) {   // If Chargecurrent bigger than ChargeThreshold (Mower is in Station and Charge)
      workTimeMins = 0;                                                                   // WorktimeMins reset
        workTimeChargeMins++;                                                             // count up the workTimeChargeMins
        timeSeconds = 0;                                                                  // seconds reset
      }
    }
    

    if ((enableSenderA) || (enableSenderB)) {

      #ifdef Screen
        u8x8.setCursor(0, 2);
        u8x8.print("Sender ON :     ");
      #endif
      #ifdef SerialOutput
        Serial.print("Sender ON : ");
      #endif

      if (enableSenderA && !enableSenderB) {
        #ifdef Screen
          u8x8.setCursor(10, 2);
          u8x8.print("A");
        #endif
        #ifdef SerialOutput
          Serial.print("A");
        #endif
      }
      if (enableSenderB && !enableSenderA) {
        #ifdef Screen
          u8x8.setCursor(10, 2);
          u8x8.print("B");
        #endif
        #ifdef SerialOutput
          Serial.print("B");
        #endif
      }
      if (enableSenderA && enableSenderB) {
        #ifdef Screen
          u8x8.setCursor(10, 2);
          u8x8.print("AB");
        #endif
        #ifdef SerialOutput
          Serial.print("AB");
        #endif
      }
    } else {
      workTimeMins = 0;       //New placed. It determinates the fault that Worktime is always 0
      #ifdef Screen
        u8x8.setCursor(0, 2);
        u8x8.print("Sender OFF      ");
      #endif
      #ifdef SerialOutput
        Serial.print("Sender OFF");
      #endif

    }
    #ifdef SerialOutput
      Serial.println("");
    #endif
  }

  // Check if a client has connected
  WiFiClient client = server.available();
  if (client) {
    unsigned long currentTime = millis();
    unsigned long previousTime = currentTime;
    unsigned long timeoutTime = 500;
    String req;
    String currentLine;
    Serial.println("New Client.");

    while (client.connected() && currentTime - previousTime <= timeoutTime) {
      currentTime = millis();
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        req += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {

            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();

            // Match the request
            if (req.indexOf("GET /A0") != -1) {
              enableSenderA = false;
              workTimeMins = 0;
              digitalWrite(pinEnableA, LOW);
              digitalWrite(pinIN1, LOW);
              digitalWrite(pinIN2, LOW);
              #ifdef SerialOutput
                Serial.println("Sender A0 = off");
              #endif
            }
            if (req.indexOf("GET /B0") != -1) {
              enableSenderB = false;
              workTimeMins = 0;
              digitalWrite(pinEnableB, LOW);
              digitalWrite(pinIN3, LOW);
              digitalWrite(pinIN4, LOW);
              #ifdef SerialOutput
                Serial.println("Sender B0 = off");
              #endif
            }

            if (req.indexOf("GET /A1") != -1) {
              workTimeMins = 0;
              enableSenderA = true;
              digitalWrite(pinEnableA, HIGH);
              digitalWrite(pinIN1, LOW);
              digitalWrite(pinIN2, LOW);
              #ifdef SerialOutput
                Serial.println("Sender A1 = on");
              #endif
            }

            if (req.indexOf("GET /B1") != -1) {
              workTimeMins = 0;
              enableSenderB = true;
              digitalWrite(pinEnableB, HIGH);
              digitalWrite(pinIN3, LOW);
              digitalWrite(pinIN4, LOW); 
              #ifdef SerialOutput
                Serial.println("Sender B1 = on");
              #endif
            }

            if (req.indexOf("GET /sigCode0") != -1) {
              sigCodeInUse = 0;
              changeArea(sigCodeInUse);
              #ifdef SerialOutput
                Serial.println("Signalcode = 0");
              #endif
            }

            if (req.indexOf("GET /sigCode1") != -1) {
              sigCodeInUse = 1;
              changeArea(sigCodeInUse);
              #ifdef SerialOutput
                Serial.println("Signalcode = 1");
              #endif
            }

            if (req.indexOf("GET /sigCode2") != -1) {
              sigCodeInUse = 2;
              changeArea(sigCodeInUse);
              #ifdef SerialOutput
                Serial.println("Signalcode = 2");
              #endif
            }

            if (req.indexOf("GET /sigCode3") != -1) {
              sigCodeInUse = 3;
              changeArea(sigCodeInUse);
              #ifdef SerialOutput
                Serial.println("Signalcode = 3");
              #endif
            }

            if (req.indexOf("GET /sigCode4") != -1) {
              sigCodeInUse = 4;
              changeArea(sigCodeInUse);
              #ifdef SerialOutput
                Serial.println("Signalcode = 4");
              #endif
            }

            if (req.indexOf("GET /sigDuration104") != -1) {
              sigDuration = 104;
              timerAlarm(timer, 104, true, 0);
              #ifdef SerialOutput
                Serial.println("Signalduration = 104");
              #endif
            }

            if (req.indexOf("GET /sigDuration50") != -1) {
              sigDuration = 50;
              timerAlarm(timer, 50, true, 0);
              #ifdef SerialOutput
                Serial.println("Signalduration = 50");
              #endif
            }

            if (req.indexOf("GET /AutoMode0") != -1) {
              AUTO_START_SIGNAL = 0;
              #ifdef SerialOutput
                Serial.println("Automode = 0");
              #endif
            }
    
            if (req.indexOf("GET /AutoMode1") != -1) {
              AUTO_START_SIGNAL = 1;
              #ifdef SerialOutput
                Serial.println("Automode = 1");
              #endif
            }

            // Display the HTMl site
            client.print("<html><head><title>ESP32 Sender Controlpanel v2.0</title></head><meta http-equiv='refresh' content='5; url=http://");
            client.print(static_ip);
            client.println("'><style type='text/css'>body{padding-left: 0em;font-family: Helvetica, Geneva, Arial, SunSans-Regular, sans-serif;color: rgb(125, 0, 0);background-color: rgb(170, 210, 255);}table.fullscreen{border: 1px solid rgb(125, 0, 0);}td.on{border-right: 1em solid rgb(0, 255, 0);}td.off{border-right: 1em solid rgb(255, 0, 0);}h1{font-family: Helvetica, Geneva, Arial, SunSans-Regular, sans-serif;}</style><body>");
            client.println("<table class='fullscreen'; width='100%'>");
            client.println("<tr>");
            client.println("<td><h1 align='center'>ESP32 Sender Controlpanel</h1></td>");
            client.println("</tr>");
            client.println("<table align='center'>");
            client.println("<tr>");
            client.println("<td>Chargevoltage:</td><td>&nbsp;</td>");
            client.print("<td>");
            client.print(ChargeBusVoltage);
            client.println("V</td>");
            client.println("</tr>");
            client.println("<tr>");
            client.println("<td>Chargecurrent:</td><td>&nbsp;</td>");
            client.print("<td>");
            client.print(ChargeCurrentPrint);
            client.println("mA</td>");
            client.println("</tr>");
            client.println("<tr>");
            client.println("<td>Perimetervoltage:</td><td>&nbsp;</td>");
            client.print("<td>");
            client.print(PeriBusVoltage);
            client.println("V</td>");
            client.println("</tr>");
            client.println("<tr>");
            client.println("<td>Perimetercurrent:</td>");
            client.println("<td>&nbsp;</td>");
            client.print("<td>");
            client.print(PeriCurrent);
            client.println("mA</td>");
            client.println("</tr>");
            client.println("<tr>");
            client.println("<td>Perimeterloop Uptime:</td>");
            client.println("<td>&nbsp;</td>");
            client.print("<td>");
            client.print(workTimeMins);
            client.println("min.</td>");
            client.println("</tr>");
            client.println("<tr>");
            client.println("<td>Signalcode:</td>");
            client.println("<td>&nbsp;</td>");
            if (sigCodeInUse == 4) {
              Serial.println("SigCodeInUse = 4 > 0");
              linktext = "/sigCode0";
            } else if (sigCodeInUse == 3) {
              Serial.println("SigCodeInUse = 3 > 4");
                linktext = "/sigCode4";
            } else if (sigCodeInUse == 2) {
                Serial.println("SigCodeInUse= 2 > 3");
                linktext = "/sigCode3";
            } else if (sigCodeInUse == 1) {
                Serial.println("SigCodeInUse= 1 > 2");
                linktext = "/sigCode2";
            } else if (sigCodeInUse == 0) {
                Serial.println("SigCodeInUse= 0 > 1");
                linktext = "/sigCode1";
            }
            client.print("<td><a href='");
            client.print(linktext);
            client.print("'>");
            client.print(sigCodeInUse);
            client.println("</a></td>");
            client.println("</tr>");
            client.println("<tr>");
            client.println("<td>Sendingspeed:</td>");
            client.println("<td>&nbsp;</td>");
            
            if (sigDuration == 104) {
              linktext = "/sigDuration50";
            } else {
              linktext = "/sigDuration104";
            }
            client.print("<td><a href='");
            client.print(linktext);
            client.print("'>");
            client.print(sigDuration);
            client.println("us/bit</a></td>");
            client.println("</tr>");
            client.println("<tr>");
            client.println("<td>Sender A:</td>");

              if(enableSenderA == true) {
              enableSenderAprint = "on";
              linktext = "/A0";
            } else if (enableSenderA == false) {
              enableSenderAprint = "off";
              linktext = "/A1";
            }

            client.print("<td class='");
            client.print(enableSenderAprint);
            client.println("'>&nbsp;</td>");
            client.print("<td><a href='");
            client.print(linktext);
            client.print("'>");
            client.print(enableSenderAprint);
            client.println("</a></td>");
            client.println("</tr>");
            client.println("<tr>");
            
            if(enableSenderB == 1) {
              enableSenderBprint = "on";
              linktext = "/B0";
            } else {
              enableSenderBprint = "off";
              linktext = "/B1";
            }

            client.print("<td>Sender B:</td><td class='");
            client.print(enableSenderBprint);
            client.print("'>&nbsp;</td>");
            client.print("<td><a href='");
            client.print(linktext);
            client.print("'>");
            client.print(enableSenderBprint);
            client.println("</a></td>");
            client.println("</tr>");
            client.println("<tr>");
            client.println("<td>Automaticmode:</td>");
            
            if(AUTO_START_SIGNAL == 1) {
              AutoStartSignalPrint = "on";
              linktext = "/AutoMode0";
            } else {
              AutoStartSignalPrint = "off";
              linktext = "/AutoMode1";
            }

            client.print("<td class='");
            client.print(AutoStartSignalPrint);
            client.print("'>&nbsp;</td>");
            client.print("<td><a href='");
            client.print(linktext);
            client.print("'>");
            client.print(AutoStartSignalPrint);
            client.println("</a></td>");
            client.println("</tr>");
            client.println("</table>");
            client.println("</table>");
            client.println("</body>");
            client.println("</html>");

            // Variablen
            // AUTO_START_SIGNAL
            // workTimeChargeMins;
            // lastChargeMins;
            // workTimeMins;
            // PeriCurrent;
            // PeriBusVoltage;
            // PeriShuntVoltage;
            // ChargeCurrentPrint;
            // ChargeBusVoltage;
            // ChargeShuntVoltage;
            // sigDuration;
            // sigCodeInUse;
            // enableSenderA;
            // enableSenderB;
            client.flush();
          }
        }
      }
    }
  }  

}
//END LOOP
