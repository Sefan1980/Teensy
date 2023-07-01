/*
  WIFI Communicating sender with 2 possible loops
  Adjust IP according to your ESP32 value 10.0.0.150 in this example
  On your browser send :
  http://10.0.0.150/A0  *************** to stop the sender on wire connected on output A
  http://10.0.0.150/A1  *************** to start the sender on wire connected on output A
  http://10.0.0.150/B0  *************** to stop the sender on wire connected on output B
  http://10.0.0.150/B1  *************** to start the sender on wire connected on output B

  http://10.0.0.150/sigCode/2  ******** to change the sigcode in use possible value are 0,1,2,3,4 ,see sigcode list
  http://10.0.0.150/?  **************** to see the state of the sender
  http://10.0.0.150/sigDuration/104  ** to change the speed sender to 104 microsecondes
  http://10.0.0.150/sigDuration/50  *** to change the speed sender to 50 microsecondes

  If USE_STATION : the sender start and stop automaticly if the mower is in the station or not

  ------COLLABORATION FROM BERNARD, SASCHA AND STEFAN------
*/

//********************* defines **********************************
#define OTAUpdates 1                  // OTA Updates
#define USE_STATION 1                 // a station is connected and is used to charge the mower
#define USE_PERI_CURRENT 1            // use Feedback for perimeter current measurements? (set to '0' if not connected!)
#define USE_BUTTON 0                  // use button to start mowing or send mower to station not finish to dev
#define WORKING_TIMEOUT_MINS 300      // timeout for perimeter switch-off if robot not in station (minutes)
#define PERI_CURRENT_MIN 200          // minimum milliAmpere for cutting wire detection
#define AUTO_START_SIGNAL 1           // use to start sender when mower leave station
#define I2C_SDA 21                    // SDA pin
#define I2C_SCL 22                    // SCL pin
#define SerialOutput 1                // Show Serial Textmessages for debugging
#define Screen 1                      // Screen or not?
#define pinIN1 12                     // M1_IN1  ESP32 GPIO12       ( connect this pin to L298N-IN1)
#define pinIN2 13                     // M1_IN2  ESP32 GPIO13       ( connect this pin to L298N-IN2)
#define pinEnableA 23                 // ENA    ESP32 GPIO23         (connect this pin to L298N-ENA)
#define pinIN3 14                     // M1_IN3  ESP32 GPIO14       ( connect this pin to L298N-IN3)
#define pinIN4 18                     // M1_IN4  ESP32 GPIO18       ( connect this pin to L298N-IN4)
#define pinEnableB 19                 // ENB    ESP32 GPIO19        (connect this pin to L298N-ENA)
#define pinDoorOpen 34                // Not in use (Magnetic switch)
#define pinDoorClose 35               // Not in use (Magnetic switch)
#define pinLDR 32                     // Not in use (Light Sensor)
#define pinGreenLED 25                // Not in use
#define pinRedLED 26                  // Not in use
#define VER "ESP32Sender 01.07.23"    // code version

//********************* includes **********************************
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>
#include <INA226_WE.h>                // INA226_WE library from Wolfgang Ewald
#ifdef OTAUpdates
  #include <ESPmDNS.h>
  #include <WiFiUdp.h>
  #include <ArduinoOTA.h>             // ArduinoOTA from Juraj Andrassy
#endif
#include <U8x8lib.h>                  // U8g2 from Oliver Kraus

//********************* Display Settings **********************************
// Please UNCOMMENT one of the contructor lines below
// U8x8 Contructor List 
// The complete list is available here: https://github.com/olikraus/u8g2/wiki/u8x8setupcpp
// Please update the pin numbers according to your setup. Use U8X8_PIN_NONE if the reset pin is not connected
//U8X8_NULL u8x8;	// null device, a 8x8 pixel display which does nothing
//U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE); 	      
//U8X8_SSD1306_128X64_ALT0_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE); 	      // same as the NONAME variant, but may solve the "every 2nd line skipped" problem
U8X8_SH1106_128X64_NONAME_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE);
// End of constructor list

//********************* WLAN Settings **********************************
const char* ssid = "Your SSID";                         // put here your acces point ssid -->  If you use the station an an Accesspoint, put the SSID for your AP here. The SSID for AP should not match an existing network.
const char* password = "Your password";                 // put here the password - min. 7 chars --> If you use the station as an Accesspoint, put the AP-password here.
IPAddress staticIP(192, 168, 178, 222);                 // put here the static IP --> Used for AP too!
IPAddress gateway(192, 168, 178, 1);                    // put here the gateway (IP of your router)
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(192, 168, 178, 1);                        // put here the dns (IP of your router)
WiFiServer server(80);
bool APconnected = false;

//********************* INA226 Settings **********************************
INA226_WE InaPeri = INA226_WE(0x40);                    // 0x40 = without bridge
INA226_WE InaCharge = INA226_WE(0x44);                  // 0x44 = bridge between A1 and VSS
float resistorPeri = 0.02;                              // for 10mOhm resistor don't try 0.01, try 0.02. For 100mOhm resistor use 0.1 
float resistorCharge = 0.02;
float rangePeri = 4.0;                                  // Range for 10mOhm Resistor: try 8.0 od 4.0 - For 100mOhm use 0.8
float rangeCharge = 4.0;

//********************* other **********************************
int column = 2;                                         //used in function scanNetwork. It's for the Screen. (Print SSID for each network found)
bool firstStart = true;
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

    } else {
      #ifdef SerialOutput
        Serial.println("ERROR");
      #endif      

      #ifdef Screen
        u8x8.clear();
        u8x8.setCursor(5,5);
        u8x8.print("ERROR");      
        delay(5000);
      #endif
      //digitalWrite(pinEnableA, LOW);
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

    } else {
        #ifdef SerialOutput
        Serial.println("ERROR");
      #endif

      #ifdef Screen
        u8x8.clear();
        u8x8.setCursor(5,5);
        u8x8.print("ERROR");
        delay(5000);
      #endif
      //digitalWrite(pinEnableA, LOW);
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
  #endif

  #ifdef Screen
    u8x8.setCursor(0,2);
    u8x8.print("sigCode in use:");
    u8x8.print(sigCodeInUse);
  #endif
  for (int uu = 0; uu <= (sigcode_size - 1); uu++) {
    #ifdef SerialOutput
      Serial.print(sigcode_norm[uu]);
      Serial.print(",");
    #endif
  

  }
  #ifdef SerialOutput
    Serial.println();
    Serial.print("New sigcode size  : ");
    Serial.println(sigcode_size);
  #endif
  
  #ifdef Screen
    u8x8.setCursor(0,4);
    u8x8.print("sigcode size:");
    u8x8.print(sigcode_size);
    delay(2000);
  #endif
}
// END ChangeArea

//********************* CONNECTION **********************************
void connection() {
  #ifdef SerialOutput
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);
  #endif

  #ifdef Screen
    u8x8.clearDisplay();
    u8x8.setCursor(0, 0);
    u8x8.println("Connecting to:");
    u8x8.print(ssid);
    delay(1500);
  #endif

  WiFi.begin(ssid, password);
  for (int i = 0; i < 60; i++) {
    if (WiFi.status() != WL_CONNECTED) {

      #ifdef SerialOutput
        Serial.println("Try connecting");
      #endif
      
      #ifdef Screen
        u8x8.setCursor(0,3);
        u8x8.print("CONNECTING");
      #endif      
      delay(500);
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    #ifdef SerialOutput
      Serial.println("WiFi connected.");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
    #endif    
    
    #ifdef Screen
      u8x8.clear();
      u8x8.setCursor(0,0);
      u8x8.print("WiFi Connected");
      u8x8.setCursor(0,2);
      u8x8.print("IP address:");
      u8x8.setCursor(0,4);
      u8x8.print(WiFi.localIP());
      delay(1000);
    #endif
   
     server.begin();
  }
}
// END Connection


//********************* openAP **********************************
static void openAP()  {

  #ifdef SerialOutput
  Serial.print("...open Accesspoint.");
  #endif

  WiFi.softAPConfig (staticIP, gateway, subnet);
  if (!WiFi.softAP(ssid, password)) {
    Serial.println("AP creation failed.");
    while(1);
  }
  IPAddress myIP = WiFi.softAPIP();
  
  #ifdef SerialOutput
  Serial.println("...Access!");
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  #endif

  #ifdef Screen
    u8x8.clear();
    u8x8.setCursor(0, 1);
    u8x8.print("AP-IP-addr:");
    u8x8.setCursor(0, 3);
    u8x8.print(myIP);
  #endif

  server.begin();
  
  #ifdef SerialOutput
    Serial.println("Server started");
  #endif

  #ifdef Screen
  u8x8.setCursor(0, 5);
  u8x8.print("Server started");
  #endif
  APconnected = true;
  delay(3000);
}  
// END openAP


//********************* SCANNETWORK **********************************
static void ScanNetwork() {

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  bool findNetwork = false;
  #ifdef Screen
    u8x8.clear();
    u8x8.setCursor(0, 0);
    if (!firstStart) {
      u8x8.println("Hotspot Lost!");
    }
    u8x8.print("Scan Network");
  #endif

  #ifdef SerialOutput
    if (!firstStart) {
      Serial.println("Hotspot Lost");
    }
    Serial.println("Scan Network");
  #endif
  
  firstStart = false;
  delay(1000);  // wait until all is disconnect
  int n = WiFi.scanNetworks();
  if (n == -1) {
    
    #ifdef Screen
      u8x8.setCursor(0, 1);
      u8x8.print("Scan running...");
      u8x8.setCursor(0, 2);
      u8x8.print("Need Reset? ");
      u8x8.setCursor(0, 3);
      u8x8.print("If sender is OFF");
    #endif
    #ifdef SerialOutput
      Serial.println("Scan running...");
      Serial.println("Need reset?");
      Serial.println("If sender is OFF");
    #endif
        
    delay(2000);
    if ((!enableSenderA) && (!enableSenderB)) ESP.restart();  // do not reset if sender is ON
  }
  if (n == -2)    //bug in esp32 if wifi is lost many time the esp32 fail to autoreconnect,maybe solve in other firmware ???????
  {
    
    #ifdef Screen
      u8x8.setCursor(0, 1);
      u8x8.print("Scan Fail.");
      u8x8.setCursor(0, 2);
      u8x8.print("Need Reset? ");
      u8x8.setCursor(0, 3);
      u8x8.print("If sender is Off");
    #endif
    #ifdef SerialOutput
      Serial.println("Scan fail.");
      Serial.println("Need reset?");
      Serial.println("If sender is OFF");
    #endif
        
    delay(3000);
    if ((!enableSenderA) && (!enableSenderB)) ESP.restart();
  }
  if (n == 0) {

    #ifdef Screen
      u8x8.setCursor(0, 2);
      u8x8.print("No networks.");
    #endif
    #ifdef SerialOutput
      Serial.println("No networks.");
    #endif
  }

  if (n > 0) {
    
    #ifdef SerialOutput
      Serial.print("Find ");
      Serial.println(n);
    #endif

    #ifdef Screen
      u8x8.clearDisplay();
      u8x8.setCursor(0, 0);
      u8x8.print("Find ");u8x8.println(n);u8x8.println(" ");
    #endif
    
    delay(1000);

    for (int i = 0; i < n; ++i) {   // Print SSID for each network found
      char currentSSID[64];
      char printCurrentSSID[16];
      WiFi.SSID(i).toCharArray(currentSSID, 64);
      WiFi.SSID(i).toCharArray(printCurrentSSID, 16);
      #ifdef SerialOutput
        Serial.print("Find Wifi : ");
        Serial.println(currentSSID);
      #endif      
      
      #ifdef Screen
        if(column >= 8) {
          column = 2;
          delay(2000);
          u8x8.clearDisplay();
          u8x8.setCursor(0, 0);
          u8x8.print("Find ");u8x8.println(n);u8x8.println(" ");
        }
        u8x8.setCursor(0, column);
        u8x8.print(printCurrentSSID);
        delay(500);
        column++;
      #endif

      if (String(currentSSID) == ssid) {
        findNetwork = true;
        //i = 200;  //to avoid loop again when connected
      }
    }
    delay(2000);
    if (findNetwork == true)  {
      connection();
    } else
    {
      openAP();
    }
  }
}
// END ScanNetwork

//********************* STATICSCREENPARTS **********************************
void StaticScreenParts() {
#ifdef Screen
  //line 0: Title
  u8x8.setCursor(0,0);
  u8x8.inverse();
  u8x8.print(" Teensy Sender  ");
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

//********************* SETUP **********************************
void setup() {
  Serial.begin(115200);                           // Serial interface start
  Wire.begin(I2C_SDA, I2C_SCL);                   // I2C interface start

  InaPeri.init();                                 // initialize INA226 for perimetermeasuring 
  InaCharge.init();                               // initialize INA226 for chargemeasuring

  u8x8.begin();                                   // Screen start
  u8x8.setFont(u8x8_font_5x8_f);                  // Screen font 
  u8x8.clear();
  timer = timerBegin(0, 80, true);                // Interrupt things
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 104, true);
  timerAlarmEnable(timer);
  pinMode(pinIN1, OUTPUT);                        // Pinmodes
  pinMode(pinIN2, OUTPUT);
  pinMode(pinEnableA, OUTPUT);
  pinMode(pinIN3, OUTPUT);
  pinMode(pinIN4, OUTPUT);
  pinMode(pinEnableB, OUTPUT);

  #ifdef SerialOutput
    Serial.println("START");
    Serial.print("Teensymower Sender ");
    Serial.println(VER);
    Serial.print(" USE_PERI_CURRENT=");
    Serial.println(USE_PERI_CURRENT);
  #endif
  #ifdef Screen
    u8x8.println("START");
    u8x8.println("Teensy Sender");
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
  // Set WiFi to station mode and disconnect from an AP if it was previously connected
  if (WiFi.config(staticIP, gateway, subnet, dns, dns) == false) {
    
    #ifdef SerialOutput
      Serial.println("WIFI Configuration failed.");
    #endif
  }  
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  if ((WiFi.status() != WL_CONNECTED)) ScanNetwork();

  //------------------------  SCREEN parts  ----------------------------------------
  #ifdef Screen            // 16x8 (??16x9??)
    u8x8.clear();
    u8x8.setCursor(0, 0);  // Set cursor position, start of line 0
    u8x8.print("TEENSYMOWER");  
    u8x8.setCursor(0, 1);  // Set cursor position, start of line 1
    u8x8.print("BB SENDER");
    u8x8.setCursor(0, 2);  // Set cursor position, start of line 2
    u8x8.print(VER);
    u8x8.setCursor(0, 3);  // Set cursor position, line 2 10th character
    u8x8.print("2 LOOPS");
    delay(3000);
  #endif
  #ifdef SerialOutput
    Serial.println("TEENSYMOWER");
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
  InaPeri.waitUntilConversionCompleted();
  InaCharge.setAverage(AVERAGE_4);
  InaCharge.setResistorRange(resistorCharge, rangeCharge);
  InaCharge.waitUntilConversionCompleted();

  //------------------------  ArduinoOTA  ----------------------------------------
 #ifdef OTAUpdates
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

    if ((WiFi.status() != WL_CONNECTED && APconnected == false)) ScanNetwork();

    if (workTimeMins >= WORKING_TIMEOUT_MINS) {
      // switch off perimeter
      enableSenderA = false;
      enableSenderB = false;

      workTimeMins = 0;
      digitalWrite(pinEnableA, LOW);
      digitalWrite(pinIN1, LOW);
      digitalWrite(pinIN2, LOW);

      digitalWrite(pinEnableB, LOW);
      digitalWrite(pinIN3, LOW);
      digitalWrite(pinIN4, LOW);

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

      ChargeCurrentPrint = ChargeCurrent;                       // just a var to print on the screen
      if (ChargeCurrent < 10) ChargeCurrentPrint = 0;           // shows 0 when the mower is not charging

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

      if (ChargeCurrent > 5) {  // mower is into the station ,in my test 410 ma are drained so possible to stop sender - When ist fully loaded the current is about 6mA.
                                // So keep it small, to avoid an activation from the perimeterwire before the mower starts.
        enableSenderA = false;
        enableSenderB = false;

        workTimeMins = 0;
        digitalWrite(pinEnableA, LOW);
        digitalWrite(pinIN1, LOW);
        digitalWrite(pinIN2, LOW);

        digitalWrite(pinEnableB, LOW);
        digitalWrite(pinIN3, LOW);
        digitalWrite(pinIN4, LOW);

      } else {
        if (AUTO_START_SIGNAL) {
          //always start to send a signal when mower leave station
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
    /*
    timeSeconds++;
    if (((enableSenderA) || (enableSenderB)) && (timeSeconds >= 60)) {
      if (workTimeMins < 1440) workTimeMins++;
      timeSeconds = 0;
    }
    */
    
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
      if ((workTimeChargeMins < 1440) && (ChargeCurrent > 10) && (timeSeconds >= 60)) {   // If Chargecurrent bigger than 10mA (Mower is in Station and Charge)
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
  
    // Read the first line of the request
    String req = client.readStringUntil('\r');
    if (req == "") return;
    #ifdef SerialOutput
      Serial.print("Client say  ");
      Serial.println(req);
      Serial.println("------------------------ - ");
    #endif
    
    // Match the request
    if (req.indexOf("GET /A0") != -1) {
      enableSenderA = false;
      workTimeMins = 0;
      digitalWrite(pinEnableA, LOW);
      digitalWrite(pinIN1, LOW);
      digitalWrite(pinIN2, LOW);
      String sResponse;
      sResponse = "SENDER A IS OFF";
  
      // Send the response to the client
      #ifdef SerialOutput
        Serial.println(sResponse);
      #endif
      client.print(sResponse);
      client.flush();
    }
    if (req.indexOf("GET /B0") != -1) {
      enableSenderB = false;
      workTimeMins = 0;
      digitalWrite(pinEnableB, LOW);
      digitalWrite(pinIN3, LOW);
      digitalWrite(pinIN4, LOW);
      String sResponse;
      sResponse = "SENDER B IS OFF";
  
      // Send the response to the client
      #ifdef SerialOutput
        Serial.println(sResponse);
      #endif
      client.print(sResponse);
      client.flush();
    }
    if (req.indexOf("GET /A1") != -1) {
      workTimeMins = 0;
      enableSenderA = true;
      digitalWrite(pinEnableA, HIGH);
      digitalWrite(pinIN1, LOW);
      digitalWrite(pinIN2, LOW);
 
      // Prepare the response
      String sResponse;
      sResponse = "SENDER A IS ON";
 
      // Send the response to the client
      #ifdef SerialOutput
        Serial.println(sResponse);
      #endif
      client.print(sResponse);
      client.flush();
    }
    if (req.indexOf("GET /B1") != -1) {
      workTimeMins = 0;
      enableSenderB = true;
      digitalWrite(pinEnableB, HIGH);
      digitalWrite(pinIN3, LOW);
      digitalWrite(pinIN4, LOW);
 
      // Prepare the response
      String sResponse;
      sResponse = "SENDER B IS ON";
 
      // Send the response to the client
      #ifdef SerialOutput
        Serial.println(sResponse);
      #endif
      client.print(sResponse);
      client.flush();
    }

    if (req.indexOf("GET /?") != -1) {
      String sResponse, sHeader;
      sResponse = "<html><head><title>TeensySender</title><META HTTP-EQUIV='refresh' CONTENT='5'></head><body><H3>MAC ADRESS = ";
      sResponse += WiFi.macAddress();
      sResponse += "<BR>Current chargetime: ";
      sResponse += workTimeChargeMins;
      sResponse += "min.<BR>Last charge : ";
      sResponse += lastChargeMins;
      sResponse += "min.<BR>Uptime perimeterloop = ";
      sResponse += workTimeMins;
      sResponse += "min.<br>Current flow in the perimeter loop = ";
      sResponse += PeriCurrent;
      sResponse += "mA<br>Voltage in the perimeterloop = ";
      sResponse += PeriBusVoltage;

//       sResponse += "V<br>PeriShuntVoltage= ";      // Shows the PeriShuntVoltage
//       sResponse += PeriShuntVoltage;
//       sResponse += "m";

      sResponse += "V<br>Chargecurrent = ";
      sResponse += ChargeCurrentPrint;
      sResponse += "mA<br>Chargevoltage = ";
      sResponse += ChargeBusVoltage;

//       sResponse += "V<br>ChargeShuntVoltage= ";      // Shows the ChargeShuntVoltage
//       sResponse += ChargeShuntVoltage;
//       sResponse += "m";

      sResponse += "V<br>Sends one bit of the signal every :";
      sResponse += sigDuration;
      sResponse += "us<br>Signalcode = ";
      sResponse += sigCodeInUse;
      sResponse += "<br>Sender A : ";
      sResponse += enableSenderA;
      sResponse += "<br>Sender B : ";
      sResponse += enableSenderB;
      sResponse += "</H3></body></html>";

      #ifdef SerialOutput
        Serial.println(sResponse);
      #endif
      client.print(sResponse);
      client.flush();
    }

    if (req.indexOf("GET /sigCode/0") != -1) {
      sigCodeInUse = 0;
      changeArea(sigCodeInUse);
 
      // Prepare the response
      String sResponse;
      sResponse = "NOW Send Signal 0";
 
      // Send the response to the client
      #ifdef SerialOutput
        Serial.println(sResponse);
      #endif
      client.print(sResponse);
      client.flush();
    }
    if (req.indexOf("GET /sigCode/1") != -1) {
      sigCodeInUse = 1;
      changeArea(sigCodeInUse);
 
      // Prepare the response
      String sResponse;
      sResponse = "NOW Send Signal 1";
 
      // Send the response to the client
      #ifdef SerialOutput
        Serial.println(sResponse);
      #endif
      client.print(sResponse);
      client.flush();
    }

    if (req.indexOf("GET /sigCode/2") != -1) {
      sigCodeInUse = 2;
      changeArea(sigCodeInUse);
 
      // Prepare the response
      String sResponse;
      sResponse = "NOW Send Signal 2";
 
      // Send the response to the client
      #ifdef SerialOutput
        Serial.println(sResponse);
      #endif
      client.print(sResponse);
      client.flush();
    }

    if (req.indexOf("GET /sigCode/3") != -1) {
      sigCodeInUse = 3;
      changeArea(sigCodeInUse);
 
      // Prepare the response
      String sResponse;
      sResponse = "NOW Send Signal 3";
 
      // Send the response to the client
      #ifdef SerialOutput
        Serial.println(sResponse);
      #endif
      client.print(sResponse);
      client.flush();
    }

    if (req.indexOf("GET /sigCode/4") != -1) {
      sigCodeInUse = 4;
      changeArea(sigCodeInUse);
 
      // Prepare the response
      String sResponse;
      sResponse = "NOW Send Signal 4";
 
      // Send the response to the client
      #ifdef SerialOutput
        Serial.println(sResponse);
      #endif
      client.print(sResponse);
      client.flush();
    }
    if (req.indexOf("GET /sigDuration/104") != -1) {
      sigDuration = 104;
      timerAlarmWrite(timer, 104, true);
 
      // Prepare the response
      String sResponse;
      sResponse = "NOW 104 microsecond signal duration";
 
      // Send the response to the client
      #ifdef SerialOutput
        Serial.println(sResponse);
      #endif
      client.print(sResponse);
      client.flush();
    }

    if (req.indexOf("GET /sigDuration/50") != -1) {
      sigDuration = 50;
      timerAlarmWrite(timer, 50, true);
 
      // Prepare the response
      String sResponse;
      sResponse = "NOW 50 microsecond signal duration";
 
      // Send the response to the client
      #ifdef SerialOutput
        Serial.println(sResponse);
      #endif
      client.print(sResponse);
      client.flush();
    }
  }
}
//END LOOP
