/*
  WIFI Communicating sender with 2 possible loop
  Adjust IP according to your ESP32 value 10.0.0.150 in this example
  On your browser send :
  http://10.0.0.150/A0   *********** to stop the sender on wire connected on output A
  http://10.0.0.150/A1   *********** to start the sender on wire connected on output A
  http://10.0.0.150/B0   *********** to stop the sender on wire connected on output B
  http://10.0.0.150/B1   *********** to start the sender on wire connected on output B

  http://10.0.0.150/sigCode/2 ******* to change the sigcode in use possible value are 0,1,2,3,4 ,see sigcode list
  http://10.0.0.150/?   *********** to see the state of the sender
  http://10.0.0.150/sigDuration/104    *********** to change the speed sender to 104 microsecondes
  http://10.0.0.150/sigDuration/50    *********** to change the speed sender to 50 microsecondes

  If USE_STATION : the sender start and stop automaticly if the mower is in the station or not


*/
#include <Wire.h>
#include <WiFi.h>
#include "INA226_WE.h"
#include <U8x8lib.h>

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
const char* ssid = "Sefan WLAN 2,4GHz";          // put here your acces point ssid
const char* password = "StefanJaninaRenkaDeik";  // put here the password
IPAddress staticIP(192,168,178,222);  // put here the static IP
IPAddress gateway(192,168,178,1);     // put here the gateway (IP of your routeur)
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(192,168,178,1);  // put here one dns (IP of your routeur)

//********************* defines **********************************
#define USE_STATION 1             // a station is connected and is used to charge the mower
#define USE_PERI_CURRENT 1        // use Feedback for perimeter current measurements? (set to '0' if not connected!)
#define USE_BUTTON 0              // use button to start mowing or send mower to station not finish to dev
#define USE_RAINFLOW 0            // check the amount of rain not finish to dev on 31/08/2020
#define WORKING_TIMEOUT_MINS 300  // timeout for perimeter switch-off if robot not in station (minutes)
#define PERI_CURRENT_MIN 100      // minimum milliAmpere for cutting wire detection
#define AUTO_START_SIGNAL 1       //use to start sender when mower leave station
#define I2C_SDA 21
#define I2C_SCL 22

#define SerialOutput 1  //Show Serial Textmessages for debugging
#define Screen 1        //Commit to deactivate

INA226_WE INAPERI = INA226_WE(0x40);
INA226_WE INACHARGE = INA226_WE(0x44);  //Bridge at A1 - VSS
WiFiServer server(80);




//********************* END Settings **********************************

byte sigCodeInUse = 1;    //1 is the original ardumower sigcode
int sigDuration = 104;    // send the bits each 104 microsecond (Also possible 50)
int8_t sigcode_norm[128];
int sigcode_size;
hw_timer_t* timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

#define pinIN1 12         // M1_IN1  ESP32 GPIO12       ( connect this pin to L298N-IN1)
#define pinIN2 13         // M1_IN2  ESP32 GPIO13       ( connect this pin to L298N-IN2)
#define pinEnableA 23     // ENA    ESP32 GPIO23         (connect this pin to L298N-ENA)
#define pinIN3 14         // M1_IN3  ESP32 GPIO14       ( connect this pin to L298N-IN3)
#define pinIN4 18         // M1_IN4  ESP32 GPIO18       ( connect this pin to L298N-IN4)
#define pinEnableB 19     // ENB    ESP32 GPIO19        (connect this pin to L298N-ENA)

#define pinDoorOpen 34    //Not in use (Magnetic switch)
#define pinDoorClose 35   //Not in use (Magnetic switch)
#define pinLDR 32         //Not in use (Light Sensor)
#define pinGreenLED 25    //Not in use
#define pinRedLED 26      //Not in use

// code version
#define VER "ESP32 3.0"

volatile int step = 0;
boolean enableSenderA = false;  //OFF on start to autorise the reset
boolean enableSenderB = false;  //OFF on start to autorise the reset

int timeSeconds = 0;
unsigned long nextTimeControl = 0;
unsigned long nextTimeInfo = 0;
unsigned long nextTimeSec = 0;
unsigned long nextTimeCheckButton = 0;
int workTimeMins = 0;

float PeriCurrent = 0.0;
float ChargeCurrent = 0.0;
float busvoltage1 = 0.0;
float shuntvoltage2 = 0.0;
float busvoltage2 = 0.0;
float loadvoltage2 = 0.0;
float DcDcOutVoltage = 0.0;

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

void IRAM_ATTR onTimer() {  // management of the signal
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

String IPAddress2String(IPAddress address) {
  return String(address[0]) + "." + String(address[1]) + "." + String(address[2]) + "." + String(address[3]);
}

void changeArea(byte areaInMowing) {  // not finish to dev
  step = 0;
  enableSenderA = false;
  enableSenderB = false;
  #ifdef SerialOutput
  Serial.print("Change to Area : ");
  Serial.println(areaInMowing);
  #endif

  #ifdef Screen
  u8x8.clear();
  u8x8.setCursor(0,0);  
  u8x8.print("Change to Area:");
  u8x8.setCursor(0,1);
  u8x8.println(areaInMowing);
  delay(1000);
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
  u8x8.setCursor(0,3);
  u8x8.print("sigcode size:");
  u8x8.print(sigcode_size);
  delay(2000);
  #endif
}

void connection() {
  #ifdef SerialOutput
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  #endif
  WiFi.begin(ssid, password);
  for (int i = 0; i < 60; i++) {
    if (WiFi.status() != WL_CONNECTED) {

      #ifdef SerialOutput
      Serial.println("Try connecting");
      #endif
      
      #ifdef Screen
      u8x8.clear();
      u8x8.setCursor(0,3);
      u8x8.print("CONNECTING");
      #endif      
      delay(250);
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
    delay(2000);
    #endif
   
     server.begin();
  }
}

static void ScanNetwork() {

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  
  #ifdef Screen
  u8x8.clear();
  u8x8.setCursor(0, 0);
  u8x8.print("Hotspot Lost");
  #endif

  #ifdef SerialOutput
  Serial.println("Hotspot Lost");
  #endif
  
  if (enableSenderA) {
    #ifdef Screen
    u8x8.setCursor(0, 3);
    u8x8.print("Sender ON ");
    #endif
    #ifdef SerialOutput
    Serial.println("Sender ON");
    #endif  
  } else {
    #ifdef Screen
    u8x8.setCursor(0, 3);
    u8x8.print("Sender OFF");
    #endif
    #ifdef SerialOutput
    Serial.println("Sender OFF");
    #endif
  }
  #ifdef Screen
  u8x8.setCursor(0, 4);
  u8x8.print("worktime= ");
  u8x8.setCursor(10, 4);
  u8x8.print("     ");
  u8x8.setCursor(10, 4);
  u8x8.print(workTimeMins, 0);
  #endif
  #ifdef SerialOutput
  Serial.print("Worktime = ");
  Serial.println(workTimeMins);
  #endif

  if (USE_PERI_CURRENT) {
    busvoltage1 = INAPERI.getBusVoltage_V();
    PeriCurrent = INAPERI.getCurrent_mA();
    DcDcOutVoltage = INAPERI.getBusVoltage_V();

    PeriCurrent = PeriCurrent - 100.0;                         //the DC/DC,ESP32,LN298N can drain up to 300 ma when scanning network
    if (PeriCurrent <= 5) PeriCurrent = 0;                     //
    PeriCurrent = PeriCurrent * busvoltage1 / DcDcOutVoltage;  // it's 3.2666 = 29.4/9.0 the power is read before the DC/DC converter so the current change according : 29.4V is the Power supply 9.0V is the DC/DC output voltage (Change according your setting)

    #ifdef Screen
    u8x8.setCursor(0, 5);
    u8x8.print("Pericurr ");
    u8x8.setCursor(10, 5);
    u8x8.print("     ");
    u8x8.setCursor(10, 5);
    u8x8.print(PeriCurrent);
    #endif
    #ifdef SerialOutput
    Serial.print("Pericurr = ");
    Serial.println(PeriCurrent);
    #endif    
  }

  if (USE_STATION) {

    ChargeCurrent = INACHARGE.getCurrent_mA();

    if (ChargeCurrent <= 5) ChargeCurrent = 0;

    #ifdef Screen //Stefan
    u8x8.setCursor(0, 6);
    u8x8.print("Charcurr ");
    u8x8.setCursor(10, 6);
    u8x8.print("     ");
    u8x8.setCursor(10, 6);
    u8x8.print(ChargeCurrent);
    #endif
    #ifdef SerialOutput
    Serial.print("Chargecurr = ");
    Serial.println(ChargeCurrent);
    #endif
  }
  delay(5000);  // wait until all is disconnect
  int n = WiFi.scanNetworks();
  if (n == -1) {
    
    #ifdef Screen
    u8x8.setCursor(0, 0);
    u8x8.print("Scan running...");
    u8x8.setCursor(1, 0);
    u8x8.print("Need Reset? ");
    u8x8.setCursor(2, 0);
    u8x8.print("If sender is OFF");
    #endif
    #ifdef SerialOutput
    Serial.println("Scan running...");
    Serial.println("Need reset?");
    Serial.println("If sender is OFF");
    #endif
        
    delay(5000);
    if ((!enableSenderA) && (!enableSenderB)) ESP.restart();  // do not reset if sender is ON
  }
  if (n == -2)
  //bug in esp32 if wifi is lost many time the esp32 fail to autoreconnect,maybe solve in other firmware ???????
  {
    
    #ifdef Screen
    u8x8.setCursor(0, 0);
    u8x8.print("Scan Fail.");
    u8x8.setCursor(1, 0);
    u8x8.print("Need Reset? ");
    u8x8.setCursor(2, 0);
    u8x8.print("If sender is Off");
    #endif
    #ifdef SerialOutput
    Serial.println("Scan fail.");
    Serial.println("Need reset?");
    Serial.println("If sender is OFF");
    #endif
        
    delay(5000);
    if ((!enableSenderA) && (!enableSenderB)) ESP.restart();
  }
  if (n == 0) {

    #ifdef Screen
    u8x8.setCursor(0, 0);
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
    u8x8.setCursor(0, 0);
    u8x8.print("Find ");
    #endif
    
    for (int i = 0; i < n; ++i) {
      // Print SSID for each network found
      char currentSSID[64];
      WiFi.SSID(i).toCharArray(currentSSID, 64);
      
      #ifdef SerialOutput
      Serial.print("Find Wifi : ");
      Serial.println(currentSSID);
      #endif      
      
      #ifdef Screen
      u8x8.setCursor(0, 5);
      u8x8.print(currentSSID);
      #endif

      delay(1500);
      if (String(currentSSID) == ssid) {
        connection();
        i = 200;  //to avoid loop again when connected
      }
    }
  }
}

// SETUP BEGIN
void setup() {
  //------------------------  Signal parts  ----------------------------------------
  Serial.begin(115200);
  Wire.begin();

  u8x8.begin();
  //u8x8.setFont(u8x8_font_amstrad_cpc_extended_f);    
  //u8x8.setFont(u8x8_font_px437wyse700b_2x2_r);
  //u8x8.setFont(u8x8_font_chroma48medium8_r);  
  //u8x8.setFont(u8x8_font_inb33_3x6_n);
  //u8x8.setFont(u8x8_font_5x7_f);
  u8x8.setFont(u8x8_font_5x8_f);
  //u8x8.setFont(u8x8_font_artosserif8_r);
  //u8x8.setFont(u8x8_font_victoriamedium8_r);
  //u8x8.setFont(u8x8_font_pressstart2p_f);
  u8x8.clear();

  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 104, true);
  timerAlarmEnable(timer);
  pinMode(pinIN1, OUTPUT);
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

  //------------------------  SCREEN parts  ----------------------------------------
  #ifdef Screen
  u8x8.clear();
  u8x8.setCursor(0, 0);  // Set cursor position, start of line 0
  u8x8.print("TEENSYMOWER");  
  u8x8.setCursor(0, 1);  // Set cursor position, start of line 1
  u8x8.print("BB SENDER");
  u8x8.setCursor(0, 2);  // Set cursor position, start of line 2
  u8x8.print(VER);
  u8x8.setCursor(0, 3);  // Set cursor position, line 2 10th character
  u8x8.print("2 LOOPS");
  #endif
  #ifdef SerialOutput
  Serial.println("TEENSYMOWER");
  Serial.println("BB SENDER");
  Serial.println(VER);
  Serial.println("2 LOOPS");
  #endif

  //------------------------  current sensor parts  ----------------------------------------
  #ifdef SerialOutput
  Serial.println("Measuring voltage and current using INA226 ...");
  #endif
  
  INAPERI.init();
  INACHARGE.init();
  delay(5000);
}
// SETUP END

// LOOP BEGIN

void loop() {
  if (millis() >= nextTimeControl) {
    nextTimeControl = millis() + 1000;  //after debug can set this to 10 secondes

    #ifdef Screen
    u8x8.setCursor(0, 4);
    u8x8.print("worktime = ");
    u8x8.setCursor(10, 4);
    u8x8.print("     ");
    u8x8.setCursor(10, 4);
    u8x8.print(workTimeMins);
    #endif
    #ifdef SerialOutput    
    Serial.print("Worktime = ");
    Serial.println(workTimeMins);
    #endif
        
    if (USE_PERI_CURRENT) {
      busvoltage1 = INAPERI.getBusVoltage_V();
      PeriCurrent = INAPERI.getCurrent_mA();
      PeriCurrent = PeriCurrent - 100.0;                         //the DC/DC,ESP32,LN298N drain 100 ma when nothing is ON and a wifi access point is found (To confirm ????)
      if (PeriCurrent <= 5) PeriCurrent = 0;                     //
      PeriCurrent = PeriCurrent * busvoltage1 / DcDcOutVoltage;  // it's 3.2666 = 29.4/9.0 the power is read before the DC/DC converter so the current change according : 29.4V is the Power supply 9.0V is the DC/DC output voltage (Change according your setting)

      if ((enableSenderA) && (PeriCurrent < PERI_CURRENT_MIN)) {
        #ifdef Screen
        u8x8.setCursor(0, 5);
        u8x8.inverse();
        u8x8.print("  Wire is Cut   ");
        u8x8.noInverse();
        #endif
        #ifdef SerialOutput
        Serial.println("WIRE IS CUT!!!");
        #endif
        
      } else {
        #ifdef Screen
        u8x8.setCursor(0, 5);
        u8x8.print("Pericurr ");
        u8x8.setCursor(8, 5);
        u8x8.print("       ");
        u8x8.setCursor(10, 5);
        u8x8.print(PeriCurrent);
        #endif
        #ifdef SerialOutput
        Serial.print("Pericurr ");
        Serial.println(PeriCurrent);
        #endif
      }
    }

    if ((WiFi.status() != WL_CONNECTED)) ScanNetwork();
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

      Serial.println("********************************   Timeout , so stop Sender  **********************************");
    }
  }

  if (millis() >= nextTimeSec) {
    nextTimeSec = millis() + 1000;

    #ifdef Screen
    u8x8.setCursor(0, 7);
    u8x8.print("                ");
    u8x8.setCursor(0, 7);
    u8x8.print("Area : ");
    u8x8.setCursor(7, 7);
    u8x8.print(sigCodeInUse);
    #endif
    #ifdef SerialOutput
    Serial.print("Area : ");
    Serial.println(sigCodeInUse);
    #endif

    if (USE_STATION) {

      busvoltage2 = INACHARGE.getBusVoltage_V();
      shuntvoltage2 = INACHARGE.getShuntVoltage_mV();
      ChargeCurrent = INACHARGE.getCurrent_mA();

      if (ChargeCurrent <= 5) ChargeCurrent = 0;
      loadvoltage2 = busvoltage2 + (shuntvoltage2 / 1000);

      #ifdef Screen
      u8x8.setCursor(0, 6);
      u8x8.print("Charcurr ");
      u8x8.setCursor(10, 6);
      u8x8.print("     ");
      u8x8.setCursor(10, 6);
      u8x8.print(ChargeCurrent);
      #endif
      #ifdef SerialOutput
      Serial.print("Charcurr ");
      Serial.println(ChargeCurrent);
      #endif

      if (ChargeCurrent > 200) {  //mower is into the station ,in my test 410 ma are drained so possible to stop sender
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
            workTimeMins = 0;
            enableSenderA = true;
            digitalWrite(pinEnableA, HIGH);
            digitalWrite(pinIN1, LOW);
            digitalWrite(pinIN2, LOW);
          } else {
            workTimeMins = 0;
            enableSenderB = true;
            digitalWrite(pinEnableB, HIGH);
            digitalWrite(pinIN3, LOW);
            digitalWrite(pinIN4, LOW);
          }
        }
      }
    }
    timeSeconds++;
    if (((enableSenderA) || (enableSenderB)) && (timeSeconds >= 60)) {
      if (workTimeMins < 1440) workTimeMins++;
      timeSeconds = 0;
    }

    if ((enableSenderA) || (enableSenderB)) {

      #ifdef Screen
      u8x8.setCursor(0,0);
      u8x8.inverse();
      u8x8.print("  Teensy Sender  ");
      u8x8.noInverse();
      u8x8.setCursor(0, 2);
      u8x8.print("Sender ON :     ");
      #endif
      #ifdef SerialOutput
      Serial.print("Sender ON : ");
      #endif

      if (enableSenderA) {
        #ifdef Screen
        u8x8.setCursor(13, 2);
        u8x8.print("A");
        #endif
        #ifdef SerialOutput
        Serial.print("A");
        #endif
      }
      if (enableSenderB) {
        #ifdef Screen
        u8x8.setCursor(15, 2);
        u8x8.print("B");
        #endif
        #ifdef SerialOutput
        Serial.print("B");
        #endif
      }
    } else {
      #ifdef Screen
      u8x8.setCursor(2, 0);
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

  if (millis() >= nextTimeInfo) {
    nextTimeInfo = millis() + 500;
    float v = 0;
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
      sResponse = "MAC ADRESS = ";
      sResponse += WiFi.macAddress();
      sResponse += " WORKING DURATION= ";
      sResponse += workTimeMins;
      sResponse += " PERI CURRENT Milli Amps= ";
      sResponse += PeriCurrent;
      sResponse += " CHARGE CURRENT Milli Amps= ";
      sResponse += ChargeCurrent;
      sResponse += " sigDuration= ";
      sResponse += sigDuration;
      sResponse += " sigCodeInUse= ";
      sResponse += sigCodeInUse;
      sResponse += " sender A : ";
      sResponse += enableSenderA;
      sResponse += " sender B : ";
      sResponse += enableSenderB;

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