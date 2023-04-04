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
//#include "INA226.h"
#include <INA226_WE.h>  //Stefan
//#define screenActive  //if uncommented, screen is active
#ifdef screenActive
#include "ACROBOTIC_SSD1306.h"
#endif

//********************* User Setting **********************************
const char* ssid = "Sefan WLAN 2,4GHz";          // put here your acces point ssid
const char* password = "StefanJaninaRenkaDeik";  // put here the password

//********************* setting for current sensor **********************************
//Use to have a correct value on perricurrent (Need to change the value each time you adjust the DC DC )

//Stefan
//float DcDcOutVoltage = 9.0; //Check if it is needed for INA226 (Sascha) 
// Wert kann direkt aus dem INAPERI gelesen werden!? Stefan

//********************* IP Adress Settings **********************************
IPAddress staticIP(192,168,178,222);  // put here the static IP
IPAddress gateway(192,168,178,1);     // put here the gateway (IP of your routeur)
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(192,168,178,1);  // put here one dns (IP of your routeur)

#define USE_STATION 1             // a station is connected and is used to charge the mower
#define USE_PERI_CURRENT 1        // use Feedback for perimeter current measurements? (set to '0' if not connected!)
#define USE_BUTTON 0              // use button to start mowing or send mower to station not finish to dev
#define USE_RAINFLOW 0            // check the amount of rain not finish to dev on 31/08/2020
#define WORKING_TIMEOUT_MINS 300  // timeout for perimeter switch-off if robot not in station (minutes)
#define PERI_CURRENT_MIN 100      // minimum milliAmpere for cutting wire detection
#define AUTO_START_SIGNAL 1       //use to start sender when mower leave station

//Stefan
//INA226 INAPERI;
//INA226 INACHARGE;
INA226_WE INAPERI = INA226_WE(0x40);
INA226_WE INACHARGE = INA226_WE(0x44);  //Bridge at A1 - VSS

//********************* END Settings **********************************
byte sigCodeInUse = 1;  //1 is the original ardumower sigcode
int sigDuration = 104;  // send the bits each 104 microsecond (Also possible 50)
int8_t sigcode_norm[128];
int sigcode_size;
hw_timer_t* timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;


//#define I2C_SDA 21
//#define I2C_SCL 22
//#define PERI_CURRENT_CHANNEL 1
//#define MOWER_STATION_CHANNEL 2
#define pinIN1 12      // M1_IN1  ESP32 GPIO15       ( connect this pin to L298N-IN1)
#define pinIN2 13      // M1_IN2  ESP32 GPIO14       ( connect this pin to L298N-IN2)
#define pinEnableA 23  // ENA    ESP32 GPIO23         (connect this pin to L298N-ENA)
//possible to use the same gpio 12 and 13  instead of 14 and 18
#define pinIN3 14      // M1_IN3  ESP32 GPIO16       ( connect this pin to L298N-IN3)
#define pinIN4 18      // M1_IN4  ESP32 GPIO18       ( connect this pin to L298N-IN4)
#define pinEnableB 19  // ENB    ESP32 GPIO19        (connect this pin to L298N-ENA)

#define pinDoorOpen 34   //Not in use (Magnetic switch)
#define pinDoorClose 35  //Not in use (Magnetic switch)
#define pinLDR 32        //Not in use (Light Sensor)
#define pinGreenLED 25   //Not in use
#define pinRedLED 9  26  //Not in use

//#define pinPushButton 34  //           (connect to Button) //R1 2.2K R2 3.3K
//#define pinRainFlow 35    //           (connect to Rain box) //R3 2.2K R4 3.3K

// code version
#define VER "ESP32 3.0"

volatile int step = 0;
boolean enableSenderA = false;  //OFF on start to autorise the reset
boolean enableSenderB = false;  //OFF on start to autorise the reset
//boolean WiffiRequestOn = true;

int timeSeconds = 0;
unsigned long nextTimeControl = 0;
unsigned long nextTimeInfo = 0;
unsigned long nextTimeSec = 0;
unsigned long nextTimeCheckButton = 0;
int workTimeMins = 0;

//boolean StartButtonProcess = false;
//int Button_pressed = 0;

float PeriCurrent = 0.0;
float ChargeCurrent = 0.0;
float busvoltage1 = 0.0;
float shuntvoltage2 = 0.0;
float busvoltage2 = 0.0;
float loadvoltage2 = 0.0;
float DcDcOutVoltage = 0.0;  //Stefan

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

WiFiServer server(80);
//SDL_Arduino_INA3221 ina3221;

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
      Serial.println("ERROR");
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
      Serial.println("ERROR");
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
  Serial.print("Change to Area : ");
  Serial.println(areaInMowing);
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

  Serial.print("New sigcode in use  : ");
  Serial.println(sigCodeInUse);

  for (int uu = 0; uu <= (sigcode_size - 1); uu++) {
    Serial.print(sigcode_norm[uu]);
    Serial.print(",");
  }
  Serial.println();
  Serial.print("New sigcode size  : ");
  Serial.println(sigcode_size);
  //enableSenderA = true;
  //enableSenderB = true;
}

void connection() {
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  //WiFi.setAutoReconnect(true);
  WiFi.begin(ssid, password);
  for (int i = 0; i < 60; i++) {
    if (WiFi.status() != WL_CONNECTED) {

     #ifdef screenActive
       oled.clearDisplay();  //Stefan  
       oled.setTextXY(0, 0);
       oled.putString("Try connecting");
     #else
     Serial.println("Try connecting");
     #endif

      delay(250);
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected.");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    String ipAddress = IPAddress2String(WiFi.localIP());

    #ifdef screenActive //Stefan
    oled.clearDisplay();
    oled.setTextXY(0, 0);
    oled.putString("WIFI Connected");
    oled.setTextXY(1, 0);
    oled.putString(ipAddress);
    #else
    Serial.println("WIFI Connected");  //Stefan
    Serial.println(ipAddress);  //Stefan
    #endif

    server.begin();
  }
}

static void ScanNetwork() {

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  
  #ifdef screenActive  //Stefan
  oled.clearDisplay();
  oled.setTextXY(0, 0);
  oled.putString("Hotspot Lost");
  #else
  Serial.println("Hotspot Lost");
  #endif
  
  if (enableSenderA) {
    #ifdef screenActive //Stefan
    oled.setTextXY(3, 0);
    oled.putString("Sender ON ");
    #else
    Serial.println("Sender ON");
    #endif  
  } else {
    #ifdef screenActive
    oled.setTextXY(3, 0);
    oled.putString("Sender OFF");
    #else
    Serial.println("Sender OFF");
    #endif
  }
  #ifdef screenActive
  oled.setTextXY(4, 0);
  oled.putString("worktime= ");
  oled.setTextXY(4, 10);
  oled.putString("     ");
  oled.setTextXY(4, 10);
  oled.putFloat(workTimeMins, 0);
  #else
  Serial.print("Worktime = ");
  Serial.println(workTimeMins);
  #endif

  if (USE_PERI_CURRENT) {

    //Stefan
    //busvoltage1 = INAPERI.readBusVoltage();
    //PeriCurrent = INAPERI.readShuntCurrent();
    busvoltage1 = INAPERI.getBusVoltage_V();
    PeriCurrent = INAPERI.getCurrent_mA();
    DcDcOutVoltage = INAPERI.getBusVoltage_V();

    PeriCurrent = PeriCurrent - 100.0;                         //the DC/DC,ESP32,LN298N can drain up to 300 ma when scanning network
    if (PeriCurrent <= 5) PeriCurrent = 0;                     //
    PeriCurrent = PeriCurrent * busvoltage1 / DcDcOutVoltage;  // it's 3.2666 = 29.4/9.0 the power is read before the DC/DC converter so the current change according : 29.4V is the Power supply 9.0V is the DC/DC output voltage (Change according your setting)

    #ifdef screenActive //Stefan
    oled.setTextXY(5, 0);
    oled.putString("Pericurr ");
    oled.setTextXY(5, 10);
    oled.putString("     ");
    oled.setTextXY(5, 10);
    oled.putFloat(PeriCurrent, 0);
    #else
    Serial.print("Pericurr = ");
    Serial.println(PeriCurrent);
    #endif    
  }

  if (USE_STATION) {
 
    //Stefan 
    //ChargeCurrent = INACHARGE.readShuntCurrent();
    ChargeCurrent = INACHARGE.getCurrent_mA();
    
    if (ChargeCurrent <= 5) ChargeCurrent = 0;
   
    #ifdef screenActive //Stefan
    oled.setTextXY(6, 0);
    oled.putString("Charcurr ");
    oled.setTextXY(6, 10);
    oled.putString("     ");
    oled.setTextXY(6, 10);
    oled.putFloat(ChargeCurrent, 0);
    #else
    Serial.print("Chargecurr = ");
    Serial.println(ChargeCurrent);
    #endif

  }
  delay(5000);  // wait until all is disconnect
  int n = WiFi.scanNetworks();
  if (n == -1) {
    
    #ifdef screenActive //Stefan
    oled.setTextXY(0, 0);
    oled.putString("Scan running...");
    oled.setTextXY(1, 0);
    oled.putString("Need Reset? ");
    oled.setTextXY(2, 0);
    oled.putString("If sender is OFF");
    #else
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
    
    #ifdef screenActive //Stefan
    oled.setTextXY(0, 0);
    oled.putString("Scan Fail.");
    oled.setTextXY(1, 0);
    oled.putString("Need Reset? ");
    oled.setTextXY(2, 0);
    oled.putString("If sender is Off");
    #else
    Serial.println("Scan fail.");
    Serial.println("Need reset?");
    Serial.println("If sender is OFF");
    #endif
        
    delay(5000);
    if ((!enableSenderA) && (!enableSenderB)) ESP.restart();
  }
  if (n == 0) {

    #ifdef screenActive //Stefan
    oled.setTextXY(0, 0);
    oled.putString("No networks.");
    #else
    Serial.println("No networks.");
    #endif

  }
  if (n > 0) {
    Serial.print("Find ");
    Serial.println(n);



    #ifdef screenActive //Stefan
    oled.setTextXY(0, 0);
    oled.putString("Find ");
    #endif
    
    for (int i = 0; i < n; ++i) {
      // Print SSID for each network found
      char currentSSID[64];
      WiFi.SSID(i).toCharArray(currentSSID, 64);
      Serial.print("Find Wifi : ");
      Serial.println(currentSSID);

      #ifdef screenActive //Stefan
      oled.setTextXY(0, 5);
      oled.putString(currentSSID);
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
//  pinMode(pinPushButton, INPUT);
//  pinMode(pinRainFlow, INPUT);

  Serial.println("START");
  Serial.print("Teensymower Sender ");
  Serial.println(VER);
  Serial.print("USE_PERI_CURRENT=");
  Serial.println(USE_PERI_CURRENT);

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
    Serial.println("WIFI Configuration failed.");
  }  
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  //------------------------  SCREEN parts  ----------------------------------------



  #ifdef screenActive //Stefan
  oled.init();  // Initialze SSD1306 OLED display
  delay(500);  
  oled.clearDisplay();  // Clear screen  
  delay(500);
  oled.setTextXY(0, 0);  // Set cursor position, start of line 0
  oled.putString("TEENSYMOWER");  
  oled.setTextXY(1, 0);  // Set cursor position, start of line 1
  oled.putString("BB SENDER");
  oled.setTextXY(2, 0);  // Set cursor position, start of line 2
  oled.putString(VER);
  oled.setTextXY(3, 0);  // Set cursor position, line 2 10th character
  oled.putString("2 LOOPS");
  #else
  Serial.println("TEENSYMOWER");
  Serial.println("BB SENDER");
  Serial.println(VER);
  Serial.println("2 LOOPS");
  #endif

  //------------------------  current sensor parts  ----------------------------------------
  Serial.println("Measuring voltage and current using INA226 ...");

  //Stefan
  //INAPERI.begin(0x41);
  //INACHARGE.begin(0x44);
  INAPERI.init();
  INACHARGE.init();

//  Serial.print("Manufactures ID=0x");
//  int MID;
//  MID = ina3221.getManufID();
//  Serial.println(MID, HEX);
  delay(5000);
}
// SETUP END

// LOOP BEGIN
void loop() {
  if (millis() >= nextTimeControl) {
    nextTimeControl = millis() + 1000;  //after debug can set this to 10 secondes
    //StartButtonProcess = false;



    #ifdef screenActive //Stefan
    oled.setTextXY(4, 0);
    oled.putString("worktime = ");
    oled.setTextXY(4, 10);
    oled.putString("     ");
    oled.setTextXY(4, 10);
    oled.putFloat(workTimeMins, 0);
    #else    
    Serial.print("Worktime = ");
    Serial.println(workTimeMins);
    #endif
    
    
    if (USE_PERI_CURRENT) {



      //Stefan
      //busvoltage1 = INAPERI.readBusVoltage();
      //PeriCurrent = INAPERI.readShuntCurrent();
      busvoltage1 = INAPERI.getBusVoltage_V();
      PeriCurrent = INAPERI.getCurrent_mA();
    


      PeriCurrent = PeriCurrent - 100.0;                         //the DC/DC,ESP32,LN298N drain 100 ma when nothing is ON and a wifi access point is found (To confirm ????)
      if (PeriCurrent <= 5) PeriCurrent = 0;                     //
      PeriCurrent = PeriCurrent * busvoltage1 / DcDcOutVoltage;  // it's 3.2666 = 29.4/9.0 the power is read before the DC/DC converter so the current change according : 29.4V is the Power supply 9.0V is the DC/DC output voltage (Change according your setting)


      if ((enableSenderA) && (PeriCurrent < PERI_CURRENT_MIN)) {



        #ifdef screenActive //Stefan
        oled.setTextXY(5, 0);
        oled.putString("  Wire is Cut  ");
        #else
        Serial.println("WIRE IS CUT!!!");
        #endif
        
      } else {
        #ifdef screenActive
        oled.setTextXY(5, 0);
        oled.putString("Pericurr ");
        oled.setTextXY(5, 10);
        oled.putString("     ");
        oled.setTextXY(5, 10);
        oled.putFloat(PeriCurrent, 0);
        #else
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

    #ifdef screenActive //Stefan
    oled.setTextXY(7, 0);
    oled.putString("                ");
    oled.setTextXY(7, 0);
    oled.putString("Area : ");
    oled.setTextXY(7, 7);
    oled.putFloat(sigCodeInUse, 0);
    #else
    Serial.print("Area : ");  //Stefan
    Serial.println(sigCodeInUse);  //Stefan
    #endif

    if (USE_STATION) {

      //busvoltage2 = INACHARGE.readBusVoltage();
      //shuntvoltage2 = INACHARGE.readShuntVoltage();
      //ChargeCurrent = INACHARGE.readShuntCurrent();
      busvoltage2 = INACHARGE.getBusVoltage_V();
      shuntvoltage2 = INACHARGE.getShuntVoltage_mV();
      ChargeCurrent = INACHARGE.getCurrent_mA();



      if (ChargeCurrent <= 5) ChargeCurrent = 0;
      loadvoltage2 = busvoltage2 + (shuntvoltage2 / 1000);

      #ifdef screenActive //Stefan
      oled.setTextXY(6, 0);
      oled.putString("Charcurr ");
      oled.setTextXY(6, 10);
      oled.putString("     ");
      oled.setTextXY(6, 10);
      oled.putFloat(ChargeCurrent, 0);
      #else
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

      #ifdef screenActive //Stefan
      oled.setTextXY(2, 0);
      oled.putString("Sender ON :     ");
      #else
      Serial.print("Sender ON : ");
      #endif

      if (enableSenderA) {
        #ifdef screenActive //Stefan
        oled.setTextXY(2, 13);
        oled.putString("A");
        #else
        Serial.print("A");
        #endif
      }
      if (enableSenderB) {
        #ifdef screenActive //Stefan
        oled.setTextXY(2, 15);
        oled.putString("B");
        #else
        Serial.print("B");  //Stefan
        #endif
      }
    } else {
      #ifdef screenActive //Stefan
      oled.setTextXY(2, 0);
      oled.putString("Sender OFF      ");
      #else
      Serial.print("Sender OFF");
      #endif

    }
    Serial.println("");
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
    Serial.print("Client say  ");
    Serial.println(req);
    Serial.println("------------------------ - ");
    //client.flush();
    // Match the request
    if (req.indexOf("GET /A0") != -1) {
      // WiffiRequestOn = false;
      enableSenderA = false;
      workTimeMins = 0;
      digitalWrite(pinEnableA, LOW);
      digitalWrite(pinIN1, LOW);
      digitalWrite(pinIN2, LOW);
      String sResponse;
      sResponse = "SENDER A IS OFF";
      // Send the response to the client
      Serial.println(sResponse);
      client.print(sResponse);
      client.flush();
    }
    if (req.indexOf("GET /B0") != -1) {
      // WiffiRequestOn = false;
      enableSenderB = false;
      workTimeMins = 0;
      digitalWrite(pinEnableB, LOW);
      digitalWrite(pinIN3, LOW);
      digitalWrite(pinIN4, LOW);
      String sResponse;
      sResponse = "SENDER B IS OFF";
      // Send the response to the client
      Serial.println(sResponse);
      client.print(sResponse);
      client.flush();
    }
    if (req.indexOf("GET /A1") != -1) {
      //WiffiRequestOn = 1;
      workTimeMins = 0;
      enableSenderA = true;
      digitalWrite(pinEnableA, HIGH);
      digitalWrite(pinIN1, LOW);
      digitalWrite(pinIN2, LOW);
      // Prepare the response
      String sResponse;
      sResponse = "SENDER A IS ON";
      // Send the response to the client
      Serial.println(sResponse);
      client.print(sResponse);
      client.flush();
    }
    if (req.indexOf("GET /B1") != -1) {
      //WiffiRequestOn = 1;
      workTimeMins = 0;
      enableSenderB = true;
      digitalWrite(pinEnableB, HIGH);
      digitalWrite(pinIN3, LOW);
      digitalWrite(pinIN4, LOW);
      // Prepare the response
      String sResponse;
      sResponse = "SENDER B IS ON";
      // Send the response to the client
      Serial.println(sResponse);
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

      Serial.println(sResponse);
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
      Serial.println(sResponse);
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
      Serial.println(sResponse);
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
      Serial.println(sResponse);
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
      Serial.println(sResponse);
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
      Serial.println(sResponse);
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
      Serial.println(sResponse);
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
      Serial.println(sResponse);
      client.print(sResponse);
      client.flush();
    }
  }
/*
  if ((USE_BUTTON) && (millis() > nextTimeCheckButton)) {
    nextTimeCheckButton = millis() + 100;
    if (StartButtonProcess) {
      oled.setTextXY(7, 0);
      oled.putString("StartButtonPress");
    }

    Button_pressed = digitalRead(pinPushButton);
    if (Button_pressed == 1) {
      Serial.println("Button pressed");
      //WiFiClient client = server.available();
      workTimeMins = 0;
      enableSenderA = true;
      digitalWrite(pinEnableA, HIGH);
      digitalWrite(pinIN1, LOW);
      digitalWrite(pinIN2, LOW);
      if (client) {
        String sResponse, sHeader;
        sResponse = "BUTTON PRESSED";
        client.print(sResponse);
        client.flush();
      }
      nextTimeCheckButton = millis() + 1000;
      StartButtonProcess = true;
      nextTimeControl = millis() + 3000;
    }
  }
*/
}
// LOOP END
