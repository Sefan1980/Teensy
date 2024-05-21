#define BLUETOOTH
//Select one of the 2 possibility AP (access point) or STA (station)

#define WiFiAccess

// Just one mode is possible
#define MODE_AP           // phone connects directly to ESP32 inside the mower - Mower is an accesspoint! 
//#define MODE_STA        // ESP32 connects to wifi router - Phone has to be connected to the same WiFi!

// OTAupdates are just possible with MODE_STA
#define OTAupdates

#define bufferSize 1024

bool debug = true;        // Show debug messages in serial console

#define VERSION "1.00"

#ifdef MODE_STA
// For station mode:
// You need to set a fix IP and gateway according your router value.
// ssid and password according to your router
const char *ssid = "Your ssid";       // You will connect your phone to this Access Point
const char *pw = "Your password";     // and this is the password

//for pfod use this IP and port 8881
IPAddress ip(10, 0, 0, 122);          // IP for your Mower
IPAddress gateway(10, 0, 0, 1);       // Gateway from your WiFi, most it's the Router IP
IPAddress netmask(255, 255, 255, 0);
#endif


#ifdef MODE_AP

// For AP mode:
// Don't forget to connect your WiFi phone to the AP
const char *ssid = "Teensy2";         // You will connect your phone to this Access Point
const char *pw = "";                  // password

//for pfod use this IP and port 8881
IPAddress ip(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 0);
IPAddress netmask(255, 255, 255, 0);
#endif
