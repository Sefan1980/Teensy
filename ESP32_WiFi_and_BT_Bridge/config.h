// use Bluetooth?
#define BLUETOOTH

// use WiFi?
#define WiFiAccess

//Select one of the 2 possibilities AP (access point) or STA (station)
// Just one mode is possible
#define MODE_AP           // phone connects directly to ESP32 inside the mower - Mower is an accesspoint! 
//#define MODE_STA        // ESP32 connects to wifi router - Phone has to be connected to the same WiFi!

// use OTAupdates? (just possible with MODE_STA)
#define OTAupdates

// buffersize
#define bufferSize 1024

// PFOD port
const int PFODport = 8881;

// Baudrate to console
const int consoleBaud = 115200;

// Baudrate to Mower
const int mowerBaud = 115200;

// Show debug messages in serial console
bool debug = true;

// version
#define VERSION "2.00"


// If you use Station mode, type in your SSID, PASSWORD, IP, GATEWAY and SUBNETMASK
#ifdef MODE_STA

// SSID and PASSWORD
const char *ssid = "Your ssid";       // You will connect your phone to this Access Point
const char *pw = "Your password";     // and this is the password

//for pfod use this IP and port 8881
IPAddress ip(192, 168, 178, 122);          // IP for your Mower
IPAddress gateway(192, 168, 178, 1);       // Gateway from your WiFi, most it's the Router IP
IPAddress netmask(255, 255, 255, 0);
#endif


// Use the following data when using the access point.
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
