#define PROTOCOL_TCP
#define bufferSize 1024

bool debug = true;

#define VERSION "2.00"

//const char *ssid = "Linda-Lu";         // WiFi SSID from your router
//const char *pw = "BesteLaune";       // and this is the password
const char *ssid = "Personal OA";         // WiFi SSID from your router
const char *pw = "scharbeutz2019";       // and this is the password

//for pfod use this IP and port 8881
//IPAddress ip(192, 168, 178, 222);
//IPAddress gateway(192, 168, 178, 1);
//IPAddress netmask(255, 255, 255, 0);

IPAddress ip(192, 168, 27, 222);
IPAddress gateway(192, 168, 27, 1);
IPAddress netmask(255, 255, 255, 0);
