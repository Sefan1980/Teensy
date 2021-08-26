/*

  choose your robot type, PCB version, baud rates, pin definitions etc.

*/

#ifndef MOWER_H
#define MOWER_H

#include <Arduino.h>
#include "robot.h"
#include "drivers.h"



/*  This software requires:
        Ardumower PCB v0.5/1.2/1.3  ( https://www.marotronics.de/Ardumower-Board-Prototyp )
        Arduino Mega or Arduino Due (Due requires PCB1.3)
        Ardumower Chassis Kit 1.0  ( http://wiki.ardumower.de/index.php?title=Ardumower_chassis ) or Ardumower Mini
*/


// ------ pins---------------------------------------

#define pinMotorLeftEnable  5         // EN motors enable
#define pinMotorLeftPWM 7          // M1_IN1 left motor PWM pin
#define pinMotorLeftDir 6         // M1_IN2 left motor Dir pin
//#define pinMotorLeftSense A1       // M1_FB  left motor current sense
//#define pinMotorLeftFault 25       // M1_SF  left motor fault

#define pinMotorRightEnable  2        // EN motors enable
#define pinMotorRightPWM  4        // M2_IN1 right motor PWM pin
#define pinMotorRightDir 3        // M2_IN2 right motor Dir pin
//#define pinMotorRightSense A0      // M2_FB  right motor current sense
//#define pinMotorRightFault 27      // M2_SF  right motor fault

#define pinMotorMowEnable 8       // EN mower motor enable      (if using MOSFET/L298N, keep unconnected)
#define pinMotorMowPWM 10           // M1_IN1 mower motor PWM pin (if using MOSFET, use this pin)
#define pinMotorMowDir 9          // M1_IN2 mower motor Dir pin (if using MOSFET, keep unconnected)
//#define pinMotorMowSense A3        // M1_FB  mower motor current sense  
//#define pinMotorMowFault 26        // M1_SF  mower motor fault   (if using MOSFET/L298N, keep unconnected)
//#define pinMotorMowRpm A11

#define pinBumperLeft 35          // bumper pins
#define pinBumperRight 36

//#define pinDropLeft 45           // drop pins                                                                                          Dropsensor - Absturzsensor
//#define pinDropRight 23          // drop pins                                                                                          Dropsensor - Absturzsensor

//#define pinSonarCenterTrigger 24   // ultrasonic sensor pins
//#define pinSonarCenterEcho 22
#define pinSonarRightTrigger D29  //BBER10
#define pinSonarRightEcho A13
#define pinSonarLeftTrigger D28   //BBER10
#define pinSonarLeftEcho A12


#define pinPerimeterRight A8       // perimeter
#define pinPerimeterLeft A9
//#define pinPerimeterCenter A6


//#define pinGreenLED 6              // DuoLED green
//#define pinRedLED 7                // DuoLED red
//#define pinLED 13                  // LED
#define pinBuzzer 37               // Buzzer
//#define pinTilt 35                 // Tilt sensor (required for TC-G158 board)
#define pinButton 38             // digital ON/OFF button
//#define pinBatteryVoltage A2       // battery voltage sensor
#define pinBatterySwitch 33         // battery-OFF switch   
//#define pinChargeVoltage A9        // charging voltage sensor
//#define pinChargeCurrent A8        // charge current sensor
#define pinChargeEnable 34          // charge relay
//#define pinRemoteMow 12            // remote control mower motor
//#define pinRemoteSteer 11          // remote control steering 
//#define pinRemoteSpeed 10          // remote control speed
//#define pinRemoteSwitch 52         // remote control switch
//#define pinVoltageMeasurement A7   // test pin for your own voltage measurements

#define pinOdometryLeft 12     // left odometry sensor
//#define pinOdometryLeft2 DAC1    // left odometry sensor (optional two-wire)
#define pinOdometryRight 11   // right odometry sensor  
//#define pinOdometryRight2 CANTX  // right odometry sensor (optional two-wire)  

//#define pinLawnFrontRecv 40        // lawn sensor front receive
//#define pinLawnFrontSend 41        // lawn sensor front sender 
//#define pinLawnBackRecv 42         // lawn sensor back receive
//#define pinLawnBackSend 43         // lawn sensor back sender 
#define pinUserSwitch1 13          // user-defined switch 1
#define pinUserSwitch2 32          // user-defined switch 2
#define pinUserSwitch3 48          // user-defined switch 3
#define pinRain 39                 // rain sensor

// IMU (compass/gyro/accel): I2C  (SCL, SDA)




// ------- baudrates---------------------------------



// ------ used serial ports for console, Bluetooth, ESP8266 -----------------------------

// Due has two serial ports: Native (SerialUSB) and Programming (Serial) -
// we use 'SerialUSB' for 'Console' so the Raspberry PI receive all data
// we use 'Serial' for 'Console' so the PC receive all data



//#define COMPASS_IS HMC5883L
#define COMPASS_IS QMC5883L
 
#define Console Serial
#define CONSOLE_BAUDRATE    115200       // baudrate used for Raspberry PI console

#define Enable_DueWatchdog true
//#define Enable_DueWatchdog false

#define autoBylaneToRandom true

#define RaspberryPIPort Serial  //The PI is connected over USB cable


#define Bluetooth Serial1  // Ardumower default OK for ESP32 or HC05
#define BLUETOOTH_BAUDRATE  115200     // baudrate used for communication with Bluetooth module (Ardumower default: 19200)


#define GpsPort Serial3  // GPS do not forget workarround if PCB1.3 use


// ------- ultrasonic config ---------------------------------------------------------
#define NO_ECHO 0


// ---- choose only one perimeter signal code ----
#define SIGCODE_1  // Ardumower default perimeter signal
//#define SIGCODE_2  // Ardumower alternative perimeter signal
//#define SIGCODE_3  // Ardumower alternative perimeter signal


/*
  Ardumower robot chassis
*/

class Mower : public Robot
{
  public:
    Mower();
    virtual void setup(void);
    // virtual void resetMotorFault();
    
    virtual void setActuator(char type, int value);
   
   
};


extern Mower robot;

#endif
