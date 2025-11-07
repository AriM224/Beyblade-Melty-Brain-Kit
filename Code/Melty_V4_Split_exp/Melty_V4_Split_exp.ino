#include <esp_task_wdt.h>
#include <Wire.h>
#include <SparkFun_LIS331.h>
#include <Adafruit_NeoPixel.h>
#include <SPI.h>
#include <IBusBM.h>
#include <WiFi.h>
#include <WiFiAP.h>
#include <ArduinoOTA.h>
#include <TelnetStream.h>
#include <Arduino.h>
#include "DShotESC.h"
#include <EEPROM.h>

//------accelerometer config------------
#define ACCEL_RANGE LIS331::HIGH_RANGE  //sets to 400g range
#define ACCEL_MAX_SCALE 400
#define ACCEL_I2C_ADDRESS 0x19 //0x18 for adafruit accel, 0x19 for sparkfun
LIS331 xl; //accelerometer object
const int accradius = 20; //radius where the g force sensor is located in millimeters

//-------ESC config--------
#define escR_gpio GPIO_NUM_36 //right esc pin assignment
#define escL_gpio GPIO_NUM_37  //left esc pin assignment
DShotESC escR;  // create servo object to control the left esc
DShotESC escL;  // create servo object to control the right esc
//library uses values between -999 and 999 to control speed of esc

//---------Other pin assignemnts----------
const int top_led_pin = A3;  //top led strip pin
const int bottom_led_pin = A2;  //bottom led strip pin
const int volt_pin = A0; //pin for measure battery voltage

//------LED definitions--------
const int NUMPIXELST = 10; //number of leds in the top strip
const int NUMPIXELSB = 8; //number of leds in the bottom strip
#define RMT_CHANNEL_MAX 1 //limits rmt channels to use
Adafruit_NeoPixel top_strip(NUMPIXELST, top_led_pin, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel bottom_strip(NUMPIXELSB, bottom_led_pin, NEO_GRB + NEO_KHZ800);
const int animSpeed = 100;   // ms between LED updates when in animation mode

//------Reciever config-----------
#define RXpin Serial1  //reciever pin assignment
IBusBM IBus;    // IBus object
const int NUM_CHANNELS = 6; //number of reciever channels to use

//------Driving characteristcs------
const int LEDheading = 330; //degree where the LED heading is centered, adjust for tuning heading vs driving direction
const int percentdecel = 20; //percentage of rotation the translation deceleration wave occurs for each motor. Should be <= 50
//const int accel_ag = 45; //0(least aggressive) - 100(most aggressive)
//const float accel_speed = (101 - accel_ag) * 0.02;

//-------Wifi config---------
const char *ssid = "Beyblade"; //wifi ssid
const char *password = "meltybrain"; //wifi password
// Start TelnetStream: Enter telnet 192.168.4.1 in CMD to view output

//-----other constants-------
const int denom = round(1 / sqrt(0.00001118*accradius)); //calculates denominator ahead of time to reduce unnecessary calculations;
const int OFFSET_ADDR = 0;  // EEPROM address to store rpm offset
const int WDT_TIMEOUT = 5;  // seconds

//=============GLOBAL VARIABLES==================
//-------LED control-------
String LEDStatus = "armed"; //tells LED control what mode to be in
unsigned long lastUpdate = 0;   // last LED animation update time
long stripstart = 0;   //time of last update
int animIndex = 0;     // position in LED animation
int direction = 1; // 1 = forward, -1 = backward
//-------rpm and heading calc--------
int rpm;
unsigned long long previoustime = 0; //will store last time that was updated
float heading = 1.0; //variable used to adjust heading using left transmitter stick
float offset = 1.0; //creates variable to tune a stable heading, will update from EEPROM in setup
int angle = 0; //creates a variable to track the rotation angle of the bot from its heading
int max_rpm = 0;  // stores the greatest rpm achieved
int max_gforce = 0; //for storing max gforce achieved
//--------translation----------
unsigned long long dtime; //variable for the time it's been since decel start
unsigned long long startime; //variable to store the time in microseconds that the decel started
bool startimeset = false;
bool off_set = false;
bool reversed = false;  //used for reverse spin direction
unsigned long long motorLsent = 0; //motor send timers
unsigned long long motorRsent = 1; //offsets it slightly from other motor
bool motorRsend = true;  //used to switch motor gets updated
unsigned long loopstart = 0; //stores last gforce read time
int motorL;
int motorR;
int spinspeed;
bool motorLsend = true;
//--------reciever----------
unsigned long chanstart = 0; //stores last RX read time
int pwm[NUM_CHANNELS]; //list values from 1000-2000
int duty[NUM_CHANNELS]; //list values from 0 - 100
int last_rec; //used to store update times for detecting signal loss
int rec_gap;
int rec_last;
bool rc_status = false; //whether getting rc signal, used for triggering failsafe
//----------other----------
float volts; //variable to store current battery voltage
int batloop; //variable for tracking battery update loop
long loop_time;
long loop_start;

//--------------------------------------------------------------------------------------------
void setup() 
{
  EEPROM.begin(512);  // Initialize EEPROM with 512 bytes of storage (adjust size if needed)
  offset = readOffsetFromEEPROM();
  delay(3000);
  Wire.begin();        //not sure what this does
  Wire.setClock(400000);  //makes reading the sensor faster somehow
  xl.setI2CAddr(ACCEL_I2C_ADDRESS); //has to do with the g force sensor
  xl.begin(LIS331::USE_I2C);  //starts I2C com with sensor
  xl.setFullScale(ACCEL_RANGE); //sets sensor scale
  IBus.begin(RXpin, IBUSBM_NOTIMER);    // iBUS object connected to RX pin and disables interrupt timer
  top_strip.begin(); //initializes LEDs
  bottom_strip.begin();
  top_strip.clear();  // Turn all LEDs off ASAP
  bottom_strip.clear();
  ArduinoOTA.setHostname("esp32-ap"); //wifi ota config
  ArduinoOTA.setPassword("admin"); //enter this if window opens in arduino IDE asking for pswrd
  escR.install(escR_gpio, RMT_CHANNEL_3); //associates pins and RMT channels to esc objects
  escL.install(escL_gpio, RMT_CHANNEL_2);
  escR.init();
	escR.setReversed(false);
	escR.set3DMode(true);
  delayMicroseconds(200);
  escL.init();
	escL.setReversed(false);
	escL.set3DMode(true);
  for (int i = 0; i < 2; i++)
	{
		escR.beep(i);
    delay(1);
    escL.beep(i);
    delay(1);
	}
  esp_task_wdt_init(WDT_TIMEOUT, true);  // Enable panic so ESP32 restarts, esp_task_wdt_reset(); feeds watchdog
  esp_task_wdt_add(NULL);  // Add current thread (loopTask) to WDT
  //Serial.print("Setup Complete");
  
}

//----------------------------------------------------------------------------------------
//functions

void get_battery()  //calculates battery voltage based on voltage divider input on pin A0
{
  if ((millis() - batloop) > 1000) //samples every second to avoid flickering
  {
    volts = (float(analogRead(volt_pin)) / float(310.0)) * 1.29;
    batloop = millis();
  }
}

void failsafe() //failsafe mode, shuts off all motors
{
      LEDStatus = "failsafe";
      motorR = 0;
      motorL = 0;
      if(rc_status)
      {
        update_motors();
      }
      updateLED();
}

//----------------------------------------------------------------------------------------
void loop() 
{
  get_battery();
  update_channels();
  heading_adj();
  heading_funct();
  calcrpm();
  if(duty[5] > 50)
  {
    wifi_mode();
  }
  if(duty[6] > 50)
  {
    reversed = true;
  }
  else
  {
    reversed = false;
  }
    if(rpm > 400)
    {
     angle = rotation_angle();
     if (isAngleInRange(LEDheading, 10)) // LED turns on within ±10° of heading
     {
       LEDStatus = "heading on";
     }
     else //turns off led at 50 degree rotation
     {
       LEDStatus = "heading off";
     }
     if(duty[2] > 55 || duty[2] < 45)
     {
      translate();
     }
     else 
     {
      spin();
     }
    
    } //-------------------------------------------
    else if(duty[3] > 10)                              //spin command
    {
      spin();
    }
    else if(reversed == true) //unstick using the right stick
    {
      motorL = map(duty[2], 0, 100, -1000, 1000);
      motorR = map(duty[2], 0, 100, 1000, -1000);
    }
    else //tank drive mode
    {
      motorL = map(duty[2], 0, 100, -100, 100) + map(duty[1], 0, 100, -40, 40);
      motorR = map(duty[2], 0, 100, 100, -100) + map(duty[1], 0, 100, -40, 40);
    }
    if(rpm < 400)
    {
      LEDStatus = "armed";
    }
  updateLED();
  loop_time = esp_timer_get_time() - loop_start;
  update_motors();
  loop_start = esp_timer_get_time();

}
