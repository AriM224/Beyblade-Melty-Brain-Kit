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
#include <DShotESC.h>
#include <EEPROM.h>

#define WDT_TIMEOUT 5  // seconds
#define ACCEL_RANGE LIS331::HIGH_RANGE  //sets to 400g range
#define ACCEL_MAX_SCALE 400
#define ACCEL_I2C_ADDRESS 0x19
#define top_led_pin A3  //top led strip pin
#define bottom_led_pin A2  //bottom led strip pin
#define NUMPIXELS 10 //number of leds in the strip
#define volt_pin A0 //pin for measure battery voltage

const char *ssid = "Brain_Rot"; //wifi ssid
const char *password = "meltybrain"; //wifi password
//use WiFi.softAP(ssid, password); to start wifi
//use ArduinoOTA.begin(); ti start ota service
//use ArduinoOTA.handle(); to update ota?
// Start TelnetStream: Enter telnet 192.168.4.1 in CMD to view output
//TelnetStream.begin(); //starts
//TelnetStream.stop(); //stops
//TelnetStream.print("stuff"); to print to telnet
IBusBM IBus;    // IBus object
//use IBus.readChannel(0); to read channel 1 value
//use IBus.loop(); to update channels (only if interrupt timer is disabled)
//returns a value from 1000-2000
LIS331 xl;
Adafruit_NeoPixel top_strip(NUMPIXELS, top_led_pin, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel bottom_strip(NUMPIXELS, bottom_led_pin, NEO_GRB + NEO_KHZ800);
//controlling LEDs:  strip.setPixelColor(Pixel #, RgbColor(255, 255, 255)); 
//index is pixel number starting from 0
//rgb values range from 0-255
//use strip.show(); to update LED
//use strip.Color(r, g, b); to get a hex
//use strip.fill(strip.Color(r, g, b), first, count); to update all LEDs (color must be hex)
//strip.clear(); turns off LED

DShotESC esc1;      // create servo object to control the left esc
DShotESC esc2;     // create servo object to control the right esc
               //library uses values between -999 and 999 to control speed of esc
              //esc1.sendThrottle3D(throttle);    //Sends the signal to the ESC
             //g force: In millimeters: G-Force = 0.0001118 x Rotor Radius x (RPM)²
const int RXpin = RX;
const int accradius = 20; //radius where the g force sensor is located in millimeters
const int NUM_CHANNELS = 6; //number of reciever channels to use
int pwm[NUM_CHANNELS];
int duty[NUM_CHANNELS];
int rpm; //gforce rpm
float gforce;
unsigned long long spintime = 100000000; //defines variable that will be used for the time in microseconds per 1 revolution
unsigned long long min_spintime = 100000000; //large number to start with 
unsigned long long currentime; //timer variable
unsigned long long previoustime = 0; //will store last time that was updated
float heading = 1.0;
int LEDheading = 320; //degree where the LED heading is centered
float offset = 1.0; //creates variable to tune heading
unsigned long long duration; //defines variable for decel duration in microseconds
int percentdecel = 20; //percentage of rotation the translation deceleration wave occurs for each motor. Should be <= 50
int transpeed; //variable to store movement speed 0-100 from ch2 duty
unsigned long long dtime; //variable for the time it's been since decel start
unsigned long long startime; //variable to store the time in microseconds that the decel started
bool startimeset = false;
bool off_set = false;
bool reversed = false;
const int OFFSET_ADDR = 0;  // EEPROM address to store the offset
int angle = 0; //creates a variable to track the rotation angle of the bot from its heading
int max_angle = 0;
//const int spintimecal = 1.06;
int denom = round(1 / sqrt(0.00001118*accradius)); //calculates denominator ahead of time to reduce unnecessary calculations;
int max_rpm = 0;  // stores the greatest rpm achieved
int max_gforce = 0; //for storing max gforce achieved
unsigned long lastRevolutionTime = 0;
unsigned long currentRevolutionTime = 0;
unsigned long long motor1sent = 0; //motor send timers
unsigned long long motor2sent = 1;
unsigned long loopstart = 0;
unsigned long chanstart = 0;
int motor1;
int motor2;
int spinspeed;
//int motor1lastsent;
//int motor2lastsent;
bool motor1send = true;
bool exportdata = false;
String LEDStatus = "armed";
String LEDColor = "green";
unsigned long lastUpdate = 0;   // last LED animation update time
int animIndex = 0;              // position in LED animation
int animSpeed = 100;            // ms between LED updates
int direction = 1; // 1 = forward, -1 = backward
int last_rec;
int rec_gap;
int rec_last;
float volts;

//--------------------------------------------------------------------------------------------
void setup() 
{
  // put your setup code here, to run once:
  //ESC1.attach(esc1pin, 1000, 2000); // (pin, min pulse width, max pulse width in microseconds) 
  //ESC2.attach(esc2pin, 1000, 2000); // (pin, min pulse width, max pulse width in microseconds) 
  //Serial.begin(115200);
  EEPROM.begin(512);  // Initialize EEPROM with 512 bytes of storage (adjust size if needed)
  offset = readOffsetFromEEPROM();
  delay(3000);
  Wire.begin();        //not sure what this does
  Wire.setClock(400000);  //makes reading the sensor faster somehow
  xl.setI2CAddr(ACCEL_I2C_ADDRESS); //has to do with the g force sensor
  xl.begin(LIS331::USE_I2C);  //begins reading it?
  xl.setFullScale(ACCEL_RANGE); //sets sensor scale
  IBus.begin(Serial1, IBUSBM_NOTIMER);    // iBUS object connected to serial RX pin and disables interrupt timer
  top_strip.begin(); //initializes LEDs
  bottom_strip.begin();
  top_strip.clear();  // Turn all LEDs off ASAP
  bottom_strip.clear();
  ArduinoOTA.setHostname("esp32-ap"); //wifi ota config
  ArduinoOTA.setPassword("admin"); //enter this if window opens in arduino IDE asking for pswrd
  esc1.install(GPIO_NUM_36, RMT_CHANNEL_2); //defines esc pins
  esc2.install(GPIO_NUM_37, RMT_CHANNEL_3);
  esc1.init();
	esc1.setReversed(false);
	esc1.set3DMode(true);
  delayMicroseconds(200);
  esc2.init();
	esc2.setReversed(false);
	esc2.set3DMode(true);
  for (int i = 0; i < 2; i++)
	{
		esc1.beep(i);
    delay(1);
    esc2.beep(i);
    delay(1);
	}
  esp_task_wdt_init(WDT_TIMEOUT, true);  // Enable panic so ESP32 restarts, esp_task_wdt_reset(); feeds watchdog
  esp_task_wdt_add(NULL);  // Add current thread (loopTask) to WDT
  //Serial.print("Setup Complete");
  
}

//----------------------------------------------------------------------------------------
//functions
int calcrpm()  // calculates the rpm based on g-force
{
  // Sample current g-force (integer assumed)
  gforce = fabs(get_accel_force_g());
  rpm = round(sqrt(gforce) * denom);
  //last_valid_rpm = rpm;

  // Update max_rpm if this is the highest so far
  if(rpm > max_rpm) 
  {
    max_rpm = rpm;
  }

  return rpm;
}

long long spintimefunct()
{
  rpm = calcrpm(); //was calcrpmIR();
  if(rpm > 200)
  {
    spintime = (60000000LL / rpm) * offset * heading; //calculates microseconds per revolution
    if (spintime < min_spintime) 
    {
      min_spintime = spintime;
    }
  }
  else
  {
    spintime = 1000; //some random placeholder
  }
  return spintime;
}
float get_accel_force_g() {
  static const int NUM_SAMPLES = 10;  // average over last 10 readings
  static float samples[NUM_SAMPLES];
  static int index = 0;
  static bool filled = false;
  static float total = 0.0;

  if ((esp_timer_get_time() - loopstart) > 20000) {
    int16_t x, y, z;
    xl.readAxes(x, y, z);
    gforce = xl.convertToG(ACCEL_MAX_SCALE, y);
    loopstart = esp_timer_get_time();

    // update rolling average
    total -= samples[index];
    samples[index] = gforce;
    total += gforce;

    index = (index + 1) % NUM_SAMPLES;
    if (index == 0) filled = true;

    if (round(gforce) > max_gforce) {
      max_gforce = round(gforce);
    }
  }

  // return current average
  if (filled) return total / NUM_SAMPLES;
  else return total / (index + 1); // partial average while filling buffer
}
int rotation_angle()
{
  spintime = spintimefunct();
  currentime = esp_timer_get_time();
  //angle = ((currentime - ((irVisibleEnd + irVisibleStart) / 2) ) / spintime) * 360;
  angle = ((double)(currentime - previoustime) / (double)spintime) * 360.0;
  if((currentime - previoustime) >= spintime)  //resets timer every revolution
  {
    previoustime = currentime;
  }
  else if(angle > max_angle)
  {
    max_angle = angle;
  }
  return angle;
}
void update_channels()
{
  esp_task_wdt_reset();  // feeds the watchdog
  if((millis() - chanstart) > 10)
  {
    IBus.loop(); //triggers instead of interrupts
  for (int channel = 1; channel <= NUM_CHANNELS; channel++) 
  {
    pwm[channel] = IBus.readChannel(channel - 1);
    duty[channel] = map(pwm[channel], 1000, 2000, 0, 100); //channels can be accessed by calling duty[0] for channel 1 etc
  }
  if(IBus.cnt_rec == last_rec)  //failsafe
  {
    rec_gap = millis() - rec_last;
    if(rec_gap > 500)
    {
      duty[1] = 50;
      duty[2] = 50;
      duty[3] = 0;
      duty[4] = 50;
      duty[5] = 100;
      duty[6] = 0;
    }
  }
  else
  {
    last_rec = IBus.cnt_rec;
    rec_last = millis();
  }
  chanstart = millis();
  }
  
}
void get_battery()  //calculates battery voltage based on voltage divider input on pin A0
{
  volts = (float(analogRead(volt_pin)) / float(310.0)) * 1.29;
}

void data_export()    //exports data to telnet client for diagnostics, wifi mode must be turned on first
{
  //Serial.println("data export");
  TelnetStream.begin(); //start telnet
  delay(2000);
    // Print data
  TelnetStream.println("Telnet stream started");
  TelnetStream.println("==== Sensor Data Export ====");
  TelnetStream.print("Max RPM: ");
  TelnetStream.println(max_rpm);
  TelnetStream.print("heading offset: ");
  TelnetStream.println(offset);
  TelnetStream.print("battery: ");
  TelnetStream.println(volts);
  TelnetStream.print("Max G-Force: ");
  TelnetStream.println(max_gforce);
  TelnetStream.println("============================");
  TelnetStream.stop(); // Close telnet connection
}
void wifi_mode()   //turns on wifi mode to connect wirelessly
{
  failsafe();
  esp_task_wdt_init(WDT_TIMEOUT, false);  // disable watchdog
  WiFi.softAP(ssid, password);
  //Serial.print("Wifi Mode");
  ArduinoOTA.begin();  //starts ota
  while(duty[5] > 50)  //stuck in loop so robot cannot run while in wifi mode, channel 5 initiates wifi
  {
    failsafe();
    update_channels();
    ArduinoOTA.handle();
    if(duty[6] < 50)  //if export switch is not triggered, will not export
    {
      exportdata = false;
    }
    if((duty[6] > 50) && (exportdata == false)) //if export switch is triggered and it has not exported yet, call export function
    {
      LEDStatus = "white";
      updateLED();
      data_export();
      exportdata = true;
      esc1.init();
      delayMicroseconds(200);
      esc2.init();
      for (int i = 0; i < 2; i++)
	    {
        delay(1);
		    esc1.beep(i);
        delay(1);
        esc2.beep(i);
	    }
    }
  }
  WiFi.softAPdisconnect(false);  //turn off wifi
  esp_task_wdt_init(WDT_TIMEOUT, true);  // enable watchdog
}
void failsafe() //failsafe mode, shuts off all motors
{
      LEDStatus = "failsafe";
      motor2 = 0;
      motor1 = 0;
      update_motors();
      updateLED();
}

//----------------------------------------------------------------------------------------
void loop() 
{
  update_channels();
  heading_adj();
  heading_funct();
  get_battery();
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
    if(calcrpm() > 400)
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
      motor1 = map(duty[2], 0, 100, -1000, 1000);
      motor2 = map(duty[2], 0, 100, 1000, -1000);
    }
    else //tank drive mode
    {
      motor1 = map(duty[2], 0, 100, -100, 100) + map(duty[1], 0, 100, -40, 40);
      motor2 = map(duty[2], 0, 100, 100, -100) + map(duty[1], 0, 100, -40, 40);
    }
    if(rpm < 400)
    {
      LEDStatus = "armed";
    }
  updateLED();
  update_motors();

}
