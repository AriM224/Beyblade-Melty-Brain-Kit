// NeoPixel Ring simple sketch (c) 2013 Shae Erisson
// Released under the GPLv3 license to match the rest of the
// Adafruit NeoPixel library

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

// Which pin on the Arduino is connected to the NeoPixels?
#define top_led_pin A3  //top led strip pin
#define bottom_led_pin A2  //bottom led strip pin

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS 10 // Popular NeoPixel ring size

// When setting up the NeoPixel library, we tell it how many pixels,
// and which pin to use to send signals. Note that for older NeoPixel
// strips you might need to change the third parameter -- see the
// strandtest example for more information on possible values.
Adafruit_NeoPixel pixels(NUMPIXELS, top_led_pin, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel pixels1(NUMPIXELS, bottom_led_pin, NEO_GRB + NEO_KHZ800);
#define DELAYVAL 1 // Time (in milliseconds) to pause between pixels

void setup() {
  // These lines are specifically to support the Adafruit Trinket 5V 16 MHz.
  // Any other board, you can remove this part (but no harm leaving it):
  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  pixels1.begin();

}

void loop() {
  pixels.clear(); // Set all pixel colors to 'off'
  pixels1.clear();
  pixels.fill(pixels.Color(255, 255, 0), 0, 10);
  pixels1.fill(pixels.Color(255, 255, 0), 0, 10);
  pixels.show();   // Send the updated pixel colors to the hardware.
  pixels1.show();
delay(5);
}
