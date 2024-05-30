/* LED LAVA LAMP test program #2 - APA102 access  
 * Make an LED pattern with a large dynamic range using the 
 * extra 5-bit brightness register in the APA102.
 *
 * It sets every LED on the strip to white, with *a dim*
 * white at the input end of the strip and the brightest
 * possible white at the other end, and a smooth logarithmic
 * gradient between them.
 *
 * The *dim* white is achieved by setting the red,
 * green, and blue color channels to 1, and setting the
 * brightness register to 1.  The brightest possibe white is
 * achieved by setting the color channels to 255 and setting the
 * brightness register to 31.
 */

#include <Arduino.h>
#include <APA102.h>
#include "LittleFS.h"

// Define how many display modes are supported
#define MODE_MAX (7)

// define how many custom slots are supported
#define CUSTOM_SLOTS (4)

// Define how many LED are in the chain
#define LED_COUNT (5)

// Define the display update cycle in ms
#define CYCLE_MS (200)

// Define the USER BOTTON input pin
#define BUTTON (12)

// Define a SHORT PRESS of USER BUTTON in ms
#define BUTTON_SHORT_PRESS_MS (1000)
#define BUTTON_SH_CYC (BUTTON_SHORT_PRESS_MS / CYCLE_MS)

// Define a LONG PRESS of USER BUTTON in ms
#define BUTTON_LONG_PRESS_MS (5000)
#define BUTTON_LG_CYC (BUTTON_LONG_PRESS_MS / CYCLE_MS)

// Define the APA102 data and clock pins
const uint8_t dataPin = 13;
const uint8_t clockPin = 14;

// Create an object for writing to the LED strip.
APA102<dataPin, clockPin> ledStrip;

// 128 entry SINE lookup table -- AMPL = 120, OFFSET = 120
uint8_t sinetbl[128] = {
   0x78, 0x7D, 0x83, 0x89, 0x8F, 0x94, 0x9A, 0xA0,
   0xA5, 0xAA, 0xB0, 0xB5, 0xBA, 0xBE, 0xC3, 0xC7,
   0xCC, 0xD0, 0xD3, 0xD7, 0xDA, 0xDE, 0xE0, 0xE3,
   0xE5, 0xE8, 0xE9, 0xEB, 0xEC, 0xED, 0xEE, 0xEE,
   0xEF, 0xEE, 0xEE, 0xED, 0xEC, 0xEB, 0xE9, 0xE8,		// sine[0x20] MAX
   0xE5, 0xE3, 0xE0, 0xDE, 0xDA, 0xD7, 0xD3, 0xD0,
   0xCC, 0xC7, 0xC3, 0xBE, 0xBA, 0xB5, 0xB0, 0xAA,
   0xA5, 0xA0, 0x9A, 0x94, 0x8F, 0x89, 0x83, 0x7D,
   0x78, 0x73, 0x6D, 0x67, 0x61, 0x5C, 0x56, 0x50,
   0x4B, 0x46, 0x40, 0x3B, 0x36, 0x32, 0x2D, 0x29,
   0x24, 0x20, 0x1D, 0x19, 0x16, 0x12, 0x10, 0x0D,
   0x0B, 0x08, 0x07, 0x05, 0x04, 0x03, 0x02, 0x02,
   0x00, 0x02, 0x02, 0x03, 0x04, 0x05, 0x07, 0x08,		// sine[0x60] MIN
   0x0B, 0x0D, 0x10, 0x12, 0x16, 0x19, 0x1D, 0x20,
   0x24, 0x29, 0x2D, 0x32, 0x36, 0x3B, 0x40, 0x46,
   0x4B, 0x50, 0x56, 0x5C, 0x61, 0x67, 0x6D, 0x73 };

uint8_t red_ph_inc[4] = {125, 62, 31, 15};     // phase increment for RED 
uint8_t green_ph_inc[4] = {93, 47, 23, 17};	  // phase increment for GREEN 
uint8_t	blue_ph_inc[4] = {26, 13, 7, 3};	  // phase increment for BLUE 
uint8_t bright_ph_inc[4] = {0, 0, 0, 0};  // phase increment for BRIGHTNESS

uint8_t red_fixed_val[4] = {255, 255, 255, 255};
uint8_t green_fixed_val[4] = {255, 255, 255, 255};
uint8_t blue_fixed_val[4] = {255, 255, 255, 255};
uint8_t bright_fixed_val[4] = {7, 15, 23, 31};

uint8_t red_val;		    // value from lookup table RED
uint8_t green_val;		  // value from lookup table BLUE
uint8_t blue_val;		    // value from lookup table GREEN
uint8_t bright_val;     // value from lookup table BRIGHTNESS

uint16_t red_phase;     // phase accumulator for RED
uint16_t green_phase;   // phase accumulator for GREEN
uint16_t blue_phase;    // phase accumulator for BLUE
uint16_t bright_phase;  // phase accumulator for BRIGHTNESS

uint8_t  disp_mode;     // mode number (0...3)
uint8_t  custom_idx;    // custom index (0...3)
uint8_t  button_deb;    // user button debounce timer
int8_t  button_st;      // user button state

uint32_t prev_ms = 0;   // for non-blocking delay
uint32_t curr_ms;       // for non-blocking delay

// turn OFF all of the LED by setting RBGI = 0000
void blankLED() {
  ledStrip.startFrame();
  for(uint8_t i = 0; i < LED_COUNT; i++)
    ledStrip.sendColor(0,0,0,0);
  ledStrip.endFrame(LED_COUNT);
}

// make all the LED the same RGBI color
void colorLED(uint8_t red,uint8_t green,uint8_t blue, uint8_t bright) {
  ledStrip.startFrame();
  for(uint8_t i = 0; i < LED_COUNT; i++)
    ledStrip.sendColor(red,green,blue,bright);
  ledStrip.endFrame(LED_COUNT);
}

void setup() {
  Serial.begin(115200);
  Serial.println("LED LAVA LAMP V3 - JAN 2023");

  // init phase indices 
  red_phase = 111 << 8;
  green_phase = 86 << 8;
  blue_phase = 98 << 8;
  bright_phase = 0 << 8;

  // init color values
  red_val = 0;
  green_val = 0;
  blue_val = 0;
  bright_val = 0;

  // select mode 0 (FAST) at startup
  disp_mode = 0;

  // initialize customer color index
  custom_idx = 0;

  // initialize button debounce timer
  button_deb = 0;

  // initialize button input
  pinMode(BUTTON,INPUT);
}

void loop()
{
  // non-blocking delay for display update / button debounce cycle
  curr_ms = millis();
  if (curr_ms - prev_ms > CYCLE_MS) {
    //Serial.println(curr_ms - prev_ms);
    prev_ms = curr_ms;

    //increment or clear button hold counter depending on state 
    button_st = digitalRead(BUTTON);
    if (button_st == LOW) {
      if (button_deb < 255) 
        button_deb++;
    } 
    else
      button_deb = 0;

    // once button is depressed for more than BUTTON_SH_CYC cycles, blank display
    // as long as it is held, orfor BUTTON_LG_CYC cycles, whichever comes first.
    if (button_deb > BUTTON_SH_CYC) {
      blankLED();    
      button_st = digitalRead(BUTTON);
      while ((button_st == LOW) && (button_deb <= BUTTON_LG_CYC)) {
        button_st = digitalRead(BUTTON);
        delay(CYCLE_MS);  // OK this is blocking but only while button is held!
        if (button_deb < 255) 
          button_deb++;
      }
          
    // if button was held for less than BUTTON_LG_CYC cycles, it was a SHORT PRESS
      if (button_deb <= BUTTON_LG_CYC) {
        button_deb = 0;
        disp_mode++;
        // ensure disp_mode is in-bounds
        if (disp_mode > MODE_MAX)
          disp_mode = 0;
        // output DISPLAY MODE value
        Serial.printf("\r\nDisplay mode : %u",disp_mode);
      }

    // if button was held for more than BUTTON_LG_CYC cycles, store color to
    // custom color array
      else {
        button_deb = 0;      
        // copy current values into first available custom color slot
        // note that CUSTOM COLOR SLOT 0...3 maps to DISPLAY MODES 4...7
        red_fixed_val[custom_idx] = red_val;
        green_fixed_val[custom_idx] = green_val;
        blue_fixed_val[custom_idx] = blue_val;
        bright_fixed_val[custom_idx] = bright_val;

				// flash display index number of times 
        for(uint8_t i = 0; i <= custom_idx; i++) {
          colorLED(red_val,green_val,blue_val,bright_val);
          delay(250);
          blankLED();
          delay(250);
        }  
        // report the custom color update
        Serial.printf("\r\nCustom color %u : %u %u %u %u",custom_idx,red_val,green_val,blue_val,bright_val);
        // increment the customer color slot (handle wraparound)
				custom_idx++;
				if (custom_idx >= CUSTOM_SLOTS) custom_idx = 0;	

        // wait for button to be released
        button_st = digitalRead(BUTTON);
        while (button_st == LOW)  {
          button_st = digitalRead(BUTTON);
          delay(CYCLE_MS);  // OK this is blocking but only while button is held!
        }
      } 
    } 
    
    // Advance the phase accumulators
    red_phase += red_ph_inc[disp_mode];
    green_phase += green_ph_inc[disp_mode];
    blue_phase += blue_ph_inc[disp_mode];
    bright_phase += bright_ph_inc[disp_mode];

    // Obtain color values from SINE table for modes 0...3
    if (disp_mode < 4) {
      red_val = sinetbl[(red_phase >> 8) & 0x7f];
      green_val = sinetbl[(green_phase >> 8) & 0x7f];
      blue_val = sinetbl[(blue_phase >> 8) & 0x7f];
      bright_val = sinetbl[(bright_phase >> 8) & 0x7f] >> 3;
    }

    // use "custom" fixed color values for modes 4...7
    else {
      red_val = red_fixed_val[disp_mode-4];
      green_val = blue_fixed_val[disp_mode-4];
      blue_val = blue_fixed_val[disp_mode-4];      
      bright_val = bright_fixed_val[disp_mode-4];      
    }

    // update the LED colors
    colorLED(red_val,green_val,blue_val,bright_val);
    //Serial.printf("\n\r%u %u %u %u",red_val,green_val,blue_val,bright_val);
  }
}

