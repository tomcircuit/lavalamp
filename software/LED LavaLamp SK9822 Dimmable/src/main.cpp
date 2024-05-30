/* 
  LED LAVA LAMP test program #3 - Dimmable  
  January 19, 2023
  Tom LeMense

 */

#include <Arduino.h>
#include <APA102.h>
#include "LittleFS.h"

#define TRUE (1 == 1)
#define FALSE (1 == 0)

// Define how many display modes are supported (0...n-1)
#define DISP_MODES (5)

// define how many brightness levels are supported (0..n-1)
#define BRIGHT_LEVELS (3)

// determine if GAMMA CORRECTION is used
#define GAMMA (TRUE)

// Define how many LED are in the chain (1..n)
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

// Gamma brightness lookup table <https://victornpb.github.io/gamma-table-generator>
// gamma = 2.50 steps = 256 range = 0-255
const uint8_t gamma_lut[256] = {
     0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
     0,   0,   0,   0,   0,   0,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,
     1,   2,   2,   2,   2,   2,   2,   2,   2,   3,   3,   3,   3,   3,   4,   4,
     4,   4,   4,   5,   5,   5,   5,   6,   6,   6,   6,   7,   7,   7,   7,   8,
     8,   8,   9,   9,   9,  10,  10,  10,  11,  11,  12,  12,  12,  13,  13,  14,
    14,  15,  15,  15,  16,  16,  17,  17,  18,  18,  19,  19,  20,  20,  21,  22,
    22,  23,  23,  24,  25,  25,  26,  26,  27,  28,  28,  29,  30,  30,  31,  32,
    33,  33,  34,  35,  36,  36,  37,  38,  39,  40,  40,  41,  42,  43,  44,  45,
    46,  46,  47,  48,  49,  50,  51,  52,  53,  54,  55,  56,  57,  58,  59,  60,
    61,  62,  63,  64,  65,  67,  68,  69,  70,  71,  72,  73,  75,  76,  77,  78,
    80,  81,  82,  83,  85,  86,  87,  89,  90,  91,  93,  94,  95,  97,  98,  99,
   101, 102, 104, 105, 107, 108, 110, 111, 113, 114, 116, 117, 119, 121, 122, 124,
   125, 127, 129, 130, 132, 134, 135, 137, 139, 141, 142, 144, 146, 148, 150, 151,
   153, 155, 157, 159, 161, 163, 165, 166, 168, 170, 172, 174, 176, 178, 180, 182,
   184, 186, 189, 191, 193, 195, 197, 199, 201, 204, 206, 208, 210, 212, 215, 217,
   219, 221, 224, 226, 228, 231, 233, 235, 238, 240, 243, 245, 248, 250, 253, 255,
  };   

uint16_t red_phase;     // phase accumulator for RED
uint16_t green_phase;   // phase accumulator for GREEN
uint16_t blue_phase;    // phase accumulator for BLUE

uint8_t red_ph_inc[4] = {125, 62, 31, 15};    // phase increment for RED 
uint8_t green_ph_inc[4] = {93, 47, 23, 17};	  // phase increment for GREEN 
uint8_t	blue_ph_inc[4] = {26, 13, 7, 3};	    // phase increment for BLUE 
uint8_t red_val;		    // value from lookup table RED
uint8_t green_val;		  // value from lookup table GREEN
uint8_t blue_val;		    // value from lookup table BLUE
uint8_t red_lamp_val = 255;   // value for RED in LAMP MODE
uint8_t green_lamp_val = 255; // value for GREEN in LAMP MODE
uint8_t blue_lamp_val = 255;	// value for BLUE in LAMP MODE

uint8_t bright_level[BRIGHT_LEVELS] = {31, 19, 11};        // FULL - MED - LOW brightness
uint8_t bright_idx;     // brightness level index (0...3)
uint8_t bright_val;     // value for BRIGHTNESS

uint8_t  disp_mode;     // mode number (0...3)
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
  Serial.println("\r\nLED LAVA LAMP V3 - JAN 2023");
  Serial.printf("%u LEVEL DIMMING\r\n", BRIGHT_LEVELS);
  if (GAMMA) 
    Serial.println("GAMMA CORRECTION");

  // init phase indices 
  red_phase = 44 << 8;
  green_phase = 111 << 8;
  blue_phase = 88 << 8;
  
  // init color values
  red_val = 0;
  green_val = 0;
  blue_val = 0;
  
  // select mode 0 (FAST) at startup
  disp_mode = 0;

  // initialize bright level index to 1 (FULL)
  bright_idx = 0;

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
    // as long as it is held, or for BUTTON_LG_CYC cycles, whichever comes first.
    if (button_deb > BUTTON_SH_CYC) {
      blankLED();    
      button_st = digitalRead(BUTTON);
      while ((button_st == LOW) && (button_deb <= BUTTON_LG_CYC)) {
        button_st = digitalRead(BUTTON);
        delay(CYCLE_MS);  // OK this is blocking but only while button is held!
        button_deb++;
      }
          
    // if button was held for less than BUTTON_LG_CYC cycles...
    // ...change the DISPLAY MODE (F-M-S-G-light)
      if (button_deb <= BUTTON_LG_CYC) {
        button_deb = 0;
        disp_mode++;
        // ensure disp_mode is in-bounds
        if (disp_mode >= DISP_MODES)
          disp_mode = 0;
        // output the new DISPLAY MODE value
        Serial.printf("\r\nDisplay mode : %u",disp_mode);
      }

    // if button was held for more than BUTTON_LG_CYC cycles...
    // ...adjust the BRIGHT VALUE (dim-med-full)
      else {
        button_deb = 0;      
        bright_idx++;
        // ensure disp_mode is in-bounds
        if (bright_idx >= BRIGHT_LEVELS)
          bright_idx = 0;
        // unblank LED to same color at new BRIGHTNESS level 
        bright_val = bright_level[bright_idx] & 0x31;
        colorLED(red_val,green_val,blue_val,bright_val);          
        // output the new BRIGHT INDEX value
        Serial.printf("\r\nBright level : %u",bright_idx);

        // wait for button to be released
        button_st = digitalRead(BUTTON);
        while (button_st == LOW)  {
          button_st = digitalRead(BUTTON);
          delay(CYCLE_MS);  // OK this is blocking but only while button is held!
        }
      } 
    } 
    
    //  modes 0...3 are COLOR SHIFTING modes
    if (disp_mode < 4) {

      // Advance the phase accumulators
      red_phase += red_ph_inc[disp_mode];
      green_phase += green_ph_inc[disp_mode];
      blue_phase += blue_ph_inc[disp_mode];

      // Obtain color values from SINE table for modes 0...3
      // table varies from 0 to 240, so add 15
      red_val = sinetbl[(red_phase >> 8) & 0x7f] + 15;
      green_val = sinetbl[(green_phase >> 8)& 0x7f] + 15;
      blue_val = sinetbl[(blue_phase >> 8)& 0x7f] + 15;

      if (GAMMA == TRUE) {
        red_val = gamma_lut[red_val];
        green_val = gamma_lut[green_val];
        blue_val = gamma_lut[blue_val];
      }
    }

    // mode 4 is LAMP mode
    else {
      red_val = red_lamp_val;
      green_val = green_lamp_val;
      blue_val = blue_lamp_val;      
    }

    // use the selected BRIGHT value from the table
    bright_val = bright_level[bright_idx] & 0x31;

    // update the LED colors
    colorLED(red_val,green_val,blue_val,bright_val);
    //Serial.printf("\n\r%u %u %u %u",red_val,green_val,blue_val,bright_val);
  }
}

