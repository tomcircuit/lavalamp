/* 
  LED LAVA LAMP test program #3 - Dimmable  
  January 19, 2023
  Tom LeMense

 */

#include <Arduino.h>
#include <APA102.h>
#include <ESP8266WiFi.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>         // https://github.com/tzapu/WiFiManager
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

// 128 entry SINE lookup table -- AMPL = 120, OFFSET = 135
uint8_t sinetbl[128] = {
  0x87, 0x8C, 0x92, 0x98, 0x9E, 0xA4, 0xA9, 0xAF, 
  0xB4, 0xBA, 0xBF, 0xC4, 0xC9, 0xCE, 0xD3, 0xD7, 
  0xDB, 0xDF, 0xE3, 0xE7, 0xEA, 0xED, 0xF0, 0xF3, 
  0xF5, 0xF7, 0xF9, 0xFB, 0xFC, 0xFD, 0xFE, 0xFE, 
  0xFF, 0xFE, 0xFE, 0xFD, 0xFC, 0xFB, 0xF9, 0xF7,  // sine[0x20] MAX
  0xF5, 0xF3, 0xF0, 0xED, 0xEA, 0xE7, 0xE3, 0xDF, 
  0xDB, 0xD7, 0xD3, 0xCE, 0xC9, 0xC4, 0xBF, 0xBA, 
  0xB4, 0xAF, 0xA9, 0xA4, 0x9E, 0x98, 0x92, 0x8C, 
  0x87, 0x82, 0x7C, 0x76, 0x70, 0x6A, 0x65, 0x5F, 
  0x5A, 0x54, 0x4F, 0x4A, 0x45, 0x40, 0x3B, 0x37, 
  0x33, 0x2F, 0x2B, 0x27, 0x24, 0x21, 0x1E, 0x1B, 
  0x19, 0x17, 0x15, 0x13, 0x12, 0x11, 0x10, 0x10, 
  0x0F, 0x10, 0x10, 0x11, 0x12, 0x13, 0x15, 0x17, // sine[0x60] MIN
  0x19, 0x1B, 0x1E, 0x21, 0x24, 0x27, 0x2B, 0x2F, 
  0x33, 0x37, 0x3B, 0x40, 0x45, 0x4A, 0x4F, 0x54, 
  0x5A, 0x5F, 0x65, 0x6A, 0x70, 0x76, 0x7C, 0x82
};

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

// Set web server port number to 80
WiFiServer server(80);

struct ColorTuple {
  uint16_t r;
  uint16_t g;
  uint16_t b
};

struct ColorPlan {
  String name;
  uint8_t efftyp;     // 0 = fixed, 1 = sine, 2 = random walk
  ColorTuple init;
  ColorTuple effect;
  bool gamma
};

struct BrightPlan {
  String name;
  uint8_t efftyp;     // 0 = fixed, 1 = fade, 2 = stars
  uint16_t init;
  uint16_t effect
};

ColorPlan colorPlan[12] = {
  { //0
    .name = "Fast",
    .efftyp = 1,
    .init.r = 111,
    .init.g = 86,
    .init.b = 98,
    .effect.r = 125,
    .effect.g = 93,
    .effect.b = 26,
    .gamma = true
  },
  { //1
    .name = "Medium",
    .efftyp = 1,
    .init.r = 111,
    .init.g = 86,
    .init.b = 98,
    .effect.r = 62,
    .effect.g = 47,
    .effect.b = 13,
    .gamma = true
  },
  { //2
    .name = "Slow",
    .efftyp = 1,
    .init.r = 111,
    .init.g = 86,
    .init.b = 98,
    .effect.r = 31,
    .effect.g = 23,
    .effect.b = 7,
    .gamma = true
  },
  { //3
    .name = "Glacial",
    .efftyp = 1,
    .init.r = 111,
    .init.g = 86,
    .init.b = 98,
    .effect.r = 15,
    .effect.g = 11,
    .effect.b = 3,
    .gamma = true
  },
  { //4
    .name = "Lamp",
    .efftyp = 0,
    .init.r = 255,
    .init.g = 255,
    .init.b = 255,
    .gamma = false
  }
};
uint8_t lastColorPlan = 4;
uint8_t curColorPlan = 0;

BrightPlan brightPlan[16] = {
  { //0
    .name = "Dim",
    .efftyp = 0,
    .init = 11
  },
  { //1
    .name = "Normal",
    .efftyp = 0,
    .init = 19
  },
  { //2
    .name = "Solar",
    .efftyp = 0,
    .init = 31
  }
};
uint8_t lastBrightPlan = 2; // highest numbered valid BrightPlan entry
uint8_t curBrightPlan = 0;  // initial BrightPlan number



uint8_t button_deb;         // user button debounce timer
int8_t button_st;           // user button state
uint32_t prev_ms = 0;       // for non-blocking delay
uint32_t curr_ms;           // for non-blocking delay

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

// set only one LED to an RGBI color
void colorOneLED(uint8_t red,uint8_t green,uint8_t blue, uint8_t bright) {
  ledStrip.startFrame();
  ledStrip.sendColor(0,0,0,0);
  ledStrip.sendColor(0,0,0,0);
  ledStrip.sendColor(red,green,blue,bright);
  ledStrip.sendColor(0,0,0,0);
  ledStrip.sendColor(0,0,0,0);
  ledStrip.endFrame(1);
}

void printWifiStatus() {
  // print the SSID of the network to witch we are attached
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());
  // print the WiFi IP address assigned to us
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
  // print the received signal strength
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

void sendHTMLpage(WiFiClient client) {
  // send a standard http response header
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println("Connection: close");
  client.println();
  // Display the HTML web page

  client.println("<!DOCTYPE HTML>");
  client.println("<html>");
  client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
  client.println("<link rel=\"icon\" href=\"data:,\">");
  
  // CSS to style the on/off buttons 
  client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
  client.println(".button { background-color: #195B6A; border: none; color: white; padding: 16px 40px;");
  client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
  client.println(".button2 {background-color: #77878A;}</style></head>");
    
  // Web Page Heading
  client.println("<body><h1>Night Light Web Server</h1>");
    
  // Display current DIM LEVEL and DISPLAY MODE
  client.println("<p>MODE - " + colorPlan[curColorPlan].name + " - " + brightPlan[curBrightPlan].name + "</p>");

  // display all of the MODE buttons
  for(uint8_t i = 0; i <= lastColorPlan; i++) {
    client.println("<p><a href=\"/m/" + String(i) + "\"><button class=\"button\">" + colorPlan[i].name + "</button></a></p>");
  }

  // display all of the BRIGHTNESS buttons
  for(uint8_t i = 0; i <= lastBrightPlan; i++) {
    client.println("<p><a href=\"/b/" + String(i) + "\"><button class=\"button\">" + brightPlan[i].name + "</button></a></p>");
  }

  // end of HTML webpage
  client.println("</body></html>");
}

void processHTMLresponse(String line) {
  // first, look for MODE selection
  if (line.indexOf("GET /m/") >= 0) {
    uint8_t index, value;
    Serial.println("color plan change");
    index = line.indexOf("GET /m/");
    index += 7;
    value = int(line.charAt(index))-int('0');
    Serial.print("converted value:");
    Serial.println(value);

    if ((value >= 0) && (value <= lastColorPlan)) {
      curColorPlan = value;
      Serial.println(curColorPlan);
    }
  }
  //next, look for BRIGHT selections
  if (line.indexOf("GET /b/") >= 0) {
    uint8_t index, value;
    Serial.println("bright plan change");
    index = line.indexOf("GET /b/");
    index += 7;
    value = int(line.charAt(index))-int('0');
    Serial.print("converted value:");
    Serial.println(value);

    if ((value >= 0) && (value <= lastBrightPlan)) {
      curBrightPlan = value;
      Serial.println(curBrightPlan);
    }
  }
}

void setup() {
  uint8_t flush_WiFi_settings = FALSE;

  // Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;

  Serial.begin(115200);
  Serial.println("\r\nLED LAVA LAMP V3 - JAN 2023");
  Serial.printf("%u LEVEL DIMMING\r\n", lastBrightPlan+1);
  Serial.println("GAMMA CORRECTION (2.5, 256)");

  // initialize BUTTON input pin
  pinMode(BUTTON,INPUT);

  // turn one LED GREEN after startup
  colorOneLED(0,128,0,15); 

  // after startup allow a 2 second interval in which 
  // the WiFi configuration is flushed (by pressing BUTTON)
  for(uint8_t i = 0; i < 100; i++) {
    button_st = digitalRead(BUTTON);
    if (button_st == LOW) {
      flush_WiFi_settings = TRUE;
      // turn one LED RED when flushing WiFi settings
      colorOneLED(128,0,0,15); 
    } 
    delay(10);
  }
  
  if (flush_WiFi_settings == TRUE) {
    // erase all the stored WiFi information
    wifiManager.resetSettings();
    ESP.restart();
  }

  // turn one LED BLUE while trying to connect
  colorOneLED(0,0,128,15); 

  // WiFi manager fetches ssid and pass from eeprom and tries to connect
  // if it does not connect it starts an access point with the name
  // "NightLightAP" and goes into a blocking loop awaiting configuration
  wifiManager.autoConnect("NightLightAP");

  // if you get here you have connected to the WiFi
  Serial.println("connected.");

  // start the WiFi server
  server.begin();

  // output the WiFi connection status
  printWifiStatus();

  // clear the button debounce timer
  button_deb = 0;
}

void loop()
{
  ColorTuple LED_phase;   // current LED phase
  ColorTuple LED_color;   // current LED color
  uint8_t LED_bright;     // current LED brightness

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
    // ...change the COLOR PLAN (advance by 1)
      if (button_deb <= BUTTON_LG_CYC) {
        button_deb = 0;
        curColorPlan++;
        // ensure disp_mode is in-bounds
        if (curColorPlan > lastColorPlan)
          curColorPlan = 0;
        // output the new COLOR PLAN value
        Serial.print("Color Plan:");
        Serial.println(curColorPlan);
      }

    // if button was held for more than BUTTON_LG_CYC cycles...
    // ...change the BRIGHT PLAN (advance by 1)
      else {
        button_deb = 0;      
        curBrightPlan++;
        // ensure disp_mode is in-bounds
        if (curBrightPlan > lastBrightPlan)
          curBrightPlan = 0;
        // unblank LED to same color at new BRIGHTNESS level 
        LED_bright = brightPlan[curBrightPlan].init & 0x31;
        colorLED(LED_color.r,LED_color.g,LED_color.b,LED_bright);          
        // output the new BRIGHT INDEX value
        Serial.print("Bright Plan:");
        Serial.println(curBrightPlan);

        // wait for button to be released
        button_st = digitalRead(BUTTON);
        while (button_st == LOW)  {
          button_st = digitalRead(BUTTON);
          delay(CYCLE_MS);  // OK this is blocking but only while button is held!
        }
      } 
    } 
    
    // ColorPlan effect type 0 -- FIXED COLOR
    if (colorPlan[curColorPlan].efftyp == 0) {
      LED_color = colorPlan[curColorPlan].init;
    }
    else if (colorPlan[curColorPlan].efftyp == 1) {
    // ColorPlan effect type 1 -- GRADIENT COLOR
    
      // advance the phase accumulators for each color
      LED_phase.r += colorPlan[curColorPlan].effect.r;
      LED_phase.g += colorPlan[curColorPlan].effect.g;
      LED_phase.b += colorPlan[curColorPlan].effect.b;

      // Obtain color values from SINE table 
      // table varies from 15 to 255 to avoid 'blackouts'
      LED_color.r = sinetbl[(LED_phase.r >> 8) & 0x7f];
      LED_color.g = sinetbl[(LED_phase.g >> 8) & 0x7f];
      LED_color.b = sinetbl[(LED_phase.b >> 8) & 0x7f];
    }


    
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

  // Listen for incoming clients
  WiFiClient client = server.accept();   
  if (client) {                             // If a new client connects,
    Serial.println("new client");
    String currentLine = "";                // make a String to hold incoming data from the client
    bool currentLineIsBlank = true;
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        // if we are at to the end of the line (received a newline
        // character) and the line is blank, the http request has ended,
        // so we should send a reply
        if (c == '\n' && currentLineIsBlank) {
          sendHTMLpage(client);
          break;
        }
        if (c == '\n') {
          // a newline char means process the response from web browser
          processHTMLresponse(currentLine);
          currentLineIsBlank = true;
          currentLine = "";
        } else if (c != '\r') {
          // collect non-return characters into currentLine
          currentLineIsBlank = false;
          currentLine += c;
        }
      }
    }
    // give the web browser time to receive the data
    delay(1);
    // Close the connection
    client.stop();
    Serial.println("client disconnected.");
    Serial.println("");
  }
}

