#include "FastLED.h"
#include "Wire.h"

#define NUM_LEDS 116
#define STRIP_1_START 0
#define STRIP_2_START 58
#define STRIP_1_LENGTH 58
#define STRIP_2_LENGTH 58
#define BRIGHTNESS 175
#define MS_DELAY 6

#define PORT 8
#define DATA_PIN 9

CRGB leds[NUM_LEDS];

int data = 0;
uint8_t patternIndex = 0;

void setup() {
    Wire.begin(PORT);
    Wire.onReceive(event);

    FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(BRIGHTNESS);
    Serial.begin(9600);
}

void loop() {
    EVERY_N_MILLISECONDS(MS_DELAY) {
        switch ((data == 0) ? patternIndex : data) {
            case 1:
                movingDots();
                break;
            case 2:
                rainbowBeat();
                break;
            case 3:
                greenWhiteGold();
                break;
            case 4:
                Fire(50, 100, true);
                break;
            case 5:
                Fire(50, 100, false);
                break;
        }
    }

    EVERY_N_SECONDS(MS_DELAY) {
        nextPattern();
    }

    FastLED.show();
}

void nextPattern() {
  // Change the number after the % 
  // to the number of patterns you have
  patternIndex = (patternIndex + 1) % 4;
  if (patternIndex == 0) patternIndex++; 
}

void event() {
  while(1 < Wire.available()) {
      char c = Wire.read();    // Receive a byte as character
      Serial.print(c);         // Print the character
  }
  data = Wire.read();
  Serial.println(data);

}

//------- Pattern dump below -------//

void movingDots() {
  
  uint16_t posBeat  = beatsin16(30/MS_DELAY, 0, NUM_LEDS - 1, 0, 0);
  uint16_t posBeat2 = beatsin16(60/MS_DELAY, 0, NUM_LEDS - 1, 0, 0);

  uint16_t posBeat3 = beatsin16(30/MS_DELAY, 0, NUM_LEDS - 1, 0, 32767);
  uint16_t posBeat4 = beatsin16(60/MS_DELAY, 0, NUM_LEDS - 1, 0, 32767);

  // Wave for LED color
  uint8_t colBeat  = beatsin8(45/MS_DELAY, 0, 255, 0, 0);

  uint8_t idx1 = ((posBeat + posBeat2) / 2);
  uint8_t idx2 = ((posBeat3 + posBeat4) / 2);

  leds[idx1]  = CHSV(colBeat, 255, 255);
  leds[idx2]  = CHSV(colBeat, 255, 255);

  fadeToBlackBy(leds, NUM_LEDS, 10);
}
 

void rainbowBeat() {
  uint16_t beatA = beatsin16(30/MS_DELAY, 0, 255);
  uint16_t beatB = beatsin16(20/MS_DELAY, 0, 255);
  fill_rainbow(leds, NUM_LEDS, ((beatA+beatB)/2), 8);
}


void greenWhiteGold() {
  uint16_t sinBeat   = beatsin16(30/MS_DELAY, 0, NUM_LEDS-1, 0, 0);
  uint16_t sinBeat2  = beatsin16(30/MS_DELAY, 0, NUM_LEDS-1, 0, 21845);
  uint16_t sinBeat3  = beatsin16(30/MS_DELAY, 0, NUM_LEDS-1, 0, 43690);

  leds[sinBeat]   = CRGB::Green;
  leds[sinBeat2]  = CRGB::Gold;
  leds[sinBeat3]  = CRGB::White;
  
  fadeToBlackBy(leds, NUM_LEDS, 10);
}

// FlameHeight - Use larger value for shorter flames, default=50.
// Sparks - Use larger value for more ignitions and a more active fire (between 0 to 255), default=100.

void Fire(int FlameHeight, int Sparks, boolean hot) {
  static byte heat[NUM_LEDS/2];
  int cooldown;
  
  // Cool down each cell a little
  for(int i = 0; i < NUM_LEDS/2; i++) {
    cooldown = random8(0, ((FlameHeight * 10) / NUM_LEDS/2) + 2);
   
    if(cooldown > heat[i]) {
      heat[i] = 0;
    }
    else {
      heat[i] = heat[i] - cooldown;
    }
  }
  
  // Heat from each cell drifts up and diffuses slightly
  for(int k = (NUM_LEDS/2 - 1); k >= 2; k--) {
    heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2]) / 3;
  }
  
  // Randomly ignite new Sparks near bottom of the flame
  if(random8() < Sparks) {
    int y = random8(7);
    heat[y] = heat[y] + random8(160, 255);
  }
  
  // Convert heat to LED colors
  for(int j = 0; j < NUM_LEDS/2; j++) {
    if (hot) {
        setPixelHeatColorWarm(58-j, heat[j]);
        setPixelHeatColorWarm(j+58, heat[j]);
    }
    else {
        setPixelHeatColorCold(58-j, heat[j]);
        setPixelHeatColorCold(j+58, heat[j]);
    }
  }
  
  FastLED.show();
}

void setPixelHeatColorWarm(int Pixel, byte temperature) {
  // Rescale heat from 0-255 to 0-191
  byte t192 = round((temperature / 255.0) * 191);
  
  // Calculate ramp up from
  byte heatramp = t192 & 0x3F; // 0...63
  heatramp <<= 2; // scale up to 0...252
  
  // Figure out which third of the spectrum we're in:
  if(t192 > 0x80) {                    // hottest
    leds[Pixel].setRGB(255, 255, heatramp);
  }
  else if(t192 > 0x40) {               // middle
    leds[Pixel].setRGB(255, heatramp, 0);
  }
  else {                               // coolest
    leds[Pixel].setRGB(heatramp, 0, 0);
  }
}

void setPixelHeatColorCold(int Pixel, byte temperature) {
  // Rescale heat from 0-255 to 0-191
  byte t192 = round((temperature / 255.0) * 191);
  
  // Calculate ramp up from
  byte heatramp = t192 & 0x3F; // 0...63
  heatramp <<= 2; // scale up to 0...252
  
  // Figure out which third of the spectrum we're in:
  if(t192 > 0x80) {                    // hottest
    leds[Pixel].setRGB(heatramp, 255, 255);
  }
  else if(t192 > 0x40) {               // middle
    leds[Pixel].setRGB(0, heatramp, 255);
  }
  else {                               // coolest
    leds[Pixel].setRGB(0, 0, heatramp);
  }
}
