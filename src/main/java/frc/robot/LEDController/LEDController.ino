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

uint8_t data = 0;
uint8_t patternIndex = 1;

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
                fire(50, 100, true);
                break;
            case 2:
                staryNight();
                break;
            case 3:
                greenWhiteGold();
                break;
            case 4:
                noiseFill();
                break;
            case 5:
                rainbowBeat();
                break;
            case 6:
                movingDots();
                break;
        }
    }

    EVERY_N_SECONDS(MS_DELAY*3) {
        nextPattern();
    }

    FastLED.show();
}

void nextPattern() {
  // Change the number after the % 
  // to the number of patterns you have
  patternIndex = (patternIndex + 1) % 6;
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
  
  uint16_t idx1 = beatsin16(30/MS_DELAY, 0, NUM_LEDS - 1, 0, 0);
  uint16_t idx2 = beatsin16(50/MS_DELAY, 0, NUM_LEDS - 1, 0, 0);

  uint16_t idx3 = beatsin16(40/MS_DELAY, 0, NUM_LEDS - 1, 0, 32767);
  uint16_t idx4 = beatsin16(25/MS_DELAY, 0, NUM_LEDS - 1, 0, 32767);

  // Wave for LED color
  uint8_t color1  = beatsin8(60/MS_DELAY, 0, 255, 0, 0);
  uint8_t color2  = beatsin8(45/MS_DELAY, 0, 255, 0, 0);

  leds[idx1]  = CHSV(color1, 255, 255);
  leds[idx2]  = CHSV(color2, 255, 255);
  leds[idx3]  = CHSV(color1, 255, 255);
  leds[idx4]  = CHSV(color2, 255, 255);

  fadeToBlackBy(leds, NUM_LEDS, 5);
}
 

void rainbowBeat() {
  uint16_t beatA = beatsin16(60/MS_DELAY, 0, 255);
  uint16_t beatB = beatsin16(40/MS_DELAY, 0, 255);
  fill_rainbow(leds, NUM_LEDS, ((beatA+beatB)/2), 8);
}


void greenWhiteGold() {
  uint16_t sinBeat   = beatsin16(20/MS_DELAY, 0, NUM_LEDS-1, 0, 0);
  uint16_t sinBeat2  = beatsin16(40/MS_DELAY, 0, NUM_LEDS-1, 0, 21845);
  uint16_t sinBeat3  = beatsin16(20/MS_DELAY, 0, NUM_LEDS-1, 0, 43690);

  uint16_t sinBeat4  = beatsin16(40/MS_DELAY, 0, NUM_LEDS-1, 0, 32767);
  uint16_t sinBeat5  = beatsin16(20/MS_DELAY, 0, NUM_LEDS-1, 0, 32767+21845);
  uint16_t sinBeat6  = beatsin16(40/MS_DELAY, 0, NUM_LEDS-1, 0, 32767+43690);

  leds[sinBeat]   = CRGB::Green;
  leds[sinBeat2]  = CRGB::Yellow;
  leds[sinBeat3]  = CRGB::White;

  leds[sinBeat4]  = CRGB::Green;
  leds[sinBeat5]  = CRGB::Yellow;
  leds[sinBeat6]  = CRGB::White;
  
  fadeToBlackBy(leds, NUM_LEDS, 3);
}

// FlameHeight - Use larger value for shorter flames, default=50.
// Sparks - Use larger value for more ignitions and a more active fire (between 0 to 255), default=100.

void fire(int FlameHeight, int Sparks, boolean hot) {
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

DEFINE_GRADIENT_PALETTE (palette) {
    0, 196, 0, 255,
    90, 48, 48, 252,
    255, 0, 212, 255
};
CRGBPalette16 myPal = palette;

void staryNight() {
    // Select a random LED and illuminate it
    EVERY_N_MILLISECONDS(15) {
      leds[random8(0, NUM_LEDS - 1)] = ColorFromPalette(myPal, random8(), 255, LINEARBLEND);
    }
    // Fade all LEDs down by 1 in brightness each time this is called
    fadeToBlackBy(leds, NUM_LEDS, 3);
}

void noiseFill() {
  uint8_t octaves = 1;
  uint16_t x = 0;
  int scale = 100;
  uint8_t hue_octaves = 1;
  uint16_t hue_x = 1;
  int hue_scale = 50;
  uint16_t ntime = millis() / 3;
  uint8_t hue_shift = 5;
  
  fill_noise16 (leds, NUM_LEDS, octaves, x, scale, hue_octaves, hue_x, hue_scale, ntime, hue_shift);
}
