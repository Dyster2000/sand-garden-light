/*
This file is part of Sand-Garden-Light.

Sand-Garden-Light is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation version 3 or later.

Sand-Garden-Light is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with Sand-Garden-Light. If not, see <https://www.gnu.org/licenses/>.
*/
#include <Arduino.h>
#include "LedDisplay.h"

#define LED_FADE_PERIOD  1000        //Amount of time in milliseconds it takes for LEDs to fade on and off.

void LedDisplay::setup()
{
  Controller = &FastLED.addLeds<WS2812B, LED_DATA_PIN, GRB>(leds, NUM_LEDS);
  clear();
  Controller->showLeds(MAX_BRIGHTNESS);
}

void LedDisplay::clear()
{
  for (int i = 0; i < NUM_LEDS; i++)
    leds[i] = CRGB::Black;
}

void LedDisplay::setMaxBrightness()
{
  setBrightness(MAX_BRIGHTNESS);
}

void LedDisplay::setBrightness(uint8_t val)
{
  FastLED.setBrightness(val);
}

void LedDisplay::indicatePattern(uint8_t value)           //used for showing which pattern is selected
{
  clear();
  if (value == 255)                                       //pattern255 is the manual drawing mode.
  {
    clear();
    leds[0] = CRGB::DarkCyan;
  }
  else
  {                                                       //all other patterns can be displayed with bitwise operations
    for (int i = 0; i < NUM_LEDS; i++)                    //iterate through each LED in the array
    {
      if (value & (1 << i))                               //bitwise AND the value of each bit in the pattern number to determine if current LED needs to be turned on. 
      {
        leds[NUM_LEDS - 1 - i] = CRGB::MediumVioletRed;   //turn on the LED if needed
      }
    }
  }
  Controller->showLeds(MAX_BRIGHTNESS);                                         //display the LEDs
}

void LedDisplay::fadePixels()
{
  fadePixels(LED_FADE_PERIOD, MAX_BRIGHTNESS);
}

void LedDisplay::fadePixels(unsigned long period, uint8_t maxBrightness)
{
  unsigned long currentTime = millis();
  unsigned long timeInCycle = currentTime % period; // Time position in current cycle
  unsigned long halfPeriod = period / 2;
  int brightness;

  // Determine phase and calculate brightness
  if (timeInCycle < halfPeriod)
  {
    // Fading in
    brightness = map(timeInCycle, 0, halfPeriod, 0, maxBrightness);
  }
  else
  {
    // Fading out
    brightness = map(timeInCycle, halfPeriod, period, maxBrightness, 0);
  }

  // Apply calculated brightness to all LEDs
  Controller->showLeds(brightness);
}

void LedDisplay::homingSequence(bool homingComplete)
{
  static unsigned long lastUpdate = 0;

  const byte fadeAmount = 150;
  const int ballWidth = 2;
  const int deltaHue  = 4;

  static byte hue = HUE_RED;
  static int direction = 1;
  static int position = 0;
  static int multiplier = 1;

  if (!homingComplete)                       //If the homing sequence is not complete, animate this pattern.
  {
    if (millis() - lastUpdate >= 100)
    {
      hue += deltaHue;
      position += direction;

      if (position == (NUM_LEDS - ballWidth) || position == 0) direction *= -1;

      for (int i = 0; i < ballWidth; i++)
      {
        leds[position + i].setHue(hue);
      }

      // Randomly fade the LEDs
      for (int j = 0; j < NUM_LEDS; j++)
      {
        leds[j] = leds[j].fadeToBlackBy(fadeAmount);  
      }
      Controller->showLeds(MAX_BRIGHTNESS);
      lastUpdate = millis();
    }
  }
  else                                     //if the homing sequence is complete, indicate that by flashing the LEDs briefly.
  {
    for (int i = 0; i < NUM_LEDS; i++)
    {
      leds[i] = CRGB::Green;
    }
    
    for (int j = 0; j < 8; j++)
    {
      Controller->showLeds(constrain(MAX_BRIGHTNESS * multiplier, 0, MAX_BRIGHTNESS));
      multiplier *= -1;
      delay(100);
    }
  }
}
