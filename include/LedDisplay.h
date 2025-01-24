/*
This file is part of Sand-Garden-Light.

Sand-Garden-Light is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation version 3 or later.

Sand-Garden-Light is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with Sand-Garden-Light. If not, see <https://www.gnu.org/licenses/>.
*/
#pragma once

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
This is a class that contains all the functions and data required to handle the LED display bar.
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <FastLED.h>                  //Controls the RGB LEDs

class LedDisplay
{
public:
  LedDisplay() = default;

  void setup();
  void clear();

  void setMaxBrightness();
  void fadePixels();

/**
 * @brief Indicates the currently selected pattern by lighting up LEDs in a specific color.
 *
 * This function uses the FastLED library to light up the LEDs in a specific pattern to indicate which pattern is selected. 
 * A solid color is shown to indicate that the machine is in pattern selection mode. If the value is 255, the manual drawing 
 * mode is indicated by lighting a single LED in DarkCyan. For other pattern values, the LEDs are lit using bitwise operations 
 * to determine which LEDs should be turned on.
 *
 * @param value The current pattern number, where 255 indicates manual drawing mode, and other values indicate specific patterns.
 * 
 * @note The .fadePixels() method can be used to make the LEDs fade, indicating that the machine is running a pattern. This function 
 * uses bitwise operations to determine the LED pattern, lighting the LEDs in MediumVioletRed for non-manual patterns.
 */
  void indicatePattern(uint8_t value);

/**
 * @brief Animates an LED bouncing pattern during the homing process and flashes green when homing is complete.
 *
 * This function animates a bouncing light pattern on the LEDs to indicate that the gantry is in the process of homing. 
 * Once homing is complete, the LEDs flash green to signal completion. The function can block execution briefly during the 
 * flashing portion after homing is done.
 *
 * @param homingComplete A boolean flag indicating whether the homing process is complete. If set to false, the animation continues. 
 * If set to true, the LEDs flash green to indicate completion.
 *
 * The animation consists of a bouncing light pattern with a color that changes over time. When the gantry finishes homing, 
 * the LEDs flash green in a blocking manner for a brief period.
 */
  void homingSequence(bool homingComplete = false);

private:
  //a proxy function for setting the brightness of the LEDs. This way the class can handle all the LED stuff
  //without relying on the user to sometimes call on FastLED directly.
  void setBrightness(uint8_t val);

  /**
 * @brief Gradually fades the LEDs on and off over time to indicate that a pattern is running.
 *
 * This function automatically controls the brightness of the LEDs, causing them to fade in and out over a specified period. 
 * It is intended to be used when the machine is running a pattern to provide a visual indication of operation.
 *
 * @param period The time in milliseconds it takes for the LEDs to fade in and out (complete cycle).
 * @param maxBrightness The maximum brightness level the LEDs will reach during the fade cycle.
 *
 * The function calculates the current brightness based on the time position in the fade cycle, applying the appropriate brightness 
 * to all LEDs using the FastLED.setBrightness() function.
 */
  void fadePixels(unsigned long period, uint8_t maxBrightness);

private:
  static constexpr int LED_DATA_PIN = A0;
  static constexpr int NUM_LEDS = 8;           //Number of LEDs in the bar.
  static constexpr int MAX_BRIGHTNESS = 40;          //Brightness values are 8-bit for a max of 255 (the range is [0-255]), this sets default maximum to 40 out of 255.

  CLEDController *Controller;
  CRGB leds[NUM_LEDS];        //array that holds the state of each LED
};
