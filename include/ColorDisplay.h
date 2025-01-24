/*
This file is part of Sand-Garden-Light.

Sand-Garden-Light is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation version 3 or later.

Sand-Garden-Light is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with Sand-Garden-Light. If not, see <https://www.gnu.org/licenses/>.
*/
#pragma once

#include <FastLED.h>                  //Controls the RGB LEDs
#include <elapsedMillis.h>

class Joystick;
class OneButtonTiny;

class ColorDisplay
{
private:
  enum class Mode
  {
    None,
    Edit,
    CycleSlow,
    CycleFast,
    ChaseLeft,
    ChaseRight
  };

public:
  ColorDisplay(Joystick &joystickControl, OneButtonTiny &button);

  void setup();
  void clear();
  void setAll(CRGB color);

  void updateColor();

private:
  void CheckCycleModeChange();

  void DoEditMode();
  void DoCycleSlowMode();
  void DoCycleFastMode();
  void DoChaseLeftMode();
  void DoChaseRightMode();

  CRGB getColorFromAngle(int angle, float dist);
  CRGB hsv2rgb(float hue, float saturation, float value);

private:
  static constexpr int LED_DATA_PIN = A4;
  static constexpr int NUM_LEDS = 38;           //Number of LEDs in the bar.
  static constexpr int MAX_BRIGHTNESS = 40;          //Brightness values are 8-bit for a max of 255 (the range is [0-255]), this sets default maximum to 40 out of 255.
  static constexpr int CYCLE_CHANGE_SLOW_MS = 100;
  static constexpr int CYCLE_CHANGE_FAST_MS = 10;
  static constexpr int CYCLE_CHASE_SLOW_MS = 100;
  static constexpr int CYCLE_CHASE_FAST_MS = 50;

  static Mode CurrentMode;

  Joystick &JoystickControl;
  OneButtonTiny &Button;

  CLEDController *Controller;
  CRGB leds[NUM_LEDS];        //array that holds the state of each LED

  int CycleAngle;
  int CycleChaseIndex;
  elapsedMillis CycleTime;
};
