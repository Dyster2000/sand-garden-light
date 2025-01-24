/*
This file is part of Sand-Garden-Light.

Sand-Garden-Light is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation version 3 or later.

Sand-Garden-Light is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with Sand-Garden-Light. If not, see <https://www.gnu.org/licenses/>.
*/
#include <Arduino.h>
#include "ColorDisplay.h"
#include "Joystick.h"
#include "MathUtils.h"
#include "OneButtonTiny.h"

ColorDisplay::Mode ColorDisplay::CurrentMode = ColorDisplay::Mode::None;

ColorDisplay::ColorDisplay(Joystick &joystick, OneButtonTiny &button)
  : JoystickControl{ joystick }
  , Button{ button }
  , CycleAngle{ 0 }
  , CycleChaseIndex{ 0 }
{
  Button.attachClick([]()
    {
      if (CurrentMode != ColorDisplay::Mode::Edit)
        CurrentMode = ColorDisplay::Mode::Edit;
      else
        CurrentMode = ColorDisplay::Mode::None;
      Serial.print("Set CurrentMode=");
      Serial.println((int)CurrentMode);
    });

  Button.attachLongPressStart([]()
    {
      if (CurrentMode == ColorDisplay::Mode::None || CurrentMode == ColorDisplay::Mode::Edit)
        CurrentMode = ColorDisplay::Mode::CycleSlow;
      Serial.print("Set CurrentMode=");
      Serial.println((int)CurrentMode);
    });
}

void ColorDisplay::setup()
{
  Controller = &FastLED.addLeds<WS2812B, LED_DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(MAX_BRIGHTNESS);
  setAll(CRGB::White);

  Controller->showLeds(MAX_BRIGHTNESS);
}

void ColorDisplay::clear()
{
  for (int i = 0; i < NUM_LEDS; i++)
    leds[i] = CRGB::Black;
}

void ColorDisplay::setAll(CRGB color)
{
  for (int i = 0; i < NUM_LEDS; i++)
    leds[i] = color;

  Controller->showLeds(MAX_BRIGHTNESS);
}

void ColorDisplay::updateColor()
{
  // Check for cycle mode change
  switch (CurrentMode)
  {
    case Mode::CycleSlow:
    case Mode::CycleFast:
    case Mode::ChaseLeft:
    case Mode::ChaseRight:
      CheckCycleModeChange();
      break;
    
    default:
      // Nothing to do
      break;
  }

  // Handle mode
  switch (CurrentMode)
  {
    case Mode::Edit:
      DoEditMode();
      break;

    case Mode::CycleSlow:
      DoCycleSlowMode();
      break;

    case Mode::CycleFast:
      DoCycleFastMode();
      break;

    case Mode::ChaseLeft:
      DoChaseLeftMode();
      break;

    case Mode::ChaseRight:
      DoChaseRightMode();
      break;
    
    default:
      // Nothing to do
      break;
  }
}

void ColorDisplay::CheckCycleModeChange()
{
  auto pos = JoystickControl.read();

  if (pos.angular < -75)
    CurrentMode = Mode::ChaseLeft;
  else if (pos.angular > 75)
    CurrentMode = Mode::ChaseRight;
  else if (pos.radial < -75)
    CurrentMode = Mode::CycleSlow;
  else if (pos.radial > 75)
    CurrentMode = Mode::CycleFast;
}

void ColorDisplay::DoEditMode()
{
  auto pos = JoystickControl.readPolar();
  auto rgb = getColorFromAngle(pos.angular, pos.radial / 100.0f);

  CycleAngle = pos.angular;
  setAll(rgb);
}

void ColorDisplay::DoCycleSlowMode()
{
  if (CycleTime >= CYCLE_CHANGE_SLOW_MS)
  {
    CycleTime = 0;
    if (++CycleAngle >= 360)
      CycleAngle = 0;

    auto rgb = getColorFromAngle(CycleAngle, 1.0f);
    setAll(rgb);
  }
}

void ColorDisplay::DoCycleFastMode()
{
  if (CycleTime >= CYCLE_CHANGE_FAST_MS)
  {
    CycleTime = 0;
    if (++CycleAngle >= 360)
      CycleAngle = 0;

    auto rgb = getColorFromAngle(CycleAngle, 1.0f);
    setAll(rgb);
  }
}

void ColorDisplay::DoChaseLeftMode()
{
  if (CycleTime >= CYCLE_CHASE_SLOW_MS)
  {
    CycleTime = 0;
    if (++CycleAngle >= 360)
      CycleAngle = 0;

    auto rgb = getColorFromAngle(CycleAngle, 1.0f);
    leds[CycleChaseIndex] = rgb;
    if (--CycleChaseIndex < 0)
      CycleChaseIndex = NUM_LEDS - 1;
    Controller->showLeds(MAX_BRIGHTNESS);
  }
}

void ColorDisplay::DoChaseRightMode()
{
  if (CycleTime >= CYCLE_CHASE_FAST_MS)
  {
    CycleTime = 0;
    if (++CycleAngle >= 360)
      CycleAngle = 0;

    auto rgb = getColorFromAngle(CycleAngle, 1.0f);
    leds[CycleChaseIndex] = rgb;
    if (--CycleChaseIndex < 0)
      CycleChaseIndex = NUM_LEDS - 1;
    Controller->showLeds(MAX_BRIGHTNESS);
  }
}

CRGB ColorDisplay::getColorFromAngle(int angle, float dist)
{
  auto hue = angle;
  auto sat = dist;
  auto val = 1;
  auto rgb = hsv2rgb(hue, sat, val);

  return rgb;
}

CRGB ColorDisplay::hsv2rgb(float hue, float saturation, float value)
{
  hue /= 60;

  auto chroma = value * saturation;
  auto x = chroma * (1 - abs(fmod(hue, 2) - 1));
  float r = 0;
  float g = 0;
  float b = 0;

  switch ((int)hue)
  {
    case 0:
      r = chroma;
      g = x;
      b = 0;
      break;
    case 1:
      r = x;
      g = chroma;
      b = 0;
      break;
    case 2:
      r = 0;
      g = chroma;
      b = x;
      break;
    case 3:
      r = 0;
      g = x;
      b = chroma;
      break;
    case 4:
      r = x;
      g = 0;
      b = chroma;
      break;
    default:
      r = chroma;
      g = 0;
      b = x;
      break;
  }

  CRGB rgb;

  rgb.red = (uint8_t)((r + value - chroma) * 255);
  rgb.green = (uint8_t)((g + value - chroma) * 255);
  rgb.blue = (uint8_t)((b + value - chroma) * 255);

  return rgb;
}