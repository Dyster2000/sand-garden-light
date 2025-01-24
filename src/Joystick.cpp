/*
This file is part of Sand-Garden-Light.

Sand-Garden-Light is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation version 3 or later.

Sand-Garden-Light is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with Sand-Garden-Light. If not, see <https://www.gnu.org/licenses/>.
*/
#include <Arduino.h>
#include "Joystick.h"
#include "MathUtils.h"

Joystick::Joystick(uint8_t horizontalPin, uint8_t verticalPin)
  : HorizontalPin{ horizontalPin }
  , VerticalPin{ verticalPin }
{
}

void Joystick::setup()
{
  //configure the joystick and button pins
  pinMode(HorizontalPin, INPUT);
  pinMode(VerticalPin, INPUT);
}

Positions Joystick::read()
{
  Positions values;
  values.angular = map(analogRead(HorizontalPin), 0, 1023, -100, 100);
  values.radial = map(analogRead(VerticalPin), 0, 1023, 100, -100);

  if (values.angular <= 5 && values.angular >= -5)
    values.angular = 0;   //apply a deadband to account for measurement error near center.
  if (values.radial <= 5 && values.radial >= -5)
    values.radial = 0;
  return values;
}

Positions Joystick::readPolar()
{
  Positions values;
  auto x = map(analogRead(HorizontalPin), 0, 1023, 100, -100);
  auto y = map(analogRead(VerticalPin), 0, 1023, 100, -100);

  if (x <= 5 && x >= -5)
    x = 0;   //apply a deadband to account for measurement error near center.
  if (y <= 5 && y >= -5)
    y = 0;

  values.angular = cartToPolarAngle(x, y);
  values.radial = round(sqrt(x * x + y * y));

  return values;
}
