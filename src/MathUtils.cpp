/*
This file is part of Sand-Garden-Light.

Sand-Garden-Light is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation version 3 or later.

Sand-Garden-Light is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with Sand-Garden-Light. If not, see <https://www.gnu.org/licenses/>.
*/
#include <Arduino.h>
#include "MathUtils.h"
#include "Constants.h"

float fmap(float n, float in_min, float in_max, float out_min, float out_max)
{
  return (n - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

long convertDegreesToSteps(float degrees)
{
  return round(fmap(degrees, 0.0, 360.0, 0.0, 2.0 * STEPS_PER_MOTOR_REV));
}

long convertRadiansToSteps(float rads)
{
  return round(fmap(rads, 0.0, 2.0 * PI, 0.0, 2.0 * STEPS_PER_MOTOR_REV));
}

float convertStepsToRadians(float steps)
{
  return fmap(steps, 0.0, 2.0 * STEPS_PER_MOTOR_REV, 0.0, 2.0 * PI);
}

float convertStepsToDegrees(int steps)
{
  return fmap(float(steps), 0.0, 2.0 * STEPS_PER_MOTOR_REV, 0.0, 360.0);
}

int convertMMToSteps(float mm)
{
  return round(mm * STEPS_PER_MM);
}

float convertStepsToMM(float steps)
{
  return steps * MM_PER_STEP;
}

int modulus(int x, int y)
{
  return x < 0 ? ((x + 1) % y) + y - 1 : x % y;
}

int calculateDistanceBetweenPoints(Positions p1, Positions p2)
{
  return round(sqrt(pow(p1.radial, 2) + pow(p2.radial, 2) - 2 * p1.radial * p2.radial * cos(convertStepsToRadians(p2.angular) - convertStepsToRadians(p1.angular))));
}

int cartToPolarAngle(int x, int y)
{
  float angle = atan2(y, x);

  if (angle < 0)
    angle += 2 * PI;
  float deg = angle * 180 / PI;

  return (int)deg;
}
