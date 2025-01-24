/*
This file is part of Sand-Garden-Light.

Sand-Garden-Light is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation version 3 or later.

Sand-Garden-Light is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with Sand-Garden-Light. If not, see <https://www.gnu.org/licenses/>.
*/
#pragma once

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
This region of code contains useful math functions for doing things like converting between units, doing modulus math that doesn't allow negative
numbers, and finding the distance between points in polar coordinates.
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Struct used for storing positions of the axes, as well as storing the values of the joystick.
struct Positions
{
  int radial;                     //the units for these values are motor steps
  int angular;                    
};                                

/**
 * @brief Maps a float value from one range to another.
 *
 * This function works similarly to the standard map() function but allows for floating-point inputs 
 * and outputs. It maps a float n from a specified input range (in_min to in_max) to a corresponding 
 * output range (out_min to out_max).
 *
 * @param n The float value to map.
 * @param in_min The lower bound of the input range.
 * @param in_max The upper bound of the input range.
 * @param out_min The lower bound of the output range.
 * @param out_max The upper bound of the output range.
 *
 * @return float The mapped value in the output range.
 */
float fmap(float n, float in_min, float in_max, float out_min, float out_max);

/**
 * @brief Converts an angular measurement in degrees to the corresponding number of steps for a stepper motor.
 *
 * This function converts a given angle in degrees to the number of motor steps required for the stepper motor 
 * to rotate by that angle. The conversion is based on the number of steps per full revolution of the motor.
 *
 * @param degrees The angle in degrees to convert.
 *
 * @return long The number of steps required for the motor to move the specified angle.
 */
long convertDegreesToSteps(float degrees);

/**
 * @brief Converts an angular measurement in radians to the corresponding number of steps for a stepper motor.
 *
 * @param rads The angle in radians to convert.
 * @return long The number of steps required for the motor to move the specified angle in radians.
 */
long convertRadiansToSteps(float rads);

/**
 * @brief Converts a number of steps to the corresponding angle in radians.
 *
 * @param steps The number of motor steps to convert.
 * @return float The equivalent angle in radians.
 */
float convertStepsToRadians(float steps);

/**
 * @brief Converts a number of steps to the corresponding angle in degrees.
 *
 * @param steps The number of motor steps to convert.
 * @return float The equivalent angle in degrees.
 */
float convertStepsToDegrees(int steps);

/**
 * @brief Converts a distance in millimeters to the corresponding number of steps for the radial axis.
 *
 * @param mm The distance in millimeters to convert.
 * @return int The equivalent number of steps.
 */
int convertMMToSteps(float mm);

/**
 * @brief Converts a number of steps to the corresponding distance in millimeters for the radial axis.
 *
 * @param steps The number of motor steps to convert.
 * @return float The equivalent distance in millimeters.
 */
float convertStepsToMM(float steps);

/**
 * @brief Computes the modulus of two integers, ensuring the result is non-negative.
 *
 * This function is a replacement for the % operator that prevents negative results by wrapping 
 * negative values around to the positive range. It is mainly used for handling angular values 
 * when the gantry wraps from 360 degrees to 0 degrees.
 *
 * @param x The dividend.
 * @param y The divisor.
 * 
 * @return int The modulus result, always non-negative.
 */
int modulus(int x, int y);

/**
 * @brief Calculates the distance between two points in polar coordinates using the law of cosines.
 *
 * This function computes the distance between two points specified in polar coordinates (radii and angles). 
 * It uses the law of cosines to perform the calculation, assuming the angles are provided in degrees 
 * and the radii in arbitrary units. The returned distance is in the same units as the radii.
 *
 * @param p1 The first point, represented as a Positions struct (with radial and angular values).
 * @param p2 The second point, represented as a Positions struct (with radial and angular values).
 * 
 * @return int The calculated distance between the two points, rounded to the nearest integer.
 */
int calculateDistanceBetweenPoints(Positions p1, Positions p2);

int cartToPolarAngle(int x, int y);
