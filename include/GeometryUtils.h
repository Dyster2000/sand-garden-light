/*
This file is part of Sand-Garden-Light.

Sand-Garden-Light is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation version 3 or later.

Sand-Garden-Light is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with Sand-Garden-Light. If not, see <https://www.gnu.org/licenses/>.
*/
#pragma once

#include "MathUtils.h"

//Movement related functions
int findShortestPathToPosition(int current, int target, int wrapValue);         //For finding the shortest path to the new position on the angular axis
int calcRadialChange(int angularMoveInSteps, int radialMoveInSteps);            //for figuring out the relative change on the radial axis
int calcRadialSteps(int current, int target, int angularOffsetSteps);           //For calculating actual number of steps radial motor needs to take.
int calculateDistanceBetweenPoints(Positions p1, Positions p2);                 //calculate distance between two points in polar coordinates. Not currently used, but useful




/**
 * @brief Reads the analog values from the joystick potentiometers and returns them as a Positions struct.
 *
 * This function reads the analog input values from the joystick's potentiometers on the specified pins,
 * maps the values to a range of -100 to 100 for the angular axis and 100 to -100 for the radial axis, 
 * and applies a deadband to eliminate small fluctuations around the center.
 *
 * @return Positions - a struct containing the mapped and processed values of the angular and radial joystick positions.
 * 
 * The deadband ensures that values near the center of the joystick are treated as zero to account for 
 * measurement noise and prevent unintended small movements.
 */
Positions readJoystick();

//Math related functions
long convertDegreesToSteps(float degrees);                                      //for converting degrees to motor steps
float convertStepsToDegrees(int steps);                                         //for converting motor steps to degrees
long convertRadiansToSteps(float rads);                                         //For converting radians to steps on angular axis
float convertStepsToRadians(float steps);                                       //For converting steps to radians on angular axis
int convertMMToSteps(float mm);                                                 //for converting millimeters to steps on radial axis
float convertStepsToMM(float steps);                                            //for converting steps to millimeters on the radial axis
float fmap(float n, float in_min, float in_max, float out_min, float out_max);  //version of map() that works for floating point numbers
int modulus(int x, int y);                                                      //Use for wrapping values around at ends of range. like %, but no negative numbers.

//Movement related functions

/**
 * @brief Calculates the shortest path to the target position on the angular axis.
 *
 * This function determines the shortest distance required to move from the current position to the 
 * target position on a circular axis, considering both clockwise and counterclockwise directions. 
 * It returns the shortest distance, taking into account a wraparound value for circular motion.
 *
 * @param current The current position on the angular axis, in steps.
 * @param target The desired target position on the angular axis, in steps.
 * @param wrapValue The wraparound point for the axis (e.g., the total number of steps per revolution).
 * 
 * @return int The shortest distance, in steps, required to move to the target position. 
 *         Positive values indicate clockwise movement, while negative values indicate counterclockwise movement.
 */
int findShortestPathToPosition(int current, int target, int wrapValue);         //For finding the shortest path to the new position on the angular axis


int calculateDistanceBetweenPoints(Positions p1, Positions p2);                 //calculate distance between two points in polar coordinates. Not currently used, but useful

//Geometry generation functions

/**
 * @brief Precomputes and returns points approximating a straight line between two positions in polar coordinates.
 *
 * This function precomputes an array of points that approximate a straight line by interpolating between two 
 * end points specified in cartesian coordinates, then converts them to polar coordinates. It stores these points 
 * in a static array and returns the next point on each function call, allowing efficient streaming of precomputed 
 * line points. The line is divided into a specified number of segments (resolution), with a maximum of 100 points.
 *
 * @param point0 The starting point of the line, specified in radial and angular steps.
 * @param point1 The ending point of the line, specified in radial and angular steps.
 * @param current The current position of the gantry, used to calculate relative motion if needed.
 * @param resolution The number of segments to divide the line into, defaulting to 100 and capped at 100.
 * @param reset Bool - set true to force recalculation for a new line.
 *
 * @return Positions The next point along the precomputed line, with radial and angular values in steps.
 *
 * @note The function handles vertical lines by temporarily rotating the points 90 degrees to avoid calculation 
 * issues, then rotates them back before returning. The line is broken into segments up to the maximum length of 
 * the array, and lines close to the center of the field are handled with a higher resolution to maintain accuracy.
 *
 * @details The first call to this function precomputes all points along the line, and subsequent calls return 
 * each point in sequence. The function resets for a new line after the last point is returned.
 */
Positions drawLine(Positions point0, Positions point1, Positions current, int resolution = 100, bool reset = false);             //For drawing a straight line between two points

/**
 * @brief Generates the vertices of a regular n-sided polygon (n-gon) and stores them in an array of Positions.
 *
 * This function computes the vertices of a regular polygon (n-gon) with a specified number of sides, radius, 
 * center point, and optional rotation. The vertices are generated in polar coordinates, with the first vertex 
 * starting at angle 0 (or rotated by the specified degrees) and are then translated to be centered around the 
 * specified center point. The generated points are stored in the provided pointArray.
 *
 * @param pointArray A pointer to the array of Positions to be filled with the vertices of the polygon.
 * @param numPoints The number of vertices (or sides) of the polygon.
 * @param centerPoint The center point of the polygon, specified as a Positions struct (radial and angular coordinates).
 * @param radius The radius of the polygon, which is the distance from the center to each vertex (in motor steps).
 * @param rotationDeg An optional rotation of the polygon in degrees, defaulting to 0.0. This rotates the polygon around its center.
 *
 * @return void
 *
 * @note The function first generates the vertices centered on the origin in polar coordinates, then translates 
 * them to the specified center point by converting to rectangular coordinates, performing the translation, and 
 * converting back to polar. The translatePoints() function is used to handle this translation process.
 *
 * @example
 * // Example of generating an octagon with a radius of 4000 steps centered on the origin:
 * int numberVertices = 8;
 * Positions vertices[numberVertices];
 * Position center = {0, 0};
 * nGonGenerator(vertices, numberVertices, center, 4000, 0.0);
 *
 * // Example of generating a circle with 360 points and a radius of 2000 steps, centered at {3000, 60 degrees}:
 * int numberVertices = 360;
 * Positions vertices[numberVertices];
 * Position center = {3000, convertDegreesToSteps(60)};
 * nGonGenerator(vertices, numberVertices, center, 2000, 0.0);
 */
void nGonGenerator(Positions *pointArray, int numPoints, Positions centerPoint, int radius, float rotationDeg = 0.0);   //generates a list of points that form a polygon's vertices

/**
 * @brief Translates an array of points along a given translation vector, shifting their position in polar coordinates.
 *
 * This function translates the points in the pointArray by converting both the points and the provided 
 * translation vector from polar to rectangular coordinates, performing the translation, and then converting 
 * the points back to polar coordinates. It is useful for shifting polygons or target positions by a specified 
 * offset. For example, this function can be used to shift the center of a polygon generated by nGonGenerator().
 *
 * @param pointArray A pointer to an array of points (of type Positions) representing the points to be translated.
 * @param numPoints The number of points in the array.
 * @param translationVector The translation vector to shift the points by, specified as a Positions struct.
 *
 * @return void - the array is modified directly because it is passed into this function as a pointer.
 *
 * @note The translation is performed by first converting the points and translation vector to rectangular 
 * coordinates (x, y), adding the corresponding components, and then converting the updated points back to 
 * polar coordinates (r, θ). This ensures that the points are translated accurately in both radial and angular 
 * dimensions. The function assumes the angular component of the translation vector is in steps, and the 
 * radial component is in motor steps.
 *
 * @example
 * // Example usage to shift a polygon to a new center:
 * Positions vertices[8];
 * Positions translationVector = {3500, convertDegreesToSteps(45)};
 * nGonGenerator(vertices, 8, {0, 0}, 4000, 0.0);
 * translatePoints(vertices, 8, translationVector);
 */
void translatePoints(Positions *pointArray, int numPoints, Positions translationVector);              //For moving an array of points along a vector to a new position

void scalePoints (Positions *pointArray, int numPoints, float scaleFactor);                            //NOT IMPLEMENTED - For scaling a shape represented by a point array up or down in size
void rotatePoints (Positions *pointArray, int numPoints, Positions rotationCenter, float rotationDeg); //NOT IMPLEMENTED - For rotating a point array around an arbitrary point
void reflectPoints(Positions *pointArray, int numPoints, Positions reflectionVector);                  //NOT IMPLEMENTED - For reflecting a point array across an arbitrary line
