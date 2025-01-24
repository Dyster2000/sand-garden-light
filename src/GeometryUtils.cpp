/*
This file is part of Sand-Garden-Light.

Sand-Garden-Light is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation version 3 or later.

Sand-Garden-Light is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with Sand-Garden-Light. If not, see <https://www.gnu.org/licenses/>.
*/
#include <Arduino.h>
#include "GeometryUtils.h"
#include "Constants.h"

#define JOYSTICK_A_PIN   A2          //Left-right axis of joystick, associated with changing angular axis in manual mode
#define JOYSTICK_R_PIN   A3          //Up-down axis of joystick, associated with changing radial axis in manual mode

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
Geometry Generation.
Functions that handle generating points and shapes for drawing. Draw straight lines, create polygons, perform the basic geometric transformations
like rotation, translation, scaling, and (eventually) reflection.
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma region GeometryGeneration

Positions drawLine(Positions point0, Positions point1, Positions current, int resolution, bool reset)
{
  //this is the nested array that will store the precomputed points. has to be static so values persist between function calls.
  //it will be of the form pointArray[100][2] = {{r0, theta0}, {r1, theta1}, ... {r99, theta99}}.
  //to access the theta value for point3 (4th point in array), you would call point3.angular = pointArray[3][1];

  //Future update: make this a single layer array of type Positions instead of type Int for simplicity.
  static int pointArray[100][2];  

  static int numPoints = 0;                           //the number of points the line will be approximated with.
  static bool newLine = true;                         //used to track if the function is being called for a new line, or if it needs to provide points for an extant line
  static float x0 = 0, x1 = 0, y0 = 0, y1 = 0;        //end points of the line
  static float xtemp = 0, ytemp = 0, thetaTemp = 0.0; //temporary storage for calculations
  static float stepover = 0;                          //how far to move along x-axis for interpolating along line
  static float m = 0;                                 //the slope of the line (y = mx + b)
  static float denom = 0;                             //the denominator in the slope calculation (x1 - x0)
  static float b = 0;                                 //the y-intercept of the line (y = mx + b)
  static bool pointsRotated = false;                  //used to indicate if points have been rotated to deal with vertical lines and need to be rotated back on output.

  Positions p0 = point0, p1 = point1;                 //containers for the points (we may need to modify their values to deal with vertical lines)
  Positions outputPoint;                              //the struct we'll use for passing the target positions out of the function
  static int outNum = 0;                              //used for tracking which point to return on each call to this function
  
  if (newLine || reset)
  {                             //if this is a new line, or the reset flag is set
    numPoints = constrain(resolution, 0, 100);     //we can approximate the line with up to 100 points. recalculate this number for each new line.
    
    //check now to see if there will be a vertical line after the coordinate transformation from polar to rectangular coords
    int comparisonA = STEPS_PER_A_AXIS_REV - max(p0.angular, p1.angular);        //units are in steps
    int comparisonB = min(p0.angular, p1.angular);

    //this next step checks to see if the line connecting these two points is within half a degree of vertical in the rectangular coordinate system.
    //From my early testing, if the lines are more than half a degree off of vertical, they render perfectly fine without special handling.
    //It's really just a vertical line that gets weird (e.g., a line connecting two points that are 45 and 315 degrees off the origin ray at the same radius).
    if ((comparisonA - comparisonB <= convertDegreesToSteps(0.5)) && (comparisonA - comparisonB >= convertDegreesToSteps(-0.5)))
    {
      pointsRotated = true;   //we're going to rotate the points by 90 degrees to deal with the nearly vertical line, so set this flag.
      p0.angular += convertDegreesToSteps(90);
      p1.angular += convertDegreesToSteps(90);
    }

    //take in the points, convert them to radians for the angular unit. only need to do this one time for a new line.
    //also convert each point from polar to cartesian coordinates.
    x0 = p0.radial * cos(convertStepsToRadians(p0.angular));        //x = r*cos(theta)
    y0 = p0.radial * sin(convertStepsToRadians(p0.angular));        //y = r*sin(theta)
    x1 = p1.radial * cos(convertStepsToRadians(p1.angular));        //x = r*cos(theta)
    y1 = p1.radial * sin(convertStepsToRadians(p1.angular));        //y = r*sin(theta)

    denom = x1 - x0;

    //calculate the slope
    m = (y1 - y0) / denom;
    //calculate the y-intercept   y = mx + b, so b = y - mx. Use point0 values for y and x
    b = y0 - (m * x0);


    if (b < 100.0 && b > -100.0)
    {      //if the line is within 100 steps of the origin
      //This takes care of lines that come really close to intercepting the origin. First, I'm using this range of values rather 
      //than saying if (b == 0.0) because this is using floating point math, and equalities like that almost never evaluate to
      //true with floats. Lines that come really close to the origin require the gantry to flip around 180 degrees in the
      //angular axis once the ball is at the center of the field. The straight line algorithm already handles this well, but if
      //the line is broken into a small number of segments, that large rotation at the center winds up drawing a small arc 
      //around the center. I dealt with this by just having the program maximize the number of segments the lines is broken
      //into for lines which come close to the center. You can adjust the values in the condition above to change what it means
      //for a line to be close to the center to fine tune how well straight lines are drawn.
      numPoints = 100;
    } 
    //This line doesn't come really close to intersecting the origin, so we'll handle it differently.
  
    //divide one axis into the number of segments required by resolution, up to a maximum of the length of the array they'll be stored in.
    //defining all of these values as static means the value will persist between function calls, but also means I have to reset them
    //to initial values once the last point in the line is returned.
    stepover = (x1 - x0) / (float)numPoints;       //should define how far to move along x axis for interpolation along line.

    for (int i = 0; i < numPoints; i++)
    {
      //start by generating absolute position values for the points along the line in terms of {r, theta}.
      //We are starting with absolute values because the end points of the line are specified in absolute coordinates.

      if (i == 0)
      {                                             //if it's the first point in the line, put the point0 values into the list to ensure we start there
        pointArray[i][0] = p0.radial;                       //these units are already in steps as absolute coordinates
        pointArray[i][1] = p0.angular;                      //units in steps, absolute coordinates. need to be changed to relative later.
      }
      else if (i == numPoints - 1)
      {                          //If it's the last point in the line, put point1 values into the list to make sure we end there.
        pointArray[i][0] = p1.radial;
        pointArray[i][1] = p1.angular;
      }
      else
      {                                                  //We're somewhere along the line that isn't the beginning or end, so we need to generate these values.
        //Calculate the next x value in the series. Use the values of i and stepover to figure out how many line segments to increment along from the starting point.
        //I'm using (i + 1) instead of i in the calculation because I'm handling the first and last points separately,
        //so by the time we get to this code, we need to move over by at least one increment of stepover, but i starts counting from 0.
        xtemp = x0 + (i + 1) * stepover;                              
        ytemp = m * xtemp + b;                                  //y = mx + b gives next y value in the series.

        //calculate the angular position of the current point.
        //atan2f(y, x) is a special version of the arctan function that returns the angle based on y and x.
        thetaTemp = atan2f(ytemp, xtemp); 

        //ata2f() has a range of (-pi, pi), and we'll need to shift that to be (0, 2pi)
        if (thetaTemp < 0)
          thetaTemp = 2.0 * PI + thetaTemp;    //this is in radians, ranging from 0 to 2pi

        //now that we know the anglular position of the point in radians, we need to find the radial position in units of steps
        //using the Pythagorean theorem (square roots calculate more efficiently than trig functions on Arduino Nano).
        //Then store the r and theta points in the array.
        pointArray[i][0] = sqrt(xtemp * xtemp + ytemp * ytemp); //the radial value of the point. This is absolute coordinates from the origin. Units are steps.
        //store the angular position converted from radians to steps. This is still in absolute coordinates, not relative.
        pointArray[i][1] = convertRadiansToSteps(thetaTemp);    
      }
      
      //finally, if we rotated the points to deal with a vertical line, rotate them back.
      if (pointsRotated)
      {
        pointArray[i][1] -= convertDegreesToSteps(90);
      }
    }

    //we need to set the newLine flag to false so that the next time this function is called,
    //we can output the points along the line rather than recalculating the points.
    newLine = false;       //later in the program, we have to reset this to true once the last line of the point is returned.
    reset = false;
    outNum = 0;            //need to reset this to 0 so we can start outputting the points, starting from the first one.
    pointsRotated = false;
  }
  
  //now we need to output the correct point in the array.
  if (outNum < numPoints)
  {
    outputPoint.radial = pointArray[outNum][0];   //put the r value into the struct
    outputPoint.angular = pointArray[outNum][1];  //put the theta value into the struct
    outNum++;                                     //increment to the next point in the array
  }

  //once the last point is ready for return, reset all the variables necessary to rerun all the calculations on the next call to this function.
  if (outNum >= numPoints)
  {
    newLine = true;
  }

  //finally, return the value of the point to be moved to!
  return outputPoint;
}

void nGonGenerator(Positions *pointArray, int numPoints, Positions centerPoint, int radius, float rotationDeg)
{
  //*pointArry is the pointer to the array that will be built out.
  //numPoints is the length of that array (equal to number of desired vertices).
  //centerPoint is the center point of the polygon (supply as a Position struct)
  //radius is the distance from the center point to a vertex. Units are motor steps.
  //rotationDeg rotates the polygon in degrees. The first vertex will always be at angle = 0, unless you specify a rotation angle.

  //Start by generating vertices in polar coords, centered on origin. 
  int angleStep = STEPS_PER_A_AXIS_REV / numPoints;      //calculate how much to step the angle over for each point

  for (int i = 0; i < numPoints; i++)
  {
    //define each vertex.
    //What i have done below is the same as doing:
    //pointArray[i].radial = radius; pointArray[i].angular = i * angleStep + convertDegreesToSteps(rotationDeg);
    //This is called aggregate initialization.

    pointArray[i] = {radius, i * angleStep + (int)convertDegreesToSteps(rotationDeg)};
  }

  //Currently all the points in the array are centered on the origin. We need to shift the points to be centered on the
  //desired center point. You can do this in polar coordinates, but it's much simpler to convert to rectangular coordinates,
  //move all the points, and then convert back to polar.
  
  if (centerPoint.radial != 0)
  {        //if the radial coordinate of the center point is not 0, we need to translate the polygon
    translatePoints(pointArray, numPoints, centerPoint);      //This moves all points in the array to be centered on the correct point
  }
}

void translatePoints(Positions *pointArray, int numPoints, Positions translationVector)
{
  if (translationVector.angular != 0 || translationVector.radial != 0)
  {    //desired polygon is not centered on origin, so we need to shift the points.
    for (int i = 0; i < numPoints; i++)
    {
      float x = pointArray[i].radial * cos(convertStepsToRadians(pointArray[i].angular));
      float y = pointArray[i].radial * sin(convertStepsToRadians(pointArray[i].angular));

      //now figure out where the center point is in rectangular coordinates
      //NOTE: at some point I want to move this calculation out of the for loop for efficiency
      float centerX = translationVector.radial * cos(convertStepsToRadians(translationVector.angular));
      float centerY = translationVector.radial * sin(convertStepsToRadians(translationVector.angular));

      //now use centerX and centerY to translate each point.
      x += centerX;      //this should shift the X coordinate appropriately
      y += centerY;     //this should shift the Y coordinate appropriately

      //now convert back into polar coordinates

      //calculate the angular position of the current point. Units are in radians.
      //atan2f(y, x) is a special version of the arctan function that returns the angle based on y and x.
      float angleTemp = atan2f(y, x); 

      //atan2f() has a range of (-pi, pi), and we'll need to shift that to be (0, 2pi)
      if (angleTemp < 0)
        angleTemp = 2.0 * PI + angleTemp;    //this is in radians, ranging from 0 to 2pi

      //now that we know the anglular position of the point in radians, we need to find the radial position in units of steps
      //using the Pythagorean theorem (square roots calculate more efficiently than trig functions on Arduino Nano).
      //Then store the r and theta points in the array.
      pointArray[i].radial = round(sqrt(x * x + y * y));   //the radial value of the point. This is absolute coordinates from the origin. Units are steps.
  
      //store the angular position converted from radians to steps. This is still in absolute coordinates.
      pointArray[i].angular = convertRadiansToSteps(angleTemp);  
    }  
  }
}

/*
NOT IMPLEMENTED.

The idea of this function is to take in an array of points that represent a shape, like a
hexagon generated by nGonGenerator, and use it to scale it up or down in size. I don't 
yet have a great idea of how to solve this problem, so I left it here as a suggestion for 
the hacker. Try your hand at solving this problem!
*/
void scalePoints (Positions *pointArray, int numPoints, float scaleFactor)
{
}

/*
NOT IMPLEMENTED.

The idea of this is totake in an array of points, such as one created by nGonGenerator,
and rotate them around an arbitrary point within the drawing field. I had planned to implement this,
but ran out of time, and have left it here as a suggestion for the hacker. Try your hand at
solving this problem!
*/
void rotatePoints(Positions *pointArray, int numPoints, Positions rotationCenter, float rotationDeg)
{
}

/*
NOT IMPLEMENTED.

The idea for this is to take in an array of points, like that generated by nGonGenerator,
and to relect it across an arbitrary line in the drawing field. I had planned to implement this,
but ran out of time, and have left it here as a suggestion for the hacker. Try your hand at
solving this problem!
*/
void reflectPoints(Positions *pointArray, int numPoints, Positions reflectionVector)
{
}

#pragma endregion GeometryGeneration
