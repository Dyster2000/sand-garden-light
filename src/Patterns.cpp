/*
This file is part of Sand-Garden-Light.

Sand-Garden-Light is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation version 3 or later.

Sand-Garden-Light is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with Sand-Garden-Light. If not, see <https://www.gnu.org/licenses/>.
*/
#include <Arduino.h>
#include "Patterns.h"
#include "GeometryUtils.h"
#include "Constants.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
This region of code contains the different pattern generating functions.
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma region Patterns

/**
 * @brief Pattern: Simple Spiral. Generates the next target position for drawing a simple inward and outward spiral.
 *
 * This function calculates the next target position for a simple spiral pattern, starting from the current position.
 * The pattern progresses by incrementally adding small values to the current angular and radial positions. The spiral
 * moves inward more quickly than it moves outward due to the mechanical relationship between the radial and angular axes.
 *
 * @param current The current position of the gantry, represented as a Positions struct.
 * @param restartPattern A flag that allows for restarting the pattern (not used in this simple version). Defaults to false.
 *
 * @return Positions The next target position for the motion controller, represented as a Positions struct.
 *
 * @note The pattern starts from the current position, so if the previous pattern leaves the ball in a specific position,
 * the spiral will continue from there. The pattern adjusts the radial and angular steps incrementally and reverses
 * direction when the radial boundaries are reached.
 */
Positions pattern_SimpleSpiral(Positions current, bool restartPattern)
{
  Positions target;                                        //This is where we'll store the value of the next target position.

  const float angleDivisions = 100.0;                      //Going to divide a full revolution into 100ths. "const" because this never changes.
  const int radialDivisions = 10 * (int)angleDivisions;    //Going to divide the radial axis into 1000ths. Try changing the 10 to a different number, like 20.

  //Calculate how many degrees we'll move over in the angular axis for the next step.
  const int angleStep = convertDegreesToSteps(360.0 / angleDivisions);

  //Calculate how far along we'll move the radial axis for the next step. 
  //The "static" keyword means that this variable is defined once when the function is run for the first time.
  //This is different than "const" because this is a variable, not a constant, so we can still change the value.
  //If the following line were to omit the "static" keyword, this variable would be reset to its initial value
  //every time the function is called, meaning that we couldn't change it between positive and negative to 
  //make the spiral grow inward or outward.
  static int radialStep = MAX_R_STEPS / radialDivisions;

  target.angular = current.angular + angleStep;            //Set the angular position of the new point to move to (target position)
  target.radial = current.radial + radialStep;             //Set the radial target position as the current radial position plus the radialStep

  if (target.radial > MAX_R_STEPS || target.radial < 0)
  {  //Logic to see if the targeted radial position is out of bounds of the radial axis
    radialStep *= -1;                                      //If we're out of bounds, switch the sign of the radialStep (now we move in the opposite direction)
    target.radial += 2 * radialStep;                       //We were already out of bounds, so we undo that by moving 2 radialSteps in the new direction.
  }

  return target;                                           //Return the target position so that the motion control functions can move to it.
}

/**
 * @brief Pattern: Cardioids. Generates the next target position for drawing repeating, slowly rotating cardioids.
 *
 * This function generates the next target position for a cardioid pattern, moving in relative coordinates by adding 43 degrees
 * to the current angular position and adjusting the radial position by 1/8th of the total radial axis. The pattern alternates
 * the direction of radial movement, creating a stepped approximation of a triangle wave along the radial axis.
 *
 * @param current The current position of the gantry, represented as a Positions struct.
 * @param restartPattern A flag that allows restarting the pattern, setting the angular and radial positions to 0. Defaults to false.
 *
 * @return Positions The next target position for the motion controller, represented as a Positions struct.
 *
 * @note This pattern works best after a reset, as it always operates in relative coordinates. If started after running another pattern,
 * the results may vary, since it builds upon the current position of the gantry.
 */
Positions pattern_Cardioids(Positions current, bool restartPattern)
{
  Positions target;
  const int radialStep = ((MAX_R_STEPS) / 8);       //we're going to take huge steps radially (this defaults to 1/8th of the radial axis)
  static int direction = 1;                         //1 means counterclockwise, -1 means clockwise
  static bool firstRun = true;

  if (firstRun || restartPattern)
  {                 //if it's the first time we're running the pattern, or if we start it from another pattern
    target.angular = 0;
    target.radial = 0;
    firstRun = false;
  }
  else
  {

    target.angular = current.angular + convertDegreesToSteps(43);   //add 43 degrees to current position

    //this block of code moves the radial axis back and forth in increments that are 1/8th the length of the total radial axis.
    //Basically, this is a stepped approximation of a triangle wave.

    int nextRadial = current.radial + (direction * radialStep);      //calculate potential next radial position

    if ((nextRadial <= MAX_R_STEPS) && (nextRadial >= 0))
    {          //If the next radial position is in bounds of the radial axis soft limits
      target.radial = nextRadial;                                    //Moves the radial axis positive direction by 1/8th the length of the axis
    }
    else
    {
      direction *= -1;                                               //switch the radial step direction
      target.radial = current.radial + (direction * radialStep);
    }
  }

  return target;
}

/**
 * @brief Pattern: Wavy Spiral. Generates the next target position for drawing a wavy spiral pattern.
 *
 * This function creates a wavy spiral pattern, which is similar to the simple spiral pattern but with an additional sine wave
 * component added to the radial position. The result is a spiral with oscillating radial movement, creating a wavy effect.
 * The sine wave's amplitude and frequency can be adjusted to control the wave's characteristics.
 *
 * @param current The current position of the gantry, represented as a Positions struct.
 * @param restartPattern A flag that allows restarting the pattern. Defaults to false.
 *
 * @return Positions The next target position for the motion controller, represented as a Positions struct.
 *
 * @note This pattern adds a sine wave to the radial position to create the wavy effect. You can modify the amplitude and frequency
 * of the wave to achieve different variations of the pattern. The radial movement is reversed when the limits of the radial axis are reached.
 */
Positions pattern_WavySpiral(Positions current, bool restartPattern)
{

  Positions target;                                  //This is where we'll store the value of the next target position.

  float angleDivisions = 100.0;                      //Going to divide a full revolution into 100ths. "const" because this never changes.
  int radialDivisions = 10 * (int)angleDivisions;    //Going to divide the radial axis into 1000ths. Try changing the 10 to a different number, like 20.

  //Add in values for the amplitude and frequency of the sine wave
  float amplitude = 200.0;
  int period = 8;

  //Calculate how many degrees we'll move over in the angular axis for the next step.
  const int angleStep = convertDegreesToSteps(360.0 / angleDivisions);

  static int radialStep = MAX_R_STEPS / radialDivisions;

  target.angular = current.angular + angleStep;            //Set the angular position of the new point to move to (target position)
  target.radial = current.radial + radialStep;             //Set the radial target position as the current radial position plus the radialStep

  if (target.radial > MAX_R_STEPS || target.radial < 0)
  {  //Logic to see if the targeted radial position is out of bounds of the radial axis
    radialStep *= -1;                                      //If we're out of bounds, switch the sign of the radialStep (now we move in the opposite direction)
    target.radial += 2 * radialStep;                       //We were already out of bounds, so we undo that by moving 2 radialSteps in the new direction.
  }

  target.radial += (int)(amplitude * sin(period * convertStepsToRadians(target.angular)));

  return target;                                           //Return the target position so that the motion control functions can move to it.
}

/**
 * @brief Pattern: Rotating Squares. Generates the next target position for drawing rotating squares, each rotated by 10 degrees.
 *
 * This function draws squares of the same size by connecting four points in sequence and rotating the square by 10 degrees
 * after completing each one. The function uses a switch-case statement to control the drawing process, ensuring each side
 * of the square is drawn in order. Once a square is complete, the vertices are rotated for the next iteration.
 *
 * @param current The current position of the gantry, represented as a Positions struct.
 * @param restartPattern A flag that allows restarting the pattern. Defaults to false.
 *
 * @return Positions The next target position for the motion controller, represented as a Positions struct.
 *
 * @note This pattern relies on a static variable to track the current step in the drawing process and uses the drawLine function
 * to move between the vertices of the square. After each square is completed, the vertices are rotated by 10 degrees for the next square.
 */
Positions pattern_RotatingSquares(Positions current, bool restartPattern)
{
  Positions target;
  static int step = 0;
  static int segments = 20;                         //Use  20 points to approximate a straight line
  static Positions p1, p2, p3, p4;                  //the four vertices of our square
  static bool firstRun = true;                      //used to track if this is the first time the function is called
  const int angleShift = convertDegreesToSteps(10); //how much we'll rotate the square
  if (firstRun || restartPattern)
  {
    p1.angular = 0;                                 //angular position of first point in absolute coordinates
    p1.radial = 7000;                               //radial position of first point in absolute coordiantes (units are steps)
    p2.angular = convertDegreesToSteps(90);
    p2.radial = 7000;
    p3.angular = convertDegreesToSteps(180);
    p3.radial = 7000;
    p4.angular = convertDegreesToSteps(270);
    p4.radial = 7000;
    firstRun = false;
  }

  switch (step)
  {
    case 0:                                                                   //if step == 0
      target = drawLine(p1, p2, current, segments);                  //target positions are the result of calling drawLine between points p1 and p2
      if ((target.angular == p2.angular) && (target.radial == p2.radial))
      {   //If we've reached the end of the line
        step++;                                                               //Increment the value of step so we can move on to the next line
      }
      break;                                                                  //have to include "break;" to avoid case fall through

    case 1:                                                                   //if step == 1
      target = drawLine(p2, p3, current, segments);
      if ((target.angular == p3.angular) && (target.radial == p3.radial))
      {
        step++;
      }
      break;

    case 2:
      target = drawLine(p3, p4, current, segments);
      if ((target.angular == p4.angular) && (target.radial == p4.radial))
      {
        step++;
      }
      break;

    case 3:
      target = drawLine(p4, p1, current, segments);
      if ((target.angular == p1.angular) && (target.radial == p1.radial))
      {
        step++;                                                               //incrementing again would take us to case 4, but we don't have that, so default gets called next
      }
      break;

    default:
      //assuming that the step number was out of bounds, so reset it
      step = 0;                 //start the square over
      target = current;         //set the target position to the current position just as a default for the default option in the switch statement.
      p1.angular += angleShift; //rotate all points in the square by 10 degrees
      p2.angular += angleShift;
      p3.angular += angleShift;
      p4.angular += angleShift;
      break;
  }

  return target;
}

/**
 * @brief Pattern: Pentagon Spiral. Generates the next target position for drawing a growing and shrinking pentagon spiral.
 *
 * This function creates a pentagon using the nGonGenerator function to generate the vertices and then iterates through
 * the vertices, connecting them with straight lines. After completing a pentagon, the radius of each vertex is adjusted
 * by a radial step value (radialStepover). When the radius exceeds the maximum or falls below zero, the direction of
 * the radial change is reversed, creating a pattern of growing and shrinking pentagons.
 *
 * @param current The current position of the gantry, represented as a Positions struct.
 * @param restartPattern A flag that allows restarting the pattern. Defaults to false.
 *
 * @return Positions The next target position for the motion controller, represented as a Positions struct.
 *
 * @note This pattern does not use a switch-case statement for sequencing but instead iterates over a list of precomputed points
 * (the vertices of the pentagon) and adjusts the radial distance of each point to create a spiral effect. The vertices are
 * recalculated when a complete pentagon is drawn.
 */

Positions pattern_PentagonSpiral(Positions current, bool restartPattern)
{
  Positions target;                                                   //Output position will be stored here
  static int start = 0;                                               //Index to the starting point of the line in the array
  static int end = 1;                                                 //Index to the end point of the line in the array
  static bool firstRun = true;                                        //Flag for tracking if a new polygon needs to be generated
  const int vertices = 5;                                             //Change this to make a different polygon
  static Positions vertexList[vertices];                               //construct an array to store the vertices of the polygon
  static int radialStepover = 500;                                    //Amount to change the radius of the polygon each cycle

  if (firstRun || restartPattern)
  {                                                     //On first function call, construct the polygon vertices
    nGonGenerator(vertexList, vertices, { 0,0 }, 1000, 0.0);             //generate the vertices of the polygon  
    firstRun = false;                                                 //Use already generated points next time this function is called
  }
  target = drawLine(vertexList[start], vertexList[end], current, 100);  //draw the line between the appropriate points

  if ((target.angular == vertexList[end].angular) &&                   //If the line is complete, need to move on to the next line
    (target.radial == vertexList[end].radial))
  {
    start++;                                                          //increment start and end points of the line in the array
    end++;
    start = modulus(start, vertices);                                 //wrap around to beginning of array if needed
    end = modulus(end, vertices);
    if (start == 0 && end == 1)
    {                                     //If we're onto a new iteration of the polygon
      for (int i = 0; i < vertices; i++)
      {                            //Increase or decrease the radius of each point
        int newR = vertexList[i].radial + radialStepover;
        if (newR > MAX_R_STEPS || newR < 0)
        {                         //If the radius is getting out of bounds
          radialStepover *= -1;                                       //Switch direction of radial change
          newR += 2 * radialStepover;                                 //move the other way
        }
        vertexList[i].radial = newR;                                   //change the radius for each point
      }
    }
  }
  return target;                                                      //return the new target position
}

/**
 * @brief Pattern: Hex Vortex. Generates the next target position for drawing a series of growing, shrinking, and rotating hexagons.
 *
 * This function generates a hexagon vortex pattern, where hexagons grow and shrink over time while rotating.
 * When the outer edge of the radial axis is reached, the ball moves along the rim before shrinking back inward.
 * The ball also dwells at the center of the field. The pattern is controlled using a switch-case sequence that
 * moves between the six vertices of the hexagon.
 *
 * @param current The current position of the gantry, represented as a Positions struct.
 * @param restartPattern A flag that allows restarting the pattern. Defaults to false.
 *
 * @return Positions The next target position for the motion controller, represented as a Positions struct.
 *
 * @note The hexagon grows and shrinks by adjusting the radius incrementally (radialStepover) and rotates
 * by shifting the angular positions of each vertex. The pattern reverses direction when the radius exceeds
 * the maximum limit or falls below zero.
 */
Positions pattern_HexagonVortex(Positions current, bool restartPattern)
{
  Positions target;
  static int step = 0;                                //using switch case to track steps again
  static int segments = 100;
  static Positions p1, p2, p3, p4, p5, p6;            //vertices of the hexagon
  static bool firstRun = true;
  const int angleShift = convertDegreesToSteps(5);
  static int radialStepover = 350;                    //how much we'll increase or decrease the size of the hexagon each iteration
  static int radius = 1000;                           //starting radius

  if (firstRun || restartPattern)
  {
    p1.angular = 0;
    p1.radial = radius;
    p2.angular = convertDegreesToSteps(60);
    p2.radial = radius;
    p3.angular = convertDegreesToSteps(120);
    p3.radial = radius;
    p4.angular = convertDegreesToSteps(180);
    p4.radial = radius;
    p5.angular = convertDegreesToSteps(240);
    p5.radial = radius;
    p6.angular = convertDegreesToSteps(300);
    p6.radial = radius;
    firstRun = false;
  }

  //the step sequencer works just like the rotating square example, but with more steps
  switch (step)
  {
    case 0:
      target = drawLine(p1, p2, current, segments);
      if ((target.angular == p2.angular) && (target.radial == p2.radial))
      {
        step++;
      }
      break;

    case 1:
      target = drawLine(p2, p3, current, segments);
      if ((target.angular == p3.angular) && (target.radial == p3.radial))
      {
        step++;
      }
      break;

    case 2:
      target = drawLine(p3, p4, current, segments);
      if ((target.angular == p4.angular) && (target.radial == p4.radial))
      {
        step++;
      }
      break;

    case 3:
      target = drawLine(p4, p5, current, segments);
      if ((target.angular == p5.angular) && (target.radial == p5.radial))
      {
        step++;
      }
      break;

    case 4:
      target = drawLine(p5, p6, current, segments);
      if ((target.angular == p6.angular) && (target.radial == p6.radial))
      {
        step++;
      }
      break;

    case 5:
      target = drawLine(p6, p1, current, segments);
      if ((target.angular == p1.angular) && (target.radial == p1.radial))
      {
        step++;
      }
      break;

    case 6:
      //reset to the beginning
      step = 0;
      target = current;         //set the target position to the current position just as a default for the default option in the switch statement.

      p1.angular += angleShift; //rotate all points
      p2.angular += angleShift;
      p3.angular += angleShift;
      p4.angular += angleShift;
      p5.angular += angleShift;
      p6.angular += angleShift;

      if ((radius + radialStepover >= MAX_R_STEPS + 2000) || (radius + radialStepover <= 0)) radialStepover *= -1;    //If we're too far out of bounds, switch directions
      radius += radialStepover;  //increase or decrease the radius for the points

      p1.radial = radius;
      p2.radial = radius;
      p3.radial = radius;
      p4.radial = radius;
      p5.radial = radius;
      p6.radial = radius;

      break;

    default:
      //assuming that the step number was out of bounds, so reset it
      step = 0;
      target = current;         //set the target position to the current position just as a default for the default option in the switch statement.
      break;
  }

  return target;
}

/**
 * @brief Pattern: Pentagon Rainbow. Generates the next target position for drawing an off-center pentagon that rotates and moves.
 *
 * This function creates a pentagon pattern that is off-center, moving the center of the pentagon to a new position and
 * rotating it slightly with each iteration. The pentagon is generated using nGonGenerator and translated to the
 * appropriate location, while the center and orientation are adjusted progressively.
 *
 * @param current The current position of the gantry, represented as a Positions struct.
 * @param restartPattern A flag that allows restarting the pattern. Defaults to false.
 *
 * @return Positions The next target position for the motion controller, represented as a Positions struct.
 *
 * @note The center of the pentagon is translated and rotated slightly on each iteration, creating a "rainbow" effect
 * as the pentagon appears in different positions. The nGonGenerator and translatePoints functions are used to
 * generate and move the pentagon.
 */
Positions pattern_PentagonRainbow(Positions current, bool restartPattern)
{
  Positions target;
  //target = current;               
  static int start = 0;
  static int end = 1;
  static bool firstRun = true;
  const int vertices = 5;
  static Positions pointList[vertices];
  static int radialStepover = 500;
  const int shiftDeg = 2;
  static int angleShift = convertDegreesToSteps(shiftDeg);
  static int shiftCounter = 1;

  if (firstRun || restartPattern)
  {
    nGonGenerator(pointList, vertices, { 0, 0 }, 3000, 0.0);      //create the polygon
    translatePoints(pointList, vertices, { 4000, 0 });            //move the polygon to the appropriate spot
    firstRun = false;
  }


  target = drawLine(pointList[start], pointList[end], current, 100);

  if ((target.angular == pointList[end].angular) && (target.radial == pointList[end].radial))
  {
    start++;
    end++;
    start = modulus(start, vertices);
    end = modulus(end, vertices);
    nGonGenerator(pointList, vertices, { 0, 0 }, 3000, shiftCounter * shiftDeg);    //build a new polygon that is rotated relative to the previous one
    translatePoints(pointList, vertices, { 4000, shiftCounter * angleShift });      //move to the correct point
    shiftCounter++;
  }
  return target;
}

/**
 * @brief Pattern: Random Walk 1. Generates random target positions, moving via the shortest path to each point.
 *
 * This function creates a random walk pattern by generating random target positions in both the radial and angular axes.
 * The motion controller moves the gantry via the shortest path to each randomly generated point, resulting in random arcs.
 *
 * @param current The current position of the gantry, represented as a Positions struct.
 * @param restartPattern A flag that allows restarting the pattern. Defaults to false.
 *
 * @return Positions The next randomly generated target position for the motion controller, represented as a Positions struct.
 *
 * @note This pattern moves the gantry using the shortest path between points, leading to random arc-shaped movements.
 */
Positions pattern_RandomWalk1(Positions current, bool restartPattern)
{
  Positions target;

  // Generate a random radial position within the bounds of your system.
  int randomRadial = random(0, MAX_R_STEPS + 1); // +1 because the upper bound is exclusive

  // Generate a random angular position within a full circle in steps.
  int randomAngular = random(0, STEPS_PER_A_AXIS_REV);

  // Set the target position to the randomly generated values.
  target.radial = randomRadial;
  target.angular = randomAngular;

  return target;
}

/**
 * @brief Pattern: Random Walk 2. Generates random target positions and moves in straight lines to each point.
 *
 * This function creates a random walk pattern by generating random target positions in both the radial and angular axes.
 * Unlike Random Walk 1, this version moves the gantry in straight lines to each random point by connecting the current
 * position to the random target using the drawLine function.
 *
 * @param current The current position of the gantry, represented as a Positions struct.
 * @param restartPattern A flag that allows restarting the pattern. Defaults to false.
 *
 * @return Positions The next target position for the motion controller, represented as a Positions struct.
 *
 * @note The function generates new random points once the gantry reaches the current random target and continues the random walk.
 */
Positions pattern_RandomWalk2(Positions current, bool restartPattern)
{
  Positions target;
  static Positions randomPoint, lastPoint = current;
  static bool makeNewRandomPoint = true;

  if (makeNewRandomPoint)
  {
// Generate a random radial position within the bounds of your system.
    randomPoint.radial = random(0, MAX_R_STEPS + 1); // +1 because the upper bound is exclusive

    // Generate a random angular position within a full circle in steps.
    randomPoint.angular = random(0, STEPS_PER_A_AXIS_REV);
    makeNewRandomPoint = false;
  }

  // Set the target position to the randomly generated values.
  target = drawLine(lastPoint, randomPoint, current, 100);

  if (target.angular == randomPoint.angular && target.radial == randomPoint.radial)
  {
    makeNewRandomPoint = true;        //next time we'll generate a new random point
    lastPoint = randomPoint;          //save this as the previous point for the next iteration
  }

  return target;
}

/**
 * @brief Pattern: Accidental Butterfly. Generates the next target position for drawing a butterfly-like pattern with oscillating radial and angular movement.
 *
 * This function creates a butterfly-shaped pattern by modifying a simple spiral pattern with sine and cosine waves that adjust both the radial
 * and angular positions. The radial and angular positions are oscillated to create the butterfly pattern. I was actually trying to do something
 * entirely different and accidentally made this butterfly.
 *
 * @param current The current position of the gantry, represented as a Positions struct.
 * @param restartPattern A flag that allows restarting the pattern. Defaults to false.
 *
 * @return Positions The next target position for the motion controller, represented as a Positions struct.
 *
 * @note The pattern adds sine and cosine-based offsets to both the radial and angular positions to create oscillating movements, leading to the butterfly shape.
 * The amplitude and frequency of the sine and cosine waves can be adjusted for different effects.
 */
Positions pattern_AccidentalButterfly(Positions current, bool restartPattern)
{
//This pattern starts out exactly the same as pattern_SimpleSpiral. The only difference is that after calculating the next position
//in the spiral, it adds the sine of the current angular position to the radial axis to make a wavy line.

  Positions target;                                        //This is where we'll store the value of the next target position.

  const float angleDivisions = 100.0;                      //Going to divide a full revolution into 100ths. "const" because this never changes.
  const int radialDivisions = 10 * (int)angleDivisions;    //Going to divide the radial axis into 1000ths. Try changing the 10 to a different number, like 20.

  //Add in values for the amplitude and frequency of the sine wave
  const float amplitude = 200.0;
  const int frequency = 8;

  //Calculate how many degrees we'll move over in the angular axis for the next step.
  const int angleStep = convertDegreesToSteps(360.0 / angleDivisions);

  //Calculate how far along we'll move the radial axis for the next step. 
  static int radialStep = MAX_R_STEPS / radialDivisions;

  target.angular = current.angular + angleStep;            //Set the angular position of the new point to move to (target position)
  target.radial = current.radial + radialStep;             //Set the radial target position as the current radial position plus the radialStep

  if (target.radial > MAX_R_STEPS || target.radial < 0)
  {  //Logic to see if the targeted radial position is out of bounds of the radial axis
    radialStep *= -1;                                      //If we're out of bounds, switch the sign of the radialStep (now we move in the opposite direction)
    target.radial += 2 * radialStep;                       //We were already out of bounds, so we undo that by moving 2 radialSteps in the new direction.
  }

  //Add a new component to the radial position to make it oscillate in and out as a sine wave.
  int rOffset = (int)(200.0 * sin(8 * convertStepsToRadians(target.angular)));
  int aOffset = (int)(40.0 * cos(3 * convertStepsToRadians(target.angular)));
  target.radial += rOffset;


  //Now do the same for the angular axis so we get some back and forth:
  target.angular += aOffset;

  return target;        //Return the target position so that the motion control functions can move to it.
}
