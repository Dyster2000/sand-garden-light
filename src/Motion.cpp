/*
This file is part of Sand-Garden-Light.

Sand-Garden-Light is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation version 3 or later.

Sand-Garden-Light is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with Sand-Garden-Light. If not, see <https://www.gnu.org/licenses/>.
*/
#include <Arduino.h>
#include <OneButtonTiny.h>    //Button management and debouncing
#include "Motion.h"
#include "Constants.h"
#include "LedDisplay.h"

//Pin definitions follow. 
//The #ifdef / #endif blocks are used to check to see if either REVERSE_R_MOTOR or REVERSE_A_MOTOR
//is defined at the very top of the code, and if they are, the order the pins are defined in changes.

#ifdef REVERSE_A_MOTOR
  #define MOTORA_IN1_PIN   12
  #define MOTORA_IN2_PIN   11
  #define MOTORA_IN3_PIN   10
  #define MOTORA_IN4_PIN   9
#endif

#ifndef REVERSE_A_MOTOR
  #define MOTORA_IN1_PIN   9
  #define MOTORA_IN2_PIN   10
  #define MOTORA_IN3_PIN   11
  #define MOTORA_IN4_PIN   12
#endif

#ifdef REVERSE_R_MOTOR
  #define MOTORR_IN1_PIN   5         
  #define MOTORR_IN2_PIN   6         
  #define MOTORR_IN3_PIN   7
  #define MOTORR_IN4_PIN   8
#endif

#ifndef REVERSE_R_MOTOR
  #define MOTORR_IN1_PIN   8         //The motor is flipped upside down in assembly, so pin order is reversed from other motor.
  #define MOTORR_IN2_PIN   7         
  #define MOTORR_IN3_PIN   6
  #define MOTORR_IN4_PIN   5
#endif

bool Motion::ButtonLongPressed = false;
  
Motion::Motion(OneButtonTiny &mainButton, LedDisplay &mainDisplay)
  : Button{ mainButton }
  , Display{ mainDisplay }
  , StepperAngle(4, MOTORA_IN1_PIN, MOTORA_IN3_PIN, MOTORA_IN2_PIN, MOTORA_IN4_PIN)
  , StepperRadius(4, MOTORR_IN1_PIN, MOTORR_IN3_PIN, MOTORR_IN2_PIN, MOTORR_IN4_PIN)
  , MotorsEnabled{ false }
  , CurrentlyMoving{ false }
{
}

void Motion::setup()
{
  //enable the motor
  enable(true);

  //set the maximum speeds and accelerations for the stepper motors.
  StepperAngle.setMaxSpeed(MAX_SPEED_A_MOTOR);
  StepperAngle.setAcceleration(5000.0);           // Need high acceleration without losing steps. 
  StepperRadius.setMaxSpeed(MAX_SPEED_R_MOTOR);
  StepperRadius.setAcceleration(5000.0);

  //crash home the radial axis. This is a blocking function.
  homeRadius();

  //disable the motors until they are needed
  enable(false);
}

void Motion::enable(bool enable)
{
  if (MotorsEnabled == enable)
    return; // Already in target state

  if (enable)
  {
    StepperAngle.enableOutputs();     //enable the motor
    StepperRadius.enableOutputs();
  }
  else
  {
    StepperAngle.disableOutputs();     //enable the motor
    StepperRadius.disableOutputs();
  }
  MotorsEnabled = enable;

  // enable(false) is called whenever we stop drawing. Handle if we were in the middle of a move to update the new current position
  if (CurrentlyMoving && !MotorsEnabled)
  {
    stopMoving();
  }
}

void Motion::manualMove(const Positions &joystickValues)
{
  if (!CurrentlyMoving)
  {
    Positions targetPositions;

    //first check if an angular change is requested by joystick input
    if (joystickValues.angular < 0)
    {
      targetPositions.angular = CurrentPosition.angular - (STEPS_PER_MOTOR_REV / 100);    //add steps to the target position
    }
    else if (joystickValues.angular > 0)
    {
      targetPositions.angular = CurrentPosition.angular + (STEPS_PER_MOTOR_REV / 100);
    }
    else
    {
      targetPositions.angular = CurrentPosition.angular;   //otherwise maintain current position
    }
    //next check if a radial change is requested by joystick input
    if (joystickValues.radial < 0)
    {
      targetPositions.radial = CurrentPosition.radial - (MAX_R_STEPS / 100);
    }
    else if (joystickValues.radial > 0)
    {
      targetPositions.radial = CurrentPosition.radial + (MAX_R_STEPS / 100);
    }
    else
    {
      targetPositions.radial = CurrentPosition.radial;
    }

    //finally, take the steps necessary to move both axes to the target position in a coordinated manner and update the current position.
    orchestrateMotion(targetPositions);

    // Flag that we are moving manually. Will go false once we reach the target
    CurrentlyMoving = true;
  }
  contineMoving();
}

void Motion::patternMove(PatternFunction patternFunc, bool patternSwitched)
{
  if (!CurrentlyMoving)
  {
    //Call the function that will generate the pattern. 
    //This automatically calls the appropriate function from the patterns[] array.
    //Pass in the currentPositions as an argument, and the pattern function returns the targetPositions.
    //Note that the target positions are absolute coordinates: e.g., a pattern might say
    //to move to (radius, angle) = (1000 steps, 45 degrees (converted to steps)).
    //There is only one position on the sand tray that corresponds to those coordinates. 
    auto targetPositions = patternFunc(CurrentPosition, patternSwitched);

    //finally, take the steps necessary to move both axes to the target position in a coordinated manner and update the current position.
    orchestrateMotion(targetPositions);

    // Flag that we are moving for a pattern. Will go false once we reach the target
    CurrentlyMoving = true;
  }
  contineMoving();
}

int Motion::calcRadialChange(int angularMoveInSteps, int radialMoveInSteps)
{
  int actualChangeR = angularMoveInSteps - radialMoveInSteps;

  //should return the number of steps R axis has moved, with A axis motion accounted for.
  //if actualChangeR is positive, radius is decreasing. 
  //if actualChangeR is negative, radius is increasing.
  return actualChangeR;          
}

void Motion::moveToPosition(long angularSteps, long radialSteps)
{
  long absStepsA = abs(angularSteps), absStepsR = abs(radialSteps);           //absolute values used to compare total distance each motor travels
  float maxSpeedA = MAX_SPEED_A_MOTOR, maxSpeedR = MAX_SPEED_R_MOTOR;
  float moveTime = 0.0;
  
  // Reduce the maximum angular speed based on the radial position of the ball.
  // If you don't do this, the ball moves too fast when it's on the outer edge of the sand tray.
  float speedReducer = ANGULAR_SPEED_SCALAR * CurrentPosition.radial;  
  maxSpeedA = MAX_SPEED_A_MOTOR - speedReducer;                         
  
  float speedA = maxSpeedA, speedR = maxSpeedR;              //one of the motors will eventually be moved at max speed, the other will be slowed down.

  //determine which motor has a shorter move, and slow it down proportional to the ratio of the distance each motor travels.

  if ((absStepsA > absStepsR) && (absStepsA != 0))
  {         //if Angle motor is moving farther. the second conditional avoids a divide by zero error.
    moveTime = (float)absStepsA / maxSpeedA;                 //how long it will take to move A axis to target at top speed.
    speedR = (float)absStepsR / moveTime;                    //recalculate speed of R motor to take same time as A motor move. Slows down R motor.

  }
  else if ((absStepsR > absStepsA) && (absStepsR != 0))
  {
    moveTime = (float)absStepsR / maxSpeedR;                 //Radial is moving farther. Time in seconds to make that move at max speed.
    speedA = (float)absStepsA / moveTime;                    //Slow down A to complete its move in same amount of time as R.
  }

  //set up the moves for each motor
  StepperAngle.move(angularSteps);       //set up distance the motor will travel in steps. This value can be positive or negative: the sign determines the direction the motor spins.
  StepperAngle.setSpeed(speedA);         //call this to ensure that the motor moves at constant speed (no accelerations).
  StepperRadius.move(radialSteps);
  StepperRadius.setSpeed(speedR);
}

void Motion::contineMoving()
{
  //constant speed move until the target position is reached.
  if ((StepperAngle.distanceToGo() != 0) || (StepperRadius.distanceToGo() != 0))
  {
    StepperAngle.runSpeedToPosition();
    StepperRadius.runSpeedToPosition();
  }
  else
    stopMoving();
}

void Motion::stopMoving()
{
  // Set flag that we reached the target
  CurrentlyMoving = false;

  //Update the current position.
  //movement can be stopped before the move is complete by long pressing the joystick button, so we have
  //to make sure that our position tracking system accounts for that. We also have to use the target positions
  //to update the current position.
  RelativeTarget.angular -= StepperAngle.distanceToGo();
  RelativeTarget.radial -= StepperRadius.distanceToGo();
  CurrentPosition.angular += RelativeTarget.angular;
  CurrentPosition.angular = modulus(CurrentPosition.angular, STEPS_PER_A_AXIS_REV); //wrap the anglular position around if it needs it. 
  CurrentPosition.radial += calcRadialChange(RelativeTarget.angular, RelativeTarget.radial);
}

void Motion::homeRadius()
{
  Button.attachLongPressStart([]()
    {
      ButtonLongPressed = true;
    });

  Serial.println("[homeRadius] Start homing radius");
  StepperRadius.move(1.1 * ACTUAL_LEN_R_STEPS);                       //Longer than actual length of axis to ensure that it fully crash homes.
  StepperRadius.setSpeed(600.0);                                      //move fast without accelerations so that the motor has less torque when it crashes.
  while (StepperRadius.distanceToGo() != 0 && !ButtonLongPressed)
  {   //run the R axis toward 0 for the entire length of the axis. Crash homing.
    StepperRadius.runSpeedToPosition();                               //non-blocking move function. has to be called in while loop.
    Display.homingSequence(false);                                    //display the homing sequence pattern on the LEDs
    Button.tick();                                                    //poll the button to see if it was long pressed
  }
  Serial.println("[homeRadius] radius done");
  ButtonLongPressed = false;
  StepperRadius.stop();

  delay(100);                                                     //brief delay.
  
  Serial.println("[homeRadius] home angle");
  StepperRadius.move(-1 * (HOMING_BUFFER + RELAXATION_BUFFER));   //move away from 0 to create a soft stop. RELAXATION_BUFFER releases tension in bead chain/flexible structures
  StepperRadius.runToPosition();                                  //blocking move.

  StepperRadius.setCurrentPosition(0);                            //set the current positions as 0 steps.
  StepperAngle.setCurrentPosition(0);                             //The current locations of the motors will be the origins of motion.

  CurrentPosition.angular = 0;                                   //set the global current position variables to 0.
  CurrentPosition.radial = 0;
  Serial.println("[homeRadius] show done");
  Display.homingSequence(true);                                   //now that homing is done, display the homing complete sequence on the LEDs
  Serial.println("[homeRadius] finished");
}

int findShortestPathToPosition(int current, int target, int wrapValue)
{
  int dist1 = modulus((target - current), wrapValue);       
  int dist2 = -1 * modulus((current - target), wrapValue);

  if (abs(dist1) <= abs(dist2))
  {
    return dist1;
  }
  else
  {
    return dist2;
  }
}

int Motion::calcRadialSteps(int current, int target, int angularOffsetSteps)
{
  return ((current - target) + angularOffsetSteps);
}

void Motion::orchestrateMotion(Positions targetPositions)
{
  //First take care of making sure that the angular values wrap around correctly,
  targetPositions.angular = modulus(targetPositions.angular, STEPS_PER_A_AXIS_REV);                                                 //wrap value around the 360 degree/0 degree transition if needed
  targetPositions.angular = findShortestPathToPosition(CurrentPosition.angular, targetPositions.angular, STEPS_PER_A_AXIS_REV);    //Find the shortest path to the new position.

  //First make sure the radial position target won't exceed the limits of the radial axis:
  targetPositions.radial = constrain(targetPositions.radial, 0, MAX_R_STEPS);

  //Update the radial target position based on how much the angular position is going to move.
  //This compensates for the mechanical link between the two axes. This also converts the absolute radial coordinate
  //into a relative coordinate, which stores how many steps the radial motor has to spin. 
  targetPositions.radial = calcRadialSteps(CurrentPosition.radial, targetPositions.radial, targetPositions.angular);

  //execute the moves. This is a blocking function: it doesn't return until the move is complete.
  //Also note that these positions are relative coordinates. The pattern function generates an 
  //absolute position as the target to move to, and then the lines of code after that calculate
  //how far the motors have to move in steps to get there. moveToPosition() takes those motor 
  //steps as its arguments. So this function just tells the motors how far they have to move.
  moveToPosition(targetPositions.angular, targetPositions.radial);

  // Save the relative move so we can adjust the current position when the move is done
  RelativeTarget = targetPositions;
}
