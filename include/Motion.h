/*
This file is part of Sand-Garden-Light.

Sand-Garden-Light is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation version 3 or later.

Sand-Garden-Light is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with Sand-Garden-Light. If not, see <https://www.gnu.org/licenses/>.
*/
#pragma once

#include <AccelStepper.h>    //Controls the stepper motors
#include "MathUtils.h"

class OneButtonTiny;
class LedDisplay;

/**
 * @brief Typedef for storing pointers to pattern-generating functions.
 * 
 * This typedef defines a custom data type PatternFunction for storing pointers to pattern functions. 
 * It allows pattern functions to be called by passing the appropriate index number to an array of pattern function pointers, 
 * simplifying the process of switching between patterns. Each pattern function takes a Positions struct and a bool as parameters 
 * and returns the next target position as a Positions struct.
 * 
 * @typedef PatternFunction
 * 
 * This typedef enables pattern switching by indexing into an array of pattern functions, making it easy to select and execute 
 * different patterns dynamically.
 */
typedef Positions (*PatternFunction)(Positions, bool);

// Handle motion on the sand table
class Motion
{
public:
  Motion(OneButtonTiny &mainButton, LedDisplay &mainDisplay);

  void setup();
  void enable(bool enable);

  void manualMove(const Positions &joystickValues);
  void patternMove(PatternFunction patternFunc, bool patternSwitched);

  /**
   * @brief Calculates the effective radial change, accounting for the motion of the angular axis.
   *
   * The radial axis movement is influenced by the angular axis movement, so this function computes the 
   * actual change in the radial axis by considering the steps taken by both the angular and radial motors.
   * 
   * @param angularMoveInSteps The number of steps the angular motor has moved.
   * @param radialMoveInSteps The number of steps the radial motor has moved.
   * 
   * @return int The effective radial change in steps, with the angular axis movement accounted for. 
   *         A positive value indicates a decrease in radius, while a negative value indicates an increase in radius.
   */
  int calcRadialChange(int angularMoveInSteps, int radialMoveInSteps);

  /**
   * @brief Moves both the angular and radial motors to their target positions simultaneously.
   *
   * This function performs relative movements of the motors by taking in the number of steps
   * required for each motor to reach its target. One motor will move at maximum speed, while the 
   * speed of the other motor is scaled to ensure both motors reach their target positions at the 
   * same time. Note that this is a blocking function, meaning no other code will execute while 
   * the motors are moving.
   *
   * @param angularSteps The number of steps the angular motor needs to move to reach its target.
   * @param radialSteps The number of steps the radial motor needs to move to reach its target.
   * 
   * The function adjusts the speed of the motors proportionally based on the distance each motor 
   * needs to travel, ensuring they complete their movements simultaneously. It also reduces the 
   * maximum speed of the angular motor based on the current radial position to avoid excessive 
   * speed at the outer edges.
   *
   * The function checks the state of the run/stop button during execution to allow for immediate 
   * termination of the movement if needed.
   */
  void moveToPosition(long angularSteps, long radialSteps);

  /**
   * @brief Calculates the number of steps required for the radial axis motor to move, accounting for the angular axis motion.
   *
   * This function computes the necessary steps for the radial axis motor to move from the current position 
   * to the target position. It compensates for the fact that the angular axis motion influences the radial 
   * axis but not vice versa. The function adjusts the radial movement based on the planned angular axis movement.
   *
   * @param current The current position of the radial axis in steps.
   * @param target The desired target position of the radial axis in steps.
   * @param angularOffsetSteps The number of steps the angular axis motor will move in the next planned move.
   * 
   * @return int The total number of steps the radial axis motor needs to move, adjusted for the angular axis offset.
   */
  int calcRadialSteps(int current, int target, int angularOffsetSteps);

  /**
   * @brief Manages the entire process of moving both the angular and radial motors to their target positions.
   *
   * This function is responsible for coordinating the motion of both motors, ensuring that the angular 
   * values wrap correctly, that the radial target stays within the defined limits, and that the radial 
   * movement compensates for any angular axis movement. It encapsulates the series of steps required to 
   * calculate the necessary movements, execute them, and update the current positions.
   *
   * @param targetPositions The desired target position for both the angular and radial axes, represented as a Positions struct.
   * 
   * This function wraps the angular target around the 360-degree transition point and calculates the shortest path 
   * to the target. It also ensures that the radial position stays within its limits, compensates for the mechanical 
   * relationship between the axes, and updates the current position after the move. If the move is interrupted (e.g., 
   * by a long joystick press), the current position tracking adjusts accordingly.
   */
  void orchestrateMotion(Positions targetPositions);

private:
  /**
   * @brief Performs crash homing on the radial axis at startup.
   *
   * This function moves the radial axis to its home position by driving the motor past the known range 
   * to ensure a hard stop at the mechanical limit. It allows the homing process to be interrupted early 
   * by a long press of the joystick button if the ball reaches the center of the sand garden.
   *
   * @details The function moves the radial axis at a high speed without acceleration to reduce torque 
   * when it reaches the mechanical stop. During the homing sequence, the function updates the LED display 
   * and checks for a long press of the joystick button to potentially terminate the homing process early. 
   * After reaching the stop, the function retracts the motor slightly to create a soft stop, releases any 
   * tension in the mechanism, and sets the current motor position as the origin (0,0).
   *
   * @note This function sets the current position of both the angular and radial motors to zero after homing.
   *
   * @return void
   */
  void homeRadius();

  /**
   * @brief Handles moving and checking when the end is reach.
   *
   * This is the "blocking" move function. If a movement is currently underway, this continues moving to the target
   * but doesn't do a hard block so the main loop can do other things at the same time (like the light ring).
   */
  void contineMoving();

  /**
   * @brief Handles updating the CurrentPosition when movement is done
   *
   * Updates the CurrentPosition when we stop moving, either by reaching the target or if movement is canceled early
   */
  void stopMoving();

private:
  static bool ButtonLongPressed;

  OneButtonTiny &Button;
  LedDisplay &Display;

  //Create two objects, one for each stepper motor.
  AccelStepper StepperAngle;     //angular axis motor
  AccelStepper StepperRadius;    //radial axis motor

  Positions CurrentPosition;  // store the current positions of the axes in this
  Positions RelativeTarget;   // How much we are currently moving.
  bool MotorsEnabled;         // used to track if motor drivers are enabled/disabled. initializes to enabled so the homing sequence can run.
  bool CurrentlyMoving;       // Track if currently doing a "blocking" pattern move
};
