/*
This file is part of Sand-Garden-Light.

Sand-Garden-Light is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation version 3 or later.

Sand-Garden-Light is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with Sand-Garden-Light. If not, see <https://www.gnu.org/licenses/>.
*/

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
INCLUDED LIBRARIES
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include <Arduino.h>
#include <elapsedMillis.h>            //Creates timer objects that are more convenient for non-blocking timing than millis()
#include <OneButtonTiny.h>            //Button management and debouncing
#include "LedDisplay.h"
#include "ColorDisplay.h"
#include "Patterns.h"
#include "Motion.h"
#include "Joystick.h"
#include "Constants.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
PREPROCESSOR DIRECTIVES.

Hardware specific defines
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define JOYSTICK_A_PIN   A2          //Left-right axis of joystick, associated with changing angular axis in manual mode
#define JOYSTICK_R_PIN   A3          //Up-down axis of joystick, associated with changing radial axis in manual mode
#define BUTTON_PIN       A1          //Joystick button pin
#define RANDOM_SEED_PIN  A6          //used to generate random numbers.

#define COLOR_JOYSTICK_H_PIN   A6          //Left-right axis of joystick, associated with changing angular axis in manual mode
#define COLOR_JOYSTICK_V_PIN   A7          //Up-down axis of joystick, associated with changing radial axis in manual mode
#define COLOR_BUTTON_PIN       A5          //Joystick button pin


/**
 * @brief Array of pattern-generating functions.
 * 
 * This array stores the functions responsible for generating different patterns, defined using the PatternFunction typedef. 
 * To add a new pattern function, follow these steps:
 * 1. Declare the new pattern function prototype (e.g., Positions pattern_42(Positions current);).
 * 2. Add the new pattern function to this array.
 * 3. Define the function at the end of the code.
 * 
 * @note The array is 0-indexed, but the controller interface (joystick and LEDs) uses 1-indexing. 
 * So index 0 is set to null since it is never used. That way the index doesn't need to be adjusted at runtime.
 */
PatternFunction patterns[] = {nullptr, pattern_SimpleSpiral, pattern_Cardioids, pattern_WavySpiral, pattern_RotatingSquares, pattern_PentagonSpiral, pattern_HexagonVortex, pattern_PentagonRainbow, pattern_RandomWalk1,
                              pattern_RandomWalk2, pattern_AccidentalButterfly};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
STATE MACHINE FLAGS:
This code uses simple state machine to keep track of which mode the machine is in (e.g., actively running a pattern, or in pattern selection mode).
These flags are used in that state machine.
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint8_t currentPattern = 1;       //default to pattern 1.
bool runPattern = false;          //this will be the start/stop flag. true means run the selected pattern.
bool autoMode = true;             //tracking if we're in automatic or manual mode. Defaults to auto on startup. If you want to start in manual drawing mode, set this to false.
bool patternSwitched = false;     //used for properly starting patterns from beginning when a new pattern is selected
uint8_t lastPattern = currentPattern; //used with currentPattern to detect pattern switching and set the patternSwitched flag.


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
MISC. GLOBAL VARIABLES.
Used for tracking time and button presses.
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

elapsedMillis lastJoystickUpdate;                    //used to track the last time the joystick was updated to prevent absurdly fast scrolling

// Objects to handle main joystick
OneButtonTiny buttonMain(BUTTON_PIN, true, true);        //set up the button (button pin, active low, enable internal pull-up resistor)
Joystick joystickMain(JOYSTICK_A_PIN, JOYSTICK_R_PIN);

// Objects to handle color joystick
OneButtonTiny buttonColor(COLOR_BUTTON_PIN, true, true);        //set up the button (button pin, active low, enable internal pull-up resistor)
Joystick joystickColor(COLOR_JOYSTICK_H_PIN, COLOR_JOYSTICK_V_PIN);

// Create Ledclasses for main display & color ring
LedDisplay displayStrip;
ColorDisplay colorStrip(joystickColor, buttonColor);

Motion motion(buttonMain, displayStrip, runPattern);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
SETUP FUNCTION (runs once when Arduino powers on)
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
  Serial.begin(9600); // initializes the Serial communication between the computer and the microcontroller

  //Generate a random seed. If you want to use pseudorandom numbers in a pattern, this makes them more random.
  //Make sure that RANDOM_SEED_PIN is an analog pin that's not connected to anything.
  randomSeed(analogRead(RANDOM_SEED_PIN));

  Serial.println("Setup joysticks");
  joystickMain.setup();
  joystickColor.setup();

  //Set up the button.
  //Single press of button is for starting or stopping the current pattern.
  buttonMain.attachClick([]()
    {       //This is called a lambda function. Basically it's a nameless function that runs when the button is single pressed.
      runPattern = !runPattern;     //this flips the boolean state of the variable. If it's true, this sets to false, if false set to true.
      Serial.print("Set runPattern=");
      Serial.println(runPattern);
    });

  Serial.println("Setup displayStrip");
  displayStrip.setup();
  Serial.println("Setup colorStrip");
  colorStrip.setup();
  Serial.println("Setup motion");
  motion.setup();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
MAIN LOOP (runs endlessly).
This manages the state machine, tracks the position of the gantry, and acquires the target positions for
the gantry from the selected pattern functions or from manual mode.
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop()
{
  //Check to see if the button has been pressed. This has to be called as often as possible to catch button presses.
  buttonMain.tick();
  buttonColor.tick();

  colorStrip.updateColor();

  //if the runPattern flag is set to true, we need to start updating target positions for the controller. run the appropriate pattern!
  if (runPattern)
  {
    #pragma region Running
    //make sure the motors are enabled, since we want to move them
    motion.enable(true);

    //First we'll check the state machine flags to see if we're in manual drawing mode.
    if (!autoMode)           //Not in autoMode, which means we're in manual drawing mode
    {
      #pragma region ManualMode

      //Use the LEDs to indicate that we are in manual drawing mode
      displayStrip.setMaxBrightness();
      displayStrip.indicatePattern(255);             //pattern 255 is used to indicate manual drawing mode on the LEDs

      auto joystickValues = joystickMain.read();          //read joystick values and store them in joystickValues struct
      motion.manualMove(joystickValues);

      #pragma endregion ManualMode
    }
    else //In this case, the state machine flags indicate that we're in automatic pattern mode, not manual mode.
    {                                        //automatic pattern mode
      #pragma region AutomaticMode
      //update the LED pattern display
      displayStrip.setMaxBrightness();
      displayStrip.indicatePattern(currentPattern);     
      
      //check to see if the pattern has been switched
      if (currentPattern != lastPattern)
      {
        patternSwitched = true;               //set the flag to indicate that the pattern has been changed
        lastPattern = currentPattern;         //now we can say that the last patten is the current pattern so that this if block will be false until pattern is changed again
      }

      motion.patternMove(patterns[currentPattern], patternSwitched);
      patternSwitched = false;    //after we've called the pattern function above, we can reset this flag to false.

      #pragma endregion AutomaticMode
    }
    #pragma endregion Running
  }
  else
  {    //In this case, runPattern is false, which means this is pattern selection mode
    #pragma region SelectionMode

    //if the motors are enabled, disable them to save power while they don't need to run
    motion.enable(false);

    //read the joystick state so that it can be used in the following if statements
    auto joystickValues = joystickMain.read();

    if (!autoMode)                                          //This means we're not in automatic mode, so we are in manual drawing mode.
    {
      displayStrip.indicatePattern(255);                         //The value 255 is used to represent manual mode on the LEDs.
      displayStrip.fadePixels();  //update the brightness of the LEDs to fade them in and out over time

      if (lastJoystickUpdate >= 200 && (joystickValues.angular >= 90 || joystickValues.angular <= -90))
      {  //the joystick is pushed all the way to the right or left
        autoMode = true;                                    //switch to automatic mode so that we can do pattern selection
        lastJoystickUpdate = 0;                             //reset the joystick update timer
      }
    }
    else
    {                                                //We're in automatic mode, which means it's time to select a pattern.
      displayStrip.indicatePattern(currentPattern);
      displayStrip.fadePixels();  

      if (lastJoystickUpdate >= 200 && joystickValues.radial >= 90)
      {                              //if it's been 200ms since last joystick update and joystick is pushed all the way up
        currentPattern++;                                                                          //increment pattern number by 1
        if ((currentPattern == 0) || (currentPattern > sizeof(patterns)/sizeof(patterns[0])))
        {  //if currentPattern equals 255 or the number of elements in the pattern array
          currentPattern = 1;                               //this handles wrapping back around to beginning of patterns.
        }
        lastJoystickUpdate = 0;                             //reset the timer that tracks the last time the joystick was updated
      }
      else if (lastJoystickUpdate >= 200 && joystickValues.radial <= -90)
      {                      //if it's been 200ms since last update and joystick is pushed all the way down
        currentPattern--;
        if (currentPattern == 0)
        {
          currentPattern = sizeof(patterns)/sizeof(patterns[0]);   //this handles wrapping up to the top end of the array that holds the patterns
        }
        lastJoystickUpdate = 0;

      }
      else if (lastJoystickUpdate >= 200 && (joystickValues.angular >= 90 || joystickValues.angular <= -90))
      {  //if the joystick was pushed all the way to the left
        autoMode = false;                                                                                         //switch to manual mode
        lastJoystickUpdate = 0;   
      }   
    }
    #pragma endregion SelectionMode
  }
}











/*
NOTES:
  -the max distance the rack can travel is 87.967mm.
  -that's spread over 28 teeth of movement of the pinion.
  -that's 3.141678 mm per tooth. love how close that is to pi.
  -the pinion is 16 teeth. 
  -1 full revolution of the pinion should move the rack 50.267mm.
  -the pulley ratio is 2:1, so two revs of the motor cause one rev of the pinion
  -there are 2048 steps per rev in the motor (may need to update with exact number).
  Note that this is an approximation since the gearbox on the motor is not an integer ratio.
  -4096 steps will drive the pinion one full revolution (so 4096 steps per 50.267mm)
  -that makes 81.485 steps per mm of rack travel. 
  -or the inverse of that if needed: .01227 mm per step
  -to fully move the rack across 87.967mm, need 7167.991 steps. 
  -round down to 7000 steps total, losing about 2mm of total travel, and use 1mm on each end
  as a soft buffer on the travel to prevent crashing.



POSSIBLE IMPROVEMENTS TO MAKE:
- Add backlash compensation. The motor gearboxes have backlash, as do the drive belts and pulleys. Every time the motor
switches direction, it has to spin a bit to take up all the slack before actually moving the gantry. It works fine as it is,
but for greater precision, take direction changes into account and add extra steps to each move to take up the slack before
executing the desired movement. 

- Add functionality for the rest of the geometric transformations: I already added translation, so add rotation, scaling,
and reflection. 

- Add proportional mixing to manual drawing mode. Right now it just works like arrow keys. Mixing would make the control
more subtle and precise.

- Normalize the speed of the ball. Right now the ball travels in a range of speeds, but it would be cool to make a sytem
that would ensure that the ball always moves at the same speed. Requires more advanced path planning than what I have here.
Note that I have a hack in place for this that makes the speed of the angular motor inversely proportional to the radius.
It actually works pretty well, but isn't the same as normalized speed.
*/
