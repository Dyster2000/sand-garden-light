/*
This file is part of Sand-Garden-Light.

Sand-Garden-Light is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation version 3 or later.

Sand-Garden-Light is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with Sand-Garden-Light. If not, see <https://www.gnu.org/licenses/>.
*/
#define STEPS_PER_MOTOR_REV   2048                                    //Number of motor steps in one revolution of the output shaft of the motor. 
#define STEPS_PER_A_AXIS_REV  2 * STEPS_PER_MOTOR_REV                 //the number of steps required to move the angular axis one full revolution
#define TRAVEL_PER_PINION_REV 50.267                                  //Distance in mm the rack moves in one complete revolution of the pinion.
#define STEPS_PER_MM          81.4849                                 //Precomputed motor steps per mm of radial axis travel. 
#define MM_PER_STEP           1.0 / STEPS_PER_MM                      //Millimeters of travel per motor step on radial axis. Evaluates to 0.01227 if STEPS_PER_REV is 2048. 
#define STEPS_PER_DEG         (STEPS_PER_A_AXIS_REV) / 360            //Motor steps per degree of motion on angular axis. Should be about 11.378 steps per degree.
#define DEG_PER_STEP          1 / STEPS_PER_DEG                       //Degrees of rotation on angular axis per motor step. About .08799 degrees.
#define STEPS_PER_RAD         STEPS_PER_MOTOR_REV / PI                //Motor steps per radian of motion on angular axis. About 652. 
#define RAD_PER_STEP          1 / STEPS_PER_RAD                       //Radians travelled on angular axis per motor step. About 0.00153

#define ACTUAL_LEN_R_MM       87.967                                  //Length in mm of the radial axis (hard limits). Derived from the CAD model of the hardware.
#define ACTUAL_LEN_R_STEPS    ACTUAL_LEN_R_MM * STEPS_PER_MM          //Maximum possible length of radius in steps of motor (hard limits). Should be 7167 when 2048 steps per rev in motor.
#define MAX_R_STEPS           7000                                    //Soft limit on how far the radius can move in terms of steps of the motor. This leaves a slight buffer on each end.
#define MAX_R_MM              MAX_R_STEPS * MM_PER_STEP               //Soft limit length in mm of the radial axis. 85.91mm. 

#define HOMING_BUFFER         (ACTUAL_LEN_R_STEPS - MAX_R_STEPS) / 2  //Crash home R axis to 0, then move this many steps in positive direction to create a soft stop.
#define RELAXATION_BUFFER     STEPS_PER_DEG * 5                       //Crash homing tensions the bead chain, and backlash and flex in the gantry need to be released.

#define MAX_SPEED_R_MOTOR     550.0                                   //Maximum speed in steps per second for radius motor. Faster than this is unreliable.
#define MAX_SPEED_A_MOTOR     550.0                                   //Maximum speed in steps per second for angle motor.

//The following is used to reduce angular speed linearly with the current position on the radial axis.
//This helps the ball move at a more consistent speed through the sand regardless of how far out it is on the radial axis.
//This is just a linear function that can be fine tuned by changing the amount removed from the max speed (currently 50.0).
//Essentially what this does is drops the speed of the angular axis to 50.0 steps per second at the very outer edge of 
//the actual length of the radial axis. This point is unreachable in typical use because of the soft limits.
#define ANGULAR_SPEED_SCALAR  (MAX_SPEED_A_MOTOR  - 150.0) / (MAX_R_STEPS)    
