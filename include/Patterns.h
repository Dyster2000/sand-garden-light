/*
This file is part of Sand-Garden-Light.

Sand-Garden-Light is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation version 3 or later.

Sand-Garden-Light is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with Sand-Garden-Light. If not, see <https://www.gnu.org/licenses/>.
*/
#pragma once

#include "MathUtils.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
This region of code contains the different pattern generating functions.
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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
typedef Positions(*PatternFunction)(Positions, bool);

//Function prototypes for pattern generators. Each pattern function has to return a struct of type Positions. 
//This will be used as the target position for the motion controller. Note that these are just
//function prototypes. They are put up here to let the compiler know that they will be defined later in the code.
Positions pattern_SimpleSpiral(Positions current, bool restartPattern = false);               //Simple spiral. Grows outward, then inward.
Positions pattern_Cardioids(Positions current, bool restartPattern = false);                  //Cardioids
Positions pattern_WavySpiral(Positions current, bool restartPattern = false);                 //Wavy spiral.
Positions pattern_RotatingSquares(Positions current, bool restartPattern = false);            //Rotating squares
Positions pattern_PentagonSpiral(Positions current, bool restartPattern = false);             //Pentagon spiral
Positions pattern_HexagonVortex(Positions current, bool restartPattern = false);              //Hexagon vortex
Positions pattern_PentagonRainbow(Positions current, bool restartPattern = false);            //Pentagon rainbow
Positions pattern_RandomWalk1(Positions current, bool restartPattern = false);                //Random walk 1 (connected by arcs)
Positions pattern_RandomWalk2(Positions current, bool restartPattern = false);                //Random walk 2 (connected by lines)
Positions pattern_AccidentalButterfly(Positions current, bool restartPattern = false);        //Accidental Butterfly
