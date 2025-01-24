/*
This file is part of Sand-Garden-Light.

Sand-Garden-Light is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation version 3 or later.

Sand-Garden-Light is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with Sand-Garden-Light. If not, see <https://www.gnu.org/licenses/>.
*/
#pragma once

struct Positions;

class Joystick
{
public:
  Joystick(uint8_t horizontalPin, uint8_t verticalPin);

  void setup();

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
  Positions read();

  /**
   * @brief Reads the analog values from the joystick potentiometers and returns them as a Positions struct converted to polar coordinates.
   *
   * This function reads the analog input values from the joystick's potentiometers on the specified pins,
   * maps the values to a range of -100 to 100 for the angular axis and 100 to -100 for the radial axis, 
   * and applies a deadband to eliminate small fluctuations around the center.
   * Once read, the X,Y values are converted to polar coordinates.
   *
   * @return Positions - a struct containing the mapped and processed values of the angular and radial joystick positions.
   * 
   * The deadband ensures that values near the center of the joystick are treated as zero to account for 
   * measurement noise and prevent unintended small movements.
   */
  Positions readPolar();

private:
  uint8_t HorizontalPin;
  uint8_t VerticalPin;
};
