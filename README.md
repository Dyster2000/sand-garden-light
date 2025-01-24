# sand-garden-light
Sand Garden with LED light ring.

This is the stock HackPack sand garden code with a LED light strip added and joystick to control the lights.

Open with VSCode using PlatformIO.

## stock code
The following files are taken directly from the stock code (originally all in main.ino), but split into different files & classes based on purpose. This was done for readability and to more easily add new code needed for light ring.
- GeometryUtils.cpp/h
- MathUtils.cpp/h
- Motion.cpp/h
- Patterns.cpp/h
- Constants.h

## modified stock code
The following started from the stock code, but contain modifications to support the new color ring.
- main.ino
- LedDisplay.cpp/h
- Joystick.cpp/h

## custom code
The following files are new and created by Jason Blood to handle the LED color ring.
- ColorDisplay.cpp/h
