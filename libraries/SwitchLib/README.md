# SwitchLib
 An Arduino Library for handling tactile switches...or anything else.

## Usage
### Creating a button
```C++
SwitchLib(int pin, uint32_t hold_lim, bool pullup)
```
  - `pin`: MCU pin associated with the tactile switch
  - `hold_lim`: Threshold in ms for when the button is considered to be "held"
  - `pullup`: Button is connected to pull-up(`true`) or pull-down(`false`) resistor
### Getting button state
```C++
// Returns true if button has switched from a released state to a pressed state
bool just_pressed = mybtn.justPressed();

// Returns true if button has switched from a pressed state to a released state
bool just_released = mybtn.justReleased();

// Returns true if button meets/exceeds it's hold threshold
bool is_held = mybtn.isHeld();
```
