#pragma once

#ifndef flipperLED_h
#define flipperLED_h

#include "configs.h"
#include "settings.h"

#include <Arduino.h>

#define B_PIN 17
#define G_PIN 16
#define R_PIN 4

extern Settings settings_obj;

class flipperLED {

  public:
    void RunSetup();
    void main();
    void attackLED();
    void sniffLED();
    void offLED();
};

#endif
