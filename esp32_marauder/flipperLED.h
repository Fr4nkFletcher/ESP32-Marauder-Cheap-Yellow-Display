#pragma once

#ifndef flipperLED_h
#define flipperLED_h

#include "configs.h"
#include "settings.h"

#include <Arduino.h>

#if defined(CYD_28) || defined(CYD_24G) || defined(CYD_24CAP) || defined(CYD_22CAP)
  #define B_PIN 17
  #define G_PIN 16
  #define R_PIN 4
#elif defined(CYD_24) || defined(CYD_35) || defined(CYD_35CAP) || defined(CYD_32) || defined(CYD_32CAP) 
  #define B_PIN 16
  #define G_PIN 17
  #define R_PIN 4
#endif

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
