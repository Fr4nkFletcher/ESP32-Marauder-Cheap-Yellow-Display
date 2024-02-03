#ifndef SwitchLib_h
#define SwitchLib_h

#include <Arduino.h>

class SwitchLib {
	private:
		int pin;
		uint32_t hold_lim;
		uint32_t cur_hold;
		
		uint32_t hold_init = millis();
		
		bool isheld;
		
		bool pullup;
		
		bool pressed;
		
		bool getButtonState();
		
	public:
		SwitchLib();
		
		SwitchLib(int pin, uint32_t hold_lim, bool pullup);
		
		int getPin();
		bool getPullup();
		
		bool justPressed();
		bool justReleased();
		bool isHeld();
};

#endif