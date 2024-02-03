#include <SwitchLib.h>

SwitchLib::SwitchLib() {
	this->pin = 0;
	this->pin = false;
	this->pressed = false;
	this->hold_lim = 2000;
	this->cur_hold = 0;
	this->isheld = false;
	
	pinMode(this->pin, INPUT);
	
	return;
}

SwitchLib::SwitchLib(int pin, uint32_t hold_lim, bool pullup) {
	this->pin = pin;
	this->pullup = pullup;
	this->pressed = false;
	this->hold_lim = hold_lim;
	this->cur_hold = 0;
	this->isheld = false;
	
	pinMode(this->pin, INPUT);
	
	return;
}

int SwitchLib::getPin() {
	return this->pin;
}

bool SwitchLib::getPullup() {
	return this->pullup;
}

bool SwitchLib::isHeld() {
	return this->isheld;
}

bool SwitchLib::getButtonState() {
	int buttonState = digitalRead(this->pin);
	
	if ((this->pullup) && (buttonState == LOW))
		return true;
	else if ((!this->pullup) && (buttonState == HIGH))
		return true;
	else
		return false;
}

bool SwitchLib::justPressed() {
	bool btn_state = this->getButtonState();
	
	// Button was JUST pressed
	if (btn_state && !this->pressed) {
		this->hold_init = millis();
		this->pressed = btn_state;
		return true;
	}
	else if (btn_state) { // Button is STILL pressed
		// Check if button is held
		//Serial.println("cur_hold: " + (String)this->cur_hold);
		if ((millis() - this->hold_init) < this->hold_lim) {
			this->isheld = false;
		}
		else {
			this->isheld = true;
		}
		
		this->pressed = btn_state;
		return false;
	}
	else { // Button is not pressed
		this->pressed = btn_state;
		this->isheld = false;
		return false;
	}
}

bool SwitchLib::justReleased() {
	bool btn_state = this->getButtonState();
	
	// Button was JUST released
	if (!btn_state && this->pressed) {
		this->isheld = false;
		this->pressed = btn_state;
		return true;
	}
	else { // Button is STILL released
		this->pressed = btn_state;
		return false;
	}
	
}