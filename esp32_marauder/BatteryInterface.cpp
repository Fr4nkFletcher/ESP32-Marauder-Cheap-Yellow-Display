#include "BatteryInterface.h"
#include "lang_var.h"
BatteryInterface::BatteryInterface() {
  
}

void BatteryInterface::main(uint32_t currentTime) {
  if (currentTime != 0 && this->i2c_supported) { // Only poll if I2C is confirmed
    if (currentTime - initTime >= 3000) {
      Serial.println("Checking Battery Level");
      this->initTime = millis();
      int8_t new_level = this->getBatteryLevel();
      if (this->battery_level != new_level) {
        Serial.println(text00 + (String)new_level);
        this->battery_level = new_level;
        Serial.println("Battery Level: " + (String)this->battery_level);
      }
    }
  }
}

void BatteryInterface::RunSetup() {
  #ifdef HAS_BATTERY
    Wire.begin(I2C_SDA, I2C_SCL);
    Serial.println("Checking for battery monitors...");

    // Check IP5306
    Wire.beginTransmission(IP5306_ADDR);
    byte error = Wire.endTransmission();
    if (error == 0) {
      Serial.println("Detected IP5306");
      this->has_ip5306 = true;
      this->i2c_supported = true;
    } else {
      Serial.println("No IP5306 found");
      this->has_ip5306 = false;
    }

    // Check MAX17048
    Wire.beginTransmission(MAX17048_ADDR);
    error = Wire.endTransmission();
    if (error == 0 && maxlipo.begin()) {
      Serial.println("Detected MAX17048");
      this->has_max17048 = true;
      this->i2c_supported = true;
    } else {
      Serial.println("No MAX17048 found");
      this->has_max17048 = false;
    }

    // Only enable i2c_supported if a device is actually present
    this->i2c_supported = (this->has_ip5306 || this->has_max17048);
    if (!this->i2c_supported) {
      Serial.println("No battery monitors detected; disabling I2C polling");
    }

    this->initTime = millis();
  #else
    this->i2c_supported = false; // Ensure disabled if no HAS_BATTERY
  #endif
}

int8_t BatteryInterface::getBatteryLevel() {
  if (!this->i2c_supported) {
    return -1; // No monitor present
  }

  if (this->has_ip5306) {
    Wire.beginTransmission(IP5306_ADDR);
    Wire.write(0x78);
    if (Wire.endTransmission(false) == 0 && Wire.requestFrom(IP5306_ADDR, 1)) {
      uint8_t val = Wire.read();
      switch (val & 0xF0) {
        case 0xE0: return 25;
        case 0xC0: return 50;
        case 0x80: return 75;
        case 0x00: return 100;
        default: return 0;
      }
    }
    Serial.println("IP5306 read failed");
    this->i2c_supported = false; // Disable on failure
    return -1;
  }

  if (this->has_max17048) {
    float percent = this->maxlipo.cellPercent();
    if (percent >= 100) return 100;
    if (percent <= 0) return 0;
    return (int8_t)percent;
  }

  return -1; // Fallback if no device is active
}