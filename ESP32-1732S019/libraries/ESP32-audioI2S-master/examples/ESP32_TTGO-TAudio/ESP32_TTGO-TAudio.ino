// Copied from https://github.com/LilyGO/TTGO-TAudio/issues/12


// Required Libraries (Download zips and add to the Arduino IDE library).
#include "Arduino.h"
#include <WM8978.h> // https://github.com/CelliesProjects/wm8978-esp32
#include <Audio.h>  // https://github.com/schreibfaul1/ESP32-audioI2S

// T-Audio 1.6 WM8978 I2C pins.
#define I2C_SDA     19
#define I2C_SCL     18

// T-Audio 1.6 WM8978 I2S pins.
#define I2S_BCK     33
#define I2S_WS      25
#define I2S_DOUT    26

// T-Audio 1.6 WM8978 MCLK gpio number
#define I2S_MCLKPIN  0

Audio audio;
WM8978 dac;

void setup() {
    Serial.begin(115200);

  // Setup wm8978 I2C interface.
  if (!dac.begin(I2C_SDA, I2C_SCL)) {
    ESP_LOGE(TAG, "Error setting up dac: System halted.");
    while (1) delay(100);
  }

  // Select I2S pins
  audio.setPinout(I2S_BCK, I2S_WS, I2S_DOUT);
  audio.i2s_mclk_pin_select(I2S_MCLKPIN);

  // WiFi Settings here.
  WiFi.begin("EnterSSIDHere", "EnterPasswordHere");
  while (!WiFi.isConnected()) {
    delay(10);
  }
  ESP_LOGI(TAG, "Connected. Starting MP3...");
  // Enter your Icecast station URL here.
  audio.setVolume(21);
  audio.connecttohost("http://hestia2.cdnstream.com/1458_128");
  // Volume control.
  dac.setSPKvol(63); // Change volume here for board speaker output (Max 63).
  dac.setHPvol(63, 63); // Change volume here for headphone jack left, right channel.
}

void loop() {
  // Start the stream.
  audio.loop();
}

// optional
void audio_info(const char *info){
    Serial.print("info        "); Serial.println(info);
}
void audio_id3data(const char *info){  //id3 metadata
    Serial.print("id3data     ");Serial.println(info);
}
void audio_eof_mp3(const char *info){  //end of file
    Serial.print("eof_mp3     ");Serial.println(info);
}
void audio_showstation(const char *info){
    Serial.print("station     ");Serial.println(info);
}
void audio_showstreamtitle(const char *info){
    Serial.print("streamtitle ");Serial.println(info);
}
void audio_bitrate(const char *info){
    Serial.print("bitrate     ");Serial.println(info);
}
void audio_commercial(const char *info){  //duration in sec
    Serial.print("commercial  ");Serial.println(info);
}
void audio_icyurl(const char *info){  //homepage
    Serial.print("icyurl      ");Serial.println(info);
}
void audio_lasthost(const char *info){  //stream URL played
    Serial.print("lasthost    ");Serial.println(info);
}
void audio_eof_speech(const char *info){
    Serial.print("eof_speech  ");Serial.println(info);
}
