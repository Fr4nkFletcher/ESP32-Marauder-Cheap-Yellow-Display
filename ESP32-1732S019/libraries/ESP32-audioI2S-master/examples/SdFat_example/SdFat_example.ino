//**********************************************************************************************************
//*    audioI2S-- I2S audiodecoder for ESP32,  SdFat example                                                             *
//**********************************************************************************************************
//
// first release on 05/2020
// updated on Sep. 27, 2021
/*

 ⒈ install SdFat V2 from https://github.com/greiman/SdFat
 ⒉ activate "SDFATFS_USED"                   in Audio.h
 ⒊ activate "#define USE_UTF8_LONG_NAMES 1"  in SdFatConfig.h

*/

#include "Arduino.h"
#include "Audio.h"
#include "SPI.h"

// Digital I/O used
#define SD_CS          5
#define SPI_MOSI      23
#define SPI_MISO      19
#define SPI_SCK       18
#define I2S_DOUT      25
#define I2S_BCLK      27
#define I2S_LRC       26

Audio audio;

void setup() {
    pinMode(SD_CS, OUTPUT);      digitalWrite(SD_CS, HIGH);
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
    SPI.setFrequency(1000000);
    Serial.begin(115200);
    SD.begin(SD_CS);

    audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
    audio.setVolume(12); // 0...21

//    audio.connecttoFS(SD, "test.mp3");
    audio.connecttoFS(SD, "良い一日私の友達.mp3");
}

void loop()
{
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
