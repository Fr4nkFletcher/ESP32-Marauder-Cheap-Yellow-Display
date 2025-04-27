#include "Arduino.h"
#include "Audio.h"
#include "SD.h"
#include "SPI.h"
#include "FS.h"
#include "Ticker.h"

// Digital I/O used
#define SD_CS          5
#define SPI_MOSI      23
#define SPI_MISO      19
#define SPI_SCK       18
#define I2S_DOUT      25
#define I2S_BCLK      27
#define I2S_LRC       26

Audio audio;
Ticker ticker;
struct tm timeinfo;
time_t now;

uint8_t hour    = 6;
uint8_t minute  = 59;
uint8_t sec     = 45;

bool f_time     = false;
int8_t timefile = -1;
char chbuf[100];

void tckr1s(){
    sec++;
    if(sec > 59)   {sec = 0;     minute++;}
    if(minute > 59){minute = 0; hour++;}
    if(hour > 23)  {hour = 0;}
    if(minute == 59 && sec == 50) f_time = true;  // flag will be set 10s before full hour
    Serial.printf("%02d:%02d:%02d\n", hour, minute, sec);
}

void setup() {
    Serial.begin(115200);
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
    SD.begin(SD_CS);
    audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
    audio.setVolume(10); // 0...21
    ticker.attach(1, tckr1s);
}

void loop(){
    audio.loop();
    if(f_time == true){
        f_time = false;
        timefile = 3;
        uint8_t next_hour = hour + 1;
        if(next_hour == 25) next_hour = 1;
        sprintf(chbuf, "/voice_time/%03d.mp3", next_hour);
        audio.connecttoFS(SD, chbuf);
    }
}

void audio_eof_mp3(const char *info){  //end of file
    //Serial.printf("file :%s\n", info);
    if(timefile>0){
        if(timefile==1){audio.connecttoFS(SD, "/voice_time/080.mp3");     timefile--;}  // stroke
        if(timefile==2){audio.connecttoFS(SD, "/voice_time/200.mp3");     timefile--;}  // precisely
        if(timefile==3){audio.connecttoFS(SD, "/voice_time/O'clock.mp3"); timefile--;}
    }
}

