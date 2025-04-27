// the pin assignment matches the Olimex ADF board

#include "Arduino.h"
#include "WiFi.h"
#include "SPI.h"
#include "SD.h"
#include "FS.h"
#include "Wire.h"
#include "ES8388.h"  // https://github.com/maditnerd/es8388
#include "Audio.h"   // https://github.com/schreibfaul1/ESP32-audioI2S


#define SD_CS         21

// GPIOs for SPI
#define SPI_MOSI      13
#define SPI_MISO      12
#define SPI_SCK       14

// I2S GPIOs
#define I2S_SDOUT     26
#define I2S_BCLK       5
#define I2S_LRCK      25
#define I2S_MCLK       0

// I2C GPIOs
#define IIC_CLK       23
#define IIC_DATA      18

// Amplifier enable
#define GPIO_PA_EN    19

char ssid[] =     "xxxxxxxxx";
char password[] = "xxxxxxxxx";


int volume = 80;                            // 0...100

ES8388 es;
Audio audio;

//----------------------------------------------------------------------------------------------------------------------

void setup()
{
    Serial.begin(115200);
    Serial.println("\r\nReset");
    Serial.printf_P(PSTR("Free mem=%d\n"), ESP.getFreeHeap());

    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
    SPI.setFrequency(1000000);

    SD.begin(SD_CS);

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    while(WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(100);
    }

    Serial.printf_P(PSTR("Connected\r\nRSSI: "));
    Serial.print(WiFi.RSSI());
    Serial.print(" IP: ");
    Serial.println(WiFi.localIP());

    Serial.printf("Connect to ES8388 codec... ");
    while (not es.begin(IIC_DATA, IIC_CLK))
    {
        Serial.printf("Failed!\n");
        delay(1000);
    }
    Serial.printf("OK\n");

    es.volume(ES8388::ES_MAIN, volume);
    es.volume(ES8388::ES_OUT1, volume);
    es.volume(ES8388::ES_OUT2, volume);
    es.mute(ES8388::ES_OUT1, false);
    es.mute(ES8388::ES_OUT2, false);
    es.mute(ES8388::ES_MAIN, false);

    // Enable amplifier
    pinMode(GPIO_PA_EN, OUTPUT);
    digitalWrite(GPIO_PA_EN, HIGH);

    audio.setPinout(I2S_BCLK, I2S_LRCK, I2S_SDOUT);
	audio.i2s_mclk_pin_select(I2S_MCLK);
    audio.setVolume(21); // 0...21

    audio.connecttohost("http://mp3channels.webradio.antenne.de:80/oldies-but-goldies");
//  audio.connecttoFS(SD, "320k_test.mp3");
//  audio.connecttospeech("Wenn die Hunde schlafen, kann der Wolf gut Schafe stehlen.", "de");
}
//----------------------------------------------------------------------------------------------------------------------
void loop()
{
    audio.loop();
}
//----------------------------------------------------------------------------------------------------------------------

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
void audio_showstreaminfo(const char *info){
    Serial.print("streaminfo  ");Serial.println(info);
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


