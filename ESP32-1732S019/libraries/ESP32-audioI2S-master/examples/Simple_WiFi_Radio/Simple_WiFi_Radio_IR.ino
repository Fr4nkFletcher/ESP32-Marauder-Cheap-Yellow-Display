// this is the same example as Webradio_I2S.ino only with an IR remote control

#include <Arduino.h>
#include <Preferences.h>
#include <SPI.h>
#include <WiFi.h>
#include "tft.h"   //see my repository at github "https://github.com/schreibfaul1/ESP32-TFT-Library-ILI9486"
#include "Audio.h" //see my repository at github "https://github.com/schreibfaul1/ESP32-audioI2S"
#include "IR.h"    //see my repository at github "ESP32-IR-Remote-Control"


#define TFT_CS        22
#define TFT_DC        21
#define TP_CS         16
#define TP_IRQ        39
#define SPI_MOSI      23
#define SPI_MISO      19
#define SPI_SCK       18
#define I2S_DOUT      25
#define I2S_BCLK      27
#define I2S_LRC       26
#define IR_PIN        34

Preferences pref;
TFT tft;        // @suppress("Abstract class cannot be instantiated")
TP tp(TP_CS, TP_IRQ);
Audio audio;
IR ir(IR_PIN);  // do not change the objectname, it must be "ir"

String ssid =     "*********";
String password = "*********";

String stations[] ={
        "0n-80s.radionetz.de:8000/0n-70s.mp3",
        "mediaserv30.live-streams.nl:8000/stream",
        "www.surfmusic.de/m3u/100-5-das-hitradio,4529.m3u",
        "stream.1a-webradio.de/deutsch/mp3-128/vtuner-1a",
        "mp3.ffh.de/radioffh/hqlivestream.aac", //  128k aac
        "www.antenne.de/webradio/antenne.m3u",
        "listen.rusongs.ru/ru-mp3-128",
        "edge.audio.3qsdn.com/senderkw-mp3",
        "macslons-irish-pub-radio.com/media.asx",
};

//some global variables

uint8_t max_volume   = 21;
uint8_t max_stations = 0;   //will be set later
uint8_t cur_station  = 0;   //current station(nr), will be set later
uint8_t cur_volume   = 0;   //will be set from stored preferences
int8_t  cur_btn      =-1;   //current button (, -1 means idle)

enum action{VOLUME_UP=0, VOLUME_DOWN=1, STATION_UP=2, STATION_DOWN=3};
enum staus {RELEASED=0, PRESSED=1};

struct _btns{
    uint16_t x; //PosX
    uint16_t y; //PosY
    uint16_t w; //Width
    uint16_t h; //Hight
    uint8_t  a; //Action
    uint8_t  s; //Status
};
typedef _btns btns;

btns btn[4];

//**************************************************************************************************
//                                              G U I                                              *
//**************************************************************************************************
void draw_button(btns b){
    uint16_t color=TFT_BLACK;
    uint8_t r=4, o=r*3;
    if(b.s==RELEASED) color=TFT_GOLD;
    if(b.s==PRESSED)  color=TFT_DEEPSKYBLUE;
    tft.drawRoundRect(b.x, b.y, b.w, b.h, r, color);
    switch(b.a){
        case VOLUME_UP:
            tft.fillTriangle(b.x+b.w/2, b.y+o, b.x+o, b.y+b.h-o, b.x+b.w-o, b.y+b.h-o, color); break;
        case VOLUME_DOWN:
            tft.fillTriangle(b.x+o, b.y+o, b.x+b.w/2, b.y+b.h-o, b.x+b.w-o, b.y+o, color); break;
        case STATION_UP:
            tft.fillTriangle(b.x+o, b.y+o, b.x+o, b.y+b.h-o, b.x+b.w-o, b.y+b.h/2, color); break;
        case STATION_DOWN:
            tft.fillTriangle(b.x+b.w-o, b.y+o, b.x+o, b.y+b.h/2, b.x+b.w-o, b.y+b.h-o, color); break;
    }
}
void write_stationNr(uint8_t nr){
    tft.fillRect(80, 250, 80, 60, TFT_BLACK);
    String snr = String(nr);
    if(snr.length()<2) snr = "0"+snr;
    tft.setCursor(98, 255);
    tft.setFont(Times_New_Roman66x53);
    tft.setTextColor(TFT_YELLOW);
    tft.print(snr);
}
void write_volume(uint8_t vol){
    tft.fillRect(320, 250, 80, 60, TFT_BLACK);
    String svol = String(vol);
    if(svol.length()<2) svol = "0"+svol;
    tft.setCursor(338, 255);
    tft.setFont(Times_New_Roman66x53);
    tft.setTextColor(TFT_YELLOW);
    tft.print(svol);
}
void write_stationName(String sName){
    tft.fillRect(0, 0, 480, 100, TFT_BLACK);
    tft.setFont(Times_New_Roman43x35);
    tft.setTextColor(TFT_CORNSILK);
    tft.setCursor(20, 20);
    tft.print(sName);
}
void write_streamTitle(String sTitle){
    tft.fillRect(0, 100, 480, 150, TFT_BLACK);
    tft.setFont(Times_New_Roman43x35);
    tft.setTextColor(TFT_LIGHTBLUE);
    tft.setCursor(20, 100);
    tft.print(sTitle);
}
void volume_up(){
    if(cur_volume < max_volume){
        cur_volume++;
        write_volume(cur_volume);
        audio.setVolume(cur_volume);
        pref.putShort("volume", cur_volume);} // store the current volume in nvs
}
void volume_down(){
    if(cur_volume>0){
        cur_volume-- ;
        write_volume(cur_volume);
        audio.setVolume(cur_volume);
        pref.putShort("volume", cur_volume);} // store the current volume in nvs
}
void station_up(){
    if(cur_station < max_stations-1){
        cur_station++;
        write_stationNr(cur_station);
        tft.fillRect(0, 0, 480, 250, TFT_BLACK);
        audio.connecttohost(stations[cur_station].c_str());
        pref.putShort("station", cur_station);} // store the current station in nvs
}
void station_down(){
    if(cur_station > 0){
        cur_station--;
        write_stationNr(cur_station);
        tft.fillRect(0, 0, 480, 250, TFT_BLACK);
        audio.connecttohost(stations[cur_station].c_str());
        pref.putShort("station", cur_station);} // store the current station in nvs
}


//**************************************************************************************************
//                                           S E T U P                                             *
//**************************************************************************************************
void setup() {
    btn[0].x= 20; btn[0].y=250; btn[0].w=60; btn[0].h=60; btn[0].a=STATION_DOWN; btn[0].s=RELEASED;
    btn[1].x=160; btn[1].y=250; btn[1].w=60; btn[1].h=60; btn[1].a=STATION_UP;   btn[1].s=RELEASED;
    btn[2].x=260; btn[2].y=250; btn[2].w=60; btn[2].h=60; btn[2].a=VOLUME_UP;    btn[2].s=RELEASED;
    btn[3].x=400; btn[3].y=250; btn[3].w=60; btn[3].h=60; btn[3].a=VOLUME_DOWN;  btn[3].s=RELEASED;
    max_stations= sizeof(stations)/sizeof(stations[0]); log_i("max stations %i", max_stations);
    Serial.begin(115200);
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
    pref.begin("WebRadio", false);  // instance of preferences for defaults (station, volume ...)
    if(pref.getShort("volume", 1000) == 1000){ // if that: pref was never been initialized
        pref.putShort("volume", 10);
        pref.putShort("station", 0);
    }
    else{ // get the stored values
        cur_station = pref.getShort("station");
        cur_volume = pref.getShort("volume");
    }
    WiFi.disconnect();
    WiFi.begin(ssid.c_str(), password.c_str());
    while (WiFi.status() != WL_CONNECTED) {delay(1500); Serial.print(".");}
    log_i("Connected to %s", WiFi.SSID().c_str());
    tft.begin(TFT_CS, TFT_DC, SPI_MOSI, SPI_MISO, SPI_SCK);
    tft.setFrequency(20000000);
    tft.setRotation(3);
    tp.setRotation(3);
    tft.setFont(Times_New_Roman43x35);
    tft.fillScreen(TFT_BLACK);
    ir.begin();  // Init InfraredDecoder
    audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
    audio.setVolume(cur_volume); // 0...21
    audio.connecttohost(stations[cur_station].c_str());
    for(uint8_t i=0; i<(sizeof(btn)/sizeof(*btn)); i++) draw_button(btn[i]);
    write_volume(cur_volume);
    write_stationNr(cur_station);
}
//**************************************************************************************************
//                                            L O O P                                              *
//**************************************************************************************************
void loop()
{
    audio.loop();
    tp.loop();
    ir.loop();
}
//**************************************************************************************************
//                                           E V E N T S                                           *
//**************************************************************************************************
void audio_info(const char *info){
    Serial.print("audio_info: "); Serial.println(info);
}
void audio_showstation(const char *info){
    write_stationName(String(info));
}
void audio_showstreamtitle(const char *info){
    String sinfo=String(info);
    sinfo.replace("|", "\n");
    write_streamTitle(sinfo);
}

void tp_pressed(uint16_t x, uint16_t y){
    for(uint8_t i=0; i<(sizeof(btn)/sizeof(*btn)); i++){
        if(x>btn[i].x && (x<btn[i].x+btn[i].w)){
            if(y>btn[i].y && (y<btn[i].y+btn[i].h)){
                cur_btn=i;
                btn[cur_btn].s=PRESSED;
                draw_button(btn[cur_btn]);
            }
        }
    }
}
void tp_released(){
    if(cur_btn !=-1){
        btn[cur_btn].s=RELEASED;
        draw_button(btn[cur_btn]);
        switch(btn[cur_btn].a){
            case VOLUME_UP:    volume_up();    break;
            case VOLUME_DOWN:  volume_down();  break;
            case STATION_UP:   station_up();   break;
            case STATION_DOWN: station_down(); break;
        }
    }
    cur_btn=-1;
}
// Events from IR Library
void ir_res(uint32_t res){
    if(res < max_stations){
        cur_station = res;
        write_stationNr(cur_station);
        tft.fillRect(0, 0, 480, 250, TFT_BLACK);
        audio.connecttohost(stations[cur_station].c_str());
        pref.putShort("station", cur_station);} // store the current station in nvs
    else{
        tft.fillRect(0, 0, 480, 250, TFT_BLACK);
        audio.connecttohost(stations[cur_station].c_str());
    }
}
void ir_number(const char* num){
    tft.fillRect(0, 0, 480, 250, TFT_BLACK);
    tft.setTextSize(7);
    tft.setTextColor(TFT_CORNSILK);
    tft.setCursor(50, 70);
    tft.print(num);
}
void ir_key(const char* key){
    switch(key[0]){
        case 'k':                   break; // OK
        case 'r':   volume_up();    break; // right
        case 'l':   volume_down();  break; // left
        case 'u':   station_up();   break; // up
        case 'd':   station_down(); break; // down
        case '#':                   break; // #
        case '*':                   break; // *
        default:    break;
    }
}
