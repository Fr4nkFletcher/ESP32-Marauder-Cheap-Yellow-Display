//**********************************************************************************************************
//*    audioI2S-- I2S audiodecoder for M5StickC Plus and SPK HAT                                           *
//**********************************************************************************************************
//
// first release on May.12/2021
//
//
// THE SOFTWARE IS PROVIDED "AS IS" FOR PRIVATE USE ONLY, IT IS NOT FOR COMMERCIAL USE IN WHOLE OR PART OR CONCEPT.
// FOR PERSONAL USE IT IS SUPPLIED WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHOR
// OR COPYRIGHT HOLDER BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE
//

#include <M5StickCPlus.h>
#include "Audio.h"

Audio audio = Audio(true);

String ssid =     "xxxxxxxx";
String password = "xxxxxxxx";


void setup() {
  M5.begin(false);  //Lcd disabled to reduce noise
  M5.Axp.ScreenBreath(1); //Lower Lcd backlight
  pinMode(36, INPUT);
  gpio_pulldown_dis(GPIO_NUM_25);
  gpio_pullup_dis(GPIO_NUM_25);
  M5.Beep.tone(44100);  //Built-in buzzer tone
  M5.Beep.end();        //disabled
  
  audio.setVolume(15); // 0...21
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid.c_str(), password.c_str());
  while (!WiFi.isConnected()) {
    delay(10);
  }
  ESP_LOGI(TAG, "Connected");
  ESP_LOGI(TAG, "Starting MP3...\n");

  audio.connecttohost("http://air.ofr.fm:8008/jazz/mp3/128");
//  audio.connecttospeech("Миска вареників з картоплею та шкварками, змащених салом!", "uk-UA");
}

void loop() {
    audio.loop();
    if(Serial.available()){ // put streamURL in serial monitor
        audio.stopSong();
        String r=Serial.readString(); 
        r.trim();
        if(r.length()>5) audio.connecttohost(r.c_str());
        log_i("free heap=%i", ESP.getFreeHeap());
    }
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