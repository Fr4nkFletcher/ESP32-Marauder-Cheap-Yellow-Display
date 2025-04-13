/*
 * Audio.cpp
 *
 *  Created on: Oct 26.2018
 *
 *  Version 2.0.6k
 *  Updated on: Oct 22.2022
 *      Author: Wolle (schreibfaul1)
 *
 */
#include "Audio.h"
#include "mp3_decoder/mp3_decoder.h"
#include "aac_decoder/aac_decoder.h"
#include "flac_decoder/flac_decoder.h"

#ifdef SDFATFS_USED
fs::SDFATFS SD_SDFAT;
#endif

//---------------------------------------------------------------------------------------------------------------------
AudioBuffer::AudioBuffer(size_t maxBlockSize) {
    // if maxBlockSize isn't set use defaultspace (1600 bytes) is enough for aac and mp3 player
    if(maxBlockSize) m_resBuffSizeRAM  = maxBlockSize;
    if(maxBlockSize) m_maxBlockSize = maxBlockSize;
}

AudioBuffer::~AudioBuffer() {
    if(m_buffer)
        free(m_buffer);
    m_buffer = NULL;
}

void AudioBuffer::setBufsize(int ram, int psram) {
    if (ram > -1) // -1 == default / no change
        m_buffSizeRAM = ram;
    if (psram > -1)
        m_buffSizePSRAM = psram;
}

size_t AudioBuffer::init() {
    if(m_buffer) free(m_buffer);
    m_buffer = NULL;
    if(psramInit() && m_buffSizePSRAM > 0) {
        // PSRAM found, AudioBuffer will be allocated in PSRAM
        m_f_psram = true;
        m_buffSize = m_buffSizePSRAM;
        m_buffer = (uint8_t*) ps_calloc(m_buffSize, sizeof(uint8_t));
        m_buffSize = m_buffSizePSRAM - m_resBuffSizePSRAM;
    }
    if(m_buffer == NULL) {
        // PSRAM not found, not configured or not enough available
        m_f_psram = false;
        m_buffSize = m_buffSizeRAM;
        m_buffer = (uint8_t*) calloc(m_buffSize, sizeof(uint8_t));
        m_buffSize = m_buffSizeRAM - m_resBuffSizeRAM;
    }
    if(!m_buffer)
        return 0;
    m_f_init = true;
    resetBuffer();
    return m_buffSize;
}

void AudioBuffer::changeMaxBlockSize(uint16_t mbs){
    m_maxBlockSize = mbs;
    return;
}

uint16_t AudioBuffer::getMaxBlockSize(){
    return m_maxBlockSize;
}

size_t AudioBuffer::freeSpace() {
    if(m_readPtr >= m_writePtr) {
        m_freeSpace = (m_readPtr - m_writePtr);
    } else {
        m_freeSpace = (m_endPtr - m_writePtr) + (m_readPtr - m_buffer);
    }
    if(m_f_start)
        m_freeSpace = m_buffSize;
    return m_freeSpace - 1;
}

size_t AudioBuffer::writeSpace() {
    if(m_readPtr >= m_writePtr) {
        m_writeSpace = (m_readPtr - m_writePtr - 1); // readPtr must not be overtaken
    } else {
        if(getReadPos() == 0)
            m_writeSpace = (m_endPtr - m_writePtr - 1);
        else
            m_writeSpace = (m_endPtr - m_writePtr);
    }
    if(m_f_start)
        m_writeSpace = m_buffSize - 1;
    return m_writeSpace;
}

size_t AudioBuffer::bufferFilled() {
    if(m_writePtr >= m_readPtr) {
        m_dataLength = (m_writePtr - m_readPtr);
    } else {
        m_dataLength = (m_endPtr - m_readPtr) + (m_writePtr - m_buffer);
    }
    return m_dataLength;
}

void AudioBuffer::bytesWritten(size_t bw) {
    m_writePtr += bw;
    if(m_writePtr == m_endPtr) {
        m_writePtr = m_buffer;
    }
    if(bw && m_f_start)
        m_f_start = false;
}

void AudioBuffer::bytesWasRead(size_t br) {
    m_readPtr += br;
    if(m_readPtr >= m_endPtr) {
        size_t tmp = m_readPtr - m_endPtr;
        m_readPtr = m_buffer + tmp;
    }
}

uint8_t* AudioBuffer::getWritePtr() {
    return m_writePtr;
}

uint8_t* AudioBuffer::getReadPtr() {
    size_t len = m_endPtr - m_readPtr;
    if(len < m_maxBlockSize) { // be sure the last frame is completed
        memcpy(m_endPtr, m_buffer, m_maxBlockSize - len);  // cpy from m_buffer to m_endPtr with len
    }
    return m_readPtr;
}

void AudioBuffer::resetBuffer() {
    m_writePtr = m_buffer;
    m_readPtr = m_buffer;
    m_endPtr = m_buffer + m_buffSize;
    m_f_start = true;
    // memset(m_buffer, 0, m_buffSize); //Clear Inputbuffer
}

uint32_t AudioBuffer::getWritePos() {
    return m_writePtr - m_buffer;
}

uint32_t AudioBuffer::getReadPos() {
    return m_readPtr - m_buffer;
}
//---------------------------------------------------------------------------------------------------------------------
Audio::Audio(bool internalDAC /* = false */, uint8_t channelEnabled /* = I2S_DAC_CHANNEL_BOTH_EN */, uint8_t i2sPort) {

    //    build-in-DAC works only with ESP32 (ESP32-S3 has no build-in-DAC)
    //    build-in-DAC last working Arduino Version: 2.0.0-RC2
    //    possible values for channelEnabled are:
    //    I2S_DAC_CHANNEL_DISABLE  = 0,     Disable I2S built-in DAC signals
    //    I2S_DAC_CHANNEL_RIGHT_EN = 1,     Enable I2S built-in DAC right channel, maps to DAC channel 1 on GPIO25
    //    I2S_DAC_CHANNEL_LEFT_EN  = 2,     Enable I2S built-in DAC left  channel, maps to DAC channel 2 on GPIO26
    //    I2S_DAC_CHANNEL_BOTH_EN  = 0x3,   Enable both of the I2S built-in DAC channels.
    //    I2S_DAC_CHANNEL_MAX      = 0x4,   I2S built-in DAC mode max index
#ifdef AUDIO_LOG
    m_f_Log = true;
#endif

    clientsecure.setInsecure();  // if that can't be resolved update to ESP32 Arduino version 1.0.5-rc05 or higher
    m_f_channelEnabled = channelEnabled;
    m_f_internalDAC = internalDAC;
    //i2s configuration
    m_i2s_num = i2sPort; // i2s port number
    m_i2s_config.sample_rate          = 16000;
    m_i2s_config.bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT;
    m_i2s_config.channel_format       = I2S_CHANNEL_FMT_RIGHT_LEFT;
    m_i2s_config.intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1; // interrupt priority
    m_i2s_config.dma_buf_count        = 16;
    m_i2s_config.dma_buf_len          = 512;
    m_i2s_config.use_apll             = APLL_DISABLE; // must be disabled in V2.0.1-RC1
    m_i2s_config.tx_desc_auto_clear   = true;   // new in V1.0.1
    m_i2s_config.fixed_mclk           = I2S_PIN_NO_CHANGE;


    if (internalDAC)  {

        #ifdef CONFIG_IDF_TARGET_ESP32  // ESP32S3 has no DAC

            log_i("internal DAC");
            m_i2s_config.mode             = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN );

            #if ESP_ARDUINO_VERSION_MAJOR >= 2
                #if ESP_ARDUINO_VERSION_PATCH  == 0
                    m_i2s_config.communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_STAND_I2S); // vers == 2.0.0
                #else
                    // V2.0.1 ... V2.0.4 will not work
                    m_i2s_config.communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_STAND_MSB); // vers >= 2.0.5
                #endif
            #else
                m_i2s_config.communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S_MSB);
            #endif

            i2s_driver_install((i2s_port_t)m_i2s_num, &m_i2s_config, 0, NULL);
            i2s_set_dac_mode((i2s_dac_mode_t)m_f_channelEnabled);
            if(m_f_channelEnabled != I2S_DAC_CHANNEL_BOTH_EN) {
                m_f_forceMono = true;
            }

        #endif

    }
    else {
        m_i2s_config.mode             = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);

        #if ESP_ARDUINO_VERSION_MAJOR >= 2
            m_i2s_config.communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_STAND_I2S); // Arduino vers. > 2.0.0
        #else
            m_i2s_config.communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB);
        #endif

        i2s_driver_install((i2s_port_t)m_i2s_num, &m_i2s_config, 0, NULL);
        m_f_forceMono = false;
    }

    i2s_zero_dma_buffer((i2s_port_t) m_i2s_num);

    for(int i = 0; i <3; i++) {
        m_filter[i].a0  = 1;
        m_filter[i].a1  = 0;
        m_filter[i].a2  = 0;
        m_filter[i].b1  = 0;
        m_filter[i].b2  = 0;
    }
}
//---------------------------------------------------------------------------------------------------------------------
void Audio::setBufsize(int rambuf_sz, int psrambuf_sz) {
    if(InBuff.isInitialized()) {
        log_e("Audio::setBufsize must not be called after audio is initialized");
        return;
    }
    InBuff.setBufsize(rambuf_sz, psrambuf_sz);
};

void Audio::initInBuff() {
    if(!InBuff.isInitialized()) {
        size_t size = InBuff.init();
        if (size > 0) {
            AUDIO_INFO("PSRAM %sfound, inputBufferSize: %u bytes", InBuff.havePSRAM()?"":"not ", size - 1);
        }
    }
    changeMaxBlockSize(1600); // default size mp3 or aac
}

//---------------------------------------------------------------------------------------------------------------------
esp_err_t Audio::I2Sstart(uint8_t i2s_num) {
    // It is not necessary to call this function after i2s_driver_install() (it is started automatically),
    // however it is necessary to call it after i2s_stop()
    return i2s_start((i2s_port_t) i2s_num);
}

esp_err_t Audio::I2Sstop(uint8_t i2s_num) {
    return i2s_stop((i2s_port_t) i2s_num);
}
//---------------------------------------------------------------------------------------------------------------------
esp_err_t Audio::i2s_mclk_pin_select(const uint8_t pin) {
    // IDF >= 4.4 use setPinout(BCLK, LRC, DOUT, DIN, MCK) only, i2s_mclk_pin_select() is no longer needed

    if(pin != 0 && pin != 1 && pin != 3) {
        log_e("Only support GPIO0/GPIO1/GPIO3, gpio_num:%d", pin);
        return ESP_ERR_INVALID_ARG;
    }

    #ifdef CONFIG_IDF_TARGET_ESP32
        switch(pin){
            case 0:
                PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0_CLK_OUT1);
                WRITE_PERI_REG(PIN_CTRL, 0xFFF0);
                break;
            case 1:
                PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0TXD_U, FUNC_U0TXD_CLK_OUT3);
                WRITE_PERI_REG(PIN_CTRL, 0xF0F0);
                break;
            case 3:
                PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0RXD_U, FUNC_U0RXD_CLK_OUT2);
                WRITE_PERI_REG(PIN_CTRL, 0xFF00);
                break;
            default:
                break;
        }
    #endif

    return ESP_OK;
}
//---------------------------------------------------------------------------------------------------------------------
Audio::~Audio() {
    //I2Sstop(m_i2s_num);
    //InBuff.~AudioBuffer(); #215 the AudioBuffer is automatically destroyed by the destructor
    setDefaults();
    if(m_playlistBuff) {free(m_playlistBuff); m_playlistBuff = NULL;}
    i2s_driver_uninstall((i2s_port_t)m_i2s_num); // #215 free I2S buffer
}
//---------------------------------------------------------------------------------------------------------------------
void Audio::setDefaults() {
    stopSong();
    initInBuff(); // initialize InputBuffer if not already done
    InBuff.resetBuffer();
    MP3Decoder_FreeBuffers();
    FLACDecoder_FreeBuffers();
    AACDecoder_FreeBuffers();
    if(m_playlistBuff)   {free(m_playlistBuff);     m_playlistBuff = NULL;} // free if stream is not m3u8
    vector_clear_and_shrink(m_playlistURL);
    vector_clear_and_shrink(m_playlistContent);
    m_hashQueue.clear(); m_hashQueue.shrink_to_fit(); // uint32_t vector
    client.stop();
    client.flush(); // release memory
    clientsecure.stop();
    clientsecure.flush();
    _client = static_cast<WiFiClient*>(&client); /* default to *something* so that no NULL deref can happen */
    playI2Sremains();
    ts_parsePacket(0, 0, 0); // reset ts routine

    AUDIO_INFO("buffers freed, free Heap: %u bytes", ESP.getFreeHeap());

    m_f_chunked = false;                                    // Assume not chunked
    m_f_firstmetabyte = false;
    m_f_playing = false;
    m_f_ssl = false;
    m_f_metadata = false;
    m_f_tts = false;
    m_f_firstCall = true;                                   // InitSequence for processWebstream and processLokalFile
    m_f_running = false;
    m_f_loop = false;                                       // Set if audio file should loop
    m_f_unsync = false;                                     // set within ID3 tag but not used
    m_f_exthdr = false;                                     // ID3 extended header
    m_f_rtsp = false;                                       // RTSP (m3u8)stream
    m_f_m3u8data = false;                                   // set again in processM3U8entries() if necessary
    m_f_continue = false;
    m_f_ts = false;

    m_streamType = ST_NONE;
    m_codec = CODEC_NONE;
    m_playlistFormat = FORMAT_NONE;
    m_datamode = AUDIO_NONE;
    m_audioCurrentTime = 0;                                 // Reset playtimer
    m_audioFileDuration = 0;
    m_audioDataStart = 0;
    m_audioDataSize = 0;
    m_avr_bitrate = 0;                                      // the same as m_bitrate if CBR, median if VBR
    m_bitRate = 0;                                          // Bitrate still unknown
    m_bytesNotDecoded = 0;                                  // counts all not decodable bytes
    m_chunkcount = 0;                                       // for chunked streams
    m_contentlength = 0;                                    // If Content-Length is known, count it
    m_curSample = 0;
    m_metaint = 0;                                          // No metaint yet
    m_LFcount = 0;                                          // For end of header detection
    m_controlCounter = 0;                                   // Status within readID3data() and readWaveHeader()
    m_channels = 2;                                         // assume stereo #209
    m_streamTitleHash = 0;
    m_file_size = 0;
    m_ID3Size = 0;
}

//---------------------------------------------------------------------------------------------------------------------
void Audio::setConnectionTimeout(uint16_t timeout_ms, uint16_t timeout_ms_ssl){
    if(timeout_ms)     m_timeout_ms     = timeout_ms;
    if(timeout_ms_ssl) m_timeout_ms_ssl = timeout_ms_ssl;
}

//---------------------------------------------------------------------------------------------------------------------
bool Audio::connecttohost(const char* host, const char* user, const char* pwd) {
    // user and pwd for authentification only, can be empty

    if(host == NULL) {
        AUDIO_INFO("Hostaddress is empty");
        return false;
    }

    uint16_t lenHost = strlen(host);

    if(lenHost >= 512 - 10) {
        AUDIO_INFO("Hostaddress is too long");
        return false;
    }

    int idx = indexOf(host, "http");
    char* l_host = (char*)malloc(lenHost + 10);
    if(idx < 0){strcpy(l_host, "http://"); strcat(l_host, host); } // amend "http;//" if not found
    else       {strcpy(l_host, (host + idx));}                     // trim left if necessary

    char* h_host = NULL; // pointer of l_host without http:// or https://
    if(startsWith(l_host, "https")) h_host = strdup(l_host + 8);
    else                            h_host = strdup(l_host + 7);

    // initializationsequence
    int16_t pos_slash;                                        // position of "/" in hostname
    int16_t pos_colon;                                        // position of ":" in hostname
    int16_t pos_ampersand;                                    // position of "&" in hostname
    uint16_t port = 80;                                       // port number

    // In the URL there may be an extension, like noisefm.ru:8000/play.m3u&t=.m3u
    pos_slash     = indexOf(h_host, "/", 0);
    pos_colon     = indexOf(h_host, ":", 0);
        if(isalpha(h_host[pos_colon + 1])) pos_colon = -1; // no portnumber follows
    pos_ampersand = indexOf(h_host, "&", 0);

    char *hostwoext = NULL;                                  // "skonto.ls.lv:8002" in "skonto.ls.lv:8002/mp3"
    char *extension = NULL;                                  // "/mp3" in "skonto.ls.lv:8002/mp3"

    if(pos_slash > 1) {
        hostwoext = (char*)malloc(pos_slash + 1);
        memcpy(hostwoext, h_host, pos_slash);
        hostwoext[pos_slash] = '\0';
        uint16_t extLen =  urlencode_expected_len(h_host + pos_slash);
        extension = (char *)malloc(extLen + 20);
        memcpy(extension, h_host + pos_slash, extLen);
        urlencode(extension, extLen, true);
    }
    else{  // url has no extension
        hostwoext = strdup(h_host);
        extension = strdup("/");
    }

    if((pos_colon >= 0) && ((pos_ampersand == -1) || (pos_ampersand > pos_colon))){
        port = atoi(h_host + pos_colon + 1);// Get portnumber as integer
        hostwoext[pos_colon] = '\0';// Host without portnumber
    }

    AUDIO_INFO("Connect to new host: \"%s\"", l_host);
    setDefaults(); // no need to stop clients if connection is established (default is true)

    if(startsWith(l_host, "https")) m_f_ssl = true;
    else                            m_f_ssl = false;

    // authentification
    uint8_t auth = strlen(user) + strlen(pwd);
    char toEncode[auth + 4];
    toEncode[0] = '\0';
    strcat(toEncode, user);
    strcat(toEncode, ":");
    strcat(toEncode, pwd);
    char authorization[base64_encode_expected_len(strlen(toEncode)) + 1];
    authorization[0] = '\0';
    b64encode((const char*)toEncode, strlen(toEncode), authorization);

    //  AUDIO_INFO("Connect to \"%s\" on port %d, extension \"%s\"", hostwoext, port, extension);

    char rqh[strlen(h_host) + strlen(authorization) + 200]; // http request header
    rqh[0] = '\0';

    strcat(rqh, "GET ");
    strcat(rqh, extension);
    strcat(rqh, " HTTP/1.1\r\n");
    strcat(rqh, "Host: ");
    strcat(rqh, hostwoext);
    strcat(rqh, "\r\n");
    strcat(rqh, "Icy-MetaData:1\r\n");
    strcat(rqh, "Authorization: Basic ");
    strcat(rqh, authorization);
    strcat(rqh, "\r\n");
    strcat(rqh, "Accept-Encoding: identity;q=1,*;q=0\r\n");
//    strcat(rqh, "User-Agent: Mozilla/5.0\r\n"); #363
    strcat(rqh, "Connection: keep-alive\r\n\r\n");

    if(ESP_ARDUINO_VERSION_MAJOR == 2 && ESP_ARDUINO_VERSION_MINOR == 0 && ESP_ARDUINO_VERSION_PATCH >= 3){
        m_timeout_ms_ssl = UINT16_MAX;  // bug in v2.0.3 if hostwoext is a IPaddr not a name
        m_timeout_ms = UINT16_MAX;  // [WiFiClient.cpp:253] connect(): select returned due to timeout 250 ms for fd 48
    }
    bool res = true; // no need to reconnect if connection exists

    if(m_f_ssl){ _client = static_cast<WiFiClient*>(&clientsecure); if(port == 80) port = 443;}
    else       { _client = static_cast<WiFiClient*>(&client);}

    uint32_t t = millis();
    if(m_f_Log) AUDIO_INFO("connect to %s on port %d path %s", hostwoext, port, extension);
    res = _client->connect(hostwoext, port, m_f_ssl ? m_timeout_ms_ssl : m_timeout_ms);
    if(res){
        uint32_t dt = millis() - t;
        strcpy(m_lastHost, l_host);
        AUDIO_INFO("%s has been established in %u ms, free Heap: %u bytes",
                    m_f_ssl?"SSL":"Connection", dt, ESP.getFreeHeap());
        m_f_running = true;
    }

    m_expectedCodec = CODEC_NONE;
    m_expectedPlsFmt = FORMAT_NONE;

    if(res){
        _client->print(rqh);
        if(endsWith(extension, ".mp3"))   m_expectedCodec = CODEC_MP3;
        if(endsWith(extension, ".aac"))   m_expectedCodec = CODEC_AAC;
        if(endsWith(extension, ".wav"))   m_expectedCodec = CODEC_WAV;
        if(endsWith(extension, ".m4a"))   m_expectedCodec = CODEC_M4A;
        if(endsWith(extension, ".flac"))  m_expectedCodec = CODEC_FLAC;
        if(endsWith(extension, ".asx"))  m_expectedPlsFmt = FORMAT_ASX;
        if(endsWith(extension, ".m3u"))  m_expectedPlsFmt = FORMAT_M3U;
        if(endsWith(extension, ".m3u8")) m_expectedPlsFmt = FORMAT_M3U8;
        if(endsWith(extension, ".pls"))  m_expectedPlsFmt = FORMAT_PLS;

        setDatamode(HTTP_RESPONSE_HEADER);   // Handle header
        m_streamType = ST_WEBSTREAM;
    }
    else{
        AUDIO_INFO("Request %s failed!", l_host);
        if(audio_showstation) audio_showstation("");
        if(audio_showstreamtitle) audio_showstreamtitle("");
        if(audio_icydescription) audio_icydescription("");
        if(audio_icyurl) audio_icyurl("");
        m_lastHost[0] = 0;
    }
    if(hostwoext) {free(hostwoext); hostwoext = NULL;}
    if(extension) {free(extension); extension = NULL;}
    if(l_host   ) {free(l_host);    l_host    = NULL;}
    if(h_host   ) {free(h_host);    h_host    = NULL;}
    return res;
}
//---------------------------------------------------------------------------------------------------------------------
bool Audio::httpPrint(const char* host) {
    // user and pwd for authentification only, can be empty

    if(host == NULL) {
        AUDIO_INFO("Hostaddress is empty");
        return false;
    }

    char* h_host = NULL; // pointer of l_host without http:// or https://
    if(m_f_ssl) h_host = strdup(host + 8);
    else        h_host = strdup(host + 7);

    int16_t pos_slash;                                        // position of "/" in hostname
    int16_t pos_colon;                                        // position of ":" in hostname
    int16_t pos_ampersand;                                    // position of "&" in hostname
    uint16_t port = 80;                                       // port number

    // In the URL there may be an extension, like noisefm.ru:8000/play.m3u&t=.m3u
    pos_slash     = indexOf(h_host, "/", 0);
    pos_colon     = indexOf(h_host, ":", 0);
        if(isalpha(h_host[pos_colon + 1])) pos_colon = -1; // no portnumber follows
    pos_ampersand = indexOf(h_host, "&", 0);

    char *hostwoext = NULL;                                  // "skonto.ls.lv:8002" in "skonto.ls.lv:8002/mp3"
    char *extension = NULL;                                  // "/mp3" in "skonto.ls.lv:8002/mp3"

    if(pos_slash > 1) {
        hostwoext = (char*)malloc(pos_slash + 1);
        memcpy(hostwoext, h_host, pos_slash);
        hostwoext[pos_slash] = '\0';
        uint16_t extLen =  urlencode_expected_len(h_host + pos_slash);
        extension = (char *)malloc(extLen + 20);
        memcpy(extension, h_host + pos_slash, extLen);
        urlencode(extension, extLen, true);
    }
    else{  // url has no extension
        hostwoext = strdup(h_host);
        extension = strdup("/");
    }

    if((pos_colon >= 0) && ((pos_ampersand == -1) || (pos_ampersand > pos_colon))){
        port = atoi(h_host + pos_colon + 1);// Get portnumber as integer
        hostwoext[pos_colon] = '\0';// Host without portnumber
    }

    AUDIO_INFO("new request: \"%s\"", host);

    char rqh[strlen(h_host) + 200]; // http request header
    rqh[0] = '\0';

    strcat(rqh, "GET ");
    strcat(rqh, extension);
    strcat(rqh, " HTTP/1.1\r\n");
    strcat(rqh, "Host: ");
    strcat(rqh, hostwoext);
    strcat(rqh, "\r\n");
    strcat(rqh, "Accept-Encoding: identity;q=1,*;q=0\r\n");
//    strcat(rqh, "User-Agent: Mozilla/5.0\r\n"); #363
    strcat(rqh, "Connection: keep-alive\r\n\r\n");

    if(m_f_ssl){ _client = static_cast<WiFiClient*>(&clientsecure); if(port == 80) port = 443;}
    else       { _client = static_cast<WiFiClient*>(&client);}

    if(!_client->connected()){
        AUDIO_INFO("The host has disconnected, reconnecting");
        if(!_client->connect(hostwoext, port)){
            log_e("connection lost");
            stopSong();
            return false;
        }
    }
    _client->print(rqh);

    if(endsWith(extension, ".mp3"))   m_expectedCodec = CODEC_MP3;
    if(endsWith(extension, ".aac"))   m_expectedCodec = CODEC_AAC;
    if(endsWith(extension, ".wav"))   m_expectedCodec = CODEC_WAV;
    if(endsWith(extension, ".m4a"))   m_expectedCodec = CODEC_M4A;
    if(endsWith(extension, ".flac"))  m_expectedCodec = CODEC_FLAC;
    if(endsWith(extension, ".asx"))  m_expectedPlsFmt = FORMAT_ASX;
    if(endsWith(extension, ".m3u"))  m_expectedPlsFmt = FORMAT_M3U;
    if(endsWith(extension, ".m3u8")) m_expectedPlsFmt = FORMAT_M3U8;
    if(endsWith(extension, ".pls"))  m_expectedPlsFmt = FORMAT_PLS;

    setDatamode(HTTP_RESPONSE_HEADER);   // Handle header
    m_streamType = ST_WEBSTREAM;
    m_contentlength = 0;
    m_f_chunked = false;

    if(hostwoext) {free(hostwoext); hostwoext = NULL;}
    if(extension) {free(extension); extension = NULL;}
    if(h_host   ) {free(h_host);    h_host    = NULL;}
    return true;
}
//---------------------------------------------------------------------------------------------------------------------
bool Audio::setFileLoop(bool input){
    m_f_loop = input;
    return input;
}
//---------------------------------------------------------------------------------------------------------------------
void Audio::UTF8toASCII(char* str){

#ifdef SDFATFS_USED
    //UTF8->UTF16 (lowbyte)
    const uint8_t ascii[60] = {
    //129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148  // UTF8(C3)
    //                Ä    Å    Æ    Ç         É                                       Ñ                  // CHAR
      000, 000, 000, 0xC4, 143, 0xC6,0xC7, 000,0xC9,000, 000, 000, 000, 000, 000, 000, 0xD1, 000, 000, 000, // ASCII (Latin1)
    //149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168
    //      Ö                             Ü              ß    à                   ä    å    æ         è
      000, 0xD6,000, 000, 000, 000, 000, 0xDC, 000, 000, 0xDF,0xE0, 000, 000, 000,0xE4,0xE5,0xE6, 000,0xE8,
    //169, 170, 171, 172. 173. 174. 175, 176, 177, 179, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188
    //      ê    ë    ì         î    ï         ñ    ò         ô         ö              ù         û    ü
      000, 0xEA, 0xEB,0xEC, 000,0xEE,0xEB, 000,0xF1,0xF2, 000,0xF4, 000,0xF6, 000, 000,0xF9, 000,0xFB,0xFC};
#else
    const uint8_t ascii[60] = {
    //129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148  // UTF8(C3)
    //                Ä    Å    Æ    Ç         É                                       Ñ                  // CHAR
      000, 000, 000, 142, 143, 146, 128, 000, 144, 000, 000, 000, 000, 000, 000, 000, 165, 000, 000, 000, // ASCII
    //149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168
    //      Ö                             Ü              ß    à                   ä    å    æ         è
      000, 153, 000, 000, 000, 000, 000, 154, 000, 000, 225, 133, 000, 000, 000, 132, 134, 145, 000, 138,
    //169, 170, 171, 172. 173. 174. 175, 176, 177, 179, 179, 180, 181, 182, 183, 184, 185, 186, 187, 188
    //      ê    ë    ì         î    ï         ñ    ò         ô         ö              ù         û    ü
      000, 136, 137, 141, 000, 140, 139, 000, 164, 149, 000, 147, 000, 148, 000, 000, 151, 000, 150, 129};
#endif

    uint16_t i = 0, j=0, s = 0;
    bool f_C3_seen = false;

    while(str[i] != 0) {                                     // convert UTF8 to ASCII
        if(str[i] == 195){                                   // C3
            i++;
            f_C3_seen = true;
            continue;
        }
        str[j] = str[i];
        if(str[j] > 128 && str[j] < 189 && f_C3_seen == true) {
            s = ascii[str[j] - 129];
            if(s != 0) str[j] = s;                         // found a related ASCII sign
            f_C3_seen = false;
        }
        i++; j++;
    }
    str[j] = 0;
}
//---------------------------------------------------------------------------------------------------------------------
bool Audio::connecttoSD(const char* path, uint32_t resumeFilePos) {
    return connecttoFS(SD, path, resumeFilePos);
}
//---------------------------------------------------------------------------------------------------------------------
bool Audio::connecttoFS(fs::FS &fs, const char* path, uint32_t resumeFilePos) {

    if(strlen(path)>255) return false;

    m_resumeFilePos = resumeFilePos;
    char audioName[256];
    setDefaults(); // free buffers an set defaults
    memcpy(audioName, path, strlen(path)+1);
    if(audioName[0] != '/'){
        for(int i = 255; i > 0; i--){
            audioName[i] = audioName[i-1];
        }
        audioName[0] = '/';
    }

    AUDIO_INFO("Reading file: \"%s\"", audioName); vTaskDelay(2);

    if(fs.exists(audioName)) {
        audiofile = fs.open(audioName); // #86
    }
    else {
        UTF8toASCII(audioName);
        if(fs.exists(audioName)) {
            audiofile = fs.open(audioName);
        }
    }

    if(!audiofile) {
        if(audio_info) {vTaskDelay(2); audio_info("Failed to open file for reading");}
        return false;
    }

    setDatamode(AUDIO_LOCALFILE);
    m_file_size = audiofile.size();//TEST loop

    char* afn = NULL;  // audioFileName

#ifdef SDFATFS_USED
    audiofile.getName(chbuf, sizeof(chbuf));
    afn = strdup(chbuf);
#else
    afn = strdup(audiofile.name());
#endif

    uint8_t dotPos = lastIndexOf(afn, ".");
    for(uint8_t i = dotPos + 1; i < strlen(afn); i++){
        afn[i] = toLowerCase(afn[i]);
    }

    if(endsWith(afn, ".mp3"))  m_codec = CODEC_MP3; // m_codec is by default CODEC_NONE
    if(endsWith(afn, ".m4a"))  m_codec = CODEC_M4A;
    if(endsWith(afn, ".aac"))  m_codec = CODEC_AAC;
    if(endsWith(afn, ".wav"))  m_codec = CODEC_WAV;
    if(endsWith(afn, ".flac")) m_codec = CODEC_FLAC;

    if(m_codec == CODEC_NONE) AUDIO_INFO("The %s format is not supported", afn + dotPos);

    if(afn) {free(afn); afn = NULL;}

    bool ret = initializeDecoder();
    if(ret) m_f_running = true;
    else audiofile.close();
    return ret;
}
//---------------------------------------------------------------------------------------------------------------------
bool Audio::connecttospeech(const char* speech, const char* lang){

    setDefaults();
    char host[] = "translate.google.com.vn";
    char path[] = "/translate_tts";

    uint16_t speechLen = strlen(speech);
    uint16_t speechBuffLen = speechLen + 300;
    memcpy(m_lastHost, speech, 256);
    char* speechBuff = (char*)malloc(speechBuffLen);
    if(!speechBuff) {log_e("out of memory"); return false;}
    memcpy(speechBuff, speech, speechLen);
    speechBuff[speechLen] = '\0';
    urlencode(speechBuff, speechBuffLen);

    char resp[strlen(speechBuff) + 200] = "";
    strcat(resp, "GET ");
    strcat(resp, path);
    strcat(resp, "?ie=UTF-8&tl=");
    strcat(resp, lang);
    strcat(resp, "&client=tw-ob&q=");
    strcat(resp, speechBuff);
    strcat(resp, " HTTP/1.1\r\n");
    strcat(resp, "Host: ");
    strcat(resp, host);
    strcat(resp, "\r\n");
    strcat(resp, "User-Agent: Mozilla/5.0 \r\n");
    strcat(resp, "Accept-Encoding: identity\r\n");
    strcat(resp, "Accept: text/html\r\n");
    strcat(resp, "Connection: close\r\n\r\n");

    if(speechBuff){free(speechBuff); speechBuff = NULL;}
    _client = static_cast<WiFiClient*>(&client);
    if(!_client->connect(host, 80)) {
        log_e("Connection failed");
        return false;
    }
    _client->print(resp);

    m_streamType = ST_WEBFILE;
    m_f_running = true;
    m_f_ssl = false;
    m_f_tts = true;
    setDatamode(HTTP_RESPONSE_HEADER);

    return true;
}
//---------------------------------------------------------------------------------------------------------------------
bool Audio::connecttomarytts(const char* speech, const char* lang, const char* voice){

    //lang:     fr, te, ru, en_US, en_GB, sv, lb, tr, de, it

    //voice:    upmc-pierre-hsmm             fr male hmm
    //          upmc-pierre                  fr male unitselection general
    //          upmc-jessica-hsmm            fr female hmm
    //          upmc-jessica                 fr female unitselection general
    //          marylux                      lb female unitselection general
    //          istc-lucia-hsmm              it female hmm
    //          enst-dennys-hsmm             fr male hmm
    //          enst-camille-hsmm            fr female hmm
    //          enst-camille                 fr female unitselection general
    //          dfki-spike-hsmm              en_GB male hmm
    //          dfki-spike                   en_GB male unitselection general
    //          dfki-prudence-hsmm           en_GB female hmm
    //          dfki-prudence                en_GB female unitselection general
    //          dfki-poppy-hsmm              en_GB female hmm
    //          dfki-poppy                   en_GB female unitselection general
    //          dfki-pavoque-styles          de male unitselection general
    //          dfki-pavoque-neutral-hsmm    de male hmm
    //          dfki-pavoque-neutral         de male unitselection general
    //          dfki-ot-hsmm                 tr male hmm
    //          dfki-ot                      tr male unitselection general
    //          dfki-obadiah-hsmm            en_GB male hmm
    //          dfki-obadiah                 en_GB male unitselection general
    //          cmu-slt-hsmm                 en_US female hmm
    //          cmu-slt                      en_US female unitselection general
    //          cmu-rms-hsmm                 en_US male hmm
    //          cmu-rms                      en_US male unitselection general
    //          cmu-nk-hsmm                  te female hmm
    //          cmu-bdl-hsmm                 en_US male hmm
    //          cmu-bdl                      en_US male unitselection general
    //          bits4                        de female unitselection general
    //          bits3-hsmm                   de male hmm
    //          bits3                        de male unitselection general
    //          bits2                        de male unitselection general
    //          bits1-hsmm                   de female hmm
    //          bits1                        de female unitselection general

    setDefaults();
    char host[] = "mary.dfki.de";
    char path[] = "/process";
    int port = 59125;

    uint16_t speechLen = strlen(speech);
    uint16_t speechBuffLen = speechLen + 300;
    memcpy(m_lastHost, speech, 256);
    char* speechBuff = (char*)malloc(speechBuffLen);
    if(!speechBuff) {log_e("out of memory"); return false;}
    memcpy(speechBuff, speech, speechLen);
    speechBuff[speechLen] = '\0';
    urlencode(speechBuff, speechBuffLen);

    char resp[strlen(speechBuff) + 200] = "";
    strcat(resp, "GET ");
    strcat(resp, path);
    strcat(resp, "?INPUT_TEXT=");
    strcat(resp, speechBuff);
    strcat(resp, "&INPUT_TYPE=TEXT");
    strcat(resp, "&OUTPUT_TYPE=AUDIO");
    strcat(resp, "&AUDIO=WAVE_FILE");
    strcat(resp, "&LOCALE=");
    strcat(resp, lang);
    strcat(resp, "&VOICE=");
    strcat(resp, voice);
    strcat(resp, " HTTP/1.1\r\n");
    strcat(resp, "Host: ");
    strcat(resp, host);
    strcat(resp, "\r\n");
    strcat(resp, "User-Agent: Mozilla/5.0 \r\n");
    strcat(resp, "Accept-Encoding: identity\r\n");
    strcat(resp, "Accept: text/html\r\n");
    strcat(resp, "Connection: close\r\n\r\n");

    if(speechBuff){free(speechBuff); speechBuff = NULL;}
    _client = static_cast<WiFiClient*>(&client);
    if(!_client->connect(host, port)) {
        log_e("Connection failed");
        return false;
    }
    _client->print(resp);

    m_streamType = ST_WEBFILE;
    m_f_running = true;
    m_f_ssl = false;
    m_f_tts = true;
    setDatamode(HTTP_RESPONSE_HEADER);

    return true;
}
//---------------------------------------------------------------------------------------------------------------------
void Audio::urlencode(char* buff, uint16_t buffLen, bool spacesOnly) {

    uint16_t len = strlen(buff);
    uint8_t* tmpbuff = (uint8_t*)malloc(buffLen);
    if(!tmpbuff) {log_e("out of memory"); return;}
    char c;
    char code0;
    char code1;
    uint16_t j = 0;
    for(int i = 0; i < len; i++) {
        c = buff[i];
        if(isalnum(c)) tmpbuff[j++] = c;
        else if(spacesOnly){
            if(c == ' '){
                tmpbuff[j++] = '%';
                tmpbuff[j++] = '2';
                tmpbuff[j++] = '0';
            }
            else{
                tmpbuff[j++] = c;
            }
        }
        else {
            code1 = (c & 0xf) + '0';
            if((c & 0xf) > 9) code1 = (c & 0xf) - 10 + 'A';
            c = (c >> 4) & 0xf;
            code0 = c + '0';
            if(c > 9) code0 = c - 10 + 'A';
            tmpbuff[j++] = '%';
            tmpbuff[j++] = code0;
            tmpbuff[j++] = code1;
        }
        if(j == buffLen - 1){
            log_e("out of memory");
            break;
        }
    }
    memcpy(buff, tmpbuff, j);
    buff[j] ='\0';
    free(tmpbuff);
}
//---------------------------------------------------------------------------------------------------------------------
void Audio::showID3Tag(const char* tag, const char* value){

    chbuf[0] = 0;
    // V2.2
    if(!strcmp(tag, "CNT")) sprintf(chbuf, "Play counter: %s", value);
    // if(!strcmp(tag, "COM")) sprintf(chbuf, "Comments: %s", value);
    if(!strcmp(tag, "CRA")) sprintf(chbuf, "Audio encryption: %s", value);
    if(!strcmp(tag, "CRM")) sprintf(chbuf, "Encrypted meta frame: %s", value);
    if(!strcmp(tag, "ETC")) sprintf(chbuf, "Event timing codes: %s", value);
    if(!strcmp(tag, "EQU")) sprintf(chbuf, "Equalization: %s", value);
    if(!strcmp(tag, "IPL")) sprintf(chbuf, "Involved people list: %s", value);
    if(!strcmp(tag, "PIC")) sprintf(chbuf, "Attached picture: %s", value);
    if(!strcmp(tag, "SLT")) sprintf(chbuf, "Synchronized lyric/text: %s", value);
    // if(!strcmp(tag, "TAL")) sprintf(chbuf, "Album/Movie/Show title: %s", value);
    if(!strcmp(tag, "TBP")) sprintf(chbuf, "BPM (Beats Per Minute): %s", value);
    if(!strcmp(tag, "TCM")) sprintf(chbuf, "Composer: %s", value);
    if(!strcmp(tag, "TCO")) sprintf(chbuf, "Content type: %s", value);
    if(!strcmp(tag, "TCR")) sprintf(chbuf, "Copyright message: %s", value);
    if(!strcmp(tag, "TDA")) sprintf(chbuf, "Date: %s", value);
    if(!strcmp(tag, "TDY")) sprintf(chbuf, "Playlist delay: %s", value);
    if(!strcmp(tag, "TEN")) sprintf(chbuf, "Encoded by: %s", value);
    if(!strcmp(tag, "TFT")) sprintf(chbuf, "File type: %s", value);
    if(!strcmp(tag, "TIM")) sprintf(chbuf, "Time: %s", value);
    if(!strcmp(tag, "TKE")) sprintf(chbuf, "Initial key: %s", value);
    if(!strcmp(tag, "TLA")) sprintf(chbuf, "Language(s): %s", value);
    if(!strcmp(tag, "TLE")) sprintf(chbuf, "Length: %s", value);
    if(!strcmp(tag, "TMT")) sprintf(chbuf, "Media type: %s", value);
    if(!strcmp(tag, "TOA")) sprintf(chbuf, "Original artist(s)/performer(s): %s", value);
    if(!strcmp(tag, "TOF")) sprintf(chbuf, "Original filename: %s", value);
    if(!strcmp(tag, "TOL")) sprintf(chbuf, "Original Lyricist(s)/text writer(s): %s", value);
    if(!strcmp(tag, "TOR")) sprintf(chbuf, "Original release year: %s", value);
    if(!strcmp(tag, "TOT")) sprintf(chbuf, "Original album/Movie/Show title: %s", value);
    if(!strcmp(tag, "TP1")) sprintf(chbuf, "Lead artist(s)/Lead performer(s)/Soloist(s)/Performing group: %s", value);
    if(!strcmp(tag, "TP2")) sprintf(chbuf, "Band/Orchestra/Accompaniment: %s", value);
    if(!strcmp(tag, "TP3")) sprintf(chbuf, "Conductor/Performer refinement: %s", value);
    if(!strcmp(tag, "TP4")) sprintf(chbuf, "Interpreted, remixed, or otherwise modified by: %s", value);
    if(!strcmp(tag, "TPA")) sprintf(chbuf, "Part of a set: %s", value);
    if(!strcmp(tag, "TPB")) sprintf(chbuf, "Publisher: %s", value);
    if(!strcmp(tag, "TRC")) sprintf(chbuf, "ISRC (International Standard Recording Code): %s", value);
    if(!strcmp(tag, "TRD")) sprintf(chbuf, "Recording dates: %s", value);
    if(!strcmp(tag, "TRK")) sprintf(chbuf, "Track number/Position in set: %s", value);
    if(!strcmp(tag, "TSI")) sprintf(chbuf, "Size: %s", value);
    if(!strcmp(tag, "TSS")) sprintf(chbuf, "Software/hardware and settings used for encoding: %s", value);
    if(!strcmp(tag, "TT1")) sprintf(chbuf, "Content group description: %s", value);
    if(!strcmp(tag, "TT2")) sprintf(chbuf, "Title/Songname/Content description: %s", value);
    if(!strcmp(tag, "TT3")) sprintf(chbuf, "Subtitle/Description refinement: %s", value);
    if(!strcmp(tag, "TXT")) sprintf(chbuf, "Lyricist/text writer: %s", value);
    if(!strcmp(tag, "TXX")) sprintf(chbuf, "User defined text information frame: %s", value);
    if(!strcmp(tag, "TYE")) sprintf(chbuf, "Year: %s", value);
    if(!strcmp(tag, "UFI")) sprintf(chbuf, "Unique file identifier: %s", value);
    if(!strcmp(tag, "ULT")) sprintf(chbuf, "Unsychronized lyric/text transcription: %s", value);
    if(!strcmp(tag, "WAF")) sprintf(chbuf, "Official audio file webpage: %s", value);
    if(!strcmp(tag, "WAR")) sprintf(chbuf, "Official artist/performer webpage: %s", value);
    if(!strcmp(tag, "WAS")) sprintf(chbuf, "Official audio source webpage: %s", value);
    if(!strcmp(tag, "WCM")) sprintf(chbuf, "Commercial information: %s", value);
    if(!strcmp(tag, "WCP")) sprintf(chbuf, "Copyright/Legal information: %s", value);
    if(!strcmp(tag, "WPB")) sprintf(chbuf, "Publishers official webpage: %s", value);
    if(!strcmp(tag, "WXX")) sprintf(chbuf, "User defined URL link frame: %s", value);

    // V2.3 V2.4 tags
    // if(!strcmp(tag, "COMM")) sprintf(chbuf, "Comment: %s", value);
    if(!strcmp(tag, "OWNE")) sprintf(chbuf, "Ownership: %s", value);
    // if(!strcmp(tag, "PRIV")) sprintf(chbuf, "Private: %s", value);
    if(!strcmp(tag, "SYLT")) sprintf(chbuf, "SynLyrics: %s", value);
    if(!strcmp(tag, "TALB")) sprintf(chbuf, "Album: %s", value);
    if(!strcmp(tag, "TBPM")) sprintf(chbuf, "BeatsPerMinute: %s", value);
    if(!strcmp(tag, "TCMP")) sprintf(chbuf, "Compilation: %s", value);
    if(!strcmp(tag, "TCOM")) sprintf(chbuf, "Composer: %s", value);
    if(!strcmp(tag, "TCON")) sprintf(chbuf, "ContentType: %s", value);
    if(!strcmp(tag, "TCOP")) sprintf(chbuf, "Copyright: %s", value);
    if(!strcmp(tag, "TDAT")) sprintf(chbuf, "Date: %s", value);
    if(!strcmp(tag, "TEXT")) sprintf(chbuf, "Lyricist: %s", value);
    if(!strcmp(tag, "TIME")) sprintf(chbuf, "Time: %s", value);
    if(!strcmp(tag, "TIT1")) sprintf(chbuf, "Grouping: %s", value);
    if(!strcmp(tag, "TIT2")) sprintf(chbuf, "Title: %s", value);
    if(!strcmp(tag, "TIT3")) sprintf(chbuf, "Subtitle: %s", value);
    if(!strcmp(tag, "TLAN")) sprintf(chbuf, "Language: %s", value);
    if(!strcmp(tag, "TLEN")) sprintf(chbuf, "Length (ms): %s", value);
    if(!strcmp(tag, "TMED")) sprintf(chbuf, "Media: %s", value);
    if(!strcmp(tag, "TOAL")) sprintf(chbuf, "OriginalAlbum: %s", value);
    if(!strcmp(tag, "TOPE")) sprintf(chbuf, "OriginalArtist: %s", value);
    if(!strcmp(tag, "TORY")) sprintf(chbuf, "OriginalReleaseYear: %s", value);
    if(!strcmp(tag, "TPE1")) sprintf(chbuf, "Artist: %s", value);
    if(!strcmp(tag, "TPE2")) sprintf(chbuf, "Band: %s", value);
    if(!strcmp(tag, "TPE3")) sprintf(chbuf, "Conductor: %s", value);
    if(!strcmp(tag, "TPE4")) sprintf(chbuf, "InterpretedBy: %s", value);
    if(!strcmp(tag, "TPOS")) sprintf(chbuf, "PartOfSet: %s", value);
    if(!strcmp(tag, "TPUB")) sprintf(chbuf, "Publisher: %s", value);
    if(!strcmp(tag, "TRCK")) sprintf(chbuf, "Track: %s", value);
    if(!strcmp(tag, "TSSE")) sprintf(chbuf, "SettingsForEncoding: %s", value);
    if(!strcmp(tag, "TRDA")) sprintf(chbuf, "RecordingDates: %s", value);
    if(!m_f_m3u8data) if(!strcmp(tag, "TXXX")) sprintf(chbuf, "UserDefinedText: %s", value);
    if(!strcmp(tag, "TYER")) sprintf(chbuf, "Year: %s", value);
    if(!strcmp(tag, "USER")) sprintf(chbuf, "TermsOfUse: %s", value);
    if(!strcmp(tag, "USLT")) sprintf(chbuf, "Lyrics: %s", value);
    if(!strcmp(tag, "WOAR")) sprintf(chbuf, "OfficialArtistWebpage: %s", value);
    if(!strcmp(tag, "XDOR")) sprintf(chbuf, "OriginalReleaseTime: %s", value);

    latinToUTF8(chbuf, sizeof(chbuf));
    if(chbuf[0] != 0) if(audio_id3data) audio_id3data(chbuf);
}
//---------------------------------------------------------------------------------------------------------------------
void Audio::unicode2utf8(char* buff, uint32_t len){
    // converts unicode in UTF-8, buff contains the string to be converted up to len
    // range U+1 ... U+FFFF
    uint8_t* tmpbuff = (uint8_t*)malloc(len * 2);
    if(!tmpbuff) {log_e("out of memory"); return;}
    bool bitorder = false;
    uint16_t j = 0;
    uint16_t k = 0;
    uint16_t m = 0;
    uint8_t uni_h = 0;
    uint8_t uni_l = 0;

    while(m < len - 1) {
        if((buff[m] == 0xFE) && (buff[m + 1] == 0xFF)) {
            bitorder = true;
            j = m + 2;
        }  // LSB/MSB
        if((buff[m] == 0xFF) && (buff[m + 1] == 0xFE)) {
            bitorder = false;
            j = m + 2;
        }  // MSB/LSB
        m++;
    } // seek for last bitorder
    m = 0;
    if(j > 0) {
        for(k = j; k < len; k += 2) {
            if(bitorder == true) {
                uni_h = (uint8_t)buff[k];
                uni_l = (uint8_t)buff[k + 1];
            }
            else {
                uni_l = (uint8_t)buff[k];
                uni_h = (uint8_t)buff[k + 1];
            }

            uint16_t uni_hl = ((uni_h << 8) | uni_l);

            if (uni_hl < 0X80){
                tmpbuff[m] = uni_l;
                m++;
            }
            else if (uni_hl < 0X800) {
                tmpbuff[m]= ((uni_hl >> 6) | 0XC0);
                m++;
                tmpbuff[m] =((uni_hl & 0X3F) | 0X80);
                m++;
            }
            else {
                tmpbuff[m] = ((uni_hl >> 12) | 0XE0);
                m++;
                tmpbuff[m] = (((uni_hl >> 6) & 0X3F) | 0X80);
                m++;
                tmpbuff[m] = ((uni_hl & 0X3F) | 0X80);
                m++;
            }
        }
    }
    buff[m] = 0;
    memcpy(buff, tmpbuff, m);
    if(tmpbuff){free(tmpbuff); tmpbuff = NULL;}
}
//---------------------------------------------------------------------------------------------------------------------
bool Audio::latinToUTF8(char* buff, size_t bufflen){
    // most stations send  strings in UTF-8 but a few sends in latin. To standardize this, all latin strings are
    // converted to UTF-8. If UTF-8 is already present, nothing is done and true is returned.
    // A conversion to UTF-8 extends the string. Therefore it is necessary to know the buffer size. If the converted
    // string does not fit into the buffer, false is returned
    // utf8 bytelength: >=0xF0 3 bytes, >=0xE0 2 bytes, >=0xC0 1 byte, e.g. e293ab is ⓫

    uint16_t pos = 0;
    uint8_t  ext_bytes = 0;
    uint16_t len = strlen(buff);
    uint8_t  c;

    while(pos < len){
        c = buff[pos];
        if(c >= 0xC2) {    // is UTF8 char
            pos++;
            if(c >= 0xC0 && buff[pos] < 0x80) {ext_bytes++; pos++;}
            if(c >= 0xE0 && buff[pos] < 0x80) {ext_bytes++; pos++;}
            if(c >= 0xF0 && buff[pos] < 0x80) {ext_bytes++; pos++;}
        }
        else pos++;
    }
    if(!ext_bytes) return true; // is UTF-8, do nothing

    pos = 0;

    while(buff[pos] != 0){
        len = strlen(buff);
        if(buff[pos] >= 0x80 && buff[pos+1] < 0x80){       // is not UTF8, is latin?
            for(int i = len+1; i > pos; i--){
                buff[i+1] = buff[i];
            }
            uint8_t c = buff[pos];
            buff[pos++] = 0xc0 | ((c >> 6) & 0x1f);      // 2+1+5 bits
            buff[pos++] = 0x80 | ((char)c & 0x3f);       // 1+1+6 bits
        }
        pos++;
        if(pos > bufflen -3){
            buff[bufflen -1] = '\0';
            return false; // do not overwrite
        }
    }
    return true;
}
//---------------------------------------------------------------------------------------------------------------------
size_t Audio::readAudioHeader(uint32_t bytes){
    size_t bytesReaded = 0;
    if(m_codec == CODEC_WAV){
        int res = read_WAV_Header(InBuff.getReadPtr(), bytes);
        if(res >= 0) bytesReaded = res;
        else{ // error, skip header
            m_controlCounter = 100;
        }
    }
    if(m_codec == CODEC_MP3){
        int res = read_ID3_Header(InBuff.getReadPtr(), bytes);
        if(res >= 0) bytesReaded = res;
        else{ // error, skip header
            m_controlCounter = 100;
        }
    }
    if(m_codec == CODEC_M4A){
        int res = read_M4A_Header(InBuff.getReadPtr(), bytes);
        if(res >= 0) bytesReaded = res;
        else{ // error, skip header
            m_controlCounter = 100;
        }
    }
    if(m_codec == CODEC_AAC){
        // stream only, no header
        m_audioDataSize = getFileSize();
        m_controlCounter = 100;
    }
    if(m_codec == CODEC_FLAC){
        int res = read_FLAC_Header(InBuff.getReadPtr(), bytes);
        if(res >= 0) bytesReaded = res;
        else{ // error, skip header
            stopSong();
            m_controlCounter = 100;
        }
    }
    if(!isRunning()){
        log_e("Processing stopped due to invalid audio header");
        return 0;
    }
    return bytesReaded;
}
//---------------------------------------------------------------------------------------------------------------------
int Audio::read_WAV_Header(uint8_t* data, size_t len) {
    static size_t headerSize;
    static uint32_t cs = 0;
    static uint8_t bts = 0;

    if(m_controlCounter == 0){
        m_controlCounter ++;
        if((*data != 'R') || (*(data + 1) != 'I') || (*(data + 2) != 'F') || (*(data + 3) != 'F')) {
            AUDIO_INFO("file has no RIFF tag");
            headerSize = 0;
            return -1; //false;
        }
        else{
            headerSize = 4;
            return 4; // ok
        }
    }

    if(m_controlCounter == 1){
        m_controlCounter ++;
        cs = (uint32_t) (*data + (*(data + 1) << 8) + (*(data + 2) << 16) + (*(data + 3) << 24) - 8);
        headerSize += 4;
        return 4; // ok
    }

    if(m_controlCounter == 2){
        m_controlCounter ++;
        if((*data  != 'W') || (*(data + 1) != 'A') || (*(data + 2) != 'V') || (*(data + 3) != 'E')) {
            AUDIO_INFO("format tag is not WAVE");
            return -1;//false;
        }
        else {
            headerSize += 4;
            return 4;
        }
    }

    if(m_controlCounter == 3){
        if((*data  == 'f') && (*(data + 1) == 'm') && (*(data + 2) == 't')) {
            m_controlCounter ++;
            headerSize += 4;
            return 4;
        }
        else{
            headerSize += 4;
            return 4;
        }
    }

    if(m_controlCounter == 4){
        m_controlCounter ++;
        cs = (uint32_t) (*data + (*(data + 1) << 8));
        if(cs > 40) return -1; //false, something going wrong
        bts = cs - 16; // bytes to skip if fmt chunk is >16
        headerSize += 4;
        return 4;
    }

    if(m_controlCounter == 5){
        m_controlCounter ++;
        uint16_t fc  = (uint16_t) (*(data + 0)  + (*(data + 1)  << 8));         // Format code
        uint16_t nic = (uint16_t) (*(data + 2)  + (*(data + 3)  << 8));         // Number of interleaved channels
        uint32_t sr  = (uint32_t) (*(data + 4)  + (*(data + 5)  << 8) +
                                  (*(data + 6)  << 16) + (*(data + 7)  << 24)); // Samplerate
        uint32_t dr  = (uint32_t) (*(data + 8)  + (*(data + 9)  << 8) +
                                  (*(data + 10) << 16) + (*(data + 11) << 24)); // Datarate
        uint16_t dbs = (uint16_t) (*(data + 12) + (*(data + 13) << 8));         // Data block size
        uint16_t bps = (uint16_t) (*(data + 14) + (*(data + 15) << 8));         // Bits per sample

        AUDIO_INFO("FormatCode: %u", fc);
        // AUDIO_INFO("Channel: %u", nic);
        // AUDIO_INFO("SampleRate: %u", sr);
        AUDIO_INFO("DataRate: %u", dr);
        AUDIO_INFO("DataBlockSize: %u", dbs);
        AUDIO_INFO("BitsPerSample: %u", bps);

        if((bps != 8) && (bps != 16)){
            AUDIO_INFO("BitsPerSample is %u,  must be 8 or 16" , bps);
            stopSong();
            return -1;
        }
        if((nic != 1) && (nic != 2)){
            AUDIO_INFO("num channels is %u,  must be 1 or 2" , nic); audio_info(chbuf);
            stopSong();
            return -1;
        }
        if(fc != 1) {
            AUDIO_INFO("format code is not 1 (PCM)");
            stopSong();
            return -1 ; //false;
        }
        setBitsPerSample(bps);
        setChannels(nic);
        setSampleRate(sr);
        setBitrate(nic * sr * bps);
    //    AUDIO_INFO("BitRate: %u", m_bitRate);
        headerSize += 16;
        return 16; // ok
    }

    if(m_controlCounter == 6){
        m_controlCounter ++;
        headerSize += bts;
        return bts; // skip to data
    }

    if(m_controlCounter == 7){
        if((*(data + 0) == 'd') && (*(data + 1) == 'a') && (*(data + 2) == 't') && (*(data + 3) == 'a')){
            m_controlCounter ++;
            vTaskDelay(30);
            headerSize += 4;
            return 4;
        }
        else{
            headerSize ++;
            return 1;
        }
    }

    if(m_controlCounter == 8){
        m_controlCounter ++;
        size_t cs =  *(data + 0) + (*(data + 1) << 8) + (*(data + 2) << 16) + (*(data + 3) << 24); //read chunkSize
        headerSize += 4;
        if(getDatamode() == AUDIO_LOCALFILE) m_contentlength = getFileSize();
        if(cs){
            m_audioDataSize = cs  - 44;
        }
        else { // sometimes there is nothing here
            if(getDatamode() == AUDIO_LOCALFILE) m_audioDataSize = getFileSize() - headerSize;
            if(m_streamType == ST_WEBFILE) m_audioDataSize = m_contentlength - headerSize;
        }
        AUDIO_INFO("Audio-Length: %u", m_audioDataSize);
        return 4;
    }
    m_controlCounter = 100; // header succesfully read
    m_audioDataStart = headerSize;
    return 0;
}
//---------------------------------------------------------------------------------------------------------------------
int Audio::read_FLAC_Header(uint8_t *data, size_t len) {
    static size_t headerSize;
    static size_t retvalue = 0;
    static bool   f_lastMetaBlock;

    if(retvalue) {
        if(retvalue > len) { // if returnvalue > bufferfillsize
            if(len > InBuff.getMaxBlockSize()) len = InBuff.getMaxBlockSize();
            retvalue -= len; // and wait for more bufferdata
            return len;
        }
        else {
            size_t tmp = retvalue;
            retvalue = 0;
            return tmp;
        }
        return 0;
    }
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if(m_controlCounter == FLAC_BEGIN) {  // init
        headerSize = 0;
        retvalue = 0;
        m_audioDataStart = 0;
        f_lastMetaBlock = false;
        m_controlCounter = FLAC_MAGIC;
        if(getDatamode() == AUDIO_LOCALFILE){
            m_contentlength = getFileSize();
            AUDIO_INFO("Content-Length: %u", m_contentlength);
        }
        return 0;
    }
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if(m_controlCounter == FLAC_MAGIC) { /* check MAGIC STRING */
        if(specialIndexOf(data, "fLaC", 10) != 0) {
            log_e("Magic String 'fLaC' not found in header");
            stopSong();
            return -1;
        }
        m_controlCounter = FLAC_MBH;
        headerSize = 4;
        retvalue = 4;
        return 0;
    }
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if(m_controlCounter == FLAC_MBH) { /* METADATA_BLOCK_HEADER */
        uint8_t blockType = *data;
        if(!f_lastMetaBlock){
            if(blockType & 128) {f_lastMetaBlock = true;}
            blockType &= 127;
            if(blockType == 0) m_controlCounter = FLAC_SINFO;
            if(blockType == 1) m_controlCounter = FLAC_PADDING;
            if(blockType == 2) m_controlCounter = FLAC_APP;
            if(blockType == 3) m_controlCounter = FLAC_SEEK;
            if(blockType == 4) m_controlCounter = FLAC_VORBIS;
            if(blockType == 5) m_controlCounter = FLAC_CUESHEET;
            if(blockType == 6) m_controlCounter = FLAC_PICTURE;
            headerSize += 1;
            retvalue = 1;
            return 0;
        }
        m_controlCounter = FLAC_OKAY;
        m_audioDataStart = headerSize;
        m_audioDataSize = m_contentlength - m_audioDataStart;
        AUDIO_INFO("Audio-Length: %u", m_audioDataSize);
        retvalue = 0;
        return 0;
    }
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if(m_controlCounter == FLAC_SINFO) { /* Stream info block */
        size_t l = bigEndian(data, 3);
        vTaskDelay(2);
        m_flacMaxBlockSize = bigEndian(data + 5, 2);
        AUDIO_INFO("FLAC maxBlockSize: %u", m_flacMaxBlockSize);
        vTaskDelay(2);
        m_flacMaxFrameSize = bigEndian(data + 10, 3);
        if(m_flacMaxFrameSize){
            AUDIO_INFO("FLAC maxFrameSize: %u", m_flacMaxFrameSize);
        }
        else {
            AUDIO_INFO("FLAC maxFrameSize: N/A");
        }
        if(m_flacMaxFrameSize > InBuff.getMaxBlockSize()) {
            log_e("FLAC maxFrameSize too large!");
            stopSong();
            return -1;
        }
//        InBuff.changeMaxBlockSize(m_flacMaxFrameSize);
        vTaskDelay(2);
        uint32_t nextval = bigEndian(data + 13, 3);
        m_flacSampleRate = nextval >> 4;
        AUDIO_INFO("FLAC sampleRate: %u", m_flacSampleRate);
        vTaskDelay(2);
        m_flacNumChannels = ((nextval & 0x06) >> 1) + 1;
        AUDIO_INFO("FLAC numChannels: %u", m_flacNumChannels);
        vTaskDelay(2);
        uint8_t bps = (nextval & 0x01) << 4;
        bps += (*(data +16) >> 4) + 1;
        m_flacBitsPerSample = bps;
        if((bps != 8) && (bps != 16)){
            log_e("bits per sample must be 8 or 16, is %i", bps);
            stopSong();
            return -1;
        }
        AUDIO_INFO("FLAC bitsPerSample: %u", m_flacBitsPerSample);
        m_flacTotalSamplesInStream = bigEndian(data + 17, 4);
        if(m_flacTotalSamplesInStream){
            AUDIO_INFO("total samples in stream: %u", m_flacTotalSamplesInStream);
        }
        else{
            AUDIO_INFO("total samples in stream: N/A");
        }
        if(bps != 0 && m_flacTotalSamplesInStream) {
            AUDIO_INFO("audio file duration: %u seconds", m_flacTotalSamplesInStream / m_flacSampleRate);
        }
        m_controlCounter = FLAC_MBH; // METADATA_BLOCK_HEADER
        retvalue = l + 3;
        headerSize += retvalue;
        return 0;
    }
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if(m_controlCounter == FLAC_PADDING) { /* PADDING */
        size_t l = bigEndian(data, 3);
        m_controlCounter = FLAC_MBH;
        retvalue = l + 3;
        headerSize += retvalue;
        return 0;
    }
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if(m_controlCounter == FLAC_APP) { /* APPLICATION */
        size_t l = bigEndian(data, 3);
        m_controlCounter = FLAC_MBH;
        retvalue = l + 3;
        headerSize += retvalue;
        return 0;
    }
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if(m_controlCounter == FLAC_SEEK) { /* SEEKTABLE */
        size_t l = bigEndian(data, 3);
        m_controlCounter = FLAC_MBH;
        retvalue = l + 3;
        headerSize += retvalue;
        return 0;
    }
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if(m_controlCounter == FLAC_VORBIS) { /* VORBIS COMMENT */                          // field names
        const char fn[7][12] = {"TITLE", "VERSION", "ALBUM", "TRACKNUMBER", "ARTIST", "PERFORMER", "GENRE"};
        int offset;
        size_t l = bigEndian(data, 3);

        for(int i = 0; i < 7; i++){
            offset = specialIndexOf(data, fn[i], len);
            if(offset >= 0){
                sprintf(chbuf, "%s: %s", fn[i], data + offset + strlen(fn[i]) + 1);
                chbuf[strlen(chbuf) - 1] = 0;
                for(int i=0; i<strlen(chbuf);i++){
                    if(chbuf[i] == 255) chbuf[i] = 0;
                }
                if(audio_id3data) audio_id3data(chbuf);
            }
        }
        m_controlCounter = FLAC_MBH;
        retvalue = l + 3;
        headerSize += retvalue;
        return 0;
    }
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if(m_controlCounter == FLAC_CUESHEET) { /* CUESHEET */
        size_t l = bigEndian(data, 3);
        m_controlCounter = FLAC_MBH;
        retvalue = l + 3;
        headerSize += retvalue;
        return 0;
    }
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if(m_controlCounter == FLAC_PICTURE) { /* PICTURE */
        size_t l = bigEndian(data, 3);
        m_controlCounter = FLAC_MBH;
        retvalue = l + 3;
        headerSize += retvalue;
        return 0;
    }
    return 0;
}
//---------------------------------------------------------------------------------------------------------------------
int Audio::read_ID3_Header(uint8_t *data, size_t len) {

    static size_t id3Size;
    static size_t headerSize;
    static uint8_t ID3version;
    static int ehsz = 0;
    static char tag[5];
    static char frameid[5];
    static size_t framesize = 0;
    static bool compressed = false;
    static bool APIC_seen = false;
    static size_t APIC_size = 0;
    static uint32_t APIC_pos = 0;
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if(m_controlCounter == 0){      /* read ID3 tag and ID3 header size */
        if(getDatamode() == AUDIO_LOCALFILE){
            ID3version = 0;
            m_contentlength = getFileSize();
            AUDIO_INFO("Content-Length: %u", m_contentlength);
        }
        m_controlCounter ++;
        APIC_seen = false;
        headerSize = 0;
        ehsz = 0;
        if(specialIndexOf(data, "ID3", 4) != 0) { // ID3 not found
            if(!m_f_m3u8data) AUDIO_INFO("file has no mp3 tag, skip metadata");
            m_audioDataSize = m_contentlength;
            if(!m_f_m3u8data) AUDIO_INFO("Audio-Length: %u", m_audioDataSize);
            return -1; // error, no ID3 signature found
        }
        ID3version = *(data + 3);
        switch(ID3version){
            case 2:
                m_f_unsync = (*(data + 5) & 0x80);
                m_f_exthdr = false;
                break;
            case 3:
            case 4:
                m_f_unsync = (*(data + 5) & 0x80); // bit7
                m_f_exthdr = (*(data + 5) & 0x40); // bit6 extended header
                break;
        };
        id3Size = bigEndian(data + 6, 4, 7); //  ID3v2 size  4 * %0xxxxxxx (shift left seven times!!)
        id3Size += 10;

        // Every read from now may be unsync'd
        if(!m_f_m3u8data) AUDIO_INFO("ID3 framesSize: %i", id3Size);
        if(!m_f_m3u8data) AUDIO_INFO("ID3 version: 2.%i", ID3version);

        if(ID3version == 2){
            m_controlCounter = 10;
        }
        headerSize = id3Size;
        m_ID3Size = id3Size;
        headerSize -= 10;

        return 10;
    }
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if(m_controlCounter == 1){      // compute extended header size if exists
        m_controlCounter ++;
        if(m_f_exthdr) {
            AUDIO_INFO("ID3 extended header");
            ehsz =  bigEndian(data, 4);
            headerSize -= 4;
            ehsz -= 4;
            return 4;
        }
        else{
            if(!m_f_m3u8data) AUDIO_INFO("ID3 normal frames");
            return 0;
        }
    }
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if(m_controlCounter == 2){      // skip extended header if exists
        if(ehsz > 256) {
            ehsz -=256;
            headerSize -= 256;
            return 256;} // Throw it away
        else           {
            m_controlCounter ++;
            headerSize -= ehsz;
            return ehsz;} // Throw it away
    }
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if(m_controlCounter == 3){      // read a ID3 frame, get the tag
        if(headerSize == 0){
            m_controlCounter = 99;
            return 0;
        }
        m_controlCounter ++;
        frameid[0] = *(data + 0);
        frameid[1] = *(data + 1);
        frameid[2] = *(data + 2);
        frameid[3] = *(data + 3);
        frameid[4] = 0;
        for(uint8_t i = 0; i < 4; i++) tag[i] = frameid[i]; // tag = frameid

        headerSize -= 4;
        if(frameid[0] == 0 && frameid[1] == 0 && frameid[2] == 0 && frameid[3] == 0) {
            // We're in padding
            m_controlCounter = 98;  // all ID3 metadata processed
        }
        return 4;
    }
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if(m_controlCounter == 4){  // get the frame size
        m_controlCounter = 6;

        if(ID3version == 4){
            framesize = bigEndian(data, 4, 7); // << 7
        }
        else {
            framesize = bigEndian(data, 4);  // << 8
        }
        headerSize -= 4;
        uint8_t flag = *(data + 4); // skip 1st flag
        (void) flag;
        headerSize--;
        compressed = (*(data + 5)) & 0x80; // Frame is compressed using [#ZLIB zlib] with 4 bytes for 'decompressed
                                           // size' appended to the frame header.
        headerSize--;
        uint32_t decompsize = 0;
        if(compressed){
            if(m_f_Log) log_i("iscompressed");
            decompsize = bigEndian(data + 6, 4);
            headerSize -= 4;
            (void) decompsize;
            if(m_f_Log) log_i("decompsize=%u", decompsize);
            return 6 + 4;
        }
        return 6;
    }
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if(m_controlCounter == 5){      // If the frame is larger than 256 bytes, skip the rest
        if(framesize > 256){
            framesize -= 256;
            headerSize -= 256;
            return 256;
        }
        else {
            m_controlCounter = 3; // check next frame
            headerSize -= framesize;
            return framesize;
        }
    }
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if(m_controlCounter == 6){      // Read the value
        m_controlCounter = 5;       // only read 256 bytes
        char value[256];
        char ch = *(data + 0);
        // $00 – ISO-8859-1 (LATIN-1, Identical to ASCII for values smaller than 0x80).
        // $01 – UCS-2 encoded Unicode with BOM, in ID3v2.2 and ID3v2.3.
        // $02 – UTF-16BE encoded Unicode without BOM, in ID3v2.4.
        // $03 – UTF-8 encoded Unicode, in ID3v2.4.
        bool isUnicode = (ch==1) ? true : false;

        if(startsWith(tag, "APIC")) { // a image embedded in file, passing it to external function
            isUnicode = false;
            if(getDatamode() == AUDIO_LOCALFILE){
                APIC_seen = true;
                APIC_pos = id3Size - headerSize;
                APIC_size = framesize;
            }
            return 0;
        }

        size_t fs = framesize;
        if(fs >255) fs = 255;
        for(int i=0; i<fs; i++){
            value[i] = *(data + i);
        }
        framesize -= fs;
        headerSize -= fs;
        value[fs] = 0;
        if(isUnicode && fs > 1) {
            unicode2utf8(value, fs);   // convert unicode to utf-8 U+0020...U+07FF
        }
        if(!isUnicode){
            uint16_t j = 0, k = 0;
            j = 0;
            k = 0;
            while(j < fs) {
                if(value[j] == 0x0A) value[j] = 0x20; // replace LF by space
                if(value[j] > 0x1F) {
                    value[k] = value[j];
                    k++;
                }
                j++;
            } //remove non printables
            if(k>0) value[k] = 0; else value[0] = 0; // new termination
        }
        showID3Tag(tag, value);
        return fs;
    }
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    // -- section V2.2 only , greater Vers above ----
    if(m_controlCounter == 10){ // frames in V2.2, 3bytes identifier, 3bytes size descriptor
        frameid[0] = *(data + 0);
        frameid[1] = *(data + 1);
        frameid[2] = *(data + 2);
        frameid[3] = 0;
        for(uint8_t i = 0; i < 4; i++) tag[i] = frameid[i]; // tag = frameid
        headerSize -= 3;
        size_t len = bigEndian(data + 3, 3);
        headerSize -= 3;
        headerSize -= len;
        char value[256];
        size_t tmp = len;
        if(tmp > 254) tmp = 254;
        memcpy(value, (data + 7), tmp);
        value[tmp+1] = 0;
        chbuf[0] = 0;

        showID3Tag(tag, value);
        if(len == 0) m_controlCounter = 98;

        return 3 + 3 + len;
    }
    // -- end section V2.2 -----------

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if(m_controlCounter == 98){ // skip all ID3 metadata (mostly spaces)
        if(headerSize > 256) {
            headerSize -=256;
            return 256;
        } // Throw it away
        else           {
            m_controlCounter = 99;
            return headerSize;
        } // Throw it away
    }
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if(m_controlCounter == 99){ //  exist another ID3tag?
        m_audioDataStart += id3Size;
        vTaskDelay(30);
        if((*(data + 0) == 'I') && (*(data + 1) == 'D') && (*(data + 2) == '3')) {
            m_controlCounter = 0;
            return 0;
        }
        else {
            m_controlCounter = 100; // ok
            m_audioDataSize = m_contentlength - m_audioDataStart;
            if(!m_f_m3u8data) AUDIO_INFO("Audio-Length: %u", m_audioDataSize);
            if(APIC_seen && audio_id3image){
                size_t pos = audiofile.position();
                audio_id3image(audiofile, APIC_pos, APIC_size);
                audiofile.seek(pos); // the filepointer could have been changed by the user, set it back
            }
            return 0;
        }
    }
    return 0;
}
//---------------------------------------------------------------------------------------------------------------------
int Audio::read_M4A_Header(uint8_t *data, size_t len) {
/*
       ftyp
         | - moov  -> trak -> ... -> mp4a contains raw block parameters
         |    L... -> ilst  contains artist, composer ....
       free (optional)
         |
       mdat contains the audio data                                                      */


    static size_t headerSize = 0;
    static size_t retvalue = 0;
    static size_t atomsize = 0;
    static size_t audioDataPos = 0;

    if(retvalue) {
        if(retvalue > len) { // if returnvalue > bufferfillsize
            if(len > InBuff.getMaxBlockSize()) len = InBuff.getMaxBlockSize();
            retvalue -= len; // and wait for more bufferdata
            return len;
        }
        else {
            size_t tmp = retvalue;
            retvalue = 0;
            return tmp;
        }
        return 0;
    }
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if(m_controlCounter == M4A_BEGIN) {  // init
        headerSize = 0;
        retvalue = 0;
        atomsize = 0;
        audioDataPos = 0;
        m_controlCounter = M4A_FTYP;
        return 0;
    }
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if(m_controlCounter == M4A_FTYP) { /* check_m4a_file */
        atomsize = bigEndian(data, 4); // length of first atom
        if(specialIndexOf(data, "ftyp", 10) != 4) {
            log_e("atom 'type' not found in header");
            stopSong();
            return -1;
        }
        int m4a  = specialIndexOf(data, "M4A ", 20);
        int isom = specialIndexOf(data, "isom", 20);

        if((m4a !=8) && (isom != 8)){
            log_e("subtype 'MA4 ' or 'isom' expected, but found '%s '", (data + 8));
            stopSong();
            return -1;
        }

        m_controlCounter = M4A_CHK;
        retvalue = atomsize;
        headerSize = atomsize;
        return 0;
    }
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if(m_controlCounter == M4A_CHK) { /* check  Tag */
        atomsize = bigEndian(data, 4); // length of this atom
        if(specialIndexOf(data, "moov", 10) == 4) {
            m_controlCounter = M4A_MOOV;
            return 0;
        }
        else if(specialIndexOf(data, "free", 10) == 4) {
            retvalue = atomsize;
            headerSize += atomsize;
            return 0;
        }
        else if(specialIndexOf(data, "mdat", 10) == 4) {
            m_controlCounter = M4A_MDAT;
            return 0;
        }
        else {
            char atomName[5];
            (void)atomName;
            atomName[0] = *data;
            atomName[1] = *(data + 1);
            atomName[2] = *(data + 2);
            atomName[3] = *(data + 3);
            atomName[4] = 0;

            if(m_f_Log) log_i("atom %s found", atomName);

            retvalue = atomsize;
            headerSize += atomsize;
            return 0;
        }
    }
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if(m_controlCounter == M4A_MOOV) {  // moov
        // we are looking for track and ilst
        if(specialIndexOf(data, "trak", len) > 0){
            int offset = specialIndexOf(data, "trak", len);
            retvalue = offset;
            atomsize -= offset;
            headerSize += offset;
            m_controlCounter = M4A_TRAK;
            return 0;
        }
        if(specialIndexOf(data, "ilst", len) > 0){
            int offset = specialIndexOf(data, "ilst", len);
            retvalue = offset;
            atomsize -= offset;
            headerSize += offset;
            m_controlCounter = M4A_ILST;
            return 0;

        }
        if (atomsize > len -10){atomsize -= (len -10); headerSize += (len -10); retvalue = (len -10);}
        else {m_controlCounter = M4A_CHK; retvalue = atomsize; headerSize += atomsize;}
        return 0;
    }
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if(m_controlCounter == M4A_TRAK) {  // trak
        if(specialIndexOf(data, "esds", len) > 0){
            int esds = specialIndexOf(data, "esds", len); // Packaging/Encapsulation And Setup Data
            uint8_t *pos = data + esds;
            uint8_t len_of_OD  = *(pos + 12); // length of this OD (which includes the next 2 tags)
            (void)len_of_OD;
            uint8_t len_of_ESD = *(pos + 20); // length of this Elementary Stream Descriptor
            (void)len_of_ESD;
            uint8_t audioType  = *(pos + 21);

            if     (audioType == 0x40) {AUDIO_INFO("AudioType: MPEG4 / Audio");} // ObjectTypeIndication
            else if(audioType == 0x66) {AUDIO_INFO("AudioType: MPEG2 / Audio");}
            else if(audioType == 0x69) {AUDIO_INFO("AudioType: MPEG2 / Audio Part 3");} // Backward Compatible Audio
            else if(audioType == 0x6B) {AUDIO_INFO("AudioType: MPEG1 / Audio");}
            else                       {AUDIO_INFO("unknown Audio Type %x", audioType);}

            uint8_t streamType = *(pos + 22);
            streamType = streamType >> 2;  // 6 bits
            if(streamType!= 5) { log_e("Streamtype is not audio!"); }

            uint32_t maxBr = bigEndian(pos + 26, 4); // max bitrate
            AUDIO_INFO("max bitrate: %i", maxBr);

            uint32_t avrBr = bigEndian(pos + 30, 4); // avg bitrate
            AUDIO_INFO("avr bitrate: %i", avrBr);

            uint16_t ASC   = bigEndian(pos + 39, 2);

            uint8_t objectType = ASC >> 11; // first 5 bits

            if     (objectType == 1) {AUDIO_INFO("AudioObjectType: AAC Main");} // Audio Object Types
            else if(objectType == 2) {AUDIO_INFO("AudioObjectType: AAC Low Complexity");}
            else if(objectType == 3) {AUDIO_INFO("AudioObjectType: AAC Scalable Sample Rate");}
            else if(objectType == 4) {AUDIO_INFO("AudioObjectType: AAC Long Term Prediction");}
            else if(objectType == 5) {AUDIO_INFO("AudioObjectType: AAC Spectral Band Replication");}
            else if(objectType == 6) {AUDIO_INFO("AudioObjectType: AAC Scalable");}
            else                     {AUDIO_INFO("unknown Audio Type %x", audioType);}

            const uint32_t samplingFrequencies[13] = {
                    96000, 88200, 64000, 48000, 44100, 32000, 24000, 22050, 16000, 12000, 11025, 8000, 7350
            };
            uint8_t sRate = (ASC & 0x0600) >> 7; // next 4 bits Sampling Frequencies
            AUDIO_INFO("Sampling Frequency: %u",samplingFrequencies[sRate]);

            uint8_t chConfig = (ASC & 0x78) >> 3;  // next 4 bits
            if(chConfig == 0) AUDIO_INFO("Channel Configurations: AOT Specifc Config");
            if(chConfig == 1) AUDIO_INFO("Channel Configurations: front-center");
            if(chConfig == 2) AUDIO_INFO("Channel Configurations: front-left, front-right");
            if(chConfig >  2) { log_e("Channel Configurations with more than 2 channels is not allowed!"); }

            uint8_t frameLengthFlag     = (ASC & 0x04);
            uint8_t dependsOnCoreCoder  = (ASC & 0x02);
            (void)dependsOnCoreCoder;
            uint8_t extensionFlag       = (ASC & 0x01);
            (void)extensionFlag;

            if(frameLengthFlag == 0) AUDIO_INFO("AAC FrameLength: 1024 bytes");
            if(frameLengthFlag == 1) AUDIO_INFO("AAC FrameLength: 960 bytes");
        }
        if(specialIndexOf(data, "mp4a", len) > 0){
            int offset = specialIndexOf(data, "mp4a", len);
            int channel = bigEndian(data + offset + 20, 2); // audio parameter must be set before starting
            int bps     = bigEndian(data + offset + 22, 2); // the aac decoder. There are RAW blocks only in m4a
            int srate   = bigEndian(data + offset + 26, 4); //
            setBitsPerSample(bps);
            setChannels(channel);
            setSampleRate(srate);
            setBitrate(bps * channel * srate);
            AUDIO_INFO("ch; %i, bps: %i, sr: %i", channel, bps, srate);
            if(audioDataPos && getDatamode() == AUDIO_LOCALFILE) {
                m_controlCounter = M4A_AMRDY;
                setFilePos(audioDataPos);
                return 0;
            }
        }
        m_controlCounter = M4A_MOOV;
        return 0;
    }
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if(m_controlCounter == M4A_ILST) {  // ilst
        const char info[12][6] = { "nam\0", "ART\0", "alb\0", "too\0",  "cmt\0",  "wrt\0",
                                   "tmpo\0", "trkn\0","day\0", "cpil\0", "aART\0", "gen\0"};
        int offset;
        for(int i=0; i < 12; i++){
            offset = specialIndexOf(data, info[i], len, true);  // seek info[] with '\0'
            if(offset>0) {
                offset += 19; if(*(data + offset) == 0) offset ++;
                char value[256];
                size_t tmp = strlen((const char*)data + offset);
                if(tmp > 254) tmp = 254;
                memcpy(value, (data + offset), tmp);
                value[tmp] = 0;
                chbuf[0] = 0;
                if(i == 0)  sprintf(chbuf, "Title: %s", value);
                if(i == 1)  sprintf(chbuf, "Artist: %s", value);
                if(i == 2)  sprintf(chbuf, "Album: %s", value);
                if(i == 3)  sprintf(chbuf, "Encoder: %s", value);
                if(i == 4)  sprintf(chbuf, "Comment: %s", value);
                if(i == 5)  sprintf(chbuf, "Composer: %s", value);
                if(i == 6)  sprintf(chbuf, "BPM: %s", value);
                if(i == 7)  sprintf(chbuf, "Track Number: %s", value);
                if(i == 8)  sprintf(chbuf, "Year: %s", value);
                if(i == 9)  sprintf(chbuf, "Compile: %s", value);
                if(i == 10) sprintf(chbuf, "Album Artist: %s", value);
                if(i == 11) sprintf(chbuf, "Types of: %s", value);
                if(chbuf[0] != 0) {
                    if(audio_id3data) audio_id3data(chbuf);
                }
            }
        }
        m_controlCounter = M4A_MOOV;
        return 0;
    }
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if(m_controlCounter == M4A_MDAT) {  // mdat
        m_audioDataSize = bigEndian(data, 4) -8; // length of this atom - strlen(M4A_MDAT)
        AUDIO_INFO( "Audio-Length: %u",m_audioDataSize);
        retvalue = 8;
        headerSize += 8;
        m_controlCounter = M4A_AMRDY;  // last step before starting the audio
        return 0;
    }

    if(m_controlCounter == M4A_AMRDY){ // almost ready
        m_audioDataStart = headerSize;
//        m_contentlength = headerSize + m_audioDataSize; // after this mdat atom there may be other atoms
        if(getDatamode() == AUDIO_LOCALFILE){
            AUDIO_INFO("Content-Length: %u", m_contentlength);
        }
        m_controlCounter = M4A_OKAY; // that's all
        return 0;
    }
    // this section should never be reached
    log_e("error");
    return 0;
}
//---------------------------------------------------------------------------------------------------------------------
int Audio::read_OGG_Header(uint8_t *data, size_t len){
    static size_t retvalue = 0;
    static size_t pageLen = 0;
    static bool   f_firstPacket = false;

    if(retvalue) {
        if(retvalue > len) { // if returnvalue > bufferfillsize
            if(len > InBuff.getMaxBlockSize()) len = InBuff.getMaxBlockSize();
            retvalue -= len; // and wait for more bufferdata
            return len;
        }
        else {
            size_t tmp = retvalue;
            retvalue = 0;
            return tmp;
        }
        return 0;
    }
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if(m_controlCounter == OGG_BEGIN) {  // init
        retvalue = 0;
        m_audioDataStart = 0;
        f_firstPacket = true;
        m_controlCounter = OGG_MAGIC;
        if(getDatamode() == AUDIO_LOCALFILE){
            m_contentlength = getFileSize();
            AUDIO_INFO("Content-Length: %u", m_contentlength);
        }
        return 0;
    }
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if(m_controlCounter == OGG_MAGIC) { /* check MAGIC STRING */
        if(specialIndexOf(data, "OggS", 10) != 0) {
            log_e("Magic String 'OggS' not found in header");
            stopSong();
            return -1;
        }
        m_controlCounter = OGG_HEADER;
        retvalue = 4;
        return 0;
    }
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if(m_controlCounter == OGG_HEADER) { /* check OGG PAGE HEADER */
        uint8_t i = 0;
        uint8_t ssv = *(data + i);                  // stream_structure_version
        (void)ssv;
        i++;
        uint8_t htf = *(data + i);                  // header_type_flag
        (void)htf;
        i++;
        uint32_t tmp = bigEndian(data + i, 4);      // absolute granule position
        uint64_t agp = (uint64_t) tmp << 32;
        i += 4;
        agp += bigEndian(data + i, 4);
        i += 4;
        uint32_t ssnr = bigEndian(data + i, 4);     // stream serial number
        (void)ssnr;
        i += 4;
        uint32_t psnr = bigEndian(data + i, 4);     // page sequence no
        (void)psnr;
        i += 4;
        uint32_t pchk = bigEndian(data + i, 4);     // page checksum
        (void)pchk;
        i += 4;
        uint8_t psegm = *(data + i);
        i++;
        uint8_t psegmBuff[256];
        pageLen = 0;
        for(uint8_t j = 0; j < psegm; j++){
            psegmBuff[j] = *(data + i);
            pageLen += psegmBuff[j];
            i++;
        }
        retvalue = i;
        if(agp == 0){
            if(f_firstPacket == true){
                f_firstPacket = false;
                m_controlCounter = OGG_FIRST; // ogg first pages
            }
            else{
                retvalue += pageLen;
                m_controlCounter = OGG_MAGIC;
            }
        }
        else{
            if(m_codec == CODEC_OGG_FLAC){
                m_controlCounter = OGG_AMRDY;
            }
            else {
                AUDIO_INFO("unknown format");
                stopSong();
                return -1;
            }
        }
        return 0;
    }
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if(m_controlCounter == OGG_FIRST) { /* check OGG FIRST PAGES (has no streaming content) */
        uint8_t i = 0;
        uint8_t obp = *(data + i);                  // oneBytePacket shold be 0x7F
        (void)obp;
        i++;
        if(specialIndexOf(data + i, "FLAC", 10) == 0){
        }
        else{
            log_e("ogg/flac support only"); // ogg/vorbis or ogg//opus not supported yet
            stopSong();
            return -1;
        }
        i += 4;
        uint8_t major_vers = *(data + i);
        (void)major_vers;
        i++;
        uint8_t minor_vers = *(data + i);
        (void)minor_vers;
        i++;
        uint16_t nonah = bigEndian(data + i, 2); // number of non audio headers (0x00 = unknown)
        (void)nonah;
        i += 2;
        if(specialIndexOf(data + i, "fLaC", 10) == 0){
            m_codec = CODEC_OGG_FLAC;
        }
        i += 4;
        // STREAMINFO metadata block begins
        uint32_t mblen = bigEndian(data + i, 4);
        (void)mblen;
        i += 4; // skip metadata block header + length
        i += 2; // skip minimun block size
        m_flacMaxBlockSize = bigEndian(data + i, 2);
        i += 2;
        vTaskDelay(2);
        AUDIO_INFO("FLAC maxBlockSize: %u", m_flacMaxBlockSize);
        i += 3; // skip minimun frame size
        vTaskDelay(2);
        m_flacMaxFrameSize = bigEndian(data + i, 3);
        i += 3;
        if(m_flacMaxFrameSize){
            AUDIO_INFO("FLAC maxFrameSize: %u", m_flacMaxFrameSize);
        }
        else {
            AUDIO_INFO("FLAC maxFrameSize: N/A");
        }
        if(m_flacMaxFrameSize > InBuff.getMaxBlockSize()) {
            log_e("FLAC maxFrameSize too large!");
            stopSong();
            return -1;
        }
        vTaskDelay(2);
        uint32_t nextval = bigEndian(data + i, 3);
        i += 3;
        m_flacSampleRate = nextval >> 4;
        AUDIO_INFO("FLAC sampleRate: %u", m_flacSampleRate);
        vTaskDelay(2);
        m_flacNumChannels = ((nextval & 0x06) >> 1) + 1;
        AUDIO_INFO("FLAC numChannels: %u", m_flacNumChannels);
        if(m_flacNumChannels != 1 && m_flacNumChannels != 2){
            vTaskDelay(2);
            AUDIO_INFO("numChannels must be 1 or 2");
            stopSong();
            return -1;
        }
        vTaskDelay(2);
        uint8_t bps = (nextval & 0x01) << 4;
        bps += (*(data +i) >> 4) + 1;
        i++;
        m_flacBitsPerSample = bps;
        if((bps != 8) && (bps != 16)){
            log_e("bits per sample must be 8 or 16, is %i", bps);
            stopSong();
            return -1;
        }
        AUDIO_INFO("FLAC bitsPerSample: %u", m_flacBitsPerSample);
        m_flacTotalSamplesInStream = bigEndian(data + i, 4);
        i++;
        if(m_flacTotalSamplesInStream) {
            AUDIO_INFO("total samples in stream: %u", m_flacTotalSamplesInStream);
        }
        else {
            AUDIO_INFO("total samples in stream: N/A");
        }
        if(bps != 0 && m_flacTotalSamplesInStream) {
            AUDIO_INFO("audio file duration: %u seconds", m_flacTotalSamplesInStream / m_flacSampleRate);
        }
        m_controlCounter = OGG_MAGIC;
        retvalue = pageLen;
        return 0;
    }
    if(m_controlCounter == OGG_AMRDY){ // ogg almost ready
        if(!psramFound()){
            AUDIO_INFO("FLAC works only with PSRAM!");
            m_f_running = false; stopSong();
            return -1;
        }
        if(!FLACDecoder_AllocateBuffers()) {m_f_running = false; stopSong(); return -1;}
        InBuff.changeMaxBlockSize(m_frameSizeFLAC);
        AUDIO_INFO("FLACDecoder has been initialized, free Heap: %u bytes", ESP.getFreeHeap());

        m_controlCounter = OGG_OKAY; // 100
        retvalue = 0;
        return 0;
    }
    return 0;
}
//---------------------------------------------------------------------------------------------------------------------
size_t Audio::process_m3u8_ID3_Header(uint8_t* packet){
    uint8_t         ID3version;
    size_t          id3Size;
    bool            m_f_unsync = false, m_f_exthdr = false;
    uint64_t        current_timestamp = 0;

    (void) m_f_unsync;         // suppress -Wunused-variable
    (void) current_timestamp;  // suppress -Wunused-variable

    if(specialIndexOf(packet, "ID3", 4) != 0) { // ID3 not found
        if(m_f_Log) log_i("m3u8 file has no mp3 tag");
        return 0; // error, no ID3 signature found
    }
    ID3version = *(packet + 3);
    switch(ID3version){
            case 2:
                m_f_unsync = (*(packet + 5) & 0x80);
                m_f_exthdr = false;
                break;
            case 3:
            case 4:
                m_f_unsync = (*(packet + 5) & 0x80); // bit7
                m_f_exthdr = (*(packet + 5) & 0x40); // bit6 extended header
                break;
    };
    id3Size = bigEndian(&packet[6], 4, 7); //  ID3v2 size  4 * %0xxxxxxx (shift left seven times!!)
    id3Size += 10;
    if(m_f_Log) log_i("ID3 framesSize: %i", id3Size);
    if(m_f_Log) log_i("ID3 version: 2.%i", ID3version);

    if(m_f_exthdr) {
        log_e("ID3 extended header in m3u8 files not supported");
        return 0;
    }
    if(m_f_Log) log_i("ID3 normal frames");

    if(specialIndexOf(&packet[10], "PRIV", 5) != 0) { // tag PRIV not found
        log_e("tag PRIV in m3u8 Id3 Header not found");
        return 0;
    }
    // if tag PRIV exists assume content is "com.apple.streaming.transportStreamTimestamp"
    // a time stamp is expected in the header.

    current_timestamp = (double)bigEndian(&packet[69], 4) / 90000; // seconds

    return id3Size;
}
//---------------------------------------------------------------------------------------------------------------------
uint32_t Audio::stopSong() {
    uint32_t pos = 0;
    if(m_f_running) {
        m_f_running = false;
        if(getDatamode() == AUDIO_LOCALFILE){
            m_streamType = ST_NONE;
            pos = getFilePos() - inBufferFilled();
            audiofile.close();
            AUDIO_INFO("Closing audio file");
        }
    }
    if(audiofile){
        // added this before putting 'm_f_localfile = false' in stopSong(); shoulf never occur....
        audiofile.close();
        AUDIO_INFO("Closing audio file");
        log_w("Closing audio file");  // for debug
    }
    memset(m_outBuff, 0, sizeof(m_outBuff));     //Clear OutputBuffer
    i2s_zero_dma_buffer((i2s_port_t) m_i2s_num);
    return pos;
}
//---------------------------------------------------------------------------------------------------------------------
void Audio::playI2Sremains() { // returns true if all dma_buffs flushed
    if(!getSampleRate()) setSampleRate(96000);
    if(!getChannels()) setChannels(2);
    if(getBitsPerSample() > 8) memset(m_outBuff,   0, sizeof(m_outBuff));     //Clear OutputBuffer (signed)
    else                       memset(m_outBuff, 128, sizeof(m_outBuff));     //Clear OutputBuffer (unsigned, PCM 8u)

    m_validSamples = m_i2s_config.dma_buf_len * m_i2s_config.dma_buf_count;
    while(m_validSamples) {
        playChunk();
    }
    i2s_zero_dma_buffer((i2s_port_t) m_i2s_num);
    return;
}
//---------------------------------------------------------------------------------------------------------------------
bool Audio::pauseResume() {
    bool retVal = false;
    if(getDatamode() == AUDIO_LOCALFILE || m_streamType == ST_WEBSTREAM) {
        m_f_running = !m_f_running;
        retVal = true;
        if(!m_f_running) {
            memset(m_outBuff, 0, sizeof(m_outBuff));               //Clear OutputBuffer
            i2s_zero_dma_buffer((i2s_port_t) m_i2s_num);
        }
    }
    return retVal;
}
//---------------------------------------------------------------------------------------------------------------------
bool Audio::playChunk() {
    // If we've got data, try and pump it out..
    int16_t sample[2];
    if(getBitsPerSample() == 8) {
        if(getChannels() == 1) {
            while(m_validSamples) {
                uint8_t x =  m_outBuff[m_curSample] & 0x00FF;
                uint8_t y = (m_outBuff[m_curSample] & 0xFF00) >> 8;
                sample[LEFTCHANNEL]  = x;
                sample[RIGHTCHANNEL] = x;
                while(1) {
                    if(playSample(sample)) break;
                } // Can't send?
                sample[LEFTCHANNEL]  = y;
                sample[RIGHTCHANNEL] = y;
                while(1) {
                    if(playSample(sample)) break;
                } // Can't send?
                m_validSamples--;
                m_curSample++;
            }
        }
        if(getChannels() == 2) {
            while(m_validSamples) {
                uint8_t x =  m_outBuff[m_curSample] & 0x00FF;
                uint8_t y = (m_outBuff[m_curSample] & 0xFF00) >> 8;
                if(!m_f_forceMono) { // stereo mode
                    sample[LEFTCHANNEL]  = x;
                    sample[RIGHTCHANNEL] = y;
                }
                else { // force mono
                    uint8_t xy = (x + y) / 2;
                    sample[LEFTCHANNEL]  = xy;
                    sample[RIGHTCHANNEL] = xy;
                }

                while(1) {
                    if(playSample(sample)) break;
                } // Can't send?
                m_validSamples--;
                m_curSample++;
            }
        }
        m_curSample = 0;
        return true;
    }
    if(getBitsPerSample() == 16) {
        if(getChannels() == 1) {
            while(m_validSamples) {
                sample[LEFTCHANNEL]  = m_outBuff[m_curSample];
                sample[RIGHTCHANNEL] = m_outBuff[m_curSample];
                if(!playSample(sample)) {
                    log_e("can't send");
                    return false;
                } // Can't send
                m_validSamples--;
                m_curSample++;
            }
        }
        if(getChannels() == 2) {
            m_curSample = 0;
            while(m_validSamples) {
                if(!m_f_forceMono) { // stereo mode
                    sample[LEFTCHANNEL]  = m_outBuff[m_curSample * 2];
                    sample[RIGHTCHANNEL] = m_outBuff[m_curSample * 2 + 1];
                }
                else { // mono mode, #100
                    int16_t xy = (m_outBuff[m_curSample * 2] + m_outBuff[m_curSample * 2 + 1]) / 2;
                    sample[LEFTCHANNEL] = xy;
                    sample[RIGHTCHANNEL] = xy;
                }
                playSample(sample);
                m_validSamples--;
                m_curSample++;
            }
        }
        m_curSample = 0;
        return true;
    }
    log_e("BitsPer Sample must be 8 or 16!");
    m_validSamples = 0;
    stopSong();
    return false;
}
//---------------------------------------------------------------------------------------------------------------------
void Audio::loop() {

    if(!m_f_running) return;

    if(m_playlistFormat != FORMAT_M3U8){ // normal process
        switch(getDatamode()){
            case AUDIO_LOCALFILE:
                processLocalFile();
                break;
            case HTTP_RESPONSE_HEADER:
                parseHttpResponseHeader();
                break;
            case AUDIO_PLAYLISTINIT:
                readPlayListData();
                break;
            case AUDIO_PLAYLISTDATA:
                if(m_playlistFormat == FORMAT_M3U)  connecttohost(parsePlaylist_M3U());
                if(m_playlistFormat == FORMAT_PLS)  connecttohost(parsePlaylist_PLS());
                if(m_playlistFormat == FORMAT_ASX)  connecttohost(parsePlaylist_ASX());
                break;
            case AUDIO_DATA:
                if(m_streamType == ST_WEBSTREAM) processWebStream();
                if(m_streamType == ST_WEBFILE)   processWebFile();
                break;
        }
    }
    else { // m3u8 datastream only
        static bool f_noNewHost = false;
        static int32_t remaintime, timestamp1, timestamp2; // m3u8 time management
        const char* host;

        switch(getDatamode()){
            case HTTP_RESPONSE_HEADER:
                playAudioData(); // fill I2S DMA buffer
                parseHttpResponseHeader();
                m_codec = CODEC_AAC;
                break;
            case AUDIO_PLAYLISTINIT:
                readPlayListData();
                break;
            case AUDIO_PLAYLISTDATA:
                host = parsePlaylist_M3U8();
                m_f_m3u8data = true;
                if(host){
                    f_noNewHost = false;
                    timestamp1 = millis();
                    httpPrint(host);
                }
                else {
                    f_noNewHost = true;
                    timestamp2 = millis() + remaintime;
                    setDatamode(AUDIO_DATA); //fake datamode, we have no new audiosequence yet, so let audio run
                }
                break;
            case AUDIO_DATA:
                if(m_f_ts) processWebStreamTS();  // aac or aacp with ts packets
                else       processWebStreamHLS(); // aac or aacp normal stream
                if(f_noNewHost){
                    m_f_continue = false;
                    if(timestamp2 < millis()) {
                        httpPrint(m_lastHost);
                        remaintime = 1000;
                    }
                }
                else{
                    if(m_f_continue){ // processWebStream() needs more data
                        remaintime = (int32_t)(m_m3u8_targetDuration * 1000) - (millis() - timestamp1);
                    //    if(m_m3u8_targetDuration < 10) remaintime += 1000;
                        m_f_continue = false;
                        setDatamode(AUDIO_PLAYLISTDATA);
                    }
                }
                break;
        }
    }
}
//---------------------------------------------------------------------------------------------------------------------
bool Audio::readPlayListData() {

    if(getDatamode() != AUDIO_PLAYLISTINIT) return false;
    if(_client->available() == 0) return false;

    uint32_t chunksize = 0; uint8_t readedBytes = 0;
    if(m_f_chunked) chunksize = chunkedDataTransfer(&readedBytes);

    // reads the content of the playlist and stores it in the vector m_contentlength
    // m_contentlength is a table of pointers to the lines
    char pl[512]; // playlistLine
    uint32_t ctl  = 0;
    int lines = 0;
    // delete all memory in m_playlistContent
    if(m_playlistFormat == FORMAT_M3U8 && !psramFound()){log_e("m3u8 playlists requires PSRAM enabled!");}
    vector_clear_and_shrink(m_playlistContent);
    while(true){  // outer while

        uint32_t ctime = millis();
        uint32_t timeout = 2000; // ms

        while(true) { // inner while
            uint16_t pos = 0;
            while(_client->available()){ // super inner while :-))
                pl[pos] = _client->read();
                ctl++;
                if(pl[pos] == '\n') {pl[pos] = '\0'; pos++; break;}
            //    if(pl[pos] == '&' ) {pl[pos] = '\0'; pos++; break;}
                if(pl[pos] == '\r') {pl[pos] = '\0'; pos++; continue;;}
                pos++;
                if(pos == 511){ pos--; continue;}
                if(pos == 510) {pl[pos] = '\0';}
                if(ctl == chunksize) {pl[pos] = '\0'; break;}
                if(ctl == m_contentlength) {pl[pos] = '\0'; break;}
            }
            if(ctl == chunksize) break;
            if(ctl == m_contentlength) break;
            if(pos) {pl[pos] = '\0'; break;}

            if(ctime + timeout < millis()) {
                log_e("timeout");
                for(int i = 0; i<m_playlistContent.size(); i++) log_e("pl%i = %s", i, m_playlistContent[i]);
                goto exit;}
        } // inner while

        if(startsWith(pl, "<!DOCTYPE")) {AUDIO_INFO("url is a webpage!"); goto exit;}
        if(startsWith(pl, "<html"))     {AUDIO_INFO("url is a webpage!"); goto exit;}
        if(strlen(pl) > 0) m_playlistContent.push_back(strdup((const char*)pl));
        if(m_playlistContent.size() == 100){
            if(m_f_Log) log_i("the maximum number of lines in the playlist has been reached");
            break;
        }
        // termination conditions
        // 1. The http response header returns a value for contentLength -> read chars until contentLength is reached
        // 2. no contentLength, but Transfer-Encoding:chunked -> compute chunksize and read until chunksize is reached
        // 3. no chunksize and no contentlengt, but Connection: close -> read all available chars
        if(ctl == m_contentlength){while(_client->available()) _client->read(); break;} // read '\n\n' if exists
        if(ctl == chunksize)      {while(_client->available()) _client->read(); break;}
        if(!_client->connected() && _client->available() == 0) break;

    } // outer while
    lines = m_playlistContent.size();
    for (int i = 0; i < lines ; i++) { // print all string in first vector of 'arr'
        if(m_f_Log) log_i("pl=%i \"%s\"", i, m_playlistContent[i]);
    }
    setDatamode(AUDIO_PLAYLISTDATA);
    return true;

    exit:
        vector_clear_and_shrink(m_playlistContent);
        m_f_running = false;
        setDatamode(AUDIO_NONE);
    return false;
}
//----------------------------------------------------------------------------------------------------------------------
const char* Audio::parsePlaylist_M3U(){
    uint8_t lines = m_playlistContent.size();
    int pos = 0;
    char* host = nullptr;

    for(int i= 0; i < lines; i++){
        if(indexOf(m_playlistContent[i], "#EXTINF:") >= 0) {            // Info?
            pos = indexOf(m_playlistContent[i], ",");                   // Comma in this line?
            if(pos > 0) {
                // Show artist and title if present in metadata
                AUDIO_INFO(m_playlistContent[i] + pos + 1);
            }
            continue;
        }
        if(startsWith(m_playlistContent[i], "#")) {                     // Commentline?
            continue;
        }

        pos = indexOf(m_playlistContent[i], "http://:@", 0);            // ":@"??  remove that!
        if(pos >= 0) {
            AUDIO_INFO("Entry in playlist found: %s", (m_playlistContent[i] + pos + 9));
            host = m_playlistContent[i] + pos + 9;
            break;
        }
        // AUDIO_INFO("Entry in playlist found: %s", pl);
        pos = indexOf(m_playlistContent[i], "http", 0);                 // Search for "http"
        if(pos >= 0) {                                                  // Does URL contain "http://"?
    //    log_e("%s pos=%i", m_playlistContent[i], pos);
            host = m_playlistContent[i] + pos;                        // Yes, set new host
            break;
        }
    }
    vector_clear_and_shrink(m_playlistContent);
    return host;
}
//----------------------------------------------------------------------------------------------------------------------
const char* Audio::parsePlaylist_PLS(){
    uint8_t lines = m_playlistContent.size();
    int pos = 0;
    char* host = nullptr;

    for(int i= 0; i < lines; i++){
        if(i == 0){
            if(strlen(m_playlistContent[0]) == 0) goto exit;            // empty line
            if(strcmp(m_playlistContent[0] , "[playlist]") != 0){       // first entry in valid pls
                setDatamode(HTTP_RESPONSE_HEADER);                      // pls is not valid
                AUDIO_INFO("pls is not valid, switch to HTTP_RESPONSE_HEADER");
                goto exit;
            }
            continue;
        }
        if(startsWith(m_playlistContent[i], "File1")) {
            if(host) continue;                                          // we have already a url
            pos = indexOf(m_playlistContent[i], "http", 0);             // File1=http://streamplus30.leonex.de:14840/;
            if(pos >= 0) {                                              // yes, URL contains "http"?
                host = m_playlistContent[i] + pos;                      // Now we have an URL for a stream in host.
            }
            continue;
        }
        if(startsWith(m_playlistContent[i], "Title1")) {                // Title1=Antenne Tirol
            const char* plsStationName = (m_playlistContent[i] + 7);
            if(audio_showstation) audio_showstation(plsStationName);
            AUDIO_INFO("StationName: \"%s\"", plsStationName);
            continue;
        }
        if(startsWith(m_playlistContent[i], "Length1")){
            continue;
        }
        if(indexOf(m_playlistContent[i], "Invalid username") >= 0){     // Unable to access account:
            goto exit;                                                  // Invalid username or password
        }
    }
    return host;

exit:
    m_f_running = false;
    stopSong();
    vector_clear_and_shrink(m_playlistContent);
    setDatamode(AUDIO_NONE);
    return nullptr;
}
//----------------------------------------------------------------------------------------------------------------------
const char* Audio::parsePlaylist_ASX(){                             // Advanced Stream Redirector
    uint8_t lines = m_playlistContent.size();
    bool f_entry = false;
    int pos = 0;
    char* host = nullptr;

    for(int i= 0; i < lines; i++){
        int p1 = indexOf(m_playlistContent[i], "<", 0);
        int p2 = indexOf(m_playlistContent[i], ">", 1);
        if(p1 >= 0 && p2 > p1){                                     // #196 set all between "< ...> to lowercase
            for(uint8_t j = p1; j < p2; j++){
                m_playlistContent[i][j] = toLowerCase(m_playlistContent[i][j]);
            }
        }
        if(indexOf(m_playlistContent[i], "<entry>") >= 0) f_entry = true; // found entry tag (returns -1 if not found)
        if(f_entry) {
            if(indexOf(m_playlistContent[i], "ref href") > 0) {     //  <ref href="http://87.98.217.63:24112/stream" />
                pos = indexOf(m_playlistContent[i], "http", 0);
                if(pos > 0) {
                    host = (m_playlistContent[i] + pos);            // http://87.98.217.63:24112/stream" />
                    int pos1 = indexOf(host, "\"", 0);              // http://87.98.217.63:24112/stream
                    if(pos1 > 0) host[pos1] = '\0';                 // Now we have an URL for a stream in host.
                }
            }
        }
        pos = indexOf(m_playlistContent[i], "<title>", 0);
        if(pos >= 0) {
            char* plsStationName = (m_playlistContent[i] + pos + 7);            // remove <Title>
            pos = indexOf(plsStationName, "</", 0);
            if(pos >= 0){
                *(plsStationName +pos) = 0;                                     // remove </Title>
            }
            if(audio_showstation) audio_showstation(plsStationName);
            AUDIO_INFO("StationName: \"%s\"", plsStationName);
        }

        if(indexOf(m_playlistContent[i], "http") == 0 && !f_entry) {            //url only in asx
            host = m_playlistContent[i];
        }
    }
    return host;
}
//----------------------------------------------------------------------------------------------------------------------
const char* Audio::parsePlaylist_M3U8(){
    uint8_t lines = m_playlistContent.size();
    bool f_begin = false;
    uint8_t occurence = 0;
    if(lines){
        for(int i= 0; i < lines; i++){
            if(strlen(m_playlistContent[i]) == 0) continue;                    // empty line
            if(startsWith(m_playlistContent[i], "#EXTM3U")){                  // what we expected
                f_begin      = true;
                continue;
            }
            if(!f_begin) continue;

            // example: redirection
            // #EXTM3U
            // #EXT-X-STREAM-INF:BANDWIDTH=22050,CODECS="mp4a.40.2"
            // http://ample.revma.ihrhls.com/zc7729/63_sdtszizjcjbz02/playlist.m3u8
            if(startsWith(m_playlistContent[i],"#EXT-X-STREAM-INF:")){
                if(occurence > 0) break; // no more than one #EXT-X-STREAM-INF: (can have different BANDWIDTH)
                occurence++;
                if((!endsWith(m_playlistContent[i+1], "m3u8" ) && indexOf(m_playlistContent[i+1], "m3u8?") == -1)){
                    // we have a new m3u8 playlist, skip to next line
                    int pos = indexOf(m_playlistContent[i], "CODECS=\"mp4a", 18);
                    // 'mp4a.40.01' AAC Main
                    // 'mp4a.40.02' AAC LC (Low Complexity)
                    // 'mp4a.40.03' AAC SSR (Scalable Sampling Rate) ??
                    // 'mp4a.40.03' AAC LTP (Long Term Prediction) ??
                    // 'mp4a.40.03' SBR (Spectral Band Replication)
                    if(pos < 0){ // not found
                        int pos1 = indexOf(m_playlistContent[i], "CODECS=", 18);
                        if(pos1 < 0) pos1 = 0;
                        log_e("codec %s in m3u8 playlist not supported", m_playlistContent[i] + pos1);
                        goto exit;
                    }
                }
                i++;                                                    // next line
                if(i == lines) continue; // and exit for()

                char* tmp = nullptr;
                if(!startsWith(m_playlistContent[i], "http")){
                  //http://livees.com/prog_index.m3u8 and prog_index48347.aac --> http://livees.com/prog_index48347.aac
                  //http://livees.com/prog_index.m3u8 and chunklist022.m3u8   --> http://livees.com/chunklist022.m3u8
                    tmp = (char*)malloc(strlen(m_lastHost)+ strlen(m_playlistContent[i]));
                    strcpy(tmp, m_lastHost);
                    int idx = lastIndexOf(tmp, "/");
                    strcpy(tmp + idx + 1, m_playlistContent[i]);
                }
                else{
                    tmp = strdup(m_playlistContent[i]);
                }
                if(m_playlistContent[i]){free(m_playlistContent[i]); m_playlistContent[i] = NULL;}
                m_playlistContent[i] = strdup(tmp);
                strcpy(m_lastHost, tmp);
                if(tmp){free(tmp); tmp = NULL;}
                if(m_f_Log) log_i("redirect %s", m_playlistContent[i]);
                return m_playlistContent[i];                            // it's a redirection, a new m3u8 playlist
            }

            // example: audio chunks
            // #EXTM3U
            // #EXT-X-TARGETDURATION:10
            // #EXT-X-MEDIA-SEQUENCE:163374040
            // #EXT-X-DISCONTINUITY
            // #EXTINF:10,title="text=\"Spot Block End\" amgTrackId=\"9876543\"",artist=" ",url="length=\"00:00:00\""
            // http://n3fa-e2.revma.ihrhls.com/zc7729/63_sdtszizjcjbz02/main/163374038.aac
            // #EXTINF:10,title="text=\"Spot Block End\" amgTrackId=\"9876543\"",artist=" ",url="length=\"00:00:00\""
            // http://n3fa-e2.revma.ihrhls.com/zc7729/63_sdtszizjcjbz02/main/163374039.aac
            if(startsWith(m_playlistContent[i], "#EXT-X-MEDIA-SEQUENCE:")){
                // do nothing, because MEDIA-SECUENCE is not set sometimes
            }
            static uint16_t targetDuration = 0;
            if(startsWith(m_playlistContent[i], "#EXT-X-TARGETDURATION:")) {
                targetDuration = atoi(m_playlistContent[i] + 22);
            }
            if(targetDuration) m_m3u8_targetDuration = targetDuration;
            if(m_f_Log) log_i("m_m3u8_targetDuration %d", m_m3u8_targetDuration);

            if(startsWith(m_playlistContent[i],"#EXTINF")) {
                if(STfromEXTINF(m_playlistContent[i])) showstreamtitle(chbuf);
                i++;
                if(i == lines) continue; // and exit for()

                char* tmp = nullptr;
                if(!startsWith(m_playlistContent[i], "http")){
                    //http://livees.com/prog_index.m3u8 and prog_index48347.aac --> http://livees.com/prog_index48347.aac
                    tmp = (char*)malloc(strlen(m_lastHost)+ strlen(m_playlistContent[i]));
                    strcpy(tmp, m_lastHost);
                    int idx = lastIndexOf(tmp, "/");
                    strcpy(tmp + idx + 1, m_playlistContent[i]);
                }
                else{
                    tmp = strdup(m_playlistContent[i]);
                }

                uint32_t hash = simpleHash(tmp);
                if(m_hashQueue.size() == 0){
                    m_hashQueue.insert(m_hashQueue.begin(), hash);
                    m_playlistURL.insert(m_playlistURL.begin(), strdup(tmp));
                }
                else{
                    bool known = false;
                    for(int i = 0; i< m_hashQueue.size(); i++){
                        if(hash == m_hashQueue[i]){
                            if(m_f_Log) log_i("file already known %s", tmp);
                            known = true;
                        }
                    }
                    if(!known){
                        m_hashQueue.insert(m_hashQueue.begin(), hash);
                        m_playlistURL.insert(m_playlistURL.begin(), strdup(tmp));
                    }
                }

                if(m_hashQueue.size() > 20)  m_hashQueue.pop_back();

                if(tmp){free(tmp); tmp = NULL;}

                if(m_playlistURL.size() == 20){
                    ESP_LOGD("", "can't stuff anymore");
                    break;
                }
                continue;
            }
        }
        vector_clear_and_shrink(m_playlistContent); //clear after reading everything, m_playlistContent.size is now 0
    }

    if(m_playlistURL.size() > 0){
        if(m_playlistBuff) {free(m_playlistBuff); m_playlistBuff = NULL;}

        if(m_playlistURL[m_playlistURL.size() -1]) {
                m_playlistBuff = strdup(m_playlistURL[m_playlistURL.size() -1]);
                free( m_playlistURL[m_playlistURL.size() -1]);
                m_playlistURL[m_playlistURL.size() -1] = NULL;
                m_playlistURL.pop_back();
                m_playlistURL.shrink_to_fit();
        }
        if(m_f_Log) log_i("now playing %s", m_playlistBuff);
        return m_playlistBuff;
    }
    else{
        return NULL;
    }
exit:
    stopSong();
    return NULL;
}
//---------------------------------------------------------------------------------------------------------------------
bool Audio::STfromEXTINF(char* str){
    // the result is copied in chbuf!!
    // extraxt StreamTitle from m3u #EXTINF line to icy-format
    // orig: #EXTINF:10,title="text="TitleName",artist="ArtistName"
    // conv: StreamTitle=TitleName - ArtistName
    // orig: #EXTINF:10,title="text=\"Spot Block End\" amgTrackId=\"9876543\"",artist=" ",url="length=\"00:00:00\""
    // conv: StreamTitle=text=\"Spot Block End\" amgTrackId=\"9876543\" -

    int t1, t2, t3, n0 = 0, n1 = 0, n2 = 0;

    t1 = indexOf(str, "title", 0);
    if(t1 > 0){
        strcpy(chbuf, "StreamTitle="); n0 = 12;
        t2 = t1 + 7; // title="
        t3 = indexOf(str, "\"", t2);
        while(str[t3 - 1] == '\\'){
            t3 = indexOf(str, "\"", t3 + 1);
        }
        if(t2 < 0 || t2 > t3) return false;
        n1 = t3 - t2;
        strncpy(chbuf + n0, str + t2, n1);
        chbuf[n1] = '\0';
    }

    t1 = indexOf(str, "artist", 0);
    if(t1 > 0){
        strcpy(chbuf + n0 + n1, " - ");   n1 += 3;
        t2 = indexOf(str, "=\"", t1); t2 += 2;
        t3 = indexOf(str, "\"", t2);
        if(t2 < 0 || t2 > t3) return false;
        n2 = t3 - t2;
        strncpy(chbuf + n0 + n1, str + t2, n2);
        chbuf[n0 + n1 + n2] = '\0';
        chbuf[n2] = '\0';
    }
    return true;
}
//---------------------------------------------------------------------------------------------------------------------
void Audio::processLocalFile() {

    if(!(audiofile && m_f_running && getDatamode() == AUDIO_LOCALFILE)) return; // guard

    const uint32_t  maxFrameSize = InBuff.getMaxBlockSize();    // every mp3/aac frame is not bigger
    static bool f_stream;
    static bool f_fileDataComplete;
    static uint32_t byteCounter;                                // count received data

    if(m_f_firstCall) {  // runs only one time per connection, prepare for start
        m_f_firstCall = false;
        f_stream = false;
        f_fileDataComplete = false;
        byteCounter = 0;
        return;
    }

    uint32_t availableBytes = maxFrameSize * 4;
    availableBytes = min(availableBytes, InBuff.writeSpace());
    availableBytes = min(availableBytes, audiofile.size() - byteCounter);
    if(m_contentlength){
        if(m_contentlength > getFilePos()) availableBytes = min(availableBytes, m_contentlength - getFilePos());
    }
    if(m_audioDataSize){
        availableBytes = min(availableBytes, m_audioDataSize + m_audioDataStart - byteCounter);
    }

    int32_t bytesAddedToBuffer = audiofile.read(InBuff.getWritePtr(), availableBytes);

    if(bytesAddedToBuffer > 0) {
        byteCounter += bytesAddedToBuffer;  // Pull request #42
        InBuff.bytesWritten(bytesAddedToBuffer);
    }

    if(!f_stream && m_controlCounter == 100) {
        f_stream = true;
        AUDIO_INFO("stream ready");
        if(m_resumeFilePos){
            if(m_resumeFilePos < m_audioDataStart) m_resumeFilePos = m_audioDataStart;
            if(m_avr_bitrate) m_audioCurrentTime = ((m_resumeFilePos - m_audioDataStart) / m_avr_bitrate) * 8;
            audiofile.seek(m_resumeFilePos);
            InBuff.resetBuffer();
            if(m_f_Log) log_i("m_resumeFilePos %i", m_resumeFilePos);
        }
    }

    if(InBuff.bufferFilled() > maxFrameSize && !f_stream) {  // waiting for buffer filled
        // read the file header first - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        if(m_controlCounter != 100){
            InBuff.bytesWasRead(readAudioHeader(InBuff.bufferFilled()));
            return;
        }
        if(m_resumeFilePos){
            if(m_resumeFilePos < m_audioDataStart) m_resumeFilePos = m_audioDataStart;
            if(m_avr_bitrate) m_audioCurrentTime = ((m_resumeFilePos - m_audioDataStart) / m_avr_bitrate) * 8;
            audiofile.seek(m_resumeFilePos);
            InBuff.resetBuffer();
            if(m_f_Log) log_i("m_resumeFilePos %i", m_resumeFilePos);
        }
        f_stream = true;  // ready to play the audio data
    }

    // end of file reached? - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if(f_fileDataComplete && InBuff.bufferFilled() < InBuff.getMaxBlockSize()){
        if(InBuff.bufferFilled()){
            if(!readID3V1Tag()){
                int bytesDecoded = sendBytes(InBuff.getReadPtr(), InBuff.bufferFilled());
                if(bytesDecoded > 2){InBuff.bytesWasRead(bytesDecoded); return;}
            }
        }
        playI2Sremains();

        if(m_f_loop  && f_stream){  //eof
            AUDIO_INFO("loop from: %u to: %u", getFilePos(), m_audioDataStart); //TEST loop
            setFilePos(m_audioDataStart);
            if(m_codec == CODEC_FLAC) FLACDecoderReset();
            /*
                The current time of the loop mode is not reset,
                which will cause the total audio duration to be exceeded.
                For example: current time   ====progress bar====>  total audio duration
                                3:43        ====================>        3:33
            */
            m_audioCurrentTime = 0;
            return;
        } //TEST loop

#ifdef SDFATFS_USED
        audiofile.getName(chbuf, sizeof(chbuf));
        char *afn =strdup(chbuf);
#else
        char *afn =strdup(audiofile.name()); // store temporary the name
#endif

        stopSong();
        if(m_codec == CODEC_MP3)   MP3Decoder_FreeBuffers();
        if(m_codec == CODEC_AAC)   AACDecoder_FreeBuffers();
        if(m_codec == CODEC_M4A)   AACDecoder_FreeBuffers();
        if(m_codec == CODEC_FLAC) FLACDecoder_FreeBuffers();
        AUDIO_INFO("End of file \"%s\"", afn);
        if(audio_eof_mp3) audio_eof_mp3(afn);
        if(afn) {free(afn); afn = NULL;}
        return;
    }

    if(byteCounter == audiofile.size())                  {f_fileDataComplete = true;}
    if(byteCounter == m_audioDataSize + m_audioDataStart){f_fileDataComplete = true;}

    // play audio data - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if(f_stream){
        // static uint8_t cnt = 0;
        // uint8_t compression;
        // if(m_codec == CODEC_WAV)  compression = 1;
        // if(m_codec == CODEC_FLAC) compression = 2;
        // compression = 6;
        // cnt++;
        // if(cnt == compression){playAudioData(); cnt = 0;}
        playAudioData();
    }
    return;
}
//----------------------------------------------------------------------------------------------------------------------
void Audio::processWebStream() {

    const uint16_t  maxFrameSize = InBuff.getMaxBlockSize();    // every mp3/aac frame is not bigger
    static bool     f_tmr_1s;
    static bool     f_stream;                                   // first audio data received
    static uint8_t  cnt_slow;
    static uint32_t chunkSize;                                  // chunkcount read from stream
    static uint32_t tmr_1s;                                     // timer 1 sec
    static uint32_t loopCnt;                                    // count loops if clientbuffer is empty

    // first call, set some values to default  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if(m_f_firstCall) { // runs only ont time per connection, prepare for start
        m_f_firstCall = false;
        f_stream = false;
        cnt_slow = 0;
        chunkSize = 0;
        loopCnt = 0;
        tmr_1s = millis();
        m_metacount = m_metaint;
        readMetadata(0, true); // reset all static vars
    }

    if(getDatamode() != AUDIO_DATA) return;              // guard
    uint32_t availableBytes = _client->available();      // available from stream
    // chunked data tramsfer - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if(m_f_chunked && availableBytes){
        uint8_t readedBytes = 0;
        if(!chunkSize) chunkSize = chunkedDataTransfer(&readedBytes);
        availableBytes = min(availableBytes, chunkSize);
    }
    // we have metadata  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if(m_f_metadata && availableBytes){
        if(m_metacount == 0) {chunkSize -= readMetadata(availableBytes); return;}
        availableBytes = min(availableBytes, m_metacount);
    }
    // timer, triggers every second  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if((tmr_1s + 1000) < millis()) {
        f_tmr_1s = true;                                        // flag will be set every second for one loop only
        tmr_1s = millis();
    }
    // if the buffer is often almost empty issue a warning - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if(InBuff.bufferFilled() > maxFrameSize) {f_tmr_1s = false; cnt_slow = 0; loopCnt = 0;}
    if(f_tmr_1s){
        cnt_slow ++;
        if(cnt_slow > 50){cnt_slow = 0; AUDIO_INFO("slow stream, dropouts are possible");}
    }
    // if the buffer can't filled for several seconds try a new connection - - - - - - - - - - - - - - - - - - - - - - -
    if(f_stream && !availableBytes){
        loopCnt++;
        if(loopCnt > 200000) {              // wait several seconds
            AUDIO_INFO("Stream lost -> try new connection");
            connecttohost(m_lastHost);
            return;
        }
    }
    // buffer fill routine - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if(availableBytes) {
        availableBytes = min(availableBytes, InBuff.writeSpace());
        int16_t bytesAddedToBuffer = _client->read(InBuff.getWritePtr(), availableBytes);

        if(bytesAddedToBuffer > 0) {
            if(m_f_metadata)            m_metacount  -= bytesAddedToBuffer;
            if(m_f_chunked)             chunkSize    -= bytesAddedToBuffer;
            InBuff.bytesWritten(bytesAddedToBuffer);
        }

        if(InBuff.bufferFilled() > maxFrameSize && !f_stream) {  // waiting for buffer filled
            f_stream = true;  // ready to play the audio data
            AUDIO_INFO("stream ready");
        }
        if(!f_stream) return;
    }

    // play audio data - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if(f_stream){
        static uint8_t cnt = 0;
        cnt++;
        if(cnt == 3){playAudioData(); cnt = 0;}
    }
}
//---------------------------------------------------------------------------------------------------------------------
void Audio::processWebFile() {

    const uint32_t  maxFrameSize = InBuff.getMaxBlockSize();    // every mp3/aac frame is not bigger
    static bool     f_stream;                                   // first audio data received
    static bool     f_webFileDataComplete;                      // all file data received
    static uint32_t byteCounter;                                // count received data
    static uint32_t chunkSize;                                  // chunkcount read from stream
    static size_t   audioDataCount;                             // counts the decoded audiodata only

    // first call, set some values to default - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if(m_f_firstCall) { // runs only ont time per connection, prepare for start
        m_f_firstCall = false;
        f_webFileDataComplete = false;
        f_stream = false;
        byteCounter = 0;
        chunkSize = 0;
        audioDataCount = 0;
    }

    if(!m_contentlength && !m_f_tts) {log_e("webfile without contentlength!"); stopSong(); return;} // guard

    uint32_t availableBytes = _client->available(); // available from stream

    // chunked data tramsfer - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if(m_f_chunked){
        uint8_t readedBytes = 0;
        if(!chunkSize) chunkSize = chunkedDataTransfer(&readedBytes);
        availableBytes = min(availableBytes, chunkSize);
        if(m_f_tts) m_contentlength = chunkSize;
    }

    // if the buffer is often almost empty issue a warning  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if(!f_webFileDataComplete && f_stream){
        slowStreamDetection(InBuff.bufferFilled(), maxFrameSize);
    }

    availableBytes = min(InBuff.writeSpace(), availableBytes);
    availableBytes = min(m_contentlength - byteCounter, availableBytes);
    if(m_audioDataSize) availableBytes = min(m_audioDataSize - (byteCounter - m_audioDataStart), availableBytes);

    int16_t bytesAddedToBuffer = _client->read(InBuff.getWritePtr(), availableBytes);

    if(bytesAddedToBuffer > 0) {
        byteCounter  += bytesAddedToBuffer;  // Pull request #42
        if(m_f_chunked)             m_chunkcount   -= bytesAddedToBuffer;
        if(m_controlCounter == 100) audioDataCount += bytesAddedToBuffer;
        InBuff.bytesWritten(bytesAddedToBuffer);
    }

    if(InBuff.bufferFilled() > maxFrameSize && !f_stream) {  // waiting for buffer filled
        f_stream = true;  // ready to play the audio data
        uint16_t filltime = millis() - m_t0;
        AUDIO_INFO("stream ready\nbuffer filled in %d ms", filltime);
    }

    if(!f_stream) return;

    // we have a webfile, read the file header first - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if(m_controlCounter != 100){
        InBuff.bytesWasRead(readAudioHeader(availableBytes));
        return;
    }

    // end of webfile reached? - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if(f_webFileDataComplete && InBuff.bufferFilled() < InBuff.getMaxBlockSize()){
        if(InBuff.bufferFilled()){
            if(!readID3V1Tag()){
                int bytesDecoded = sendBytes(InBuff.getReadPtr(), InBuff.bufferFilled());
                if(bytesDecoded > 2){InBuff.bytesWasRead(bytesDecoded); return;}
            }
        }
        playI2Sremains();
        stopSong(); // Correct close when play known length sound #74 and before callback #11
        if(m_f_tts){
            AUDIO_INFO("End of speech: \"%s\"", m_lastHost);
            if(audio_eof_speech) audio_eof_speech(m_lastHost);
        }
        else{
            AUDIO_INFO("End of webstream: \"%s\"", m_lastHost);
            if(audio_eof_stream) audio_eof_stream(m_lastHost);
        }
        return;
    }

    if(byteCounter == m_contentlength)                    {f_webFileDataComplete = true;}
    if(byteCounter - m_audioDataStart == m_audioDataSize) {f_webFileDataComplete = true;}

    // play audio data - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if(f_stream){
        static uint8_t cnt = 0;
        uint8_t compression;
        if(m_codec == CODEC_WAV)  compression = 1;
        if(m_codec == CODEC_FLAC) compression = 2;
        else compression = 6;
        cnt++;
        if(cnt == compression){playAudioData(); cnt = 0;}
    }
    return;
}
//---------------------------------------------------------------------------------------------------------------------
void Audio::processWebStreamTS() {

    const uint16_t  maxFrameSize = InBuff.getMaxBlockSize();    // every mp3/aac frame is not bigger
    uint32_t        availableBytes;                             // available bytes in stream
    static bool     f_tmr_1s;
    static bool     f_stream;                                   // first audio data received
    static bool     f_firstPacket;
    static uint32_t byteCounter;                                // count received data
    static uint32_t tmr_1s;                                     // timer 1 sec
    static uint32_t loopCnt;                                    // count loops if clientbuffer is empty
    static uint8_t  ts_packet[188];                             // m3u8 transport stream is 188 bytes long
    uint8_t         ts_packetStart = 0;
    uint8_t         ts_packetLength = 0;
    static uint8_t  ts_packetPtr = 0;
    const uint8_t   ts_packetsize = 188;
    static size_t   chunkSize = 0;

    // first call, set some values to default - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if(m_f_firstCall) { // runs only ont time per connection, prepare for start
        f_stream = false;
        f_firstPacket = true;
        byteCounter = 0;
        chunkSize = 0;
        loopCnt = 0;
        tmr_1s = millis();
        m_t0 = millis();
        ts_packetPtr = 0;
        m_controlCounter = 0;
        m_f_firstCall = false;
    }

    if(getDatamode() != AUDIO_DATA) return;        // guard

    if(InBuff.freeSpace() < maxFrameSize && f_stream){playAudioData(); return;}

    availableBytes = _client->available();
    if(availableBytes){
        uint8_t readedBytes = 0;
        if(m_f_chunked) chunkSize = chunkedDataTransfer(&readedBytes);
        int res = _client->read(ts_packet + ts_packetPtr, ts_packetsize - ts_packetPtr);
        if(res > 0){
            ts_packetPtr += res;
            byteCounter += res;
            if(ts_packetPtr < ts_packetsize)  return;
            ts_packetPtr = 0;
            if(f_firstPacket){  // search for ID3 Header in the first packet
                f_firstPacket = false;
                uint8_t ID3_HeaderSize = process_m3u8_ID3_Header(ts_packet);
                if(ID3_HeaderSize > ts_packetsize){
                    log_e("ID3 Header is too big");
                    stopSong();
                    return;
                }
                if(ID3_HeaderSize){
                    memcpy(ts_packet, &ts_packet[ID3_HeaderSize], ts_packetsize - ID3_HeaderSize);
                    ts_packetPtr = ts_packetsize - ID3_HeaderSize;
                    return;
                }
            }
            ts_parsePacket(&ts_packet[0], &ts_packetStart, &ts_packetLength);

            if(ts_packetLength) {
                size_t ws = InBuff.writeSpace();
                if(ws >= ts_packetLength){
                    memcpy(InBuff.getWritePtr(), ts_packet + ts_packetStart, ts_packetLength);
                    InBuff.bytesWritten(ts_packetLength);
                }
                else{
                    memcpy(InBuff.getWritePtr(), ts_packet + ts_packetStart, ws);
                    InBuff.bytesWritten(ws);
                    memcpy(InBuff.getWritePtr(), &ts_packet[ws + ts_packetStart], ts_packetLength -ws);
                    InBuff.bytesWritten(ts_packetLength -ws);
                }
            }
            if(byteCounter == m_contentlength  || byteCounter == chunkSize){
                byteCounter = 0;
                m_f_continue = true;
            }
            if(byteCounter > m_contentlength) log_e("byteCounter overflow");
        }

    }

    // timer, triggers every second - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if((tmr_1s + 1000) < millis()) {
        f_tmr_1s = true;                                        // flag will be set every second for one loop only
        tmr_1s = millis();
    }

    // if the buffer is often almost empty issue a warning  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if(InBuff.bufferFilled() < maxFrameSize && f_stream){
        static uint8_t cnt_slow = 0;
        cnt_slow ++;
        if(f_tmr_1s) {
            if(cnt_slow > 50 && audio_info) audio_info("slow stream, dropouts are possible");
            f_tmr_1s = false;
            cnt_slow = 0;
        }
    }

    // if the buffer can't filled for several seconds try a new connection  - - - - - - - - - - - - - - - - - - - - - -
    if(f_stream && !availableBytes){
        loopCnt++;
        if(loopCnt > 200000) {              // wait several seconds
            loopCnt = 0;
            AUDIO_INFO("Stream lost -> try new connection");
            httpPrint(m_lastHost);
            return;
        }
    }
    if(availableBytes) loopCnt = 0;

    // buffer fill routine  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if(true) { // statement has no effect
        if(InBuff.bufferFilled() > maxFrameSize && !f_stream) {  // waiting for buffer filled
            f_stream = true;  // ready to play the audio data
            uint16_t filltime = millis() - m_t0;
            if(m_f_Log) AUDIO_INFO("stream ready");
            if(m_f_Log) AUDIO_INFO("buffer filled in %d ms", filltime);
        }
        if(!f_stream) return;
    }

    // play audio data - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if(f_stream){
        static uint8_t cnt = 0;
        cnt++;
        if(cnt == 6){playAudioData(); cnt = 0;}
    }
    return;
}
//---------------------------------------------------------------------------------------------------------------------
void Audio::processWebStreamHLS() {

    const uint16_t  maxFrameSize = InBuff.getMaxBlockSize();    // every mp3/aac frame is not bigger
    const uint16_t  ID3BuffSize = 1024;
    uint32_t        availableBytes;                             // available bytes in stream
    static bool     f_tmr_1s;
    static bool     f_stream;                                   // first audio data received
    static bool     firstBytes;
    static uint32_t byteCounter;                                // count received data
    static size_t   chunkSize = 0;
    static uint32_t tmr_1s;                                     // timer 1 sec
    static uint32_t loopCnt;                                    // count loops if clientbuffer is empty
    static uint16_t ID3WritePtr;
    static uint16_t ID3ReadPtr;
    static uint8_t* ID3Buff;

    // first call, set some values to default - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if(m_f_firstCall) { // runs only ont time per connection, prepare for start
        f_stream = false;
        byteCounter = 0;
        chunkSize = 0;
        loopCnt = 0;
        ID3WritePtr = 0;
        ID3ReadPtr = 0;
        tmr_1s = millis();
        m_t0 = millis();
        m_f_firstCall = false;
        firstBytes = true;
        ID3Buff = (uint8_t*)malloc(ID3BuffSize);
        m_controlCounter = 0;
    }

    if(getDatamode() != AUDIO_DATA) return;        // guard

    availableBytes = _client->available();
    if(availableBytes){ // an ID3 header could come here
        uint8_t readedBytes = 0;

        if(m_f_chunked && !chunkSize) {chunkSize = chunkedDataTransfer(&readedBytes); byteCounter += readedBytes;}

        if(firstBytes){
            if(ID3WritePtr < ID3BuffSize){
                ID3WritePtr += _client->readBytes(&ID3Buff[ID3WritePtr], ID3BuffSize - ID3WritePtr);
                return;
            }
            if(m_controlCounter < 100){
                int res = read_ID3_Header(&ID3Buff[ID3ReadPtr], ID3BuffSize - ID3ReadPtr);
                if(res >= 0) ID3ReadPtr += res;
                if(ID3ReadPtr > ID3BuffSize) {log_e("buffer overflow"); stopSong(); return;}
                return;
            }
            if(m_controlCounter != 100) return;

            size_t ws = InBuff.writeSpace();
            if(ws >= ID3BuffSize - ID3ReadPtr){
                memcpy(InBuff.getWritePtr(), &ID3Buff[ID3ReadPtr], ID3BuffSize - ID3ReadPtr);
                InBuff.bytesWritten(ID3BuffSize - ID3ReadPtr);
            }
            else{
                memcpy(InBuff.getWritePtr(), &ID3Buff[ID3ReadPtr], ws);
                InBuff.bytesWritten(ws);
                memcpy(InBuff.getWritePtr(), &ID3Buff[ws + ID3ReadPtr], ID3BuffSize - (ID3ReadPtr + ws));
                InBuff.bytesWritten(ID3BuffSize - (ID3ReadPtr + ws));
            }
            if(ID3Buff) free(ID3Buff);
            byteCounter += ID3BuffSize;
            ID3Buff = NULL;
            firstBytes = false;
        }

        size_t bytesWasWritten = 0;
        if(InBuff.writeSpace() >= availableBytes){
            bytesWasWritten = _client->read(InBuff.getWritePtr(), availableBytes);
        }
        else{
            bytesWasWritten = _client->read(InBuff.getWritePtr(), InBuff.writeSpace());
        }
        InBuff.bytesWritten(bytesWasWritten);

        byteCounter += bytesWasWritten;
        if(byteCounter == m_contentlength || byteCounter == chunkSize){
            byteCounter = 0;
            m_f_continue = true;
        }
    }

    // timer, triggers every second - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if((tmr_1s + 1000) < millis()) {
        f_tmr_1s = true;                                        // flag will be set every second for one loop only
        tmr_1s = millis();
    }

    // if the buffer is often almost empty issue a warning  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if(InBuff.bufferFilled() < maxFrameSize && f_stream){
        static uint8_t cnt_slow = 0;
        cnt_slow ++;
        if(f_tmr_1s) {
            if(cnt_slow > 25 && audio_info) audio_info("slow stream, dropouts are possible");
            f_tmr_1s = false;
            cnt_slow = 0;
        }
    }

    // if the buffer can't filled for several seconds try a new connection  - - - - - - - - - - - - - - - - - - - - - -
    if(f_stream && !availableBytes){
        loopCnt++;
        if(loopCnt > 200000) {              // wait several seconds
            loopCnt = 0;
            AUDIO_INFO("Stream lost -> try new connection");
            httpPrint(m_lastHost);
            return;
        }
    }
    if(availableBytes) loopCnt = 0;

    if(InBuff.bufferFilled() > maxFrameSize && !f_stream) {  // waiting for buffer filled
        f_stream = true;  // ready to play the audio data
        uint16_t filltime = millis() - m_t0;
        if(m_f_Log) AUDIO_INFO("stream ready");
        if(m_f_Log) AUDIO_INFO("buffer filled in %d ms", filltime);
    }

    // play audio data - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if(f_stream){
        static uint8_t cnt = 0;
        cnt++;
        if(cnt == 1){playAudioData(); cnt = 0;}
    }
    return;
}
//---------------------------------------------------------------------------------------------------------------------
void Audio::playAudioData(){

    if(InBuff.bufferFilled() < InBuff.getMaxBlockSize()) return; // guard

    int bytesDecoded = sendBytes(InBuff.getReadPtr(), InBuff.getMaxBlockSize());

    if(bytesDecoded < 0) {  // no syncword found or decode error, try next chunk
        log_i("err bytesDecoded %i", bytesDecoded);
        uint8_t next = 200;
        if(InBuff.bufferFilled() < next) next = InBuff.bufferFilled();
        InBuff.bytesWasRead(next); // try next chunk
        m_bytesNotDecoded += next;
    }
    else {
        if(bytesDecoded > 0) {InBuff.bytesWasRead(bytesDecoded); return;}
        if(bytesDecoded == 0) return; // syncword at pos0 found
    }

    return;
}
//---------------------------------------------------------------------------------------------------------------------
bool Audio::parseHttpResponseHeader() { // this is the response to a GET / request

    if(getDatamode() != HTTP_RESPONSE_HEADER) return false;
    if(_client->available() == 0) return false;

    char rhl[512]; // responseHeaderline
    bool ct_seen = false;
    uint32_t ctime = millis();
    uint32_t timeout = 2500; // ms

    while(true){  // outer while
        uint16_t pos = 0;
        if((millis() - ctime) > timeout) {
            log_e("timeout");
            goto exit;
        }
        while(_client->available()){
            uint8_t b = _client->read();
            if(b == '\n') {
                if(!pos){ // empty line received, is the last line of this responseHeader
                    if(ct_seen) goto lastToDo;
                    else goto exit;
                }
                break;
            }
            if(b == '\r') rhl[pos] = 0;
            if(b < 0x20) continue;
            rhl[pos] = b;
            pos++;
            if(pos == 511){pos = 510; continue;}
            if(pos == 510){
                rhl[pos] = '\0';
                if(m_f_Log) log_i("responseHeaderline overflow");
            }
        } // inner while

        if(!pos){vTaskDelay(3); continue;}

        if(m_f_Log) {log_i("httpResponseHeader: %s", rhl);}

        int16_t posColon = indexOf(rhl, ":", 0); // lowercase all letters up to the colon
        if(posColon >= 0) {
            for(int i=0; i< posColon; i++) {
                rhl[i] = toLowerCase(rhl[i]);
            }
        }

        if(startsWith(rhl, "HTTP/")){ // HTTP status error code
            char statusCode[5];
            statusCode[0] = rhl[9];
            statusCode[1] = rhl[10];
            statusCode[2] = rhl[11];
            statusCode[3] = '\0';
            int sc = atoi(statusCode);
            if(sc > 310){ // e.g. HTTP/1.1 301 Moved Permanently
                if(audio_showstreamtitle) audio_showstreamtitle(rhl);
                goto exit;
            }
        }

        else if(startsWith(rhl, "content-type:")){ // content-type: text/html; charset=UTF-8
            int idx = indexOf(rhl + 13, ";");
            if(idx >0) rhl[13 + idx] = '\0';
            if(parseContentType(rhl + 13)) ct_seen = true;
            else goto exit;
        }

        else if(startsWith(rhl, "location:")) {
            int pos = indexOf(rhl, "http", 0);
            if(pos >= 0){
                const char* c_host = (rhl + pos);
                if(strcmp(c_host, m_lastHost) != 0) { // prevent a loop
                    int pos_slash = indexOf(c_host, "/", 9);
                    if(pos_slash > 9){
                        if(!strncmp(c_host, m_lastHost, pos_slash)){
                            AUDIO_INFO("redirect to new extension at existing host \"%s\"", c_host);
                            if(m_playlistFormat == FORMAT_M3U8) {
                                strcpy(m_lastHost, c_host);
                                m_f_m3u8data = true;
                            }
                            httpPrint(c_host);
                            while(_client->available()) _client->read(); // empty client buffer
                            return true;
                        }
                    }
                    AUDIO_INFO("redirect to new host \"%s\"", c_host);
                    connecttohost(c_host);
                    return true;
                }
            }
        }

        else if(startsWith(rhl, "content-encoding:")){
            if(indexOf(rhl, "gzip")){
                AUDIO_INFO("can't extract gzip");
                goto exit;
            }
        }

        else if(startsWith(rhl, "content-disposition:")) {
            int pos1, pos2; // pos3;
            // e.g we have this headerline:  content-disposition: attachment; filename=stream.asx
            // filename is: "stream.asx"
            pos1 = indexOf(rhl, "filename=", 0);
            if(pos1 > 0){
                pos1 += 9;
                if(rhl[pos1] == '\"') pos1++;  // remove '\"' around filename if present
                pos2 = strlen(rhl);
                if(rhl[pos2 - 1] == '\"') rhl[pos2 - 1] = '\0';
            }
            AUDIO_INFO("Filename is %s", rhl + pos1);
        }

        // if(startsWith(rhl, "set-cookie:")         ||
        //         startsWith(rhl, "pragma:")        ||
        //         startsWith(rhl, "expires:")       ||
        //         startsWith(rhl, "cache-control:") ||
        //         startsWith(rhl, "icy-pub:")       ||
        //         startsWith(rhl, "p3p:")           ||
        //         startsWith(rhl, "accept-ranges:") ){
        //     ; // do nothing
        // }

        else if(startsWith(rhl, "connection:")) {
            if(indexOf(rhl, "close", 0) >= 0) {; /* do nothing */}
        }

        else if(startsWith(rhl, "icy-genre:")) {
            ; // do nothing Ambient, Rock, etc
        }

        else if(startsWith(rhl, "icy-br:")) {
            const char* c_bitRate = (rhl + 7);
            int32_t br = atoi(c_bitRate); // Found bitrate tag, read the bitrate in Kbit
            br = br * 1000;
            setBitrate(br);
            sprintf(chbuf, "%d", getBitRate());
            if(audio_bitrate) audio_bitrate(chbuf);
        }

        else if(startsWith(rhl, "icy-metaint:")) {
            const char* c_metaint = (rhl + 12);
            int32_t i_metaint = atoi(c_metaint);
            m_metaint = i_metaint;
            if(m_metaint) m_f_metadata = true;                       // Multimediastream
        }

        else if(startsWith(rhl, "icy-name:")) {
            char* c_icyname = (rhl + 9); // Get station name
            trim(c_icyname);
            if(strlen(c_icyname) > 0) {
                if(!m_f_Log) AUDIO_INFO("icy-name: %s", c_icyname);
                if(audio_showstation) audio_showstation(c_icyname);
            }
        }

        else if(startsWith(rhl, "content-length:")) {
            const char* c_cl = (rhl + 15);
            int32_t i_cl = atoi(c_cl);
            m_contentlength = i_cl;
            m_streamType = ST_WEBFILE; // Stream comes from a fileserver
            if(m_f_Log) AUDIO_INFO("content-length: %i", m_contentlength);
        }

        else if(startsWith(rhl, "icy-description:")) {
            const char* c_idesc = (rhl + 16);
            while(c_idesc[0] == ' ') c_idesc++;
            latinToUTF8(rhl, sizeof(rhl)); // if already UTF-0 do nothing, otherwise convert to UTF-8
            if(audio_icydescription) audio_icydescription(c_idesc);
        }

        else if((startsWith(rhl, "transfer-encoding:"))){
            if(endsWith(rhl, "chunked") || endsWith(rhl, "Chunked") ) { // Station provides chunked transfer
                m_f_chunked = true;
                if(!m_f_Log) AUDIO_INFO("chunked data transfer");
                m_chunkcount = 0;                         // Expect chunkcount in DATA
            }
        }

        else if(startsWith(rhl, "icy-url:")) {
            char* icyurl = (rhl + 8);
            trim(icyurl);
            if(audio_icyurl) audio_icyurl(icyurl);
        }

        else if(startsWith(rhl, "www-authenticate:")) {
            AUDIO_INFO("authentification failed, wrong credentials?");
            goto exit;
        }
        else {;}
    } // outer while

    exit:  // termination condition
        if(audio_showstation) audio_showstation("");
        if(audio_icydescription) audio_icydescription("");
        if(audio_icyurl) audio_icyurl("");
        m_lastHost[0] = '\0';
        setDatamode(AUDIO_NONE);
        stopSong();
        return false;

    lastToDo:
        if(m_codec != CODEC_NONE){
            setDatamode(AUDIO_DATA); // Expecting data now
            if(!initializeDecoder()) return false;
            if(m_f_Log) {log_i("Switch to DATA, metaint is %d", m_metaint);}
            if(m_playlistFormat != FORMAT_M3U8 && audio_lasthost) audio_lasthost(m_lastHost);
            m_controlCounter = 0;
            m_f_firstCall = true;
        }
        else if(m_playlistFormat != FORMAT_NONE){
            setDatamode(AUDIO_PLAYLISTINIT); // playlist expected
            if(m_f_Log) {log_i("now parse playlist");}
        }
        else{
            AUDIO_INFO("unknown content found at: %s", m_lastHost);
            goto exit;
        }
        return true;
}
//---------------------------------------------------------------------------------------------------------------------
bool Audio:: initializeDecoder(){
    switch(m_codec){
        case CODEC_MP3:
            if(!MP3Decoder_AllocateBuffers()) goto exit;
            AUDIO_INFO("MP3Decoder has been initialized, free Heap: %u bytes", ESP.getFreeHeap());
            InBuff.changeMaxBlockSize(m_frameSizeMP3);
            break;
        case CODEC_AAC:
            if(!AACDecoder_IsInit()){
                if(!AACDecoder_AllocateBuffers()) goto exit;
                AUDIO_INFO("AACDecoder has been initialized, free Heap: %u bytes", ESP.getFreeHeap());
                InBuff.changeMaxBlockSize(m_frameSizeAAC);
            }
            break;
        case CODEC_M4A:
            if(!AACDecoder_IsInit()){
                if(!AACDecoder_AllocateBuffers()) goto exit;
                AUDIO_INFO("AACDecoder has been initialized, free Heap: %u bytes", ESP.getFreeHeap());
                InBuff.changeMaxBlockSize(m_frameSizeAAC);
            }
            break;
        case CODEC_FLAC:
            if(!psramFound()){
                AUDIO_INFO("FLAC works only with PSRAM!");
                goto exit;
            }
            if(!FLACDecoder_AllocateBuffers()) goto exit;
            InBuff.changeMaxBlockSize(m_frameSizeFLAC);
            AUDIO_INFO("FLACDecoder has been initialized, free Heap: %u bytes", ESP.getFreeHeap());
            break;
        case CODEC_WAV:
            InBuff.changeMaxBlockSize(m_frameSizeWav);
            break;
        case CODEC_OGG:
            m_codec = CODEC_OGG;
            AUDIO_INFO("ogg not supported");
            goto exit;
            break;
        default:
            goto exit;
            break;
    }
    return true;

    exit:
        stopSong();
        return false;
}
//---------------------------------------------------------------------------------------------------------------------
bool Audio::parseContentType(char* ct) {

    enum : int {CT_NONE, CT_MP3, CT_AAC, CT_M4A, CT_WAV, CT_OGG, CT_FLAC, CT_PLS, CT_M3U, CT_ASX,
                CT_M3U8, CT_TXT, CT_AACP};

    strlwr(ct);
    trim(ct);

    m_codec = CODEC_NONE;
    int ct_val = CT_NONE;

    if(!strcmp(ct, "audio/mpeg"))            ct_val = CT_MP3;
    else if(!strcmp(ct, "audio/mpeg3"))      ct_val = CT_MP3;
    else if(!strcmp(ct, "audio/x-mpeg"))     ct_val = CT_MP3;
    else if(!strcmp(ct, "audio/x-mpeg-3"))   ct_val = CT_MP3;
    else if(!strcmp(ct, "audio/mp3"))        ct_val = CT_MP3;

    else if(!strcmp(ct, "audio/aac"))        ct_val = CT_AAC;
    else if(!strcmp(ct, "audio/x-aac"))      ct_val = CT_AAC;
    else if(!strcmp(ct, "audio/aacp")){      ct_val = CT_AAC; if(m_playlistFormat == FORMAT_M3U8) m_f_ts = true;}
    else if(!strcmp(ct, "video/mp2t")){      ct_val = CT_AAC; m_f_ts = true;} // assume AAC transport stream
    else if(!strcmp(ct, "audio/mp4"))        ct_val = CT_M4A;
    else if(!strcmp(ct, "audio/m4a"))        ct_val = CT_M4A;
    else if(!strcmp(ct, "audio/x-m4a"))      ct_val = CT_M4A;

    else if(!strcmp(ct, "audio/wav"))        ct_val = CT_WAV;
    else if(!strcmp(ct, "audio/x-wav"))      ct_val = CT_WAV;

    else if(!strcmp(ct, "audio/flac"))       ct_val = CT_FLAC;

    else if(!strcmp(ct, "audio/scpls"))      ct_val = CT_PLS;
    else if(!strcmp(ct, "audio/x-scpls"))    ct_val = CT_PLS;
    else if(!strcmp(ct, "application/pls+xml")) ct_val = CT_PLS;
    else if(!strcmp(ct, "audio/mpegurl"))   {ct_val = CT_M3U; if(m_expectedPlsFmt == FORMAT_M3U8) ct_val = CT_M3U8;}
    else if(!strcmp(ct, "audio/x-mpegurl"))  ct_val = CT_M3U;
    else if(!strcmp(ct, "audio/ms-asf"))     ct_val = CT_ASX;
    else if(!strcmp(ct, "video/x-ms-asf"))   ct_val = CT_ASX;

    else if(!strcmp(ct, "application/ogg"))  ct_val = CT_OGG;
    else if(!strcmp(ct, "application/vnd.apple.mpegurl")) ct_val = CT_M3U8;
    else if(!strcmp(ct, "application/x-mpegurl")) ct_val =CT_M3U8;

    else if(!strcmp(ct, "application/octet-stream")) ct_val = CT_TXT; // ??? listen.radionomy.com/1oldies before redirection
    else if(!strcmp(ct, "text/html"))        ct_val = CT_TXT;
    else if(!strcmp(ct, "text/plain"))       ct_val = CT_TXT;

    else if(ct_val == CT_NONE){
        AUDIO_INFO("ContentType %s not supported", ct);
        return false; // nothing valid had been seen
    }
    else {;}

    switch(ct_val){
        case CT_MP3:
            m_codec = CODEC_MP3;
            if(m_f_Log) { log_i("ContentType %s, format is mp3", ct); } //ok is likely mp3
            break;
        case CT_AAC:
            m_codec = CODEC_AAC;
            if(m_f_Log) { log_i("ContentType %s, format is aac", ct); }
            break;
        case CT_M4A:
            m_codec = CODEC_M4A;
            if(m_f_Log) { log_i("ContentType %s, format is aac", ct); }
            break;
        case CT_FLAC:
            m_codec = CODEC_FLAC;
            if(m_f_Log) { log_i("ContentType %s, format is flac", ct); }
            break;
        case CT_WAV:
            m_codec = CODEC_WAV;
            if(m_f_Log) { log_i("ContentType %s, format is wav", ct); }
            break;
        case CT_OGG:
            m_codec = CODEC_OGG;
            if(m_f_Log) { log_i("ContentType %s found", ct); }
            break;

        case CT_PLS:
            m_playlistFormat = FORMAT_PLS;
            break;
        case CT_M3U:
            m_playlistFormat = FORMAT_M3U;
            break;
        case CT_ASX:
            m_playlistFormat = FORMAT_ASX;
            break;
        case CT_M3U8:
            m_playlistFormat = FORMAT_M3U8;
            break;
        case CT_TXT: // overwrite text/plain
            if(m_expectedCodec == CODEC_AAC){ m_codec = CODEC_AAC; if(m_f_Log) log_i("set ct from M3U8 to AAC");}
            if(m_expectedCodec == CODEC_MP3){ m_codec = CODEC_MP3; if(m_f_Log) log_i("set ct from M3U8 to MP3");}

            if(m_expectedPlsFmt == FORMAT_ASX){ m_playlistFormat = FORMAT_ASX;  if(m_f_Log) log_i("set playlist format to ASX");}
            if(m_expectedPlsFmt == FORMAT_M3U){ m_playlistFormat = FORMAT_M3U;  if(m_f_Log) log_i("set playlist format to M3U");}
            if(m_expectedPlsFmt == FORMAT_M3U8){m_playlistFormat = FORMAT_M3U8; if(m_f_Log) log_i("set playlist format to M3U8");}
            if(m_expectedPlsFmt == FORMAT_PLS){ m_playlistFormat = FORMAT_PLS;  if(m_f_Log) log_i("set playlist format to PLS");}
            break;
        default:
            AUDIO_INFO("%s, unsupported audio format", ct);
            return false;
            break;
    }
    return true;
}
//---------------------------------------------------------------------------------------------------------------------
void Audio::showstreamtitle(const char* ml) {
    // example for ml:
    // StreamTitle='Oliver Frank - Mega Hitmix';StreamUrl='www.radio-welle-woerthersee.at';
    // or adw_ad='true';durationMilliseconds='10135';adId='34254';insertionType='preroll';

    int16_t idx1, idx2;
    uint16_t i = 0, hash = 0;

    idx1 = indexOf(ml, "StreamTitle=", 0);
    if(idx1 >= 0){                                                              // Streamtitle found
        idx2 = indexOf(ml, ";", idx1);
        char *sTit;
        if(idx2 >= 0){sTit = strndup(ml + idx1, idx2 + 1); sTit[idx2] = '\0';}
        else          sTit =  strdup(ml);

        while(i < strlen(sTit)){hash += sTit[i] * i+1; i++;}

        if(m_streamTitleHash != hash){
            m_streamTitleHash = hash;
            AUDIO_INFO("%s", sTit);
            uint8_t pos = 12;                                                   // remove "StreamTitle="
            if(sTit[pos] == '\'') pos++;                                        // remove leading  \'
            if(sTit[strlen(sTit) - 1] == '\'') sTit[strlen(sTit) -1] = '\0';    // remove trailing \'
            if(audio_showstreamtitle) audio_showstreamtitle(sTit + pos);
        }
        if(sTit) {free(sTit); sTit = NULL;}
    }

    idx1 = indexOf(ml, "StreamUrl=", 0);
    idx2 = indexOf(ml, ";", idx1);
    if(idx1 >= 0 && idx2 > idx1){                                               // StreamURL found
        uint16_t len = idx2 - idx1;
        char *sUrl;
        sUrl = strndup(ml + idx1, len + 1); sUrl[len] = '\0';

        while(i < strlen(sUrl)){hash += sUrl[i] * i+1; i++;}
        if(m_streamTitleHash != hash){
            m_streamTitleHash = hash;
            AUDIO_INFO("%s", sUrl);
        }
        if(sUrl) {free(sUrl); sUrl = NULL;}
    }

    idx1 = indexOf(ml, "adw_ad=", 0);
    if(idx1 >= 0){                                                              // Advertisement found
        idx1 = indexOf(ml, "durationMilliseconds=", 0);
        idx2 = indexOf(ml, ";", idx1);
        if(idx1 >= 0 && idx2 > idx1){
            uint16_t len = idx2 - idx1;
            char *sAdv;
            sAdv = strndup(ml + idx1, len + 1); sAdv[len] = '\0';
            AUDIO_INFO("%s", sAdv);
            uint8_t pos = 21;                                                   // remove "StreamTitle="
            if(sAdv[pos] == '\'') pos++;                                        // remove leading  \'
            if(sAdv[strlen(sAdv) - 1] == '\'') sAdv[strlen(sAdv) -1] = '\0';    // remove trailing \'
            if(audio_commercial) audio_commercial(sAdv + pos);
            if(sAdv){free(sAdv); sAdv = NULL;}
        }
    }
}
//---------------------------------------------------------------------------------------------------------------------
void Audio::showCodecParams(){
    // print Codec Parameter (mp3, aac) in audio_info()

    AUDIO_INFO("Channels: %i", getChannels());
    AUDIO_INFO("SampleRate: %i", getSampleRate());
    AUDIO_INFO("BitsPerSample: %i", getBitsPerSample());
    if(getBitRate()) {AUDIO_INFO("BitRate: %i", getBitRate());}
    else             {AUDIO_INFO("BitRate: N/A");}

    if(m_codec == CODEC_AAC || m_codec == CODEC_M4A){
        uint8_t answ;
        if((answ = AACGetFormat()) < 4){
            const char hf[4][8] = {"unknown", "ADTS", "ADIF", "RAW"};
            sprintf(chbuf, "AAC HeaderFormat: %s", hf[answ]);
            audio_info(chbuf);
        }
        if(answ == 1){ // ADTS Header
            const char co[2][23] = {"MPEG-4", "MPEG-2"};
            sprintf(chbuf, "AAC Codec: %s", co[AACGetID()]);
            audio_info(chbuf);
            if(AACGetProfile() <5){
                const char pr[4][23] = {"Main", "LowComplexity", "Scalable Sampling Rate", "reserved"};
                sprintf(chbuf, "AAC Profile: %s", pr[answ]);
                audio_info(chbuf);
            }
        }
    }
}
//---------------------------------------------------------------------------------------------------------------------
int Audio::findNextSync(uint8_t* data, size_t len){
    // Mp3 and aac audio data are divided into frames. At the beginning of each frame there is a sync word.
    // The sync word is 0xFFF. This is followed by information about the structure of the frame.
    // Wav files have no frames
    // Return: 0 the synchronous word was found at position 0
    //         > 0 is the offset to the next sync word
    //         -1 the sync word was not found within the block with the length len

    int nextSync;
    static uint32_t swnf = 0;
    if(m_codec == CODEC_WAV)  {
        m_f_playing = true; nextSync = 0;
    }
    if(m_codec == CODEC_MP3) {
        nextSync = MP3FindSyncWord(data, len);
    }
    if(m_codec == CODEC_AAC) {
        nextSync = AACFindSyncWord(data, len);
    }
    if(m_codec == CODEC_M4A) {
        AACSetRawBlockParams(0, 2,44100, 1); m_f_playing = true; nextSync = 0;
    }
    if(m_codec == CODEC_FLAC) {
        FLACSetRawBlockParams(m_flacNumChannels,   m_flacSampleRate,
                              m_flacBitsPerSample, m_flacTotalSamplesInStream, m_audioDataSize);
        nextSync = FLACFindSyncWord(data, len);
    }
    if(m_codec == CODEC_OGG_FLAC) {
        FLACSetRawBlockParams(m_flacNumChannels,   m_flacSampleRate,
                              m_flacBitsPerSample, m_flacTotalSamplesInStream, m_audioDataSize);
        nextSync = FLACFindSyncWord(data, len);
    }
    if(nextSync == -1) {
         if(audio_info && swnf == 0) audio_info("syncword not found");
         if(m_codec == CODEC_OGG_FLAC){
             nextSync = len;
         }
         else {
             swnf++; // syncword not found counter, can be multimediadata
         }
     }
     if (nextSync == 0){
         if(audio_info && swnf>0){
             sprintf(chbuf, "syncword not found %i times", swnf);
             audio_info(chbuf);
             swnf = 0;
         }
         else {
             if(audio_info) audio_info("syncword found at pos 0");
         }
     }
     if(nextSync > 0){
         AUDIO_INFO("syncword found at pos %i", nextSync);
     }
     return nextSync;
}
//---------------------------------------------------------------------------------------------------------------------
int Audio::sendBytes(uint8_t* data, size_t len) {
    int bytesLeft;
    static bool f_setDecodeParamsOnce = true;
    int nextSync = 0;
    if(!m_f_playing) {
        f_setDecodeParamsOnce = true;
        nextSync = findNextSync(data, len);
        if(nextSync == 0) { m_f_playing = true;}
        return nextSync;
    }
    // m_f_playing is true at this pos
    bytesLeft = len;
    int ret = 0;
    int bytesDecoded = 0;

    switch(m_codec){
        case CODEC_WAV:      memmove(m_outBuff, data , len); //copy len data in outbuff and set validsamples and bytesdecoded=len
                             if(getBitsPerSample() == 16) m_validSamples = len / (2 * getChannels());
                             if(getBitsPerSample() == 8 ) m_validSamples = len / 2;
                             bytesLeft = 0; break;
        case CODEC_MP3:      ret = MP3Decode(data, &bytesLeft, m_outBuff, 0); break;
        case CODEC_AAC:      ret = AACDecode(data, &bytesLeft, m_outBuff);    break;
        case CODEC_M4A:      ret = AACDecode(data, &bytesLeft, m_outBuff);    break;
        case CODEC_FLAC:     ret = FLACDecode(data, &bytesLeft, m_outBuff);   break;
        case CODEC_OGG_FLAC: ret = FLACDecode(data, &bytesLeft, m_outBuff);   break; // FLAC webstream wrapped in OGG
        default: {log_e("no valid codec found codec = %d", m_codec); stopSong();}
    }

    bytesDecoded = len - bytesLeft;
    if(bytesDecoded == 0 && ret == 0){ // unlikely framesize
            if(audio_info) audio_info("framesize is 0, start decoding again");
            m_f_playing = false; // seek for new syncword
        // we're here because there was a wrong sync word
        // so skip two sync bytes and seek for next
        return 1;
    }
    if(ret < 0) { // Error, skip the frame...
        if(m_f_Log) if(m_codec == CODEC_M4A){log_i("begin not found"); return 1;}
        i2s_zero_dma_buffer((i2s_port_t)m_i2s_num);
        if(!getChannels() && (ret == -2)) {
             ; // suppress errorcode MAINDATA_UNDERFLOW
        }
        else {
            printDecodeError(ret);
            m_f_playing = false; // seek for new syncword
        }
        if(!bytesDecoded) bytesDecoded = 2;
        return bytesDecoded;
    }
    else{  // ret>=0
        if(f_setDecodeParamsOnce){
            f_setDecodeParamsOnce = false;
            m_PlayingStartTime = millis();

            if(m_codec == CODEC_MP3){
                setChannels(MP3GetChannels());
                setSampleRate(MP3GetSampRate());
                setBitsPerSample(MP3GetBitsPerSample());
                setBitrate(MP3GetBitrate());
            }
            if(m_codec == CODEC_AAC || m_codec == CODEC_M4A){
                setChannels(AACGetChannels());
                setSampleRate(AACGetSampRate());
                setBitsPerSample(AACGetBitsPerSample());
                setBitrate(AACGetBitrate());
            }
            if(m_codec == CODEC_FLAC || m_codec == CODEC_OGG_FLAC){
                setChannels(FLACGetChannels());
                setSampleRate(FLACGetSampRate());
                setBitsPerSample(FLACGetBitsPerSample());
                setBitrate(FLACGetBitRate());
            }
            showCodecParams();
        }
        if(m_codec == CODEC_MP3){
            m_validSamples = MP3GetOutputSamps() / getChannels();
        }
        if((m_codec == CODEC_AAC) || (m_codec == CODEC_M4A)){
            m_validSamples = AACGetOutputSamps() / getChannels();
        }
        if((m_codec == CODEC_FLAC) || (m_codec == CODEC_OGG_FLAC)){
            m_validSamples = FLACGetOutputSamps() / getChannels();
        }
    }
    compute_audioCurrentTime(bytesDecoded);

    if(audio_process_extern){
        bool continueI2S = false;
        audio_process_extern(m_outBuff, m_validSamples, &continueI2S);
        if(!continueI2S){
            return bytesDecoded;
        }
    }
    while(m_validSamples) {
        playChunk();
    }
    return bytesDecoded;
}
//---------------------------------------------------------------------------------------------------------------------
void Audio::compute_audioCurrentTime(int bd) {
    static uint16_t loop_counter = 0;
    static int old_bitrate = 0;
    static uint64_t sum_bitrate = 0;
    static boolean f_CBR = true; // constant bitrate

    if(m_codec == CODEC_MP3) {setBitrate(MP3GetBitrate()) ;} // if not CBR, bitrate can be changed
    if(m_codec == CODEC_M4A) {setBitrate(AACGetBitrate()) ;} // if not CBR, bitrate can be changed
    if(m_codec == CODEC_AAC) {setBitrate(AACGetBitrate()) ;} // if not CBR, bitrate can be changed
    if(m_codec == CODEC_FLAC){setBitrate(FLACGetBitRate());} // if not CBR, bitrate can be changed
    if(!getBitRate()) return;

    //- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if(m_avr_bitrate == 0) { // first time
        loop_counter = 0;
        old_bitrate = 0;
        sum_bitrate = 0;
        f_CBR = true;
        m_avr_bitrate = getBitRate();
        old_bitrate = getBitRate();
    }
    if(!m_avr_bitrate) return;
    //- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    if(loop_counter < 1000) loop_counter ++;

    if((old_bitrate != getBitRate()) && f_CBR) {
        if(audio_info) audio_info("VBR recognized, audioFileDuration is estimated");
        f_CBR = false; // variable bitrate
    }
    old_bitrate = getBitRate();

    if(!f_CBR) {
        if(loop_counter > 20 && loop_counter < 200) {
            // if VBR: m_avr_bitrate is average of the first values of m_bitrate
            sum_bitrate += getBitRate();
            m_avr_bitrate = sum_bitrate / (loop_counter - 20);
            if(loop_counter == 199 && m_resumeFilePos){
                m_audioCurrentTime = ((getFilePos() - m_audioDataStart - inBufferFilled()) / m_avr_bitrate) * 8; // #293
            }
        }
    }
    else {
        if(loop_counter == 2){
            m_avr_bitrate = getBitRate();
            if(m_resumeFilePos){  // if connecttoFS() is called with resumeFilePos != 0
                m_audioCurrentTime = ((getFilePos() - m_audioDataStart - inBufferFilled()) / m_avr_bitrate) * 8; // #293
            }
        }
    }
    m_audioCurrentTime += ((float)bd / m_avr_bitrate) * 8;
}
//---------------------------------------------------------------------------------------------------------------------
void Audio::printDecodeError(int r) {
    const char *e;

    if(m_codec == CODEC_MP3){
        switch(r){
            case ERR_MP3_NONE:                              e = "NONE";                             break;
            case ERR_MP3_INDATA_UNDERFLOW:                  e = "INDATA_UNDERFLOW";                 break;
            case ERR_MP3_MAINDATA_UNDERFLOW:                e = "MAINDATA_UNDERFLOW";               break;
            case ERR_MP3_FREE_BITRATE_SYNC:                 e = "FREE_BITRATE_SYNC";                break;
            case ERR_MP3_OUT_OF_MEMORY:                     e = "OUT_OF_MEMORY";                    break;
            case ERR_MP3_NULL_POINTER:                      e = "NULL_POINTER";                     break;
            case ERR_MP3_INVALID_FRAMEHEADER:               e = "INVALID_FRAMEHEADER";              break;
            case ERR_MP3_INVALID_SIDEINFO:                  e = "INVALID_SIDEINFO";                 break;
            case ERR_MP3_INVALID_SCALEFACT:                 e = "INVALID_SCALEFACT";                break;
            case ERR_MP3_INVALID_HUFFCODES:                 e = "INVALID_HUFFCODES";                break;
            case ERR_MP3_INVALID_DEQUANTIZE:                e = "INVALID_DEQUANTIZE";               break;
            case ERR_MP3_INVALID_IMDCT:                     e = "INVALID_IMDCT";                    break;
            case ERR_MP3_INVALID_SUBBAND:                   e = "INVALID_SUBBAND";                  break;
            default: e = "ERR_UNKNOWN";
        }
        AUDIO_INFO("MP3 decode error %d : %s", r, e);
     }
    if(m_codec == CODEC_AAC){
        switch(r){
            case ERR_AAC_NONE:                              e = "NONE";                             break;
            case ERR_AAC_INDATA_UNDERFLOW:                  e = "INDATA_UNDERFLOW";                 break;
            case ERR_AAC_NULL_POINTER:                      e = "NULL_POINTER";                     break;
            case ERR_AAC_INVALID_ADTS_HEADER:               e = "INVALID_ADTS_HEADER";              break;
            case ERR_AAC_INVALID_ADIF_HEADER:               e = "INVALID_ADIF_HEADER";              break;
            case ERR_AAC_INVALID_FRAME:                     e = "INVALID_FRAME";                    break;
            case ERR_AAC_MPEG4_UNSUPPORTED:                 e = "MPEG4_UNSUPPORTED";                break;
            case ERR_AAC_CHANNEL_MAP:                       e = "CHANNEL_MAP";                      break;
            case ERR_AAC_SYNTAX_ELEMENT:                    e = "SYNTAX_ELEMENT";                   break;
            case ERR_AAC_DEQUANT:                           e = "DEQUANT";                          break;
            case ERR_AAC_STEREO_PROCESS:                    e = "STEREO_PROCESS";                   break;
            case ERR_AAC_PNS:                               e = "PNS";                              break;
            case ERR_AAC_SHORT_BLOCK_DEINT:                 e = "SHORT_BLOCK_DEINT";                break;
            case ERR_AAC_TNS:                               e = "TNS";                              break;
            case ERR_AAC_IMDCT:                             e = "IMDCT";                            break;
            case ERR_AAC_SBR_INIT:                          e = "SBR_INIT";                         break;
            case ERR_AAC_SBR_BITSTREAM:                     e = "SBR_BITSTREAM";                    break;
            case ERR_AAC_SBR_DATA:                          e = "SBR_DATA";                         break;
            case ERR_AAC_SBR_PCM_FORMAT:                    e = "SBR_PCM_FORMAT";                   break;
            case ERR_AAC_SBR_NCHANS_TOO_HIGH:               e = "SBR_NCHANS_TOO_HIGH";              break;
            case ERR_AAC_SBR_SINGLERATE_UNSUPPORTED:        e = "BR_SINGLERATE_UNSUPPORTED";        break;
            case ERR_AAC_NCHANS_TOO_HIGH:                   e = "NCHANS_TOO_HIGH";                  break;
            case ERR_AAC_RAWBLOCK_PARAMS:                   e = "RAWBLOCK_PARAMS";                  break;
            default: e = "ERR_UNKNOWN";
        }
        AUDIO_INFO("AAC decode error %d : %s", r, e);
    }
    if(m_codec == CODEC_FLAC){
        switch(r){
            case ERR_FLAC_NONE:                             e = "NONE";                             break;
            case ERR_FLAC_BLOCKSIZE_TOO_BIG:                e = "BLOCKSIZE TOO BIG";                break;
            case ERR_FLAC_RESERVED_BLOCKSIZE_UNSUPPORTED:   e = "Reserved Blocksize unsupported";   break;
            case ERR_FLAC_SYNC_CODE_NOT_FOUND:              e = "SYNC CODE NOT FOUND";              break;
            case ERR_FLAC_UNKNOWN_CHANNEL_ASSIGNMENT:       e = "UNKNOWN CHANNEL ASSIGNMENT";       break;
            case ERR_FLAC_RESERVED_CHANNEL_ASSIGNMENT:      e = "RESERVED CHANNEL ASSIGNMENT";      break;
            case ERR_FLAC_RESERVED_SUB_TYPE:                e = "RESERVED SUB TYPE";                break;
            case ERR_FLAC_PREORDER_TOO_BIG:                 e = "PREORDER TOO BIG";                 break;
            case ERR_FLAC_RESERVED_RESIDUAL_CODING:         e = "RESERVED RESIDUAL CODING";         break;
            case ERR_FLAC_WRONG_RICE_PARTITION_NR:          e = "WRONG RICE PARTITION NR";          break;
            case ERR_FLAC_BITS_PER_SAMPLE_TOO_BIG:          e = "BITS PER SAMPLE > 16";             break;
            case ERR_FLAG_BITS_PER_SAMPLE_UNKNOWN:          e = "BITS PER SAMPLE UNKNOWN";          break;
            default: e = "ERR_UNKNOWN";
        }
        AUDIO_INFO("FLAC decode error %d : %s", r, e);
    }
}
//---------------------------------------------------------------------------------------------------------------------
bool Audio::setPinout(uint8_t BCLK, uint8_t LRC, uint8_t DOUT, int8_t DIN, int8_t MCK) {

    m_pin_config.bck_io_num   = BCLK;
    m_pin_config.ws_io_num    = LRC; //  wclk
    m_pin_config.data_out_num = DOUT;
    m_pin_config.data_in_num  = DIN;
#if(ESP_IDF_VERSION_MAJOR >= 4 && ESP_IDF_VERSION_MINOR >= 4)
    m_pin_config.mck_io_num   = MCK;
#endif

    const esp_err_t result = i2s_set_pin((i2s_port_t) m_i2s_num, &m_pin_config);
    return (result == ESP_OK);
}
//---------------------------------------------------------------------------------------------------------------------
uint32_t Audio::getFileSize() {
    if(!audiofile) return 0;
    return audiofile.size();
}
//---------------------------------------------------------------------------------------------------------------------
uint32_t Audio::getFilePos() {
    if(!audiofile) return 0;
    return audiofile.position();
}
//---------------------------------------------------------------------------------------------------------------------
uint32_t Audio::getAudioDataStartPos() {
    if(!audiofile) return 0;
    return m_audioDataStart;
}
//---------------------------------------------------------------------------------------------------------------------
uint32_t Audio::getAudioFileDuration() {
    if(getDatamode() == AUDIO_LOCALFILE) {if(!audiofile) return 0;}
    if(m_streamType == ST_WEBFILE)   {if(!m_contentlength) return 0;}

    if     (m_avr_bitrate && m_codec == CODEC_MP3)   m_audioFileDuration = 8 * (m_audioDataSize / m_avr_bitrate); // #289
    else if(m_avr_bitrate && m_codec == CODEC_WAV)   m_audioFileDuration = 8 * (m_audioDataSize / m_avr_bitrate);
    else if(m_avr_bitrate && m_codec == CODEC_M4A)   m_audioFileDuration = 8 * (m_audioDataSize / m_avr_bitrate);
    else if(m_avr_bitrate && m_codec == CODEC_AAC)   m_audioFileDuration = 8 * (m_audioDataSize / m_avr_bitrate);
    else if(                 m_codec == CODEC_FLAC)  m_audioFileDuration = FLACGetAudioFileDuration();
    else return 0;
    return m_audioFileDuration;
}
//---------------------------------------------------------------------------------------------------------------------
uint32_t Audio::getAudioCurrentTime() {  // return current time in seconds
    return (uint32_t) m_audioCurrentTime;
}
//---------------------------------------------------------------------------------------------------------------------
bool Audio::setAudioPlayPosition(uint16_t sec){
    // Jump to an absolute position in time within an audio file
    // e.g. setAudioPlayPosition(300) sets the pointer at pos 5 min
    // works only with format mp3 or wav
    if(m_codec == CODEC_M4A)  return false;
    if(sec > getAudioFileDuration()) sec = getAudioFileDuration();
    uint32_t filepos = m_audioDataStart + (m_avr_bitrate * sec / 8);

    return setFilePos(filepos);
}
//---------------------------------------------------------------------------------------------------------------------
uint32_t Audio::getTotalPlayingTime() {
    // Is set to zero by a connectToXXX() and starts as soon as the first audio data is available,
    // the time counting is not interrupted by a 'pause / resume' and is not reset by a fileloop
    return millis() - m_PlayingStartTime;
}
//---------------------------------------------------------------------------------------------------------------------
bool Audio::setTimeOffset(int sec){
    // fast forward or rewind the current position in seconds
    // audiosource must be a mp3, aac or wav file

    if(!audiofile || !m_avr_bitrate) return false;

    uint32_t oneSec  = m_avr_bitrate / 8;                   // bytes decoded in one sec
    int32_t  offset  = oneSec * sec;                        // bytes to be wind/rewind
    uint32_t startAB = m_audioDataStart;                    // audioblock begin
    uint32_t endAB   = m_audioDataStart + m_audioDataSize;  // audioblock end

    if(m_codec == CODEC_MP3 || m_codec == CODEC_AAC || m_codec == CODEC_WAV || m_codec == CODEC_FLAC){
        int32_t pos = getFilePos();
        pos += offset;
        if(pos <  (int32_t)startAB) pos = startAB;
        if(pos >= (int32_t)endAB)   pos = endAB;
        setFilePos(pos);
        return true;
    }
    return false;
}
//---------------------------------------------------------------------------------------------------------------------
bool Audio::setFilePos(uint32_t pos) {
    if(!audiofile) return false;
//    if(!m_avr_bitrate) return false;
    if(m_codec == CODEC_M4A) return false;
    m_f_playing = false;
    if(m_codec == CODEC_MP3) MP3Decoder_ClearBuffer();
    if(m_codec == CODEC_WAV) {while((pos % 4) != 0) pos++;} // must be divisible by four
    if(m_codec == CODEC_FLAC) FLACDecoderReset();
    InBuff.resetBuffer();
    if(pos < m_audioDataStart) pos = m_audioDataStart; // issue #96
    if(m_avr_bitrate) m_audioCurrentTime = ((pos-m_audioDataStart) / m_avr_bitrate) * 8; // #96
    return audiofile.seek(pos);
}
//---------------------------------------------------------------------------------------------------------------------
bool Audio::audioFileSeek(const float speed) {
    // 0.5 is half speed
    // 1.0 is normal speed
    // 1.5 is one and half speed
    if((speed > 1.5f) || (speed < 0.25f)) return false;

    uint32_t srate = getSampleRate() * speed;
    i2s_set_sample_rates((i2s_port_t)m_i2s_num, srate);
    return true;
}
//---------------------------------------------------------------------------------------------------------------------
bool Audio::setSampleRate(uint32_t sampRate) {
    if(!sampRate) sampRate = 16000; // fuse, if there is no value -> set default #209
    i2s_set_sample_rates((i2s_port_t)m_i2s_num, sampRate);
    m_sampleRate = sampRate;
    IIR_calculateCoefficients(m_gain0, m_gain1, m_gain2); // must be recalculated after each samplerate change
    return true;
}
uint32_t Audio::getSampleRate(){
    return m_sampleRate;
}
//---------------------------------------------------------------------------------------------------------------------
bool Audio::setBitsPerSample(int bits) {
    if((bits != 16) && (bits != 8)) return false;
    m_bitsPerSample = bits;
    return true;
}
uint8_t Audio::getBitsPerSample(){
    return m_bitsPerSample;
}
//---------------------------------------------------------------------------------------------------------------------
bool Audio::setChannels(int ch) {
    if((ch < 1) || (ch > 2)) return false;
    m_channels = ch;
    return true;
}
uint8_t Audio::getChannels(){
    if (m_channels == 0) {    // this should not happen! #209
        m_channels = 2;
    }
    return m_channels;
}
//---------------------------------------------------------------------------------------------------------------------
bool Audio::setBitrate(int br){
    m_bitRate = br;
    if(br)return true;
    return false;
}
uint32_t Audio::getBitRate(bool avg){
    if (avg)
        return m_avr_bitrate;
    return m_bitRate;
}
//---------------------------------------------------------------------------------------------------------------------
void Audio::setI2SCommFMT_LSB(bool commFMT) {
    // false: I2S communication format is by default I2S_COMM_FORMAT_I2S_MSB, right->left (AC101, PCM5102A)
    // true:  changed to I2S_COMM_FORMAT_I2S_LSB for some DACs (PT8211)
    //        Japanese or called LSBJ (Least Significant Bit Justified) format

    if (commFMT) {
        if(m_f_Log) log_i("commFMT LSB");

        #if ESP_ARDUINO_VERSION_MAJOR >= 2
            m_i2s_config.communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_STAND_MSB); // v >= 2.0.0
        #else
            m_i2s_config.communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_LSB);
        #endif

    }
    else {
        if(m_f_Log) log_i("commFMT MSB");

        #if ESP_ARDUINO_VERSION_MAJOR >= 2
            m_i2s_config.communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_STAND_I2S); // vers >= 2.0.0
        #else
            m_i2s_config.communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB);
        #endif

    }
    AUDIO_INFO("commFMT = %i", m_i2s_config.communication_format);
    i2s_driver_uninstall((i2s_port_t)m_i2s_num);
    i2s_driver_install  ((i2s_port_t)m_i2s_num, &m_i2s_config, 0, NULL);
}
//---------------------------------------------------------------------------------------------------------------------
bool Audio::playSample(int16_t sample[2]) {

    if (getBitsPerSample() == 8) { // Upsample from unsigned 8 bits to signed 16 bits
        sample[LEFTCHANNEL]  = ((sample[LEFTCHANNEL]  & 0xff) -128) << 8;
        sample[RIGHTCHANNEL] = ((sample[RIGHTCHANNEL] & 0xff) -128) << 8;
    }

    sample[LEFTCHANNEL]  = sample[LEFTCHANNEL]  >> 1; // half Vin so we can boost up to 6dB in filters
    sample[RIGHTCHANNEL] = sample[RIGHTCHANNEL] >> 1;

    // Filterchain, can commented out if not used
    sample = IIR_filterChain0(sample);
    sample = IIR_filterChain1(sample);
    sample = IIR_filterChain2(sample);
    //-------------------------------------------

    uint32_t s32 = Gain(sample); // vosample2lume;

    if(audio_process_i2s){
		// process audio sample just before writing to i2s
        bool continueI2S = false;
        audio_process_i2s(&s32, &continueI2S);
        if(!continueI2S){
            return true;
        }
    }

    if(m_f_internalDAC) {
        s32 += 0x80008000;
    }
    m_i2s_bytesWritten = 0;
    esp_err_t err = i2s_write((i2s_port_t) m_i2s_num, (const char*) &s32, sizeof(uint32_t), &m_i2s_bytesWritten, 100);
    if(err != ESP_OK) {
        log_e("ESP32 Errorcode %i", err);
        return false;
    }
    if(m_i2s_bytesWritten < 4) {
        log_e("Can't stuff any more in I2S..."); // increase waitingtime or outputbuffer
        return false;
    }
    return true;
}
//---------------------------------------------------------------------------------------------------------------------
void Audio::setTone(int8_t gainLowPass, int8_t gainBandPass, int8_t gainHighPass){
    // see https://www.earlevel.com/main/2013/10/13/biquad-calculator-v2/
    // values can be between -40 ... +6 (dB)

    m_gain0 = gainLowPass;
    m_gain1 = gainBandPass;
    m_gain2 = gainHighPass;

    IIR_calculateCoefficients(m_gain0, m_gain1, m_gain2);

    /*
        This will cause a clicking sound when adjusting the EQ.
        Because when the EQ is adjusted, the IIR filter will be cleared and played,
        mixed in the audio data frame, and a click-like sound will be produced.
    */
    /*
    int16_t tmp[2]; tmp[0] = 0; tmp[1]= 0;

    IIR_filterChain0(tmp, true ); // flush the filter
    IIR_filterChain1(tmp, true ); // flush the filter
    IIR_filterChain2(tmp, true ); // flush the filter
    */
}
//---------------------------------------------------------------------------------------------------------------------
void Audio::forceMono(bool m) { // #100 mono option
    m_f_forceMono = m; // false stereo, true mono
}
//---------------------------------------------------------------------------------------------------------------------
void Audio::setBalance(int8_t bal){ // bal -16...16
    if(bal < -16) bal = -16;
    if(bal >  16) bal =  16;
    m_balance = bal;
}
//---------------------------------------------------------------------------------------------------------------------
void Audio::setVolume(uint8_t vol) { // vol 22 steps, 0...21
    if(vol > 21) vol = 21;
    m_vol = volumetable[vol];
}
//---------------------------------------------------------------------------------------------------------------------
uint8_t Audio::getVolume() {
    for(uint8_t i = 0; i < 22; i++) {
        if(volumetable[i] == m_vol) return i;
    }
    m_vol = 12; // if m_vol not found in table
    return m_vol;
}
//---------------------------------------------------------------------------------------------------------------------
uint8_t Audio::getI2sPort() {
    return m_i2s_num;
}
//---------------------------------------------------------------------------------------------------------------------
int32_t Audio::Gain(int16_t s[2]) {
    int32_t v[2];
    float step = (float)m_vol /64;
    uint8_t l = 0, r = 0;

    if(m_balance < 0){
        step = step * (float)(abs(m_balance) * 4);
        l = (uint8_t)(step);
    }
    if(m_balance > 0){
        step = step * m_balance * 4;
        r = (uint8_t)(step);
    }

    v[LEFTCHANNEL] = (s[LEFTCHANNEL]  * (m_vol - l)) >> 6;
    v[RIGHTCHANNEL]= (s[RIGHTCHANNEL] * (m_vol - r)) >> 6;

    return (v[LEFTCHANNEL] << 16) | (v[RIGHTCHANNEL] & 0xffff);
}
//---------------------------------------------------------------------------------------------------------------------
uint32_t Audio::inBufferFilled() {
    // current audio input buffer fillsize in bytes
    return InBuff.bufferFilled();
}
//---------------------------------------------------------------------------------------------------------------------
uint32_t Audio::inBufferFree() {
    // current audio input buffer free space in bytes
    return InBuff.freeSpace();
}
//---------------------------------------------------------------------------------------------------------------------
//            ***     D i g i t a l   b i q u a d r a t i c     f i l t e r     ***
//---------------------------------------------------------------------------------------------------------------------
void Audio::IIR_calculateCoefficients(int8_t G0, int8_t G1, int8_t G2){  // Infinite Impulse Response (IIR) filters

    // G1 - gain low shelf   set between -40 ... +6 dB
    // G2 - gain peakEQ      set between -40 ... +6 dB
    // G3 - gain high shelf  set between -40 ... +6 dB
    // https://www.earlevel.com/main/2012/11/26/biquad-c-source-code/

    if(getSampleRate() < 1000) return;  // fuse

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    if(G0 < -40) G0 = -40;      // -40dB -> Vin*0.01
    if(G0 > 6) G0 = 6;          // +6dB -> Vin*2
    if(G1 < -40) G1 = -40;
    if(G1 > 6) G1 = 6;
    if(G2 < -40) G2 = -40;
    if(G2 > 6) G2 = 6;

    const float FcLS   =  500;  // Frequency LowShelf[Hz]
    const float FcPKEQ = 3000;  // Frequency PeakEQ[Hz]
    const float FcHS   = 6000;  // Frequency HighShelf[Hz]

    float K, norm, Q, Fc, V ;

    // LOWSHELF
    Fc = (float)FcLS / (float)getSampleRate(); // Cutoff frequency
    K = tanf((float)PI * Fc);
    V = powf(10, fabs(G0) / 20.0);

    if (G0 >= 0) {  // boost
        norm = 1 / (1 + sqrtf(2) * K + K * K);
        m_filter[LOWSHELF].a0 = (1 + sqrtf(2*V) * K + V * K * K) * norm;
        m_filter[LOWSHELF].a1 = 2 * (V * K * K - 1) * norm;
        m_filter[LOWSHELF].a2 = (1 - sqrtf(2*V) * K + V * K * K) * norm;
        m_filter[LOWSHELF].b1 = 2 * (K * K - 1) * norm;
        m_filter[LOWSHELF].b2 = (1 - sqrtf(2) * K + K * K) * norm;
    }
    else {          // cut
        norm = 1 / (1 + sqrtf(2*V) * K + V * K * K);
        m_filter[LOWSHELF].a0 = (1 + sqrtf(2) * K + K * K) * norm;
        m_filter[LOWSHELF].a1 = 2 * (K * K - 1) * norm;
        m_filter[LOWSHELF].a2 = (1 - sqrtf(2) * K + K * K) * norm;
        m_filter[LOWSHELF].b1 = 2 * (V * K * K - 1) * norm;
        m_filter[LOWSHELF].b2 = (1 - sqrtf(2*V) * K + V * K * K) * norm;
    }

    // PEAK EQ
    Fc = (float)FcPKEQ / (float)getSampleRate(); // Cutoff frequency
    K = tanf((float)PI * Fc);
    V = powf(10, fabs(G1) / 20.0);
    Q = 2.5; // Quality factor
    if (G1 >= 0) { // boost
        norm = 1 / (1 + 1/Q * K + K * K);
        m_filter[PEAKEQ].a0 = (1 + V/Q * K + K * K) * norm;
        m_filter[PEAKEQ].a1 = 2 * (K * K - 1) * norm;
        m_filter[PEAKEQ].a2 = (1 - V/Q * K + K * K) * norm;
        m_filter[PEAKEQ].b1 = m_filter[PEAKEQ].a1;
        m_filter[PEAKEQ].b2 = (1 - 1/Q * K + K * K) * norm;
    }
    else {    // cut
        norm = 1 / (1 + V/Q * K + K * K);
        m_filter[PEAKEQ].a0 = (1 + 1/Q * K + K * K) * norm;
        m_filter[PEAKEQ].a1 = 2 * (K * K - 1) * norm;
        m_filter[PEAKEQ].a2 = (1 - 1/Q * K + K * K) * norm;
        m_filter[PEAKEQ].b1 = m_filter[PEAKEQ].a1;
        m_filter[PEAKEQ].b2 = (1 - V/Q * K + K * K) * norm;
    }

    // HIGHSHELF
    Fc = (float)FcHS / (float)getSampleRate(); // Cutoff frequency
    K = tanf((float)PI * Fc);
    V = powf(10, fabs(G2) / 20.0);
    if (G2 >= 0) {  // boost
        norm = 1 / (1 + sqrtf(2) * K + K * K);
        m_filter[HIFGSHELF].a0 = (V + sqrtf(2*V) * K + K * K) * norm;
        m_filter[HIFGSHELF].a1 = 2 * (K * K - V) * norm;
        m_filter[HIFGSHELF].a2 = (V - sqrtf(2*V) * K + K * K) * norm;
        m_filter[HIFGSHELF].b1 = 2 * (K * K - 1) * norm;
        m_filter[HIFGSHELF].b2 = (1 - sqrtf(2) * K + K * K) * norm;
    }
    else {
        norm = 1 / (V + sqrtf(2*V) * K + K * K);
        m_filter[HIFGSHELF].a0 = (1 + sqrtf(2) * K + K * K) * norm;
        m_filter[HIFGSHELF].a1 = 2 * (K * K - 1) * norm;
        m_filter[HIFGSHELF].a2 = (1 - sqrtf(2) * K + K * K) * norm;
        m_filter[HIFGSHELF].b1 = 2 * (K * K - V) * norm;
        m_filter[HIFGSHELF].b2 = (V - sqrtf(2*V) * K + K * K) * norm;
    }

//    log_i("LS a0=%f, a1=%f, a2=%f, b1=%f, b2=%f", m_filter[0].a0, m_filter[0].a1, m_filter[0].a2,
//                                                  m_filter[0].b1, m_filter[0].b2);
//    log_i("EQ a0=%f, a1=%f, a2=%f, b1=%f, b2=%f", m_filter[1].a0, m_filter[1].a1, m_filter[1].a2,
//                                                  m_filter[1].b1, m_filter[1].b2);
//    log_i("HS a0=%f, a1=%f, a2=%f, b1=%f, b2=%f", m_filter[2].a0, m_filter[2].a1, m_filter[2].a2,
//                                                  m_filter[2].b1, m_filter[2].b2);
}
//---------------------------------------------------------------------------------------------------------------------
int16_t* Audio::IIR_filterChain0(int16_t iir_in[2], bool clear){  // Infinite Impulse Response (IIR) filters

    uint8_t z1 = 0, z2 = 1;
    enum: uint8_t {in = 0, out = 1};
    float inSample[2];
    float outSample[2];
    static int16_t iir_out[2];

    if(clear){
        memset(m_filterBuff, 0, sizeof(m_filterBuff));            // zero IIR filterbuffer
        iir_out[0] = 0;
        iir_out[1] = 0;
        iir_in[0]  = 0;
        iir_in[1]  = 0;
    }

    inSample[LEFTCHANNEL]  = (float)(iir_in[LEFTCHANNEL]);
    inSample[RIGHTCHANNEL] = (float)(iir_in[RIGHTCHANNEL]);

    outSample[LEFTCHANNEL] =   m_filter[0].a0  * inSample[LEFTCHANNEL]
                             + m_filter[0].a1  * m_filterBuff[0][z1][in] [LEFTCHANNEL]
                             + m_filter[0].a2  * m_filterBuff[0][z2][in] [LEFTCHANNEL]
                             - m_filter[0].b1  * m_filterBuff[0][z1][out][LEFTCHANNEL]
                             - m_filter[0].b2  * m_filterBuff[0][z2][out][LEFTCHANNEL];

    m_filterBuff[0][z2][in] [LEFTCHANNEL]  = m_filterBuff[0][z1][in][LEFTCHANNEL];
    m_filterBuff[0][z1][in] [LEFTCHANNEL]  = inSample[LEFTCHANNEL];
    m_filterBuff[0][z2][out][LEFTCHANNEL]  = m_filterBuff[0][z1][out][LEFTCHANNEL];
    m_filterBuff[0][z1][out][LEFTCHANNEL]  = outSample[LEFTCHANNEL];
    iir_out[LEFTCHANNEL] = (int16_t)outSample[LEFTCHANNEL];


    outSample[RIGHTCHANNEL] =  m_filter[0].a0 * inSample[RIGHTCHANNEL]
                             + m_filter[0].a1 * m_filterBuff[0][z1][in] [RIGHTCHANNEL]
                             + m_filter[0].a2 * m_filterBuff[0][z2][in] [RIGHTCHANNEL]
                             - m_filter[0].b1 * m_filterBuff[0][z1][out][RIGHTCHANNEL]
                             - m_filter[0].b2 * m_filterBuff[0][z2][out][RIGHTCHANNEL];

    m_filterBuff[0][z2][in] [RIGHTCHANNEL] = m_filterBuff[0][z1][in][RIGHTCHANNEL];
    m_filterBuff[0][z1][in] [RIGHTCHANNEL] = inSample[RIGHTCHANNEL];
    m_filterBuff[0][z2][out][RIGHTCHANNEL] = m_filterBuff[0][z1][out][RIGHTCHANNEL];
    m_filterBuff[0][z1][out][RIGHTCHANNEL] = outSample[RIGHTCHANNEL];
    iir_out[RIGHTCHANNEL] = (int16_t) outSample[RIGHTCHANNEL];

    return iir_out;
}
//---------------------------------------------------------------------------------------------------------------------
int16_t* Audio::IIR_filterChain1(int16_t iir_in[2], bool clear){  // Infinite Impulse Response (IIR) filters

    uint8_t z1 = 0, z2 = 1;
    enum: uint8_t {in = 0, out = 1};
    float inSample[2];
    float outSample[2];
    static int16_t iir_out[2];

    if(clear){
        memset(m_filterBuff, 0, sizeof(m_filterBuff));            // zero IIR filterbuffer
        iir_out[0] = 0;
        iir_out[1] = 0;
        iir_in[0]  = 0;
        iir_in[1]  = 0;
    }

    inSample[LEFTCHANNEL]  = (float)(iir_in[LEFTCHANNEL]);
    inSample[RIGHTCHANNEL] = (float)(iir_in[RIGHTCHANNEL]);

    outSample[LEFTCHANNEL] =   m_filter[1].a0  * inSample[LEFTCHANNEL]
                             + m_filter[1].a1  * m_filterBuff[1][z1][in] [LEFTCHANNEL]
                             + m_filter[1].a2  * m_filterBuff[1][z2][in] [LEFTCHANNEL]
                             - m_filter[1].b1  * m_filterBuff[1][z1][out][LEFTCHANNEL]
                             - m_filter[1].b2  * m_filterBuff[1][z2][out][LEFTCHANNEL];

    m_filterBuff[1][z2][in] [LEFTCHANNEL]  = m_filterBuff[1][z1][in][LEFTCHANNEL];
    m_filterBuff[1][z1][in] [LEFTCHANNEL]  = inSample[LEFTCHANNEL];
    m_filterBuff[1][z2][out][LEFTCHANNEL]  = m_filterBuff[1][z1][out][LEFTCHANNEL];
    m_filterBuff[1][z1][out][LEFTCHANNEL]  = outSample[LEFTCHANNEL];
    iir_out[LEFTCHANNEL] = (int16_t)outSample[LEFTCHANNEL];


    outSample[RIGHTCHANNEL] =  m_filter[1].a0 * inSample[RIGHTCHANNEL]
                             + m_filter[1].a1 * m_filterBuff[1][z1][in] [RIGHTCHANNEL]
                             + m_filter[1].a2 * m_filterBuff[1][z2][in] [RIGHTCHANNEL]
                             - m_filter[1].b1 * m_filterBuff[1][z1][out][RIGHTCHANNEL]
                             - m_filter[1].b2 * m_filterBuff[1][z2][out][RIGHTCHANNEL];

    m_filterBuff[1][z2][in] [RIGHTCHANNEL] = m_filterBuff[1][z1][in][RIGHTCHANNEL];
    m_filterBuff[1][z1][in] [RIGHTCHANNEL] = inSample[RIGHTCHANNEL];
    m_filterBuff[1][z2][out][RIGHTCHANNEL] = m_filterBuff[1][z1][out][RIGHTCHANNEL];
    m_filterBuff[1][z1][out][RIGHTCHANNEL] = outSample[RIGHTCHANNEL];
    iir_out[RIGHTCHANNEL] = (int16_t) outSample[RIGHTCHANNEL];

    return iir_out;
}
//---------------------------------------------------------------------------------------------------------------------
int16_t* Audio::IIR_filterChain2(int16_t iir_in[2], bool clear){  // Infinite Impulse Response (IIR) filters

    uint8_t z1 = 0, z2 = 1;
    enum: uint8_t {in = 0, out = 1};
    float inSample[2];
    float outSample[2];
    static int16_t iir_out[2];

    if(clear){
        memset(m_filterBuff, 0, sizeof(m_filterBuff));            // zero IIR filterbuffer
        iir_out[0] = 0;
        iir_out[1] = 0;
        iir_in[0]  = 0;
        iir_in[1]  = 0;
    }

    inSample[LEFTCHANNEL]  = (float)(iir_in[LEFTCHANNEL]);
    inSample[RIGHTCHANNEL] = (float)(iir_in[RIGHTCHANNEL]);

    outSample[LEFTCHANNEL] =   m_filter[2].a0  * inSample[LEFTCHANNEL]
                             + m_filter[2].a1  * m_filterBuff[2][z1][in] [LEFTCHANNEL]
                             + m_filter[2].a2  * m_filterBuff[2][z2][in] [LEFTCHANNEL]
                             - m_filter[2].b1  * m_filterBuff[2][z1][out][LEFTCHANNEL]
                             - m_filter[2].b2  * m_filterBuff[2][z2][out][LEFTCHANNEL];

    m_filterBuff[2][z2][in] [LEFTCHANNEL]  = m_filterBuff[2][z1][in][LEFTCHANNEL];
    m_filterBuff[2][z1][in] [LEFTCHANNEL]  = inSample[LEFTCHANNEL];
    m_filterBuff[2][z2][out][LEFTCHANNEL]  = m_filterBuff[2][z1][out][LEFTCHANNEL];
    m_filterBuff[2][z1][out][LEFTCHANNEL]  = outSample[LEFTCHANNEL];
    iir_out[LEFTCHANNEL] = (int16_t)outSample[LEFTCHANNEL];


    outSample[RIGHTCHANNEL] =  m_filter[2].a0 * inSample[RIGHTCHANNEL]
                             + m_filter[2].a1 * m_filterBuff[2][z1][in] [RIGHTCHANNEL]
                             + m_filter[2].a2 * m_filterBuff[2][z2][in] [RIGHTCHANNEL]
                             - m_filter[2].b1 * m_filterBuff[2][z1][out][RIGHTCHANNEL]
                             - m_filter[2].b2 * m_filterBuff[2][z2][out][RIGHTCHANNEL];

    m_filterBuff[2][z2][in] [RIGHTCHANNEL] = m_filterBuff[2][z1][in][RIGHTCHANNEL];
    m_filterBuff[2][z1][in] [RIGHTCHANNEL] = inSample[RIGHTCHANNEL];
    m_filterBuff[2][z2][out][RIGHTCHANNEL] = m_filterBuff[2][z1][out][RIGHTCHANNEL];
    m_filterBuff[2][z1][out][RIGHTCHANNEL] = outSample[RIGHTCHANNEL];
    iir_out[RIGHTCHANNEL] = (int16_t) outSample[RIGHTCHANNEL];

    return iir_out;
}
//----------------------------------------------------------------------------------------------------------------------
//    AAC - T R A N S P O R T S T R E A M
//----------------------------------------------------------------------------------------------------------------------
bool Audio::ts_parsePacket(uint8_t* packet, uint8_t* packetStart, uint8_t* packetLength) {

    const uint8_t TS_PACKET_SIZE = 188;
    const uint8_t PAYLOAD_SIZE = 184;
    const uint8_t PID_ARRAY_LEN = 4;

    (void) PAYLOAD_SIZE;  // suppress [-Wunused-variable]

    typedef struct{
        int number= 0;
        int pids[PID_ARRAY_LEN];
    } pid_array;

    static pid_array pidsOfPMT;
    static int PES_DataLength = 0;
    static int pidOfAAC = 0;

    if(packet == NULL){
        if(m_f_Log) log_i("parseTS reset");
        for(int i = 0; i < PID_ARRAY_LEN; i++) pidsOfPMT.pids[i] = 0;
        PES_DataLength = 0;
        pidOfAAC = 0;
        return true;
    }

    // --------------------------------------------------------------------------------------------------------
    // 0. Byte SyncByte  | 0 | 1 | 0 | 0 | 0 | 1 | 1 | 1 | always bit pattern of 0x47
    //---------------------------------------------------------------------------------------------------------
    // 1. Byte           |PUSI|TP|   |PID|PID|PID|PID|PID|
    //---------------------------------------------------------------------------------------------------------
    // 2. Byte           |PID|PID|PID|PID|PID|PID|PID|PID|
    //---------------------------------------------------------------------------------------------------------
    // 3. Byte           |TSC|TSC|AFC|AFC|CC |CC |CC |CC |
    //---------------------------------------------------------------------------------------------------------
    // 4.-187. Byte      |Payload data if AFC==01 or 11  |
    //---------------------------------------------------------------------------------------------------------

    // PUSI Payload unit start indicator, set when this packet contains the first byte of a new payload unit.
    //      The first byte of the payload will indicate where this new payload unit starts.
    // TP   Transport priority, set when the current packet has a higher priority than other packets with the same PID.
    // PID  Packet Identifier, describing the payload data.
    // TSC  Transport scrambling control, '00' = Not scrambled.
    // AFC  Adaptation field control, 01 – no adaptation field, payload only, 10 – adaptation field only, no payload,
    //                                11 – adaptation field followed by payload, 00 – RESERVED for future use
    // CC   Continuity counter, Sequence number of payload packets (0x00 to 0x0F) within each stream (except PID 8191)

    if(packet[0] != 0x47) {
        log_e("ts SyncByte not found, first bytes are %X %X %X %X", packet[0], packet[1], packet[2], packet[3]);
        stopSong();
        return false;
    }
    int PID = (packet[1] & 0x1F) << 8 | (packet[2] & 0xFF);
    if(m_f_Log) log_i("PID: 0x%04X(%d)", PID, PID);
    int PUSI = (packet[1] & 0x40) >> 6;
    if(m_f_Log) log_i("Payload Unit Start Indicator: %d", PUSI);
    int AFC = (packet[3] & 0x30) >> 4;
    if(m_f_Log) log_i("Adaption Field Control: %d", AFC);

    int AFL = -1;
    if((AFC & 0b10) == 0b10) {  // AFC '11' Adaptation Field followed
        AFL = packet[4] & 0xFF; // Adaptation Field Length
        if(m_f_Log) log_i("Adaptation Field Length: %d", AFL);
    }
    int PLS = PUSI ? 5 : 4;     // PayLoadStart, Payload Unit Start Indicator

    if(PID == 0) {
        // Program Association Table (PAT) - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        if(m_f_Log)  log_i("PAT");
        pidsOfPMT.number = 0;
        pidOfAAC = 0;
        int startOfProgramNums = 8;
        int lengthOfPATValue = 4;
        int sectionLength = ((packet[PLS + 1] & 0x0F) << 8) | (packet[PLS + 2] & 0xFF);
        if(m_f_Log) log_i("Section Length: %d", sectionLength);
        int program_number, program_map_PID;
        int indexOfPids = 0;
        (void) program_number; // [-Wunused-but-set-variable]
        for(int i = startOfProgramNums; i <= sectionLength; i += lengthOfPATValue) {
            program_number = ((packet[PLS + i] & 0xFF) << 8) | (packet[PLS + i + 1] & 0xFF);
            program_map_PID = ((packet[PLS + i + 2] & 0x1F) << 8) | (packet[PLS + i + 3] & 0xFF);
            if(m_f_Log) log_i("Program Num: 0x%04X(%d) PMT PID: 0x%04X(%d)", program_number, program_number,
                   program_map_PID, program_map_PID);
            pidsOfPMT.pids[indexOfPids++] = program_map_PID;
        }
        pidsOfPMT.number = indexOfPids;
        *packetStart = 0;
        *packetLength = 0;
        return true;

    }
    else if(PID == pidOfAAC) {
        static uint8_t fillData = 0;
        if(m_f_Log) log_i("AAC");
        uint8_t posOfPacketStart = 4;
        if(AFL >= 0) {posOfPacketStart = 5 + AFL;
        if(m_f_Log) log_i("posOfPacketStart: %d", posOfPacketStart);}
        // Packetized Elementary Stream (PES) - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        if(m_f_Log) log_i("PES_DataLength %i", PES_DataLength);
        if (PES_DataLength > 0) {
            *packetStart    = posOfPacketStart + fillData;
            *packetLength   = TS_PACKET_SIZE - posOfPacketStart - fillData;
            fillData = 0;
            PES_DataLength -= (*packetLength);
            return true;
        }
        else{
            int firstByte  = packet[posOfPacketStart] & 0xFF;
            int secondByte = packet[posOfPacketStart + 1] & 0xFF;
            int thirdByte  = packet[posOfPacketStart + 2] & 0xFF;
            if(m_f_Log) log_i("First 3 bytes: %02X %02X %02X", firstByte, secondByte, thirdByte);
            if(firstByte == 0x00 && secondByte == 0x00 && thirdByte == 0x01) { // Packet start code prefix
                // PES
                uint8_t StreamID = packet[posOfPacketStart + 3] & 0xFF;
                if(StreamID >= 0xC0 && StreamID <= 0xDF) {;} // okay ist audio stream
                if(StreamID >= 0xE0 && StreamID <= 0xEF) {log_e("video stream!"); return false;}
                uint8_t PES_HeaderDataLength = packet[posOfPacketStart + 8] & 0xFF;
                if(m_f_Log) log_i("PES_headerDataLength %d", PES_HeaderDataLength);
                int PES_PacketLength =
                    ((packet[posOfPacketStart + 4] & 0xFF) << 8) + (packet[posOfPacketStart + 5] & 0xFF);
                if(m_f_Log) log_i("PES Packet length: %d", PES_PacketLength);
                PES_DataLength = PES_PacketLength;
                int startOfData = PES_HeaderDataLength + 9;
                if(posOfPacketStart + startOfData >= 188){ // only fillers in packet
                    if(m_f_Log) log_e("posOfPacketStart + startOfData %i", posOfPacketStart + startOfData);
                    *packetStart = 0;
                    *packetLength = 0;
                    PES_DataLength -= (PES_HeaderDataLength + 3);
                    fillData = (posOfPacketStart + startOfData) - 188;
                    if(m_f_Log) log_i("fillData %i", fillData);
                    return true;
                }
                if(m_f_Log) log_i("First AAC data byte: %02X", packet[posOfPacketStart + startOfData]);
                if(m_f_Log) log_i("Second AAC data byte: %02X", packet[posOfPacketStart + startOfData + 1]);
                *packetStart = posOfPacketStart + startOfData;
                *packetLength = TS_PACKET_SIZE - posOfPacketStart - startOfData;
                PES_DataLength -= (*packetLength);
                PES_DataLength -= (PES_HeaderDataLength + 3);
                return true;
            }
        }
        *packetStart = 0;
        *packetLength = 0;
        log_e("PES not found");
        return false;
    }
    else if(pidsOfPMT.number) {
        //  Program Map Table (PMT) - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        for(int i = 0; i < pidsOfPMT.number; i++) {
            if(PID == pidsOfPMT.pids[i]) {
                if(m_f_Log) log_i("PMT");
                int staticLengthOfPMT = 12;
                int sectionLength = ((packet[PLS + 1] & 0x0F) << 8) | (packet[PLS + 2] & 0xFF);
                if(m_f_Log) log_i("Section Length: %d", sectionLength);
                int programInfoLength = ((packet[PLS + 10] & 0x0F) << 8) | (packet[PLS + 11] & 0xFF);
                if(m_f_Log) log_i("Program Info Length: %d", programInfoLength);
                int cursor = staticLengthOfPMT + programInfoLength;
                while(cursor < sectionLength - 1) {
                    int streamType = packet[PLS + cursor] & 0xFF;
                    int elementaryPID = ((packet[PLS + cursor + 1] & 0x1F) << 8) | (packet[PLS + cursor + 2] & 0xFF);
                    if(m_f_Log) log_i("Stream Type: 0x%02X Elementary PID: 0x%04X", streamType, elementaryPID);

                    if(streamType == 0x0F || streamType == 0x11) {
                        if(m_f_Log) log_i("AAC PID discover");
                        pidOfAAC= elementaryPID;
                    }
                    int esInfoLength = ((packet[PLS + cursor + 3] & 0x0F) << 8) | (packet[PLS + cursor + 4] & 0xFF);
                    if(m_f_Log) log_i("ES Info Length: 0x%04X", esInfoLength);
                    cursor += 5 + esInfoLength;
                }
            }
        }
        *packetStart = 0;
        *packetLength = 0;
        return true;
    }
    // PES received before PAT and PMT seen
    *packetStart = 0;
    *packetLength = 0;
    return false;
}
//----------------------------------------------------------------------------------------------------------------------
//    W E B S T R E A M  -  H E L P   F U N C T I O N S
//----------------------------------------------------------------------------------------------------------------------
uint16_t Audio::readMetadata(uint16_t maxBytes, bool first) {

    static uint16_t pos_ml = 0;                          // determines the current position in metaline
    static uint16_t metalen = 0;
    uint16_t res = 0;
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if(first){
        pos_ml = 0;
        metalen = 0;
        return 0;
    }
    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    if(!maxBytes) return 0;  // guard

    if(!metalen) {
        int b = _client->read();   // First byte of metadata?
        metalen = b * 16 ;                              // New count for metadata including length byte
        if(metalen > 512){
            AUDIO_INFO("Metadata block to long! Skipping all Metadata from now on.");
            m_f_metadata = false;                        // expect stream without metadata
            return 1;
        }
        pos_ml = 0; chbuf[pos_ml] = 0;                   // Prepare for new line
        res = 1;
    }
    if(!metalen) {m_metacount = m_metaint; return res;} // metalen is 0
    uint16_t a = _client->readBytes(&chbuf[pos_ml], min((uint16_t)(metalen - pos_ml), (uint16_t)(maxBytes -1)));
    res += a;
    pos_ml += a;
    if(pos_ml == metalen) {
        chbuf[pos_ml] = '\0';
        if(strlen(chbuf)) {                             // Any info present?
            // metaline contains artist and song name.  For example:
            // "StreamTitle='Don McLean - American Pie';StreamUrl='';"
            // Sometimes it is just other info like:
            // "StreamTitle='60s 03 05 Magic60s';StreamUrl='';"
            // Isolate the StreamTitle, remove leading and trailing quotes if present.
            if(m_f_Log) log_i("metaline %s", chbuf);
            latinToUTF8(chbuf, sizeof(chbuf)); // convert to UTF-8 if necessary
            int pos = indexOf(chbuf, "song_spot", 0);    // remove some irrelevant infos
            if(pos > 3) {                                // e.g. song_spot="T" MediaBaseId="0" itunesTrackId="0"
                chbuf[pos] = 0;
            }
            showstreamtitle(chbuf);   // Show artist and title if present in metadata
        }
        m_metacount = m_metaint;
        metalen = 0;
        pos_ml = 0;
    }
    return res;
}
//----------------------------------------------------------------------------------------------------------------------
size_t Audio::chunkedDataTransfer(uint8_t* bytes){
    uint8_t byteCounter = 0;
    size_t chunksize = 0;
    int b = 0;
    uint32_t ctime = millis();
    uint32_t timeout = 2000; // ms
    while(true){
        if(ctime + timeout < millis()) {
            log_e("timeout");
            stopSong();
            return 0;
        }
        b = _client->read();
        byteCounter++;
        if(b < 0) continue;  // -1 no data available
        if(b == '\n') break;
        if(b < '0') continue;
        // We have received a hexadecimal character.  Decode it and add to the result.
        b = toupper(b) - '0';                       // Be sure we have uppercase
        if(b > 9) b = b - 7;                        // Translate A..F to 10..15
        chunksize = (chunksize << 4) + b;
    }
    if(m_f_Log) log_i("chunksize %d", chunksize);
    *bytes = byteCounter;
    return chunksize;
}
//----------------------------------------------------------------------------------------------------------------------
bool Audio::readID3V1Tag(){
    // this is an V1.x id3tag after an audio block, ID3 v1 tags are ASCII
    // Version 1.x is a fixed size at the end of the file (128 bytes) after a <TAG> keyword.
    if(m_codec != CODEC_MP3) return false;
    if(InBuff.bufferFilled() == 128 && startsWith((const char*)InBuff.getReadPtr(), "TAG")){ // maybe a V1.x TAG
        char title[31];
        memcpy(title,   InBuff.getReadPtr() + 3 +  0,  30);  title[30]  = '\0'; latinToUTF8(title, sizeof(title));
        char artist[31];
        memcpy(artist,  InBuff.getReadPtr() + 3 + 30,  30); artist[30]  = '\0'; latinToUTF8(artist, sizeof(artist));
        char album[31];
        memcpy(album,   InBuff.getReadPtr() + 3 + 60,  30);  album[30]  = '\0'; latinToUTF8(album, sizeof(album));
        char year[5];
        memcpy(year,    InBuff.getReadPtr() + 3 + 90,   4);  year[4]    = '\0'; latinToUTF8(year, sizeof(year));
        char comment[31];
        memcpy(comment, InBuff.getReadPtr() + 3 + 94,  30); comment[30] = '\0'; latinToUTF8(comment, sizeof(comment));
        uint8_t zeroByte = *(InBuff.getReadPtr() + 125);
        uint8_t track    = *(InBuff.getReadPtr() + 126);
        uint8_t genre    = *(InBuff.getReadPtr() + 127);
        if(zeroByte) {AUDIO_INFO("ID3 version: 1");} //[2]
        else         {AUDIO_INFO("ID3 Version 1.1");}
        if(strlen(title))  {sprintf(chbuf, "Title: %s",        title);   if(audio_id3data) audio_id3data(chbuf);}
        if(strlen(artist)) {sprintf(chbuf, "Artist: %s",       artist);  if(audio_id3data) audio_id3data(chbuf);}
        if(strlen(album))  {sprintf(chbuf, "Album: %s",        album);   if(audio_id3data) audio_id3data(chbuf);}
        if(strlen(year))   {sprintf(chbuf, "Year: %s",         year);    if(audio_id3data) audio_id3data(chbuf);}
        if(strlen(comment)){sprintf(chbuf, "Comment: %s",      comment); if(audio_id3data) audio_id3data(chbuf);}
        if(zeroByte == 0)  {sprintf(chbuf, "Track Number: %d", track);   if(audio_id3data) audio_id3data(chbuf);}
        if(genre < 192)    {sprintf(chbuf, "Genre: %d",        genre);   if(audio_id3data) audio_id3data(chbuf);} //[1]
        return true;
    }
    if(InBuff.bufferFilled() == 227 && startsWith((const char*)InBuff.getReadPtr(), "TAG+")){ // ID3V1EnhancedTAG
        AUDIO_INFO("ID3 version: 1 - Enhanced TAG");
        char title[61];
        memcpy(title,   InBuff.getReadPtr() + 4 +   0,  60);  title[60] = '\0'; latinToUTF8(title, sizeof(title));
        char artist[61];
        memcpy(artist,  InBuff.getReadPtr() + 4 +  60,  60); artist[60] = '\0'; latinToUTF8(artist, sizeof(artist));
        char album[61];
        memcpy(album,   InBuff.getReadPtr() + 4 + 120,  60);  album[60] = '\0'; latinToUTF8(album, sizeof(album));
        // one byte "speed" 0=unset, 1=slow, 2= medium, 3=fast, 4=hardcore
        char genre[31];
        memcpy(genre,   InBuff.getReadPtr() + 5 + 180,  30);  genre[30] = '\0'; latinToUTF8(genre, sizeof(genre));
        // six bytes "start-time", the start of the music as mmm:ss
        // six bytes "end-time",   the end of the music as mmm:ss
        if(strlen(title))  {sprintf(chbuf, "Title: %s",  title);  if(audio_id3data) audio_id3data(chbuf);}
        if(strlen(artist)) {sprintf(chbuf, "Artist: %s", artist); if(audio_id3data) audio_id3data(chbuf);}
        if(strlen(album))  {sprintf(chbuf, "Album: %s",  album);  if(audio_id3data) audio_id3data(chbuf);}
        if(strlen(genre))  {sprintf(chbuf, "Genre: %s",  genre);  if(audio_id3data) audio_id3data(chbuf);}
        return true;
    }
    return false;
    // [1] https://en.wikipedia.org/wiki/List_of_ID3v1_Genres
    // [2] https://en.wikipedia.org/wiki/ID3#ID3v1_and_ID3v1.1[5]
}
//----------------------------------------------------------------------------------------------------------------------
void Audio::slowStreamDetection(uint32_t inBuffFilled, uint32_t maxFrameSize){
    static uint32_t tmr_1s   = millis(); // timer 1 sec
    static bool     f_tmr_1s = false;
    static uint8_t  cnt_slow = 0;
    if(tmr_1s + 1000 < millis()) {f_tmr_1s = true; tmr_1s = millis();}
    if(m_codec == CODEC_WAV)  maxFrameSize /= 4;
    if(m_codec == CODEC_FLAC) maxFrameSize /= 2;
    if(inBuffFilled < maxFrameSize){
        cnt_slow ++;
        if(f_tmr_1s) {
            if(cnt_slow > 50) AUDIO_INFO("slow stream, dropouts are possible");
            f_tmr_1s = false;
            cnt_slow = 0;
        }
    }
    else cnt_slow = 0;
}
