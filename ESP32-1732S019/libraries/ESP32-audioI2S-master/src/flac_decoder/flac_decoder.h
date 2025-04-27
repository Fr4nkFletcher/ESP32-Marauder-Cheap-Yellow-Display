/*
 * flac_decoder.h
 *
 * Created on: Jul 03,2020
 * Updated on: Apr 27,2021
 *
 *      Author: wolle
 *
 *  Restrictions:
 *  blocksize must not exceed 8192
 *  bits per sample must be 8 or 16
 *  num Channels must be 1 or 2
 *
 *
 */
#pragma once
#pragma GCC optimize ("Ofast")

#include "Arduino.h"

#define MAX_CHANNELS 2
#define MAX_BLOCKSIZE 8192
#define APLL_DISABLE 0
#define EXTERNAL_I2S  0


typedef struct FLACsubFramesBuff_t{
    int32_t samplesBuffer[MAX_CHANNELS][MAX_BLOCKSIZE];
}FLACsubframesBuffer_t;

enum : uint8_t {FLACDECODER_INIT, FLACDECODER_READ_IN, FLACDECODER_WRITE_OUT};
enum : uint8_t {DECODE_FRAME, DECODE_SUBFRAMES, OUT_SAMPLES};
enum : int8_t  {GIVE_NEXT_LOOP = +1,
                ERR_FLAC_NONE = 0,
                ERR_FLAC_BLOCKSIZE_TOO_BIG = -1,
                ERR_FLAC_RESERVED_BLOCKSIZE_UNSUPPORTED = -2,
                ERR_FLAC_SYNC_CODE_NOT_FOUND = -3,
                ERR_FLAC_UNKNOWN_CHANNEL_ASSIGNMENT = -4,
                ERR_FLAC_RESERVED_CHANNEL_ASSIGNMENT = -5,
                ERR_FLAC_RESERVED_SUB_TYPE = -6,
                ERR_FLAC_PREORDER_TOO_BIG = -7,
                ERR_FLAC_RESERVED_RESIDUAL_CODING = -8,
                ERR_FLAC_WRONG_RICE_PARTITION_NR = -9,
                ERR_FLAC_BITS_PER_SAMPLE_TOO_BIG = -10,
                ERR_FLAG_BITS_PER_SAMPLE_UNKNOWN = 11};

typedef struct FLACMetadataBlock_t{
                              // METADATA_BLOCK_STREAMINFO
    uint16_t minblocksize;    // The minimum block size (in samples) used in the stream.
                              //----------------------------------------------------------------------------------------
                              // The maximum block size (in samples) used in the stream.
    uint16_t maxblocksize;    // (Minimum blocksize == maximum blocksize) implies a fixed-blocksize stream.
                              //----------------------------------------------------------------------------------------
                              // The minimum frame size (in bytes) used in the stream.
    uint32_t minframesize;    // May be 0 to imply the value is not known.
                              //----------------------------------------------------------------------------------------
                              // The maximum frame size (in bytes) used in the stream.
    uint32_t maxframesize;    // May be 0 to imply the value is not known.
                              //----------------------------------------------------------------------------------------
                              // Sample rate in Hz. Though 20 bits are available,
                              // the maximum sample rate is limited by the structure of frame headers to 655350Hz.
    uint32_t sampleRate;      // Also, a value of 0 is invalid.
                              //----------------------------------------------------------------------------------------
                              // Number of channels FLAC supports from 1 to 8 channels
    uint8_t  numChannels;     // 000 : 1 channel .... 111 : 8 channels
                              //----------------------------------------------------------------------------------------
                              // Sample size in bits:
                              // 000 : get from STREAMINFO metadata block
                              // 001 : 8 bits per sample
                              // 010 : 12 bits per sample
                              // 011 : reserved
                              // 100 : 16 bits per sample
                              // 101 : 20 bits per sample
                              // 110 : 24 bits per sample
     uint8_t  bitsPerSample;  // 111 : reserved
                              //----------------------------------------------------------------------------------------
                              // Total samples in stream. 'Samples' means inter-channel sample,
                              // i.e. one second of 44.1Khz audio will have 44100 samples regardless of the number
     uint64_t totalSamples;   // of channels. A value of zero here means the number of total samples is unknown.
                              //----------------------------------------------------------------------------------------
     uint32_t audioDataLength;// is not the filelength, is only the length of the audio datablock in bytes



}FLACMetadataBlock_t;


typedef struct FLACFrameHeader_t {
                              // 0 : fixed-blocksize stream; frame header encodes the frame number
    uint8_t blockingStrategy; // 1 : variable-blocksize stream; frame header encodes the sample number
                              //----------------------------------------------------------------------------------------
                              // Block size in inter-channel samples:
                              // 0000 : reserved
                              // 0001 : 192 samples
                              // 0010-0101 : 576 * (2^(n-2)) samples, i.e. 576/1152/2304/4608
                              // 0110 : get 8 bit (blocksize-1) from end of header
                              // 0111 : get 16 bit (blocksize-1) from end of header
    uint8_t blockSizeCode;    // 1000-1111 : 256 * (2^(n-8)) samples, i.e. 256/512/1024/2048/4096/8192/16384/32768
                              //----------------------------------------------------------------------------------------
                              // 0000 : get from STREAMINFO metadata block
                              // 0001 : 88.2kHz
                              // 0010 : 176.4kHz
                              // 0011 : 192kHz
                              // 0100 : 8kHz
                              // 0101 : 16kHz
                              // 0110 : 22.05kHz
                              // 0111 : 24kHz
                              // 1000 : 32kHz
                              // 1001 : 44.1kHz
                              // 1010 : 48kHz
                              // 1011 : 96kHz
                              // 1100 : get 8 bit sample rate (in kHz) from end of header
                              // 1101 : get 16 bit sample rate (in Hz) from end of header
                              // 1110 : get 16 bit sample rate (in tens of Hz) from end of header
    uint8_t sampleRateCode;   // 1111 : invalid, to prevent sync-fooling string of 1s
                              //----------------------------------------------------------------------------------------
                              // Channel assignment
                              // 0000 1 channel: mono
                              // 0001 2 channels: left, right
                              // 0010 3 channels
                              // 0011 4 channels
                              // 0100 5 channels
                              // 0101 6 channels
                              // 0110 7 channels
                              // 0111 8 channels
                              // 1000 : left/side stereo: channel 0 is the left channel, channel 1 is the side(difference) channel
                              // 1001 : right/side stereo: channel 0 is the side(difference) channel, channel 1 is the right channel
                              // 1010 : mid/side stereo: channel 0 is the mid(average) channel, channel 1 is the side(difference) channel
    uint8_t chanAsgn;         // 1011-1111 : reserved
                              //----------------------------------------------------------------------------------------
                              // Sample size in bits:
                              // 000 : get from STREAMINFO metadata block
                              // 001 : 8 bits per sample
                              // 010 : 12 bits per sample
                              // 011 : reserved
                              // 100 : 16 bits per sample
                              // 101 : 20 bits per sample
                              // 110 : 24 bits per sample
    uint8_t sampleSizeCode;   // 111 : reserved
                              //----------------------------------------------------------------------------------------
    uint32_t totalSamples;    // totalSamplesInStream
                              //----------------------------------------------------------------------------------------
    uint32_t bitrate;         // bitrate


}FLACFrameHeader_t;

int      FLACFindSyncWord(unsigned char *buf, int nBytes);
int      FLACFindOggSyncWord(unsigned char *buf, int nBytes);
int      FLACparseOggHeader(unsigned char *buf);
bool     FLACDecoder_AllocateBuffers(void);
void     FLACDecoder_ClearBuffer();
void     FLACDecoder_FreeBuffers();
void     FLACSetRawBlockParams(uint8_t Chans, uint32_t SampRate, uint8_t BPS, uint32_t tsis, uint32_t AuDaLength);
void     FLACDecoderReset();
int8_t   FLACDecode(uint8_t *inbuf, int *bytesLeft, short *outbuf);
uint16_t FLACGetOutputSamps();
uint64_t FLACGetTotoalSamplesInStream();
uint8_t  FLACGetBitsPerSample();
uint8_t  FLACGetChannels();
uint32_t FLACGetSampRate();
uint32_t FLACGetBitRate();
uint32_t FLACGetAudioFileDuration();
uint32_t readUint(uint8_t nBits);
int32_t  readSignedInt(int nBits);
int64_t  readRiceSignedInt(uint8_t param);
void     alignToByte();
int8_t   decodeSubframes();
int8_t   decodeSubframe(uint8_t sampleDepth, uint8_t ch);
int8_t   decodeFixedPredictionSubframe(uint8_t predOrder, uint8_t sampleDepth, uint8_t ch);
int8_t   decodeLinearPredictiveCodingSubframe(int lpcOrder, int sampleDepth, uint8_t ch);
int8_t   decodeResiduals(uint8_t warmup, uint8_t ch);
void     restoreLinearPrediction(uint8_t ch, uint8_t shift);


