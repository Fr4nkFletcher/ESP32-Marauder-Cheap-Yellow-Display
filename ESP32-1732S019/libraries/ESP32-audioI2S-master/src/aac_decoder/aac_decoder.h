// based on helix aac decoder
#pragma once
//#pragma GCC optimize ("O3")
//#pragma GCC diagnostic ignored "-Wnarrowing"

#include "Arduino.h"

#define AAC_ENABLE_MPEG4

#if (defined CONFIG_IDF_TARGET_ESP32S3 && defined BOARD_HAS_PSRAM)
    #define AAC_ENABLE_SBR  // needs additional 60KB DRAM,
#endif

#define ASSERT(x) /* do nothing */

#ifndef MAX
#define MAX(a,b)    std::max(a,b)
#endif

#ifndef MIN
#define MIN(a,b)    std::min(a,b)
#endif


/* AAC file format */
enum {
    AAC_FF_Unknown = 0,        /* should be 0 on init */
    AAC_FF_ADTS    = 1,
    AAC_FF_ADIF    = 2,
    AAC_FF_RAW     =  3
};

/* syntactic element type */
enum {
    AAC_ID_INVALID = -1,
    AAC_ID_SCE     =  0,
    AAC_ID_CPE     =  1,
    AAC_ID_CCE     =  2,
    AAC_ID_LFE     =  3,
    AAC_ID_DSE     =  4,
    AAC_ID_PCE     =  5,
    AAC_ID_FIL     =  6,
    AAC_ID_END     =  7
};

enum {
    ERR_AAC_NONE                          =   0,
    ERR_AAC_INDATA_UNDERFLOW              =  -1,
    ERR_AAC_NULL_POINTER                  =  -2,
    ERR_AAC_INVALID_ADTS_HEADER           =  -3,
    ERR_AAC_INVALID_ADIF_HEADER           =  -4,
    ERR_AAC_INVALID_FRAME                 =  -5,
    ERR_AAC_MPEG4_UNSUPPORTED             =  -6,
    ERR_AAC_CHANNEL_MAP                   =  -7,
    ERR_AAC_SYNTAX_ELEMENT                =  -8,
    ERR_AAC_DEQUANT                       =  -9,
    ERR_AAC_STEREO_PROCESS                = -10,
    ERR_AAC_PNS                           = -11,
    ERR_AAC_SHORT_BLOCK_DEINT             = -12,
    ERR_AAC_TNS                           = -13,
    ERR_AAC_IMDCT                         = -14,
    ERR_AAC_NCHANS_TOO_HIGH               = -15,
    ERR_AAC_SBR_INIT                      = -16,
    ERR_AAC_SBR_BITSTREAM                 = -17,
    ERR_AAC_SBR_DATA                      = -18,
    ERR_AAC_SBR_PCM_FORMAT                = -19,
    ERR_AAC_SBR_NCHANS_TOO_HIGH           = -20,
    ERR_AAC_SBR_SINGLERATE_UNSUPPORTED    = -21,
    ERR_AAC_RAWBLOCK_PARAMS               = -22,
    ERR_AAC_UNKNOWN                       = -9999
};

enum {
    SBR_GRID_FIXFIX = 0,
    SBR_GRID_FIXVAR = 1,
    SBR_GRID_VARFIX = 2,
    SBR_GRID_VARVAR = 3
};

enum {
    HuffTabSBR_tEnv15 =    0,
    HuffTabSBR_fEnv15 =    1,
    HuffTabSBR_tEnv15b =   2,
    HuffTabSBR_fEnv15b =   3,
    HuffTabSBR_tEnv30 =    4,
    HuffTabSBR_fEnv30 =    5,
    HuffTabSBR_tEnv30b =   6,
    HuffTabSBR_fEnv30b =   7,
    HuffTabSBR_tNoise30 =  8,
    HuffTabSBR_fNoise30 =  5,
    HuffTabSBR_tNoise30b = 9,
    HuffTabSBR_fNoise30b = 7
};

typedef struct _AACDecInfo_t {
    /* raw decoded data, before rounding to 16-bit PCM (for postprocessing such as SBR) */
    void  *rawSampleBuf[2];
    int   rawSampleBytes;
    int   rawSampleFBits;
    /* fill data (can be used for processing SBR or other extensions) */
    uint8_t *fillBuf;
    int   fillCount;
    int   fillExtType;
    int   prevBlockID;    /* block information */
    int   currBlockID;
    int   currInstTag;
    int   sbDeinterleaveReqd[2]; // [MAX_NCHANS_ELEM]
    int   adtsBlocksLeft;
    int   bitRate;    /* user-accessible info */
    int   nChans;
    int   sampRate;
    float compressionRatio;
    int   id;         /* 0: MPEG-4, 1: MPEG2 */
    int   profile;    /* 0: Main profile, 1: LowComplexity (LC), 2: ScalableSamplingRate (SSR), 3: reserved */
    int   format;
    int   sbrEnabled;
    int   tnsUsed;
    int   pnsUsed;
    int   frameCount;
} AACDecInfo_t;


typedef struct _aac_BitStreamInfo_t {
    uint8_t *bytePtr;
    unsigned int iCache;
    int cachedBits;
    int nBytes;
} aac_BitStreamInfo_t;

typedef union _U64 {
    int64_t w64;
    struct {
        unsigned int lo32;
        signed int  hi32;
    } r;
} U64;

typedef struct _AACFrameInfo_t {
    int bitRate;
    int nChans;
    int sampRateCore;
    int sampRateOut;
    int bitsPerSample;
    int outputSamps;
    int profile;
    int tnsUsed;
    int pnsUsed;
} AACFrameInfo_t;

typedef struct _HuffInfo_t {
    int maxBits;              /* number of bits in longest codeword */
    uint8_t count[20];        /*  count[MAX_HUFF_BITS] = number of codes with length i+1 bits */
    int offset;               /* offset into symbol table */
} HuffInfo_t;

typedef struct _PulseInfo_t {
    uint8_t pulseDataPresent;
    uint8_t numPulse;
    uint8_t startSFB;
    uint8_t offset[4]; // [MAX_PULSES]
    uint8_t amp[4];    // [MAX_PULSES]
} PulseInfo_t;

typedef struct _TNSInfo_t {
    uint8_t tnsDataPresent;
    uint8_t numFilt[8]; // [MAX_TNS_FILTERS] max 1 filter each for 8 short windows, or 3 filters for 1 long window
    uint8_t coefRes[8]; // [MAX_TNS_FILTERS]
    uint8_t length[8];  // [MAX_TNS_FILTERS]
    uint8_t order[8];   // [MAX_TNS_FILTERS]
    uint8_t dir[8];     // [MAX_TNS_FILTERS]
    int8_t   coef[60];        // [MAX_TNS_COEFS] max 3 filters * 20 coefs for 1 long window,
                              //  or 1 filter * 7 coefs for each of 8 short windows
} TNSInfo_t;

typedef struct _GainControlInfo_t {
    uint8_t gainControlDataPresent;
    uint8_t maxBand;
    uint8_t adjNum[3][8];      // [MAX_GAIN_BANDS][MAX_GAIN_WIN]
    uint8_t alevCode[3][8][7]; // [MAX_GAIN_BANDS][MAX_GAIN_WIN][MAX_GAIN_ADJUST]
    uint8_t alocCode[3][8][7]; // [MAX_GAIN_BANDS][MAX_GAIN_WIN][MAX_GAIN_ADJUST]
} GainControlInfo_t;

typedef struct _ICSInfo_t {
    uint8_t icsResBit;
    uint8_t winSequence;
    uint8_t winShape;
    uint8_t maxSFB;
    uint8_t sfGroup;
    uint8_t predictorDataPresent;
    uint8_t predictorReset;
    uint8_t predictorResetGroupNum;
    uint8_t predictionUsed[41]; // [MAX_PRED_SFB]
    uint8_t numWinGroup;
    uint8_t winGroupLen[8];     // [MAX_WIN_GROUPS]
} ICSInfo_t;

typedef struct _ADTSHeader_t {
    /* fixed */
    uint8_t id;                         /* MPEG bit - should be 1 */
    uint8_t layer;                      /* MPEG layer - should be 0 */
    uint8_t protectBit;                 /* 0 = CRC word follows, 1 = no CRC word */
    uint8_t profile;                    /* 0 = main, 1 = LC, 2 = SSR, 3 = reserved */
    uint8_t sampRateIdx;                /* sample rate index range = [0, 11] */
    uint8_t privateBit;                 /* ignore */
    uint8_t channelConfig;              /* 0 = implicit, >0 = use default table */
    uint8_t origCopy;                   /* 0 = copy, 1 = original */
    uint8_t home;                       /* ignore */
    /* variable */
    uint8_t copyBit;                    /* 1 bit of the 72-bit copyright ID (transmitted as 1 bit per frame) */
    uint8_t copyStart;                  /* 1 = this bit starts the 72-bit ID, 0 = it does not */
    int     frameLength;                /* length of frame */
    int     bufferFull;                 /* number of 32-bit words left in enc buffer, 0x7FF = VBR */
    uint8_t numRawDataBlocks;           /* number of raw data blocks in frame */
    /* CRC */
    int     crcCheckWord;                     /* 16-bit CRC check word (present if protectBit == 0) */
} ADTSHeader_t;

typedef struct _ADIFHeader_t {
    uint8_t  copyBit;                    /* 0 = no copyright ID, 1 = 72-bit copyright ID follows immediately */
    uint8_t  origCopy;                   /* 0 = copy, 1 = original */
    uint8_t  home;                       /* ignore */
    uint8_t  bsType;                     /* bitstream type: 0 = CBR, 1 = VBR */
    int      bitRate;                    /* bitRate: CBR = bits/sec, VBR = peak bits/frame, 0 = unknown */
    uint8_t  numPCE;                     /* number of program config elements (max = 16) */
    int      bufferFull;                 /* bits left in bit reservoir */
    uint8_t  copyID[9];                  /* [ADIF_COPYID_SIZE] optional 72-bit copyright ID */
} ADIFHeader_t;

/* sizeof(ProgConfigElement_t) = 82 bytes (if KEEP_PCE_COMMENTS not defined) */
typedef struct _ProgConfigElement_t {
    uint8_t  elemInstTag;   /* element instance tag */
    uint8_t  profile;       /* 0 = main, 1 = LC, 2 = SSR, 3 = reserved */
    uint8_t  sampRateIdx;   /* sample rate index range = [0, 11] */
    uint8_t  numFCE;        /* number of front channel elements (max = 15) */
    uint8_t  numSCE;        /* number of side channel elements (max = 15) */
    uint8_t  numBCE;        /* number of back channel elements (max = 15) */
    uint8_t  numLCE;        /* number of LFE channel elements (max = 3) */
    uint8_t  numADE;        /* number of associated data elements (max = 7) */
    uint8_t  numCCE;        /* number of valid channel coupling elements (max = 15) */
    uint8_t  monoMixdown;   /* mono mixdown: bit 4 = present flag, bits 3-0 = element number */
    uint8_t  stereoMixdown; /* stereo mixdown: bit 4 = present flag, bits 3-0 = element number */
    uint8_t  matrixMixdown; /* bit 4 = present flag, bit 3 = unused,bits 2-1 = index, bit 0 = pseudo-surround enable */
    uint8_t  fce[15];       /* [MAX_NUM_FCE] front element channel pair: bit 4 = SCE/CPE flag, bits 3-0 = inst tag */
    uint8_t  sce[15];       /* [MAX_NUM_SCE] side element channel pair: bit 4 = SCE/CPE flag, bits 3-0 = inst tag */
    uint8_t  bce[15];       /* [MAX_NUM_BCE] back element channel pair: bit 4 = SCE/CPE flag, bits 3-0 = inst tag */
    uint8_t  lce[3];        /* [MAX_NUM_LCE] instance tag for LFE elements */
    uint8_t  ade[7];        /* [MAX_NUM_ADE] instance tag for ADE elements */
    uint8_t  cce[15];       /* [MAX_NUM_BCE] channel coupling elements: bit 4 = switching flag, bits 3-0 = inst tag */
} ProgConfigElement_t;

typedef struct _SBRHeader {
    int      count;

    uint8_t  ampRes;
    uint8_t  startFreq;
    uint8_t  stopFreq;
    uint8_t  crossOverBand;
    uint8_t  resBitsHdr;
    uint8_t  hdrExtra1;
    uint8_t  hdrExtra2;

    uint8_t  freqScale;
    uint8_t  alterScale;
    uint8_t  noiseBands;

    uint8_t  limiterBands;
    uint8_t  limiterGains;
    uint8_t  interpFreq;
    uint8_t  smoothMode;
} SBRHeader;

/* need one SBRGrid per channel, updated every frame */
typedef struct _SBRGrid {
    uint8_t  frameClass;
    uint8_t  ampResFrame;
    uint8_t  pointer;

    uint8_t  numEnv;                       /* L_E */
    uint8_t  envTimeBorder[5 + 1];   // [MAX_NUM_ENV+1] /* t_E */
    uint8_t  freqRes[5];             // [MAX_NUM_ENV]/* r */
    uint8_t  numNoiseFloors;                           /* L_Q */
    uint8_t  noiseTimeBorder[2 + 1]; // [MAX_NUM_NOISE_FLOORS+1] /* t_Q */

    uint8_t  numEnvPrev;
    uint8_t  numNoiseFloorsPrev;
    uint8_t  freqResPrev;
} SBRGrid;

/* need one SBRFreq per element (SCE/CPE/LFE), updated only on header reset */
typedef struct _SBRFreq {
    int      kStart;               /* k_x */
    int      nMaster;
    int      nHigh;
    int      nLow;
    int      nLimiter;             /* N_l */
    int      numQMFBands;          /* M */
    int      numNoiseFloorBands;   /* Nq */
    int      kStartPrev;
    int      numQMFBandsPrev;
    uint8_t  freqMaster[48 + 1];     // [MAX_QMF_BANDS + 1]      /* not necessary to save this  after derived tables are generated */
    uint8_t  freqHigh[48 + 1];       // [MAX_QMF_BANDS + 1]
    uint8_t  freqLow[48 / 2 + 1];    // [MAX_QMF_BANDS / 2 + 1]  /* nLow = nHigh - (nHigh >> 1) */
    uint8_t  freqNoise[5 + 1];       // [MAX_NUM_NOISE_FLOOR_BANDS+1]
    uint8_t  freqLimiter[48 / 2 + 5];// [MAX_QMF_BANDS / 2 + MAX_NUM_PATCHES]    /* max (intermediate) size = nLow + numPatches - 1 */

    uint8_t  numPatches;
    uint8_t  patchNumSubbands[5 + 1];  // [MAX_NUM_PATCHES + 1]
    uint8_t  patchStartSubband[5 + 1]; // [MAX_NUM_PATCHES + 1]
} SBRFreq;

typedef struct _SBRChan {
    int      reset;
    uint8_t  deltaFlagEnv[5];          // [MAX_NUM_ENV]
    uint8_t  deltaFlagNoise[2];        // [MAX_NUM_NOISE_FLOORS]
    int8_t   envDataQuant[5][48];      // [MAX_NUM_ENV][MAX_QMF_BANDS] /* range = [0, 127] */
    int8_t   noiseDataQuant[2][5];     // [MAX_NUM_NOISE_FLOORS][MAX_NUM_NOISE_FLOOR_BANDS]

    uint8_t  invfMode[2][5];           // [2][MAX_NUM_NOISE_FLOOR_BANDS] /* invfMode[0/1][band] = prev/curr */
    int      chirpFact[5];             // [MAX_NUM_NOISE_FLOOR_BANDS]  /* bwArray */
    uint8_t  addHarmonicFlag[2];       /* addHarmonicFlag[0/1] = prev/curr */
    uint8_t  addHarmonic[2][64];       /* addHarmonic[0/1][band] = prev/curr */

    int      gbMask[2];    /* gbMask[0/1] = XBuf[0-31]/XBuf[32-39] */
    int8_t   laPrev;

    int      noiseTabIndex;
    int      sinIndex;
    int      gainNoiseIndex;
    int      gTemp[5][48];  // [MAX_NUM_SMOOTH_COEFS][MAX_QMF_BANDS]
    int      qTemp[5][48];  // [MAX_NUM_SMOOTH_COEFS][MAX_QMF_BANDS]

} SBRChan;


/* state info struct for baseline (MPEG-4 LC) decoding */
typedef struct _PSInfoBase_t {
    int      dataCount;
    uint8_t  dataBuf[510]; // [DATA_BUF_SIZE]
    int      fillCount;
    uint8_t  fillBuf[269]; //[FILL_BUF_SIZE]
    /* state information which is the same throughout whole frame */
    int      nChans;
    int      useImpChanMap;
    int      sampRateIdx;
    /* state information which can be overwritten by subsequent elements within frame */
    ICSInfo_t  icsInfo[2]; // [MAX_NCHANS_ELEM]
    int      commonWin;
    short    scaleFactors[2][15*8]; // [MAX_NCHANS_ELEM][MAX_SF_BANDS]
    uint8_t  sfbCodeBook[2][15*8]; // [MAX_NCHANS_ELEM][MAX_SF_BANDS]
    int      msMaskPresent;
    uint8_t  msMaskBits[(15 * 8 + 7) >> 3]; // [MAX_MS_MASK_BYTES]
    int      pnsUsed[2]; // [MAX_NCHANS_ELEM]
    int      pnsLastVal;
    int      intensityUsed[2]; // [MAX_NCHANS_ELEM]
//    PulseInfo_t           pulseInfo[2]; // [MAX_NCHANS_ELEM]
    TNSInfo_t   tnsInfo[2]; // [MAX_NCHANS_ELEM]
    int      tnsLPCBuf[20]; // [MAX_TNS_ORDER]
    int      tnsWorkBuf[20]; //[MAX_TNS_ORDER]
    GainControlInfo_t     gainControlInfo[2]; // [MAX_NCHANS_ELEM]
    int      gbCurrent[2];  // [MAX_NCHANS_ELEM]
    int      coef[2][1024]; // [MAX_NCHANS_ELEM][AAC_MAX_NSAMPS]
#ifdef AAC_ENABLE_SBR
    int      sbrWorkBuf[2][1024]; // [MAX_NCHANS_ELEM][AAC_MAX_NSAMPS];
#endif
    /* state information which must be saved for each element and used in next frame */
    int      overlap[2][1024];  // [AAC_MAX_NCHANS][AAC_MAX_NSAMPS]
    int      prevWinShape[2]; // [AAC_MAX_NCHANS]
} PSInfoBase_t;

typedef struct _PSInfoSBR {
    /* save for entire file */
    int      frameCount;
    int      sampRateIdx;

    /* state info that must be saved for each channel */
    SBRHeader   sbrHdr[2];
    SBRGrid     sbrGrid[2];
    SBRFreq     sbrFreq[2];
    SBRChan     sbrChan[2];

    /* temp variables, no need to save between blocks */
    uint8_t  dataExtra;
    uint8_t  resBitsData;
    uint8_t  extendedDataPresent;
    int      extendedDataSize;

    int8_t   envDataDequantScale[2][5];  // [MAX_NCHANS_ELEM][MAX_NUM_ENV
    int      envDataDequant[2][5][48];   // [MAX_NCHANS_ELEM][MAX_NUM_ENV][MAX_QMF_BANDS
    int      noiseDataDequant[2][2][5];  // [MAX_NCHANS_ELEM][MAX_NUM_NOISE_FLOORS][MAX_NUM_NOISE_FLOOR_BANDS]

    int      eCurr[48];    // [MAX_QMF_BANDS]
    uint8_t  eCurrExp[48]; // [MAX_QMF_BANDS]
    uint8_t  eCurrExpMax;
    int8_t   la;

    int      crcCheckWord;
    int      couplingFlag;
    int      envBand;
    int      eOMGainMax;
    int      gainMax;
    int      gainMaxFBits;
    int      noiseFloorBand;
    int      qp1Inv;
    int      qqp1Inv;
    int      sMapped;
    int      sBand;
    int      highBand;

    int      sumEOrigMapped;
    int      sumECurrGLim;
    int      sumSM;
    int      sumQM;
    int      gLimBoost[48];
    int      qmLimBoost[48];
    int      smBoost[48];

    int      smBuf[48];
    int      qmLimBuf[48];
    int      gLimBuf[48];
    int      gLimFbits[48];

    int      gFiltLast[48];
    int      qFiltLast[48];

    /* large buffers */
    int      delayIdxQMFA[2];        // [AAC_MAX_NCHANS]
    int      delayQMFA[2][10 * 32];  // [AAC_MAX_NCHANS][DELAY_SAMPS_QMFA]
    int      delayIdxQMFS[2];        // [AAC_MAX_NCHANS]
    int      delayQMFS[2][10 * 128]; // [AAC_MAX_NCHANS][DELAY_SAMPS_QMFS]
    int      XBufDelay[2][8][64][2]; // [AAC_MAX_NCHANS][HF_GEN][64][2]
    int      XBuf[32+8][64][2];
} PSInfoSBR_t;

bool AACDecoder_AllocateBuffers(void);
int AACFlushCodec();
void AACDecoder_FreeBuffers(void);
bool AACDecoder_IsInit(void);
int AACFindSyncWord(uint8_t *buf, int nBytes);
int AACSetRawBlockParams(int copyLast, int nChans, int sampRateCore, int profile);
int AACDecode(uint8_t *inbuf, int *bytesLeft, short *outbuf);
int AACGetSampRate();
int AACGetChannels();
int AACGetID(); // 0-MPEG4, 1-MPEG2
uint8_t AACGetProfile(); // 0-Main, 1-LC, 2-SSR, 3-reserved
uint8_t AACGetFormat(); // 0-unknown 1-ADTS 2-ADIF, 3-RAW
int AACGetBitsPerSample();
int AACGetBitrate();
int AACGetOutputSamps();
int AACGetBitrate();
void DecodeLPCCoefs(int order, int res, int8_t *filtCoef, int *a, int *b);
int FilterRegion(int size, int dir, int order, int *audioCoef, int *a, int *hist);
int TNSFilter(int ch);
int DecodeSingleChannelElement();
int DecodeChannelPairElement();
int DecodeLFEChannelElement();
int DecodeDataStreamElement();
int DecodeProgramConfigElement(uint8_t idx);
int DecodeFillElement();
int DecodeNextElement(uint8_t **buf, int *bitOffset, int *bitsAvail);
void PreMultiply(int tabidx, int *zbuf1);
void PostMultiply(int tabidx, int *fft1);
void PreMultiplyRescale(int tabidx, int *zbuf1, int es);
void PostMultiplyRescale(int tabidx, int *fft1, int es);
void DCT4(int tabidx, int *coef, int gb);
void BitReverse(int *inout, int tabidx);
void R4FirstPass(int *x, int bg);
void R8FirstPass(int *x, int bg);
void R4Core(int *x, int bg, int gp, int *wtab);
void R4FFT(int tabidx, int *x);
void UnpackZeros(int nVals, int *coef);
void UnpackQuads(int cb, int nVals, int *coef);
void UnpackPairsNoEsc(int cb, int nVals, int *coef);
void UnpackPairsEsc(int cb, int nVals, int *coef);
void DecodeSpectrumLong(int ch);
void DecodeSpectrumShort(int ch);
void DecWindowOverlap(int *buf0, int *over0, short *pcm0, int nChans, int winTypeCurr, int winTypePrev);
void DecWindowOverlapLongStart(int *buf0, int *over0, short *pcm0, int nChans, int winTypeCurr, int winTypePrev);
void DecWindowOverlapLongStop(int *buf0, int *over0, short *pcm0, int nChans, int winTypeCurr, int winTypePrev);
void DecWindowOverlapShort(int *buf0, int *over0, short *pcm0, int nChans, int winTypeCurr, int winTypePrev);
int IMDCT(int ch, int chOut, short *outbuf);
void DecodeICSInfo(ICSInfo_t *icsInfo, int sampRateIdx);
void DecodeSectionData(int winSequence, int numWinGrp, int maxSFB, uint8_t *sfbCodeBook);
int DecodeOneScaleFactor();
void DecodeScaleFactors(int numWinGrp, int maxSFB, int globalGain, uint8_t *sfbCodeBook, short *scaleFactors);
void DecodePulseInfo(uint8_t ch);
void DecodeTNSInfo(int winSequence, TNSInfo_t *ti, int8_t *tnsCoef);
void DecodeGainControlInfo(int winSequence, GainControlInfo_t *gi);
void DecodeICS(int ch);
int DecodeNoiselessData(uint8_t **buf, int *bitOffset, int *bitsAvail, int ch);
int DecodeHuffmanScalar(const signed short *huffTab, const HuffInfo_t *huffTabInfo, unsigned int bitBuf, int32_t *val);
int UnpackADTSHeader(uint8_t **buf, int *bitOffset, int *bitsAvail);
int GetADTSChannelMapping(uint8_t *buf, int bitOffset, int bitsAvail);
int GetNumChannelsADIF(int nPCE);
int GetSampleRateIdxADIF(int nPCE);
int UnpackADIFHeader(uint8_t **buf, int *bitOffset, int *bitsAvail);
int SetRawBlockParams(int copyLast, int nChans, int sampRate, int profile);
int PrepareRawBlock();
int DequantBlock(int *inbuf, int nSamps, int scale);
int AACDequantize(int ch);
int DeinterleaveShortBlocks(int ch);
unsigned int Get32BitVal(unsigned int *last);
int InvRootR(int r);
int ScaleNoiseVector(int *coef, int nVals, int sf);
void GenerateNoiseVector(int *coef, int *last, int nVals);
void CopyNoiseVector(int *coefL, int *coefR, int nVals);
int PNS(int ch);
int GetSampRateIdx(int sampRate);
void StereoProcessGroup(int *coefL, int *coefR, const uint16_t *sfbTab, int msMaskPres, uint8_t *msMaskPtr,
        int msMaskOffset, int maxSFB, uint8_t *cbRight, short *sfRight, int *gbCurrent);
int StereoProcess();
int RatioPowInv(int a, int b, int c);
int SqrtFix(int q, int fBitsIn, int *fBitsOut);
int InvRNormalized(int r);
void BitReverse32(int *inout);
void R8FirstPass32(int *r0);
void R4Core32(int *r0);
void FFT32C(int *x);
void CVKernel1(int *XBuf, int *accBuf);
void CVKernel2(int *XBuf, int *accBuf);
void SetBitstreamPointer(int nBytes, uint8_t *buf);
inline void RefillBitstreamCache();
unsigned int GetBits(int nBits);
unsigned int GetBitsNoAdvance(int nBits);
void AdvanceBitstream(int nBits);
int CalcBitsUsed(uint8_t *startBuf, int startOffset);
void ByteAlignBitstream();
// SBR
void InitSBRState();
int DecodeSBRBitstream(int chBase);
int DecodeSBRData(int chBase, short *outbuf);
int FlushCodecSBR();
void BubbleSort(uint8_t *v, int nItems);
uint8_t VMin(uint8_t *v, int nItems);
uint8_t VMax(uint8_t *v, int nItems);
int CalcFreqMasterScaleZero(uint8_t *freqMaster, int alterScale, int k0, int k2);
int CalcFreqMaster(uint8_t *freqMaster, int freqScale, int alterScale, int k0, int k2);
int CalcFreqHigh(uint8_t *freqHigh, uint8_t *freqMaster, int nMaster, int crossOverBand);
int CalcFreqLow(uint8_t *freqLow, uint8_t *freqHigh, int nHigh);
int CalcFreqNoise(uint8_t *freqNoise, uint8_t *freqLow, int nLow, int kStart, int k2, int noiseBands);
int BuildPatches(uint8_t *patchNumSubbands, uint8_t *patchStartSubband, uint8_t *freqMaster, int nMaster, int k0,
        int kStart, int numQMFBands, int sampRateIdx);
int FindFreq(uint8_t *freq, int nFreq, uint8_t val);
void RemoveFreq(uint8_t *freq, int nFreq, int removeIdx);
int CalcFreqLimiter(uint8_t *freqLimiter, uint8_t *patchNumSubbands, uint8_t *freqLow, int nLow, int kStart,
        int limiterBands, int numPatches);
int CalcFreqTables(SBRHeader *sbrHdr, SBRFreq *sbrFreq, int sampRateIdx);
void EstimateEnvelope(SBRHeader *sbrHdr, SBRGrid *sbrGrid, SBRFreq *sbrFreq, int env);
int GetSMapped(SBRGrid *sbrGrid, SBRFreq *sbrFreq, SBRChan *sbrChan, int env, int band, int la);
void CalcMaxGain(SBRHeader *sbrHdr, SBRGrid *sbrGrid, SBRFreq *sbrFreq, int ch, int env, int lim, int fbitsDQ);
void CalcNoiseDivFactors(int q, int *qp1Inv, int *qqp1Inv);
void CalcComponentGains(SBRGrid *sbrGrid, SBRFreq *sbrFreq, SBRChan *sbrChan, int ch, int env, int lim, int fbitsDQ);
void ApplyBoost(SBRFreq *sbrFreq, int lim, int fbitsDQ);
void CalcGain(SBRHeader *sbrHdr, SBRGrid *sbrGrid, SBRFreq *sbrFreq, SBRChan *sbrChan, int ch, int env);
void MapHF(SBRHeader *sbrHdr, SBRGrid *sbrGrid, SBRFreq *sbrFreq, SBRChan *sbrChan, int env, int hfReset);
void AdjustHighFreq(SBRHeader *sbrHdr, SBRGrid *sbrGrid, SBRFreq *sbrFreq, SBRChan *sbrChan, int ch);
int CalcCovariance1(int *XBuf, int *p01reN, int *p01imN, int *p12reN, int *p12imN, int *p11reN, int *p22reN);
int CalcCovariance2(int *XBuf, int *p02reN, int *p02imN);
void CalcLPCoefs(int *XBuf, int *a0re, int *a0im, int *a1re, int *a1im, int gb);
void GenerateHighFreq(SBRGrid *sbrGrid, SBRFreq *sbrFreq, SBRChan *sbrChan, int ch);
int DecodeHuffmanScalar(const signed int *huffTab, const HuffInfo_t *huffTabInfo, unsigned int bitBuf, signed int *val);
int DecodeOneSymbol(int huffTabIndex);
int DequantizeEnvelope(int nBands, int ampRes, int8_t *envQuant, int *envDequant);
void DequantizeNoise(int nBands, int8_t *noiseQuant, int *noiseDequant);
void DecodeSBREnvelope(SBRGrid *sbrGrid, SBRFreq *sbrFreq, SBRChan *sbrChan, int ch);
void DecodeSBRNoise(SBRGrid *sbrGrid, SBRFreq *sbrFreq, SBRChan *sbrChan, int ch);
void UncoupleSBREnvelope(SBRGrid *sbrGrid, SBRFreq *sbrFreq, SBRChan *sbrChanR);
void UncoupleSBRNoise(SBRGrid *sbrGrid, SBRFreq *sbrFreq, SBRChan *sbrChanR);
void DecWindowOverlapNoClip(int *buf0, int *over0, int *out0, int winTypeCurr, int winTypePrev);
void DecWindowOverlapLongStartNoClip(int *buf0, int *over0, int *out0, int winTypeCurr, int winTypePrev);
void DecWindowOverlapLongStopNoClip(int *buf0, int *over0, int *out0, int winTypeCurr, int winTypePrev);
void DecWindowOverlapShortNoClip(int *buf0, int *over0, int *out0, int winTypeCurr, int winTypePrev);
void PreMultiply64(int *zbuf1);
void PostMultiply64(int *fft1, int nSampsOut);
void QMFAnalysisConv(int *cTab, int *delay, int dIdx, int *uBuf);
int QMFAnalysis(int *inbuf, int *delay, int *XBuf, int fBitsIn, int *delayIdx, int qmfaBands);
void QMFSynthesisConv(int *cPtr, int *delay, int dIdx, short *outbuf, int nChans);
void QMFSynthesis(int *inbuf, int *delay, int *delayIdx, int qmfsBands, short *outbuf, int nChans);
int UnpackSBRHeader(SBRHeader *sbrHdr);
void UnpackSBRGrid(SBRHeader *sbrHdr, SBRGrid *sbrGrid);
void UnpackDeltaTimeFreq(int numEnv, uint8_t *deltaFlagEnv, int numNoiseFloors, uint8_t *deltaFlagNoise);
void UnpackInverseFilterMode(int numNoiseFloorBands, uint8_t *mode);
void UnpackSinusoids(int nHigh, int addHarmonicFlag, uint8_t *addHarmonic);
void CopyCouplingGrid(SBRGrid *sbrGridLeft, SBRGrid *sbrGridRight);
void CopyCouplingInverseFilterMode(int numNoiseFloorBands, uint8_t *modeLeft, uint8_t *modeRight);
void UnpackSBRSingleChannel(int chBase);
void UnpackSBRChannelPair(int chBase);
