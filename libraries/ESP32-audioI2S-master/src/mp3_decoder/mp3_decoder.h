// based om helix mp3 decoder
#pragma once

#include "Arduino.h"
#include "assert.h"

static const uint8_t  m_HUFF_PAIRTABS          =32;
static const uint8_t  m_BLOCK_SIZE             =18;
static const uint8_t  m_NBANDS                 =32;
static const uint8_t  m_MAX_REORDER_SAMPS      =(192-126)*3;      // largest critical band for short blocks (see sfBandTable)
static const uint16_t m_VBUF_LENGTH            =17*2* m_NBANDS;    // for double-sized vbuf FIFO
static const uint8_t  m_MAX_SCFBD              =4;     // max scalefactor bands per channel
static const uint16_t m_MAINBUF_SIZE           =1940;
static const uint8_t  m_MAX_NGRAN              =2;     // max granules
static const uint8_t  m_MAX_NCHAN              =2;     // max channels
static const uint16_t m_MAX_NSAMP              =576;   // max samples per channel, per granule

enum {
    ERR_MP3_NONE =                  0,
    ERR_MP3_INDATA_UNDERFLOW =     -1,
    ERR_MP3_MAINDATA_UNDERFLOW =   -2,
    ERR_MP3_FREE_BITRATE_SYNC =    -3,
    ERR_MP3_OUT_OF_MEMORY =        -4,
    ERR_MP3_NULL_POINTER =         -5,
    ERR_MP3_INVALID_FRAMEHEADER =  -6,
    ERR_MP3_INVALID_SIDEINFO =     -7,
    ERR_MP3_INVALID_SCALEFACT =    -8,
    ERR_MP3_INVALID_HUFFCODES =    -9,
    ERR_MP3_INVALID_DEQUANTIZE =   -10,
    ERR_MP3_INVALID_IMDCT =        -11,
    ERR_MP3_INVALID_SUBBAND =      -12,

    ERR_UNKNOWN =                  -9999
};

typedef struct MP3FrameInfo {
    int bitrate;
    int nChans;
    int samprate;
    int bitsPerSample;
    int outputSamps;
    int layer;
    int version;
} MP3FrameInfo_t;

typedef struct SFBandTable {
    int/*short*/ l[23];
    int/*short*/ s[14];
} SFBandTable_t;

typedef struct BitStreamInfo {
    unsigned char *bytePtr;
    unsigned int iCache;
    int cachedBits;
    int nBytes;
} BitStreamInfo_t;

typedef enum {          /* map these to the corresponding 2-bit values in the frame header */
    Stereo = 0x00,      /* two independent channels, but L and R frames might have different # of bits */
    Joint = 0x01,       /* coupled channels - layer III: mix of M-S and intensity, Layers I/II: intensity and direct coding only */
    Dual = 0x02,        /* two independent channels, L and R always have exactly 1/2 the total bitrate */
    Mono = 0x03         /* one channel */
} StereoMode_t;

typedef enum {          /* map to 0,1,2 to make table indexing easier */
    MPEG1 =  0,
    MPEG2 =  1,
    MPEG25 = 2
} MPEGVersion_t;

typedef struct FrameHeader {
    int layer;              /* layer index (1, 2, or 3) */
    int crc;                /* CRC flag: 0 = disabled, 1 = enabled */
    int brIdx;              /* bitrate index (0 - 15) */
    int srIdx;              /* sample rate index (0 - 2) */
    int paddingBit;         /* padding flag: 0 = no padding, 1 = single pad byte */
    int privateBit;         /* unused */
    int modeExt;            /* used to decipher joint stereo mode */
    int copyFlag;           /* copyright flag: 0 = no, 1 = yes */
    int origFlag;           /* original flag: 0 = copy, 1 = original */
    int emphasis;           /* deemphasis mode */
    int CRCWord;            /* CRC word (16 bits, 0 if crc not enabled) */
} FrameHeader_t;

typedef struct SideInfoSub {
    int part23Length;       /* number of bits in main data */
    int nBigvals;           /* 2x this = first set of Huffman cw's (maximum amplitude can be > 1) */
    int globalGain;         /* overall gain for dequantizer */
    int sfCompress;         /* unpacked to figure out number of bits in scale factors */
    int winSwitchFlag;      /* window switching flag */
    int blockType;          /* block type */
    int mixedBlock;         /* 0 = regular block (all short or long), 1 = mixed block */
    int tableSelect[3];     /* index of Huffman tables for the big values regions */
    int subBlockGain[3];    /* subblock gain offset, relative to global gain */
    int region0Count;       /* 1+region0Count = num scale factor bands in first region of bigvals */
    int region1Count;       /* 1+region1Count = num scale factor bands in second region of bigvals */
    int preFlag;            /* for optional high frequency boost */
    int sfactScale;         /* scaling of the scalefactors */
    int count1TableSelect;  /* index of Huffman table for quad codewords */
} SideInfoSub_t;

typedef struct SideInfo {
    int mainDataBegin;
    int privateBits;
    int scfsi[m_MAX_NCHAN][m_MAX_SCFBD];                /* 4 scalefactor bands per channel */
} SideInfo_t;

typedef struct {
    int cbType;             /* pure long = 0, pure short = 1, mixed = 2 */
    int cbEndS[3];          /* number nonzero short cb's, per subbblock */
    int cbEndSMax;          /* max of cbEndS[] */
    int cbEndL;             /* number nonzero long cb's  */
} CriticalBandInfo_t;

typedef struct DequantInfo {
    int workBuf[m_MAX_REORDER_SAMPS];             /* workbuf for reordering short blocks */
} DequantInfo_t;

typedef struct HuffmanInfo {
    int huffDecBuf[m_MAX_NCHAN][m_MAX_NSAMP];       /* used both for decoded Huffman values and dequantized coefficients */
    int nonZeroBound[m_MAX_NCHAN];                /* number of coeffs in huffDecBuf[ch] which can be > 0 */
    int gb[m_MAX_NCHAN];                          /* minimum number of guard bits in huffDecBuf[ch] */
} HuffmanInfo_t;

typedef enum HuffTabType {
    noBits,
    oneShot,
    loopNoLinbits,
    loopLinbits,
    quadA,
    quadB,
    invalidTab
} HuffTabType_t;

typedef struct HuffTabLookup {
    int linBits;
    int  tabType; /*HuffTabType*/
} HuffTabLookup_t;

typedef struct IMDCTInfo {
    int outBuf[m_MAX_NCHAN][m_BLOCK_SIZE][m_NBANDS];  /* output of IMDCT */
    int overBuf[m_MAX_NCHAN][m_MAX_NSAMP / 2];      /* overlap-add buffer (by symmetry, only need 1/2 size) */
    int numPrevIMDCT[m_MAX_NCHAN];                /* how many IMDCT's calculated in this channel on prev. granule */
    int prevType[m_MAX_NCHAN];
    int prevWinSwitch[m_MAX_NCHAN];
    int gb[m_MAX_NCHAN];
} IMDCTInfo_t;

typedef struct BlockCount {
    int nBlocksLong;
    int nBlocksTotal;
    int nBlocksPrev;
    int prevType;
    int prevWinSwitch;
    int currWinSwitch;
    int gbIn;
    int gbOut;
} BlockCount_t;

typedef struct ScaleFactorInfoSub {    /* max bits in scalefactors = 5, so use char's to save space */
    char l[23];            /* [band] */
    char s[13][3];         /* [band][window] */
} ScaleFactorInfoSub_t;

typedef struct ScaleFactorJS { /* used in MPEG 2, 2.5 intensity (joint) stereo only */
    int intensityScale;
    int slen[4];
    int nr[4];
} ScaleFactorJS_t;

/* NOTE - could get by with smaller vbuf if memory is more important than speed
 *  (in Subband, instead of replicating each block in FDCT32 you would do a memmove on the
 *   last 15 blocks to shift them down one, a hardware style FIFO)
 */
typedef struct SubbandInfo {
    int vbuf[m_MAX_NCHAN * m_VBUF_LENGTH];      /* vbuf for fast DCT-based synthesis PQMF - double size for speed (no modulo indexing) */
    int vindex;                             /* internal index for tracking position in vbuf */
} SubbandInfo_t;

typedef struct MP3DecInfo {
    /* buffer which must be large enough to hold largest possible main_data section */
    unsigned char mainBuf[m_MAINBUF_SIZE];
    /* special info for "free" bitrate files */
    int freeBitrateFlag;
    int freeBitrateSlots;
    /* user-accessible info */
    int bitrate;
    int nChans;
    int samprate;
    int nGrans;             /* granules per frame */
    int nGranSamps;         /* samples per granule */
    int nSlots;
    int layer;

    int mainDataBegin;
    int mainDataBytes;
    int part23Length[m_MAX_NGRAN][m_MAX_NCHAN];
} MP3DecInfo_t;




/* format = Q31
 * #define M_PI 3.14159265358979323846
 * double u = 2.0 * M_PI / 9.0;
 * float c0 = sqrt(3.0) / 2.0;
 * float c1 = cos(u);
 * float c2 = cos(2*u);
 * float c3 = sin(u);
 * float c4 = sin(2*u);
 */

const int c9_0 = 0x6ed9eba1;
const int c9_1 = 0x620dbe8b;
const int c9_2 = 0x163a1a7e;
const int c9_3 = 0x5246dd49;
const int c9_4 = 0x7e0e2e32;



const int c3_0 = 0x6ed9eba1; /* format = Q31, cos(pi/6) */
const int c6[3] = { 0x7ba3751d, 0x5a82799a, 0x2120fb83 }; /* format = Q31, cos(((0:2) + 0.5) * (pi/6)) */

/* format = Q31
 * cos(((0:8) + 0.5) * (pi/18))
 */
const uint32_t c18[9] = { 0x7f834ed0, 0x7ba3751d, 0x7401e4c1, 0x68d9f964, 0x5a82799a, 0x496af3e2, 0x36185aee, 0x2120fb83, 0x0b27eb5c};

/* scale factor lengths (num bits) */
const char m_SFLenTab[16][2] = { {0, 0}, {0, 1}, {0, 2}, {0, 3}, {3, 0}, {1, 1}, {1, 2}, {1, 3},
                                 {2, 1}, {2, 2}, {2, 3}, {3, 1}, {3, 2}, {3, 3}, {4, 2}, {4, 3}};

/* NRTab[size + 3*is_right][block type][partition]
 *   block type index: 0 = (bt0,bt1,bt3), 1 = bt2 non-mixed, 2 = bt2 mixed
 *   partition: scale factor groups (sfb1 through sfb4)
 * for block type = 2 (mixed or non-mixed) / by 3 is rolled into this table
 *   (for 3 short blocks per long block)
 * see 2.4.3.2 in MPEG 2 (low sample rate) spec
 * stuff rolled into this table:
 *   NRTab[x][1][y]   --> (NRTab[x][1][y])   / 3
 *   NRTab[x][2][>=1] --> (NRTab[x][2][>=1]) / 3  (first partition is long block)
 */
const char NRTab[6][3][4] = {
    {{ 6,  5, 5, 5}, {3, 3, 3, 3}, {6, 3, 3, 3}},
    {{ 6,  5, 7, 3}, {3, 3, 4, 2}, {6, 3, 4, 2}},
    {{11, 10, 0, 0}, {6, 6, 0, 0}, {6, 3, 6, 0}},
    {{ 7,  7, 7, 0}, {4, 4, 4, 0}, {6, 5, 4, 0}},
    {{ 6,  6, 6, 3}, {4, 3, 3, 2}, {6, 4, 3, 2}},
    {{ 8,  8, 5, 0}, {5, 4, 3, 0}, {6, 6, 3, 0}}
};



/* optional pre-emphasis for high-frequency scale factor bands */
const char preTab[22] = { 0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,2,2,3,3,3,2,0 };

/* pow(2,-i/4) for i=0..3, Q31 format */
const int pow14[4] PROGMEM = {
    0x7fffffff, 0x6ba27e65, 0x5a82799a, 0x4c1bf829
};


/*
 * Minimax polynomial approximation to pow(x, 4/3), over the range
 *  poly43lo: x = [0.5, 0.7071]
 *  poly43hi: x = [0.7071, 1.0]
 *
 * Relative error < 1E-7
 * Coefs are scaled by 4, 2, 1, 0.5, 0.25
 */
const unsigned int poly43lo[5] PROGMEM = { 0x29a0bda9, 0xb02e4828, 0x5957aa1b, 0x236c498d, 0xff581859 };
const unsigned int poly43hi[5] PROGMEM = { 0x10852163, 0xd333f6a4, 0x46e9408b, 0x27c2cef0, 0xfef577b4 };

/* pow(2, i*4/3) as exp and frac */
const int pow2exp[8] PROGMEM = { 14, 13, 11, 10, 9, 7, 6, 5 };

const int pow2frac[8] PROGMEM = {
    0x6597fa94, 0x50a28be6, 0x7fffffff, 0x6597fa94,
    0x50a28be6, 0x7fffffff, 0x6597fa94, 0x50a28be6
};

const uint16_t m_HUFF_OFFSET_01=  0;
const uint16_t m_HUFF_OFFSET_02=  9 + m_HUFF_OFFSET_01;
const uint16_t m_HUFF_OFFSET_03= 65 + m_HUFF_OFFSET_02;
const uint16_t m_HUFF_OFFSET_05= 65 + m_HUFF_OFFSET_03;
const uint16_t m_HUFF_OFFSET_06=257 + m_HUFF_OFFSET_05;
const uint16_t m_HUFF_OFFSET_07=129 + m_HUFF_OFFSET_06;
const uint16_t m_HUFF_OFFSET_08=110 + m_HUFF_OFFSET_07;
const uint16_t m_HUFF_OFFSET_09=280 + m_HUFF_OFFSET_08;
const uint16_t m_HUFF_OFFSET_10= 93 + m_HUFF_OFFSET_09;
const uint16_t m_HUFF_OFFSET_11=320 + m_HUFF_OFFSET_10;
const uint16_t m_HUFF_OFFSET_12=296 + m_HUFF_OFFSET_11;
const uint16_t m_HUFF_OFFSET_13=185 + m_HUFF_OFFSET_12;
const uint16_t m_HUFF_OFFSET_15=497 + m_HUFF_OFFSET_13;
const uint16_t m_HUFF_OFFSET_16=580 + m_HUFF_OFFSET_15;
const uint16_t m_HUFF_OFFSET_24=651 + m_HUFF_OFFSET_16;

const int huffTabOffset[m_HUFF_PAIRTABS] PROGMEM = {
    0,                   m_HUFF_OFFSET_01,    m_HUFF_OFFSET_02,    m_HUFF_OFFSET_03,
    0,                   m_HUFF_OFFSET_05,    m_HUFF_OFFSET_06,    m_HUFF_OFFSET_07,
    m_HUFF_OFFSET_08,    m_HUFF_OFFSET_09,    m_HUFF_OFFSET_10,    m_HUFF_OFFSET_11,
    m_HUFF_OFFSET_12,    m_HUFF_OFFSET_13,    0,                   m_HUFF_OFFSET_15,
    m_HUFF_OFFSET_16,    m_HUFF_OFFSET_16,    m_HUFF_OFFSET_16,    m_HUFF_OFFSET_16,
    m_HUFF_OFFSET_16,    m_HUFF_OFFSET_16,    m_HUFF_OFFSET_16,    m_HUFF_OFFSET_16,
    m_HUFF_OFFSET_24,    m_HUFF_OFFSET_24,    m_HUFF_OFFSET_24,    m_HUFF_OFFSET_24,
    m_HUFF_OFFSET_24,    m_HUFF_OFFSET_24,    m_HUFF_OFFSET_24,    m_HUFF_OFFSET_24,};

const HuffTabLookup_t huffTabLookup[m_HUFF_PAIRTABS] PROGMEM = {
    { 0,  noBits },
    { 0,  oneShot },
    { 0,  oneShot },
    { 0,  oneShot },
    { 0,  invalidTab },
    { 0,  oneShot },
    { 0,  oneShot },
    { 0,  loopNoLinbits },
    { 0,  loopNoLinbits },
    { 0,  loopNoLinbits },
    { 0,  loopNoLinbits },
    { 0,  loopNoLinbits },
    { 0,  loopNoLinbits },
    { 0,  loopNoLinbits },
    { 0,  invalidTab },
    { 0,  loopNoLinbits },
    { 1,  loopLinbits },
    { 2,  loopLinbits },
    { 3,  loopLinbits },
    { 4,  loopLinbits },
    { 6,  loopLinbits },
    { 8,  loopLinbits },
    { 10, loopLinbits },
    { 13, loopLinbits },
    { 4,  loopLinbits },
    { 5,  loopLinbits },
    { 6,  loopLinbits },
    { 7,  loopLinbits },
    { 8,  loopLinbits },
    { 9,  loopLinbits },
    { 11, loopLinbits },
    { 13, loopLinbits },
};


const int quadTabOffset[2] PROGMEM = {0, 64};
const int quadTabMaxBits[2] PROGMEM = {6, 4};

/* indexing = [version][samplerate index]
 * sample rate of frame (Hz)
 */
const int samplerateTab[3][3] PROGMEM = {
        { 44100, 48000, 32000 }, /* MPEG-1 */
        { 22050, 24000, 16000 }, /* MPEG-2 */
        { 11025, 12000, 8000  }, /* MPEG-2.5 */
};



/* indexing = [version][layer]
 * number of samples in one frame (per channel)
 */
const int/*short*/samplesPerFrameTab[3][3] PROGMEM = { { 384, 1152, 1152 }, /* MPEG1 */
{ 384, 1152, 576 }, /* MPEG2 */
{ 384, 1152, 576 }, /* MPEG2.5 */
};

/* layers 1, 2, 3 */
const short bitsPerSlotTab[3] = { 32, 8, 8 };

/* indexing = [version][mono/stereo]
 * number of bytes in side info section of bitstream
 */
const int/*short*/sideBytesTab[3][2] PROGMEM = { { 17, 32 }, /* MPEG-1:   mono, stereo */
{ 9, 17 }, /* MPEG-2:   mono, stereo */
{ 9, 17 }, /* MPEG-2.5: mono, stereo */
};

/* indexing = [version][sampleRate][long (.l) or short (.s) block]
 *   sfBandTable[v][s].l[cb] = index of first bin in critical band cb (long blocks)
 *   sfBandTable[v][s].s[cb] = index of first bin in critical band cb (short blocks)
 */
const SFBandTable_t sfBandTable[3][3] PROGMEM = {
    { /* MPEG-1 (44, 48, 32 kHz) */
        {   {0, 4, 8, 12, 16, 20, 24, 30, 36, 44, 52,  62,  74,  90, 110, 134, 162, 196, 238, 288, 342, 418, 576 },
            {0, 4, 8, 12, 16, 22, 30, 40, 52, 66, 84, 106, 136, 192}    },
        {   {0, 4, 8, 12, 16, 20, 24, 30, 36, 42, 50,  60,  72,  88, 106, 128, 156, 190, 230, 276, 330, 384, 576 },
            {0, 4, 8, 12, 16, 22, 28, 38, 50, 64, 80, 100, 126, 192}    },
        {   {0, 4, 8, 12, 16, 20, 24, 30, 36, 44,  54,  66,  82, 102, 126, 156, 194, 240, 296, 364, 448, 550, 576 },
            {0, 4, 8, 12, 16, 22, 30, 42, 58, 78, 104, 138, 180, 192}   }   },
    { /* MPEG-2 (22, 24, 16 kHz) */
        {   {0, 6, 12, 18, 24, 30, 36, 44, 54, 66,  80,  96, 116, 140, 168, 200, 238, 284, 336, 396, 464, 522, 576 },
            {0, 4,  8, 12, 18, 24, 32, 42, 56, 74, 100, 132, 174, 192}  },
        {   {0, 6, 12, 18, 24, 30, 36, 44, 54, 66,  80,  96, 114, 136, 162, 194, 232, 278, 332, 394, 464, 540, 576 },
            {0, 4,  8, 12, 18, 26, 36, 48, 62, 80, 104, 136, 180, 192}  },
        {   {0, 6, 12, 18, 24, 30, 36, 44, 54, 66, 80, 96, 116, 140, 168, 200, 238, 284, 336, 396, 464, 522, 576 },
            {0, 4, 8, 12, 18, 26, 36, 48, 62, 80, 104, 134, 174, 192}   },  },
    { /* MPEG-2.5 (11, 12, 8 kHz) */
        {   {0, 6, 12, 18, 24, 30, 36, 44, 54, 66,  80,  96, 116, 140, 168, 200, 238, 284, 336, 396, 464, 522, 576 },
            {0, 4,  8, 12, 18, 26, 36, 48, 62, 80, 104, 134, 174, 192 }  },
        {   {0, 6, 12, 18, 24, 30, 36, 44, 54, 66,  80,  96, 116, 140, 168, 200, 238, 284, 336, 396, 464, 522, 576 },
            {0, 4,  8, 12, 18, 26, 36, 48, 62, 80, 104, 134, 174, 192 }  },
        {   {0, 12, 24, 36, 48, 60, 72, 88, 108, 132, 160, 192, 232, 280, 336, 400, 476, 566, 568, 570, 572, 574, 576 },
            {0,  8, 16, 24, 36, 52, 72, 96, 124, 160, 162, 164, 166, 192 }   },   },
};


/* indexing = [intensity scale on/off][left/right]
 * format = Q30, range = [0.0, 1.414]
 *
 * illegal intensity position scalefactors (see comments on ISFMpeg1)
 */
const int ISFIIP[2][2] PROGMEM = {
    {0x40000000, 0x00000000}, /* mid-side off */
    {0x40000000, 0x40000000}, /* mid-side on */
};

const unsigned char uniqueIDTab[8] = {0x5f, 0x4b, 0x43, 0x5f, 0x5f, 0x4a, 0x52, 0x5f};

/* anti-alias coefficients - see spec Annex B, table 3-B.9
 *   csa[0][i] = CSi, csa[1][i] = CAi
 * format = Q31
 */
const uint32_t csa[8][2] PROGMEM = {
    {0x6dc253f0, 0xbe2500aa},
    {0x70dcebe4, 0xc39e4949},
    {0x798d6e73, 0xd7e33f4a},
    {0x7ddd40a7, 0xe8b71176},
    {0x7f6d20b7, 0xf3e4fe2f},
    {0x7fe47e40, 0xfac1a3c7},
    {0x7ffcb263, 0xfe2ebdc6},
    {0x7fffc694, 0xff86c25d},
};

/* format = Q30, right shifted by 12 (sign bits only in top 12 - undo this when rounding to short)
 *   this is to enable early-terminating multiplies on ARM
 * range = [-1.144287109, 1.144989014]
 * max gain of filter (per output sample) ~= 2.731
 *
 * new (properly sign-flipped) values
 *  - these actually are correct to 32 bits, (floating-pt coefficients in spec
 *      chosen such that only ~20 bits are required)
 *
 * Reordering - see table 3-B.3 in spec (appendix B)
 *
 * polyCoef[i] =
 *   D[ 0, 32, 64, ... 480],   i = [  0, 15]
 *   D[ 1, 33, 65, ... 481],   i = [ 16, 31]
 *   D[ 2, 34, 66, ... 482],   i = [ 32, 47]
 *     ...
 *   D[15, 47, 79, ... 495],   i = [240,255]
 *
 * also exploits symmetry: D[i] = -D[512 - i], for i = [1, 255]
 *
 * polyCoef[256, 257, ... 263] are for special case of sample 16 (out of 0)
 *   see PolyphaseStereo() and PolyphaseMono()
 */

// prototypes
bool MP3Decoder_AllocateBuffers(void);
void MP3Decoder_FreeBuffers();
int  MP3Decode( unsigned char *inbuf, int *bytesLeft, short *outbuf, int useSize);
void MP3GetLastFrameInfo();
int  MP3GetNextFrameInfo(unsigned char *buf);
int  MP3FindSyncWord(unsigned char *buf, int nBytes);
int  MP3GetSampRate();
int  MP3GetChannels();
int  MP3GetBitsPerSample();
int  MP3GetBitrate();
int  MP3GetOutputSamps();

//internally used
void MP3Decoder_ClearBuffer(void);
void PolyphaseMono(short *pcm, int *vbuf, const uint32_t *coefBase);
void PolyphaseStereo(short *pcm, int *vbuf, const uint32_t *coefBase);
void SetBitstreamPointer(BitStreamInfo_t *bsi, int nBytes, unsigned char *buf);
unsigned int GetBits(BitStreamInfo_t *bsi, int nBits);
int CalcBitsUsed(BitStreamInfo_t *bsi, unsigned char *startBuf, int startOffset);
int DequantChannel(int *sampleBuf, int *workBuf, int *nonZeroBound, SideInfoSub_t *sis, ScaleFactorInfoSub_t *sfis, CriticalBandInfo_t *cbi);
void MidSideProc(int x[m_MAX_NCHAN][m_MAX_NSAMP], int nSamps, int mOut[2]);
void IntensityProcMPEG1(int x[m_MAX_NCHAN][m_MAX_NSAMP], int nSamps, ScaleFactorInfoSub_t *sfis,	CriticalBandInfo_t *cbi, int midSideFlag, int mixFlag, int mOut[2]);
void IntensityProcMPEG2(int x[m_MAX_NCHAN][m_MAX_NSAMP], int nSamps, ScaleFactorInfoSub_t *sfis, CriticalBandInfo_t *cbi, ScaleFactorJS_t *sfjs, int midSideFlag, int mixFlag, int mOut[2]);
void FDCT32(int *x, int *d, int offset, int oddBlock, int gb);// __attribute__ ((section (".data")));
void FreeBuffers();
int CheckPadBit();
int UnpackFrameHeader(unsigned char *buf);
int UnpackSideInfo(unsigned char *buf);
int DecodeHuffman( unsigned char *buf, int *bitOffset, int huffBlockBits, int gr, int ch);
int MP3Dequantize( int gr);
int IMDCT( int gr, int ch);
int UnpackScaleFactors( unsigned char *buf, int *bitOffset, int bitsAvail, int gr, int ch);
int Subband(short *pcmBuf);
short ClipToShort(int x, int fracBits);
void RefillBitstreamCache(BitStreamInfo_t *bsi);
void UnpackSFMPEG1(BitStreamInfo_t *bsi, SideInfoSub_t *sis, ScaleFactorInfoSub_t *sfis, int *scfsi, int gr, ScaleFactorInfoSub_t *sfisGr0);
void UnpackSFMPEG2(BitStreamInfo_t *bsi, SideInfoSub_t *sis, ScaleFactorInfoSub_t *sfis, int gr, int ch, int modeExt, ScaleFactorJS_t *sfjs);
int MP3FindFreeSync(unsigned char *buf, unsigned char firstFH[4], int nBytes);
void MP3ClearBadFrame( short *outbuf);
int DecodeHuffmanPairs(int *xy, int nVals, int tabIdx, int bitsLeft, unsigned char *buf, int bitOffset);
int DecodeHuffmanQuads(int *vwxy, int nVals, int tabIdx, int bitsLeft, unsigned char *buf, int bitOffset);
int DequantBlock(int *inbuf, int *outbuf, int num, int scale);
void AntiAlias(int *x, int nBfly);
void WinPrevious(int *xPrev, int *xPrevWin, int btPrev);
int FreqInvertRescale(int *y, int *xPrev, int blockIdx, int es);
void idct9(int *x);
int IMDCT36(int *xCurr, int *xPrev, int *y, int btCurr, int btPrev, int blockIdx, int gb);
void imdct12(int *x, int *out);
int IMDCT12x3(int *xCurr, int *xPrev, int *y, int btPrev, int blockIdx, int gb);
int HybridTransform(int *xCurr, int *xPrev, int y[m_BLOCK_SIZE][m_NBANDS], SideInfoSub_t *sis, BlockCount_t *bc);
inline uint64_t SAR64(uint64_t x, int n) {return x >> n;}
inline int MULSHIFT32(int x, int y) { int z; z = (uint64_t) x * (uint64_t) y >> 32; return z;}
inline uint64_t MADD64(uint64_t sum64, int x, int y) {sum64 += (uint64_t) x * (uint64_t) y; return sum64;}/* returns 64-bit value in [edx:eax] */
inline uint64_t xSAR64(uint64_t x, int n){return x >> n;}
inline int FASTABS(int x){ return __builtin_abs(x);} //xtensa has a fast abs instruction //fb
#define CLZ(x) __builtin_clz(x) //fb
