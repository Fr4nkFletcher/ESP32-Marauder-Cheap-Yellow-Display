/*
 * aac_decoder.cpp
 * libhelix_HAACDECODER
 *
 *  Created on: 26.10.2018
 *  Updated on: 27.05.2022
 ************************************************************************************/

#include "aac_decoder.h"

const uint32_t SQRTHALF             = 0x5a82799a;    /* sqrt(0.5), format = Q31 */
const uint32_t Q28_2                = 0x20000000;    /* Q28: 2.0 */
const uint32_t Q28_15               = 0x30000000;    /* Q28: 1.5 */
const uint8_t  NUM_ITER_IRN         = 5;
const uint8_t  NUM_TERMS_RPI        = 5;
const uint32_t LOG2_EXP_INV         = 0x58b90bfc;    /* 1/log2(e), Q31 */
const uint8_t  SF_OFFSET            = 100;
const uint8_t  AAC_PROFILE_LC       = 1;
const uint8_t  NUM_TIME_SLOTS       = 16;
const uint8_t  SAMPLES_PER_SLOT     = 2;             /* RATE in spec */
const uint8_t  SYNCWORDH            = 0xff;          /* 12-bit syncword */
const uint8_t  SYNCWORDL            = 0xf0;
const uint8_t  NUM_SAMPLE_RATES     = 12;
const uint8_t  NUM_DEF_CHAN_MAPS    = 8;
const uint32_t NSAMPS_LONG          = 1024;
const uint8_t  NSAMPS_SHORT         = 128;
const uint8_t  NUM_SYN_ID_BITS      = 3;
const uint8_t  NUM_INST_TAG_BITS    = 4;
const uint8_t  NWINDOWS_LONG        = 1;
const uint8_t  NWINDOWS_SHORT       = 8;
const uint8_t  AAC_MAX_NCHANS       = 2;             /* set to default max number of channels  */
const uint16_t AAC_MAX_NSAMPS       = 1024;
const uint8_t  MAX_NCHANS_ELEM      = 2;             /* max number of channels in any single bitstream element */
const uint8_t  MAX_NUM_PCE_ADIF     = 16;
const uint8_t  ADIF_COPYID_SIZE     = 9;
const uint8_t  HUFFTAB_SPEC_OFFSET  = 1;
const uint8_t  FBITS_OUT_DQ_OFF     = 20 - 15;       /* (FBITS_OUT_DQ - SF_DQ_OFFSET)  */
const uint8_t  GBITS_IN_DCT4        = 4;             /* min guard bits in for DCT4 */
const uint8_t  FBITS_LOST_DCT4      = 1;             /* number of fraction bits lost (>> out) in DCT-IV */
const uint8_t  FBITS_OUT_IMDCT      = 3;
const uint8_t  NUM_IMDCT_SIZES      = 2;
const uint8_t  FBITS_LPC_COEFS      = 20;
const uint8_t  NUM_ITER_INVSQRT     = 4;
const uint32_t X0_COEF_2            = 0xc0000000;    /* Q29: -2.0 */
const uint32_t X0_OFF_2             = 0x60000000;    /* Q29:  3.0 */
const uint32_t Q26_3                = 0x0c000000;    /* Q26:  3.0 */
const uint8_t  EXT_SBR_DATA         = 0x0d;
const uint8_t  EXT_SBR_DATA_CRC     = 0x0e;
const uint8_t  NUM_SAMPLE_RATES_SBR = 9;             /* downsampled (single-rate) mode unsupported */
const uint8_t  MAX_NUM_PATCHES      = 5;
const uint8_t  MAX_QMF_BANDS        = 48;            /* max QMF subbands covered by SBR (4.6.18.3.6) */
const uint8_t  MAX_NUM_ENV          = 5;
const uint8_t  NUM_QMF_DELAY_BUFS   = 10;
const uint8_t  FBITS_IN_QMFA        = 14;
const uint8_t  NOISE_FLOOR_OFFSET   = 6;
const uint8_t  FBITS_OUT_DQ_NOISE   = 24;            /* range of Q_orig = [2^-24, 2^6] */
const uint8_t  FBITS_LOST_QMFA      = (1 + 2 + 3 + 2 + 1);
const uint8_t  FBITS_OUT_QMFA       = (FBITS_IN_QMFA - FBITS_LOST_QMFA);
const uint8_t  FBITS_IN_QMFS        = FBITS_OUT_QMFA;
const uint8_t  FBITS_LOST_DCT4_64   = (2 + 3 + 2);   /* 2 in premul, 3 in FFT, 2 in postmul */
const uint8_t  FBITS_OUT_QMFS       = (FBITS_IN_QMFS - FBITS_LOST_DCT4_64 + 6 - 1);
const uint8_t  RND_VAL              = (1 << (FBITS_OUT_QMFS-1));
const uint8_t  HF_ADJ               = 2;
const uint8_t  HF_GEN               = 8;
const uint8_t  FBITS_LPCOEFS        = 29;            /* Q29 for range of (-4, 4) */
const uint32_t MAG_16               = (16 * (1 << (32 - (2*(32-FBITS_LPCOEFS)))));     /* i.e. 16 in Q26 format */
const uint32_t RELAX_COEF           = 0x7ffff79c;    /* 1.0 / (1.0 + 1e-6), Q31 */
const uint8_t  MAX_NUM_SMOOTH_COEFS = 5;
const uint8_t  FBITS_OUT_DQ_ENV     = 29;            /* dequantized env scalefactors are Q(29 - envDataDequantScale) */
const uint8_t  FBITS_GLIM_BOOST     = 24;
const uint8_t  FBITS_QLIM_BOOST     = 14;
const uint8_t  MIN_GBITS_IN_QMFS    = 2;
const uint16_t nmdctTab[2]          = {128, 1024};
const uint8_t  postSkip[2]          = {15, 1};
const uint16_t nfftTab[2]           = {64, 512};
const uint8_t  nfftlog2Tab[2]       = {6, 9};
const uint8_t  cos4sin4tabOffset[2] = {0, 128};

PSInfoBase_t        *m_PSInfoBase;
AACDecInfo_t        *m_AACDecInfo;
AACFrameInfo_t       m_AACFrameInfo;
ADTSHeader_t         m_fhADTS;
ADIFHeader_t         m_fhADIF;
ProgConfigElement_t *m_pce[16];
PulseInfo_t          m_pulseInfo[2]; // [MAX_NCHANS_ELEM]
aac_BitStreamInfo_t  m_aac_BitStreamInfo;
PSInfoSBR_t         *m_PSInfoSBR;

//----------------------------------------------------------------------------------------------------------------------
inline int MULSHIFT32(int x, int y){
    int z; z = (int64_t)x * (int64_t)y >> 32;
    return z;
}
inline int CLZ(int x){
#ifdef __XTENSA__
    return __builtin_clz(x);
#else
    int numZeros;
    if(!x) return 32; /* count leading zeros with binary search (function should be 17 ARM instructions total) */
    numZeros = 1;
    if (!((unsigned int)x >> 16))    { numZeros += 16; x <<= 16; }
    if (!((unsigned int)x >> 24))    { numZeros +=  8; x <<=  8; }
    if (!((unsigned int)x >> 28))    { numZeros +=  4; x <<=  4; }
    if (!((unsigned int)x >> 30))    { numZeros +=  2; x <<=  2; }
    numZeros -= ((unsigned int)x >> 31);
    return numZeros;
#endif
}
inline int FASTABS(int x){
#ifdef __XTENSA__ //fb
    return __builtin_abs(x);
#else
    int sign;
    sign = x >> (sizeof(int) * 8 - 1);
    x ^= sign; x -= sign; return x;
#endif
}
inline int64_t MADD64(int64_t sum64, int x, int y){
    sum64 += (int64_t)x * (int64_t)y;
    return sum64;
}
inline short CLIPTOSHORT(int x){
#ifdef __XTENSA__ //fb
    asm ("clamps %0, %1, 15" : "=a" (x) : "a" (x) : );
    return x;
#else
    int sign; /* clip to [-32768, 32767] */
    sign = x >> 31;
    if (sign != (x >> 15)) x = sign ^ ((1 << 15) - 1);
    return (short)x;
#endif
}
inline int CLIP_2N(int y, int n){
#ifdef __XTENSA__ //fb
    int x = 1 << n; \
    if (y < -x) y = -x; \
    x--; \
    if (y > x) y = x; \
    return y;
#else
    int sign = y >> 31;
    if(sign != (y >> n))
        y = sign ^ ((1 << n) - 1);
    return y;
#endif
}
inline int CLIP_2N_SHIFT30(int y, int n){
    int sign = y >> 31;
    if(sign != (y >> (30 - n)))
            y = sign ^ (0x3fffffff);
    else
        y = (y << n);
    return y;
}
//----------------------------------------------------------------------------------------------------------------------

const uint32_t cos4sin4tab[128 + 1024] PROGMEM = {
/* 128 - format = Q30 * 2^-7 */
0xbf9bc731, 0xff9b783c, 0xbed5332c, 0xc002c697, 0xbe112251, 0xfe096c8d, 0xbd4f9c30, 0xc00f1c4a,
0xbc90a83f, 0xfc77ae5e, 0xbbd44dd9, 0xc0254e27, 0xbb1a9443, 0xfae67ba2, 0xba6382a6, 0xc04558c0,
0xb9af200f, 0xf9561237, 0xb8fd7373, 0xc06f3726, 0xb84e83ac, 0xf7c6afdc, 0xb7a25779, 0xc0a2e2e3,
0xb6f8f57c, 0xf6389228, 0xb652643e, 0xc0e05401, 0xb5aeaa2a, 0xf4abf67e, 0xb50dcd90, 0xc1278104,
0xb46fd4a4, 0xf3211a07, 0xb3d4c57c, 0xc1785ef4, 0xb33ca614, 0xf19839a6, 0xb2a77c49, 0xc1d2e158,
0xb2154dda, 0xf01191f3, 0xb186206b, 0xc236fa3b, 0xb0f9f981, 0xee8d5f29, 0xb070de82, 0xc2a49a2e,
0xafead4b9, 0xed0bdd25, 0xaf67e14f, 0xc31bb049, 0xaee80952, 0xeb8d475b, 0xae6b51ae, 0xc39c2a2f,
0xadf1bf34, 0xea11d8c8, 0xad7b5692, 0xc425f410, 0xad081c5a, 0xe899cbf1, 0xac9814fd, 0xc4b8f8ad,
0xac2b44cc, 0xe7255ad1, 0xabc1aff9, 0xc555215a, 0xab5b5a96, 0xe5b4bed8, 0xaaf84896, 0xc5fa5603,
0xaa987dca, 0xe44830dd, 0xaa3bfde3, 0xc6a87d2d, 0xa9e2cc73, 0xe2dfe917, 0xa98cece9, 0xc75f7bfe,
0xa93a6296, 0xe17c1f15, 0xa8eb30a7, 0xc81f363d, 0xa89f5a2b, 0xe01d09b4, 0xa856e20e, 0xc8e78e5b,
0xa811cb1b, 0xdec2df18, 0xa7d017fc, 0xc9b86572, 0xa791cb39, 0xdd6dd4a2, 0xa756e73a, 0xca919b4e,
0xa71f6e43, 0xdc1e1ee9, 0xa6eb6279, 0xcb730e70, 0xa6bac5dc, 0xdad3f1b1, 0xa68d9a4c, 0xcc5c9c14,
0xa663e188, 0xd98f7fe6, 0xa63d9d2b, 0xcd4e2037, 0xa61aceaf, 0xd850fb8e, 0xa5fb776b, 0xce47759a,
0xa5df9894, 0xd71895c9, 0xa5c7333e, 0xcf4875ca, 0xa5b2485a, 0xd5e67ec1, 0xa5a0d8b5, 0xd050f926,
0xa592e4fd, 0xd4bae5ab, 0xa5886dba, 0xd160d6e5, 0xa5817354, 0xd395f8ba, 0xa57df60f, 0xd277e518,
/* 1024 - format = Q30 * 2^-10 */
0xbff3703e, 0xfff36f02, 0xbfda5824, 0xc0000b1a, 0xbfc149ed, 0xffc12b16, 0xbfa845a0, 0xc0003c74,
0xbf8f4b3e, 0xff8ee750, 0xbf765acc, 0xc0009547, 0xbf5d744e, 0xff5ca3d0, 0xbf4497c8, 0xc0011594,
0xbf2bc53d, 0xff2a60b4, 0xbf12fcb2, 0xc001bd5c, 0xbefa3e2a, 0xfef81e1d, 0xbee189a8, 0xc0028c9c,
0xbec8df32, 0xfec5dc28, 0xbeb03eca, 0xc0038356, 0xbe97a875, 0xfe939af5, 0xbe7f1c36, 0xc004a188,
0xbe669a10, 0xfe615aa3, 0xbe4e2209, 0xc005e731, 0xbe35b423, 0xfe2f1b50, 0xbe1d5062, 0xc0075452,
0xbe04f6cb, 0xfdfcdd1d, 0xbdeca760, 0xc008e8e8, 0xbdd46225, 0xfdcaa027, 0xbdbc2720, 0xc00aa4f3,
0xbda3f652, 0xfd98648d, 0xbd8bcfbf, 0xc00c8872, 0xbd73b36d, 0xfd662a70, 0xbd5ba15d, 0xc00e9364,
0xbd439995, 0xfd33f1ed, 0xbd2b9c17, 0xc010c5c7, 0xbd13a8e7, 0xfd01bb24, 0xbcfbc00a, 0xc0131f9b,
0xbce3e182, 0xfccf8634, 0xbccc0d53, 0xc015a0dd, 0xbcb44382, 0xfc9d533b, 0xbc9c8411, 0xc018498c,
0xbc84cf05, 0xfc6b2259, 0xbc6d2461, 0xc01b19a7, 0xbc558428, 0xfc38f3ac, 0xbc3dee5f, 0xc01e112b,
0xbc266309, 0xfc06c754, 0xbc0ee22a, 0xc0213018, 0xbbf76bc4, 0xfbd49d70, 0xbbdfffdd, 0xc024766a,
0xbbc89e77, 0xfba2761e, 0xbbb14796, 0xc027e421, 0xbb99fb3e, 0xfb70517d, 0xbb82b972, 0xc02b7939,
0xbb6b8235, 0xfb3e2fac, 0xbb54558d, 0xc02f35b1, 0xbb3d337b, 0xfb0c10cb, 0xbb261c04, 0xc0331986,
0xbb0f0f2b, 0xfad9f4f8, 0xbaf80cf4, 0xc03724b6, 0xbae11561, 0xfaa7dc52, 0xbaca2878, 0xc03b573f,
0xbab3463b, 0xfa75c6f8, 0xba9c6eae, 0xc03fb11d, 0xba85a1d4, 0xfa43b508, 0xba6edfb1, 0xc044324f,
0xba582849, 0xfa11a6a3, 0xba417b9e, 0xc048dad1, 0xba2ad9b5, 0xf9df9be6, 0xba144291, 0xc04daaa1,
0xb9fdb635, 0xf9ad94f0, 0xb9e734a4, 0xc052a1bb, 0xb9d0bde4, 0xf97b91e1, 0xb9ba51f6, 0xc057c01d,
0xb9a3f0de, 0xf94992d7, 0xb98d9aa0, 0xc05d05c3, 0xb9774f3f, 0xf91797f0, 0xb9610ebe, 0xc06272aa,
0xb94ad922, 0xf8e5a14d, 0xb934ae6d, 0xc06806ce, 0xb91e8ea3, 0xf8b3af0c, 0xb90879c7, 0xc06dc22e,
0xb8f26fdc, 0xf881c14b, 0xb8dc70e7, 0xc073a4c3, 0xb8c67cea, 0xf84fd829, 0xb8b093ea, 0xc079ae8c,
0xb89ab5e8, 0xf81df3c5, 0xb884e2e9, 0xc07fdf85, 0xb86f1af0, 0xf7ec143e, 0xb8595e00, 0xc08637a9,
0xb843ac1d, 0xf7ba39b3, 0xb82e0549, 0xc08cb6f5, 0xb818698a, 0xf7886442, 0xb802d8e0, 0xc0935d64,
0xb7ed5351, 0xf756940a, 0xb7d7d8df, 0xc09a2af3, 0xb7c2698e, 0xf724c92a, 0xb7ad0561, 0xc0a11f9d,
0xb797ac5b, 0xf6f303c0, 0xb7825e80, 0xc0a83b5e, 0xb76d1bd2, 0xf6c143ec, 0xb757e455, 0xc0af7e33,
0xb742b80d, 0xf68f89cb, 0xb72d96fd, 0xc0b6e815, 0xb7188127, 0xf65dd57d, 0xb7037690, 0xc0be7901,
0xb6ee773a, 0xf62c2721, 0xb6d98328, 0xc0c630f2, 0xb6c49a5e, 0xf5fa7ed4, 0xb6afbce0, 0xc0ce0fe3,
0xb69aeab0, 0xf5c8dcb6, 0xb68623d1, 0xc0d615cf, 0xb6716847, 0xf59740e5, 0xb65cb815, 0xc0de42b2,
0xb648133e, 0xf565ab80, 0xb63379c5, 0xc0e69686, 0xb61eebae, 0xf5341ca5, 0xb60a68fb, 0xc0ef1147,
0xb5f5f1b1, 0xf5029473, 0xb5e185d1, 0xc0f7b2ee, 0xb5cd255f, 0xf4d11308, 0xb5b8d05f, 0xc1007b77,
0xb5a486d2, 0xf49f9884, 0xb59048be, 0xc1096add, 0xb57c1624, 0xf46e2504, 0xb567ef08, 0xc1128119,
0xb553d36c, 0xf43cb8a7, 0xb53fc355, 0xc11bbe26, 0xb52bbec4, 0xf40b538b, 0xb517c5be, 0xc12521ff,
0xb503d845, 0xf3d9f5cf, 0xb4eff65c, 0xc12eac9d, 0xb4dc2007, 0xf3a89f92, 0xb4c85548, 0xc1385dfb,
0xb4b49622, 0xf37750f2, 0xb4a0e299, 0xc1423613, 0xb48d3ab0, 0xf3460a0d, 0xb4799e69, 0xc14c34df,
0xb4660dc8, 0xf314cb02, 0xb45288cf, 0xc1565a58, 0xb43f0f82, 0xf2e393ef, 0xb42ba1e4, 0xc160a678,
0xb4183ff7, 0xf2b264f2, 0xb404e9bf, 0xc16b193a, 0xb3f19f3e, 0xf2813e2a, 0xb3de6078, 0xc175b296,
0xb3cb2d70, 0xf2501fb5, 0xb3b80628, 0xc1807285, 0xb3a4eaa4, 0xf21f09b1, 0xb391dae6, 0xc18b5903,
0xb37ed6f1, 0xf1edfc3d, 0xb36bdec9, 0xc1966606, 0xb358f26f, 0xf1bcf777, 0xb34611e8, 0xc1a1998a,
0xb3333d36, 0xf18bfb7d, 0xb320745c, 0xc1acf386, 0xb30db75d, 0xf15b086d, 0xb2fb063b, 0xc1b873f5,
0xb2e860fa, 0xf12a1e66, 0xb2d5c79d, 0xc1c41ace, 0xb2c33a26, 0xf0f93d86, 0xb2b0b898, 0xc1cfe80a,
0xb29e42f6, 0xf0c865ea, 0xb28bd943, 0xc1dbdba3, 0xb2797b82, 0xf09797b2, 0xb26729b5, 0xc1e7f591,
0xb254e3e0, 0xf066d2fa, 0xb242aa05, 0xc1f435cc, 0xb2307c27, 0xf03617e2, 0xb21e5a49, 0xc2009c4e,
0xb20c446d, 0xf0056687, 0xb1fa3a97, 0xc20d290d, 0xb1e83cc9, 0xefd4bf08, 0xb1d64b06, 0xc219dc03,
0xb1c46551, 0xefa42181, 0xb1b28bad, 0xc226b528, 0xb1a0be1b, 0xef738e12, 0xb18efca0, 0xc233b473,
0xb17d473d, 0xef4304d8, 0xb16b9df6, 0xc240d9de, 0xb15a00cd, 0xef1285f2, 0xb1486fc5, 0xc24e255e,
0xb136eae1, 0xeee2117c, 0xb1257223, 0xc25b96ee, 0xb114058e, 0xeeb1a796, 0xb102a524, 0xc2692e83,
0xb0f150e9, 0xee81485c, 0xb0e008e0, 0xc276ec16, 0xb0cecd09, 0xee50f3ed, 0xb0bd9d6a, 0xc284cf9f,
0xb0ac7a03, 0xee20aa67, 0xb09b62d8, 0xc292d914, 0xb08a57eb, 0xedf06be6, 0xb079593f, 0xc2a1086d,
0xb06866d7, 0xedc0388a, 0xb05780b5, 0xc2af5da2, 0xb046a6db, 0xed901070, 0xb035d94e, 0xc2bdd8a9,
0xb025180e, 0xed5ff3b5, 0xb014631e, 0xc2cc7979, 0xb003ba82, 0xed2fe277, 0xaff31e3b, 0xc2db400a,
0xafe28e4d, 0xecffdcd4, 0xafd20ab9, 0xc2ea2c53, 0xafc19383, 0xeccfe2ea, 0xafb128ad, 0xc2f93e4a,
0xafa0ca39, 0xec9ff4d6, 0xaf90782a, 0xc30875e5, 0xaf803283, 0xec7012b5, 0xaf6ff945, 0xc317d31c,
0xaf5fcc74, 0xec403ca5, 0xaf4fac12, 0xc32755e5, 0xaf3f9822, 0xec1072c4, 0xaf2f90a5, 0xc336fe37,
0xaf1f959f, 0xebe0b52f, 0xaf0fa712, 0xc346cc07, 0xaeffc500, 0xebb10404, 0xaeefef6c, 0xc356bf4d,
0xaee02658, 0xeb815f60, 0xaed069c7, 0xc366d7fd, 0xaec0b9bb, 0xeb51c760, 0xaeb11636, 0xc377160f,
0xaea17f3b, 0xeb223c22, 0xae91f4cd, 0xc3877978, 0xae8276ed, 0xeaf2bdc3, 0xae73059f, 0xc398022f,
0xae63a0e3, 0xeac34c60, 0xae5448be, 0xc3a8b028, 0xae44fd31, 0xea93e817, 0xae35be3f, 0xc3b9835a,
0xae268be9, 0xea649105, 0xae176633, 0xc3ca7bba, 0xae084d1f, 0xea354746, 0xadf940ae, 0xc3db993e,
0xadea40e4, 0xea060af9, 0xaddb4dc2, 0xc3ecdbdc, 0xadcc674b, 0xe9d6dc3b, 0xadbd8d82, 0xc3fe4388,
0xadaec067, 0xe9a7bb28, 0xad9fffff, 0xc40fd037, 0xad914c4b, 0xe978a7dd, 0xad82a54c, 0xc42181e0,
0xad740b07, 0xe949a278, 0xad657d7c, 0xc4335877, 0xad56fcaf, 0xe91aab16, 0xad4888a0, 0xc44553f2,
0xad3a2153, 0xe8ebc1d3, 0xad2bc6ca, 0xc4577444, 0xad1d7907, 0xe8bce6cd, 0xad0f380c, 0xc469b963,
0xad0103db, 0xe88e1a20, 0xacf2dc77, 0xc47c2344, 0xace4c1e2, 0xe85f5be9, 0xacd6b41e, 0xc48eb1db,
0xacc8b32c, 0xe830ac45, 0xacbabf10, 0xc4a1651c, 0xacacd7cb, 0xe8020b52, 0xac9efd60, 0xc4b43cfd,
0xac912fd1, 0xe7d3792b, 0xac836f1f, 0xc4c73972, 0xac75bb4d, 0xe7a4f5ed, 0xac68145d, 0xc4da5a6f,
0xac5a7a52, 0xe77681b6, 0xac4ced2c, 0xc4ed9fe7, 0xac3f6cef, 0xe7481ca1, 0xac31f99d, 0xc50109d0,
0xac249336, 0xe719c6cb, 0xac1739bf, 0xc514981d, 0xac09ed38, 0xe6eb8052, 0xabfcada3, 0xc5284ac3,
0xabef7b04, 0xe6bd4951, 0xabe2555b, 0xc53c21b4, 0xabd53caa, 0xe68f21e5, 0xabc830f5, 0xc5501ce5,
0xabbb323c, 0xe6610a2a, 0xabae4082, 0xc5643c4a, 0xaba15bc9, 0xe633023e, 0xab948413, 0xc5787fd6,
0xab87b962, 0xe6050a3b, 0xab7afbb7, 0xc58ce77c, 0xab6e4b15, 0xe5d72240, 0xab61a77d, 0xc5a17330,
0xab5510f3, 0xe5a94a67, 0xab488776, 0xc5b622e6, 0xab3c0b0b, 0xe57b82cd, 0xab2f9bb1, 0xc5caf690,
0xab23396c, 0xe54dcb8f, 0xab16e43d, 0xc5dfee22, 0xab0a9c27, 0xe52024c9, 0xaafe612a, 0xc5f5098f,
0xaaf23349, 0xe4f28e96, 0xaae61286, 0xc60a48c9, 0xaad9fee3, 0xe4c50914, 0xaacdf861, 0xc61fabc4,
0xaac1ff03, 0xe497945d, 0xaab612ca, 0xc6353273, 0xaaaa33b8, 0xe46a308f, 0xaa9e61cf, 0xc64adcc7,
0xaa929d10, 0xe43cddc4, 0xaa86e57e, 0xc660aab5, 0xaa7b3b1b, 0xe40f9c1a, 0xaa6f9de7, 0xc6769c2e,
0xaa640de6, 0xe3e26bac, 0xaa588b18, 0xc68cb124, 0xaa4d157f, 0xe3b54c95, 0xaa41ad1e, 0xc6a2e98b,
0xaa3651f6, 0xe3883ef2, 0xaa2b0409, 0xc6b94554, 0xaa1fc358, 0xe35b42df, 0xaa148fe6, 0xc6cfc472,
0xaa0969b3, 0xe32e5876, 0xa9fe50c2, 0xc6e666d7, 0xa9f34515, 0xe3017fd5, 0xa9e846ad, 0xc6fd2c75,
0xa9dd558b, 0xe2d4b916, 0xa9d271b2, 0xc714153e, 0xa9c79b23, 0xe2a80456, 0xa9bcd1e0, 0xc72b2123,
0xa9b215ea, 0xe27b61af, 0xa9a76744, 0xc7425016, 0xa99cc5ee, 0xe24ed13d, 0xa99231eb, 0xc759a20a,
0xa987ab3c, 0xe222531c, 0xa97d31e3, 0xc77116f0, 0xa972c5e1, 0xe1f5e768, 0xa9686738, 0xc788aeb9,
0xa95e15e9, 0xe1c98e3b, 0xa953d1f7, 0xc7a06957, 0xa9499b62, 0xe19d47b1, 0xa93f722c, 0xc7b846ba,
0xa9355658, 0xe17113e5, 0xa92b47e5, 0xc7d046d6, 0xa92146d7, 0xe144f2f3, 0xa917532e, 0xc7e8699a,
0xa90d6cec, 0xe118e4f6, 0xa9039413, 0xc800aef7, 0xa8f9c8a4, 0xe0ecea09, 0xa8f00aa0, 0xc81916df,
0xa8e65a0a, 0xe0c10247, 0xa8dcb6e2, 0xc831a143, 0xa8d3212a, 0xe0952dcb, 0xa8c998e3, 0xc84a4e14,
0xa8c01e10, 0xe0696cb0, 0xa8b6b0b1, 0xc8631d42, 0xa8ad50c8, 0xe03dbf11, 0xa8a3fe57, 0xc87c0ebd,
0xa89ab95e, 0xe012250a, 0xa89181df, 0xc8952278, 0xa88857dc, 0xdfe69eb4, 0xa87f3b57, 0xc8ae5862,
0xa8762c4f, 0xdfbb2c2c, 0xa86d2ac8, 0xc8c7b06b, 0xa86436c2, 0xdf8fcd8b, 0xa85b503e, 0xc8e12a84,
0xa852773f, 0xdf6482ed, 0xa849abc4, 0xc8fac69e, 0xa840edd1, 0xdf394c6b, 0xa8383d66, 0xc91484a8,
0xa82f9a84, 0xdf0e2a22, 0xa827052d, 0xc92e6492, 0xa81e7d62, 0xdee31c2b, 0xa8160324, 0xc948664d,
0xa80d9675, 0xdeb822a1, 0xa8053756, 0xc96289c9, 0xa7fce5c9, 0xde8d3d9e, 0xa7f4a1ce, 0xc97ccef5,
0xa7ec6b66, 0xde626d3e, 0xa7e44294, 0xc99735c2, 0xa7dc2759, 0xde37b199, 0xa7d419b4, 0xc9b1be1e,
0xa7cc19a9, 0xde0d0acc, 0xa7c42738, 0xc9cc67fa, 0xa7bc4262, 0xdde278ef, 0xa7b46b29, 0xc9e73346,
0xa7aca18e, 0xddb7fc1e, 0xa7a4e591, 0xca021fef, 0xa79d3735, 0xdd8d9472, 0xa795967a, 0xca1d2de7,
0xa78e0361, 0xdd634206, 0xa7867dec, 0xca385d1d, 0xa77f061c, 0xdd3904f4, 0xa7779bf2, 0xca53ad7e,
0xa7703f70, 0xdd0edd55, 0xa768f095, 0xca6f1efc, 0xa761af64, 0xdce4cb44, 0xa75a7bdd, 0xca8ab184,
0xa7535602, 0xdcbacedb, 0xa74c3dd4, 0xcaa66506, 0xa7453353, 0xdc90e834, 0xa73e3681, 0xcac23971,
0xa7374760, 0xdc671768, 0xa73065ef, 0xcade2eb3, 0xa7299231, 0xdc3d5c91, 0xa722cc25, 0xcafa44bc,
0xa71c13ce, 0xdc13b7c9, 0xa715692c, 0xcb167b79, 0xa70ecc41, 0xdbea292b, 0xa7083d0d, 0xcb32d2da,
0xa701bb91, 0xdbc0b0ce, 0xa6fb47ce, 0xcb4f4acd, 0xa6f4e1c6, 0xdb974ece, 0xa6ee8979, 0xcb6be341,
0xa6e83ee8, 0xdb6e0342, 0xa6e20214, 0xcb889c23, 0xa6dbd2ff, 0xdb44ce46, 0xa6d5b1a9, 0xcba57563,
0xa6cf9e13, 0xdb1baff2, 0xa6c9983e, 0xcbc26eee, 0xa6c3a02b, 0xdaf2a860, 0xa6bdb5da, 0xcbdf88b3,
0xa6b7d94e, 0xdac9b7a9, 0xa6b20a86, 0xcbfcc29f, 0xa6ac4984, 0xdaa0dde7, 0xa6a69649, 0xcc1a1ca0,
0xa6a0f0d5, 0xda781b31, 0xa69b5929, 0xcc3796a5, 0xa695cf46, 0xda4f6fa3, 0xa690532d, 0xcc55309b,
0xa68ae4df, 0xda26db54, 0xa685845c, 0xcc72ea70, 0xa68031a6, 0xd9fe5e5e, 0xa67aecbd, 0xcc90c412,
0xa675b5a3, 0xd9d5f8d9, 0xa6708c57, 0xccaebd6e, 0xa66b70db, 0xd9adaadf, 0xa6666330, 0xccccd671,
0xa6616355, 0xd9857489, 0xa65c714d, 0xcceb0f0a, 0xa6578d18, 0xd95d55ef, 0xa652b6b6, 0xcd096725,
0xa64dee28, 0xd9354f2a, 0xa6493370, 0xcd27deb0, 0xa644868d, 0xd90d6053, 0xa63fe781, 0xcd467599,
0xa63b564c, 0xd8e58982, 0xa636d2ee, 0xcd652bcb, 0xa6325d6a, 0xd8bdcad0, 0xa62df5bf, 0xcd840134,
0xa6299bed, 0xd8962456, 0xa6254ff7, 0xcda2f5c2, 0xa62111db, 0xd86e962b, 0xa61ce19c, 0xcdc20960,
0xa618bf39, 0xd8472069, 0xa614aab3, 0xcde13bfd, 0xa610a40c, 0xd81fc328, 0xa60cab43, 0xce008d84,
0xa608c058, 0xd7f87e7f, 0xa604e34e, 0xce1ffde2, 0xa6011424, 0xd7d15288, 0xa5fd52db, 0xce3f8d05,
0xa5f99f73, 0xd7aa3f5a, 0xa5f5f9ed, 0xce5f3ad8, 0xa5f2624a, 0xd783450d, 0xa5eed88a, 0xce7f0748,
0xa5eb5cae, 0xd75c63ba, 0xa5e7eeb6, 0xce9ef241, 0xa5e48ea3, 0xd7359b78, 0xa5e13c75, 0xcebefbb0,
0xa5ddf82d, 0xd70eec60, 0xa5dac1cb, 0xcedf2380, 0xa5d79950, 0xd6e85689, 0xa5d47ebc, 0xceff699f,
0xa5d17210, 0xd6c1da0b, 0xa5ce734d, 0xcf1fcdf8, 0xa5cb8272, 0xd69b76fe, 0xa5c89f80, 0xcf405077,
0xa5c5ca77, 0xd6752d79, 0xa5c30359, 0xcf60f108, 0xa5c04a25, 0xd64efd94, 0xa5bd9edc, 0xcf81af97,
0xa5bb017f, 0xd628e767, 0xa5b8720d, 0xcfa28c10, 0xa5b5f087, 0xd602eb0a, 0xa5b37cee, 0xcfc3865e,
0xa5b11741, 0xd5dd0892, 0xa5aebf82, 0xcfe49e6d, 0xa5ac75b0, 0xd5b74019, 0xa5aa39cd, 0xd005d42a,
0xa5a80bd7, 0xd59191b5, 0xa5a5ebd0, 0xd027277e, 0xa5a3d9b8, 0xd56bfd7d, 0xa5a1d590, 0xd0489856,
0xa59fdf57, 0xd5468389, 0xa59df70e, 0xd06a269d, 0xa59c1cb5, 0xd52123f0, 0xa59a504c, 0xd08bd23f,
0xa59891d4, 0xd4fbdec9, 0xa596e14e, 0xd0ad9b26, 0xa5953eb8, 0xd4d6b42b, 0xa593aa14, 0xd0cf813e,
0xa5922362, 0xd4b1a42c, 0xa590aaa2, 0xd0f18472, 0xa58f3fd4, 0xd48caee4, 0xa58de2f8, 0xd113a4ad,
0xa58c940f, 0xd467d469, 0xa58b5319, 0xd135e1d9, 0xa58a2016, 0xd44314d3, 0xa588fb06, 0xd1583be2,
0xa587e3ea, 0xd41e7037, 0xa586dac1, 0xd17ab2b3, 0xa585df8c, 0xd3f9e6ad, 0xa584f24b, 0xd19d4636,
0xa58412fe, 0xd3d5784a, 0xa58341a5, 0xd1bff656, 0xa5827e40, 0xd3b12526, 0xa581c8d0, 0xd1e2c2fd,
0xa5812154, 0xd38ced57, 0xa58087cd, 0xd205ac17, 0xa57ffc3b, 0xd368d0f3, 0xa57f7e9d, 0xd228b18d,
0xa57f0ef5, 0xd344d011, 0xa57ead41, 0xd24bd34a, 0xa57e5982, 0xd320eac6, 0xa57e13b8, 0xd26f1138,
0xa57ddbe4, 0xd2fd2129, 0xa57db204, 0xd2926b41, 0xa57d961a, 0xd2d97350, 0xa57d8825, 0xd2b5e151,
};

const int cos1sin1tab[514] PROGMEM = {
/* format = Q30 */
0x40000000, 0x00000000, 0x40323034, 0x003243f1, 0x406438cf, 0x006487c4, 0x409619b2, 0x0096cb58,
0x40c7d2bd, 0x00c90e90, 0x40f963d3, 0x00fb514b, 0x412accd4, 0x012d936c, 0x415c0da3, 0x015fd4d2,
0x418d2621, 0x0192155f, 0x41be162f, 0x01c454f5, 0x41eeddaf, 0x01f69373, 0x421f7c84, 0x0228d0bb,
0x424ff28f, 0x025b0caf, 0x42803fb2, 0x028d472e, 0x42b063d0, 0x02bf801a, 0x42e05ecb, 0x02f1b755,
0x43103085, 0x0323ecbe, 0x433fd8e1, 0x03562038, 0x436f57c1, 0x038851a2, 0x439ead09, 0x03ba80df,
0x43cdd89a, 0x03ecadcf, 0x43fcda59, 0x041ed854, 0x442bb227, 0x0451004d, 0x445a5fe8, 0x0483259d,
0x4488e37f, 0x04b54825, 0x44b73ccf, 0x04e767c5, 0x44e56bbd, 0x0519845e, 0x4513702a, 0x054b9dd3,
0x454149fc, 0x057db403, 0x456ef916, 0x05afc6d0, 0x459c7d5a, 0x05e1d61b, 0x45c9d6af, 0x0613e1c5,
0x45f704f7, 0x0645e9af, 0x46240816, 0x0677edbb, 0x4650dff1, 0x06a9edc9, 0x467d8c6d, 0x06dbe9bb,
0x46aa0d6d, 0x070de172, 0x46d662d6, 0x073fd4cf, 0x47028c8d, 0x0771c3b3, 0x472e8a76, 0x07a3adff,
0x475a5c77, 0x07d59396, 0x47860275, 0x08077457, 0x47b17c54, 0x08395024, 0x47dcc9f9, 0x086b26de,
0x4807eb4b, 0x089cf867, 0x4832e02d, 0x08cec4a0, 0x485da887, 0x09008b6a, 0x4888443d, 0x09324ca7,
0x48b2b335, 0x09640837, 0x48dcf556, 0x0995bdfd, 0x49070a84, 0x09c76dd8, 0x4930f2a6, 0x09f917ac,
0x495aada2, 0x0a2abb59, 0x49843b5f, 0x0a5c58c0, 0x49ad9bc2, 0x0a8defc3, 0x49d6ceb3, 0x0abf8043,
0x49ffd417, 0x0af10a22, 0x4a28abd6, 0x0b228d42, 0x4a5155d6, 0x0b540982, 0x4a79d1ff, 0x0b857ec7,
0x4aa22036, 0x0bb6ecef, 0x4aca4065, 0x0be853de, 0x4af23270, 0x0c19b374, 0x4b19f641, 0x0c4b0b94,
0x4b418bbe, 0x0c7c5c1e, 0x4b68f2cf, 0x0cada4f5, 0x4b902b5c, 0x0cdee5f9, 0x4bb7354d, 0x0d101f0e,
0x4bde1089, 0x0d415013, 0x4c04bcf8, 0x0d7278eb, 0x4c2b3a84, 0x0da39978, 0x4c518913, 0x0dd4b19a,
0x4c77a88e, 0x0e05c135, 0x4c9d98de, 0x0e36c82a, 0x4cc359ec, 0x0e67c65a, 0x4ce8eb9f, 0x0e98bba7,
0x4d0e4de2, 0x0ec9a7f3, 0x4d33809c, 0x0efa8b20, 0x4d5883b7, 0x0f2b650f, 0x4d7d571c, 0x0f5c35a3,
0x4da1fab5, 0x0f8cfcbe, 0x4dc66e6a, 0x0fbdba40, 0x4deab226, 0x0fee6e0d, 0x4e0ec5d1, 0x101f1807,
0x4e32a956, 0x104fb80e, 0x4e565c9f, 0x10804e06, 0x4e79df95, 0x10b0d9d0, 0x4e9d3222, 0x10e15b4e,
0x4ec05432, 0x1111d263, 0x4ee345ad, 0x11423ef0, 0x4f06067f, 0x1172a0d7, 0x4f289692, 0x11a2f7fc,
0x4f4af5d1, 0x11d3443f, 0x4f6d2427, 0x12038584, 0x4f8f217e, 0x1233bbac, 0x4fb0edc1, 0x1263e699,
0x4fd288dc, 0x1294062f, 0x4ff3f2bb, 0x12c41a4f, 0x50152b47, 0x12f422db, 0x5036326e, 0x13241fb6,
0x50570819, 0x135410c3, 0x5077ac37, 0x1383f5e3, 0x50981eb1, 0x13b3cefa, 0x50b85f74, 0x13e39be9,
0x50d86e6d, 0x14135c94, 0x50f84b87, 0x144310dd, 0x5117f6ae, 0x1472b8a5, 0x51376fd0, 0x14a253d1,
0x5156b6d9, 0x14d1e242, 0x5175cbb5, 0x150163dc, 0x5194ae52, 0x1530d881, 0x51b35e9b, 0x15604013,
0x51d1dc80, 0x158f9a76, 0x51f027eb, 0x15bee78c, 0x520e40cc, 0x15ee2738, 0x522c270f, 0x161d595d,
0x5249daa2, 0x164c7ddd, 0x52675b72, 0x167b949d, 0x5284a96e, 0x16aa9d7e, 0x52a1c482, 0x16d99864,
0x52beac9f, 0x17088531, 0x52db61b0, 0x173763c9, 0x52f7e3a6, 0x1766340f, 0x5314326d, 0x1794f5e6,
0x53304df6, 0x17c3a931, 0x534c362d, 0x17f24dd3, 0x5367eb03, 0x1820e3b0, 0x53836c66, 0x184f6aab,
0x539eba45, 0x187de2a7, 0x53b9d48f, 0x18ac4b87, 0x53d4bb34, 0x18daa52f, 0x53ef6e23, 0x1908ef82,
0x5409ed4b, 0x19372a64, 0x5424389d, 0x196555b8, 0x543e5007, 0x19937161, 0x5458337a, 0x19c17d44,
0x5471e2e6, 0x19ef7944, 0x548b5e3b, 0x1a1d6544, 0x54a4a56a, 0x1a4b4128, 0x54bdb862, 0x1a790cd4,
0x54d69714, 0x1aa6c82b, 0x54ef4171, 0x1ad47312, 0x5507b76a, 0x1b020d6c, 0x551ff8ef, 0x1b2f971e,
0x553805f2, 0x1b5d100a, 0x554fde64, 0x1b8a7815, 0x55678236, 0x1bb7cf23, 0x557ef15a, 0x1be51518,
0x55962bc0, 0x1c1249d8, 0x55ad315b, 0x1c3f6d47, 0x55c4021d, 0x1c6c7f4a, 0x55da9df7, 0x1c997fc4,
0x55f104dc, 0x1cc66e99, 0x560736bd, 0x1cf34baf, 0x561d338d, 0x1d2016e9, 0x5632fb3f, 0x1d4cd02c,
0x56488dc5, 0x1d79775c, 0x565deb11, 0x1da60c5d, 0x56731317, 0x1dd28f15, 0x568805c9, 0x1dfeff67,
0x569cc31b, 0x1e2b5d38, 0x56b14b00, 0x1e57a86d, 0x56c59d6a, 0x1e83e0eb, 0x56d9ba4e, 0x1eb00696,
0x56eda1a0, 0x1edc1953, 0x57015352, 0x1f081907, 0x5714cf59, 0x1f340596, 0x572815a8, 0x1f5fdee6,
0x573b2635, 0x1f8ba4dc, 0x574e00f2, 0x1fb7575c, 0x5760a5d5, 0x1fe2f64c, 0x577314d2, 0x200e8190,
0x57854ddd, 0x2039f90f, 0x579750ec, 0x20655cac, 0x57a91df2, 0x2090ac4d, 0x57bab4e6, 0x20bbe7d8,
0x57cc15bc, 0x20e70f32, 0x57dd406a, 0x21122240, 0x57ee34e5, 0x213d20e8, 0x57fef323, 0x21680b0f,
0x580f7b19, 0x2192e09b, 0x581fccbc, 0x21bda171, 0x582fe804, 0x21e84d76, 0x583fcce6, 0x2212e492,
0x584f7b58, 0x223d66a8, 0x585ef351, 0x2267d3a0, 0x586e34c7, 0x22922b5e, 0x587d3fb0, 0x22bc6dca,
0x588c1404, 0x22e69ac8, 0x589ab1b9, 0x2310b23e, 0x58a918c6, 0x233ab414, 0x58b74923, 0x2364a02e,
0x58c542c5, 0x238e7673, 0x58d305a6, 0x23b836ca, 0x58e091bd, 0x23e1e117, 0x58ede700, 0x240b7543,
0x58fb0568, 0x2434f332, 0x5907eced, 0x245e5acc, 0x59149d87, 0x2487abf7, 0x5921172e, 0x24b0e699,
0x592d59da, 0x24da0a9a, 0x59396584, 0x250317df, 0x59453a24, 0x252c0e4f, 0x5950d7b3, 0x2554edd1,
0x595c3e2a, 0x257db64c, 0x59676d82, 0x25a667a7, 0x597265b4, 0x25cf01c8, 0x597d26b8, 0x25f78497,
0x5987b08a, 0x261feffa, 0x59920321, 0x264843d9, 0x599c1e78, 0x2670801a, 0x59a60288, 0x2698a4a6,
0x59afaf4c, 0x26c0b162, 0x59b924bc, 0x26e8a637, 0x59c262d5, 0x2710830c, 0x59cb698f, 0x273847c8,
0x59d438e5, 0x275ff452, 0x59dcd0d3, 0x27878893, 0x59e53151, 0x27af0472, 0x59ed5a5c, 0x27d667d5,
0x59f54bee, 0x27fdb2a7, 0x59fd0603, 0x2824e4cc, 0x5a048895, 0x284bfe2f, 0x5a0bd3a1, 0x2872feb6,
0x5a12e720, 0x2899e64a, 0x5a19c310, 0x28c0b4d2, 0x5a20676c, 0x28e76a37, 0x5a26d42f, 0x290e0661,
0x5a2d0957, 0x29348937, 0x5a3306de, 0x295af2a3, 0x5a38ccc2, 0x2981428c, 0x5a3e5afe, 0x29a778db,
0x5a43b190, 0x29cd9578, 0x5a48d074, 0x29f3984c, 0x5a4db7a6, 0x2a19813f, 0x5a526725, 0x2a3f503a,
0x5a56deec, 0x2a650525, 0x5a5b1efa, 0x2a8a9fea, 0x5a5f274b, 0x2ab02071, 0x5a62f7dd, 0x2ad586a3,
0x5a6690ae, 0x2afad269, 0x5a69f1bb, 0x2b2003ac, 0x5a6d1b03, 0x2b451a55, 0x5a700c84, 0x2b6a164d,
0x5a72c63b, 0x2b8ef77d, 0x5a754827, 0x2bb3bdce, 0x5a779246, 0x2bd8692b, 0x5a79a498, 0x2bfcf97c,
0x5a7b7f1a, 0x2c216eaa, 0x5a7d21cc, 0x2c45c8a0, 0x5a7e8cac, 0x2c6a0746, 0x5a7fbfbb, 0x2c8e2a87,
0x5a80baf6, 0x2cb2324c, 0x5a817e5d, 0x2cd61e7f, 0x5a8209f1, 0x2cf9ef09, 0x5a825db0, 0x2d1da3d5,
0x5a82799a, 0x2d413ccd,
};

const uint8_t sinWindowOffset[NUM_IMDCT_SIZES] PROGMEM = {0, 128};

const int sinWindow[128 + 1024] PROGMEM = {
/* 128 - format = Q31 * 2^0 */
0x00c90f88, 0x7fff6216, 0x025b26d7, 0x7ffa72d1, 0x03ed26e6, 0x7ff09478, 0x057f0035, 0x7fe1c76b,
0x0710a345, 0x7fce0c3e, 0x08a2009a, 0x7fb563b3, 0x0a3308bd, 0x7f97cebd, 0x0bc3ac35, 0x7f754e80,
0x0d53db92, 0x7f4de451, 0x0ee38766, 0x7f2191b4, 0x1072a048, 0x7ef05860, 0x120116d5, 0x7eba3a39,
0x138edbb1, 0x7e7f3957, 0x151bdf86, 0x7e3f57ff, 0x16a81305, 0x7dfa98a8, 0x183366e9, 0x7db0fdf8,
0x19bdcbf3, 0x7d628ac6, 0x1b4732ef, 0x7d0f4218, 0x1ccf8cb3, 0x7cb72724, 0x1e56ca1e, 0x7c5a3d50,
0x1fdcdc1b, 0x7bf88830, 0x2161b3a0, 0x7b920b89, 0x22e541af, 0x7b26cb4f, 0x24677758, 0x7ab6cba4,
0x25e845b6, 0x7a4210d8, 0x27679df4, 0x79c89f6e, 0x28e5714b, 0x794a7c12, 0x2a61b101, 0x78c7aba2,
0x2bdc4e6f, 0x78403329, 0x2d553afc, 0x77b417df, 0x2ecc681e, 0x77235f2d, 0x3041c761, 0x768e0ea6,
0x31b54a5e, 0x75f42c0b, 0x3326e2c3, 0x7555bd4c, 0x34968250, 0x74b2c884, 0x36041ad9, 0x740b53fb,
0x376f9e46, 0x735f6626, 0x38d8fe93, 0x72af05a7, 0x3a402dd2, 0x71fa3949, 0x3ba51e29, 0x71410805,
0x3d07c1d6, 0x708378ff, 0x3e680b2c, 0x6fc19385, 0x3fc5ec98, 0x6efb5f12, 0x4121589b, 0x6e30e34a,
0x427a41d0, 0x6d6227fa, 0x43d09aed, 0x6c8f351c, 0x452456bd, 0x6bb812d1, 0x46756828, 0x6adcc964,
0x47c3c22f, 0x69fd614a, 0x490f57ee, 0x6919e320, 0x4a581c9e, 0x683257ab, 0x4b9e0390, 0x6746c7d8,
0x4ce10034, 0x66573cbb, 0x4e210617, 0x6563bf92, 0x4f5e08e3, 0x646c59bf, 0x5097fc5e, 0x637114cc,
0x51ced46e, 0x6271fa69, 0x53028518, 0x616f146c, 0x5433027d, 0x60686ccf, 0x556040e2, 0x5f5e0db3,
0x568a34a9, 0x5e50015d, 0x57b0d256, 0x5d3e5237, 0x58d40e8c, 0x5c290acc, 0x59f3de12, 0x5b1035cf,
/* 1024 - format = Q31 * 2^0 */
0x001921fb, 0x7ffffd88, 0x004b65ee, 0x7fffe9cb, 0x007da9d4, 0x7fffc251, 0x00afeda8, 0x7fff8719,
0x00e23160, 0x7fff3824, 0x011474f6, 0x7ffed572, 0x0146b860, 0x7ffe5f03, 0x0178fb99, 0x7ffdd4d7,
0x01ab3e97, 0x7ffd36ee, 0x01dd8154, 0x7ffc8549, 0x020fc3c6, 0x7ffbbfe6, 0x024205e8, 0x7ffae6c7,
0x027447b0, 0x7ff9f9ec, 0x02a68917, 0x7ff8f954, 0x02d8ca16, 0x7ff7e500, 0x030b0aa4, 0x7ff6bcf0,
0x033d4abb, 0x7ff58125, 0x036f8a51, 0x7ff4319d, 0x03a1c960, 0x7ff2ce5b, 0x03d407df, 0x7ff1575d,
0x040645c7, 0x7fefcca4, 0x04388310, 0x7fee2e30, 0x046abfb3, 0x7fec7c02, 0x049cfba7, 0x7feab61a,
0x04cf36e5, 0x7fe8dc78, 0x05017165, 0x7fe6ef1c, 0x0533ab20, 0x7fe4ee06, 0x0565e40d, 0x7fe2d938,
0x05981c26, 0x7fe0b0b1, 0x05ca5361, 0x7fde7471, 0x05fc89b8, 0x7fdc247a, 0x062ebf22, 0x7fd9c0ca,
0x0660f398, 0x7fd74964, 0x06932713, 0x7fd4be46, 0x06c5598a, 0x7fd21f72, 0x06f78af6, 0x7fcf6ce8,
0x0729bb4e, 0x7fcca6a7, 0x075bea8c, 0x7fc9ccb2, 0x078e18a7, 0x7fc6df08, 0x07c04598, 0x7fc3dda9,
0x07f27157, 0x7fc0c896, 0x08249bdd, 0x7fbd9fd0, 0x0856c520, 0x7fba6357, 0x0888ed1b, 0x7fb7132b,
0x08bb13c5, 0x7fb3af4e, 0x08ed3916, 0x7fb037bf, 0x091f5d06, 0x7facac7f, 0x09517f8f, 0x7fa90d8e,
0x0983a0a7, 0x7fa55aee, 0x09b5c048, 0x7fa1949e, 0x09e7de6a, 0x7f9dbaa0, 0x0a19fb04, 0x7f99ccf4,
0x0a4c1610, 0x7f95cb9a, 0x0a7e2f85, 0x7f91b694, 0x0ab0475c, 0x7f8d8de1, 0x0ae25d8d, 0x7f895182,
0x0b147211, 0x7f850179, 0x0b4684df, 0x7f809dc5, 0x0b7895f0, 0x7f7c2668, 0x0baaa53b, 0x7f779b62,
0x0bdcb2bb, 0x7f72fcb4, 0x0c0ebe66, 0x7f6e4a5e, 0x0c40c835, 0x7f698461, 0x0c72d020, 0x7f64aabf,
0x0ca4d620, 0x7f5fbd77, 0x0cd6da2d, 0x7f5abc8a, 0x0d08dc3f, 0x7f55a7fa, 0x0d3adc4e, 0x7f507fc7,
0x0d6cda53, 0x7f4b43f2, 0x0d9ed646, 0x7f45f47b, 0x0dd0d01f, 0x7f409164, 0x0e02c7d7, 0x7f3b1aad,
0x0e34bd66, 0x7f359057, 0x0e66b0c3, 0x7f2ff263, 0x0e98a1e9, 0x7f2a40d2, 0x0eca90ce, 0x7f247ba5,
0x0efc7d6b, 0x7f1ea2dc, 0x0f2e67b8, 0x7f18b679, 0x0f604faf, 0x7f12b67c, 0x0f923546, 0x7f0ca2e7,
0x0fc41876, 0x7f067bba, 0x0ff5f938, 0x7f0040f6, 0x1027d784, 0x7ef9f29d, 0x1059b352, 0x7ef390ae,
0x108b8c9b, 0x7eed1b2c, 0x10bd6356, 0x7ee69217, 0x10ef377d, 0x7edff570, 0x11210907, 0x7ed94538,
0x1152d7ed, 0x7ed28171, 0x1184a427, 0x7ecbaa1a, 0x11b66dad, 0x7ec4bf36, 0x11e83478, 0x7ebdc0c6,
0x1219f880, 0x7eb6aeca, 0x124bb9be, 0x7eaf8943, 0x127d7829, 0x7ea85033, 0x12af33ba, 0x7ea1039b,
0x12e0ec6a, 0x7e99a37c, 0x1312a230, 0x7e922fd6, 0x13445505, 0x7e8aa8ac, 0x137604e2, 0x7e830dff,
0x13a7b1bf, 0x7e7b5fce, 0x13d95b93, 0x7e739e1d, 0x140b0258, 0x7e6bc8eb, 0x143ca605, 0x7e63e03b,
0x146e4694, 0x7e5be40c, 0x149fe3fc, 0x7e53d462, 0x14d17e36, 0x7e4bb13c, 0x1503153a, 0x7e437a9c,
0x1534a901, 0x7e3b3083, 0x15663982, 0x7e32d2f4, 0x1597c6b7, 0x7e2a61ed, 0x15c95097, 0x7e21dd73,
0x15fad71b, 0x7e194584, 0x162c5a3b, 0x7e109a24, 0x165dd9f0, 0x7e07db52, 0x168f5632, 0x7dff0911,
0x16c0cef9, 0x7df62362, 0x16f2443e, 0x7ded2a47, 0x1723b5f9, 0x7de41dc0, 0x17552422, 0x7ddafdce,
0x17868eb3, 0x7dd1ca75, 0x17b7f5a3, 0x7dc883b4, 0x17e958ea, 0x7dbf298d, 0x181ab881, 0x7db5bc02,
0x184c1461, 0x7dac3b15, 0x187d6c82, 0x7da2a6c6, 0x18aec0db, 0x7d98ff17, 0x18e01167, 0x7d8f4409,
0x19115e1c, 0x7d85759f, 0x1942a6f3, 0x7d7b93da, 0x1973ebe6, 0x7d719eba, 0x19a52ceb, 0x7d679642,
0x19d669fc, 0x7d5d7a74, 0x1a07a311, 0x7d534b50, 0x1a38d823, 0x7d4908d9, 0x1a6a0929, 0x7d3eb30f,
0x1a9b361d, 0x7d3449f5, 0x1acc5ef6, 0x7d29cd8c, 0x1afd83ad, 0x7d1f3dd6, 0x1b2ea43a, 0x7d149ad5,
0x1b5fc097, 0x7d09e489, 0x1b90d8bb, 0x7cff1af5, 0x1bc1ec9e, 0x7cf43e1a, 0x1bf2fc3a, 0x7ce94dfb,
0x1c240786, 0x7cde4a98, 0x1c550e7c, 0x7cd333f3, 0x1c861113, 0x7cc80a0f, 0x1cb70f43, 0x7cbcccec,
0x1ce80906, 0x7cb17c8d, 0x1d18fe54, 0x7ca618f3, 0x1d49ef26, 0x7c9aa221, 0x1d7adb73, 0x7c8f1817,
0x1dabc334, 0x7c837ad8, 0x1ddca662, 0x7c77ca65, 0x1e0d84f5, 0x7c6c06c0, 0x1e3e5ee5, 0x7c602fec,
0x1e6f342c, 0x7c5445e9, 0x1ea004c1, 0x7c4848ba, 0x1ed0d09d, 0x7c3c3860, 0x1f0197b8, 0x7c3014de,
0x1f325a0b, 0x7c23de35, 0x1f63178f, 0x7c179467, 0x1f93d03c, 0x7c0b3777, 0x1fc4840a, 0x7bfec765,
0x1ff532f2, 0x7bf24434, 0x2025dcec, 0x7be5ade6, 0x205681f1, 0x7bd9047c, 0x208721f9, 0x7bcc47fa,
0x20b7bcfe, 0x7bbf7860, 0x20e852f6, 0x7bb295b0, 0x2118e3dc, 0x7ba59fee, 0x21496fa7, 0x7b989719,
0x2179f64f, 0x7b8b7b36, 0x21aa77cf, 0x7b7e4c45, 0x21daf41d, 0x7b710a49, 0x220b6b32, 0x7b63b543,
0x223bdd08, 0x7b564d36, 0x226c4996, 0x7b48d225, 0x229cb0d5, 0x7b3b4410, 0x22cd12bd, 0x7b2da2fa,
0x22fd6f48, 0x7b1feee5, 0x232dc66d, 0x7b1227d3, 0x235e1826, 0x7b044dc7, 0x238e646a, 0x7af660c2,
0x23beab33, 0x7ae860c7, 0x23eeec78, 0x7ada4dd8, 0x241f2833, 0x7acc27f7, 0x244f5e5c, 0x7abdef25,
0x247f8eec, 0x7aafa367, 0x24afb9da, 0x7aa144bc, 0x24dfdf20, 0x7a92d329, 0x250ffeb7, 0x7a844eae,
0x25401896, 0x7a75b74f, 0x25702cb7, 0x7a670d0d, 0x25a03b11, 0x7a584feb, 0x25d0439f, 0x7a497feb,
0x26004657, 0x7a3a9d0f, 0x26304333, 0x7a2ba75a, 0x26603a2c, 0x7a1c9ece, 0x26902b39, 0x7a0d836d,
0x26c01655, 0x79fe5539, 0x26effb76, 0x79ef1436, 0x271fda96, 0x79dfc064, 0x274fb3ae, 0x79d059c8,
0x277f86b5, 0x79c0e062, 0x27af53a6, 0x79b15435, 0x27df1a77, 0x79a1b545, 0x280edb23, 0x79920392,
0x283e95a1, 0x79823f20, 0x286e49ea, 0x797267f2, 0x289df7f8, 0x79627e08, 0x28cd9fc1, 0x79528167,
0x28fd4140, 0x79427210, 0x292cdc6d, 0x79325006, 0x295c7140, 0x79221b4b, 0x298bffb2, 0x7911d3e2,
0x29bb87bc, 0x790179cd, 0x29eb0957, 0x78f10d0f, 0x2a1a847b, 0x78e08dab, 0x2a49f920, 0x78cffba3,
0x2a796740, 0x78bf56f9, 0x2aa8ced3, 0x78ae9fb0, 0x2ad82fd2, 0x789dd5cb, 0x2b078a36, 0x788cf94c,
0x2b36ddf7, 0x787c0a36, 0x2b662b0e, 0x786b088c, 0x2b957173, 0x7859f44f, 0x2bc4b120, 0x7848cd83,
0x2bf3ea0d, 0x7837942b, 0x2c231c33, 0x78264849, 0x2c52478a, 0x7814e9df, 0x2c816c0c, 0x780378f1,
0x2cb089b1, 0x77f1f581, 0x2cdfa071, 0x77e05f91, 0x2d0eb046, 0x77ceb725, 0x2d3db928, 0x77bcfc3f,
0x2d6cbb10, 0x77ab2ee2, 0x2d9bb5f6, 0x77994f11, 0x2dcaa9d5, 0x77875cce, 0x2df996a3, 0x7775581d,
0x2e287c5a, 0x776340ff, 0x2e575af3, 0x77511778, 0x2e863267, 0x773edb8b, 0x2eb502ae, 0x772c8d3a,
0x2ee3cbc1, 0x771a2c88, 0x2f128d99, 0x7707b979, 0x2f41482e, 0x76f5340e, 0x2f6ffb7a, 0x76e29c4b,
0x2f9ea775, 0x76cff232, 0x2fcd4c19, 0x76bd35c7, 0x2ffbe95d, 0x76aa670d, 0x302a7f3a, 0x76978605,
0x30590dab, 0x768492b4, 0x308794a6, 0x76718d1c, 0x30b61426, 0x765e7540, 0x30e48c22, 0x764b4b23,
0x3112fc95, 0x76380ec8, 0x31416576, 0x7624c031, 0x316fc6be, 0x76115f63, 0x319e2067, 0x75fdec60,
0x31cc7269, 0x75ea672a, 0x31fabcbd, 0x75d6cfc5, 0x3228ff5c, 0x75c32634, 0x32573a3f, 0x75af6a7b,
0x32856d5e, 0x759b9c9b, 0x32b398b3, 0x7587bc98, 0x32e1bc36, 0x7573ca75, 0x330fd7e1, 0x755fc635,
0x333debab, 0x754bafdc, 0x336bf78f, 0x7537876c, 0x3399fb85, 0x75234ce8, 0x33c7f785, 0x750f0054,
0x33f5eb89, 0x74faa1b3, 0x3423d78a, 0x74e63108, 0x3451bb81, 0x74d1ae55, 0x347f9766, 0x74bd199f,
0x34ad6b32, 0x74a872e8, 0x34db36df, 0x7493ba34, 0x3508fa66, 0x747eef85, 0x3536b5be, 0x746a12df,
0x356468e2, 0x74552446, 0x359213c9, 0x744023bc, 0x35bfb66e, 0x742b1144, 0x35ed50c9, 0x7415ece2,
0x361ae2d3, 0x7400b69a, 0x36486c86, 0x73eb6e6e, 0x3675edd9, 0x73d61461, 0x36a366c6, 0x73c0a878,
0x36d0d746, 0x73ab2ab4, 0x36fe3f52, 0x73959b1b, 0x372b9ee3, 0x737ff9ae, 0x3758f5f2, 0x736a4671,
0x37864477, 0x73548168, 0x37b38a6d, 0x733eaa96, 0x37e0c7cc, 0x7328c1ff, 0x380dfc8d, 0x7312c7a5,
0x383b28a9, 0x72fcbb8c, 0x38684c19, 0x72e69db7, 0x389566d6, 0x72d06e2b, 0x38c278d9, 0x72ba2cea,
0x38ef821c, 0x72a3d9f7, 0x391c8297, 0x728d7557, 0x39497a43, 0x7276ff0d, 0x39766919, 0x7260771b,
0x39a34f13, 0x7249dd86, 0x39d02c2a, 0x72333251, 0x39fd0056, 0x721c7580, 0x3a29cb91, 0x7205a716,
0x3a568dd4, 0x71eec716, 0x3a834717, 0x71d7d585, 0x3aaff755, 0x71c0d265, 0x3adc9e86, 0x71a9bdba,
0x3b093ca3, 0x71929789, 0x3b35d1a5, 0x717b5fd3, 0x3b625d86, 0x7164169d, 0x3b8ee03e, 0x714cbbeb,
0x3bbb59c7, 0x71354fc0, 0x3be7ca1a, 0x711dd220, 0x3c143130, 0x7106430e, 0x3c408f03, 0x70eea28e,
0x3c6ce38a, 0x70d6f0a4, 0x3c992ec0, 0x70bf2d53, 0x3cc5709e, 0x70a7589f, 0x3cf1a91c, 0x708f728b,
0x3d1dd835, 0x70777b1c, 0x3d49fde1, 0x705f7255, 0x3d761a19, 0x70475839, 0x3da22cd7, 0x702f2ccd,
0x3dce3614, 0x7016f014, 0x3dfa35c8, 0x6ffea212, 0x3e262bee, 0x6fe642ca, 0x3e52187f, 0x6fcdd241,
0x3e7dfb73, 0x6fb5507a, 0x3ea9d4c3, 0x6f9cbd79, 0x3ed5a46b, 0x6f841942, 0x3f016a61, 0x6f6b63d8,
0x3f2d26a0, 0x6f529d40, 0x3f58d921, 0x6f39c57d, 0x3f8481dd, 0x6f20dc92, 0x3fb020ce, 0x6f07e285,
0x3fdbb5ec, 0x6eeed758, 0x40074132, 0x6ed5bb10, 0x4032c297, 0x6ebc8db0, 0x405e3a16, 0x6ea34f3d,
0x4089a7a8, 0x6e89ffb9, 0x40b50b46, 0x6e709f2a, 0x40e064ea, 0x6e572d93, 0x410bb48c, 0x6e3daaf8,
0x4136fa27, 0x6e24175c, 0x416235b2, 0x6e0a72c5, 0x418d6729, 0x6df0bd35, 0x41b88e84, 0x6dd6f6b1,
0x41e3abbc, 0x6dbd1f3c, 0x420ebecb, 0x6da336dc, 0x4239c7aa, 0x6d893d93, 0x4264c653, 0x6d6f3365,
0x428fbabe, 0x6d551858, 0x42baa4e6, 0x6d3aec6e, 0x42e584c3, 0x6d20afac, 0x43105a50, 0x6d066215,
0x433b2585, 0x6cec03af, 0x4365e65b, 0x6cd1947c, 0x43909ccd, 0x6cb71482, 0x43bb48d4, 0x6c9c83c3,
0x43e5ea68, 0x6c81e245, 0x44108184, 0x6c67300b, 0x443b0e21, 0x6c4c6d1a, 0x44659039, 0x6c319975,
0x449007c4, 0x6c16b521, 0x44ba74bd, 0x6bfbc021, 0x44e4d71c, 0x6be0ba7b, 0x450f2edb, 0x6bc5a431,
0x45397bf4, 0x6baa7d49, 0x4563be60, 0x6b8f45c7, 0x458df619, 0x6b73fdae, 0x45b82318, 0x6b58a503,
0x45e24556, 0x6b3d3bcb, 0x460c5cce, 0x6b21c208, 0x46366978, 0x6b0637c1, 0x46606b4e, 0x6aea9cf8,
0x468a624a, 0x6acef1b2, 0x46b44e65, 0x6ab335f4, 0x46de2f99, 0x6a9769c1, 0x470805df, 0x6a7b8d1e,
0x4731d131, 0x6a5fa010, 0x475b9188, 0x6a43a29a, 0x478546de, 0x6a2794c1, 0x47aef12c, 0x6a0b7689,
0x47d8906d, 0x69ef47f6, 0x48022499, 0x69d3090e, 0x482badab, 0x69b6b9d3, 0x48552b9b, 0x699a5a4c,
0x487e9e64, 0x697dea7b, 0x48a805ff, 0x69616a65, 0x48d16265, 0x6944da10, 0x48fab391, 0x6928397e,
0x4923f97b, 0x690b88b5, 0x494d341e, 0x68eec7b9, 0x49766373, 0x68d1f68f, 0x499f8774, 0x68b5153a,
0x49c8a01b, 0x689823bf, 0x49f1ad61, 0x687b2224, 0x4a1aaf3f, 0x685e106c, 0x4a43a5b0, 0x6840ee9b,
0x4a6c90ad, 0x6823bcb7, 0x4a957030, 0x68067ac3, 0x4abe4433, 0x67e928c5, 0x4ae70caf, 0x67cbc6c0,
0x4b0fc99d, 0x67ae54ba, 0x4b387af9, 0x6790d2b6, 0x4b6120bb, 0x677340ba, 0x4b89badd, 0x67559eca,
0x4bb24958, 0x6737ecea, 0x4bdacc28, 0x671a2b20, 0x4c034345, 0x66fc596f, 0x4c2baea9, 0x66de77dc,
0x4c540e4e, 0x66c0866d, 0x4c7c622d, 0x66a28524, 0x4ca4aa41, 0x66847408, 0x4ccce684, 0x6666531d,
0x4cf516ee, 0x66482267, 0x4d1d3b7a, 0x6629e1ec, 0x4d455422, 0x660b91af, 0x4d6d60df, 0x65ed31b5,
0x4d9561ac, 0x65cec204, 0x4dbd5682, 0x65b0429f, 0x4de53f5a, 0x6591b38c, 0x4e0d1c30, 0x657314cf,
0x4e34ecfc, 0x6554666d, 0x4e5cb1b9, 0x6535a86b, 0x4e846a60, 0x6516dacd, 0x4eac16eb, 0x64f7fd98,
0x4ed3b755, 0x64d910d1, 0x4efb4b96, 0x64ba147d, 0x4f22d3aa, 0x649b08a0, 0x4f4a4f89, 0x647bed3f,
0x4f71bf2e, 0x645cc260, 0x4f992293, 0x643d8806, 0x4fc079b1, 0x641e3e38, 0x4fe7c483, 0x63fee4f8,
0x500f0302, 0x63df7c4d, 0x50363529, 0x63c0043b, 0x505d5af1, 0x63a07cc7, 0x50847454, 0x6380e5f6,
0x50ab814d, 0x63613fcd, 0x50d281d5, 0x63418a50, 0x50f975e6, 0x6321c585, 0x51205d7b, 0x6301f171,
0x5147388c, 0x62e20e17, 0x516e0715, 0x62c21b7e, 0x5194c910, 0x62a219aa, 0x51bb7e75, 0x628208a1,
0x51e22740, 0x6261e866, 0x5208c36a, 0x6241b8ff, 0x522f52ee, 0x62217a72, 0x5255d5c5, 0x62012cc2,
0x527c4bea, 0x61e0cff5, 0x52a2b556, 0x61c06410, 0x52c91204, 0x619fe918, 0x52ef61ee, 0x617f5f12,
0x5315a50e, 0x615ec603, 0x533bdb5d, 0x613e1df0, 0x536204d7, 0x611d66de, 0x53882175, 0x60fca0d2,
0x53ae3131, 0x60dbcbd1, 0x53d43406, 0x60bae7e1, 0x53fa29ed, 0x6099f505, 0x542012e1, 0x6078f344,
0x5445eedb, 0x6057e2a2, 0x546bbdd7, 0x6036c325, 0x54917fce, 0x601594d1, 0x54b734ba, 0x5ff457ad,
0x54dcdc96, 0x5fd30bbc, 0x5502775c, 0x5fb1b104, 0x55280505, 0x5f90478a, 0x554d858d, 0x5f6ecf53,
0x5572f8ed, 0x5f4d4865, 0x55985f20, 0x5f2bb2c5, 0x55bdb81f, 0x5f0a0e77, 0x55e303e6, 0x5ee85b82,
0x5608426e, 0x5ec699e9, 0x562d73b2, 0x5ea4c9b3, 0x565297ab, 0x5e82eae5, 0x5677ae54, 0x5e60fd84,
0x569cb7a8, 0x5e3f0194, 0x56c1b3a1, 0x5e1cf71c, 0x56e6a239, 0x5dfade20, 0x570b8369, 0x5dd8b6a7,
0x5730572e, 0x5db680b4, 0x57551d80, 0x5d943c4e, 0x5779d65b, 0x5d71e979, 0x579e81b8, 0x5d4f883b,
0x57c31f92, 0x5d2d189a, 0x57e7afe4, 0x5d0a9a9a, 0x580c32a7, 0x5ce80e41, 0x5830a7d6, 0x5cc57394,
0x58550f6c, 0x5ca2ca99, 0x58796962, 0x5c801354, 0x589db5b3, 0x5c5d4dcc, 0x58c1f45b, 0x5c3a7a05,
0x58e62552, 0x5c179806, 0x590a4893, 0x5bf4a7d2, 0x592e5e19, 0x5bd1a971, 0x595265df, 0x5bae9ce7,
0x59765fde, 0x5b8b8239, 0x599a4c12, 0x5b68596d, 0x59be2a74, 0x5b452288, 0x59e1faff, 0x5b21dd90,
0x5a05bdae, 0x5afe8a8b, 0x5a29727b, 0x5adb297d, 0x5a4d1960, 0x5ab7ba6c, 0x5a70b258, 0x5a943d5e,
};

const int kbdWindowOffset[NUM_IMDCT_SIZES] PROGMEM = {0, 128};

const int kbdWindow[128 + 1024] PROGMEM = {
/* 128 - format = Q31 * 2^0 */
0x00016f63, 0x7ffffffe, 0x0003e382, 0x7ffffff1, 0x00078f64, 0x7fffffc7, 0x000cc323, 0x7fffff5d,
0x0013d9ed, 0x7ffffe76, 0x001d3a9d, 0x7ffffcaa, 0x0029581f, 0x7ffff953, 0x0038b1bd, 0x7ffff372,
0x004bd34d, 0x7fffe98b, 0x00635538, 0x7fffd975, 0x007fdc64, 0x7fffc024, 0x00a219f1, 0x7fff995b,
0x00cacad0, 0x7fff5f5b, 0x00fab72d, 0x7fff0a75, 0x0132b1af, 0x7ffe9091, 0x01739689, 0x7ffde49e,
0x01be4a63, 0x7ffcf5ef, 0x0213b910, 0x7ffbaf84, 0x0274d41e, 0x7ff9f73a, 0x02e2913a, 0x7ff7acf1,
0x035de86c, 0x7ff4a99a, 0x03e7d233, 0x7ff0be3d, 0x0481457c, 0x7febb2f1, 0x052b357c, 0x7fe545d4,
0x05e68f77, 0x7fdd2a02, 0x06b4386f, 0x7fd30695, 0x07950acb, 0x7fc675b4, 0x0889d3ef, 0x7fb703be,
0x099351e0, 0x7fa42e89, 0x0ab230e0, 0x7f8d64d8, 0x0be70923, 0x7f7205f8, 0x0d325c93, 0x7f516195,
0x0e9494ae, 0x7f2ab7d0, 0x100e0085, 0x7efd3997, 0x119ed2ef, 0x7ec8094a, 0x134720d8, 0x7e8a3ba7,
0x1506dfdc, 0x7e42d906, 0x16dde50b, 0x7df0dee4, 0x18cbe3f7, 0x7d9341b4, 0x1ad06e07, 0x7d28ef02,
0x1ceaf215, 0x7cb0cfcc, 0x1f1abc4f, 0x7c29cb20, 0x215ef677, 0x7b92c8eb, 0x23b6a867, 0x7aeab4ec,
0x2620b8ec, 0x7a3081d0, 0x289beef5, 0x79632c5a, 0x2b26f30b, 0x7881be95, 0x2dc0511f, 0x778b5304,
0x30667aa2, 0x767f17c0, 0x3317c8dd, 0x755c5178, 0x35d27f98, 0x74225e50, 0x3894cff3, 0x72d0b887,
0x3b5cdb7b, 0x7166f8e7, 0x3e28b770, 0x6fe4d8e8, 0x40f6702a, 0x6e4a3491, 0x43c40caa, 0x6c970bfc,
0x468f9231, 0x6acb8483, 0x495707f5, 0x68e7e994, 0x4c187ac7, 0x66ecad1c, 0x4ed200c5, 0x64da6797,
0x5181bcea, 0x62b1d7b7, 0x5425e28e, 0x6073e1ae, 0x56bcb8c2, 0x5e218e16, 0x59449d76, 0x5bbc0875,
/* 1024 - format = Q31 * 2^0 */
0x0009962f, 0x7fffffa4, 0x000e16fb, 0x7fffff39, 0x0011ea65, 0x7ffffebf, 0x0015750e, 0x7ffffe34,
0x0018dc74, 0x7ffffd96, 0x001c332e, 0x7ffffce5, 0x001f83f5, 0x7ffffc1f, 0x0022d59a, 0x7ffffb43,
0x00262cc2, 0x7ffffa4f, 0x00298cc4, 0x7ffff942, 0x002cf81f, 0x7ffff81a, 0x003070c4, 0x7ffff6d6,
0x0033f840, 0x7ffff573, 0x00378fd9, 0x7ffff3f1, 0x003b38a1, 0x7ffff24d, 0x003ef381, 0x7ffff085,
0x0042c147, 0x7fffee98, 0x0046a2a8, 0x7fffec83, 0x004a9847, 0x7fffea44, 0x004ea2b7, 0x7fffe7d8,
0x0052c283, 0x7fffe53f, 0x0056f829, 0x7fffe274, 0x005b4422, 0x7fffdf76, 0x005fa6dd, 0x7fffdc43,
0x006420c8, 0x7fffd8d6, 0x0068b249, 0x7fffd52f, 0x006d5bc4, 0x7fffd149, 0x00721d9a, 0x7fffcd22,
0x0076f828, 0x7fffc8b6, 0x007bebca, 0x7fffc404, 0x0080f8d9, 0x7fffbf06, 0x00861fae, 0x7fffb9bb,
0x008b609e, 0x7fffb41e, 0x0090bbff, 0x7fffae2c, 0x00963224, 0x7fffa7e1, 0x009bc362, 0x7fffa13a,
0x00a17009, 0x7fff9a32, 0x00a7386c, 0x7fff92c5, 0x00ad1cdc, 0x7fff8af0, 0x00b31da8, 0x7fff82ad,
0x00b93b21, 0x7fff79f9, 0x00bf7596, 0x7fff70cf, 0x00c5cd57, 0x7fff672a, 0x00cc42b1, 0x7fff5d05,
0x00d2d5f3, 0x7fff525c, 0x00d9876c, 0x7fff4729, 0x00e05769, 0x7fff3b66, 0x00e74638, 0x7fff2f10,
0x00ee5426, 0x7fff221f, 0x00f58182, 0x7fff148e, 0x00fcce97, 0x7fff0658, 0x01043bb3, 0x7ffef776,
0x010bc923, 0x7ffee7e2, 0x01137733, 0x7ffed795, 0x011b4631, 0x7ffec68a, 0x01233669, 0x7ffeb4ba,
0x012b4827, 0x7ffea21d, 0x01337bb8, 0x7ffe8eac, 0x013bd167, 0x7ffe7a61, 0x01444982, 0x7ffe6533,
0x014ce454, 0x7ffe4f1c, 0x0155a229, 0x7ffe3813, 0x015e834d, 0x7ffe2011, 0x0167880c, 0x7ffe070d,
0x0170b0b2, 0x7ffdecff, 0x0179fd8b, 0x7ffdd1df, 0x01836ee1, 0x7ffdb5a2, 0x018d0500, 0x7ffd9842,
0x0196c035, 0x7ffd79b3, 0x01a0a0ca, 0x7ffd59ee, 0x01aaa70a, 0x7ffd38e8, 0x01b4d341, 0x7ffd1697,
0x01bf25b9, 0x7ffcf2f2, 0x01c99ebd, 0x7ffccdee, 0x01d43e99, 0x7ffca780, 0x01df0597, 0x7ffc7f9e,
0x01e9f401, 0x7ffc563d, 0x01f50a22, 0x7ffc2b51, 0x02004844, 0x7ffbfecf, 0x020baeb1, 0x7ffbd0ab,
0x02173db4, 0x7ffba0da, 0x0222f596, 0x7ffb6f4f, 0x022ed6a1, 0x7ffb3bfd, 0x023ae11f, 0x7ffb06d8,
0x02471558, 0x7ffacfd3, 0x02537397, 0x7ffa96e0, 0x025ffc25, 0x7ffa5bf2, 0x026caf4a, 0x7ffa1efc,
0x02798d4f, 0x7ff9dfee, 0x0286967c, 0x7ff99ebb, 0x0293cb1b, 0x7ff95b55, 0x02a12b72, 0x7ff915ab,
0x02aeb7cb, 0x7ff8cdaf, 0x02bc706d, 0x7ff88351, 0x02ca559f, 0x7ff83682, 0x02d867a9, 0x7ff7e731,
0x02e6a6d2, 0x7ff7954e, 0x02f51361, 0x7ff740c8, 0x0303ad9c, 0x7ff6e98e, 0x031275ca, 0x7ff68f8f,
0x03216c30, 0x7ff632ba, 0x03309116, 0x7ff5d2fb, 0x033fe4bf, 0x7ff57042, 0x034f6773, 0x7ff50a7a,
0x035f1975, 0x7ff4a192, 0x036efb0a, 0x7ff43576, 0x037f0c78, 0x7ff3c612, 0x038f4e02, 0x7ff35353,
0x039fbfeb, 0x7ff2dd24, 0x03b06279, 0x7ff26370, 0x03c135ed, 0x7ff1e623, 0x03d23a8b, 0x7ff16527,
0x03e37095, 0x7ff0e067, 0x03f4d84e, 0x7ff057cc, 0x040671f7, 0x7fefcb40, 0x04183dd3, 0x7fef3aad,
0x042a3c22, 0x7feea5fa, 0x043c6d25, 0x7fee0d11, 0x044ed11d, 0x7fed6fda, 0x04616849, 0x7fecce3d,
0x047432eb, 0x7fec2821, 0x04873140, 0x7feb7d6c, 0x049a6388, 0x7feace07, 0x04adca01, 0x7fea19d6,
0x04c164ea, 0x7fe960c0, 0x04d53481, 0x7fe8a2aa, 0x04e93902, 0x7fe7df79, 0x04fd72aa, 0x7fe71712,
0x0511e1b6, 0x7fe6495a, 0x05268663, 0x7fe57634, 0x053b60eb, 0x7fe49d83, 0x05507189, 0x7fe3bf2b,
0x0565b879, 0x7fe2db0f, 0x057b35f4, 0x7fe1f110, 0x0590ea35, 0x7fe10111, 0x05a6d574, 0x7fe00af3,
0x05bcf7ea, 0x7fdf0e97, 0x05d351cf, 0x7fde0bdd, 0x05e9e35c, 0x7fdd02a6, 0x0600acc8, 0x7fdbf2d2,
0x0617ae48, 0x7fdadc40, 0x062ee814, 0x7fd9becf, 0x06465a62, 0x7fd89a5e, 0x065e0565, 0x7fd76eca,
0x0675e954, 0x7fd63bf1, 0x068e0662, 0x7fd501b0, 0x06a65cc3, 0x7fd3bfe4, 0x06beecaa, 0x7fd2766a,
0x06d7b648, 0x7fd1251e, 0x06f0b9d1, 0x7fcfcbda, 0x0709f775, 0x7fce6a7a, 0x07236f65, 0x7fcd00d8,
0x073d21d2, 0x7fcb8ecf, 0x07570eea, 0x7fca1439, 0x077136dd, 0x7fc890ed, 0x078b99da, 0x7fc704c7,
0x07a6380d, 0x7fc56f9d, 0x07c111a4, 0x7fc3d147, 0x07dc26cc, 0x7fc2299e, 0x07f777b1, 0x7fc07878,
0x0813047d, 0x7fbebdac, 0x082ecd5b, 0x7fbcf90f, 0x084ad276, 0x7fbb2a78, 0x086713f7, 0x7fb951bc,
0x08839206, 0x7fb76eaf, 0x08a04ccb, 0x7fb58126, 0x08bd446e, 0x7fb388f4, 0x08da7915, 0x7fb185ee,
0x08f7eae7, 0x7faf77e5, 0x09159a09, 0x7fad5ead, 0x0933869f, 0x7fab3a17, 0x0951b0cd, 0x7fa909f6,
0x097018b7, 0x7fa6ce1a, 0x098ebe7f, 0x7fa48653, 0x09ada248, 0x7fa23273, 0x09ccc431, 0x7f9fd249,
0x09ec245b, 0x7f9d65a4, 0x0a0bc2e7, 0x7f9aec53, 0x0a2b9ff3, 0x7f986625, 0x0a4bbb9e, 0x7f95d2e7,
0x0a6c1604, 0x7f933267, 0x0a8caf43, 0x7f908472, 0x0aad8776, 0x7f8dc8d5, 0x0ace9eb9, 0x7f8aff5c,
0x0aeff526, 0x7f8827d3, 0x0b118ad8, 0x7f854204, 0x0b335fe6, 0x7f824dbb, 0x0b557469, 0x7f7f4ac3,
0x0b77c879, 0x7f7c38e4, 0x0b9a5c2b, 0x7f7917e9, 0x0bbd2f97, 0x7f75e79b, 0x0be042d0, 0x7f72a7c3,
0x0c0395ec, 0x7f6f5828, 0x0c2728fd, 0x7f6bf892, 0x0c4afc16, 0x7f6888c9, 0x0c6f0f4a, 0x7f650894,
0x0c9362a8, 0x7f6177b9, 0x0cb7f642, 0x7f5dd5ff, 0x0cdcca26, 0x7f5a232a, 0x0d01de63, 0x7f565f00,
0x0d273307, 0x7f528947, 0x0d4cc81f, 0x7f4ea1c2, 0x0d729db7, 0x7f4aa835, 0x0d98b3da, 0x7f469c65,
0x0dbf0a92, 0x7f427e13, 0x0de5a1e9, 0x7f3e4d04, 0x0e0c79e7, 0x7f3a08f9, 0x0e339295, 0x7f35b1b4,
0x0e5aebfa, 0x7f3146f8, 0x0e82861a, 0x7f2cc884, 0x0eaa60fd, 0x7f28361b, 0x0ed27ca5, 0x7f238f7c,
0x0efad917, 0x7f1ed467, 0x0f237656, 0x7f1a049d, 0x0f4c5462, 0x7f151fdc, 0x0f75733d, 0x7f1025e3,
0x0f9ed2e6, 0x7f0b1672, 0x0fc8735e, 0x7f05f146, 0x0ff254a1, 0x7f00b61d, 0x101c76ae, 0x7efb64b4,
0x1046d981, 0x7ef5fcca, 0x10717d15, 0x7ef07e19, 0x109c6165, 0x7eeae860, 0x10c7866a, 0x7ee53b5b,
0x10f2ec1e, 0x7edf76c4, 0x111e9279, 0x7ed99a58, 0x114a7971, 0x7ed3a5d1, 0x1176a0fc, 0x7ecd98eb,
0x11a30910, 0x7ec77360, 0x11cfb1a1, 0x7ec134eb, 0x11fc9aa2, 0x7ebadd44, 0x1229c406, 0x7eb46c27,
0x12572dbf, 0x7eade14c, 0x1284d7bc, 0x7ea73c6c, 0x12b2c1ed, 0x7ea07d41, 0x12e0ec42, 0x7e99a382,
0x130f56a8, 0x7e92aee7, 0x133e010b, 0x7e8b9f2a, 0x136ceb59, 0x7e847402, 0x139c157b, 0x7e7d2d25,
0x13cb7f5d, 0x7e75ca4c, 0x13fb28e6, 0x7e6e4b2d, 0x142b1200, 0x7e66af7f, 0x145b3a92, 0x7e5ef6f8,
0x148ba281, 0x7e572150, 0x14bc49b4, 0x7e4f2e3b, 0x14ed300f, 0x7e471d70, 0x151e5575, 0x7e3eeea5,
0x154fb9c9, 0x7e36a18e, 0x15815ced, 0x7e2e35e2, 0x15b33ec1, 0x7e25ab56, 0x15e55f25, 0x7e1d019e,
0x1617bdf9, 0x7e14386e, 0x164a5b19, 0x7e0b4f7d, 0x167d3662, 0x7e02467e, 0x16b04fb2, 0x7df91d25,
0x16e3a6e2, 0x7defd327, 0x17173bce, 0x7de66837, 0x174b0e4d, 0x7ddcdc0a, 0x177f1e39, 0x7dd32e53,
0x17b36b69, 0x7dc95ec6, 0x17e7f5b3, 0x7dbf6d17, 0x181cbcec, 0x7db558f9, 0x1851c0e9, 0x7dab221f,
0x1887017d, 0x7da0c83c, 0x18bc7e7c, 0x7d964b05, 0x18f237b6, 0x7d8baa2b, 0x19282cfd, 0x7d80e563,
0x195e5e20, 0x7d75fc5e, 0x1994caee, 0x7d6aeed0, 0x19cb7335, 0x7d5fbc6d, 0x1a0256c2, 0x7d5464e6,
0x1a397561, 0x7d48e7ef, 0x1a70cede, 0x7d3d453b, 0x1aa86301, 0x7d317c7c, 0x1ae03195, 0x7d258d65,
0x1b183a63, 0x7d1977aa, 0x1b507d30, 0x7d0d3afc, 0x1b88f9c5, 0x7d00d710, 0x1bc1afe6, 0x7cf44b97,
0x1bfa9f58, 0x7ce79846, 0x1c33c7e0, 0x7cdabcce, 0x1c6d293f, 0x7ccdb8e4, 0x1ca6c337, 0x7cc08c39,
0x1ce0958a, 0x7cb33682, 0x1d1a9ff8, 0x7ca5b772, 0x1d54e240, 0x7c980ebd, 0x1d8f5c21, 0x7c8a3c14,
0x1dca0d56, 0x7c7c3f2e, 0x1e04f59f, 0x7c6e17bc, 0x1e4014b4, 0x7c5fc573, 0x1e7b6a53, 0x7c514807,
0x1eb6f633, 0x7c429f2c, 0x1ef2b80f, 0x7c33ca96, 0x1f2eaf9e, 0x7c24c9fa, 0x1f6adc98, 0x7c159d0d,
0x1fa73eb2, 0x7c064383, 0x1fe3d5a3, 0x7bf6bd11, 0x2020a11e, 0x7be7096c, 0x205da0d8, 0x7bd7284a,
0x209ad483, 0x7bc71960, 0x20d83bd1, 0x7bb6dc65, 0x2115d674, 0x7ba6710d, 0x2153a41b, 0x7b95d710,
0x2191a476, 0x7b850e24, 0x21cfd734, 0x7b7415ff, 0x220e3c02, 0x7b62ee59, 0x224cd28d, 0x7b5196e9,
0x228b9a82, 0x7b400f67, 0x22ca938a, 0x7b2e578a, 0x2309bd52, 0x7b1c6f0b, 0x23491783, 0x7b0a55a1,
0x2388a1c4, 0x7af80b07, 0x23c85bbf, 0x7ae58ef5, 0x2408451a, 0x7ad2e124, 0x24485d7c, 0x7ac0014e,
0x2488a48a, 0x7aacef2e, 0x24c919e9, 0x7a99aa7e, 0x2509bd3d, 0x7a8632f8, 0x254a8e29, 0x7a728858,
0x258b8c50, 0x7a5eaa5a, 0x25ccb753, 0x7a4a98b9, 0x260e0ed3, 0x7a365333, 0x264f9271, 0x7a21d983,
0x269141cb, 0x7a0d2b68, 0x26d31c80, 0x79f8489e, 0x2715222f, 0x79e330e4, 0x27575273, 0x79cde3f8,
0x2799acea, 0x79b8619a, 0x27dc3130, 0x79a2a989, 0x281ededf, 0x798cbb85, 0x2861b591, 0x7976974e,
0x28a4b4e0, 0x79603ca5, 0x28e7dc65, 0x7949ab4c, 0x292b2bb8, 0x7932e304, 0x296ea270, 0x791be390,
0x29b24024, 0x7904acb3, 0x29f6046b, 0x78ed3e30, 0x2a39eed8, 0x78d597cc, 0x2a7dff02, 0x78bdb94a,
0x2ac2347c, 0x78a5a270, 0x2b068eda, 0x788d5304, 0x2b4b0dae, 0x7874cacb, 0x2b8fb08a, 0x785c098d,
0x2bd47700, 0x78430f11, 0x2c1960a1, 0x7829db1f, 0x2c5e6cfd, 0x78106d7f, 0x2ca39ba3, 0x77f6c5fb,
0x2ce8ec23, 0x77dce45c, 0x2d2e5e0b, 0x77c2c86e, 0x2d73f0e8, 0x77a871fa, 0x2db9a449, 0x778de0cd,
0x2dff77b8, 0x777314b2, 0x2e456ac4, 0x77580d78, 0x2e8b7cf6, 0x773ccaeb, 0x2ed1addb, 0x77214cdb,
0x2f17fcfb, 0x77059315, 0x2f5e69e2, 0x76e99d69, 0x2fa4f419, 0x76cd6ba9, 0x2feb9b27, 0x76b0fda4,
0x30325e96, 0x7694532e, 0x30793dee, 0x76776c17, 0x30c038b5, 0x765a4834, 0x31074e72, 0x763ce759,
0x314e7eab, 0x761f4959, 0x3195c8e6, 0x76016e0b, 0x31dd2ca9, 0x75e35545, 0x3224a979, 0x75c4fedc,
0x326c3ed8, 0x75a66aab, 0x32b3ec4d, 0x75879887, 0x32fbb159, 0x7568884b, 0x33438d81, 0x754939d1,
0x338b8045, 0x7529acf4, 0x33d3892a, 0x7509e18e, 0x341ba7b1, 0x74e9d77d, 0x3463db5a, 0x74c98e9e,
0x34ac23a7, 0x74a906cd, 0x34f48019, 0x74883fec, 0x353cf02f, 0x746739d8, 0x3585736a, 0x7445f472,
0x35ce0949, 0x74246f9c, 0x3616b14c, 0x7402ab37, 0x365f6af0, 0x73e0a727, 0x36a835b5, 0x73be6350,
0x36f11118, 0x739bdf95, 0x3739fc98, 0x73791bdd, 0x3782f7b2, 0x7356180e, 0x37cc01e3, 0x7332d410,
0x38151aa8, 0x730f4fc9, 0x385e417e, 0x72eb8b24, 0x38a775e1, 0x72c7860a, 0x38f0b74d, 0x72a34066,
0x393a053e, 0x727eba24, 0x39835f30, 0x7259f331, 0x39ccc49e, 0x7234eb79, 0x3a163503, 0x720fa2eb,
0x3a5fafda, 0x71ea1977, 0x3aa9349e, 0x71c44f0c, 0x3af2c2ca, 0x719e439d, 0x3b3c59d7, 0x7177f71a,
0x3b85f940, 0x71516978, 0x3bcfa07e, 0x712a9aaa, 0x3c194f0d, 0x71038aa4, 0x3c630464, 0x70dc395e,
0x3cacbfff, 0x70b4a6cd, 0x3cf68155, 0x708cd2e9, 0x3d4047e1, 0x7064bdab, 0x3d8a131c, 0x703c670d,
0x3dd3e27e, 0x7013cf0a, 0x3e1db580, 0x6feaf59c, 0x3e678b9b, 0x6fc1dac1, 0x3eb16449, 0x6f987e76,
0x3efb3f01, 0x6f6ee0b9, 0x3f451b3d, 0x6f45018b, 0x3f8ef874, 0x6f1ae0eb, 0x3fd8d620, 0x6ef07edb,
0x4022b3b9, 0x6ec5db5d, 0x406c90b7, 0x6e9af675, 0x40b66c93, 0x6e6fd027, 0x410046c5, 0x6e446879,
0x414a1ec6, 0x6e18bf71, 0x4193f40d, 0x6decd517, 0x41ddc615, 0x6dc0a972, 0x42279455, 0x6d943c8d,
0x42715e45, 0x6d678e71, 0x42bb235f, 0x6d3a9f2a, 0x4304e31a, 0x6d0d6ec5, 0x434e9cf1, 0x6cdffd4f,
0x4398505b, 0x6cb24ad6, 0x43e1fcd1, 0x6c84576b, 0x442ba1cd, 0x6c56231c, 0x44753ec7, 0x6c27adfd,
0x44bed33a, 0x6bf8f81e, 0x45085e9d, 0x6bca0195, 0x4551e06b, 0x6b9aca75, 0x459b581e, 0x6b6b52d5,
0x45e4c52f, 0x6b3b9ac9, 0x462e2717, 0x6b0ba26b, 0x46777d52, 0x6adb69d3, 0x46c0c75a, 0x6aaaf11b,
0x470a04a9, 0x6a7a385c, 0x475334b9, 0x6a493fb3, 0x479c5707, 0x6a18073d, 0x47e56b0c, 0x69e68f17,
0x482e7045, 0x69b4d761, 0x4877662c, 0x6982e039, 0x48c04c3f, 0x6950a9c0, 0x490921f8, 0x691e341a,
0x4951e6d5, 0x68eb7f67, 0x499a9a51, 0x68b88bcd, 0x49e33beb, 0x68855970, 0x4a2bcb1f, 0x6851e875,
0x4a74476b, 0x681e3905, 0x4abcb04c, 0x67ea4b47, 0x4b050541, 0x67b61f63, 0x4b4d45c9, 0x6781b585,
0x4b957162, 0x674d0dd6, 0x4bdd878c, 0x67182883, 0x4c2587c6, 0x66e305b8, 0x4c6d7190, 0x66ada5a5,
0x4cb5446a, 0x66780878, 0x4cfcffd5, 0x66422e60, 0x4d44a353, 0x660c1790, 0x4d8c2e64, 0x65d5c439,
0x4dd3a08c, 0x659f348e, 0x4e1af94b, 0x656868c3, 0x4e623825, 0x6531610d, 0x4ea95c9d, 0x64fa1da3,
0x4ef06637, 0x64c29ebb, 0x4f375477, 0x648ae48d, 0x4f7e26e1, 0x6452ef53, 0x4fc4dcfb, 0x641abf46,
0x500b7649, 0x63e254a2, 0x5051f253, 0x63a9afa2, 0x5098509f, 0x6370d083, 0x50de90b3, 0x6337b784,
0x5124b218, 0x62fe64e3, 0x516ab455, 0x62c4d8e0, 0x51b096f3, 0x628b13bc, 0x51f6597b, 0x625115b8,
0x523bfb78, 0x6216df18, 0x52817c72, 0x61dc701f, 0x52c6dbf5, 0x61a1c912, 0x530c198d, 0x6166ea36,
0x535134c5, 0x612bd3d2, 0x53962d2a, 0x60f0862d, 0x53db024a, 0x60b50190, 0x541fb3b1, 0x60794644,
0x546440ef, 0x603d5494, 0x54a8a992, 0x60012cca, 0x54eced2b, 0x5fc4cf33, 0x55310b48, 0x5f883c1c,
0x5575037c, 0x5f4b73d2, 0x55b8d558, 0x5f0e76a5, 0x55fc806f, 0x5ed144e5, 0x56400452, 0x5e93dee1,
0x56836096, 0x5e5644ec, 0x56c694cf, 0x5e187757, 0x5709a092, 0x5dda7677, 0x574c8374, 0x5d9c429f,
0x578f3d0d, 0x5d5ddc24, 0x57d1ccf2, 0x5d1f435d, 0x581432bd, 0x5ce078a0, 0x58566e04, 0x5ca17c45,
0x58987e63, 0x5c624ea4, 0x58da6372, 0x5c22f016, 0x591c1ccc, 0x5be360f6, 0x595daa0d, 0x5ba3a19f,
0x599f0ad1, 0x5b63b26c, 0x59e03eb6, 0x5b2393ba, 0x5a214558, 0x5ae345e7, 0x5a621e56, 0x5aa2c951,
};
/* bit reverse tables for FFT */
const uint8_t bitrevtabOffset[NUM_IMDCT_SIZES] PROGMEM = {0, 17};
const uint8_t bitrevtab[17 + 129] PROGMEM = {
/* nfft = 64 */
0x01, 0x08, 0x02, 0x04, 0x03, 0x0c, 0x05, 0x0a, 0x07, 0x0e, 0x0b, 0x0d, 0x00, 0x06, 0x09, 0x0f,
0x00,
/* nfft = 512 */
0x01, 0x40, 0x02, 0x20, 0x03, 0x60, 0x04, 0x10, 0x05, 0x50, 0x06, 0x30, 0x07, 0x70, 0x09, 0x48,
0x0a, 0x28, 0x0b, 0x68, 0x0c, 0x18, 0x0d, 0x58, 0x0e, 0x38, 0x0f, 0x78, 0x11, 0x44, 0x12, 0x24,
0x13, 0x64, 0x15, 0x54, 0x16, 0x34, 0x17, 0x74, 0x19, 0x4c, 0x1a, 0x2c, 0x1b, 0x6c, 0x1d, 0x5c,
0x1e, 0x3c, 0x1f, 0x7c, 0x21, 0x42, 0x23, 0x62, 0x25, 0x52, 0x26, 0x32, 0x27, 0x72, 0x29, 0x4a,
0x2b, 0x6a, 0x2d, 0x5a, 0x2e, 0x3a, 0x2f, 0x7a, 0x31, 0x46, 0x33, 0x66, 0x35, 0x56, 0x37, 0x76,
0x39, 0x4e, 0x3b, 0x6e, 0x3d, 0x5e, 0x3f, 0x7e, 0x43, 0x61, 0x45, 0x51, 0x47, 0x71, 0x4b, 0x69,
0x4d, 0x59, 0x4f, 0x79, 0x53, 0x65, 0x57, 0x75, 0x5b, 0x6d, 0x5f, 0x7d, 0x67, 0x73, 0x6f, 0x7b,
0x00, 0x08, 0x14, 0x1c, 0x22, 0x2a, 0x36, 0x3e, 0x41, 0x49, 0x55, 0x5d, 0x63, 0x6b, 0x77, 0x7f,
0x00,
};

const uint8_t uniqueIDTab[8] = {0x5f, 0x4b, 0x43, 0x5f, 0x5f, 0x4a, 0x52, 0x5f};

const uint32_t twidTabOdd[8*6 + 32*6 + 128*6] PROGMEM = {
    0x40000000, 0x00000000, 0x40000000, 0x00000000, 0x40000000, 0x00000000, 0x539eba45, 0xe7821d59,
    0x4b418bbe, 0xf383a3e2, 0x58c542c5, 0xdc71898d, 0x5a82799a, 0xd2bec333, 0x539eba45, 0xe7821d59,
    0x539eba45, 0xc4df2862, 0x539eba45, 0xc4df2862, 0x58c542c5, 0xdc71898d, 0x3248d382, 0xc13ad060,
    0x40000000, 0xc0000000, 0x5a82799a, 0xd2bec333, 0x00000000, 0xd2bec333, 0x22a2f4f8, 0xc4df2862,
    0x58c542c5, 0xcac933ae, 0xcdb72c7e, 0xf383a3e2, 0x00000000, 0xd2bec333, 0x539eba45, 0xc4df2862,
    0xac6145bb, 0x187de2a7, 0xdd5d0b08, 0xe7821d59, 0x4b418bbe, 0xc13ad060, 0xa73abd3b, 0x3536cc52,
    0x40000000, 0x00000000, 0x40000000, 0x00000000, 0x40000000, 0x00000000, 0x45f704f7, 0xf9ba1651,
    0x43103085, 0xfcdc1342, 0x48b2b335, 0xf69bf7c9, 0x4b418bbe, 0xf383a3e2, 0x45f704f7, 0xf9ba1651,
    0x4fd288dc, 0xed6bf9d1, 0x4fd288dc, 0xed6bf9d1, 0x48b2b335, 0xf69bf7c9, 0x553805f2, 0xe4a2eff6,
    0x539eba45, 0xe7821d59, 0x4b418bbe, 0xf383a3e2, 0x58c542c5, 0xdc71898d, 0x569cc31b, 0xe1d4a2c8,
    0x4da1fab5, 0xf0730342, 0x5a6690ae, 0xd5052d97, 0x58c542c5, 0xdc71898d, 0x4fd288dc, 0xed6bf9d1,
    0x5a12e720, 0xce86ff2a, 0x5a12e720, 0xd76619b6, 0x51d1dc80, 0xea70658a, 0x57cc15bc, 0xc91af976,
    0x5a82799a, 0xd2bec333, 0x539eba45, 0xe7821d59, 0x539eba45, 0xc4df2862, 0x5a12e720, 0xce86ff2a,
    0x553805f2, 0xe4a2eff6, 0x4da1fab5, 0xc1eb0209, 0x58c542c5, 0xcac933ae, 0x569cc31b, 0xe1d4a2c8,
    0x45f704f7, 0xc04ee4b8, 0x569cc31b, 0xc78e9a1d, 0x57cc15bc, 0xdf18f0ce, 0x3cc85709, 0xc013bc39,
    0x539eba45, 0xc4df2862, 0x58c542c5, 0xdc71898d, 0x3248d382, 0xc13ad060, 0x4fd288dc, 0xc2c17d52,
    0x5987b08a, 0xd9e01006, 0x26b2a794, 0xc3bdbdf6, 0x4b418bbe, 0xc13ad060, 0x5a12e720, 0xd76619b6,
    0x1a4608ab, 0xc78e9a1d, 0x45f704f7, 0xc04ee4b8, 0x5a6690ae, 0xd5052d97, 0x0d47d096, 0xcc983f70,
    0x40000000, 0xc0000000, 0x5a82799a, 0xd2bec333, 0x00000000, 0xd2bec333, 0x396b3199, 0xc04ee4b8,
    0x5a6690ae, 0xd09441bb, 0xf2b82f6a, 0xd9e01006, 0x3248d382, 0xc13ad060, 0x5a12e720, 0xce86ff2a,
    0xe5b9f755, 0xe1d4a2c8, 0x2aaa7c7f, 0xc2c17d52, 0x5987b08a, 0xcc983f70, 0xd94d586c, 0xea70658a,
    0x22a2f4f8, 0xc4df2862, 0x58c542c5, 0xcac933ae, 0xcdb72c7e, 0xf383a3e2, 0x1a4608ab, 0xc78e9a1d,
    0x57cc15bc, 0xc91af976, 0xc337a8f7, 0xfcdc1342, 0x11a855df, 0xcac933ae, 0x569cc31b, 0xc78e9a1d,
    0xba08fb09, 0x0645e9af, 0x08df1a8c, 0xce86ff2a, 0x553805f2, 0xc6250a18, 0xb25e054b, 0x0f8cfcbe,
    0x00000000, 0xd2bec333, 0x539eba45, 0xc4df2862, 0xac6145bb, 0x187de2a7, 0xf720e574, 0xd76619b6,
    0x51d1dc80, 0xc3bdbdf6, 0xa833ea44, 0x20e70f32, 0xee57aa21, 0xdc71898d, 0x4fd288dc, 0xc2c17d52,
    0xa5ed18e0, 0x2899e64a, 0xe5b9f755, 0xe1d4a2c8, 0x4da1fab5, 0xc1eb0209, 0xa5996f52, 0x2f6bbe45,
    0xdd5d0b08, 0xe7821d59, 0x4b418bbe, 0xc13ad060, 0xa73abd3b, 0x3536cc52, 0xd5558381, 0xed6bf9d1,
    0x48b2b335, 0xc0b15502, 0xaac7fa0e, 0x39daf5e8, 0xcdb72c7e, 0xf383a3e2, 0x45f704f7, 0xc04ee4b8,
    0xb02d7724, 0x3d3e82ae, 0xc694ce67, 0xf9ba1651, 0x43103085, 0xc013bc39, 0xb74d4ccb, 0x3f4eaafe,
    0x40000000, 0x00000000, 0x40000000, 0x00000000, 0x40000000, 0x00000000, 0x418d2621, 0xfe6deaa1,
    0x40c7d2bd, 0xff36f170, 0x424ff28f, 0xfda4f351, 0x43103085, 0xfcdc1342, 0x418d2621, 0xfe6deaa1,
    0x4488e37f, 0xfb4ab7db, 0x4488e37f, 0xfb4ab7db, 0x424ff28f, 0xfda4f351, 0x46aa0d6d, 0xf8f21e8e,
    0x45f704f7, 0xf9ba1651, 0x43103085, 0xfcdc1342, 0x48b2b335, 0xf69bf7c9, 0x475a5c77, 0xf82a6c6a,
    0x43cdd89a, 0xfc135231, 0x4aa22036, 0xf4491311, 0x48b2b335, 0xf69bf7c9, 0x4488e37f, 0xfb4ab7db,
    0x4c77a88e, 0xf1fa3ecb, 0x49ffd417, 0xf50ef5de, 0x454149fc, 0xfa824bfd, 0x4e32a956, 0xefb047f2,
    0x4b418bbe, 0xf383a3e2, 0x45f704f7, 0xf9ba1651, 0x4fd288dc, 0xed6bf9d1, 0x4c77a88e, 0xf1fa3ecb,
    0x46aa0d6d, 0xf8f21e8e, 0x5156b6d9, 0xeb2e1dbe, 0x4da1fab5, 0xf0730342, 0x475a5c77, 0xf82a6c6a,
    0x52beac9f, 0xe8f77acf, 0x4ec05432, 0xeeee2d9d, 0x4807eb4b, 0xf7630799, 0x5409ed4b, 0xe6c8d59c,
    0x4fd288dc, 0xed6bf9d1, 0x48b2b335, 0xf69bf7c9, 0x553805f2, 0xe4a2eff6, 0x50d86e6d, 0xebeca36c,
    0x495aada2, 0xf5d544a7, 0x56488dc5, 0xe28688a4, 0x51d1dc80, 0xea70658a, 0x49ffd417, 0xf50ef5de,
    0x573b2635, 0xe0745b24, 0x52beac9f, 0xe8f77acf, 0x4aa22036, 0xf4491311, 0x580f7b19, 0xde6d1f65,
    0x539eba45, 0xe7821d59, 0x4b418bbe, 0xf383a3e2, 0x58c542c5, 0xdc71898d, 0x5471e2e6, 0xe61086bc,
    0x4bde1089, 0xf2beafed, 0x595c3e2a, 0xda8249b4, 0x553805f2, 0xe4a2eff6, 0x4c77a88e, 0xf1fa3ecb,
    0x59d438e5, 0xd8a00bae, 0x55f104dc, 0xe3399167, 0x4d0e4de2, 0xf136580d, 0x5a2d0957, 0xd6cb76c9,
    0x569cc31b, 0xe1d4a2c8, 0x4da1fab5, 0xf0730342, 0x5a6690ae, 0xd5052d97, 0x573b2635, 0xe0745b24,
    0x4e32a956, 0xefb047f2, 0x5a80baf6, 0xd34dcdb4, 0x57cc15bc, 0xdf18f0ce, 0x4ec05432, 0xeeee2d9d,
    0x5a7b7f1a, 0xd1a5ef90, 0x584f7b58, 0xddc29958, 0x4f4af5d1, 0xee2cbbc1, 0x5a56deec, 0xd00e2639,
    0x58c542c5, 0xdc71898d, 0x4fd288dc, 0xed6bf9d1, 0x5a12e720, 0xce86ff2a, 0x592d59da, 0xdb25f566,
    0x50570819, 0xecabef3d, 0x59afaf4c, 0xcd110216, 0x5987b08a, 0xd9e01006, 0x50d86e6d, 0xebeca36c,
    0x592d59da, 0xcbacb0bf, 0x59d438e5, 0xd8a00bae, 0x5156b6d9, 0xeb2e1dbe, 0x588c1404, 0xca5a86c4,
    0x5a12e720, 0xd76619b6, 0x51d1dc80, 0xea70658a, 0x57cc15bc, 0xc91af976, 0x5a43b190, 0xd6326a88,
    0x5249daa2, 0xe9b38223, 0x56eda1a0, 0xc7ee77b3, 0x5a6690ae, 0xd5052d97, 0x52beac9f, 0xe8f77acf,
    0x55f104dc, 0xc6d569be, 0x5a7b7f1a, 0xd3de9156, 0x53304df6, 0xe83c56cf, 0x54d69714, 0xc5d03118,
    0x5a82799a, 0xd2bec333, 0x539eba45, 0xe7821d59, 0x539eba45, 0xc4df2862, 0x5a7b7f1a, 0xd1a5ef90,
    0x5409ed4b, 0xe6c8d59c, 0x5249daa2, 0xc402a33c, 0x5a6690ae, 0xd09441bb, 0x5471e2e6, 0xe61086bc,
    0x50d86e6d, 0xc33aee27, 0x5a43b190, 0xcf89e3e8, 0x54d69714, 0xe55937d5, 0x4f4af5d1, 0xc2884e6e,
    0x5a12e720, 0xce86ff2a, 0x553805f2, 0xe4a2eff6, 0x4da1fab5, 0xc1eb0209, 0x59d438e5, 0xcd8bbb6d,
    0x55962bc0, 0xe3edb628, 0x4bde1089, 0xc1633f8a, 0x5987b08a, 0xcc983f70, 0x55f104dc, 0xe3399167,
    0x49ffd417, 0xc0f1360b, 0x592d59da, 0xcbacb0bf, 0x56488dc5, 0xe28688a4, 0x4807eb4b, 0xc0950d1d,
    0x58c542c5, 0xcac933ae, 0x569cc31b, 0xe1d4a2c8, 0x45f704f7, 0xc04ee4b8, 0x584f7b58, 0xc9edeb50,
    0x56eda1a0, 0xe123e6ad, 0x43cdd89a, 0xc01ed535, 0x57cc15bc, 0xc91af976, 0x573b2635, 0xe0745b24,
    0x418d2621, 0xc004ef3f, 0x573b2635, 0xc8507ea7, 0x57854ddd, 0xdfc606f1, 0x3f35b59d, 0xc0013bd3,
    0x569cc31b, 0xc78e9a1d, 0x57cc15bc, 0xdf18f0ce, 0x3cc85709, 0xc013bc39, 0x55f104dc, 0xc6d569be,
    0x580f7b19, 0xde6d1f65, 0x3a45e1f7, 0xc03c6a07, 0x553805f2, 0xc6250a18, 0x584f7b58, 0xddc29958,
    0x37af354c, 0xc07b371e, 0x5471e2e6, 0xc57d965d, 0x588c1404, 0xdd196538, 0x350536f1, 0xc0d00db6,
    0x539eba45, 0xc4df2862, 0x58c542c5, 0xdc71898d, 0x3248d382, 0xc13ad060, 0x52beac9f, 0xc449d892,
    0x58fb0568, 0xdbcb0cce, 0x2f7afdfc, 0xc1bb5a11, 0x51d1dc80, 0xc3bdbdf6, 0x592d59da, 0xdb25f566,
    0x2c9caf6c, 0xc2517e31, 0x50d86e6d, 0xc33aee27, 0x595c3e2a, 0xda8249b4, 0x29aee694, 0xc2fd08a9,
    0x4fd288dc, 0xc2c17d52, 0x5987b08a, 0xd9e01006, 0x26b2a794, 0xc3bdbdf6, 0x4ec05432, 0xc2517e31,
    0x59afaf4c, 0xd93f4e9e, 0x23a8fb93, 0xc4935b3c, 0x4da1fab5, 0xc1eb0209, 0x59d438e5, 0xd8a00bae,
    0x2092f05f, 0xc57d965d, 0x4c77a88e, 0xc18e18a7, 0x59f54bee, 0xd8024d59, 0x1d719810, 0xc67c1e18,
    0x4b418bbe, 0xc13ad060, 0x5a12e720, 0xd76619b6, 0x1a4608ab, 0xc78e9a1d, 0x49ffd417, 0xc0f1360b,
    0x5a2d0957, 0xd6cb76c9, 0x17115bc0, 0xc8b4ab32, 0x48b2b335, 0xc0b15502, 0x5a43b190, 0xd6326a88,
    0x13d4ae08, 0xc9edeb50, 0x475a5c77, 0xc07b371e, 0x5a56deec, 0xd59afadb, 0x10911f04, 0xcb39edca,
    0x45f704f7, 0xc04ee4b8, 0x5a6690ae, 0xd5052d97, 0x0d47d096, 0xcc983f70, 0x4488e37f, 0xc02c64a6,
    0x5a72c63b, 0xd4710883, 0x09f9e6a1, 0xce0866b8, 0x43103085, 0xc013bc39, 0x5a7b7f1a, 0xd3de9156,
    0x06a886a0, 0xcf89e3e8, 0x418d2621, 0xc004ef3f, 0x5a80baf6, 0xd34dcdb4, 0x0354d741, 0xd11c3142,
    0x40000000, 0xc0000000, 0x5a82799a, 0xd2bec333, 0x00000000, 0xd2bec333, 0x3e68fb62, 0xc004ef3f,
    0x5a80baf6, 0xd2317756, 0xfcab28bf, 0xd4710883, 0x3cc85709, 0xc013bc39, 0x5a7b7f1a, 0xd1a5ef90,
    0xf9577960, 0xd6326a88, 0x3b1e5335, 0xc02c64a6, 0x5a72c63b, 0xd11c3142, 0xf606195f, 0xd8024d59,
    0x396b3199, 0xc04ee4b8, 0x5a6690ae, 0xd09441bb, 0xf2b82f6a, 0xd9e01006, 0x37af354c, 0xc07b371e,
    0x5a56deec, 0xd00e2639, 0xef6ee0fc, 0xdbcb0cce, 0x35eaa2c7, 0xc0b15502, 0x5a43b190, 0xcf89e3e8,
    0xec2b51f8, 0xddc29958, 0x341dbfd3, 0xc0f1360b, 0x5a2d0957, 0xcf077fe1, 0xe8eea440, 0xdfc606f1,
    0x3248d382, 0xc13ad060, 0x5a12e720, 0xce86ff2a, 0xe5b9f755, 0xe1d4a2c8, 0x306c2624, 0xc18e18a7,
    0x59f54bee, 0xce0866b8, 0xe28e67f0, 0xe3edb628, 0x2e88013a, 0xc1eb0209, 0x59d438e5, 0xcd8bbb6d,
    0xdf6d0fa1, 0xe61086bc, 0x2c9caf6c, 0xc2517e31, 0x59afaf4c, 0xcd110216, 0xdc57046d, 0xe83c56cf,
    0x2aaa7c7f, 0xc2c17d52, 0x5987b08a, 0xcc983f70, 0xd94d586c, 0xea70658a, 0x28b1b544, 0xc33aee27,
    0x595c3e2a, 0xcc217822, 0xd651196c, 0xecabef3d, 0x26b2a794, 0xc3bdbdf6, 0x592d59da, 0xcbacb0bf,
    0xd3635094, 0xeeee2d9d, 0x24ada23d, 0xc449d892, 0x58fb0568, 0xcb39edca, 0xd0850204, 0xf136580d,
    0x22a2f4f8, 0xc4df2862, 0x58c542c5, 0xcac933ae, 0xcdb72c7e, 0xf383a3e2, 0x2092f05f, 0xc57d965d,
    0x588c1404, 0xca5a86c4, 0xcafac90f, 0xf5d544a7, 0x1e7de5df, 0xc6250a18, 0x584f7b58, 0xc9edeb50,
    0xc850cab4, 0xf82a6c6a, 0x1c6427a9, 0xc6d569be, 0x580f7b19, 0xc9836582, 0xc5ba1e09, 0xfa824bfd,
    0x1a4608ab, 0xc78e9a1d, 0x57cc15bc, 0xc91af976, 0xc337a8f7, 0xfcdc1342, 0x1823dc7d, 0xc8507ea7,
    0x57854ddd, 0xc8b4ab32, 0xc0ca4a63, 0xff36f170, 0x15fdf758, 0xc91af976, 0x573b2635, 0xc8507ea7,
    0xbe72d9df, 0x0192155f, 0x13d4ae08, 0xc9edeb50, 0x56eda1a0, 0xc7ee77b3, 0xbc322766, 0x03ecadcf,
    0x11a855df, 0xcac933ae, 0x569cc31b, 0xc78e9a1d, 0xba08fb09, 0x0645e9af, 0x0f7944a7, 0xcbacb0bf,
    0x56488dc5, 0xc730e997, 0xb7f814b5, 0x089cf867, 0x0d47d096, 0xcc983f70, 0x55f104dc, 0xc6d569be,
    0xb6002be9, 0x0af10a22, 0x0b145041, 0xcd8bbb6d, 0x55962bc0, 0xc67c1e18, 0xb421ef77, 0x0d415013,
    0x08df1a8c, 0xce86ff2a, 0x553805f2, 0xc6250a18, 0xb25e054b, 0x0f8cfcbe, 0x06a886a0, 0xcf89e3e8,
    0x54d69714, 0xc5d03118, 0xb0b50a2f, 0x11d3443f, 0x0470ebdc, 0xd09441bb, 0x5471e2e6, 0xc57d965d,
    0xaf279193, 0x14135c94, 0x0238a1c6, 0xd1a5ef90, 0x5409ed4b, 0xc52d3d18, 0xadb6255e, 0x164c7ddd,
    0x00000000, 0xd2bec333, 0x539eba45, 0xc4df2862, 0xac6145bb, 0x187de2a7, 0xfdc75e3a, 0xd3de9156,
    0x53304df6, 0xc4935b3c, 0xab2968ec, 0x1aa6c82b, 0xfb8f1424, 0xd5052d97, 0x52beac9f, 0xc449d892,
    0xaa0efb24, 0x1cc66e99, 0xf9577960, 0xd6326a88, 0x5249daa2, 0xc402a33c, 0xa9125e60, 0x1edc1953,
    0xf720e574, 0xd76619b6, 0x51d1dc80, 0xc3bdbdf6, 0xa833ea44, 0x20e70f32, 0xf4ebafbf, 0xd8a00bae,
    0x5156b6d9, 0xc37b2b6a, 0xa773ebfc, 0x22e69ac8, 0xf2b82f6a, 0xd9e01006, 0x50d86e6d, 0xc33aee27,
    0xa6d2a626, 0x24da0a9a, 0xf086bb59, 0xdb25f566, 0x50570819, 0xc2fd08a9, 0xa65050b4, 0x26c0b162,
    0xee57aa21, 0xdc71898d, 0x4fd288dc, 0xc2c17d52, 0xa5ed18e0, 0x2899e64a, 0xec2b51f8, 0xddc29958,
    0x4f4af5d1, 0xc2884e6e, 0xa5a92114, 0x2a650525, 0xea0208a8, 0xdf18f0ce, 0x4ec05432, 0xc2517e31,
    0xa58480e6, 0x2c216eaa, 0xe7dc2383, 0xe0745b24, 0x4e32a956, 0xc21d0eb8, 0xa57f450a, 0x2dce88aa,
    0xe5b9f755, 0xe1d4a2c8, 0x4da1fab5, 0xc1eb0209, 0xa5996f52, 0x2f6bbe45, 0xe39bd857, 0xe3399167,
    0x4d0e4de2, 0xc1bb5a11, 0xa5d2f6a9, 0x30f8801f, 0xe1821a21, 0xe4a2eff6, 0x4c77a88e, 0xc18e18a7,
    0xa62bc71b, 0x32744493, 0xdf6d0fa1, 0xe61086bc, 0x4bde1089, 0xc1633f8a, 0xa6a3c1d6, 0x33de87de,
    0xdd5d0b08, 0xe7821d59, 0x4b418bbe, 0xc13ad060, 0xa73abd3b, 0x3536cc52, 0xdb525dc3, 0xe8f77acf,
    0x4aa22036, 0xc114ccb9, 0xa7f084e7, 0x367c9a7e, 0xd94d586c, 0xea70658a, 0x49ffd417, 0xc0f1360b,
    0xa8c4d9cb, 0x37af8159, 0xd74e4abc, 0xebeca36c, 0x495aada2, 0xc0d00db6, 0xa9b7723b, 0x38cf1669,
    0xd5558381, 0xed6bf9d1, 0x48b2b335, 0xc0b15502, 0xaac7fa0e, 0x39daf5e8, 0xd3635094, 0xeeee2d9d,
    0x4807eb4b, 0xc0950d1d, 0xabf612b5, 0x3ad2c2e8, 0xd177fec6, 0xf0730342, 0x475a5c77, 0xc07b371e,
    0xad415361, 0x3bb6276e, 0xcf93d9dc, 0xf1fa3ecb, 0x46aa0d6d, 0xc063d405, 0xaea94927, 0x3c84d496,
    0xcdb72c7e, 0xf383a3e2, 0x45f704f7, 0xc04ee4b8, 0xb02d7724, 0x3d3e82ae, 0xcbe2402d, 0xf50ef5de,
    0x454149fc, 0xc03c6a07, 0xb1cd56aa, 0x3de2f148, 0xca155d39, 0xf69bf7c9, 0x4488e37f, 0xc02c64a6,
    0xb3885772, 0x3e71e759, 0xc850cab4, 0xf82a6c6a, 0x43cdd89a, 0xc01ed535, 0xb55ddfca, 0x3eeb3347,
    0xc694ce67, 0xf9ba1651, 0x43103085, 0xc013bc39, 0xb74d4ccb, 0x3f4eaafe, 0xc4e1accb, 0xfb4ab7db,
    0x424ff28f, 0xc00b1a20, 0xb955f293, 0x3f9c2bfb, 0xc337a8f7, 0xfcdc1342, 0x418d2621, 0xc004ef3f,
    0xbb771c81, 0x3fd39b5a, 0xc197049e, 0xfe6deaa1, 0x40c7d2bd, 0xc0013bd3, 0xbdb00d71, 0x3ff4e5e0,
};

const uint32_t twidTabEven[4*6 + 16*6 + 64*6] PROGMEM = {
    0x40000000, 0x00000000, 0x40000000, 0x00000000, 0x40000000, 0x00000000, 0x5a82799a, 0xd2bec333,
    0x539eba45, 0xe7821d59, 0x539eba45, 0xc4df2862, 0x40000000, 0xc0000000, 0x5a82799a, 0xd2bec333,
    0x00000000, 0xd2bec333, 0x00000000, 0xd2bec333, 0x539eba45, 0xc4df2862, 0xac6145bb, 0x187de2a7,
    0x40000000, 0x00000000, 0x40000000, 0x00000000, 0x40000000, 0x00000000, 0x4b418bbe, 0xf383a3e2,
    0x45f704f7, 0xf9ba1651, 0x4fd288dc, 0xed6bf9d1, 0x539eba45, 0xe7821d59, 0x4b418bbe, 0xf383a3e2,
    0x58c542c5, 0xdc71898d, 0x58c542c5, 0xdc71898d, 0x4fd288dc, 0xed6bf9d1, 0x5a12e720, 0xce86ff2a,
    0x5a82799a, 0xd2bec333, 0x539eba45, 0xe7821d59, 0x539eba45, 0xc4df2862, 0x58c542c5, 0xcac933ae,
    0x569cc31b, 0xe1d4a2c8, 0x45f704f7, 0xc04ee4b8, 0x539eba45, 0xc4df2862, 0x58c542c5, 0xdc71898d,
    0x3248d382, 0xc13ad060, 0x4b418bbe, 0xc13ad060, 0x5a12e720, 0xd76619b6, 0x1a4608ab, 0xc78e9a1d,
    0x40000000, 0xc0000000, 0x5a82799a, 0xd2bec333, 0x00000000, 0xd2bec333, 0x3248d382, 0xc13ad060,
    0x5a12e720, 0xce86ff2a, 0xe5b9f755, 0xe1d4a2c8, 0x22a2f4f8, 0xc4df2862, 0x58c542c5, 0xcac933ae,
    0xcdb72c7e, 0xf383a3e2, 0x11a855df, 0xcac933ae, 0x569cc31b, 0xc78e9a1d, 0xba08fb09, 0x0645e9af,
    0x00000000, 0xd2bec333, 0x539eba45, 0xc4df2862, 0xac6145bb, 0x187de2a7, 0xee57aa21, 0xdc71898d,
    0x4fd288dc, 0xc2c17d52, 0xa5ed18e0, 0x2899e64a, 0xdd5d0b08, 0xe7821d59, 0x4b418bbe, 0xc13ad060,
    0xa73abd3b, 0x3536cc52, 0xcdb72c7e, 0xf383a3e2, 0x45f704f7, 0xc04ee4b8, 0xb02d7724, 0x3d3e82ae,
    0x40000000, 0x00000000, 0x40000000, 0x00000000, 0x40000000, 0x00000000, 0x43103085, 0xfcdc1342,
    0x418d2621, 0xfe6deaa1, 0x4488e37f, 0xfb4ab7db, 0x45f704f7, 0xf9ba1651, 0x43103085, 0xfcdc1342,
    0x48b2b335, 0xf69bf7c9, 0x48b2b335, 0xf69bf7c9, 0x4488e37f, 0xfb4ab7db, 0x4c77a88e, 0xf1fa3ecb,
    0x4b418bbe, 0xf383a3e2, 0x45f704f7, 0xf9ba1651, 0x4fd288dc, 0xed6bf9d1, 0x4da1fab5, 0xf0730342,
    0x475a5c77, 0xf82a6c6a, 0x52beac9f, 0xe8f77acf, 0x4fd288dc, 0xed6bf9d1, 0x48b2b335, 0xf69bf7c9,
    0x553805f2, 0xe4a2eff6, 0x51d1dc80, 0xea70658a, 0x49ffd417, 0xf50ef5de, 0x573b2635, 0xe0745b24,
    0x539eba45, 0xe7821d59, 0x4b418bbe, 0xf383a3e2, 0x58c542c5, 0xdc71898d, 0x553805f2, 0xe4a2eff6,
    0x4c77a88e, 0xf1fa3ecb, 0x59d438e5, 0xd8a00bae, 0x569cc31b, 0xe1d4a2c8, 0x4da1fab5, 0xf0730342,
    0x5a6690ae, 0xd5052d97, 0x57cc15bc, 0xdf18f0ce, 0x4ec05432, 0xeeee2d9d, 0x5a7b7f1a, 0xd1a5ef90,
    0x58c542c5, 0xdc71898d, 0x4fd288dc, 0xed6bf9d1, 0x5a12e720, 0xce86ff2a, 0x5987b08a, 0xd9e01006,
    0x50d86e6d, 0xebeca36c, 0x592d59da, 0xcbacb0bf, 0x5a12e720, 0xd76619b6, 0x51d1dc80, 0xea70658a,
    0x57cc15bc, 0xc91af976, 0x5a6690ae, 0xd5052d97, 0x52beac9f, 0xe8f77acf, 0x55f104dc, 0xc6d569be,
    0x5a82799a, 0xd2bec333, 0x539eba45, 0xe7821d59, 0x539eba45, 0xc4df2862, 0x5a6690ae, 0xd09441bb,
    0x5471e2e6, 0xe61086bc, 0x50d86e6d, 0xc33aee27, 0x5a12e720, 0xce86ff2a, 0x553805f2, 0xe4a2eff6,
    0x4da1fab5, 0xc1eb0209, 0x5987b08a, 0xcc983f70, 0x55f104dc, 0xe3399167, 0x49ffd417, 0xc0f1360b,
    0x58c542c5, 0xcac933ae, 0x569cc31b, 0xe1d4a2c8, 0x45f704f7, 0xc04ee4b8, 0x57cc15bc, 0xc91af976,
    0x573b2635, 0xe0745b24, 0x418d2621, 0xc004ef3f, 0x569cc31b, 0xc78e9a1d, 0x57cc15bc, 0xdf18f0ce,
    0x3cc85709, 0xc013bc39, 0x553805f2, 0xc6250a18, 0x584f7b58, 0xddc29958, 0x37af354c, 0xc07b371e,
    0x539eba45, 0xc4df2862, 0x58c542c5, 0xdc71898d, 0x3248d382, 0xc13ad060, 0x51d1dc80, 0xc3bdbdf6,
    0x592d59da, 0xdb25f566, 0x2c9caf6c, 0xc2517e31, 0x4fd288dc, 0xc2c17d52, 0x5987b08a, 0xd9e01006,
    0x26b2a794, 0xc3bdbdf6, 0x4da1fab5, 0xc1eb0209, 0x59d438e5, 0xd8a00bae, 0x2092f05f, 0xc57d965d,
    0x4b418bbe, 0xc13ad060, 0x5a12e720, 0xd76619b6, 0x1a4608ab, 0xc78e9a1d, 0x48b2b335, 0xc0b15502,
    0x5a43b190, 0xd6326a88, 0x13d4ae08, 0xc9edeb50, 0x45f704f7, 0xc04ee4b8, 0x5a6690ae, 0xd5052d97,
    0x0d47d096, 0xcc983f70, 0x43103085, 0xc013bc39, 0x5a7b7f1a, 0xd3de9156, 0x06a886a0, 0xcf89e3e8,
    0x40000000, 0xc0000000, 0x5a82799a, 0xd2bec333, 0x00000000, 0xd2bec333, 0x3cc85709, 0xc013bc39,
    0x5a7b7f1a, 0xd1a5ef90, 0xf9577960, 0xd6326a88, 0x396b3199, 0xc04ee4b8, 0x5a6690ae, 0xd09441bb,
    0xf2b82f6a, 0xd9e01006, 0x35eaa2c7, 0xc0b15502, 0x5a43b190, 0xcf89e3e8, 0xec2b51f8, 0xddc29958,
    0x3248d382, 0xc13ad060, 0x5a12e720, 0xce86ff2a, 0xe5b9f755, 0xe1d4a2c8, 0x2e88013a, 0xc1eb0209,
    0x59d438e5, 0xcd8bbb6d, 0xdf6d0fa1, 0xe61086bc, 0x2aaa7c7f, 0xc2c17d52, 0x5987b08a, 0xcc983f70,
    0xd94d586c, 0xea70658a, 0x26b2a794, 0xc3bdbdf6, 0x592d59da, 0xcbacb0bf, 0xd3635094, 0xeeee2d9d,
    0x22a2f4f8, 0xc4df2862, 0x58c542c5, 0xcac933ae, 0xcdb72c7e, 0xf383a3e2, 0x1e7de5df, 0xc6250a18,
    0x584f7b58, 0xc9edeb50, 0xc850cab4, 0xf82a6c6a, 0x1a4608ab, 0xc78e9a1d, 0x57cc15bc, 0xc91af976,
    0xc337a8f7, 0xfcdc1342, 0x15fdf758, 0xc91af976, 0x573b2635, 0xc8507ea7, 0xbe72d9df, 0x0192155f,
    0x11a855df, 0xcac933ae, 0x569cc31b, 0xc78e9a1d, 0xba08fb09, 0x0645e9af, 0x0d47d096, 0xcc983f70,
    0x55f104dc, 0xc6d569be, 0xb6002be9, 0x0af10a22, 0x08df1a8c, 0xce86ff2a, 0x553805f2, 0xc6250a18,
    0xb25e054b, 0x0f8cfcbe, 0x0470ebdc, 0xd09441bb, 0x5471e2e6, 0xc57d965d, 0xaf279193, 0x14135c94,
    0x00000000, 0xd2bec333, 0x539eba45, 0xc4df2862, 0xac6145bb, 0x187de2a7, 0xfb8f1424, 0xd5052d97,
    0x52beac9f, 0xc449d892, 0xaa0efb24, 0x1cc66e99, 0xf720e574, 0xd76619b6, 0x51d1dc80, 0xc3bdbdf6,
    0xa833ea44, 0x20e70f32, 0xf2b82f6a, 0xd9e01006, 0x50d86e6d, 0xc33aee27, 0xa6d2a626, 0x24da0a9a,
    0xee57aa21, 0xdc71898d, 0x4fd288dc, 0xc2c17d52, 0xa5ed18e0, 0x2899e64a, 0xea0208a8, 0xdf18f0ce,
    0x4ec05432, 0xc2517e31, 0xa58480e6, 0x2c216eaa, 0xe5b9f755, 0xe1d4a2c8, 0x4da1fab5, 0xc1eb0209,
    0xa5996f52, 0x2f6bbe45, 0xe1821a21, 0xe4a2eff6, 0x4c77a88e, 0xc18e18a7, 0xa62bc71b, 0x32744493,
    0xdd5d0b08, 0xe7821d59, 0x4b418bbe, 0xc13ad060, 0xa73abd3b, 0x3536cc52, 0xd94d586c, 0xea70658a,
    0x49ffd417, 0xc0f1360b, 0xa8c4d9cb, 0x37af8159, 0xd5558381, 0xed6bf9d1, 0x48b2b335, 0xc0b15502,
    0xaac7fa0e, 0x39daf5e8, 0xd177fec6, 0xf0730342, 0x475a5c77, 0xc07b371e, 0xad415361, 0x3bb6276e,
    0xcdb72c7e, 0xf383a3e2, 0x45f704f7, 0xc04ee4b8, 0xb02d7724, 0x3d3e82ae, 0xca155d39, 0xf69bf7c9,
    0x4488e37f, 0xc02c64a6, 0xb3885772, 0x3e71e759, 0xc694ce67, 0xf9ba1651, 0x43103085, 0xc013bc39,
    0xb74d4ccb, 0x3f4eaafe, 0xc337a8f7, 0xfcdc1342, 0x418d2621, 0xc004ef3f, 0xbb771c81, 0x3fd39b5a,
};

/* log2Tab[x] = floor(log2(x)), format = Q28 */
const int log2Tab[65] PROGMEM = {
    0x00000000, 0x00000000, 0x10000000, 0x195c01a3, 0x20000000, 0x25269e12, 0x295c01a3, 0x2ceaecfe,
    0x30000000, 0x32b80347, 0x35269e12, 0x3759d4f8, 0x395c01a3, 0x3b350047, 0x3ceaecfe, 0x3e829fb6,
    0x40000000, 0x41663f6f, 0x42b80347, 0x43f782d7, 0x45269e12, 0x4646eea2, 0x4759d4f8, 0x48608280,
    0x495c01a3, 0x4a4d3c25, 0x4b350047, 0x4c1404ea, 0x4ceaecfe, 0x4dba4a47, 0x4e829fb6, 0x4f446359,
    0x50000000, 0x50b5d69b, 0x51663f6f, 0x52118b11, 0x52b80347, 0x5359ebc5, 0x53f782d7, 0x549101ea,
    0x55269e12, 0x55b88873, 0x5646eea2, 0x56d1fafd, 0x5759d4f8, 0x57dea15a, 0x58608280, 0x58df988f,
    0x595c01a3, 0x59d5d9fd, 0x5a4d3c25, 0x5ac24113, 0x5b350047, 0x5ba58feb, 0x5c1404ea, 0x5c80730b,
    0x5ceaecfe, 0x5d53847a, 0x5dba4a47, 0x5e1f4e51, 0x5e829fb6, 0x5ee44cd5, 0x5f446359, 0x5fa2f045,
    0x60000000
};

const HuffInfo_t huffTabSpecInfo[11] PROGMEM = {
    /* table 0 not used */
    {11, {  1,  0,  0,  0,  8,  0, 24,  0, 24,  8, 16,  0,  0,  0,  0,  0,  0,  0,  0,  0},   0},
    { 9, {  0,  0,  1,  1,  7, 24, 15, 19, 14,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0},  81},
    {16, {  1,  0,  0,  4,  2,  6,  3,  5, 15, 15,  8,  9,  3,  3,  5,  2,  0,  0,  0,  0}, 162},
    {12, {  0,  0,  0, 10,  6,  0,  9, 21,  8, 14, 11,  2,  0,  0,  0,  0,  0,  0,  0,  0}, 243},
    {13, {  1,  0,  0,  4,  4,  0,  4, 12, 12, 12, 18, 10,  4,  0,  0,  0,  0,  0,  0,  0}, 324},
    {11, {  0,  0,  0,  9,  0, 16, 13,  8, 23,  8,  4,  0,  0,  0,  0,  0,  0,  0,  0,  0}, 405},
    {12, {  1,  0,  2,  1,  0,  4,  5, 10, 14, 15,  8,  4,  0,  0,  0,  0,  0,  0,  0,  0}, 486},
    {10, {  0,  0,  1,  5,  7, 10, 14, 15,  8,  4,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0}, 550},
    {15, {  1,  0,  2,  1,  0,  4,  3,  8, 11, 20, 31, 38, 32, 14,  4,  0,  0,  0,  0,  0}, 614},
    {12, {  0,  0,  0,  3,  8, 14, 17, 25, 31, 41, 22,  8,  0,  0,  0,  0,  0,  0,  0,  0}, 783},
    {12, {  0,  0,  0,  2,  6,  7, 16, 59, 55, 95, 43,  6,  0,  0,  0,  0,  0,  0,  0,  0}, 952},
};

const short huffTabSpec[1241] PROGMEM = {
    /* spectrum table 1 [81] (signed) */
    0x0000, 0x0200, 0x0e00, 0x0007, 0x0040, 0x0001, 0x0038, 0x0008, 0x01c0, 0x03c0, 0x0e40, 0x0039, 0x0078, 0x01c8, 0x000f, 0x0240,
    0x003f, 0x0fc0, 0x01f8, 0x0238, 0x0047, 0x0e08, 0x0009, 0x0208, 0x01c1, 0x0048, 0x0041, 0x0e38, 0x0201, 0x0e07, 0x0207, 0x0e01,
    0x01c7, 0x0278, 0x0e78, 0x03c8, 0x004f, 0x0079, 0x01c9, 0x01cf, 0x03f8, 0x0239, 0x007f, 0x0e48, 0x0e0f, 0x0fc8, 0x01f9, 0x03c1,
    0x03c7, 0x0e47, 0x0ff8, 0x01ff, 0x0049, 0x020f, 0x0241, 0x0e41, 0x0248, 0x0fc1, 0x0e3f, 0x0247, 0x023f, 0x0e39, 0x0fc7, 0x0e09,
    0x0209, 0x03cf, 0x0e79, 0x0e4f, 0x03f9, 0x0249, 0x0fc9, 0x027f, 0x0fcf, 0x0fff, 0x0279, 0x03c9, 0x0e49, 0x0e7f, 0x0ff9, 0x03ff,
    0x024f,
    /* spectrum table 2 [81] (signed) */
    0x0000, 0x0200, 0x0e00, 0x0001, 0x0038, 0x0007, 0x01c0, 0x0008, 0x0040, 0x01c8, 0x0e40, 0x0078, 0x000f, 0x0047, 0x0039, 0x0e07,
    0x03c0, 0x0238, 0x0fc0, 0x003f, 0x0208, 0x0201, 0x01c1, 0x0e08, 0x0041, 0x01f8, 0x0e01, 0x01c7, 0x0e38, 0x0240, 0x0048, 0x0009,
    0x0207, 0x0079, 0x0239, 0x0e78, 0x01cf, 0x03c8, 0x0247, 0x0209, 0x0e48, 0x01f9, 0x0248, 0x0e0f, 0x0ff8, 0x0e39, 0x03f8, 0x0278,
    0x03c1, 0x0e47, 0x0fc8, 0x0e09, 0x0fc1, 0x0fc7, 0x01ff, 0x020f, 0x023f, 0x007f, 0x0049, 0x0e41, 0x0e3f, 0x004f, 0x03c7, 0x01c9,
    0x0241, 0x03cf, 0x0e79, 0x03f9, 0x0fff, 0x0e4f, 0x0e49, 0x0249, 0x0fcf, 0x03c9, 0x0e7f, 0x0fc9, 0x027f, 0x03ff, 0x0ff9, 0x0279,
    0x024f,
    /* spectrum table 3 [81] (unsigned) */
    0x0000, 0x1200, 0x1001, 0x1040, 0x1008, 0x2240, 0x2009, 0x2048, 0x2041, 0x2208, 0x3049, 0x2201, 0x3248, 0x4249, 0x3209, 0x3241,
    0x1400, 0x1002, 0x200a, 0x2440, 0x3288, 0x2011, 0x3051, 0x2280, 0x304a, 0x3448, 0x1010, 0x2088, 0x2050, 0x1080, 0x2042, 0x2408,
    0x4289, 0x3089, 0x3250, 0x4251, 0x3281, 0x2210, 0x3211, 0x2081, 0x4449, 0x424a, 0x3441, 0x320a, 0x2012, 0x3052, 0x3488, 0x3290,
    0x2202, 0x2401, 0x3091, 0x2480, 0x4291, 0x3242, 0x3409, 0x4252, 0x4489, 0x2090, 0x308a, 0x3212, 0x3481, 0x3450, 0x3490, 0x3092,
    0x4491, 0x4451, 0x428a, 0x4292, 0x2082, 0x2410, 0x3282, 0x3411, 0x444a, 0x3442, 0x4492, 0x448a, 0x4452, 0x340a, 0x2402, 0x3482,
    0x3412,
    /* spectrum table 4 [81] (unsigned) */
    0x4249, 0x3049, 0x3241, 0x3248, 0x3209, 0x1200, 0x2240, 0x0000, 0x2009, 0x2208, 0x2201, 0x2048, 0x1001, 0x2041, 0x1008, 0x1040,
    0x4449, 0x4251, 0x4289, 0x424a, 0x3448, 0x3441, 0x3288, 0x3409, 0x3051, 0x304a, 0x3250, 0x3089, 0x320a, 0x3281, 0x3242, 0x3211,
    0x2440, 0x2408, 0x2280, 0x2401, 0x2042, 0x2088, 0x200a, 0x2050, 0x2081, 0x2202, 0x2011, 0x2210, 0x1400, 0x1002, 0x1080, 0x1010,
    0x4291, 0x4489, 0x4451, 0x4252, 0x428a, 0x444a, 0x3290, 0x3488, 0x3450, 0x3091, 0x3052, 0x3481, 0x308a, 0x3411, 0x3212, 0x4491,
    0x3282, 0x340a, 0x3442, 0x4292, 0x4452, 0x448a, 0x2090, 0x2480, 0x2012, 0x2410, 0x2082, 0x2402, 0x4492, 0x3092, 0x3490, 0x3482,
    0x3412,
    /* spectrum table 5 [81] (signed) */
    0x0000, 0x03e0, 0x0020, 0x0001, 0x001f, 0x003f, 0x03e1, 0x03ff, 0x0021, 0x03c0, 0x0002, 0x0040, 0x001e, 0x03df, 0x0041, 0x03fe,
    0x0022, 0x03c1, 0x005f, 0x03e2, 0x003e, 0x03a0, 0x0060, 0x001d, 0x0003, 0x03bf, 0x0023, 0x0061, 0x03fd, 0x03a1, 0x007f, 0x003d,
    0x03e3, 0x03c2, 0x0042, 0x03de, 0x005e, 0x03be, 0x007e, 0x03c3, 0x005d, 0x0062, 0x0043, 0x03a2, 0x03dd, 0x001c, 0x0380, 0x0081,
    0x0080, 0x039f, 0x0004, 0x009f, 0x03fc, 0x0024, 0x03e4, 0x0381, 0x003c, 0x007d, 0x03bd, 0x03a3, 0x03c4, 0x039e, 0x0082, 0x005c,
    0x0044, 0x0063, 0x0382, 0x03dc, 0x009e, 0x007c, 0x039d, 0x0383, 0x0064, 0x03a4, 0x0083, 0x009d, 0x03bc, 0x009c, 0x0384, 0x0084,
    0x039c,
    /* spectrum table 6 [81] (signed) */
    0x0000, 0x0020, 0x001f, 0x0001, 0x03e0, 0x0021, 0x03e1, 0x003f, 0x03ff, 0x005f, 0x0041, 0x03c1, 0x03df, 0x03c0, 0x03e2, 0x0040,
    0x003e, 0x0022, 0x001e, 0x03fe, 0x0002, 0x005e, 0x03c2, 0x03de, 0x0042, 0x03a1, 0x0061, 0x007f, 0x03e3, 0x03bf, 0x0023, 0x003d,
    0x03fd, 0x0060, 0x03a0, 0x001d, 0x0003, 0x0062, 0x03be, 0x03c3, 0x0043, 0x007e, 0x005d, 0x03dd, 0x03a2, 0x0063, 0x007d, 0x03bd,
    0x03a3, 0x003c, 0x03fc, 0x0081, 0x0381, 0x039f, 0x0024, 0x009f, 0x03e4, 0x001c, 0x0382, 0x039e, 0x0044, 0x03dc, 0x0380, 0x0082,
    0x009e, 0x03c4, 0x0080, 0x005c, 0x0004, 0x03bc, 0x03a4, 0x007c, 0x009d, 0x0064, 0x0083, 0x0383, 0x039d, 0x0084, 0x0384, 0x039c,
    0x009c,
    /* spectrum table 7 [64] (unsigned) */
    0x0000, 0x0420, 0x0401, 0x0821, 0x0841, 0x0822, 0x0440, 0x0402, 0x0861, 0x0823, 0x0842, 0x0460, 0x0403, 0x0843, 0x0862, 0x0824,
    0x0881, 0x0825, 0x08a1, 0x0863, 0x0844, 0x0404, 0x0480, 0x0882, 0x0845, 0x08a2, 0x0405, 0x08c1, 0x04a0, 0x0826, 0x0883, 0x0865,
    0x0864, 0x08a3, 0x0846, 0x08c2, 0x0827, 0x0866, 0x0406, 0x04c0, 0x0884, 0x08e1, 0x0885, 0x08e2, 0x08a4, 0x08c3, 0x0847, 0x08e3,
    0x08c4, 0x08a5, 0x0886, 0x0867, 0x04e0, 0x0407, 0x08c5, 0x08a6, 0x08e4, 0x0887, 0x08a7, 0x08e5, 0x08e6, 0x08c6, 0x08c7, 0x08e7,
    /* spectrum table 8 [64] (unsigned) */
    0x0821, 0x0841, 0x0420, 0x0822, 0x0401, 0x0842, 0x0000, 0x0440, 0x0402, 0x0861, 0x0823, 0x0862, 0x0843, 0x0863, 0x0881, 0x0824,
    0x0882, 0x0844, 0x0460, 0x0403, 0x0883, 0x0864, 0x08a2, 0x08a1, 0x0845, 0x0825, 0x08a3, 0x0865, 0x0884, 0x08a4, 0x0404, 0x0885,
    0x0480, 0x0846, 0x08c2, 0x08c1, 0x0826, 0x0866, 0x08c3, 0x08a5, 0x04a0, 0x08c4, 0x0405, 0x0886, 0x08e1, 0x08e2, 0x0847, 0x08c5,
    0x08e3, 0x0827, 0x08a6, 0x0867, 0x08c6, 0x08e4, 0x04c0, 0x0887, 0x0406, 0x08e5, 0x08e6, 0x08c7, 0x08a7, 0x04e0, 0x0407, 0x08e7,
    /* spectrum table 9 [169] (unsigned) */
    0x0000, 0x0420, 0x0401, 0x0821, 0x0841, 0x0822, 0x0440, 0x0402, 0x0861, 0x0842, 0x0823, 0x0460, 0x0403, 0x0843, 0x0862, 0x0824,
    0x0881, 0x0844, 0x0825, 0x0882, 0x0863, 0x0404, 0x0480, 0x08a1, 0x0845, 0x0826, 0x0864, 0x08a2, 0x08c1, 0x0883, 0x0405, 0x0846,
    0x04a0, 0x0827, 0x0865, 0x0828, 0x0901, 0x0884, 0x08a3, 0x08c2, 0x08e1, 0x0406, 0x0902, 0x0848, 0x0866, 0x0847, 0x0885, 0x0921,
    0x0829, 0x08e2, 0x04c0, 0x08a4, 0x08c3, 0x0903, 0x0407, 0x0922, 0x0868, 0x0886, 0x0867, 0x0408, 0x0941, 0x08c4, 0x0849, 0x08a5,
    0x0500, 0x04e0, 0x08e3, 0x0942, 0x0923, 0x0904, 0x082a, 0x08e4, 0x08c5, 0x08a6, 0x0888, 0x0887, 0x0869, 0x0961, 0x08a8, 0x0520,
    0x0905, 0x0943, 0x084a, 0x0409, 0x0962, 0x0924, 0x08c6, 0x0981, 0x0889, 0x0906, 0x082b, 0x0925, 0x0944, 0x08a7, 0x08e5, 0x084b,
    0x082c, 0x0982, 0x0963, 0x086a, 0x08a9, 0x08c7, 0x0907, 0x0964, 0x040a, 0x08e6, 0x0983, 0x0540, 0x0945, 0x088a, 0x08c8, 0x084c,
    0x0926, 0x0927, 0x088b, 0x0560, 0x08c9, 0x086b, 0x08aa, 0x0908, 0x08e8, 0x0985, 0x086c, 0x0965, 0x08e7, 0x0984, 0x0966, 0x0946,
    0x088c, 0x08e9, 0x08ab, 0x040b, 0x0986, 0x08ca, 0x0580, 0x0947, 0x08ac, 0x08ea, 0x0928, 0x040c, 0x0967, 0x0909, 0x0929, 0x0948,
    0x08eb, 0x0987, 0x08cb, 0x090b, 0x0968, 0x08ec, 0x08cc, 0x090a, 0x0949, 0x090c, 0x092a, 0x092b, 0x092c, 0x094b, 0x0989, 0x094a,
    0x0969, 0x0988, 0x096a, 0x098a, 0x098b, 0x094c, 0x096b, 0x096c, 0x098c,
    /* spectrum table 10 [169] (unsigned) */
    0x0821, 0x0822, 0x0841, 0x0842, 0x0420, 0x0401, 0x0823, 0x0862, 0x0861, 0x0843, 0x0863, 0x0440, 0x0402, 0x0844, 0x0882, 0x0824,
    0x0881, 0x0000, 0x0883, 0x0864, 0x0460, 0x0403, 0x0884, 0x0845, 0x08a2, 0x0825, 0x08a1, 0x08a3, 0x0865, 0x08a4, 0x0885, 0x08c2,
    0x0846, 0x08c3, 0x0480, 0x08c1, 0x0404, 0x0826, 0x0866, 0x08a5, 0x08c4, 0x0886, 0x08c5, 0x08e2, 0x0867, 0x0847, 0x08a6, 0x0902,
    0x08e3, 0x04a0, 0x08e1, 0x0405, 0x0901, 0x0827, 0x0903, 0x08e4, 0x0887, 0x0848, 0x08c6, 0x08e5, 0x0828, 0x0868, 0x0904, 0x0888,
    0x08a7, 0x0905, 0x08a8, 0x08e6, 0x08c7, 0x0922, 0x04c0, 0x08c8, 0x0923, 0x0869, 0x0921, 0x0849, 0x0406, 0x0906, 0x0924, 0x0889,
    0x0942, 0x0829, 0x08e7, 0x0907, 0x0925, 0x08e8, 0x0943, 0x08a9, 0x0944, 0x084a, 0x0941, 0x086a, 0x0926, 0x08c9, 0x0500, 0x088a,
    0x04e0, 0x0962, 0x08e9, 0x0963, 0x0946, 0x082a, 0x0961, 0x0927, 0x0407, 0x0908, 0x0945, 0x086b, 0x08aa, 0x0909, 0x0965, 0x0408,
    0x0964, 0x084b, 0x08ea, 0x08ca, 0x0947, 0x088b, 0x082b, 0x0982, 0x0928, 0x0983, 0x0966, 0x08ab, 0x0984, 0x0967, 0x0985, 0x086c,
    0x08cb, 0x0520, 0x0948, 0x0540, 0x0981, 0x0409, 0x088c, 0x0929, 0x0986, 0x084c, 0x090a, 0x092a, 0x082c, 0x0968, 0x0987, 0x08eb,
    0x08ac, 0x08cc, 0x0949, 0x090b, 0x0988, 0x040a, 0x08ec, 0x0560, 0x094a, 0x0969, 0x096a, 0x040b, 0x096b, 0x092b, 0x094b, 0x0580,
    0x090c, 0x0989, 0x094c, 0x092c, 0x096c, 0x098b, 0x040c, 0x098a, 0x098c,
    /* spectrum table 11 [289] (unsigned) */
    0x0000, 0x2041, 0x2410, 0x1040, 0x1001, 0x2081, 0x2042, 0x2082, 0x2043, 0x20c1, 0x20c2, 0x1080, 0x2083, 0x1002, 0x20c3, 0x2101,
    0x2044, 0x2102, 0x2084, 0x2103, 0x20c4, 0x10c0, 0x1003, 0x2141, 0x2142, 0x2085, 0x2104, 0x2045, 0x2143, 0x20c5, 0x2144, 0x2105,
    0x2182, 0x2086, 0x2181, 0x2183, 0x20c6, 0x2046, 0x2110, 0x20d0, 0x2405, 0x2403, 0x2404, 0x2184, 0x2406, 0x1100, 0x2106, 0x1004,
    0x2090, 0x2145, 0x2150, 0x2407, 0x2402, 0x2408, 0x2087, 0x21c2, 0x20c7, 0x2185, 0x2146, 0x2190, 0x240a, 0x21c3, 0x21c1, 0x2409,
    0x21d0, 0x2050, 0x2047, 0x2107, 0x240b, 0x21c4, 0x240c, 0x2210, 0x2401, 0x2186, 0x2250, 0x2088, 0x2147, 0x2290, 0x240d, 0x2203,
    0x2202, 0x20c8, 0x1140, 0x240e, 0x22d0, 0x21c5, 0x2108, 0x2187, 0x21c6, 0x1005, 0x2204, 0x240f, 0x2310, 0x2048, 0x2201, 0x2390,
    0x2148, 0x2350, 0x20c9, 0x2205, 0x21c7, 0x2089, 0x2206, 0x2242, 0x2243, 0x23d0, 0x2109, 0x2188, 0x1180, 0x2244, 0x2149, 0x2207,
    0x21c8, 0x2049, 0x2283, 0x1006, 0x2282, 0x2241, 0x2245, 0x210a, 0x208a, 0x2246, 0x20ca, 0x2189, 0x2284, 0x2208, 0x2285, 0x2247,
    0x22c3, 0x204a, 0x11c0, 0x2286, 0x21c9, 0x20cb, 0x214a, 0x2281, 0x210b, 0x22c2, 0x2342, 0x218a, 0x2343, 0x208b, 0x1400, 0x214b,
    0x22c5, 0x22c4, 0x2248, 0x21ca, 0x2209, 0x1010, 0x210d, 0x1007, 0x20cd, 0x22c6, 0x2341, 0x2344, 0x2303, 0x208d, 0x2345, 0x220a,
    0x218b, 0x2288, 0x2287, 0x2382, 0x2304, 0x204b, 0x210c, 0x22c1, 0x20cc, 0x204d, 0x2302, 0x21cb, 0x20ce, 0x214c, 0x214d, 0x2384,
    0x210e, 0x22c7, 0x2383, 0x2305, 0x2346, 0x2306, 0x1200, 0x22c8, 0x208c, 0x2249, 0x2385, 0x218d, 0x228a, 0x23c2, 0x220b, 0x224a,
    0x2386, 0x2289, 0x214e, 0x22c9, 0x2381, 0x208e, 0x218c, 0x204c, 0x2348, 0x1008, 0x2347, 0x21cc, 0x2307, 0x21cd, 0x23c3, 0x2301,
    0x218e, 0x208f, 0x23c5, 0x23c4, 0x204e, 0x224b, 0x210f, 0x2387, 0x220d, 0x2349, 0x220c, 0x214f, 0x20cf, 0x228b, 0x22ca, 0x2308,
    0x23c6, 0x23c7, 0x220e, 0x23c1, 0x21ce, 0x1240, 0x1009, 0x224d, 0x224c, 0x2309, 0x2388, 0x228d, 0x2389, 0x230a, 0x218f, 0x21cf,
    0x224e, 0x23c8, 0x22cb, 0x22ce, 0x204f, 0x228c, 0x228e, 0x234b, 0x234a, 0x22cd, 0x22cc, 0x220f, 0x238b, 0x234c, 0x230d, 0x23c9,
    0x238a, 0x1280, 0x230b, 0x224f, 0x100a, 0x230c, 0x12c0, 0x230e, 0x228f, 0x234d, 0x100d, 0x238c, 0x23ca, 0x23cb, 0x22cf, 0x238d,
    0x1340, 0x100b, 0x234e, 0x23cc, 0x23cd, 0x230f, 0x1380, 0x238e, 0x234f, 0x1300, 0x238f, 0x100e, 0x100c, 0x23ce, 0x13c0, 0x100f,
    0x23cf,
};

/* coefficient table 4.A.87, format = Q31
 * reordered as cTab[0], cTab[64], cTab[128], ... cTab[576], cTab[1], cTab[65], cTab[129], ... cTab[639]
 * keeping full table (not using symmetry) to allow sequential access in synth filter inner loop
 * format = Q31
 */
const uint32_t cTabS[640] PROGMEM = {
    0x00000000, 0x0055dba1, 0x01b2e41d, 0x09015651, 0x2e3a7532, 0x6d474e1d, 0xd1c58ace, 0x09015651, 0xfe4d1be3, 0x0055dba1,
    0xffede50e, 0x005b5371, 0x01d78bfc, 0x08d3e41b, 0x2faa221c, 0x6d41d963, 0xd3337b3d, 0x09299ead, 0xfe70b8d1, 0x0050b177,
    0xffed978a, 0x006090c4, 0x01fd3ba0, 0x08a24899, 0x311af3a4, 0x6d32730f, 0xd49fd55f, 0x094d7ec2, 0xfe933dc0, 0x004b6c46,
    0xffefc9b9, 0x0065fde5, 0x02244a24, 0x086b1eeb, 0x328cc6f0, 0x6d18520e, 0xd60a46e5, 0x096d0e21, 0xfeb48d0d, 0x00465348,
    0xfff0065d, 0x006b47fa, 0x024bf7a1, 0x082f552e, 0x33ff670e, 0x6cf4073e, 0xd7722f04, 0x09881dc5, 0xfed4bec3, 0x004103f4,
    0xffeff6ca, 0x0070c8a5, 0x0274ba43, 0x07ee507c, 0x3572ec70, 0x6cc59bab, 0xd8d7f21f, 0x099ec3dc, 0xfef3f6ab, 0x003c1fa4,
    0xffef7b8b, 0x0075fded, 0x029e35b4, 0x07a8127d, 0x36e69691, 0x6c8c4c7a, 0xda3b176a, 0x09b18a1d, 0xff120d70, 0x003745f9,
    0xffeedfa4, 0x007b3875, 0x02c89901, 0x075ca90c, 0x385a49c4, 0x6c492217, 0xdb9b5b12, 0x09c018ce, 0xff2ef725, 0x00329ab6,
    0xffee1650, 0x00807994, 0x02f3e48d, 0x070bbf58, 0x39ce0477, 0x6bfbdd98, 0xdcf898fb, 0x09caeb0f, 0xff4aabc8, 0x002d8e42,
    0xffed651d, 0x0085c217, 0x03201116, 0x06b559c3, 0x3b415115, 0x6ba4629f, 0xde529086, 0x09d1fa23, 0xff6542d1, 0x00293718,
    0xffecc31b, 0x008a7dd7, 0x034d01f0, 0x06593912, 0x3cb41219, 0x6b42a864, 0xdfa93ab5, 0x09d5560b, 0xff7ee3f1, 0x0024dd50,
    0xffebe77b, 0x008f4bfc, 0x037ad438, 0x05f7fb90, 0x3e25b17e, 0x6ad73e8d, 0xe0fc421e, 0x09d52709, 0xff975c01, 0x002064f8,
    0xffeb50b2, 0x009424c6, 0x03a966bb, 0x0590a67d, 0x3f962fb8, 0x6a619c5e, 0xe24b8f66, 0x09d19ca9, 0xffaea5d6, 0x001c3549,
    0xffea9192, 0x0098b855, 0x03d8afe6, 0x05237f9d, 0x41058bc6, 0x69e29784, 0xe396a45d, 0x09cab9f2, 0xffc4e365, 0x0018703f,
    0xffe9ca76, 0x009d10bf, 0x04083fec, 0x04b0adcb, 0x4272a385, 0x6959709d, 0xe4de0cb0, 0x09c0e59f, 0xffda17f2, 0x001471f8,
    0xffe940f4, 0x00a1039c, 0x043889c6, 0x0437fb0a, 0x43de620a, 0x68c7269b, 0xe620c476, 0x09b3d77f, 0xffee183b, 0x0010bc63,
    0xffe88ba8, 0x00a520bb, 0x04694101, 0x03b8f8dc, 0x4547daea, 0x682b39a4, 0xe75f8bb8, 0x09a3e163, 0x0000e790, 0x000d31b5,
    0xffe83a07, 0x00a8739d, 0x049aa82f, 0x03343533, 0x46aea856, 0x6785c24d, 0xe89971b7, 0x099140a7, 0x00131c75, 0x0009aa3f,
    0xffe79e16, 0x00abe79e, 0x04cc2fcf, 0x02a99097, 0x4812f848, 0x66d76725, 0xe9cea84a, 0x097c1ee8, 0x0023b989, 0x0006b1cf,
    0xffe7746e, 0x00af374c, 0x04fe20be, 0x02186a91, 0x4973fef1, 0x661fd6b8, 0xeafee7f1, 0x0963ed46, 0x0033b927, 0x00039609,
    0xffe6d466, 0x00b1978d, 0x05303f87, 0x01816e06, 0x4ad237a2, 0x655f63f2, 0xec2a3f5f, 0x0949eaac, 0x00426f36, 0x00007134,
    0xffe6afee, 0x00b3d15c, 0x05626209, 0x00e42fa2, 0x4c2ca3df, 0x64964063, 0xed50a31d, 0x092d7970, 0x00504f41, 0xfffdfa25,
    0xffe65416, 0x00b5c867, 0x05950122, 0x0040c496, 0x4d83976c, 0x63c45243, 0xee71b2fe, 0x090ec1fc, 0x005d36df, 0xfffb42b0,
    0xffe681c6, 0x00b74c37, 0x05c76fed, 0xff96db90, 0x4ed62be3, 0x62ea6474, 0xef8d4d7b, 0x08edfeaa, 0x006928a0, 0xfff91fca,
    0xffe66dd0, 0x00b8394b, 0x05f9c051, 0xfee723c6, 0x5024d70e, 0x6207f220, 0xf0a3959f, 0x08cb4e23, 0x007400b8, 0xfff681d6,
    0xffe66fac, 0x00b8fe0d, 0x062bf5ec, 0xfe310657, 0x516eefb9, 0x611d58a3, 0xf1b461ab, 0x08a75da4, 0x007e0393, 0xfff48700,
    0xffe69423, 0x00b8c6b0, 0x065dd56a, 0xfd7475d8, 0x52b449de, 0x602b0c7f, 0xf2bf6ea4, 0x0880ffdd, 0x00872c63, 0xfff294c3,
    0xffe6fed4, 0x00b85f70, 0x068f8b44, 0xfcb1d740, 0x53f495aa, 0x5f30ff5f, 0xf3c4e887, 0x08594887, 0x008f87aa, 0xfff0e7ef,
    0xffe75361, 0x00b73ab0, 0x06c0f0c0, 0xfbe8f5bd, 0x552f8ff7, 0x5e2f6367, 0xf4c473c6, 0x08303897, 0x0096dcc2, 0xffef2395,
    0xffe80414, 0x00b58c8c, 0x06f1825d, 0xfb19b7bd, 0x56654bdd, 0x5d26be9b, 0xf5be0fa9, 0x08061671, 0x009da526, 0xffedc418,
    0xffe85b4b, 0x00b36acd, 0x0721bf22, 0xfa44a069, 0x579505f5, 0x5c16d0ae, 0xf6b1f3c3, 0x07da2b7f, 0x00a3508f, 0xffec8409,
    0xffe954d0, 0x00b06b68, 0x075112a2, 0xf96916f5, 0x58befacd, 0x5b001db8, 0xf79fa13a, 0x07ad8c26, 0x00a85e94, 0xffeb3849,
    0xffea353a, 0x00acbd2f, 0x077fedb3, 0xf887507c, 0x59e2f69e, 0x59e2f69e, 0xf887507c, 0x077fedb3, 0x00acbd2f, 0xffea353a,
    0xffeb3849, 0x00a85e94, 0x07ad8c26, 0xf79fa13a, 0x5b001db8, 0x58befacd, 0xf96916f5, 0x075112a2, 0x00b06b68, 0xffe954d0,
    0xffec8409, 0x00a3508f, 0x07da2b7f, 0xf6b1f3c3, 0x5c16d0ae, 0x579505f5, 0xfa44a069, 0x0721bf22, 0x00b36acd, 0xffe85b4b,
    0xffedc418, 0x009da526, 0x08061671, 0xf5be0fa9, 0x5d26be9b, 0x56654bdd, 0xfb19b7bd, 0x06f1825d, 0x00b58c8c, 0xffe80414,
    0xffef2395, 0x0096dcc2, 0x08303897, 0xf4c473c6, 0x5e2f6367, 0x552f8ff7, 0xfbe8f5bd, 0x06c0f0c0, 0x00b73ab0, 0xffe75361,
    0xfff0e7ef, 0x008f87aa, 0x08594887, 0xf3c4e887, 0x5f30ff5f, 0x53f495aa, 0xfcb1d740, 0x068f8b44, 0x00b85f70, 0xffe6fed4,
    0xfff294c3, 0x00872c63, 0x0880ffdd, 0xf2bf6ea4, 0x602b0c7f, 0x52b449de, 0xfd7475d8, 0x065dd56a, 0x00b8c6b0, 0xffe69423,
    0xfff48700, 0x007e0393, 0x08a75da4, 0xf1b461ab, 0x611d58a3, 0x516eefb9, 0xfe310657, 0x062bf5ec, 0x00b8fe0d, 0xffe66fac,
    0xfff681d6, 0x007400b8, 0x08cb4e23, 0xf0a3959f, 0x6207f220, 0x5024d70e, 0xfee723c6, 0x05f9c051, 0x00b8394b, 0xffe66dd0,
    0xfff91fca, 0x006928a0, 0x08edfeaa, 0xef8d4d7b, 0x62ea6474, 0x4ed62be3, 0xff96db90, 0x05c76fed, 0x00b74c37, 0xffe681c6,
    0xfffb42b0, 0x005d36df, 0x090ec1fc, 0xee71b2fe, 0x63c45243, 0x4d83976c, 0x0040c496, 0x05950122, 0x00b5c867, 0xffe65416,
    0xfffdfa25, 0x00504f41, 0x092d7970, 0xed50a31d, 0x64964063, 0x4c2ca3df, 0x00e42fa2, 0x05626209, 0x00b3d15c, 0xffe6afee,
    0x00007134, 0x00426f36, 0x0949eaac, 0xec2a3f5f, 0x655f63f2, 0x4ad237a2, 0x01816e06, 0x05303f87, 0x00b1978d, 0xffe6d466,
    0x00039609, 0x0033b927, 0x0963ed46, 0xeafee7f1, 0x661fd6b8, 0x4973fef1, 0x02186a91, 0x04fe20be, 0x00af374c, 0xffe7746e,
    0x0006b1cf, 0x0023b989, 0x097c1ee8, 0xe9cea84a, 0x66d76725, 0x4812f848, 0x02a99097, 0x04cc2fcf, 0x00abe79e, 0xffe79e16,
    0x0009aa3f, 0x00131c75, 0x099140a7, 0xe89971b7, 0x6785c24d, 0x46aea856, 0x03343533, 0x049aa82f, 0x00a8739d, 0xffe83a07,
    0x000d31b5, 0x0000e790, 0x09a3e163, 0xe75f8bb8, 0x682b39a4, 0x4547daea, 0x03b8f8dc, 0x04694101, 0x00a520bb, 0xffe88ba8,
    0x0010bc63, 0xffee183b, 0x09b3d77f, 0xe620c476, 0x68c7269b, 0x43de620a, 0x0437fb0a, 0x043889c6, 0x00a1039c, 0xffe940f4,
    0x001471f8, 0xffda17f2, 0x09c0e59f, 0xe4de0cb0, 0x6959709d, 0x4272a385, 0x04b0adcb, 0x04083fec, 0x009d10bf, 0xffe9ca76,
    0x0018703f, 0xffc4e365, 0x09cab9f2, 0xe396a45d, 0x69e29784, 0x41058bc6, 0x05237f9d, 0x03d8afe6, 0x0098b855, 0xffea9192,
    0x001c3549, 0xffaea5d6, 0x09d19ca9, 0xe24b8f66, 0x6a619c5e, 0x3f962fb8, 0x0590a67d, 0x03a966bb, 0x009424c6, 0xffeb50b2,
    0x002064f8, 0xff975c01, 0x09d52709, 0xe0fc421e, 0x6ad73e8d, 0x3e25b17e, 0x05f7fb90, 0x037ad438, 0x008f4bfc, 0xffebe77b,
    0x0024dd50, 0xff7ee3f1, 0x09d5560b, 0xdfa93ab5, 0x6b42a864, 0x3cb41219, 0x06593912, 0x034d01f0, 0x008a7dd7, 0xffecc31b,
    0x00293718, 0xff6542d1, 0x09d1fa23, 0xde529086, 0x6ba4629f, 0x3b415115, 0x06b559c3, 0x03201116, 0x0085c217, 0xffed651d,
    0x002d8e42, 0xff4aabc8, 0x09caeb0f, 0xdcf898fb, 0x6bfbdd98, 0x39ce0477, 0x070bbf58, 0x02f3e48d, 0x00807994, 0xffee1650,
    0x00329ab6, 0xff2ef725, 0x09c018ce, 0xdb9b5b12, 0x6c492217, 0x385a49c4, 0x075ca90c, 0x02c89901, 0x007b3875, 0xffeedfa4,
    0x003745f9, 0xff120d70, 0x09b18a1d, 0xda3b176a, 0x6c8c4c7a, 0x36e69691, 0x07a8127d, 0x029e35b4, 0x0075fded, 0xffef7b8b,
    0x003c1fa4, 0xfef3f6ab, 0x099ec3dc, 0xd8d7f21f, 0x6cc59bab, 0x3572ec70, 0x07ee507c, 0x0274ba43, 0x0070c8a5, 0xffeff6ca,
    0x004103f4, 0xfed4bec3, 0x09881dc5, 0xd7722f04, 0x6cf4073e, 0x33ff670e, 0x082f552e, 0x024bf7a1, 0x006b47fa, 0xfff0065d,
    0x00465348, 0xfeb48d0d, 0x096d0e21, 0xd60a46e5, 0x6d18520e, 0x328cc6f0, 0x086b1eeb, 0x02244a24, 0x0065fde5, 0xffefc9b9,
    0x004b6c46, 0xfe933dc0, 0x094d7ec2, 0xd49fd55f, 0x6d32730f, 0x311af3a4, 0x08a24899, 0x01fd3ba0, 0x006090c4, 0xffed978a,
    0x0050b177, 0xfe70b8d1, 0x09299ead, 0xd3337b3d, 0x6d41d963, 0x2faa221c, 0x08d3e41b, 0x01d78bfc, 0x005b5371, 0xffede50f,
};

const HuffInfo_t huffTabScaleFactInfo PROGMEM =
    {19, { 1,  0,  1,  3,  2,  4,  3,  5,  4,  6,  6,  6,  5,  8,  4,  7,  3,  7, 46,  0},   0};

/* note - includes offset of -60 (4.6.2.3 in spec) */
const short huffTabScaleFact[121] PROGMEM = { /* scale factor table [121] */
       0,   -1,    1,   -2,    2,   -3,    3,   -4,    4,   -5,    5,    6,   -6,    7,   -7,    8,
      -8,    9,   -9,   10,  -10,  -11,   11,   12,  -12,   13,  -13,   14,  -14,   16,   15,   17,
      18,  -15,  -17,  -16,   19,  -18,  -19,   20,  -20,   21,  -21,   22,  -22,   23,  -23,  -25,
      25,  -27,  -24,  -26,   24,  -28,   27,   29,  -30,  -29,   26,  -31,  -34,  -33,  -32,  -36,
      28,  -35,  -38,  -37,   30,  -39,  -41,  -57,  -59,  -58,  -60,   38,   39,   40,   41,   42,
      57,   37,   31,   32,   33,   34,   35,   36,   44,   51,   52,   53,   54,   55,   56,   50,
      45,   46,   47,   48,   49,   58,  -54,  -52,  -51,  -50,  -55,   43,   60,   59,  -56,  -53,
     -45,  -44,  -42,  -40,  -43,  -49,  -48,  -46,  -47,
};

/* noise table 4.A.88, format = Q31 */
const uint32_t noiseTab[512*2] PROGMEM = {
    0x8010fd38, 0xb3dc7948, 0x7c4e2301, 0xa9904192, 0x121622a7, 0x86489625, 0xc3d53d25, 0xd0343fa9,
    0x674d6f70, 0x25f4e9fd, 0xce1a8c8b, 0x72a726c5, 0xfea6efc6, 0xaa4adb1a, 0x8b2dd628, 0xf14029e4,
    0x46321c1a, 0x604889a0, 0x33363b63, 0x815ed069, 0x802b4315, 0x8f2bf7f3, 0x85b86073, 0x745cfb46,
    0xc57886b3, 0xb76731f0, 0xa2a66772, 0x828ca631, 0x60cc145e, 0x1ad1010f, 0x090c83d4, 0x9bd7ba87,
    0x5f5aeea2, 0x8b4dbd99, 0x848e7b1e, 0x86bb9fa2, 0x26f18ae5, 0xc0b81194, 0x553407bf, 0x52c17953,
    0x755f468d, 0x166b04f8, 0xa5687981, 0x4343248b, 0xa6558d5e, 0xc5f6fab7, 0x80a4fb8c, 0x8cb53cb7,
    0x7da68a54, 0x9cd8df8a, 0xba05376c, 0xfcb58ee2, 0xfdd657a4, 0x005e35ca, 0x91c75c55, 0x367651e6,
    0x816abf85, 0x8f831c4f, 0x423f9c9c, 0x55aa919e, 0x80779834, 0xb59f4244, 0x800a095c, 0x7de9e0cc,
    0x46bda5cb, 0x4c184464, 0x2c438f71, 0x797216b5, 0x5035cee6, 0xa0c3a26e, 0x9d3f95fa, 0xd4a100c0,
    0x8ac30dac, 0x04b87397, 0x9e5ac516, 0x8b0b442e, 0x66210ad6, 0x88ba7598, 0x45b9bd33, 0xf0be5087,
    0x9261b85e, 0x364f6a31, 0x891c4b50, 0x23ad08ce, 0xf10366a6, 0x80414276, 0x1b562e06, 0x8be21591,
    0x9e798195, 0x7fb4045c, 0x7d9506cf, 0x854e691f, 0x9207f092, 0x7a94c9d5, 0x88911536, 0x3f45cc61,
    0x27059279, 0xa5b57109, 0x6d2bb67b, 0x3bdc5379, 0x74e662d8, 0x80348f8c, 0xf875e638, 0x5a8caea1,
    0x2459ae75, 0x2c54b939, 0x79ee3203, 0xb9bc8683, 0x9b6f630c, 0x9f45b351, 0x8563b2b9, 0xe5dbba41,
    0x697c7d0d, 0x7bb7c90e, 0xac900866, 0x8e6b5177, 0x8822dd37, 0x7fd5a91e, 0x7506da05, 0x82302aca,
    0xa5e4be04, 0x4b4288eb, 0x00b8bc9f, 0x4f1033e4, 0x7200d612, 0x43900c8c, 0xa815b900, 0x676ed1d4,
    0x5c5f23b2, 0xa758ee11, 0xaf73abfa, 0x11714ec0, 0x265239e0, 0xc50de679, 0x8a84e341, 0xa1438354,
    0x7f1a341f, 0x343ec96b, 0x696e71b0, 0xa13bde39, 0x81e75094, 0x80091111, 0x853a73bf, 0x80f9c1ee,
    0xe4980086, 0x886a8e28, 0xa7e89426, 0xdd93edd7, 0x7592100d, 0x0bfa8123, 0x850a26d4, 0x2e34f395,
    0x421b6c00, 0xa4a462e4, 0x4e3f5090, 0x3c189f4c, 0x3c971a56, 0xdd0376d2, 0x747a5367, 0x7bcbc9d7,
    0x3966be6a, 0x7efda616, 0x55445e15, 0x7ba2ab3f, 0x5fe684f2, 0x8cf42af9, 0x808c61c3, 0x4390c27b,
    0x7cac62ff, 0xea6cab22, 0x5d0902ad, 0xc27b7208, 0x7a27389d, 0x5820a357, 0xa29bbe59, 0x9df0f1fd,
    0x92bd67e5, 0x7195b587, 0x97cac65b, 0x8339807e, 0x8f72d832, 0x5fad8685, 0xa462d9d3, 0x81d46214,
    0x6ae93e1d, 0x6b23a5b9, 0xc2732874, 0x81795268, 0x7c568cb6, 0x668513ea, 0x428d024e, 0x66b78b3a,
    0xfee9ef03, 0x9ddcbb82, 0xa605f07e, 0x46dc55e0, 0x85415054, 0xc89ec271, 0x7c42edfb, 0x0befe59b,
    0x89b8f607, 0x6d732a1a, 0xa7081ebd, 0x7e403258, 0x21feeb7b, 0x5dd7a1e7, 0x23e3a31a, 0x129bc896,
    0xa11a6b54, 0x7f1e031c, 0xfdc1a4d1, 0x96402e53, 0xb9700f1a, 0x8168ecd6, 0x7d63d3cc, 0x87a70d65,
    0x81075a7a, 0x55c8caa7, 0xa95d00b5, 0x102b1652, 0x0bb30215, 0xe5b63237, 0xa446ca44, 0x82d4c333,
    0x67b2e094, 0x44c3d661, 0x33fd6036, 0xde1ea2a1, 0xa95e8e47, 0x78f66eb9, 0x6f2aef1e, 0xe8887247,
    0x80a3b70e, 0xfca0d9d3, 0x6bf0fd20, 0x0d5226de, 0xf4341c87, 0x5902df05, 0x7ff1a38d, 0xf02e5a5b,
    0x99f129af, 0x8ac63d01, 0x7b53f599, 0x7bb32532, 0x99ac59b0, 0x5255a80f, 0xf1320a41, 0x2497aa5c,
    0xcce60bd8, 0x787c634b, 0x7ed58c5b, 0x8a28eb3a, 0x24a5e647, 0x8b79a2c1, 0x955f5ce5, 0xa9d12bc4,
    0x7a1e20c6, 0x3eeda7ac, 0xf7be823a, 0x042924ce, 0x808b3f03, 0x364248da, 0xac2895e5, 0x69a8b5fa,
    0x97fe8b63, 0xbdeac9aa, 0x8073e0ad, 0x6c25dba7, 0x005e51d2, 0x52e74389, 0x59d3988c, 0xe5d1f39c,
    0x7b57dc91, 0x341adbe7, 0xa7d42b8d, 0x74e9f335, 0xd35bf7d8, 0x5b7c0a4b, 0x75bc0874, 0x552129bf,
    0x8144b70d, 0x6de93bbb, 0x5825f14b, 0x473ec5ca, 0x80a8f37c, 0xe6552d69, 0x7898360b, 0x806379b0,
    0xa9b59339, 0x3f6bf60c, 0xc367d731, 0x920ade99, 0x125592f7, 0x877e5ed1, 0xda895d95, 0x075f2ece,
    0x380e5f5e, 0x9b006b62, 0xd17a6dd2, 0x530a0e13, 0xf4cc9a14, 0x7d0a0ed4, 0x847c6e3f, 0xbaee4975,
    0x47131163, 0x64fb2cac, 0x5e2100a6, 0x7b756a42, 0xd87609f4, 0x98bfe48c, 0x0493745e, 0x836c5784,
    0x7e5ccb40, 0x3df6b476, 0x97700d28, 0x8bbd93fd, 0x56de9cdb, 0x680b4e65, 0xebc3d90e, 0x6d286793,
    0x6753712e, 0xe05c98a7, 0x3d2b6b85, 0xc4b18ddb, 0x7b59b869, 0x31435688, 0x811888e9, 0xe011ee7a,
    0x6a5844f9, 0x86ae35ea, 0xb4cbc10b, 0x01a6f5d6, 0x7a49ed64, 0x927caa49, 0x847ddaed, 0xae0d9bb6,
    0x836bdb04, 0x0fd810a6, 0x74fe126b, 0x4a346b5f, 0x80184d36, 0x5afd153c, 0x90cc8102, 0xe606d0e6,
    0xde69aa58, 0xa89f1222, 0xe06df715, 0x8fd16144, 0x0317c3e8, 0x22ce92fc, 0x690c3eca, 0x93166f02,
    0x71573414, 0x8d43cffb, 0xe8bd0bb6, 0xde86770f, 0x0bf99a41, 0x4633a661, 0xba064108, 0x7adafae3,
    0x2f6cde5d, 0xb350a52c, 0xa5ebfb0b, 0x74c57b46, 0xd3b603b5, 0x80b70892, 0xa7f7fa53, 0xd94b566c,
    0xdda3fd86, 0x6a635793, 0x3ed005ca, 0xc5f087d8, 0x31e3a746, 0x7a4278f9, 0x82def1f9, 0x06caa2b2,
    0xe9d2c349, 0x8940e7f7, 0x7feef8dd, 0x4a9b01f0, 0xacde69f8, 0x57ddc280, 0xf09e4ba4, 0xb6d9f729,
    0xb48c18f2, 0xd3654aa9, 0xca7a03c8, 0x14d57545, 0x7fda87a5, 0x0e411366, 0xb77d0df0, 0x8c2aa467,
    0x787f2590, 0x2d292db1, 0x9f12682c, 0x44ac364d, 0x1a4b31a6, 0x871f7ded, 0x7ff99167, 0x6630a1d5,
    0x25385eb9, 0x2d4dd549, 0xaf8a7004, 0x319ebe0f, 0x379ab730, 0x81dc56a4, 0x822d8523, 0x1ae8554c,
    0x18fa0786, 0x875f7de4, 0x85ca350f, 0x7de818dc, 0x7786a38f, 0xa5456355, 0x92e60f88, 0xf5526122,
    0x916039bc, 0xc561e2de, 0x31c42042, 0x7c82e290, 0x75d158b2, 0xb015bda1, 0x7220c750, 0x46565441,
    0xd0da1fdd, 0x7b777481, 0x782e73c6, 0x8cd72b7b, 0x7f1006aa, 0xfb30e51e, 0x87994818, 0x34e7c7db,
    0x7faae06b, 0xea74fbc0, 0xd20c7af4, 0xc44f396b, 0x06b4234e, 0xdf2e2a93, 0x2efb07c8, 0xce861911,
    0x7550ea05, 0xd8d90bbb, 0x58522eec, 0x746b3520, 0xce844ce9, 0x7f5cacc3, 0xda8f17e0, 0x2fedf9cb,
    0xb2f77ec4, 0x6f13f4c0, 0x834de085, 0x7b7ace4b, 0x713b16ac, 0x499c5ab0, 0x06a7961d, 0x1b39a48a,
    0xbb853e6e, 0x7c781cc1, 0xc0baebf5, 0x7dace394, 0x815ceebc, 0xcc7b27d4, 0x8274b181, 0xa2be40a2,
    0xdd01d5dc, 0x7fefeb14, 0x0813ec78, 0xba3077cc, 0xe5cf1e1c, 0xedcfacae, 0x54c43a9b, 0x5cd62a42,
    0x93806b55, 0x03095c5b, 0x8e076ae3, 0x71bfcd2a, 0x7ac1989b, 0x623bc71a, 0x5e15d4d2, 0xfb341dd1,
    0xd75dfbca, 0xd0da32be, 0xd4569063, 0x337869da, 0x3d30606a, 0xcd89cca2, 0x7dd2ae36, 0x028c03cd,
    0xd85e052c, 0xe8dc9ec5, 0x7ffd9241, 0xde5bf4c6, 0x88c4b235, 0x8228be2e, 0x7fe6ec64, 0x996abe6a,
    0xdeb0666d, 0x9eb86611, 0xd249b922, 0x18b3e26b, 0x80211168, 0x5f8bb99c, 0x6ecb0dd2, 0x4728ff8d,
    0x2ac325b8, 0x6e5169d2, 0x7ebbd68d, 0x05e41d17, 0xaaa19f28, 0x8ab238a6, 0x51f105be, 0x140809cc,
    0x7f7345d9, 0x3aae5a9d, 0xaecec6e4, 0x1afb3473, 0xf6229ed1, 0x8d55f467, 0x7e32003a, 0x70f30c14,
    0x6686f33f, 0xd0d45ed8, 0x644fab57, 0x3a3fbbd3, 0x0b255fc4, 0x679a1701, 0x90e17b6e, 0x325d537b,
    0xcd7b9b87, 0xaa7be2a2, 0x7d47c966, 0xa33dbce5, 0x8659c3bb, 0x72a41367, 0x15c446e0, 0x45fe8b0a,
    0x9d8ddf26, 0x84d47643, 0x7fabe0da, 0x36a70122, 0x7a28ebfe, 0x7c29b8b8, 0x7f760406, 0xbabe4672,
    0x23ea216e, 0x92bcc50a, 0x6d20dba2, 0xad5a7c7e, 0xbf3897f5, 0xabb793e1, 0x8391fc7e, 0xe270291c,
    0x7a248d58, 0x80f8fd15, 0x83ef19f3, 0x5e6ece7d, 0x278430c1, 0x35239f4d, 0xe09c073b, 0x50e78cb5,
    0xd4b811bd, 0xce834ee0, 0xf88aaa34, 0xf71da5a9, 0xe2b0a1d5, 0x7c3aef31, 0xe84eabca, 0x3ce25964,
    0xf29336d3, 0x8fa78b2c, 0xa3fc3415, 0x63e1313d, 0x7fbc74e0, 0x7340bc93, 0x49ae583b, 0x8b79de4b,
    0x25011ce9, 0x7b462279, 0x36007db0, 0x3da1599c, 0x77780772, 0xc845c9bb, 0x83ba68be, 0x6ee507d1,
    0x2f0159b8, 0x5392c4ed, 0x98336ff6, 0x0b3c7f11, 0xde697aac, 0x893fc8d0, 0x6b83f8f3, 0x47799a0d,
    0x801d9dfc, 0x8516a83e, 0x5f8d22ec, 0x0f8ba384, 0xa049dc4b, 0xdd920b05, 0x7a99bc9f, 0x9ad19344,
    0x7a345dba, 0xf501a13f, 0x3e58bf19, 0x7fffaf9a, 0x3b4e1511, 0x0e08b991, 0x9e157620, 0x7230a326,
    0x4977f9ff, 0x2d2bbae1, 0x607aa7fc, 0x7bc85d5f, 0xb441bbbe, 0x8d8fa5f2, 0x601cce26, 0xda1884f2,
    0x81c82d64, 0x200b709c, 0xcbd36abe, 0x8cbdddd3, 0x55ab61d3, 0x7e3ee993, 0x833f18aa, 0xffc1aaea,
    0x7362e16a, 0x7fb85db2, 0x904ee04c, 0x7f04dca6, 0x8ad7a046, 0xebe7d8f7, 0xfbc4c687, 0xd0609458,
    0x093ed977, 0x8e546085, 0x7f5b8236, 0x7c47e118, 0xa01f2641, 0x7ffb3e48, 0x05de7cda, 0x7fc281b9,
    0x8e0278fc, 0xd74e6d07, 0x94c24450, 0x7cf9e641, 0x2ad27871, 0x919fa815, 0x805fd205, 0x7758397f,
    0xe2c7e02c, 0x1828e194, 0x5613d6fe, 0xfb55359f, 0xf9699516, 0x8978ee26, 0x7feebad9, 0x77d71d82,
    0x55b28b60, 0x7e997600, 0x80821a6b, 0xc6d78af1, 0x691822ab, 0x7f6982a0, 0x7ef56f99, 0x5c307f40,
    0xac6f8b76, 0x42cc8ba4, 0x782c61d9, 0xa0224dd0, 0x7bd234d1, 0x74576e3b, 0xe38cfe9a, 0x491e66ef,
    0xc78291c5, 0x895bb87f, 0x924f7889, 0x71b89394, 0x757b779d, 0xc4a9c604, 0x5cdf7829, 0x8020e9df,
    0x805e8245, 0x4a82c398, 0x6360bd62, 0x78bb60fc, 0x09e0d014, 0x4b0ea180, 0xb841978b, 0x69a0e864,
    0x7df35977, 0x3284b0dd, 0x3cdc2efd, 0x57d31f5e, 0x541069cc, 0x1776e92e, 0x04309ea3, 0xa015eb2d,
    0xce7bfabc, 0x41b638f8, 0x8365932e, 0x846ab44c, 0xbbcc80cb, 0x8afa6cac, 0x7fc422ea, 0x4e403fc0,
    0xbfac9aee, 0x8e4c6709, 0x028e01fb, 0x6d160a9b, 0x7fe93004, 0x790f9cdc, 0x6a1f37a0, 0xf7e7ef30,
    0xb4ea0f04, 0x7bf4c8e6, 0xe981701f, 0xc258a9d3, 0x6acbbfba, 0xef5479c7, 0x079c8bd8, 0x1a410f56,
    0x6853b799, 0x86cd4f01, 0xc66e23b6, 0x34585565, 0x8d1fe00d, 0x7fcdba1a, 0x32c9717b, 0xa02f9f48,
    0xf64940db, 0x5ed7d8f1, 0x61b823b2, 0x356f8918, 0xa0a7151e, 0x793fc969, 0x530beaeb, 0x34e93270,
    0x4fc4ddb5, 0x88d58b6c, 0x36094774, 0xf620ac80, 0x03763a72, 0xf910c9a6, 0x6666fb2d, 0x752c8be8,
    0x9a6dfdd8, 0xd1a7117d, 0x51c1b1d4, 0x0a67773d, 0x43b32a79, 0x4cdcd085, 0x5f067d30, 0x05bfe92a,
    0x7ed7d203, 0xe71a3c85, 0x99127ce2, 0x8eb3cac4, 0xad4bbcea, 0x5c6a0fd0, 0x0eec04af, 0x94e95cd4,
    0x8654f921, 0x83eabb5d, 0xb058d7ca, 0x69f12d3c, 0x03d881b2, 0x80558ef7, 0x82938cb3, 0x2ec0e1d6,
    0x80044422, 0xd1e47051, 0x720fc6ff, 0x82b20316, 0x0d527b02, 0x63049a15, 0x7ad5b9ad, 0xd2a4641d,
    0x41144f86, 0x7b04917a, 0x15c4a2c0, 0x9da07916, 0x211df54a, 0x7fdd09af, 0xfe924f3f, 0x7e132cfe,
    0x9a1d18d6, 0x7c56508b, 0x80f0f0af, 0x8095ced6, 0x8037d0d7, 0x026719d1, 0xa55fec43, 0x2b1c7cb7,
    0xa5cd5ac1, 0x77639fad, 0x7fcd8b62, 0x81a18c27, 0xaee4912e, 0xeae9eebe, 0xeb3081de, 0x8532aada,
    0xc822362e, 0x86a649a9, 0x8031a71d, 0x7b319dc6, 0xea8022e6, 0x814bc5a9, 0x8f62f7a1, 0xa430ea17,
    0x388deafb, 0x883b5185, 0x776fe13c, 0x801c683f, 0x87c11b98, 0xb7cbc644, 0x8e9ad3e8, 0x3cf5a10c,
    0x7ff6a634, 0x949ef096, 0x9f84aa7c, 0x010af13f, 0x782d1de8, 0xf18e492a, 0x6cf63b01, 0x4301cd81,
    0x32d15c9e, 0x68ad8cef, 0xd09bd2d6, 0x908c5c15, 0xd1e36260, 0x2c5bfdd0, 0x88765a99, 0x93deba1e,
    0xac6ae342, 0xe865b84c, 0x0f4f2847, 0x7fdf0499, 0x78b1c9b3, 0x6a73261e, 0x601a96f6, 0xd2847933,
    0x489aa888, 0xe12e8093, 0x3bfa5a5f, 0xd96ba5f7, 0x7c8f4c8d, 0x80940c6f, 0xcef9dd1a, 0x7e1a055f,
    0x3483558b, 0x02b59cc4, 0x0c56333e, 0x05a5b813, 0x92d66287, 0x7516b679, 0x71bfe03f, 0x8056bf68,
    0xc24d0724, 0x8416bcf3, 0x234afbdb, 0x4b0d6f9c, 0xaba97333, 0x4b4f42b6, 0x7e8343ab, 0x7ffe2603,
    0xe590f73c, 0x45e10c76, 0xb07a6a78, 0xb35609d3, 0x1a027dfd, 0x90cb6e20, 0x82d3fe38, 0x7b409257,
    0x0e395afa, 0x1b802093, 0xcb0c6c59, 0x241e17e7, 0x1ee3ea0a, 0x41a82302, 0xab04350a, 0xf570beb7,
    0xbb444b9b, 0x83021459, 0x838d65dc, 0x1c439c84, 0x6fdcc454, 0xef9ef325, 0x18626c1c, 0x020d251f,
    0xc4aae786, 0x8614cb48, 0xf6f53ca6, 0x8710dbab, 0x89abec0d, 0xf29d41c1, 0x94b50336, 0xfdd49178,
    0x604658d1, 0x800e85be, 0xca1bb079, 0x7fa48eeb, 0xa3b7fafe, 0xd330436b, 0x64eb604c, 0x43a658ae,
    0x7caa1337, 0xddd445e6, 0x7efbf955, 0xb706ec71, 0x624a6b53, 0x9e0e231f, 0x97097248, 0xa1e1a17a,
    0x68dd2e44, 0x7f9d2e14, 0xddcc7074, 0x58324197, 0xc88fc426, 0x6d3640ae, 0x7ef83600, 0x759a0270,
    0x98b6d854, 0xd63c9b84, 0x372474a2, 0xe3f18cfd, 0x56ab0bdb, 0x85c9be7e, 0x47dfcfeb, 0xa5830d41,
    0x0ddd6283, 0xf4f480ad, 0x74c60e38, 0xab8943c3, 0xc1508fe7, 0x480cdc39, 0x8e097362, 0xa44793be,
    0x538b7e18, 0x545f5b41, 0x56529175, 0x9771a97e, 0xc2da7421, 0xea8265f2, 0x805d1163, 0x883c5d28,
    0x8ba94c48, 0x4f676e65, 0xf78735b3, 0xe1853671, 0x7f454f53, 0x18147f85, 0x7d09e15d, 0xdb4f3494,
    0x795c8973, 0x83310632, 0x85d8061c, 0x9a1a0ebf, 0xc125583c, 0x2a1b1a95, 0x7fd9103f, 0x71e98c72,
    0x40932ed7, 0x91ed227a, 0x3c5e560e, 0xe816dee9, 0xb0891b80, 0x600038ba, 0xc7d9a80d, 0x7fff5e09,
    0x7e3f4351, 0xbb6b4424, 0xb14448d4, 0x8d6bb7e1, 0xfb153626, 0xa68ad537, 0xd9782006, 0xf62f6991,
    0x359ba8c1, 0x02ccff0b, 0x91bf2256, 0x7ea71c4d, 0x560ce5df, 0xeeba289b, 0xa574c4e7, 0x9e04f6ee,
    0x7860a5ec, 0x0b8db4a2, 0x968ba3d7, 0x0b6c77df, 0xd6f3157d, 0x402eff1a, 0x49b820b3, 0x8152aebb,
    0xd180b0b6, 0x098604d4, 0x7ff92224, 0xede9c996, 0x89c58061, 0x829624c4, 0xc6e71ea7, 0xba94d915,
    0x389c3cf6, 0x5b4c5a06, 0x04b335e6, 0x516a8aab, 0x42c8d7d9, 0x92b12af6, 0x86c8549f, 0xfda98acf,
    0x819673b6, 0x69545dac, 0x6feaa230, 0x726e6d3f, 0x886ebdfe, 0x34f5730a, 0x7af63ba2, 0x77307bbf,
    0x7cd80630, 0x6e45efe0, 0x7f8ad7eb, 0x59d7df99, 0x86c70946, 0xda233629, 0x753f6cbf, 0x825eeb40,
};

/* sample rates (table 4.5.1) */
const int sampRateTab[12] PROGMEM = {
    96000, 88200, 64000, 48000, 44100, 32000,
    24000, 22050, 16000, 12000, 11025,  8000
};

/* max scalefactor band for prediction (main profile only) */
const uint8_t predSFBMax[12] PROGMEM = {
    33, 33, 38, 40, 40, 40, 41, 41, 37, 37, 37, 34
};

/* channel mapping (table 1.6.3.4) (-1 = unknown, so need to determine mapping based on rules in 8.5.1) */
const int8_t channelMapTab[8] PROGMEM = {
    -1, 1, 2, 3, 4, 5, 6, 8
};

/* number of channels in each element (SCE, CPE, etc.)
 * see AACElementID in aaccommon.h
 */
const uint8_t elementNumChans[8] PROGMEM = {
    1, 2, 0, 1, 0, 0, 0, 0
};

/* total number of scale factor bands in one window */
const uint8_t /*char*/ sfBandTotalShort[12] PROGMEM = {
    12, 12, 12, 14, 14, 14, 15, 15, 15, 15, 15, 15
};

const uint8_t /*char*/ sfBandTotalLong[12] PROGMEM = {
    41, 41, 47, 49, 49, 51, 47, 47, 43, 43, 43, 40
};

/* scale factor band tables */
const uint8_t sfBandTabShortOffset[12] PROGMEM = {0, 0, 0, 13, 13, 13, 28, 28, 44, 44, 44, 60};

const uint16_t sfBandTabShort[76] PROGMEM = {
    /* short block 64, 88, 96 kHz [13] (tables 4.5.24, 4.5.26) */
    0,   4,   8,  12,  16,  20,  24,  32,  40,  48,  64,  92, 128,

    /* short block 32, 44, 48 kHz [15] (table 4.5.15) */
    0,   4,   8,  12,  16,  20,  28,  36,  44,  56,  68,  80,  96, 112, 128,

    /* short block 22, 24 kHz [16] (table 4.5.22) */
    0,   4,   8,  12,  16,  20,  24,  28,  36,  44,  52,  64,  76,  92, 108, 128,

    /* short block 11, 12, 16 kHz [16] (table 4.5.20) */
    0,   4,   8,  12,  16,  20,  24,  28,  32,  40,  48,  60,  72,  88, 108, 128,

    /* short block 8 kHz [16] (table 4.5.18) */
    0,   4,   8,  12,  16,  20,  24,  28,  36,  44,  52,  60,  72,  88, 108, 128
};

const uint16_t sfBandTabLongOffset[12] PROGMEM = {0, 0, 42, 90, 90, 140, 192, 192, 240, 240, 240, 284};

const uint16_t sfBandTabLong[325] PROGMEM = {
    /* long block 88, 96 kHz [42] (table 4.5.25) */
      0,   4,   8,  12,  16,  20,  24,  28,  32,  36,  40,  44,  48,   52,
     56,  64,  72,  80,  88,  96, 108, 120, 132, 144, 156, 172, 188,  212,
    240, 276, 320, 384, 448, 512, 576, 640, 704, 768, 832, 896, 960, 1024,

    /* long block 64 kHz [48] (table 4.5.13) */
      0,   4,   8,  12,  16,  20,  24,  28,  32,  36,  40,  44,  48,  52,  56,   64,
     72,  80,  88, 100, 112, 124, 140, 156, 172, 192, 216, 240, 268, 304, 344,  384,
    424, 464, 504, 544, 584, 624, 664, 704, 744, 784, 824, 864, 904, 944, 984, 1024,

    /* long block 44, 48 kHz [50] (table 4.5.14) */
      0,   4,   8,  12,  16,  20,  24,  28,  32,  36,  40,  48,  56,  64,  72,   80,  88,
     96, 108, 120, 132, 144, 160, 176, 196, 216, 240, 264, 292, 320, 352, 384,  416, 448,
    480, 512, 544, 576, 608, 640, 672, 704, 736, 768, 800, 832, 864, 896, 928, 1024,

    /* long block 32 kHz [52] (table 4.5.16) */
      0,   4,   8,  12,  16,  20,  24,  28,  32,  36,  40,  48,  56,  64,  72,   80,  88,  96,
    108, 120, 132, 144, 160, 176, 196, 216, 240, 264, 292, 320, 352, 384, 416,  448, 480, 512,
    544, 576, 608, 640, 672, 704, 736, 768, 800, 832, 864, 896, 928, 960, 992, 1024,

    /* long block 22, 24 kHz [48] (table 4.5.21) */
      0,   4,   8,  12,  16,  20,  24,  28,  32,  36,  40,  44,  52,  60,  68,   76,
     84,  92, 100, 108, 116, 124, 136, 148, 160, 172, 188, 204, 220, 240, 260,  284,
    308, 336, 364, 396, 432, 468, 508, 552, 600, 652, 704, 768, 832, 896, 960, 1024,

    /* long block 11, 12, 16 kHz [44] (table 4.5.19) */
      0,   8,  16,  24,  32,  40,  48,  56,  64,  72,  80,  88, 100,  112, 124,
    136, 148, 160, 172, 184, 196, 212, 228, 244, 260, 280, 300, 320,  344, 368,
    396, 424, 456, 492, 532, 572, 616, 664, 716, 772, 832, 896, 960, 1024,

    /* long block 8 kHz [41] (table 4.5.17) */
      0,  12,  24,  36,  48,  60,  72,  84,  96, 108, 120, 132,  144, 156,
    172, 188, 204, 220, 236, 252, 268, 288, 308, 328, 348, 372,  396, 420,
    448, 476, 508, 544, 580, 620, 664, 712, 764, 820, 880, 944, 1024
};

/* TNS max bands (table 4.139) and max order (table 4.138) */
const uint8_t tnsMaxBandsShortOffset[3] PROGMEM = {0, 0, 12};

const uint16_t tnsMaxBandsShort[2*12] PROGMEM = {
     9,  9, 10, 14, 14, 14, 14, 14, 14, 14, 14, 14,        /* short block, Main/LC */
     7,  7,  7,  6,  6,  6,  7,  7,  8,  8,  8,  7        /* short block, SSR */
};

const uint8_t tnsMaxOrderShort[3] PROGMEM = {7, 7, 7};

const uint8_t tnsMaxBandsLongOffset[3] PROGMEM = {0, 0, 12};

const uint16_t tnsMaxBandsLong[2*12] PROGMEM = {
    31, 31, 34, 40, 42, 51, 46, 46, 42, 42, 42, 39,        /* long block, Main/LC */
    28, 28, 27, 26, 26, 26, 29, 29, 23, 23, 23, 19,        /* long block, SSR */
};

const uint8_t tnsMaxOrderLong[3] PROGMEM = {20, 12, 12};


/* k0Tab[sampRateIdx][k] = k0 = startMin + offset(bs_start_freq) for given sample rate (4.6.18.3.2.1)
 * downsampled (single-rate) SBR not currently supported
 */
const uint8_t k0Tab[NUM_SAMPLE_RATES_SBR][16] = {
    {  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 16, 18, 20, 23, 27, 31 }, /* 96 kHz */
    {  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 16, 18, 20, 23, 27, 31 }, /* 88 kHz */
    {  6,  8,  9, 10, 11, 12, 13, 14, 15, 16, 17, 19, 21, 23, 26, 30 }, /* 64 kHz */
    {  7,  9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 20, 22, 24, 27, 31 }, /* 48 kHz */
    {  8, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 21, 23, 25, 28, 32 }, /* 44 kHz */
    { 10, 12, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 25, 27, 29, 32 }, /* 32 kHz */
    { 11, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 25, 27, 29, 32 }, /* 24 kHz */
    { 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 26, 28, 30 }, /* 22 kHz */
    { 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31 }, /* 16 kHz */
};


/* k2Tab[sampRateIdx][k] = stopVector(bs_stop_freq) for given sample rate, bs_stop_freq = [0, 13] (4.6.18.3.2.1)
 * generated with Matlab script calc_stopvec.m
 * downsampled (single-rate) SBR not currently supported
 */
const uint8_t k2Tab[NUM_SAMPLE_RATES_SBR][14] = {
    { 13, 15, 17, 19, 21, 24, 27, 31, 35, 39, 44, 50, 57, 64 }, /* 96 kHz */
    { 15, 17, 19, 21, 23, 26, 29, 33, 37, 41, 46, 51, 57, 64 }, /* 88 kHz */
    { 20, 22, 24, 26, 28, 31, 34, 37, 41, 45, 49, 54, 59, 64 }, /* 64 kHz */
    { 21, 23, 25, 27, 29, 32, 35, 38, 41, 45, 49, 54, 59, 64 }, /* 48 kHz */
    { 23, 25, 27, 29, 31, 34, 37, 40, 43, 47, 51, 55, 59, 64 }, /* 44 kHz */
    { 32, 34, 36, 38, 40, 42, 44, 46, 49, 52, 55, 58, 61, 64 }, /* 32 kHz */
    { 32, 34, 36, 38, 40, 42, 44, 46, 49, 52, 55, 58, 61, 64 }, /* 24 kHz */
    { 35, 36, 38, 40, 42, 44, 46, 48, 50, 52, 55, 58, 61, 64 }, /* 22 kHz */
    { 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 60, 62, 64 }, /* 16 kHz */
};

const HuffInfo_t huffTabSBRInfo[10] PROGMEM = {
    {19, { 0,  2,  2,  2,  2,  2,  2,  2,  2,  2,  1,  2,  3,  4,  2,  7,  4,  8, 72,  0},   0},
    {20, { 0,  2,  2,  2,  2,  2,  1,  3,  3,  2,  4,  4,  4,  3,  2,  5,  6, 13, 15, 46}, 121},
    {17, { 1,  1,  1,  1,  1,  1,  1,  1,  1,  0,  2,  2,  0,  0,  1, 25, 10,  0,  0,  0}, 242},
    {19, { 1,  1,  1,  1,  1,  1,  1,  1,  1,  0,  3,  1,  0,  1,  1,  2,  1, 29,  2,  0}, 291},
    {19, { 1,  1,  1,  1,  1,  1,  1,  1,  1,  0,  2,  1,  2,  5,  1,  4,  2,  3, 34,  0}, 340},
    {20, { 1,  1,  1,  1,  1,  1,  0,  2,  2,  2,  2,  2,  1,  2,  3,  4,  4,  7, 10, 16}, 403},
    {14, { 1,  1,  1,  1,  1,  1,  1,  1,  1,  0,  0,  1, 13,  2,  0,  0,  0,  0,  0,  0}, 466},
    {14, { 1,  1,  1,  1,  1,  1,  1,  1,  1,  0,  1,  1,  6,  8,  0,  0,  0,  0,  0,  0}, 491},
    {14, { 1,  1,  1,  1,  1,  1,  0,  2,  0,  1,  1,  0, 51,  2,  0,  0,  0,  0,  0,  0}, 516},
    { 8, { 1,  1,  1,  0,  1,  1,  0, 20,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0}, 579},
};

/* Huffman tables from appendix 4.A.6.1, includes offset of -LAV[i] for table i */
const short huffTabSBR[604] PROGMEM = {
        /* SBR table sbr_tenv15 [121] (signed) */
       0,   -1,    1,   -2,    2,   -3,    3,   -4,    4,   -5,    5,   -6,    6,   -7,    7,   -8,
      -9,    8,  -10,    9,  -11,   10,  -12,  -13,   11,  -14,   12,  -15,  -16,   13,  -19,  -18,
     -17,   14,  -24,  -20,   16,  -26,  -21,   15,  -23,  -25,  -22,  -60,  -59,  -58,  -57,  -56,
     -55,  -54,  -53,  -52,  -51,  -50,  -49,  -48,  -47,  -46,  -45,  -44,  -43,  -42,  -41,  -40,
     -39,  -38,  -37,  -36,  -35,  -34,  -33,  -32,  -31,  -30,  -29,  -28,  -27,   17,   18,   19,
      20,   21,   22,   23,   24,   25,   26,   27,   28,   29,   30,   31,   32,   33,   34,   35,
      36,   37,   38,   39,   40,   41,   42,   43,   44,   45,   46,   47,   48,   49,   50,   51,
      52,   53,   54,   55,   56,   57,   58,   59,   60,
        /* SBR table sbr_fenv15 [121] (signed) */
       0,   -1,    1,   -2,   -3,    2,   -4,    3,   -5,    4,   -6,    5,   -7,    6,   -8,    7,
      -9,    8,  -10,    9,  -11,   10,   11,  -12,   12,  -13,   13,   14,  -14,  -15,   15,   16,
      17,  -16,  -17,  -18,  -19,   18,   19,  -20,  -21,   20,   21,  -24,  -23,  -22,  -26,  -28,
      22,   23,   25,  -41,  -25,   26,   27,  -30,  -27,   24,   28,   44,  -51,  -46,  -44,  -43,
     -37,  -33,  -31,  -29,   30,   37,   42,   47,   48,  -60,  -59,  -58,  -57,  -56,  -55,  -54,
     -53,  -52,  -50,  -49,  -48,  -47,  -45,  -42,  -40,  -39,  -38,  -36,  -35,  -34,  -32,   29,
      31,   32,   33,   34,   35,   36,   38,   39,   40,   41,   43,   45,   46,   49,   50,   51,
      52,   53,   54,   55,   56,   57,   58,   59,   60,
        /* SBR table sbr_tenv15b [49] (signed) */
       0,    1,   -1,    2,   -2,    3,   -3,    4,   -4,   -5,    5,   -6,    6,    7,   -7,    8,
     -24,  -23,  -22,  -21,  -20,  -19,  -18,  -17,  -16,  -15,  -14,  -13,  -12,  -11,  -10,   -9,
      -8,    9,   10,   11,   12,   13,   14,   15,   16,   17,   18,   19,   20,   21,   22,   23,
      24,
        /* SBR table sbr_fenv15b [49] (signed) */
       0,   -1,    1,   -2,    2,    3,   -3,   -4,    4,   -5,    5,   -6,    6,   -7,    7,    8,
      -9,   -8,  -24,  -23,  -22,  -21,  -20,  -19,  -18,  -17,  -16,  -15,  -14,  -13,  -12,  -11,
     -10,    9,   10,   11,   12,   13,   14,   15,   16,   17,   18,   19,   20,   21,   22,   23,
      24,
        /* SBR table sbr_tenv30 [63] (signed) */
       0,   -1,    1,   -2,    2,   -3,    3,   -4,    4,   -5,    5,   -6,   -7,    6,   -8,    7,
      -9,  -10,    8,    9,   10,  -13,  -11,  -12,  -14,   11,   12,  -31,  -30,  -29,  -28,  -27,
     -26,  -25,  -24,  -23,  -22,  -21,  -20,  -19,  -18,  -17,  -16,  -15,   13,   14,   15,   16,
      17,   18,   19,   20,   21,   22,   23,   24,   25,   26,   27,   28,   29,   30,   31,
        /* SBR table sbr_fenv30 [63] (signed) */
       0,   -1,    1,   -2,    2,   -3,    3,   -4,    4,   -5,    5,   -6,    6,   -7,    7,   -8,
       8,    9,   -9,  -10,   10,   11,  -11,  -12,   12,   13,  -13,  -15,   14,   15,  -14,   18,
     -18,  -24,  -19,   16,   17,  -22,  -21,  -16,   20,   21,   22,   25,  -23,  -20,   24,  -31,
     -30,  -29,  -28,  -27,  -26,  -25,  -17,   19,   23,   26,   27,   28,   29,   30,   31,
        /* SBR table sbr_tenv30b [25] (signed) */
       0,    1,   -1,   -2,    2,    3,   -3,   -4,    4,   -5,  -12,  -11,  -10,   -9,   -8,   -7,
      -6,    5,    6,    7,    8,    9,   10,   11,   12,
        /* SBR table sbr_fenv30b [25] (signed) */
       0,   -1,    1,   -2,    2,    3,   -3,   -4,    4,   -5,    5,    6,  -12,  -11,  -10,   -9,
      -8,   -7,   -6,    7,    8,    9,   10,   11,   12,
        /* SBR table sbr_tnoise30 [63] (signed) */
       0,    1,   -1,   -2,    2,   -3,    3,   -4,    4,   -5,    5,   11,  -31,  -30,  -29,  -28,
     -27,  -26,  -25,  -24,  -23,  -22,  -21,  -20,  -19,  -18,  -17,  -16,  -15,  -14,  -13,  -12,
     -11,  -10,   -9,   -8,   -7,   -6,    6,    7,    8,    9,   10,   12,   13,   14,   15,   16,
      17,   18,   19,   20,   21,   22,   23,   24,   25,   26,   27,   28,   29,   30,   31,
        /* SBR table sbr_tnoise30b [25] (signed) */
       0,   -1,    1,   -2,    2,  -12,  -11,  -10,   -9,   -8,   -7,   -6,   -5,   -4,   -3,    3,
       4,    5,    6,    7,    8,    9,   10,   11,   12,
};

/* newBWTab[prev invfMode][curr invfMode], format = Q31 (table 4.158)
 * sample file which uses all of these: al_sbr_sr_64_2_fsaac32.aac
 */
static const int newBWTab[4][4] PROGMEM = {
    {0x00000000, 0x4ccccccd, 0x73333333, 0x7d70a3d7},
    {0x4ccccccd, 0x60000000, 0x73333333, 0x7d70a3d7},
    {0x00000000, 0x60000000, 0x73333333, 0x7d70a3d7},
    {0x00000000, 0x60000000, 0x73333333, 0x7d70a3d7},
};

/* NINT(2.048E6 / Fs) (figure 4.47)
 * downsampled (single-rate) SBR not currently supported
 */
const uint8_t goalSBTab[NUM_SAMPLE_RATES_SBR] = {
    21, 23, 32, 43, 46, 64, 85, 93, 128
};

/* twiddle table for radix 4 pass, format = Q31 */
static const uint32_t twidTabOdd32[8*6] = {
    0x40000000, 0x00000000, 0x40000000, 0x00000000, 0x40000000, 0x00000000, 0x539eba45, 0xe7821d59,
    0x4b418bbe, 0xf383a3e2, 0x58c542c5, 0xdc71898d, 0x5a82799a, 0xd2bec333, 0x539eba45, 0xe7821d59,
    0x539eba45, 0xc4df2862, 0x539eba45, 0xc4df2862, 0x58c542c5, 0xdc71898d, 0x3248d382, 0xc13ad060,
    0x40000000, 0xc0000000, 0x5a82799a, 0xd2bec333, 0x00000000, 0xd2bec333, 0x22a2f4f8, 0xc4df2862,
    0x58c542c5, 0xcac933ae, 0xcdb72c7e, 0xf383a3e2, 0x00000000, 0xd2bec333, 0x539eba45, 0xc4df2862,
    0xac6145bb, 0x187de2a7, 0xdd5d0b08, 0xe7821d59, 0x4b418bbe, 0xc13ad060, 0xa73abd3b, 0x3536cc52,
};

/* PostMultiply64() table
 * format = Q30
 * reordered for sequential access
 *
 * for (i = 0; i <= (32/2); i++) {
 *   angle = i * M_PI / 64;
 *   x = (cos(angle) + sin(angle));
 *   x = sin(angle);
 * }
 */
static const int cos1sin1tab64[34] PROGMEM = {
    0x40000000, 0x00000000, 0x43103085, 0x0323ecbe, 0x45f704f7, 0x0645e9af, 0x48b2b335, 0x09640837,
    0x4b418bbe, 0x0c7c5c1e, 0x4da1fab5, 0x0f8cfcbe, 0x4fd288dc, 0x1294062f, 0x51d1dc80, 0x158f9a76,
    0x539eba45, 0x187de2a7, 0x553805f2, 0x1b5d100a, 0x569cc31b, 0x1e2b5d38, 0x57cc15bc, 0x20e70f32,
    0x58c542c5, 0x238e7673, 0x5987b08a, 0x261feffa, 0x5a12e720, 0x2899e64a, 0x5a6690ae, 0x2afad269,
    0x5a82799a, 0x2d413ccd,
};

/* coefficient table 4.A.87, format = Q31
 * reordered as:
 *   cTab[0],  cTab[64],  cTab[128], cTab[192], cTab[256],
 *   cTab[2],  cTab[66],  cTab[130], cTab[194], cTab[258],
 *   ...
 *   cTab[64], cTab[128], cTab[192], cTab[256], cTab[320]
 *
 * NOTE: cTab[1, 2, ... , 318, 319] = cTab[639, 638, ... 322, 321]
 *   except cTab[384] = -cTab[256], cTab[512] = -cTab[128]
 */
const uint32_t cTabA[165] PROGMEM = {
    0x00000000, 0x0055dba1, 0x01b2e41d, 0x09015651, 0x2e3a7532, 0xffed978a, 0x006090c4, 0x01fd3ba0, 0x08a24899, 0x311af3a4,
    0xfff0065d, 0x006b47fa, 0x024bf7a1, 0x082f552e, 0x33ff670e, 0xffef7b8b, 0x0075fded, 0x029e35b4, 0x07a8127d, 0x36e69691,
    0xffee1650, 0x00807994, 0x02f3e48d, 0x070bbf58, 0x39ce0477, 0xffecc31b, 0x008a7dd7, 0x034d01f0, 0x06593912, 0x3cb41219,
    0xffeb50b2, 0x009424c6, 0x03a966bb, 0x0590a67d, 0x3f962fb8, 0xffe9ca76, 0x009d10bf, 0x04083fec, 0x04b0adcb, 0x4272a385,
    0xffe88ba8, 0x00a520bb, 0x04694101, 0x03b8f8dc, 0x4547daea, 0xffe79e16, 0x00abe79e, 0x04cc2fcf, 0x02a99097, 0x4812f848,
    0xffe6d466, 0x00b1978d, 0x05303f87, 0x01816e06, 0x4ad237a2, 0xffe65416, 0x00b5c867, 0x05950122, 0x0040c496, 0x4d83976c,
    0xffe66dd0, 0x00b8394b, 0x05f9c051, 0xfee723c6, 0x5024d70e, 0xffe69423, 0x00b8c6b0, 0x065dd56a, 0xfd7475d8, 0x52b449de,
    0xffe75361, 0x00b73ab0, 0x06c0f0c0, 0xfbe8f5bd, 0x552f8ff7, 0xffe85b4b, 0x00b36acd, 0x0721bf22, 0xfa44a069, 0x579505f5,
    0xffea353a, 0x00acbd2f, 0x077fedb3, 0xf887507c, 0x59e2f69e, 0xffec8409, 0x00a3508f, 0x07da2b7f, 0xf6b1f3c3, 0x5c16d0ae,
    0xffef2395, 0x0096dcc2, 0x08303897, 0xf4c473c6, 0x5e2f6367, 0xfff294c3, 0x00872c63, 0x0880ffdd, 0xf2bf6ea4, 0x602b0c7f,
    0xfff681d6, 0x007400b8, 0x08cb4e23, 0xf0a3959f, 0x6207f220, 0xfffb42b0, 0x005d36df, 0x090ec1fc, 0xee71b2fe, 0x63c45243,
    0x00007134, 0x00426f36, 0x0949eaac, 0xec2a3f5f, 0x655f63f2, 0x0006b1cf, 0x0023b989, 0x097c1ee8, 0xe9cea84a, 0x66d76725,
    0x000d31b5, 0x0000e790, 0x09a3e163, 0xe75f8bb8, 0x682b39a4, 0x001471f8, 0xffda17f2, 0x09c0e59f, 0xe4de0cb0, 0x6959709d,
    0x001c3549, 0xffaea5d6, 0x09d19ca9, 0xe24b8f66, 0x6a619c5e, 0x0024dd50, 0xff7ee3f1, 0x09d5560b, 0xdfa93ab5, 0x6b42a864,
    0x002d8e42, 0xff4aabc8, 0x09caeb0f, 0xdcf898fb, 0x6bfbdd98, 0x003745f9, 0xff120d70, 0x09b18a1d, 0xda3b176a, 0x6c8c4c7a,
    0x004103f4, 0xfed4bec3, 0x09881dc5, 0xd7722f04, 0x6cf4073e, 0x004b6c46, 0xfe933dc0, 0x094d7ec2, 0xd49fd55f, 0x6d32730f,
    0x0055dba1, 0x01b2e41d, 0x09015651, 0x2e3a7532, 0x6d474e1d,
};

/* PreMultiply64() table
 * format = Q30
 * reordered for sequential access
 *
 * for (i = 0; i < 64/4; i++) {
 *   angle = (i + 0.25) * M_PI / nmdct;
 *   x = (cos(angle) + sin(angle));
 *   x =  sin(angle);
 *
 *   angle = (nmdct/2 - 1 - i + 0.25) * M_PI / nmdct;
 *   x = (cos(angle) + sin(angle));
 *   x =  sin(angle);
 * }
 */
static const int cos4sin4tab64[64] PROGMEM = {
    0x40c7d2bd, 0x00c90e90, 0x424ff28f, 0x3ff4e5e0, 0x43cdd89a, 0x03ecadcf, 0x454149fc, 0x3fc395f9,
    0x46aa0d6d, 0x070de172, 0x4807eb4b, 0x3f6af2e3, 0x495aada2, 0x0a2abb59, 0x4aa22036, 0x3eeb3347,
    0x4bde1089, 0x0d415013, 0x4d0e4de2, 0x3e44a5ef, 0x4e32a956, 0x104fb80e, 0x4f4af5d1, 0x3d77b192,
    0x50570819, 0x135410c3, 0x5156b6d9, 0x3c84d496, 0x5249daa2, 0x164c7ddd, 0x53304df6, 0x3b6ca4c4,
    0x5409ed4b, 0x19372a64, 0x54d69714, 0x3a2fcee8, 0x55962bc0, 0x1c1249d8, 0x56488dc5, 0x38cf1669,
    0x56eda1a0, 0x1edc1953, 0x57854ddd, 0x374b54ce, 0x580f7b19, 0x2192e09b, 0x588c1404, 0x35a5793c,
    0x58fb0568, 0x2434f332, 0x595c3e2a, 0x33de87de, 0x59afaf4c, 0x26c0b162, 0x59f54bee, 0x31f79948,
    0x5a2d0957, 0x29348937, 0x5a56deec, 0x2ff1d9c7, 0x5a72c63b, 0x2b8ef77d, 0x5a80baf6, 0x2dce88aa,
};

/* invBandTab[i] = 1.0 / (i + 1), Q31 */
static const int invBandTab[64] PROGMEM = {
    0x7fffffff, 0x40000000, 0x2aaaaaab, 0x20000000, 0x1999999a, 0x15555555, 0x12492492, 0x10000000,
    0x0e38e38e, 0x0ccccccd, 0x0ba2e8ba, 0x0aaaaaab, 0x09d89d8a, 0x09249249, 0x08888889, 0x08000000,
    0x07878788, 0x071c71c7, 0x06bca1af, 0x06666666, 0x06186186, 0x05d1745d, 0x0590b216, 0x05555555,
    0x051eb852, 0x04ec4ec5, 0x04bda12f, 0x04924925, 0x0469ee58, 0x04444444, 0x04210842, 0x04000000,
    0x03e0f83e, 0x03c3c3c4, 0x03a83a84, 0x038e38e4, 0x03759f23, 0x035e50d8, 0x03483483, 0x03333333,
    0x031f3832, 0x030c30c3, 0x02fa0be8, 0x02e8ba2f, 0x02d82d83, 0x02c8590b, 0x02b93105, 0x02aaaaab,
    0x029cbc15, 0x028f5c29, 0x02828283, 0x02762762, 0x026a439f, 0x025ed098, 0x0253c825, 0x02492492,
    0x023ee090, 0x0234f72c, 0x022b63cc, 0x02222222, 0x02192e2a, 0x02108421, 0x02082082, 0x02000000,
};

static const uint32_t poly43lo[5] PROGMEM = { 0x29a0bda9, 0xb02e4828, 0x5957aa1b, 0x236c498d, 0xff581859 };
static const uint32_t poly43hi[5] PROGMEM = { 0x10852163, 0xd333f6a4, 0x46e9408b, 0x27c2cef0, 0xfef577b4 };

/* pow2exp[i] = pow(2, i*4/3) exponent */
static const uint16_t pow2exp[8] PROGMEM = { 14, 13, 11, 10, 9, 7, 6, 5 };

/* pow2exp[i] = pow(2, i*4/3) fraction */
static const int pow2frac[8] PROGMEM = {
    0x6597fa94, 0x50a28be6, 0x7fffffff, 0x6597fa94,
    0x50a28be6, 0x7fffffff, 0x6597fa94, 0x50a28be6
};

/* pow(2, i/4.0) for i = [0,1,2,3], format = Q30 */
static const int pow14[4] PROGMEM = {
    0x40000000, 0x4c1bf829, 0x5a82799a, 0x6ba27e65
};

/* pow(2, i/4.0) * pow(j, 4.0/3.0) for i = [0,1,2,3],  j = [0,1,2,...,15]
 * format = Q28 for j = [0-3], Q25 for j = [4-15]
 */
static const uint32_t pow43_14[4][16] PROGMEM = {
    {
    0x00000000, 0x10000000, 0x285145f3, 0x453a5cdb, /* Q28 */
    0x0cb2ff53, 0x111989d6, 0x15ce31c8, 0x1ac7f203, /* Q25 */
    0x20000000, 0x257106b9, 0x2b16b4a3, 0x30ed74b4, /* Q25 */
    0x36f23fa5, 0x3d227bd3, 0x437be656, 0x49fc823c, /* Q25 */
    },
    {
    0x00000000, 0x1306fe0a, 0x2ff221af, 0x52538f52,
    0x0f1a1bf4, 0x1455ccc2, 0x19ee62a8, 0x1fd92396,
    0x260dfc14, 0x2c8694d8, 0x333dcb29, 0x3a2f5c7a,
    0x4157aed5, 0x48b3aaa3, 0x50409f76, 0x57fc3010,
    },
    {
    0x00000000, 0x16a09e66, 0x39047c0f, 0x61e734aa,
    0x11f59ac4, 0x182ec633, 0x1ed66a45, 0x25dfc55a,
    0x2d413ccd, 0x34f3462d, 0x3cefc603, 0x4531ab69,
    0x4db4adf8, 0x56752054, 0x5f6fcfcd, 0x68a1eca1,
    },
    {
    0x00000000, 0x1ae89f99, 0x43ce3e4b, 0x746d57b2,
    0x155b8109, 0x1cc21cdc, 0x24ac1839, 0x2d0a479e,
    0x35d13f33, 0x3ef80748, 0x48775c93, 0x524938cd,
    0x5c68841d, 0x66d0df0a, 0x717e7bfe, 0x7c6e0305,
    },
};

/* pow(j, 4.0 / 3.0) for j = [16,17,18,...,63], format = Q23 */
static const int pow43[48] PROGMEM = {
    0x1428a2fa, 0x15db1bd6, 0x1796302c, 0x19598d85,
    0x1b24e8bb, 0x1cf7fcfa, 0x1ed28af2, 0x20b4582a,
    0x229d2e6e, 0x248cdb55, 0x26832fda, 0x28800000,
    0x2a832287, 0x2c8c70a8, 0x2e9bc5d8, 0x30b0ff99,
    0x32cbfd4a, 0x34eca001, 0x3712ca62, 0x393e6088,
    0x3b6f47e0, 0x3da56717, 0x3fe0a5fc, 0x4220ed72,
    0x44662758, 0x46b03e7c, 0x48ff1e87, 0x4b52b3f3,
    0x4daaebfd, 0x5007b497, 0x5268fc62, 0x54ceb29c,
    0x5738c721, 0x59a72a59, 0x5c19cd35, 0x5e90a129,
    0x610b9821, 0x638aa47f, 0x660db90f, 0x6894c90b,
    0x6b1fc80c, 0x6daeaa0d, 0x70416360, 0x72d7e8b0,
    0x75722ef9, 0x78102b85, 0x7ab1d3ec, 0x7d571e09,
};

/* invTab[x] = 1/(x+1), format = Q30 */
static const int invTab[5] PROGMEM = {0x40000000, 0x20000000, 0x15555555, 0x10000000, 0x0ccccccd};

/* inverse quantization tables for TNS filter coefficients, format = Q31
 * see bottom of file for table generation
 * negative (vs. spec) since we use MADD for filter kernel
 */
static const uint32_t invQuant3[16] PROGMEM = {
    0x00000000, 0xc8767f65, 0x9becf22c, 0x83358feb, 0x83358feb, 0x9becf22c, 0xc8767f65, 0x00000000,
    0x2bc750e9, 0x5246dd49, 0x6ed9eba1, 0x7e0e2e32, 0x7e0e2e32, 0x6ed9eba1, 0x5246dd49, 0x2bc750e9,
};

static const uint32_t invQuant4[16] PROGMEM = {
    0x00000000, 0xe5632654, 0xcbf00dbe, 0xb4c373ee, 0xa0e0a15f, 0x9126145f, 0x8643c7b3, 0x80b381ac,
    0x7f7437ad, 0x7b1d1a49, 0x7294b5f2, 0x66256db2, 0x563ba8aa, 0x4362210e, 0x2e3d2abb, 0x17851aad,
};

static const int8_t sgnMask[3] = {0x02,  0x04,  0x08};
static const int8_t negMask[3] = {~0x03, ~0x07, ~0x0f};

/***********************************************************************************************************************
 * Function:    AACDecoder_AllocateBuffers
 *
 * Description: allocate all the memory needed for the AAC decoder
 *              try heap first, because it's faster
 *
 * Inputs:      none
 *
 * Outputs:     none
 *
 * Return:      false if not enough memory, otherwise true
 *
 **********************************************************************************************************************/

#ifdef CONFIG_IDF_TARGET_ESP32S3
    // ESP32-S3: If there is PSRAM, prefer it
    #define __malloc_heap_psram(size) \
        heap_caps_malloc_prefer(size, 2, MALLOC_CAP_DEFAULT|MALLOC_CAP_SPIRAM, MALLOC_CAP_DEFAULT|MALLOC_CAP_INTERNAL)
#else
    // ESP32, PSRAM is too slow, prefer SRAM
    #define __malloc_heap_psram(size) \
        heap_caps_malloc_prefer(size, 2, MALLOC_CAP_DEFAULT|MALLOC_CAP_INTERNAL, MALLOC_CAP_DEFAULT|MALLOC_CAP_SPIRAM)
#endif

bool AACDecoder_AllocateBuffers(void){

    /* here, sizes are: AACDecInfo_t:96 PSInfoBase_t:27364 ProgConfigElement_t*16:1312 PSInfoSBR_t:50788 */
#ifdef AAC_ENABLE_SBR
    if(!m_PSInfoSBR) {m_PSInfoSBR   = (PSInfoSBR_t*)__malloc_heap_psram(sizeof(PSInfoSBR_t));}

    if(!m_PSInfoSBR) {
        log_e("OOM in SBR, can't allocate %d bytes\n", sizeof(PSInfoSBR_t));
        return false; // ERR_AAC_SBR_INIT;
    }
    else {
        log_d("AAC Spectral Band Replication enabled, %d additional bytes allocated", sizeof(PSInfoSBR_t));
    }
#endif

    /* these could fall back to PSRAM if not enough heap available */
    if(!m_AACDecInfo) {m_AACDecInfo = (AACDecInfo_t*)        __malloc_heap_psram(sizeof(AACDecInfo_t));}
    if(!m_PSInfoBase) {m_PSInfoBase = (PSInfoBase_t*)        __malloc_heap_psram(sizeof(PSInfoBase_t));}
    if(!m_pce[0])     {m_pce[0]     = (ProgConfigElement_t*) __malloc_heap_psram(sizeof(ProgConfigElement_t)*16);}

    if(!m_AACDecInfo || !m_PSInfoBase || !m_pce[0]) {
            log_e("not enough memory to allocate aacdecoder buffers");
            AACDecoder_FreeBuffers();
            return false;
    }

    // Clear Buffer
    memset( m_AACDecInfo,        0, sizeof(AACDecInfo_t));              //Clear AACDecInfo
    memset( m_PSInfoBase,        0, sizeof(PSInfoBase_t));              //Clear PSInfoBase
    memset(&m_AACFrameInfo,      0, sizeof(AACFrameInfo_t));            //Clear AACFrameInfo
    memset(&m_fhADTS,            0, sizeof(ADTSHeader_t));              //Clear fhADTS
    memset(&m_fhADIF,            0, sizeof(ADIFHeader_t));              //Clear fhADIS
    memset( m_pce[0],            0, sizeof(ProgConfigElement_t) * 16);  //Clear ProgConfigElement
    memset(&m_pulseInfo[0],      0, sizeof(PulseInfo_t) *2);            //Clear PulseInfo
    memset(&m_aac_BitStreamInfo, 0, sizeof(aac_BitStreamInfo_t));       //Clear aac_BitStreamInfo
#ifdef AAC_ENABLE_SBR
    memset( m_PSInfoSBR,         0, sizeof(PSInfoSBR_t));               //Clear PSInfoSBR
    InitSBRState();
#endif

    m_AACDecInfo->prevBlockID = AAC_ID_INVALID;
    m_AACDecInfo->currBlockID = AAC_ID_INVALID;
    m_AACDecInfo->currInstTag = -1;
    for(int ch = 0; ch < MAX_NCHANS_ELEM; ch++)
        m_AACDecInfo->sbDeinterleaveReqd[ch] = 0;
    m_AACDecInfo->adtsBlocksLeft = 0;
    m_AACDecInfo->tnsUsed = 0;
    m_AACDecInfo->pnsUsed = 0;

    return true;
}

/**************************************************************************************
 * Function:    AACFlushCodec
 *
 * Description: flush internal codec state (after seeking, for example)
 *
 * Inputs:      valid AAC decoder instance pointer (HAACDecoder)
 *
 * Outputs:     updated state variables in aacDecInfo
 *
 * Return:      0 if successful, error code (< 0) if error
 **************************************************************************************/
int AACFlushCodec()
{
    int ch;

    if (!m_AACDecInfo)
        return ERR_AAC_NULL_POINTER;

    /* reset common state variables which change per-frame
     * don't touch state variables which are (usually) constant for entire clip
     *   (nChans, sampRate, profile, format, sbrEnabled)
     */
    m_AACDecInfo->prevBlockID = AAC_ID_INVALID;
    m_AACDecInfo->currBlockID = AAC_ID_INVALID;
    m_AACDecInfo->currInstTag = -1;
    for (ch = 0; ch < MAX_NCHANS_ELEM; ch++)
        m_AACDecInfo->sbDeinterleaveReqd[ch] = 0;
    m_AACDecInfo->adtsBlocksLeft = 0;
    m_AACDecInfo->tnsUsed = 0;
    m_AACDecInfo->pnsUsed = 0;

    /* reset internal codec state (flush overlap buffers, etc.) */
    memset(m_PSInfoBase->overlap, 0,  AAC_MAX_NCHANS * AAC_MAX_NSAMPS * sizeof(int));
    memset(m_PSInfoBase->prevWinShape, 0, AAC_MAX_NCHANS * sizeof(int));

    return ERR_AAC_NONE;
}
/***********************************************************************************************************************
 * Function:    AACDecoder_FreeBuffers
 *
 * Description: allocate all the memory needed for the AAC decoder
 *
 * Inputs:      none
 *
 * Outputs:     none
 *
 * Return:      none

 **********************************************************************************************************************/
void AACDecoder_FreeBuffers(void) {

//    uint32_t i = ESP.getFreeHeap();

    if(m_AACDecInfo)                         {free(m_AACDecInfo);    m_AACDecInfo=NULL;}
    if(m_PSInfoBase)                         {free(m_PSInfoBase);    m_PSInfoBase=NULL;}
    if(m_pce[0])                             {free(m_pce[0]);        m_pce[0]=NULL;}

#ifdef AAC_ENABLE_SBR
    if(m_PSInfoSBR)                           {free(m_PSInfoSBR);    m_PSInfoSBR=NULL;}               //Clear AACDecInfo
#endif

//    log_i("AACDecoder: %lu bytes memory was freed", ESP.getFreeHeap() - i);
}

/***********************************************************************************************************************
 * Function:    AACDecoder_IsInit
 *
 * Description: returns AAC decoder initialization status
 *
 * Inputs:      none
 *
 * Outputs:     none
 *
 * Return:      true if buffers allocated, otherwise false

 **********************************************************************************************************************/
bool AACDecoder_IsInit(void) {
    if(m_AACDecInfo && m_PSInfoBase && m_pce[0]){
        return true;
    }
    return false;
}

/***********************************************************************************************************************
 * Function:    AACDecoder_FreeBuffers
 *
 * Description: allocate all the memory needed for the AAC decoder
 *
 * Inputs:      none
 *
 * Outputs:     none
 *
 * Return:      none

 **********************************************************************************************************************/

/***********************************************************************************************************************
 * Function:    AACFindSyncWord
 *
 * Description: locate the next byte-alinged sync word in the raw AAC stream
 *
 * Inputs:      buffer to search for sync word
 *              max number of bytes to search in buffer
 *
 * Outputs:     none
 *
 * Return:      offset to first sync word (bytes from start of buf)
 *              -1 if sync not found after searching nBytes
 **********************************************************************************************************************/
int AACFindSyncWord(uint8_t *buf, int nBytes)
{
    int i;

    /* find byte-aligned syncword (12 bits = 0xFFF) */
    for (i = 0; i < nBytes - 1; i++) {
        if ( (buf[i+0] & SYNCWORDH) == SYNCWORDH && (buf[i+1] & SYNCWORDL) == SYNCWORDL )
            return i;
    }

    return -1;
}
//**************************************************************************************
int AACGetSampRate(){return m_AACDecInfo->sampRate * (m_AACDecInfo->sbrEnabled ? 2 : 1);}
int AACGetChannels(){return m_AACDecInfo->nChans;}
int AACGetBitsPerSample(){return 16;}
int AACGetID() {return m_AACDecInfo->id;} // 0-MPEG4, 1-MPEG2
uint8_t AACGetProfile() {return (uint8_t)m_AACDecInfo->profile;} // 0-Main, 1-LC, 2-SSR, 3-reserved
uint8_t AACGetFormat() {return (uint8_t)m_AACDecInfo->format;}   // 0-unknown 1-ADTS 2-ADIF, 3-RAW
int AACGetOutputSamps(){return m_AACDecInfo->nChans * AAC_MAX_NSAMPS  * (m_AACDecInfo->sbrEnabled ? 2 : 1);}
int AACGetBitrate() {
    uint32_t br = AACGetBitsPerSample() * AACGetChannels() *  AACGetSampRate();
    return (br / m_AACDecInfo->compressionRatio);
}
/**************************************************************************************
 * Function:    AACSetRawBlockParams
 *
 * Description: set internal state variables for decoding a stream of raw data blocks
 *
 * Inputs:      flag indicating source of parameters
 *              nChans, sampRate,
 *              and profile  0 = main, 1 = LC, 2 = SSR, 3 = reserved
 *                optionally filled-in
 *
 * Outputs:     updated codec state
 *
 * Return:      0 if successful, error code (< 0) if error
 *
 * Notes:       if copyLast == 1, then the codec sets up its internal state (for
 *                decoding raw blocks) based on previously-decoded ADTS header info
 *              if copyLast == 0, then the codec uses the values passed in
 *                aacFrameInfo to configure its internal state (useful when the
 *                source is MP4 format, for example)
 **************************************************************************************/
int AACSetRawBlockParams(int copyLast, int nChans, int sampRateCore, int profile)
{
    if (!m_AACDecInfo)
        return ERR_AAC_NULL_POINTER;

    m_AACDecInfo->format = AAC_FF_RAW;
    if (copyLast)
        return SetRawBlockParams(1, 0, 0, 0);
    else
        return SetRawBlockParams(0, nChans, sampRateCore, profile);
}

/***********************************************************************************************************************
 * Function:    AACDecode
 *
 * Description: decode AAC frame
 *
 * Inputs:      double pointer to buffer of AAC data
 *              pointer to number of valid bytes remaining in inbuf
 *              pointer to outbuf, big enough to hold one frame of decoded PCM samples
 *
 * Outputs:     PCM data in outbuf, interleaved LRLRLR... if stereo
 *                number of output samples = 1024 per channel
 *              updated inbuf pointer
 *              updated bytesLeft
 *
 * Return:      0 if successful, error code (< 0) if error
 *
 * Notes:       inbuf pointer and bytesLeft are not updated until whole frame is
 *                successfully decoded, so if ERR_AAC_INDATA_UNDERFLOW is returned
 *                just call AACDecode again with more data in inbuf
 **********************************************************************************************************************/
int AACDecode(uint8_t *inbuf, int *bytesLeft, short *outbuf)
{
    int err, offset, bitOffset, bitsAvail;
    int ch, baseChan, elementChans;
    uint8_t *inptr;

#ifdef AAC_ENABLE_SBR
    int baseChanSBR, elementChansSBR;
#endif

    /* make local copies (see "Notes" above) */
    inptr = inbuf;
    bitOffset = 0;
    bitsAvail = (*bytesLeft) << 3;

    /* first time through figure out what the file format is */
    if (m_AACDecInfo->format == AAC_FF_Unknown) {
        if (bitsAvail < 32)
            return ERR_AAC_INDATA_UNDERFLOW;

        if ((inptr)[0] == 'A' && (inptr)[1] == 'D' && (inptr)[2] == 'I' && (inptr)[3] == 'F') {
            /* unpack ADIF header */
            m_AACDecInfo->format = AAC_FF_ADIF;
            err = UnpackADIFHeader(&inptr, &bitOffset, &bitsAvail);
            if (err)
                return err;
        } else {
            /* assume ADTS by default */
            m_AACDecInfo->format = AAC_FF_ADTS;
        }
    }
    /* if ADTS, search for start of next frame */
    if (m_AACDecInfo->format == AAC_FF_ADTS) {
        /* can have 1-4 raw data blocks per ADTS frame (header only present for first one) */
        if (m_AACDecInfo->adtsBlocksLeft == 0) {
            offset = AACFindSyncWord(inptr, bitsAvail >> 3);
            if (offset < 0)
                return ERR_AAC_INDATA_UNDERFLOW;
            inptr += offset;
            bitsAvail -= (offset << 3);

            err = UnpackADTSHeader(&inptr, &bitOffset, &bitsAvail);
            if (err)
                return err;

            if (m_AACDecInfo->nChans == -1) {
                /* figure out implicit channel mapping if necessary */
                err = GetADTSChannelMapping(inptr, bitOffset, bitsAvail);
                if (err)
                    return err;
            }
        }
        m_AACDecInfo->adtsBlocksLeft--;
    } else if (m_AACDecInfo->format == AAC_FF_RAW) {
        err = PrepareRawBlock();
        if (err)
            return err;
    }

    /* check for valid number of channels */
    if (m_AACDecInfo->nChans > AAC_MAX_NCHANS || m_AACDecInfo->nChans <= 0)
        return ERR_AAC_NCHANS_TOO_HIGH;

    /* will be set later if active in this frame */
    m_AACDecInfo->tnsUsed = 0;
    m_AACDecInfo->pnsUsed = 0;

    bitOffset = 0;
    baseChan = 0;

#ifdef AAC_ENABLE_SBR
    baseChanSBR = 0;
#endif

    do {
        /* parse next syntactic element */
        err = DecodeNextElement(&inptr, &bitOffset, &bitsAvail);
        if (err)
            return err;

        elementChans = elementNumChans[m_AACDecInfo->currBlockID];
        if (baseChan + elementChans > AAC_MAX_NCHANS)
            return ERR_AAC_NCHANS_TOO_HIGH;

        /* noiseless decoder and dequantizer */
        for (ch = 0; ch < elementChans; ch++) {
            err = DecodeNoiselessData(&inptr, &bitOffset, &bitsAvail, ch);

            if (err)
                return err;

            if (AACDequantize(ch))
                return ERR_AAC_DEQUANT;
        }

        /* mid-side and intensity stereo */
        if (m_AACDecInfo->currBlockID == AAC_ID_CPE) {
            if (StereoProcess())
                return ERR_AAC_STEREO_PROCESS;
        }

        /* PNS, TNS, inverse transform */
        for (ch = 0; ch < elementChans; ch++) {

            if (PNS(ch))
                return ERR_AAC_PNS;

            if (m_AACDecInfo->sbDeinterleaveReqd[ch]) {
                /* deinterleave short blocks, if required */
                if (DeinterleaveShortBlocks(ch))
                    return ERR_AAC_SHORT_BLOCK_DEINT;
                m_AACDecInfo->sbDeinterleaveReqd[ch] = 0;
            }

            if (TNSFilter(ch))
                return ERR_AAC_TNS;

            if (IMDCT(ch, baseChan + ch, outbuf))
                return ERR_AAC_IMDCT;
        }

#ifdef AAC_ENABLE_SBR
        if (m_AACDecInfo->sbrEnabled && (m_AACDecInfo->currBlockID == AAC_ID_FIL ||
                                         m_AACDecInfo->currBlockID == AAC_ID_LFE)) {
            if (m_AACDecInfo->currBlockID == AAC_ID_LFE)
                elementChansSBR = elementNumChans[AAC_ID_LFE];
            else if (m_AACDecInfo->currBlockID == AAC_ID_FIL && (m_AACDecInfo->prevBlockID == AAC_ID_SCE ||
                                                                 m_AACDecInfo->prevBlockID == AAC_ID_CPE))
                elementChansSBR = elementNumChans[m_AACDecInfo->prevBlockID];
            else
                elementChansSBR = 0;

            if (baseChanSBR + elementChansSBR > AAC_MAX_NCHANS)
                return ERR_AAC_SBR_NCHANS_TOO_HIGH;

            /* parse SBR extension data if present (contained in a fill element) */
            if (DecodeSBRBitstream(baseChanSBR))
                return ERR_AAC_SBR_BITSTREAM;

            /* apply SBR */
            if (DecodeSBRData(baseChanSBR, outbuf))
                return ERR_AAC_SBR_DATA;

            baseChanSBR += elementChansSBR;
        }
#endif

    baseChan += elementChans;
    } while (m_AACDecInfo->currBlockID != AAC_ID_END);

    /* byte align after each raw_data_block */
    if (bitOffset) {
        inptr++;
        bitsAvail -= (8-bitOffset);
        bitOffset = 0;
        if (bitsAvail < 0)
            return ERR_AAC_INDATA_UNDERFLOW;
    }

    m_AACDecInfo->compressionRatio = (float)(AACGetOutputSamps()) * 2 / (inptr - inbuf);

    /* update pointers */
    m_AACDecInfo->frameCount++;
    *bytesLeft -= (inptr - inbuf);
    inbuf = inptr;

    return ERR_AAC_NONE;
}
/***********************************************************************************************************************
 * Function:    DecodeLPCCoefs
 *
 * Description: decode LPC coefficients for TNS
 *
 * Inputs:      order of TNS filter
 *              resolution of coefficients (3 or 4 bits)
 *              coefficients unpacked from bitstream
 *              scratch buffer (b) of size >= order
 *
 * Outputs:     LPC coefficients in Q(FBITS_LPC_COEFS), in 'a'
 *
 * Return:      none
 *
 * Notes:       assumes no guard bits in input transform coefficients
 *              a[i] = Q(FBITS_LPC_COEFS), don't store a0 = 1.0
 *                (so a[0] = first delay tap, etc.)
 *              max abs(a[i]) < log2(order), so for max order = 20 a[i] < 4.4
 *                (up to 3 bits of gain) so a[i] has at least 31 - FBITS_LPC_COEFS - 3
 *                guard bits
 *              to ensure no intermediate overflow in all-pole filter, set
 *                FBITS_LPC_COEFS such that number of guard bits >= log2(max order)
 **********************************************************************************************************************/
void DecodeLPCCoefs(int order, int res, int8_t *filtCoef, int *a, int *b)
{
    int i, m, t;
    const uint32_t *invQuantTab;

    if (res == 3)            invQuantTab = invQuant3;
    else if (res == 4)       invQuantTab = invQuant4;
    else                    return;

    for (m = 0; m < order; m++) {
        t = invQuantTab[filtCoef[m] & 0x0f];    /* t = Q31 */
        for (i = 0; i < m; i++)
            b[i] = a[i] - (MULSHIFT32(t, a[m-i-1]) << 1);
        for (i = 0; i < m; i++)
            a[i] = b[i];
        a[m] = t >> (31 - FBITS_LPC_COEFS);
    }
}

/***********************************************************************************************************************
 * Function:    FilterRegion
 *
 * Description: apply LPC filter to one region of coefficients
 *
 * Inputs:      number of transform coefficients in this region
 *              direction flag (forward = 1, backward = -1)
 *              order of filter
 *              'size' transform coefficients
 *              'order' LPC coefficients in Q(FBITS_LPC_COEFS)
 *              scratch buffer for history (must be >= order samples long)
 *
 * Outputs:     filtered transform coefficients
 *
 * Return:      guard bit mask (OR of abs value of all filtered transform coefs)
 *
 * Notes:       assumes no guard bits in input transform coefficients
 *              gains 0 int bits
 *              history buffer does not need to be preserved between regions
 **********************************************************************************************************************/
int FilterRegion(int size, int dir, int order, int *audioCoef, int *a, int *hist)
{
    int i, j, y, hi32, inc, gbMask;
    U64 sum64;

    /* init history to 0 every time */
    for (i = 0; i < order; i++)
        hist[i] = 0;

    sum64.w64 = 0;     /* avoid warning */
    gbMask = 0;
    inc = (dir ? -1 : 1);
    do {
        /* sum64 = a0*y[n] = 1.0*y[n] */
        y = *audioCoef;
        sum64.r.hi32 = y >> (32 - FBITS_LPC_COEFS);
        sum64.r.lo32 = y << FBITS_LPC_COEFS;

        /* sum64 += (a1*y[n-1] + a2*y[n-2] + ... + a[order-1]*y[n-(order-1)]) */
        for (j = order - 1; j > 0; j--) {
            sum64.w64 = MADD64(sum64.w64, hist[j], a[j]);
            hist[j] = hist[j-1];
        }
        sum64.w64 = MADD64(sum64.w64, hist[0], a[0]);
        y = (sum64.r.hi32 << (32 - FBITS_LPC_COEFS)) | (sum64.r.lo32 >> FBITS_LPC_COEFS);

        /* clip output (rare) */
        hi32 = sum64.r.hi32;
        if ((hi32 >> 31) != (hi32 >> (FBITS_LPC_COEFS-1)))
            y = (hi32 >> 31) ^ 0x7fffffff;

        hist[0] = y;
        *audioCoef = y;
        audioCoef += inc;
        gbMask |= FASTABS(y);
    } while (--size);

    return gbMask;
}

/***********************************************************************************************************************
 * Function:    TNSFilter
 *
 * Description: apply temporal noise shaping, if enabled
 *
 * Inputs:      index of current channel
 *
 * Outputs:     updated transform coefficients
 *              updated minimum guard bit count for this channel
 *
 * Return:      0 if successful, -1 if error
 **********************************************************************************************************************/
int TNSFilter(int ch)
{
    int win, winLen, nWindows, nSFB, filt, bottom, top, order, maxOrder, dir;
    int start, end, size, tnsMaxBand, numFilt, gbMask;
    int *audioCoef;
    uint8_t *filtLength, *filtOrder, *filtRes, *filtDir;
    int8_t *filtCoef;
    const uint16_t *tnsMaxBandTab;
    const uint16_t *sfbTab;
    ICSInfo_t *icsInfo;
    TNSInfo_t *ti;

    icsInfo = (ch == 1 && m_PSInfoBase->commonWin == 1) ? &(m_PSInfoBase->icsInfo[0]) : &(m_PSInfoBase->icsInfo[ch]);
    ti = &m_PSInfoBase->tnsInfo[ch];

    if (!ti->tnsDataPresent)
        return 0;

    if (icsInfo->winSequence == 2) {
        nWindows = NWINDOWS_SHORT;
        winLen = NSAMPS_SHORT;
        nSFB = sfBandTotalShort[m_PSInfoBase->sampRateIdx];
        maxOrder = tnsMaxOrderShort[m_AACDecInfo->profile];
        sfbTab = sfBandTabShort + sfBandTabShortOffset[m_PSInfoBase->sampRateIdx];
        tnsMaxBandTab = tnsMaxBandsShort + tnsMaxBandsShortOffset[m_AACDecInfo->profile];
        tnsMaxBand = tnsMaxBandTab[m_PSInfoBase->sampRateIdx];
    } else {
        nWindows = NWINDOWS_LONG;
        winLen = NSAMPS_LONG;
        nSFB = sfBandTotalLong[m_PSInfoBase->sampRateIdx];
        maxOrder = tnsMaxOrderLong[m_AACDecInfo->profile];
        sfbTab = sfBandTabLong + sfBandTabLongOffset[m_PSInfoBase->sampRateIdx];
        tnsMaxBandTab = tnsMaxBandsLong + tnsMaxBandsLongOffset[m_AACDecInfo->profile];
        tnsMaxBand = tnsMaxBandTab[m_PSInfoBase->sampRateIdx];
    }

    if (tnsMaxBand > icsInfo->maxSFB)
        tnsMaxBand = icsInfo->maxSFB;

    filtRes =    ti->coefRes;
    filtLength = ti->length;
    filtOrder =  ti->order;
    filtDir =    ti->dir;
    filtCoef =   ti->coef;

    gbMask = 0;
    audioCoef =  m_PSInfoBase->coef[ch];
    for (win = 0; win < nWindows; win++) {
        bottom = nSFB;
        numFilt = ti->numFilt[win];
        for (filt = 0; filt < numFilt; filt++) {
            top = bottom;
            bottom = top - *filtLength++;
            bottom = MAX(bottom, 0);
            order = *filtOrder++;
            order = MIN(order, maxOrder);

            if (order) {
                start = sfbTab[MIN(bottom, tnsMaxBand)];
                end   = sfbTab[MIN(top, tnsMaxBand)];
                size = end - start;
                if (size > 0) {
                    dir = *filtDir++;
                    if (dir)
                        start = end - 1;

                    DecodeLPCCoefs(order, filtRes[win], filtCoef, m_PSInfoBase->tnsLPCBuf, m_PSInfoBase->tnsWorkBuf);
                    gbMask |= FilterRegion(size, dir, order, audioCoef + start, m_PSInfoBase->tnsLPCBuf,
                                                                                           m_PSInfoBase->tnsWorkBuf);
                }
                filtCoef += order;
            }
        }
        audioCoef += winLen;
    }

    /* update guard bit count if necessary */
    size = CLZ(gbMask) - 1;
    if (m_PSInfoBase->gbCurrent[ch] > size)
        m_PSInfoBase->gbCurrent[ch] = size;

    return 0;
}

/***********************************************************************************************************************
 * Function:    DecodeSingleChannelElement
 *
 * Description: decode one SCE
 *
 * Inputs:      none
 *
 * Outputs:     updated element instance tag
 *
 * Return:      0 if successful, -1 if error
 *
 * Notes:       doesn't decode individual channel stream (part of DecodeNoiselessData)
 **********************************************************************************************************************/
int DecodeSingleChannelElement()
{
    /* read instance tag */
    m_AACDecInfo->currInstTag = GetBits(NUM_INST_TAG_BITS);

    return 0;
}

/***********************************************************************************************************************
 * Function:    DecodeChannelPairElement
 *
 * Description: decode one CPE
 *
 * Inputs:       none
 *
 * Outputs:     updated element instance tag
 *              updated commonWin
 *              updated ICS info, if commonWin == 1
 *              updated mid-side stereo info, if commonWin == 1
 *
 * Return:      0 if successful, -1 if error
 *
 * Notes:       doesn't decode individual channel stream (part of DecodeNoiselessData)
 **********************************************************************************************************************/
int DecodeChannelPairElement()
{
    int sfb, gp, maskOffset;
    uint8_t currBit, *maskPtr;
    ICSInfo_t *icsInfo;


    icsInfo = m_PSInfoBase->icsInfo;

    /* read instance tag */
    m_AACDecInfo->currInstTag = GetBits(NUM_INST_TAG_BITS);

    /* read common window flag and mid-side info (if present)
     * store msMask bits in m_PSInfoBase->msMaskBits[] as follows:
     *  long blocks -  pack bits for each SFB in range [0, maxSFB) starting with lsb of msMaskBits[0]
     *  short blocks - pack bits for each SFB in range [0, maxSFB), for each group [0, 7]
     * msMaskPresent = 0 means no M/S coding
     *               = 1 means m_PSInfoBase->msMaskBits contains 1 bit per SFB to toggle M/S coding
     *               = 2 means all SFB's are M/S coded (so m_PSInfoBase->msMaskBits is not needed)
     */
    m_PSInfoBase->commonWin = GetBits(1);
    if (m_PSInfoBase->commonWin) {
        DecodeICSInfo(icsInfo, m_PSInfoBase->sampRateIdx);
        m_PSInfoBase->msMaskPresent = GetBits(2);
        if (m_PSInfoBase->msMaskPresent == 1) {
            maskPtr = m_PSInfoBase->msMaskBits;
            *maskPtr = 0;
            maskOffset = 0;
            for (gp = 0; gp < icsInfo->numWinGroup; gp++) {
                for (sfb = 0; sfb < icsInfo->maxSFB; sfb++) {
                    currBit = (uint8_t)GetBits(1);
                    *maskPtr |= currBit << maskOffset;
                    if (++maskOffset == 8) {
                        maskPtr++;
                        *maskPtr = 0;
                        maskOffset = 0;
                    }
                }
            }
        }
    }

    return 0;
}

/***********************************************************************************************************************
 * Function:    DecodeLFEChannelElement
 *
 * Description: decode one LFE
 *
 * Inputs:      none
 *
 * Outputs:     updated element instance tag
 *
 * Return:      0 if successful, -1 if error
 *
 * Notes:       doesn't decode individual channel stream (part of DecodeNoiselessData)
 **********************************************************************************************************************/
int DecodeLFEChannelElement()
{
    /* read instance tag */
    m_AACDecInfo->currInstTag = GetBits( NUM_INST_TAG_BITS);

    return 0;
}

/***********************************************************************************************************************
 * Function:    DecodeDataStreamElement
 *
 * Description: decode one DSE
 *
 * Inputs:      none
 *
 * Outputs:     updated element instance tag
 *              filled in data stream buffer
 *
 * Return:      0 if successful, -1 if error
 **********************************************************************************************************************/
int DecodeDataStreamElement()
{
    uint32_t byteAlign, dataCount;
    uint8_t *dataBuf;

    m_AACDecInfo->currInstTag = GetBits( NUM_INST_TAG_BITS);
    byteAlign = GetBits(1);
    dataCount = GetBits(8);
    if (dataCount == 255)
        dataCount += GetBits(8);

    if (byteAlign)
        ByteAlignBitstream();

    m_PSInfoBase->dataCount = dataCount;
    dataBuf = m_PSInfoBase->dataBuf;
    while (dataCount--)
        *dataBuf++ = GetBits(8);

    return 0;
}

/***********************************************************************************************************************
 * Function:    DecodeProgramConfigElement
 *
 * Description: decode one PCE
 *
 * Inputs:      none
 *
 * Outputs:     filled-in ProgConfigElement_t struct
 *              updated aac_BitStreamInfo_t struct
 *
 * Return:      0 if successful, error code (< 0) if error
 *
 * Notes:       #define KEEP_PCE_COMMENTS to save the comment field of the PCE
 *                (otherwise we just skip it in the bitstream, to save memory)
 **********************************************************************************************************************/
int DecodeProgramConfigElement(uint8_t idx)
{
    int i;

    m_pce[idx]->elemInstTag =   GetBits(4);
    m_pce[idx]->profile =       GetBits(2);
    m_pce[idx]->sampRateIdx =   GetBits(4);
    m_pce[idx]->numFCE =        GetBits(4);
    m_pce[idx]->numSCE =        GetBits(4);
    m_pce[idx]->numBCE =        GetBits(4);
    m_pce[idx]->numLCE =        GetBits(2);
    m_pce[idx]->numADE =        GetBits(3);
    m_pce[idx]->numCCE =        GetBits(4);

    m_pce[idx]->monoMixdown = GetBits(1) << 4;    /* present flag */
    if (m_pce[idx]->monoMixdown)
        m_pce[idx]->monoMixdown |= GetBits(4);    /* element number */

    m_pce[idx]->stereoMixdown = GetBits(1) << 4;    /* present flag */
    if (m_pce[idx]->stereoMixdown)
        m_pce[idx]->stereoMixdown  |= GetBits(4);    /* element number */

    m_pce[idx]->matrixMixdown = GetBits(1) << 4;    /* present flag */
    if (m_pce[idx]->matrixMixdown) {
        m_pce[idx]->matrixMixdown  |= GetBits(2) << 1;    /* index */
        m_pce[idx]->matrixMixdown  |= GetBits(1);            /* pseudo-surround enable */
    }

    for (i = 0; i < m_pce[idx]->numFCE; i++) {
        m_pce[idx]->fce[i]  = GetBits(1) << 4;    /* is_cpe flag */
        m_pce[idx]->fce[i] |= GetBits(4);            /* tag select */
    }

    for (i = 0; i < m_pce[idx]->numSCE; i++) {
        m_pce[idx]->sce[i]  = GetBits(1) << 4;    /* is_cpe flag */
        m_pce[idx]->sce[i] |= GetBits(4);            /* tag select */
    }

    for (i = 0; i < m_pce[idx]->numBCE; i++) {
        m_pce[idx]->bce[i]  = GetBits(1) << 4;    /* is_cpe flag */
        m_pce[idx]->bce[i] |= GetBits(4);            /* tag select */
    }

    for (i = 0; i < m_pce[idx]->numLCE; i++)
        m_pce[idx]->lce[i] = GetBits(4);            /* tag select */

    for (i = 0; i < m_pce[idx]->numADE; i++)
        m_pce[idx]->ade[i] = GetBits(4);            /* tag select */

    for (i = 0; i < m_pce[idx]->numCCE; i++) {
        m_pce[idx]->cce[i]  = GetBits(1) << 4;    /* independent/dependent flag */
        m_pce[idx]->cce[i] |= GetBits(4);            /* tag select */
    }

    ByteAlignBitstream();
    /* eat comment bytes and throw away */
    i = GetBits(8);
    while (i--)
        GetBits(8);

    return 0;
}

/***********************************************************************************************************************
 * Function:    DecodeFillElement
 *
 * Description: decode one fill element
 *
 * Inputs:      none
 *                (14496-3, table 4.4.11)
 *
 * Outputs:     updated element instance tag
 *              unpacked extension payload
 *
 * Return:      0 if successful, -1 if error
 **********************************************************************************************************************/
int DecodeFillElement()
{
    unsigned int fillCount;
    uint8_t *fillBuf;

    fillCount = GetBits(4);
    if (fillCount == 15)
        fillCount += (GetBits(8) - 1);

    m_PSInfoBase->fillCount = fillCount;
    fillBuf = m_PSInfoBase->fillBuf;
    while (fillCount--)
        *fillBuf++ = GetBits(8);

    m_AACDecInfo->currInstTag = -1;    /* fill elements don't have instance tag */
    m_AACDecInfo->fillExtType = 0;

#ifdef AAC_ENABLE_SBR
    /* check for SBR
     * aacDecInfo->sbrEnabled is sticky (reset each raw_data_block), so for multichannel
     *    need to verify that all SCE/CPE/ICCE have valid SBR fill element following, and
     *    must upsample by 2 for LFE
     */
    if (m_PSInfoBase->fillCount > 0) {
        m_AACDecInfo->fillExtType = (int)((m_PSInfoBase->fillBuf[0] >> 4) & 0x0f);
        if (m_AACDecInfo->fillExtType == EXT_SBR_DATA || m_AACDecInfo->fillExtType == EXT_SBR_DATA_CRC)
            m_AACDecInfo->sbrEnabled = 1;
    }
#endif


    m_AACDecInfo->fillBuf = m_PSInfoBase->fillBuf;
    m_AACDecInfo->fillCount = m_PSInfoBase->fillCount;

    return 0;
}

/***********************************************************************************************************************
 * Function:    DecodeNextElement
 *
 * Description: decode next syntactic element in AAC frame
 *
 * Inputs:      double pointer to buffer containing next element
 *              pointer to bit offset
 *              pointer to number of valid bits remaining in buf
 *
 * Outputs:     type of element decoded (aacDecInfo->currBlockID)
 *              type of element decoded last time (aacDecInfo->prevBlockID)
 *              updated aacDecInfo state, depending on which element was decoded
 *              updated buffer pointer
 *              updated bit offset
 *              updated number of available bits
 *
 * Return:      0 if successful, error code (< 0) if error
 **********************************************************************************************************************/
int DecodeNextElement(uint8_t **buf, int *bitOffset, int *bitsAvail)
{
    int err, bitsUsed;

    /* init bitstream reader */
    SetBitstreamPointer((*bitsAvail + 7) >> 3, *buf);
    GetBits(*bitOffset);

    m_AACDecInfo->prevBlockID = m_AACDecInfo->currBlockID;
    m_AACDecInfo->currBlockID = GetBits(NUM_SYN_ID_BITS);

    /* set defaults (could be overwritten by DecodeXXXElement(), depending on currBlockID) */
    m_PSInfoBase->commonWin = 0;

    err = 0;
    switch (m_AACDecInfo->currBlockID) {
    case AAC_ID_SCE:
        err = DecodeSingleChannelElement();
        break;
    case AAC_ID_CPE:
        err = DecodeChannelPairElement();
        break;
    case AAC_ID_CCE:
        break;
    case AAC_ID_LFE:
        err = DecodeLFEChannelElement();
        break;
    case AAC_ID_DSE:
        err = DecodeDataStreamElement();
        break;
    case AAC_ID_PCE:
        err = DecodeProgramConfigElement(0);
        break;
    case AAC_ID_FIL:
        err = DecodeFillElement();
        break;
    case AAC_ID_END:
        break;
    }
    if (err)
        return ERR_AAC_SYNTAX_ELEMENT;

    /* update bitstream reader */
    bitsUsed = CalcBitsUsed(*buf, *bitOffset);
    *buf += (bitsUsed + *bitOffset) >> 3;
    *bitOffset = (bitsUsed + *bitOffset) & 0x07;
    *bitsAvail -= bitsUsed;

    if (*bitsAvail < 0)
        return ERR_AAC_INDATA_UNDERFLOW;

    return ERR_AAC_NONE;
}

/***********************************************************************************************************************
 * Function:    PreMultiply
 *
 * Description: pre-twiddle stage of DCT4
 *
 * Inputs:      table index (for transform size)
 *              buffer of nmdct samples
 *
 * Outputs:     processed samples in same buffer
 *
 * Return:      none
 *
 * Notes:       minimum 1 GB in, 2 GB out, gains 5 (short) or 8 (long) frac bits
 *              i.e. gains 2-7= -5 int bits (short) or 2-10 = -8 int bits (long)
 *              normalization by -1/N is rolled into tables here (see trigtabs.c)
 *              uses 3-mul, 3-add butterflies instead of 4-mul, 2-add
 **********************************************************************************************************************/
void PreMultiply(int tabidx, int *zbuf1)
{
    int i, nmdct, ar1, ai1, ar2, ai2, z1, z2;
    int t, cms2, cps2a, sin2a, cps2b, sin2b;
    int *zbuf2;
    const uint32_t *csptr;

    nmdct = nmdctTab[tabidx];
    zbuf2 = zbuf1 + nmdct - 1;
    csptr = cos4sin4tab + cos4sin4tabOffset[tabidx];

    /* whole thing should fit in registers - verify that compiler does this */
    for (i = nmdct >> 2; i != 0; i--) {
        /* cps2 = (cos+sin), sin2 = sin, cms2 = (cos-sin) */
        cps2a = *csptr++;
        sin2a = *csptr++;
        cps2b = *csptr++;
        sin2b = *csptr++;

        ar1 = *(zbuf1 + 0);
        ai2 = *(zbuf1 + 1);
        ai1 = *(zbuf2 + 0);
        ar2 = *(zbuf2 - 1);

        /* gain 2 ints bit from MULSHIFT32 by Q30, but drop 7 or 10 int bits from table scaling of 1/M
         * max per-sample gain (ignoring implicit scaling) = MAX(sin(angle)+cos(angle)) = 1.414
         * i.e. gain 1 GB since worst case is sin(angle) = cos(angle) = 0.707 (Q30), gain 2 from
         *   extra sign bits, and eat one in adding
         */
        t  = MULSHIFT32(sin2a, ar1 + ai1);
        z2 = MULSHIFT32(cps2a, ai1) - t;
        cms2 = cps2a - 2*sin2a;
        z1 = MULSHIFT32(cms2, ar1) + t;
        *zbuf1++ = z1;    /* cos*ar1 + sin*ai1 */
        *zbuf1++ = z2;    /* cos*ai1 - sin*ar1 */

        t  = MULSHIFT32(sin2b, ar2 + ai2);
        z2 = MULSHIFT32(cps2b, ai2) - t;
        cms2 = cps2b - 2*sin2b;
        z1 = MULSHIFT32(cms2, ar2) + t;
        *zbuf2-- = z2;    /* cos*ai2 - sin*ar2 */
        *zbuf2-- = z1;    /* cos*ar2 + sin*ai2 */
    }
}

/***********************************************************************************************************************
 * Function:    PostMultiply
 *
 * Description: post-twiddle stage of DCT4
 *
 * Inputs:      table index (for transform size)
 *              buffer of nmdct samples
 *
 * Outputs:     processed samples in same buffer
 *
 * Return:      none
 *
 * Notes:       minimum 1 GB in, 2 GB out - gains 2 int bits
 *              uses 3-mul, 3-add butterflies instead of 4-mul, 2-add
 **********************************************************************************************************************/
void PostMultiply(int tabidx, int *fft1)
{
    int i, nmdct, ar1, ai1, ar2, ai2, skipFactor;
    int t, cms2, cps2, sin2;
    int *fft2;
    const int *csptr;

    nmdct = nmdctTab[tabidx];
    csptr = cos1sin1tab;
    skipFactor = postSkip[tabidx];
    fft2 = fft1 + nmdct - 1;

    /* load coeffs for first pass
     * cps2 = (cos+sin), sin2 = sin, cms2 = (cos-sin)
     */
    cps2 = *csptr++;
    sin2 = *csptr;
    csptr += skipFactor;
    cms2 = cps2 - 2*sin2;

    for (i = nmdct >> 2; i != 0; i--) {
        ar1 = *(fft1 + 0);
        ai1 = *(fft1 + 1);
        ar2 = *(fft2 - 1);
        ai2 = *(fft2 + 0);

        /* gain 2 ints bit from MULSHIFT32 by Q30
         * max per-sample gain = MAX(sin(angle)+cos(angle)) = 1.414
         * i.e. gain 1 GB since worst case is sin(angle) = cos(angle) = 0.707 (Q30), gain 2 from
         *   extra sign bits, and eat one in adding
         */
        t = MULSHIFT32(sin2, ar1 + ai1);
        *fft2-- = t - MULSHIFT32(cps2, ai1);    /* sin*ar1 - cos*ai1 */
        *fft1++ = t + MULSHIFT32(cms2, ar1);    /* cos*ar1 + sin*ai1 */
        cps2 = *csptr++;
        sin2 = *csptr;
        csptr += skipFactor;

        ai2 = -ai2;
        t = MULSHIFT32(sin2, ar2 + ai2);
        *fft2-- = t - MULSHIFT32(cps2, ai2);    /* sin*ar1 - cos*ai1 */
        cms2 = cps2 - 2*sin2;
        *fft1++ = t + MULSHIFT32(cms2, ar2);    /* cos*ar1 + sin*ai1 */
    }
}

/***********************************************************************************************************************
 * Function:    PreMultiplyRescale
 *
 * Description: pre-twiddle stage of DCT4, with rescaling for extra guard bits
 *
 * Inputs:      table index (for transform size)
 *              buffer of nmdct samples
 *              number of guard bits to add to input before processing
 *
 * Outputs:     processed samples in same buffer
 *
 * Return:      none
 *
 * Notes:       see notes on PreMultiply(), above
 **********************************************************************************************************************/
void PreMultiplyRescale(int tabidx, int *zbuf1, int es)
{
    int i, nmdct, ar1, ai1, ar2, ai2, z1, z2;
    int t, cms2, cps2a, sin2a, cps2b, sin2b;
    int *zbuf2;
    const uint32_t *csptr;

    nmdct = nmdctTab[tabidx];
    zbuf2 = zbuf1 + nmdct - 1;
    csptr = cos4sin4tab + cos4sin4tabOffset[tabidx];

    /* whole thing should fit in registers - verify that compiler does this */
    for (i = nmdct >> 2; i != 0; i--) {
        /* cps2 = (cos+sin), sin2 = sin, cms2 = (cos-sin) */
        cps2a = *csptr++;
        sin2a = *csptr++;
        cps2b = *csptr++;
        sin2b = *csptr++;

        ar1 = *(zbuf1 + 0) >> es;
        ai1 = *(zbuf2 + 0) >> es;
        ai2 = *(zbuf1 + 1) >> es;

        t  = MULSHIFT32(sin2a, ar1 + ai1);
        z2 = MULSHIFT32(cps2a, ai1) - t;
        cms2 = cps2a - 2*sin2a;
        z1 = MULSHIFT32(cms2, ar1) + t;
        *zbuf1++ = z1;
        *zbuf1++ = z2;

        ar2 = *(zbuf2 - 1) >> es;    /* do here to free up register used for es */

        t  = MULSHIFT32(sin2b, ar2 + ai2);
        z2 = MULSHIFT32(cps2b, ai2) - t;
        cms2 = cps2b - 2*sin2b;
        z1 = MULSHIFT32(cms2, ar2) + t;
        *zbuf2-- = z2;
        *zbuf2-- = z1;

    }
}

/***********************************************************************************************************************
 * Function:    PostMultiplyRescale
 *
 * Description: post-twiddle stage of DCT4, with rescaling for extra guard bits
 *
 * Inputs:      table index (for transform size)
 *              buffer of nmdct samples
 *              number of guard bits to remove from output
 *
 * Outputs:     processed samples in same buffer
 *
 * Return:      none
 *
 * Notes:       clips output to [-2^30, 2^30 - 1], guaranteeing at least 1 guard bit
 *              see notes on PostMultiply(), above
 **********************************************************************************************************************/
void PostMultiplyRescale(int tabidx, int *fft1, int es)
{
    int i, nmdct, ar1, ai1, ar2, ai2, skipFactor, z;
    int t, cs2, sin2;
    int *fft2;
    const int *csptr;

    nmdct = nmdctTab[tabidx];
    csptr = cos1sin1tab;
    skipFactor = postSkip[tabidx];
    fft2 = fft1 + nmdct - 1;

    /* load coeffs for first pass
     * cps2 = (cos+sin), sin2 = sin, cms2 = (cos-sin)
     */
    cs2 = *csptr++;
    sin2 = *csptr;
    csptr += skipFactor;

    for (i = nmdct >> 2; i != 0; i--) {
        ar1 = *(fft1 + 0);
        ai1 = *(fft1 + 1);
        ai2 = *(fft2 + 0);

        t = MULSHIFT32(sin2, ar1 + ai1);
        z = t - MULSHIFT32(cs2, ai1);
        {int sign = (z) >> 31; if (sign != (z) >> (30 - (es))) {(z) = sign ^ (0x3fffffff);} else {(z) = (z) << (es);}}
        *fft2-- = z;
        cs2 -= 2*sin2;
        z = t + MULSHIFT32(cs2, ar1);
        {int sign = (z) >> 31; if (sign != (z) >> (30 - (es))) {(z) = sign ^ (0x3fffffff);} else {(z) = (z) << (es);}}
        *fft1++ = z;

        cs2 = *csptr++;
        sin2 = *csptr;
        csptr += skipFactor;

        ar2 = *fft2;
        ai2 = -ai2;
        t = MULSHIFT32(sin2, ar2 + ai2);
        z = t - MULSHIFT32(cs2, ai2);
        {int sign = (z) >> 31; if (sign != (z) >> (30 - (es))) {(z) = sign ^ (0x3fffffff);} else {(z) = (z) << (es);}}
        *fft2-- = z;
        cs2 -= 2*sin2;
        z = t + MULSHIFT32(cs2, ar2);
        {int sign = (z) >> 31; if (sign != (z) >> (30 - (es))) {(z) = sign ^ (0x3fffffff);} else {(z) = (z) << (es);}}
        *fft1++ = z;
        cs2 += 2*sin2;
    }
}

/***********************************************************************************************************************
 * Function:    DCT4
 *
 * Description: type-IV DCT
 *
 * Inputs:      table index (for transform size)
 *              buffer of nmdct samples
 *              number of guard bits in the input buffer
 *
 * Outputs:     processed samples in same buffer
 *
 * Return:      none
 *
 * Notes:       operates in-place
 *              if number of guard bits in input is < GBITS_IN_DCT4, the input is
 *                scaled (>>) before the DCT4 and rescaled (<<, with clipping) after
 *                the DCT4 (rare)
 *              the output has FBITS_LOST_DCT4 fewer fraction bits than the input
 *              the output will always have at least 1 guard bit (GBITS_IN_DCT4 >= 4)
 *              int bits gained per stage (PreMul + FFT + PostMul)
 *                 short blocks = (-5 + 4 + 2) = 1 total
 *                 long blocks =  (-8 + 7 + 2) = 1 total
 **********************************************************************************************************************/
void DCT4(int tabidx, int *coef, int gb)
{
    int es;

    /* fast in-place DCT-IV - adds guard bits if necessary */
    if (gb < GBITS_IN_DCT4) {
        es = GBITS_IN_DCT4 - gb;
        PreMultiplyRescale(tabidx, coef, es);
        R4FFT(tabidx, coef);
        PostMultiplyRescale(tabidx, coef, es);
    } else {
        PreMultiply(tabidx, coef);
        R4FFT(tabidx, coef);
        PostMultiply(tabidx, coef);
    }
}

/***********************************************************************************************************************
 * Function:    BitReverse
 *
 * Description: Ken's fast in-place bit reverse, using super-small table
 *
 * Inputs:      buffer of samples
 *              table index (for transform size)
 *
 * Outputs:     bit-reversed samples in same buffer
 *
 * Return:      none
 **********************************************************************************************************************/
void BitReverse(int *inout, int tabidx)
{
    int *part0, *part1;
    int a,b, t;
    const uint8_t* tab = bitrevtab + bitrevtabOffset[tabidx];
    int nbits = nfftlog2Tab[tabidx];

    part0 = inout;
    part1 = inout + (1 << nbits);

    while ((a = pgm_read_byte(tab++)) != 0) {
        b = pgm_read_byte(tab++);

        t=part0[4*a+0]; part0[4*a+0]=part0[4*b+0]; part0[4*b+0]=t; /* 0xxx0 <-> 0yyy0 */
        t=part0[4*a+1]; part0[4*a+1]=part0[4*b+1]; part0[4*b+1]=t;

        t=part0[4*a+2]; part0[4*a+2]=part1[4*b+0]; part1[4*b+0]=t; /* 0xxx0 <-> 0yyy0 */
        t=part0[4*a+3]; part0[4*a+3]=part1[4*b+1]; part1[4*b+1]=t;

        t=part1[4*a+0]; part1[4*a+0]=part0[4*b+2]; part0[4*b+2]=t; /* 1xxx0 <-> 0yyy1 */
        t=part1[4*a+1]; part1[4*a+1]=part0[4*b+3]; part0[4*b+3]=t;

        t=part1[4*a+2]; part1[4*a+2]=part1[4*b+2]; part1[4*b+2]=t; /* 1xxx1 <-> 1yyy1 */
        t=part1[4*a+3]; part1[4*a+3]=part1[4*b+3]; part1[4*b+3]=t;
    }

    do {
        t=part0[4*a+2]; part0[4*a+2]=part1[4*a+0]; part1[4*a+0]=t; /* 0xxx1 <-> 1xxx0 */
        t=part0[4*a+3]; part0[4*a+3]=part1[4*a+1]; part1[4*a+1]=t;
    } while ((a = pgm_read_byte(tab++)) != 0);


}

/***********************************************************************************************************************
 * Function:    R4FirstPass
 *
 * Description: radix-4 trivial pass for decimation-in-time FFT
 *
 * Inputs:      buffer of (bit-reversed) samples
 *              number of R4 butterflies per group (i.e. nfft / 4)
 *
 * Outputs:     processed samples in same buffer
 *
 * Return:      none
 *
 * Notes:       assumes 2 guard bits, gains no integer bits,
 *                guard bits out = guard bits in - 2
 **********************************************************************************************************************/
void R4FirstPass(int *x, int bg)
{
    int ar, ai, br, bi, cr, ci, dr, di;

    for (; bg != 0; bg--) {

        ar = x[0] + x[2];
        br = x[0] - x[2];
        ai = x[1] + x[3];
        bi = x[1] - x[3];
        cr = x[4] + x[6];
        dr = x[4] - x[6];
        ci = x[5] + x[7];
        di = x[5] - x[7];

        /* max per-sample gain = 4.0 (adding 4 inputs together) */
        x[0] = ar + cr;
        x[4] = ar - cr;
        x[1] = ai + ci;
        x[5] = ai - ci;
        x[2] = br + di;
        x[6] = br - di;
        x[3] = bi - dr;
        x[7] = bi + dr;

        x += 8;
    }
}

/***********************************************************************************************************************
 * Function:    R8FirstPass
 *
 * Description: radix-8 trivial pass for decimation-in-time FFT
 *
 * Inputs:      buffer of (bit-reversed) samples
 *              number of R8 butterflies per group (i.e. nfft / 8)
 *
 * Outputs:     processed samples in same buffer
 *
 * Return:      none
 *
 * Notes:       assumes 3 guard bits, gains 1 integer bit
 *              guard bits out = guard bits in - 3 (if inputs are full scale)
 *                or guard bits in - 2 (if inputs bounded to +/- sqrt(2)/2)
 *              see scaling comments in code
 **********************************************************************************************************************/
void R8FirstPass(int *x, int bg)
{
    int ar, ai, br, bi, cr, ci, dr, di;
    int sr, si, tr, ti, ur, ui, vr, vi;
    int wr, wi, xr, xi, yr, yi, zr, zi;

    for (; bg != 0; bg--) {

        ar = x[0] + x[2];
        br = x[0] - x[2];
        ai = x[1] + x[3];
        bi = x[1] - x[3];
        cr = x[4] + x[6];
        dr = x[4] - x[6];
        ci = x[5] + x[7];
        di = x[5] - x[7];

        sr = ar + cr;
        ur = ar - cr;
        si = ai + ci;
        ui = ai - ci;
        tr = br - di;
        vr = br + di;
        ti = bi + dr;
        vi = bi - dr;

        ar = x[ 8] + x[10];
        br = x[ 8] - x[10];
        ai = x[ 9] + x[11];
        bi = x[ 9] - x[11];
        cr = x[12] + x[14];
        dr = x[12] - x[14];
        ci = x[13] + x[15];
        di = x[13] - x[15];

        /* max gain of wr/wi/yr/yi vs input = 2
         *  (sum of 4 samples >> 1)
         */
        wr = (ar + cr) >> 1;
        yr = (ar - cr) >> 1;
        wi = (ai + ci) >> 1;
        yi = (ai - ci) >> 1;

        /* max gain of output vs input = 4
         *  (sum of 4 samples >> 1 + sum of 4 samples >> 1)
         */
        x[ 0] = (sr >> 1) + wr;
        x[ 8] = (sr >> 1) - wr;
        x[ 1] = (si >> 1) + wi;
        x[ 9] = (si >> 1) - wi;
        x[ 4] = (ur >> 1) + yi;
        x[12] = (ur >> 1) - yi;
        x[ 5] = (ui >> 1) - yr;
        x[13] = (ui >> 1) + yr;

        ar = br - di;
        cr = br + di;
        ai = bi + dr;
        ci = bi - dr;

        /* max gain of xr/xi/zr/zi vs input = 4*sqrt(2)/2 = 2*sqrt(2)
         *  (sum of 8 samples, multiply by sqrt(2)/2, implicit >> 1 from Q31)
         */
        xr = MULSHIFT32(SQRTHALF, ar - ai);
        xi = MULSHIFT32(SQRTHALF, ar + ai);
        zr = MULSHIFT32(SQRTHALF, cr - ci);
        zi = MULSHIFT32(SQRTHALF, cr + ci);

        /* max gain of output vs input = (2 + 2*sqrt(2) ~= 4.83)
         *  (sum of 4 samples >> 1, plus xr/xi/zr/zi with gain of 2*sqrt(2))
         * in absolute terms, we have max gain of appx 9.656 (4 + 0.707*8)
         *  but we also gain 1 int bit (from MULSHIFT32 or from explicit >> 1)
         */
        x[ 6] = (tr >> 1) - xr;
        x[14] = (tr >> 1) + xr;
        x[ 7] = (ti >> 1) - xi;
        x[15] = (ti >> 1) + xi;
        x[ 2] = (vr >> 1) + zi;
        x[10] = (vr >> 1) - zi;
        x[ 3] = (vi >> 1) - zr;
        x[11] = (vi >> 1) + zr;

        x += 16;
    }
}

/***********************************************************************************************************************
 * Function:    R4Core
 *
 * Description: radix-4 pass for decimation-in-time FFT
 *
 * Inputs:      buffer of samples
 *              number of R4 butterflies per group
 *              number of R4 groups per pass
 *              pointer to twiddle factors tables
 *
 * Outputs:     processed samples in same buffer
 *
 * Return:      none
 *
 * Notes:       gain 2 integer bits per pass (see scaling comments in code)
 *              min 1 GB in
 *              gbOut = gbIn - 1 (short block) or gbIn - 2 (long block)
 *              uses 3-mul, 3-add butterflies instead of 4-mul, 2-add
 **********************************************************************************************************************/
void R4Core(int *x, int bg, int gp, int *wtab)
{
    int ar, ai, br, bi, cr, ci, dr, di, tr, ti;
    int wd, ws, wi;
    int i, j, step;
    int *xptr, *wptr;

    for (; bg != 0; gp <<= 2, bg >>= 2) {

        step = 2*gp;
        xptr = x;

        /* max per-sample gain, per group < 1 + 3*sqrt(2) ~= 5.25 if inputs x are full-scale
         * do 3 groups for long block, 2 groups for short block (gain 2 int bits per group)
         *
         * very conservative scaling:
         *   group 1: max gain = 5.25,           int bits gained = 2, gb used = 1 (2^3 = 8)
         *   group 2: max gain = 5.25^2 = 27.6,  int bits gained = 4, gb used = 1 (2^5 = 32)
         *   group 3: max gain = 5.25^3 = 144.7, int bits gained = 6, gb used = 2 (2^8 = 256)
         */
        for (i = bg; i != 0; i--) {

            wptr = wtab;

            for (j = gp; j != 0; j--) {

                ar = xptr[0];
                ai = xptr[1];
                xptr += step;

                /* gain 2 int bits for br/bi, cr/ci, dr/di (MULSHIFT32 by Q30)
                 * gain 1 net GB
                 */
                ws = wptr[0];
                wi = wptr[1];
                br = xptr[0];
                bi = xptr[1];
                wd = ws + 2*wi;
                tr = MULSHIFT32(wi, br + bi);
                br = MULSHIFT32(wd, br) - tr;    /* cos*br + sin*bi */
                bi = MULSHIFT32(ws, bi) + tr;    /* cos*bi - sin*br */
                xptr += step;

                ws = wptr[2];
                wi = wptr[3];
                cr = xptr[0];
                ci = xptr[1];
                wd = ws + 2*wi;
                tr = MULSHIFT32(wi, cr + ci);
                cr = MULSHIFT32(wd, cr) - tr;
                ci = MULSHIFT32(ws, ci) + tr;
                xptr += step;

                ws = wptr[4];
                wi = wptr[5];
                dr = xptr[0];
                di = xptr[1];
                wd = ws + 2*wi;
                tr = MULSHIFT32(wi, dr + di);
                dr = MULSHIFT32(wd, dr) - tr;
                di = MULSHIFT32(ws, di) + tr;
                wptr += 6;

                tr = ar;
                ti = ai;
                ar = (tr >> 2) - br;
                ai = (ti >> 2) - bi;
                br = (tr >> 2) + br;
                bi = (ti >> 2) + bi;

                tr = cr;
                ti = ci;
                cr = tr + dr;
                ci = di - ti;
                dr = tr - dr;
                di = di + ti;

                xptr[0] = ar + ci;
                xptr[1] = ai + dr;
                xptr -= step;
                xptr[0] = br - cr;
                xptr[1] = bi - di;
                xptr -= step;
                xptr[0] = ar - ci;
                xptr[1] = ai - dr;
                xptr -= step;
                xptr[0] = br + cr;
                xptr[1] = bi + di;
                xptr += 2;
            }
            xptr += 3*step;
        }
        wtab += 3*step;
    }
}


/***********************************************************************************************************************
 * Function:    R4FFT
 *
 * Description: Ken's very fast in-place radix-4 decimation-in-time FFT
 *
 * Inputs:      table index (for transform size)
 *              buffer of samples (non bit-reversed)
 *
 * Outputs:     processed samples in same buffer
 *
 * Return:      none
 *
 * Notes:       assumes 5 guard bits in for nfft <= 512
 *              gbOut = gbIn - 4 (assuming input is from PreMultiply)
 *              gains log2(nfft) - 2 int bits total
 *                so gain 7 int bits (LONG), 4 int bits (SHORT)
 **********************************************************************************************************************/
void R4FFT(int tabidx, int *x)
{
    int order = nfftlog2Tab[tabidx];
    int nfft = nfftTab[tabidx];

    /* decimation in time */
    BitReverse(x, tabidx);

    if (order & 0x1) {
        /* long block: order = 9, nfft = 512 */
        R8FirstPass(x, nfft >> 3);                        /* gain 1 int bit,  lose 2 GB */
        R4Core(x, nfft >> 5, 8, (int *)twidTabOdd);        /* gain 6 int bits, lose 2 GB */
    } else {
        /* short block: order = 6, nfft = 64 */
        R4FirstPass(x, nfft >> 2);                        /* gain 0 int bits, lose 2 GB */
        R4Core(x, nfft >> 4, 4, (int *)twidTabEven);    /* gain 4 int bits, lose 1 GB */
    }
}

/***********************************************************************************************************************
 * Function:    UnpackZeros
 *
 * Description: fill a section of coefficients with zeros
 *
 * Inputs:      number of coefficients
 *
 * Outputs:     nVals zeros, starting at coef
 *
 * Return:      none
 *
 * Notes:       assumes nVals is always a multiple of 4 because all scalefactor bands
 *                are a multiple of 4 coefficients long
 **********************************************************************************************************************/
void UnpackZeros(int nVals, int *coef)
{
    while (nVals > 0) {
        *coef++ = 0;
        *coef++ = 0;
        *coef++ = 0;
        *coef++ = 0;
        nVals -= 4;
    }
}

/***********************************************************************************************************************
 * Function:    UnpackQuads
 *
 * Description: decode a section of 4-way vector Huffman coded coefficients
 *
 * Inputs       index of Huffman codebook
 *              number of coefficients
 *
 * Outputs:     nVals coefficients, starting at coef
 *
 * Return:      none
 *
 * Notes:       assumes nVals is always a multiple of 4 because all scalefactor bands
 *                are a multiple of 4 coefficients long
 **********************************************************************************************************************/
void UnpackQuads(int cb, int nVals, int *coef)
{
    int w, x, y, z, maxBits, nCodeBits, nSignBits, val;
    uint32_t bitBuf;

    maxBits = huffTabSpecInfo[cb - HUFFTAB_SPEC_OFFSET].maxBits + 4;
    while (nVals > 0) {
        /* decode quad */
        bitBuf = GetBitsNoAdvance(maxBits) << (32 - maxBits);
        nCodeBits = DecodeHuffmanScalar(huffTabSpec, &huffTabSpecInfo[cb - HUFFTAB_SPEC_OFFSET], bitBuf, &val);

        w = (((int32_t)(val) << 20) >>   29);    /* bits 11-9, sign-extend */
        x = (((int32_t)(val) << 23) >>   29);    /* bits  8-6, sign-extend */
        y = (((int32_t)(val) << 26) >>   29);    /* bits  5-3, sign-extend */
        z = (((int32_t)(val) << 29) >>   29);    /* bits  2-0, sign-extend */

        bitBuf <<= nCodeBits;
        nSignBits = (int)(((uint32_t)(val) << 17) >> 29);    /* bits 14-12, unsigned */

        AdvanceBitstream(nCodeBits + nSignBits);
        if (nSignBits) {
            if (w)    {w ^= ((int32_t)bitBuf >> 31); w -= ((int32_t)bitBuf >> 31); bitBuf <<= 1;}
            if (x)    {x ^= ((int32_t)bitBuf >> 31); x -= ((int32_t)bitBuf >> 31); bitBuf <<= 1;}
            if (y)    {y ^= ((int32_t)bitBuf >> 31); y -= ((int32_t)bitBuf >> 31); bitBuf <<= 1;}
            if (z)    {z ^= ((int32_t)bitBuf >> 31); z -= ((int32_t)bitBuf >> 31); bitBuf <<= 1;}
        }
        *coef++ = w; *coef++ = x; *coef++ = y; *coef++ = z;
        nVals -= 4;
    }
}

/***********************************************************************************************************************
 * Function:    UnpackPairsNoEsc
 *
 * Description: decode a section of 2-way vector Huffman coded coefficients,
 *                using non-esc tables (5 through 10)
 *
 * Inputs       index of Huffman codebook (must not be the escape codebook)
 *              number of coefficients
 *
 * Outputs:     nVals coefficients, starting at coef
 *
 * Return:      none
 *
 * Notes:       assumes nVals is always a multiple of 2 because all scalefactor bands
 *                are a multiple of 4 coefficients long
 **********************************************************************************************************************/
void UnpackPairsNoEsc(int cb, int nVals, int *coef)
{
    int y, z, maxBits, nCodeBits, nSignBits, val;
    uint32_t bitBuf;

    maxBits = huffTabSpecInfo[cb - HUFFTAB_SPEC_OFFSET].maxBits + 2;
    while (nVals > 0) {
        /* decode pair */
        bitBuf = GetBitsNoAdvance(maxBits) << (32 - maxBits);
        nCodeBits = DecodeHuffmanScalar(huffTabSpec, &huffTabSpecInfo[cb-HUFFTAB_SPEC_OFFSET], bitBuf, &val);

        y = (((int32_t)(val) << 22) >>   27);    /* bits  9-5, sign-extend */
        z = (((int32_t)(val) << 27) >>   27);    /* bits  4-0, sign-extend */

        bitBuf <<= nCodeBits;
        nSignBits = (((uint32_t)(val) << 20) >> 30);    /* bits 11-10, unsigned */
        AdvanceBitstream(nCodeBits + nSignBits);
        if (nSignBits) {
            if (y)    {y ^= ((int32_t)bitBuf >> 31); y -= ((int32_t)bitBuf >> 31); bitBuf <<= 1;}
            if (z)    {z ^= ((int32_t)bitBuf >> 31); z -= ((int32_t)bitBuf >> 31); bitBuf <<= 1;}
        }
        *coef++ = y; *coef++ = z;
        nVals -= 2;
    }
}

/***********************************************************************************************************************
 * Function:    UnpackPairsEsc
 *
 * Description: decode a section of 2-way vector Huffman coded coefficients,
 *                using esc table (11)
 *
 * Inputs       index of Huffman codebook (must be the escape codebook)
 *              number of coefficients
 *
 * Outputs:     nVals coefficients, starting at coef
 *
 * Return:      none
 *
 * Notes:       assumes nVals is always a multiple of 2 because all scalefactor bands
 *                are a multiple of 4 coefficients long
 **********************************************************************************************************************/
void UnpackPairsEsc(int cb, int nVals, int *coef)
{
    int y, z, maxBits, nCodeBits, nSignBits, n, val;
    uint32_t bitBuf;

    maxBits = huffTabSpecInfo[cb - HUFFTAB_SPEC_OFFSET].maxBits + 2;
    while (nVals > 0) {
        /* decode pair with escape value */
        bitBuf = GetBitsNoAdvance(maxBits) << (32 - maxBits);
        nCodeBits = DecodeHuffmanScalar(huffTabSpec, &huffTabSpecInfo[cb-HUFFTAB_SPEC_OFFSET], bitBuf, &val);

        y = (((int32_t)(val) << 20) >>   26);    /* bits 11-6, sign-extend */
        z = (((int32_t)(val) << 26) >>   26);    /* bits  5-0, sign-extend */

        bitBuf <<= nCodeBits;
        nSignBits = (((uint32_t)(val) << 18) >> 30);    /* bits 13-12, unsigned */
        AdvanceBitstream(nCodeBits + nSignBits);

        if (y == 16) {
            n = 4;
            while (GetBits(1) == 1)
                n++;
            y = (1 << n) + GetBits(n);
        }
        if (z == 16) {
            n = 4;
            while (GetBits(1) == 1)
                n++;
            z = (1 << n) + GetBits(n);
        }

        if (nSignBits) {
            if (y)    {y ^= ((int32_t)bitBuf >> 31); y -= ((int32_t)bitBuf >> 31); bitBuf <<= 1;}
            if (z)    {z ^= ((int32_t)bitBuf >> 31); z -= ((int32_t)bitBuf >> 31); bitBuf <<= 1;}
        }

        *coef++ = y; *coef++ = z;
        nVals -= 2;
    }
}

/***********************************************************************************************************************
 * Function:    DecodeSpectrumLong
 *
 * Description: decode transform coefficients for frame with one long block
 *
 * Inputs:      index of current channel
 *
 * Outputs:     decoded, quantized coefficients for this channel
 *
 * Return:      none
 *
 * Notes:       adds in pulse data if present
 *              fills coefficient buffer with zeros in any region not coded with
 *                codebook in range [1, 11] (including sfb's above sfbMax)
 **********************************************************************************************************************/
void DecodeSpectrumLong(int ch)
{
    int i, sfb, cb, nVals, offset;
    const uint16_t *sfbTab;
    uint8_t *sfbCodeBook;
    int *coef;
    ICSInfo_t *icsInfo;

    coef = m_PSInfoBase->coef[ch];
    icsInfo = (ch == 1 && m_PSInfoBase->commonWin == 1) ? &(m_PSInfoBase->icsInfo[0]) : &(m_PSInfoBase->icsInfo[ch]);

    /* decode long block */
    sfbTab = sfBandTabLong + sfBandTabLongOffset[m_PSInfoBase->sampRateIdx];
    sfbCodeBook = m_PSInfoBase->sfbCodeBook[ch];
    for (sfb = 0; sfb < icsInfo->maxSFB; sfb++) {
        cb = *sfbCodeBook++;
        nVals = sfbTab[sfb+1] - sfbTab[sfb];

        if (cb == 0)
            UnpackZeros(nVals, coef);
        else if (cb <= 4)
            UnpackQuads(cb, nVals, coef);
        else if (cb <= 10)
            UnpackPairsNoEsc(cb, nVals, coef);
        else if (cb == 11)
            UnpackPairsEsc(cb, nVals, coef);
        else
            UnpackZeros(nVals, coef);

        coef += nVals;
    }

    /* fill with zeros above maxSFB */
    nVals = NSAMPS_LONG - sfbTab[sfb];
    UnpackZeros(nVals, coef);

    /* add pulse data, if present */
    if (m_pulseInfo[ch].pulseDataPresent) {
        coef = m_PSInfoBase->coef[ch];
        offset = sfbTab[m_pulseInfo[ch].startSFB];
        for (i = 0; i < m_pulseInfo[ch].numPulse; i++) {
            offset += m_pulseInfo[ch].offset[i];
            if (coef[offset] > 0)
                coef[offset] += m_pulseInfo[ch].amp[i];
            else
                coef[offset] -= m_pulseInfo[ch].amp[i];
        }
        ASSERT(offset < NSAMPS_LONG);
    }
}

/***********************************************************************************************************************
 * Function:    DecodeSpectrumShort
 *
 * Description: decode transform coefficients for frame with eight short blocks
 *
 * Inputs:      index of current channel
 *
 * Outputs:     decoded, quantized coefficients for this channel
 *
 * Return:      none
 *
 * Notes:       fills coefficient buffer with zeros in any region not coded with
 *                codebook in range [1, 11] (including sfb's above sfbMax)
 *              deinterleaves window groups into 8 windows
 **********************************************************************************************************************/
void DecodeSpectrumShort(int ch)
{
    int gp, cb, nVals=0, win, offset, sfb;
    const uint16_t *sfbTab;
    uint8_t *sfbCodeBook;
    int *coef;
    ICSInfo_t *icsInfo;

    coef = m_PSInfoBase->coef[ch];
    icsInfo = (ch == 1 && m_PSInfoBase->commonWin == 1) ? &(m_PSInfoBase->icsInfo[0]) : &(m_PSInfoBase->icsInfo[ch]);

    /* decode short blocks, deinterleaving in-place */
    sfbTab = sfBandTabShort + sfBandTabShortOffset[m_PSInfoBase->sampRateIdx];
    sfbCodeBook = m_PSInfoBase->sfbCodeBook[ch];
    for (gp = 0; gp < icsInfo->numWinGroup; gp++) {
        for (sfb = 0; sfb < icsInfo->maxSFB; sfb++) {
            nVals = sfbTab[sfb+1] - sfbTab[sfb];
            cb = *sfbCodeBook++;

            for (win = 0; win < icsInfo->winGroupLen[gp]; win++) {
                offset = win*NSAMPS_SHORT;
                if (cb == 0)
                    UnpackZeros(nVals, coef + offset);
                else if (cb <= 4)
                    UnpackQuads(cb, nVals, coef + offset);
                else if (cb <= 10)
                    UnpackPairsNoEsc(cb, nVals, coef + offset);
                else if (cb == 11)
                    UnpackPairsEsc(cb, nVals, coef + offset);
                else
                    UnpackZeros(nVals, coef + offset);
            }
            coef += nVals;
        }

        /* fill with zeros above maxSFB */
        for (win = 0; win < icsInfo->winGroupLen[gp]; win++) {
            offset = win*NSAMPS_SHORT;
            nVals = NSAMPS_SHORT - sfbTab[sfb];
            UnpackZeros(nVals, coef + offset);
        }
        coef += nVals;
        coef += (icsInfo->winGroupLen[gp] - 1)*NSAMPS_SHORT;
    }

    ASSERT(coef == m_PSInfoBase->coef[ch] + NSAMPS_LONG);
}

#ifndef AAC_ENABLE_SBR
/***********************************************************************************************************************
 * Function:    DecWindowOverlap
 *
 * Description: apply synthesis window, do overlap-add, clip to 16-bit PCM,
 *                for winSequence LONG-LONG
 *
 * Inputs:      input buffer (output of type-IV DCT)
 *              overlap buffer (saved from last time)
 *              number of channels
 *              window type (sin or KBD) for input buffer
 *              window type (sin or KBD) for overlap buffer
 *
 * Outputs:     one channel, one frame of 16-bit PCM, interleaved by nChans
 *
 * Return:      none
 *
 * Notes:       this processes one channel at a time, but skips every other sample in
 *                the output buffer (pcm) for stereo interleaving
 *              this should fit in registers on ARM
 *
 **********************************************************************************************************************/
void DecWindowOverlap(int *buf0, int *over0, short *pcm0, int nChans, int winTypeCurr, int winTypePrev)
{
    int in, w0, w1, f0, f1;
    int *buf1, *over1;
    short *pcm1;
    const int *wndCurr, *wndPrev;

    buf0 += (1024 >> 1);
    buf1  = buf0  - 1;
    pcm1  = pcm0 + (1024 - 1) * nChans;
    over1 = over0 + 1024 - 1;

    wndPrev = (winTypePrev == 1 ? kbdWindow + kbdWindowOffset[1] : sinWindow + sinWindowOffset[1]);
    if (winTypeCurr == winTypePrev) {
        /* cut window loads in half since current and overlap sections use same symmetric window */
        do {
            w0 = *wndPrev++;
            w1 = *wndPrev++;
            in = *buf0++;

            f0 = MULSHIFT32(w0, in);
            f1 = MULSHIFT32(w1, in);

            in = *over0;
            *pcm0 = CLIPTOSHORT( (in - f0 + (1 << (FBITS_OUT_IMDCT-1))) >> FBITS_OUT_IMDCT );
            pcm0 += nChans;

            in = *over1;
            *pcm1 = CLIPTOSHORT( (in + f1 + (1 << (FBITS_OUT_IMDCT-1))) >> FBITS_OUT_IMDCT );
            pcm1 -= nChans;

            in = *buf1--;
            *over1-- = MULSHIFT32(w0, in);
            *over0++ = MULSHIFT32(w1, in);
        } while (over0 < over1);
    } else {
        /* different windows for current and overlap parts - should still fit in registers on ARM w/o stack spill */
        wndCurr = (winTypeCurr == 1 ? kbdWindow + kbdWindowOffset[1] : sinWindow + sinWindowOffset[1]);
        do {
            w0 = *wndPrev++;
            w1 = *wndPrev++;
            in = *buf0++;

            f0 = MULSHIFT32(w0, in);
            f1 = MULSHIFT32(w1, in);

            in = *over0;
            *pcm0 = CLIPTOSHORT( (in - f0 + (1 << (FBITS_OUT_IMDCT-1))) >> FBITS_OUT_IMDCT );
            pcm0 += nChans;

            in = *over1;
            *pcm1 = CLIPTOSHORT( (in + f1 + (1 << (FBITS_OUT_IMDCT-1))) >> FBITS_OUT_IMDCT );
            pcm1 -= nChans;

            w0 = *wndCurr++;
            w1 = *wndCurr++;
            in = *buf1--;

            *over1-- = MULSHIFT32(w0, in);
            *over0++ = MULSHIFT32(w1, in);
        } while (over0 < over1);
    }
}

/***********************************************************************************************************************
 * Function:    DecWindowOverlapLongStart
 *
 * Description: apply synthesis window, do overlap-add, clip to 16-bit PCM,
 *                for winSequence LONG-START
 *
 * Inputs:      input buffer (output of type-IV DCT)
 *              overlap buffer (saved from last time)
 *              number of channels
 *              window type (sin or KBD) for input buffer
 *              window type (sin or KBD) for overlap buffer
 *
 * Outputs:     one channel, one frame of 16-bit PCM, interleaved by nChans
 *
 * Return:      none
 *
 * Notes:       this processes one channel at a time, but skips every other sample in
 *                the output buffer (pcm) for stereo interleaving
 *              this should fit in registers on ARM
 **********************************************************************************************************************/
void DecWindowOverlapLongStart(int *buf0, int *over0, short *pcm0, int nChans, int winTypeCurr, int winTypePrev)
{
    int i,  in, w0, w1, f0, f1;
    int *buf1, *over1;
    short *pcm1;
    const int *wndPrev, *wndCurr;

    buf0 += (1024 >> 1);
    buf1  = buf0  - 1;
    pcm1  = pcm0 + (1024 - 1) * nChans;
    over1 = over0 + 1024 - 1;

    wndPrev = (winTypePrev == 1 ? kbdWindow + kbdWindowOffset[1] : sinWindow + sinWindowOffset[1]);
    i = 448;    /* 2 outputs, 2 overlaps per loop */
    do {
        w0 = *wndPrev++;
        w1 = *wndPrev++;
        in = *buf0++;

        f0 = MULSHIFT32(w0, in);
        f1 = MULSHIFT32(w1, in);

        in = *over0;
        *pcm0 = CLIPTOSHORT( (in - f0 + (1 << (FBITS_OUT_IMDCT-1))) >> FBITS_OUT_IMDCT );
        pcm0 += nChans;

        in = *over1;
        *pcm1 = CLIPTOSHORT( (in + f1 + (1 << (FBITS_OUT_IMDCT-1))) >> FBITS_OUT_IMDCT );
        pcm1 -= nChans;

        in = *buf1--;

        *over1-- = 0;        /* Wn = 0 for n = (2047, 2046, ... 1600) */
        *over0++ = in >> 1;    /* Wn = 1 for n = (1024, 1025, ... 1471) */
    } while (--i);

    wndCurr = (winTypeCurr == 1 ? kbdWindow + kbdWindowOffset[0] : sinWindow + sinWindowOffset[0]);

    /* do 64 more loops - 2 outputs, 2 overlaps per loop */
    do {
        w0 = *wndPrev++;
        w1 = *wndPrev++;
        in = *buf0++;

        f0 = MULSHIFT32(w0, in);
        f1 = MULSHIFT32(w1, in);

        in = *over0;
        *pcm0 = CLIPTOSHORT( (in - f0 + (1 << (FBITS_OUT_IMDCT-1))) >> FBITS_OUT_IMDCT );
        pcm0 += nChans;

        in = *over1;
        *pcm1 = CLIPTOSHORT( (in + f1 + (1 << (FBITS_OUT_IMDCT-1))) >> FBITS_OUT_IMDCT );
        pcm1 -= nChans;

        w0 = *wndCurr++;    /* W[0], W[1], ... --> W[255], W[254], ... */
        w1 = *wndCurr++;    /* W[127], W[126], ... --> W[128], W[129], ... */
        in = *buf1--;

        *over1-- = MULSHIFT32(w0, in);    /* Wn = short window for n = (1599, 1598, ... , 1536) */
        *over0++ = MULSHIFT32(w1, in);    /* Wn = short window for n = (1472, 1473, ... , 1535) */
    } while (over0 < over1);
}

/***********************************************************************************************************************
 * Function:    DecWindowOverlapLongStop
 *
 * Description: apply synthesis window, do overlap-add, clip to 16-bit PCM,
 *                for winSequence LONG-STOP
 *
 * Inputs:      input buffer (output of type-IV DCT)
 *              overlap buffer (saved from last time)
 *              number of channels
 *              window type (sin or KBD) for input buffer
 *              window type (sin or KBD) for overlap buffer
 *
 * Outputs:     one channel, one frame of 16-bit PCM, interleaved by nChans
 *
 * Return:      none
 *
 * Notes:       this processes one channel at a time, but skips every other sample in
 *                the output buffer (pcm) for stereo interleaving
 *              this should fit in registers on ARM
 **********************************************************************************************************************/
void DecWindowOverlapLongStop(int *buf0, int *over0, short *pcm0, int nChans, int winTypeCurr, int winTypePrev)
{
    int i, in, w0, w1, f0, f1;
    int *buf1, *over1;
    short *pcm1;
    const int *wndPrev, *wndCurr;

    buf0 += (1024 >> 1);
    buf1  = buf0  - 1;
    pcm1  = pcm0 + (1024 - 1) * nChans;
    over1 = over0 + 1024 - 1;

    wndPrev = (winTypePrev == 1 ? kbdWindow + kbdWindowOffset[0] : sinWindow + sinWindowOffset[0]);
    wndCurr = (winTypeCurr == 1 ? kbdWindow + kbdWindowOffset[1] : sinWindow + sinWindowOffset[1]);

    i = 448;    /* 2 outputs, 2 overlaps per loop */
    do {
        /* Wn = 0 for n = (0, 1, ... 447) */
        /* Wn = 1 for n = (576, 577, ... 1023) */
        in = *buf0++;
        f1 = in >> 1;    /* scale since skipping multiply by Q31 */

        in = *over0;
        *pcm0 = CLIPTOSHORT( (in + (1 << (FBITS_OUT_IMDCT-1))) >> FBITS_OUT_IMDCT );
        pcm0 += nChans;

        in = *over1;
        *pcm1 = CLIPTOSHORT( (in + f1 + (1 << (FBITS_OUT_IMDCT-1))) >> FBITS_OUT_IMDCT );
        pcm1 -= nChans;

        w0 = *wndCurr++;
        w1 = *wndCurr++;
        in = *buf1--;

        *over1-- = MULSHIFT32(w0, in);
        *over0++ = MULSHIFT32(w1, in);
    } while (--i);

    /* do 64 more loops - 2 outputs, 2 overlaps per loop */
    do {
        w0 = *wndPrev++;    /* W[0], W[1], ...W[63] */
        w1 = *wndPrev++;    /* W[127], W[126], ... W[64] */
        in = *buf0++;

        f0 = MULSHIFT32(w0, in);
        f1 = MULSHIFT32(w1, in);

        in = *over0;
        *pcm0 = CLIPTOSHORT( (in - f0 + (1 << (FBITS_OUT_IMDCT-1))) >> FBITS_OUT_IMDCT );
        pcm0 += nChans;

        in = *over1;
        *pcm1 = CLIPTOSHORT( (in + f1 + (1 << (FBITS_OUT_IMDCT-1))) >> FBITS_OUT_IMDCT );
        pcm1 -= nChans;

        w0 = *wndCurr++;
        w1 = *wndCurr++;
        in = *buf1--;

        *over1-- = MULSHIFT32(w0, in);
        *over0++ = MULSHIFT32(w1, in);
    } while (over0 < over1);
}

/***********************************************************************************************************************
 * Function:    DecWindowOverlapShort
 *
 * Description: apply synthesis window, do overlap-add, clip to 16-bit PCM,
 *                for winSequence EIGHT-SHORT (does all 8 short blocks)
 *
 * Inputs:      input buffer (output of type-IV DCT)
 *              overlap buffer (saved from last time)
 *              number of channels
 *              window type (sin or KBD) for input buffer
 *              window type (sin or KBD) for overlap buffer
 *
 * Outputs:     one channel, one frame of 16-bit PCM, interleaved by nChans
 *
 * Return:      none
 *
 * Notes:       this processes one channel at a time, but skips every other sample in
 *                the output buffer (pcm) for stereo interleaving
 *              this should fit in registers on ARM
 **********************************************************************************************************************/
void DecWindowOverlapShort(int *buf0, int *over0, short *pcm0, int nChans, int winTypeCurr, int winTypePrev)
{
    int i, in, w0, w1, f0, f1;
    int *buf1, *over1;
    short *pcm1;
    const int *wndPrev, *wndCurr;

    wndPrev = (winTypePrev == 1 ? kbdWindow + kbdWindowOffset[0] : sinWindow + sinWindowOffset[0]);
    wndCurr = (winTypeCurr == 1 ? kbdWindow + kbdWindowOffset[0] : sinWindow + sinWindowOffset[0]);

    /* pcm[0-447] = 0 + overlap[0-447] */
    i = 448;
    do {
        f0 = *over0++;
        f1 = *over0++;
        *pcm0 = CLIPTOSHORT( (f0 + (1 << (FBITS_OUT_IMDCT-1))) >> FBITS_OUT_IMDCT );    pcm0 += nChans;
        *pcm0 = CLIPTOSHORT( (f1 + (1 << (FBITS_OUT_IMDCT-1))) >> FBITS_OUT_IMDCT );    pcm0 += nChans;
        i -= 2;
    } while (i);

    /* pcm[448-575] = Wp[0-127] * block0[0-127] + overlap[448-575] */
    pcm1  = pcm0 + (128 - 1) * nChans;
    over1 = over0 + 128 - 1;
    buf0 += 64;
    buf1  = buf0  - 1;
    do {
        w0 = *wndPrev++;    /* W[0], W[1], ...W[63] */
        w1 = *wndPrev++;    /* W[127], W[126], ... W[64] */
        in = *buf0++;

        f0 = MULSHIFT32(w0, in);
        f1 = MULSHIFT32(w1, in);

        in = *over0;
        *pcm0 = CLIPTOSHORT( (in - f0 + (1 << (FBITS_OUT_IMDCT-1))) >> FBITS_OUT_IMDCT );
        pcm0 += nChans;

        in = *over1;
        *pcm1 = CLIPTOSHORT( (in + f1 + (1 << (FBITS_OUT_IMDCT-1))) >> FBITS_OUT_IMDCT );
        pcm1 -= nChans;

        w0 = *wndCurr++;
        w1 = *wndCurr++;
        in = *buf1--;

        /* save over0/over1 for next short block, in the slots just vacated */
        *over1-- = MULSHIFT32(w0, in);
        *over0++ = MULSHIFT32(w1, in);
    } while (over0 < over1);

    /* pcm[576-703] = Wc[128-255] * block0[128-255] + Wc[0-127] * block1[0-127] + overlap[576-703]
     * pcm[704-831] = Wc[128-255] * block1[128-255] + Wc[0-127] * block2[0-127] + overlap[704-831]
     * pcm[832-959] = Wc[128-255] * block2[128-255] + Wc[0-127] * block3[0-127] + overlap[832-959]
     */
    for (i = 0; i < 3; i++) {
        pcm0 += 64 * nChans;
        pcm1 = pcm0 + (128 - 1) * nChans;
        over0 += 64;
        over1 = over0 + 128 - 1;
        buf0 += 64;
        buf1 = buf0 - 1;
        wndCurr -= 128;

        do {
            w0 = *wndCurr++;    /* W[0], W[1], ...W[63] */
            w1 = *wndCurr++;    /* W[127], W[126], ... W[64] */
            in = *buf0++;

            f0 = MULSHIFT32(w0, in);
            f1 = MULSHIFT32(w1, in);

            in  = *(over0 - 128);    /* from last short block */
            in += *(over0 + 0);        /* from last full frame */
            *pcm0 = CLIPTOSHORT( (in - f0 + (1 << (FBITS_OUT_IMDCT-1))) >> FBITS_OUT_IMDCT );
            pcm0 += nChans;

            in  = *(over1 - 128);    /* from last short block */
            in += *(over1 + 0);        /* from last full frame */
            *pcm1 = CLIPTOSHORT( (in + f1 + (1 << (FBITS_OUT_IMDCT-1))) >> FBITS_OUT_IMDCT );
            pcm1 -= nChans;

            /* save over0/over1 for next short block, in the slots just vacated */
            in = *buf1--;
            *over1-- = MULSHIFT32(w0, in);
            *over0++ = MULSHIFT32(w1, in);
        } while (over0 < over1);
    }

    /* pcm[960-1023] = Wc[128-191] * block3[128-191] + Wc[0-63]   * block4[0-63] + overlap[960-1023]
     * over[0-63]    = Wc[192-255] * block3[192-255] + Wc[64-127] * block4[64-127]
     */
    pcm0 += 64 * nChans;
    over0 -= 832;                /* points at overlap[64] */
    over1 = over0 + 128 - 1;    /* points at overlap[191] */
    buf0 += 64;
    buf1 = buf0 - 1;
    wndCurr -= 128;
    do {
        w0 = *wndCurr++;    /* W[0], W[1], ...W[63] */
        w1 = *wndCurr++;    /* W[127], W[126], ... W[64] */
        in = *buf0++;

        f0 = MULSHIFT32(w0, in);
        f1 = MULSHIFT32(w1, in);

        in  = *(over0 + 768);    /* from last short block */
        in += *(over0 + 896);    /* from last full frame */
        *pcm0 = CLIPTOSHORT( (in - f0 + (1 << (FBITS_OUT_IMDCT-1))) >> FBITS_OUT_IMDCT );
        pcm0 += nChans;

        in  = *(over1 + 768);    /* from last short block */
        *(over1 - 128) = in + f1;

        in = *buf1--;
        *over1-- = MULSHIFT32(w0, in);    /* save in overlap[128-191] */
        *over0++ = MULSHIFT32(w1, in);    /* save in overlap[64-127] */
    } while (over0 < over1);

    /* over0 now points at overlap[128] */

    /* over[64-191]   = Wc[128-255] * block4[128-255] + Wc[0-127] * block5[0-127]
     * over[192-319]  = Wc[128-255] * block5[128-255] + Wc[0-127] * block6[0-127]
     * over[320-447]  = Wc[128-255] * block6[128-255] + Wc[0-127] * block7[0-127]
     * over[448-576]  = Wc[128-255] * block7[128-255]
     */
    for (i = 0; i < 3; i++) {
        over0 += 64;
        over1 = over0 + 128 - 1;
        buf0 += 64;
        buf1 = buf0 - 1;
        wndCurr -= 128;
        do {
            w0 = *wndCurr++;    /* W[0], W[1], ...W[63] */
            w1 = *wndCurr++;    /* W[127], W[126], ... W[64] */
            in = *buf0++;

            f0 = MULSHIFT32(w0, in);
            f1 = MULSHIFT32(w1, in);

            /* from last short block */
            *(over0 - 128) -= f0;
            *(over1 - 128)+= f1;

            in = *buf1--;
            *over1-- = MULSHIFT32(w0, in);
            *over0++ = MULSHIFT32(w1, in);
        } while (over0 < over1);
    }

    /* over[576-1024] = 0 */
    i = 448;
    over0 += 64;
    do {
        *over0++ = 0;
        *over0++ = 0;
        *over0++ = 0;
        *over0++ = 0;
        i -= 4;
    } while (i);
}

#endif  /* !AAC_ENABLE_SBR */

/***********************************************************************************************************************
 * Function:    IMDCT
 *
 * Description: inverse transform and convert to 16-bit PCM
 *
 * Inputs:      index of current channel (0 for SCE/LFE, 0 or 1 for CPE)
 *              output channel (range = [0, nChans-1])
 *
 * Outputs:     complete frame of decoded PCM, after inverse transform
 *
 * Return:      0 if successful, -1 if error
 *
 * Notes:       If AAC_ENABLE_SBR is defined at compile time then window + overlap
 *                does NOT clip to 16-bit PCM and does NOT interleave channels
 *              If AAC_ENABLE_SBR is NOT defined at compile time, then window + overlap
 *                does clip to 16-bit PCM and interleaves channels
 *              If SBR is enabled at compile time, but we don't know whether it is
 *                actually used for this frame (e.g. the first frame of a stream),
 *                we need to produce both clipped 16-bit PCM in outbuf AND
 *                unclipped 32-bit PCM in the SBR input buffer. In this case we make
 *                a separate pass over the 32-bit PCM to produce 16-bit PCM output.
 *                This inflicts a slight performance hit when decoding non-SBR files.
 **********************************************************************************************************************/
int IMDCT(int ch, int chOut, short *outbuf)
{
    int i;
    ICSInfo_t *icsInfo;

    icsInfo = (ch == 1 && m_PSInfoBase->commonWin == 1) ? &(m_PSInfoBase->icsInfo[0]) : &(m_PSInfoBase->icsInfo[ch]);
    outbuf += chOut;

    /* optimized type-IV DCT (operates inplace) */
    if (icsInfo->winSequence == 2) {
        /* 8 short blocks */
        for (i = 0; i < 8; i++)
            DCT4(0, m_PSInfoBase->coef[ch] + i*128, m_PSInfoBase->gbCurrent[ch]);
    } else {
        /* 1 long block */
        DCT4(1, m_PSInfoBase->coef[ch], m_PSInfoBase->gbCurrent[ch]);
    }

#ifdef AAC_ENABLE_SBR
    /* window, overlap-add, don't clip to short (send to SBR decoder)
     * store the decoded 32-bit samples in top half (second AAC_MAX_NSAMPS samples) of coef buffer
     */
    if (icsInfo->winSequence == 0)
        DecWindowOverlapNoClip(m_PSInfoBase->coef[ch], m_PSInfoBase->overlap[chOut],
                               m_PSInfoBase->sbrWorkBuf[ch], icsInfo->winShape, m_PSInfoBase->prevWinShape[chOut]);
    else if (icsInfo->winSequence == 1)
        DecWindowOverlapLongStartNoClip(m_PSInfoBase->coef[ch], m_PSInfoBase->overlap[chOut],
                                        m_PSInfoBase->sbrWorkBuf[ch], icsInfo->winShape, m_PSInfoBase->prevWinShape[chOut]);
    else if (icsInfo->winSequence == 2)
        DecWindowOverlapShortNoClip(m_PSInfoBase->coef[ch], m_PSInfoBase->overlap[chOut],
                                    m_PSInfoBase->sbrWorkBuf[ch], icsInfo->winShape, m_PSInfoBase->prevWinShape[chOut]);
    else if (icsInfo->winSequence == 3)
        DecWindowOverlapLongStopNoClip(m_PSInfoBase->coef[ch], m_PSInfoBase->overlap[chOut],
                                       m_PSInfoBase->sbrWorkBuf[ch], icsInfo->winShape, m_PSInfoBase->prevWinShape[chOut]);

    if (!m_AACDecInfo->sbrEnabled) {
        for (i = 0; i < AAC_MAX_NSAMPS; i++) {
            *outbuf = CLIPTOSHORT((m_PSInfoBase->sbrWorkBuf[ch][i] + RND_VAL) >> FBITS_OUT_IMDCT);
            outbuf += m_AACDecInfo->nChans;
        }
    }

    m_AACDecInfo->rawSampleBuf[ch] = m_PSInfoBase->sbrWorkBuf[ch];
    m_AACDecInfo->rawSampleBytes = sizeof(int);
    m_AACDecInfo->rawSampleFBits = FBITS_OUT_IMDCT;
#else
    /* window, overlap-add, round to PCM - optimized for each window sequence */
    if (icsInfo->winSequence == 0)
        DecWindowOverlap(m_PSInfoBase->coef[ch], m_PSInfoBase->overlap[chOut], outbuf, m_AACDecInfo->nChans,
                                                                  icsInfo->winShape, m_PSInfoBase->prevWinShape[chOut]);
    else if (icsInfo->winSequence == 1)
        DecWindowOverlapLongStart(m_PSInfoBase->coef[ch], m_PSInfoBase->overlap[chOut], outbuf, m_AACDecInfo->nChans,
                                                                  icsInfo->winShape, m_PSInfoBase->prevWinShape[chOut]);
    else if (icsInfo->winSequence == 2)
        DecWindowOverlapShort(m_PSInfoBase->coef[ch], m_PSInfoBase->overlap[chOut], outbuf, m_AACDecInfo->nChans,
                                                                  icsInfo->winShape, m_PSInfoBase->prevWinShape[chOut]);
    else if (icsInfo->winSequence == 3)
        DecWindowOverlapLongStop(m_PSInfoBase->coef[ch], m_PSInfoBase->overlap[chOut], outbuf, m_AACDecInfo->nChans,
                                                                  icsInfo->winShape, m_PSInfoBase->prevWinShape[chOut]);

    m_AACDecInfo->rawSampleBuf[ch] = 0;
    m_AACDecInfo->rawSampleBytes = 0;
    m_AACDecInfo->rawSampleFBits = 0;
#endif

    m_PSInfoBase->prevWinShape[chOut] = icsInfo->winShape;

    return 0;
}

/***********************************************************************************************************************
 * Function:    DecodeICSInfo
 *
 * Description: decode individual channel stream info
 *
 * Inputs:      sample rate index
 *
 * Outputs:     updated icsInfo struct
 *
 * Return:      none
 **********************************************************************************************************************/
void DecodeICSInfo(ICSInfo_t *icsInfo, int sampRateIdx)
{
    int sfb, g, mask;

    icsInfo->icsResBit =      GetBits(1);
    icsInfo->winSequence =    GetBits(2);
    icsInfo->winShape =       GetBits(1);
    if (icsInfo->winSequence == 2) {
        /* short block */
        icsInfo->maxSFB =     GetBits(4);
        icsInfo->sfGroup =    GetBits(7);
        icsInfo->numWinGroup =    1;
        icsInfo->winGroupLen[0] = 1;
        mask = 0x40;    /* start with bit 6 */
        for (g = 0; g < 7; g++) {
            if (icsInfo->sfGroup & mask)    {
                icsInfo->winGroupLen[icsInfo->numWinGroup - 1]++;
            } else {
                icsInfo->numWinGroup++;
                icsInfo->winGroupLen[icsInfo->numWinGroup - 1] = 1;
            }
            mask >>= 1;
        }
    } else {
        /* long block */
        icsInfo->maxSFB =               GetBits(6);
        icsInfo->predictorDataPresent = GetBits(1);
        if (icsInfo->predictorDataPresent) {
            icsInfo->predictorReset =   GetBits(1);
            if (icsInfo->predictorReset)
                icsInfo->predictorResetGroupNum = GetBits(5);
            for (sfb = 0; sfb < MIN(icsInfo->maxSFB, predSFBMax[sampRateIdx]); sfb++)
                icsInfo->predictionUsed[sfb] = GetBits(1);
        }
        icsInfo->numWinGroup = 1;
        icsInfo->winGroupLen[0] = 1;
    }
}

/***********************************************************************************************************************
 * Function:    DecodeSectionData
 *
 * Description: decode section data (scale factor band groupings and
 *                associated Huffman codebooks)
 *
 * Inputs:      window sequence (short or long blocks)
 *              number of window groups (1 for long blocks, 1-8 for short blocks)
 *              max coded scalefactor band
 *
 * Outputs:     index of Huffman codebook for each scalefactor band in each section
 *
 * Return:      none
 *
 * Notes:       sectCB, sectEnd, sfbCodeBook, ordered by window groups for short blocks
 **********************************************************************************************************************/
void DecodeSectionData(int winSequence, int numWinGrp, int maxSFB, uint8_t *sfbCodeBook)
{
    int g, cb, sfb;
    int sectLen, sectLenBits, sectLenIncr, sectEscapeVal;

    sectLenBits = (winSequence == 2 ? 3 : 5);
    sectEscapeVal = (1 << sectLenBits) - 1;

    for (g = 0; g < numWinGrp; g++) {
        sfb = 0;
        while (sfb < maxSFB) {
            cb = GetBits(4);    /* next section codebook */
            sectLen = 0;
            do {
                sectLenIncr = GetBits(sectLenBits);
                sectLen += sectLenIncr;
            } while (sectLenIncr == sectEscapeVal);

            sfb += sectLen;
            while (sectLen--)
                *sfbCodeBook++ = (uint8_t)cb;
        }
        ASSERT(sfb == maxSFB);
    }
}

/***********************************************************************************************************************
 * Function:    DecodeOneScaleFactor
 *
 * Description: decode one scalefactor using scalefactor Huffman codebook
 *
 * Inputs:      none
 *
 * Outputs:     none
 *
 * Return:      one decoded scalefactor, including index_offset of -60
 **********************************************************************************************************************/
int DecodeOneScaleFactor()
{
    int nBits, val;
    uint32_t bitBuf;

    /* decode next scalefactor from bitstream */
    bitBuf = GetBitsNoAdvance(huffTabScaleFactInfo.maxBits) << (32 - huffTabScaleFactInfo.maxBits);
    nBits = DecodeHuffmanScalar(huffTabScaleFact, &huffTabScaleFactInfo, bitBuf, &val);
    AdvanceBitstream(nBits);
    return val;
}

/***********************************************************************************************************************
 * Function:    DecodeScaleFactors
 *
 * Description: decode scalefactors, PNS energy, and intensity stereo weights
 *
 * Inputs:      number of window groups (1 for long blocks, 1-8 for short blocks)
 *              max coded scalefactor band
 *              global gain (starting value for differential scalefactor coding)
 *              index of Huffman codebook for each scalefactor band in each section
 *
 * Outputs:     decoded scalefactor for each section
 *
 * Return:      none
 *
 * Notes:       sfbCodeBook, scaleFactors ordered by window groups for short blocks
 *              for section with codebook 13, scaleFactors buffer has decoded PNS
 *                energy instead of regular scalefactor
 *              for section with codebook 14 or 15, scaleFactors buffer has intensity
 *                stereo weight instead of regular scalefactor
 **********************************************************************************************************************/
void DecodeScaleFactors(int numWinGrp, int maxSFB, int globalGain,
                               uint8_t *sfbCodeBook, short *scaleFactors)
{
    int g, sfbCB, nrg, npf, val, sf, is;

    /* starting values for differential coding */
    sf = globalGain;
    is = 0;
    nrg = globalGain - 90 - 256;
    npf = 1;

    for (g = 0; g < numWinGrp * maxSFB; g++) {
        sfbCB = *sfbCodeBook++;

        if (sfbCB  == 14 || sfbCB == 15) {
            /* intensity stereo - differential coding */
            val = DecodeOneScaleFactor();
            is += val;
            *scaleFactors++ = (short)is;
        } else if (sfbCB == 13) {
            /* PNS - first energy is directly coded, rest are Huffman coded (npf = noise_pcm_flag) */
            if (npf) {
                val = GetBits(9);
                npf = 0;
            } else {
                val = DecodeOneScaleFactor();
            }
            nrg += val;
            *scaleFactors++ = (short)nrg;
        } else if (sfbCB >= 1 && sfbCB <= 11) {
            /* regular (non-zero) region - differential coding */
            val = DecodeOneScaleFactor();
            sf += val;
            *scaleFactors++ = (short)sf;
        } else {
            /* inactive scalefactor band if codebook 0 */
            *scaleFactors++ = 0;
        }
    }
}

/***********************************************************************************************************************
 * Function:    DecodePulseInfo
 *
 * Description: decode pulse information
 *
 * Inputs:      none
 *
 * Outputs:     updated PulseInfo_t struct
 *
 * Return:      none
 **********************************************************************************************************************/
void DecodePulseInfo(uint8_t ch)
{
    int i;

    m_pulseInfo[ch].numPulse = GetBits(2) + 1;        /* add 1 here */
    m_pulseInfo[ch].startSFB = GetBits(6);
    for (i = 0; i < m_pulseInfo[ch].numPulse; i++) {
        m_pulseInfo[ch].offset[i] = GetBits(5);
        m_pulseInfo[ch].amp[i] = GetBits(4);
    }
}

/***********************************************************************************************************************
 * Function:    DecodeTNSInfo
 *
 * Description: decode TNS filter information
 *
 * Inputs:      window sequence (short or long blocks)
 *
 * Outputs:     updated TNSInfo_t struct
 *              buffer of decoded (signed) TNS filter coefficients
 *
 * Return:      none
 **********************************************************************************************************************/
void DecodeTNSInfo(int winSequence, TNSInfo_t *ti, int8_t *tnsCoef)
{
    int i, w, f, coefBits, compress;
    int8_t c, s, n;
    uint8_t *filtLength, *filtOrder, *filtDir;

    filtLength = ti->length;
    filtOrder =  ti->order;
    filtDir =    ti->dir;

    if (winSequence == 2) {
        /* short blocks */
        for (w = 0; w < NWINDOWS_SHORT; w++) {
            ti->numFilt[w] = GetBits(1);
            if (ti->numFilt[w]) {
                ti->coefRes[w] = GetBits(1) + 3;
                *filtLength =    GetBits(4);
                *filtOrder =     GetBits(3);
                if (*filtOrder) {
                    *filtDir++ =      GetBits(1);
                    compress =        GetBits(1);
                    coefBits = (int)ti->coefRes[w] - compress;    /* 2, 3, or 4 */
                    s = sgnMask[coefBits - 2];
                    n = negMask[coefBits - 2];
                    for (i = 0; i < *filtOrder; i++) {
                        c = GetBits(coefBits);
                        if (c & s)    c |= n;
                        *tnsCoef++ = c;
                    }
                }
                filtLength++;
                filtOrder++;
            }
        }
    } else {
        /* long blocks */
        ti->numFilt[0] = GetBits(2);
        if (ti->numFilt[0])
            ti->coefRes[0] = GetBits(1) + 3;
        for (f = 0; f < ti->numFilt[0]; f++) {
            *filtLength =      GetBits(6);
            *filtOrder =       GetBits(5);
            if (*filtOrder) {
                *filtDir++ =     GetBits(1);
                compress =       GetBits(1);
                coefBits = (int)ti->coefRes[0] - compress;    /* 2, 3, or 4 */
                s = sgnMask[coefBits - 2];
                n = negMask[coefBits - 2];
                for (i = 0; i < *filtOrder; i++) {
                    c = GetBits(coefBits);
                    if (c & s)    c |= n;
                    *tnsCoef++ = c;
                }
            }
            filtLength++;
            filtOrder++;
        }
    }
}

/* bitstream field lengths for gain control data:
 *   gainBits[winSequence][0] = maxWindow (how many gain windows there are)
 *   gainBits[winSequence][1] = locBitsZero (bits for alocCode if window == 0)
 *   gainBits[winSequence][2] = locBits (bits for alocCode if window != 0)
 */
static const uint8_t gainBits[4][3] = {
    {1, 5, 5},  /* long */
    {2, 4, 2},  /* start */
    {8, 2, 2},  /* short */
    {2, 4, 5},  /* stop */
};

/***********************************************************************************************************************
 * Function:    DecodeGainControlInfo
 *
 * Description: decode gain control information (SSR profile only)
 *
 * Inputs:      window sequence (short or long blocks)
 *
 * Outputs:     updated GainControlInfo_t struct
 *
 * Return:      none
 **********************************************************************************************************************/
void DecodeGainControlInfo(int winSequence, GainControlInfo_t *gi)
{
    int bd, wd, ad;
    int locBits, locBitsZero, maxWin;

    gi->maxBand = GetBits(2);
    maxWin =      (int)gainBits[winSequence][0];
    locBitsZero = (int)gainBits[winSequence][1];
    locBits =     (int)gainBits[winSequence][2];

    for (bd = 1; bd <= gi->maxBand; bd++) {
        for (wd = 0; wd < maxWin; wd++) {
            gi->adjNum[bd][wd] = GetBits(3);
            for (ad = 0; ad < gi->adjNum[bd][wd]; ad++) {
                gi->alevCode[bd][wd][ad] = GetBits(4);
                gi->alocCode[bd][wd][ad] = GetBits(wd == 0 ? locBitsZero : locBits);
            }
        }
    }
}

/***********************************************************************************************************************
 * Function:    DecodeICS
 *
 * Description: decode individual channel stream
 *
 * Inputs:      index of current channel
 *
 * Outputs:     updated section data, scale factor data, pulse data, TNS data,
 *                and gain control data
 *
 * Return:      none
 **********************************************************************************************************************/
void DecodeICS(int ch)
{
    int globalGain;
    ICSInfo_t *icsInfo;
    TNSInfo_t *ti;
    GainControlInfo_t *gi;

    icsInfo = (ch == 1 && m_PSInfoBase->commonWin == 1) ? &(m_PSInfoBase->icsInfo[0]) : &(m_PSInfoBase->icsInfo[ch]);

    globalGain = GetBits(8);
    if (!m_PSInfoBase->commonWin)
        DecodeICSInfo(icsInfo, m_PSInfoBase->sampRateIdx);

    DecodeSectionData(icsInfo->winSequence, icsInfo->numWinGroup, icsInfo->maxSFB, m_PSInfoBase->sfbCodeBook[ch]);

    DecodeScaleFactors(icsInfo->numWinGroup, icsInfo->maxSFB, globalGain, m_PSInfoBase->sfbCodeBook[ch],
                                                                                        m_PSInfoBase->scaleFactors[ch]);

    m_pulseInfo[ch].pulseDataPresent = GetBits(1);
    if (m_pulseInfo[ch].pulseDataPresent)
        DecodePulseInfo(ch);

    ti = &m_PSInfoBase->tnsInfo[ch];
    ti->tnsDataPresent = GetBits(1);
    if (ti->tnsDataPresent)
        DecodeTNSInfo(icsInfo->winSequence, ti, ti->coef);

    gi = &m_PSInfoBase->gainControlInfo[ch];
    gi->gainControlDataPresent = GetBits(1);
    if (gi->gainControlDataPresent)
        DecodeGainControlInfo(icsInfo->winSequence, gi);
}

/***********************************************************************************************************************
 * Function:    DecodeNoiselessData
 *
 * Description: decode noiseless data (side info and transform coefficients)
 *
 * Inputs:      double pointer to buffer pointing to start of individual channel stream
 *                (14496-3, table 4.4.24)
 *              pointer to bit offset
 *              pointer to number of valid bits remaining in buf
 *              index of current channel
 *
 * Outputs:     updated global gain, section data, scale factor data, pulse data,
 *                TNS data, gain control data, and spectral data
 *
 * Return:      0 if successful, error code (< 0) if error
 **********************************************************************************************************************/
int DecodeNoiselessData(uint8_t **buf, int *bitOffset, int *bitsAvail, int ch)
{
    int bitsUsed;
    ICSInfo_t *icsInfo;

    icsInfo = (ch == 1 && m_PSInfoBase->commonWin == 1) ? &(m_PSInfoBase->icsInfo[0]) : &(m_PSInfoBase->icsInfo[ch]);

    SetBitstreamPointer((*bitsAvail+7) >> 3, *buf);
    GetBits(*bitOffset);

    DecodeICS(ch);

    if (icsInfo->winSequence == 2)
        DecodeSpectrumShort(ch);
    else
        DecodeSpectrumLong(ch);

    bitsUsed = CalcBitsUsed(*buf, *bitOffset);
    *buf += ((bitsUsed + *bitOffset) >> 3);
    *bitOffset = ((bitsUsed + *bitOffset) & 0x07);
    *bitsAvail -= bitsUsed;

    m_AACDecInfo->sbDeinterleaveReqd[ch] = 0;
    m_AACDecInfo->tnsUsed |= m_PSInfoBase->tnsInfo[ch].tnsDataPresent;    /* set flag if TNS used for any channel */

    return ERR_AAC_NONE;
}
/***********************************************************************************************************************
 * Function:    DecodeHuffmanScalar
 *
 * Description: decode one Huffman symbol from bitstream
 *
 * Inputs:      pointers to Huffman table and info struct
 *              left-aligned bit buffer with >= huffTabInfo->maxBits bits
 *
 * Outputs:     decoded symbol in *val
 *
 * Return:      number of bits in symbol
 *
 * Notes:       assumes canonical Huffman codes:
 *                first CW always 0, we have "count" CW's of length "nBits" bits
 *                starting CW for codes of length nBits+1 =
 *                  (startCW[nBits] + count[nBits]) << 1
 *                if there are no codes at nBits, then we just keep << 1 each time
 *                  (since count[nBits] = 0)
 **********************************************************************************************************************/
int DecodeHuffmanScalar(const signed short *huffTab, const HuffInfo_t *huffTabInfo, uint32_t bitBuf, int32_t *val)
{
    uint32_t count, start, shift, t;
    const uint8_t *countPtr;
    const signed short *map;

    map = huffTab + huffTabInfo->offset;
    countPtr = huffTabInfo->count;

    start = 0;
    count = 0;
    shift = 32;
    do {
        start += count;
        start <<= 1;
        map += count;
        count = *countPtr++;
        shift--;
        t = (bitBuf >> shift) - start;
    } while (t >= count);

    *val = (int32_t)map[t];
    return (countPtr - huffTabInfo->count);
}

/***********************************************************************************************************************
* Function:    UnpackADTSHeader
*
* Description: parse the ADTS frame header and initialize decoder state, Audio Data Transport Stream
*
* Inputs:      double pointer to buffer with complete ADTS frame header (byte aligned)
*                header size = 7 bytes, plus 2 if CRC
*
* Outputs:     filled in ADTS struct
*              updated buffer pointer
*              updated bit offset
*              updated number of available bits
*
* Return:      0 if successful, error code (< 0) if error
*              verify that fixed fields don't change between frames
***********************************************************************************************************************/
int UnpackADTSHeader(uint8_t **buf, int *bitOffset, int *bitsAvail)
{
    int bitsUsed;

    /* init bitstream reader */
    SetBitstreamPointer((*bitsAvail + 7) >> 3, *buf);
    GetBits(*bitOffset);

    /* verify that first 12 bits of header are syncword */
    if (GetBits(12) != 0x0fff) {
        return ERR_AAC_INVALID_ADTS_HEADER;
    }

    /* fixed fields - should not change from frame to frame */
    m_fhADTS.id =               GetBits(1);
    m_fhADTS.layer =            GetBits(2);
    m_fhADTS.protectBit =       GetBits(1);
    m_fhADTS.profile =          GetBits(2);
    m_fhADTS.sampRateIdx =      GetBits(4);
    m_fhADTS.privateBit =       GetBits(1);
    m_fhADTS.channelConfig =    GetBits(3);
    m_fhADTS.origCopy =         GetBits(1);
    m_fhADTS.home =             GetBits(1);

    /* variable fields - can change from frame to frame */
    m_fhADTS.copyBit =          GetBits(1);
    m_fhADTS.copyStart =        GetBits(1);
    m_fhADTS.frameLength =      GetBits(13);
    m_fhADTS.bufferFull =       GetBits(11);
    m_fhADTS.numRawDataBlocks = GetBits(2) + 1;

    /* note - MPEG4 spec, correction 1 changes how CRC is handled when protectBit == 0 and numRawDataBlocks > 1 */
    if (m_fhADTS.protectBit == 0)
        m_fhADTS.crcCheckWord = GetBits(16);

    /* byte align */
    ByteAlignBitstream();    /* should always be aligned anyway */

    /* check validity of header */
    if (m_fhADTS.layer != 0 || m_fhADTS.profile != AAC_PROFILE_LC ||
        m_fhADTS.sampRateIdx >= NUM_SAMPLE_RATES || m_fhADTS.channelConfig >= NUM_DEF_CHAN_MAPS)
        return ERR_AAC_INVALID_ADTS_HEADER;

#ifndef AAC_ENABLE_MPEG4
    if (m_fhADTS.id != 1)
        return ERR_AAC_MPEG4_UNSUPPORTED;
#endif


    /* update codec info */
    m_PSInfoBase->sampRateIdx = m_fhADTS.sampRateIdx;
    if (!m_PSInfoBase->useImpChanMap)
        m_PSInfoBase->nChans = channelMapTab[m_fhADTS.channelConfig];

    /* syntactic element fields will be read from bitstream for each element */
    m_AACDecInfo->prevBlockID = AAC_ID_INVALID;
    m_AACDecInfo->currBlockID = AAC_ID_INVALID;
    m_AACDecInfo->currInstTag = -1;

    /* fill in user-accessible data */
    m_AACDecInfo->bitRate = 0;
    m_AACDecInfo->nChans = m_PSInfoBase->nChans;
    m_AACDecInfo->sampRate = sampRateTab[m_PSInfoBase->sampRateIdx];
    m_AACDecInfo->id = m_fhADTS.id;
    m_AACDecInfo->profile = m_fhADTS.profile;
    m_AACDecInfo->sbrEnabled = 0;
    m_AACDecInfo->adtsBlocksLeft = m_fhADTS.numRawDataBlocks;

    /* update bitstream reader */
    bitsUsed = CalcBitsUsed(*buf, *bitOffset);
    *buf += (bitsUsed + *bitOffset) >> 3;
    *bitOffset = (bitsUsed + *bitOffset) & 0x07;
    *bitsAvail -= bitsUsed ;
    if (*bitsAvail < 0)
        return ERR_AAC_INDATA_UNDERFLOW;

    return ERR_AAC_NONE;
}

/***********************************************************************************************************************
* Function:    GetADTSChannelMapping
*
* Description: determine the number of channels from implicit mapping rules
*
* Inputs:      pointer to start of raw_data_block
*              bit offset
*              bits available
*
* Outputs:     updated number of channels
*
* Return:      0 if successful, error code (< 0) if error
*
* Notes:       calculates total number of channels using rules in 14496-3, 4.5.1.2.1
*              does not attempt to deduce speaker geometry
***********************************************************************************************************************/
int GetADTSChannelMapping(uint8_t *buf, int bitOffset, int bitsAvail)
{
    int ch, nChans, elementChans, err;

    nChans = 0;
    do {
        /* parse next syntactic element */
        err = DecodeNextElement(&buf, &bitOffset, &bitsAvail);
        if (err)
            return err;

        elementChans = elementNumChans[m_AACDecInfo->currBlockID];
        nChans += elementChans;

        for (ch = 0; ch < elementChans; ch++) {
            err = DecodeNoiselessData(&buf, &bitOffset, &bitsAvail, ch);
            if (err)
                return err;
        }
    } while (m_AACDecInfo->currBlockID != AAC_ID_END);

    if (nChans <= 0)
        return ERR_AAC_CHANNEL_MAP;

    /* update number of channels in codec state and user-accessible info structs */
    m_PSInfoBase->nChans = nChans;
    m_AACDecInfo->nChans = m_PSInfoBase->nChans;
    m_PSInfoBase->useImpChanMap = 1;

    return ERR_AAC_NONE;
}

/***********************************************************************************************************************
* Function:    GetNumChannelsADIF
*
* Description: get number of channels from program config elements in an ADIF file
*
* Inputs:      array of filled-in program config element structures
*              number of PCE's
*
* Outputs:     none
*
* Return:      total number of channels in file
*              -1 if error (invalid number of PCE's or unsupported mode)
***********************************************************************************************************************/
int GetNumChannelsADIF(int nPCE)
{
    int i, j, nChans;

    if (nPCE < 1 || nPCE > MAX_NUM_PCE_ADIF)
        return -1;

    nChans = 0;
    for (i = 0; i < nPCE; i++) {
        /* for now: only support LC, no channel coupling */
        if (m_pce[i]->profile != AAC_PROFILE_LC || m_pce[i]->numCCE > 0)
            return -1;

        /* add up number of channels in all channel elements (assume all single-channel) */
       nChans += m_pce[i]->numFCE;
       nChans += m_pce[i]->numSCE;
       nChans += m_pce[i]->numBCE;
       nChans += m_pce[i]->numLCE;

        /* add one more for every element which is a channel pair */
       for (j = 0; j < m_pce[i]->numFCE; j++) {
           if ((m_pce[i]->fce[j] & 0x10) >> 4)  /* bit 4 = SCE/CPE flag */
               nChans++;
       }
       for (j = 0; j < m_pce[i]->numSCE; j++) {
           if ((m_pce[i]->sce[j] & 0x10) >> 4)  /* bit 4 = SCE/CPE flag */
               nChans++;
       }
       for (j = 0; j < m_pce[i]->numBCE; j++) {
           if ((m_pce[i]->bce[j] & 0x10) >> 4)  /* bit 4 = SCE/CPE flag */
               nChans++;
       }

    }

    return nChans;
}

/***********************************************************************************************************************
* Function:    GetSampleRateIdxADIF
*
* Description: get sampling rate index from program config elements in an ADIF file
*
* Inputs:      array of filled-in program config element structures
*              number of PCE's
*
* Outputs:     none
*
* Return:      sample rate of file
*              -1 if error (invalid number of PCE's or sample rate mismatch)
***********************************************************************************************************************/
int GetSampleRateIdxADIF(int nPCE)
{
    int i, idx;

    if (nPCE < 1 || nPCE > MAX_NUM_PCE_ADIF)
        return -1;

    /* make sure all PCE's have the same sample rate */
    idx = m_pce[0]->sampRateIdx;
    for (i = 1; i < nPCE; i++) {
        if (m_pce[i]->sampRateIdx != idx)
            return -1;
    }

    return idx;
}

/***********************************************************************************************************************
* Function:    UnpackADIFHeader
*
* Description: parse the ADIF file header and initialize decoder state
*
* Inputs:      double pointer to buffer with complete ADIF header
*                (starting at 'A' in 'ADIF' tag)
*              pointer to bit offset
*              pointer to number of valid bits remaining in inbuf
*
* Outputs:     filled-in ADIF struct
*              updated buffer pointer
*              updated bit offset
*              updated number of available bits
*
* Return:      0 if successful, error code (< 0) if error
***********************************************************************************************************************/
int UnpackADIFHeader(uint8_t **buf, int *bitOffset, int *bitsAvail)
{
    uint8_t i;
    int bitsUsed;

    /* init bitstream reader */
    SetBitstreamPointer((*bitsAvail + 7) >> 3, *buf);
    GetBits(*bitOffset);

    /* verify that first 32 bits of header are "ADIF" */
    if (GetBits(8) != 'A' || GetBits(8) != 'D' || GetBits(8) != 'I' || GetBits(8) != 'F')
        return ERR_AAC_INVALID_ADIF_HEADER;

    /* read ADIF header fields */
    m_fhADIF.copyBit = GetBits(1);
    if (m_fhADIF.copyBit) {
        for (i = 0; i < ADIF_COPYID_SIZE; i++)
            m_fhADIF.copyID[i] = GetBits(8);
    }
    m_fhADIF.origCopy = GetBits(1);
    m_fhADIF.home =     GetBits(1);
    m_fhADIF.bsType =   GetBits(1);
    m_fhADIF.bitRate =  GetBits(23);
    m_fhADIF.numPCE =   GetBits(4) + 1;    /* add 1 (so range = [1, 16]) */
    if (m_fhADIF.bsType == 0)
        m_fhADIF.bufferFull = GetBits(20);

    /* parse all program config elements */
    for (i = 0; i < m_fhADIF.numPCE; i++)
        DecodeProgramConfigElement(i);

    /* byte align */
    ByteAlignBitstream();

    /* update codec info */
    m_PSInfoBase->nChans = GetNumChannelsADIF(m_fhADIF.numPCE);
    m_PSInfoBase->sampRateIdx = GetSampleRateIdxADIF(m_fhADIF.numPCE);

    /* check validity of header */
    if (m_PSInfoBase->nChans < 0 || m_PSInfoBase->sampRateIdx < 0 || m_PSInfoBase->sampRateIdx >= NUM_SAMPLE_RATES)
        return ERR_AAC_INVALID_ADIF_HEADER;

    /* syntactic element fields will be read from bitstream for each element */
    m_AACDecInfo->prevBlockID = AAC_ID_INVALID;
    m_AACDecInfo->currBlockID = AAC_ID_INVALID;
    m_AACDecInfo->currInstTag = -1;

    /* fill in user-accessible data */
    m_AACDecInfo->bitRate = 0;
    m_AACDecInfo->nChans = m_PSInfoBase->nChans;
    m_AACDecInfo->sampRate = sampRateTab[m_PSInfoBase->sampRateIdx];
    m_AACDecInfo->profile = m_pce[0]->profile;
    m_AACDecInfo->sbrEnabled = 0;

    /* update bitstream reader */
    bitsUsed = CalcBitsUsed(*buf, *bitOffset);
    *buf += (bitsUsed + *bitOffset) >> 3;
    *bitOffset = (bitsUsed + *bitOffset) & 0x07;
    *bitsAvail -= bitsUsed ;
    if (*bitsAvail < 0)
        return ERR_AAC_INDATA_UNDERFLOW;

    return ERR_AAC_NONE;
}

/***********************************************************************************************************************
* Function:    SetRawBlockParams
*
* Description: set internal state variables for decoding a stream of raw data blocks
*
* Inputs:      flag indicating source of parameters (from previous headers or passed
*                explicitly by caller)
*              number of channels
*              sample rate
*              profile ID
*
* Outputs:     updated state variables in aacDecInfo
*
* Return:      0 if successful, error code (< 0) if error
*
* Notes:       if copyLast == 1, then m_PSInfoBase->nChans, m_PSInfoBase->sampRateIdx, and
*                aacDecInfo->profile are not changed (it's assumed that we already
*                set them, such as by a previous call to UnpackADTSHeader())
*              if copyLast == 0, then the parameters we passed in are used instead
***********************************************************************************************************************/
int SetRawBlockParams(int copyLast, int nChans, int sampRate, int profile)
{
    int idx;

    if (!copyLast) {
        m_AACDecInfo->profile = profile;
        m_PSInfoBase->nChans = nChans;
        for (idx = 0; idx < NUM_SAMPLE_RATES; idx++) {
            if (sampRate == sampRateTab[idx]) {
                m_PSInfoBase->sampRateIdx = idx;
                break;
            }
        }
        if (idx == NUM_SAMPLE_RATES)
            return ERR_AAC_INVALID_FRAME;
    }
    m_AACDecInfo->nChans = m_PSInfoBase->nChans;
    m_AACDecInfo->sampRate = sampRateTab[m_PSInfoBase->sampRateIdx];

    /* check validity of header */
    if (m_PSInfoBase->sampRateIdx >= NUM_SAMPLE_RATES || m_PSInfoBase->sampRateIdx < 0 ||
        m_AACDecInfo->profile != AAC_PROFILE_LC)
        return ERR_AAC_RAWBLOCK_PARAMS;

    return ERR_AAC_NONE;
}

/***********************************************************************************************************************
* Function:    PrepareRawBlock
*
* Description: reset per-block state variables for raw blocks (no ADTS/ADIF headers)
*
* Inputs:      none
*
* Outputs:     updated state variables in aacDecInfo
*
* Return:      0 if successful, error code (< 0) if error
***********************************************************************************************************************/
int PrepareRawBlock()
{
    /* syntactic element fields will be read from bitstream for each element */
    m_AACDecInfo->prevBlockID = AAC_ID_INVALID;
    m_AACDecInfo->currBlockID = AAC_ID_INVALID;
    m_AACDecInfo->currInstTag = -1;

    /* fill in user-accessible data */
    m_AACDecInfo->bitRate = 0;
    m_AACDecInfo->sbrEnabled = 0;

    return ERR_AAC_NONE;
}

/***********************************************************************************************************************
 * Function:    DequantBlock
 *
 * Description: dequantize one block of transform coefficients (in-place)
 *
 * Inputs:      quantized transform coefficients, range = [0, 8191]
 *              number of samples to dequantize
 *              scalefactor for this block of data, range = [0, 256]
 *
 * Outputs:     dequantized transform coefficients in Q(FBITS_OUT_DQ_OFF)
 *
 * Return:      guard bit mask (OR of abs value of all dequantized coefs)
 *
 * Notes:       applies dequant formula y = pow(x, 4.0/3.0) * pow(2, (scale - 100)/4.0)
 *                * pow(2, FBITS_OUT_DQ_OFF)
 *              clips outputs to Q(FBITS_OUT_DQ_OFF)
 *              output has no minimum number of guard bits
 **********************************************************************************************************************/
int DequantBlock(int *inbuf, int nSamps, int scale)
{
    int iSamp, scalef, scalei, x, y, gbMask, shift, tab4[4];
    const uint32_t *tab16, *coef;

    if (nSamps <= 0)
        return 0;

    scale -= SF_OFFSET;    /* new range = [-100, 156] */

    /* with two's complement numbers, scalei/scalef factorization works for pos and neg values of scale:
     *  [+4...+7] >> 2 = +1, [ 0...+3] >> 2 = 0, [-4...-1] >> 2 = -1, [-8...-5] >> 2 = -2 ...
     *  (-1 & 0x3) = 3, (-2 & 0x3) = 2, (-3 & 0x3) = 1, (0 & 0x3) = 0
     *
     * Example: 2^(-5/4) = 2^(-1) * 2^(-1/4) = 2^-2 * 2^(3/4)
     */
    tab16 = pow43_14[scale & 0x3];
    scalef = pow14[scale & 0x3];
    scalei = (scale >> 2) + FBITS_OUT_DQ_OFF;

    /* cache first 4 values:
     * tab16[j] = Q28 for j = [0,3]
     * tab4[x] = x^(4.0/3.0) * 2^(0.25*scale), Q(FBITS_OUT_DQ_OFF)
     */
    shift = 28 - scalei;
    if (shift > 31) {
        tab4[0] = tab4[1] = tab4[2] = tab4[3] = 0;
    } else if (shift <= 0) {
        shift = -shift;
        if (shift > 31)
            shift = 31;
        for (x = 0; x < 4; x++) {
            y = tab16[x];
            if (y > (0x7fffffff >> shift))
                y = 0x7fffffff;        /* clip (rare) */
            else
                y <<= shift;
            tab4[x] = y;
        }
    } else {
        tab4[0] = 0;
        tab4[1] = tab16[1] >> shift;
        tab4[2] = tab16[2] >> shift;
        tab4[3] = tab16[3] >> shift;
    }

    gbMask = 0;
    do {
        iSamp = *inbuf;
        x = FASTABS(iSamp);

        if (x < 4) {
            y = tab4[x];
        } else  {

            if (x < 16) {
                /* result: y = Q25 (tab16 = Q25) */
                y = tab16[x];
                shift = 25 - scalei;
            } else if (x < 64) {
                /* result: y = Q21 (pow43tab[j] = Q23, scalef = Q30) */
                y = pow43[x-16];
                shift = 21 - scalei;
                y = MULSHIFT32(y, scalef);
            } else {
                /* normalize to [0x40000000, 0x7fffffff]
                 * input x = [64, 8191] = [64, 2^13-1]
                 * ranges:
                 *  shift = 7:   64 -  127
                 *  shift = 6:  128 -  255
                 *  shift = 5:  256 -  511
                 *  shift = 4:  512 - 1023
                 *  shift = 3: 1024 - 2047
                 *  shift = 2: 2048 - 4095
                 *  shift = 1: 4096 - 8191
                 */
                x <<= 17;
                shift = 0;
                if (x < 0x08000000)
                    x <<= 4, shift += 4;
                if (x < 0x20000000)
                    x <<= 2, shift += 2;
                if (x < 0x40000000)
                    x <<= 1, shift += 1;

                coef = (x < SQRTHALF) ? poly43lo : poly43hi;

                /* polynomial */
                y = coef[0];
                y = MULSHIFT32(y, x) + coef[1];
                y = MULSHIFT32(y, x) + coef[2];
                y = MULSHIFT32(y, x) + coef[3];
                y = MULSHIFT32(y, x) + coef[4];
                y = MULSHIFT32(y, pow2frac[shift]) << 3;

                /* fractional scale
                 * result: y = Q21 (pow43tab[j] = Q23, scalef = Q30)
                 */
                y = MULSHIFT32(y, scalef);    /* now y is Q24 */
                shift = 24 - scalei - pow2exp[shift];
            }

            /* integer scale */
            if (shift <= 0) {
                shift = -shift;
                if (shift > 31)
                    shift = 31;

                if (y > (0x7fffffff >> shift))
                    y = 0x7fffffff;        /* clip (rare) */
                else
                    y <<= shift;
            } else {
                if (shift > 31)
                    shift = 31;
                y >>= shift;
            }
        }

        /* sign and store (gbMask used to count GB's) */
        gbMask |= y;

        /* apply sign */
        iSamp >>= 31;
        y ^= iSamp;
        y -= iSamp;

        *inbuf++ = y;
    } while (--nSamps);

    return gbMask;
}

/***********************************************************************************************************************
 * Function:    AACDequantize
 *
 * Description: dequantize all transform coefficients for one channel
 *
 * Inputs:      index of current channel
 *
 * Outputs:     dequantized coefficients, including short-block deinterleaving
 *              flags indicating if intensity and/or PNS is active
 *              minimum guard bit count for dequantized coefficients
 *
 * Return:      0 if successful, error code (< 0) if error
 **********************************************************************************************************************/
int AACDequantize(int ch)
{
    int gp, cb, sfb, win, width, nSamps, gbMask;
    int *coef;
    const uint16_t *sfbTab;
    uint8_t *sfbCodeBook;
    short *scaleFactors;
    ICSInfo_t *icsInfo;

    icsInfo = (ch == 1 && m_PSInfoBase->commonWin == 1) ? &(m_PSInfoBase->icsInfo[0]) : &(m_PSInfoBase->icsInfo[ch]);

    if (icsInfo->winSequence == 2) {
        sfbTab = sfBandTabShort + sfBandTabShortOffset[m_PSInfoBase->sampRateIdx];
        nSamps = NSAMPS_SHORT;
    } else {
        sfbTab = sfBandTabLong + sfBandTabLongOffset[m_PSInfoBase->sampRateIdx];
        nSamps = NSAMPS_LONG;
    }
    coef = m_PSInfoBase->coef[ch];
    sfbCodeBook = m_PSInfoBase->sfbCodeBook[ch];
    scaleFactors = m_PSInfoBase->scaleFactors[ch];

    m_PSInfoBase->intensityUsed[ch] = 0;
    m_PSInfoBase->pnsUsed[ch] = 0;
    gbMask = 0;
    for (gp = 0; gp < icsInfo->numWinGroup; gp++) {
        for (win = 0; win < icsInfo->winGroupLen[gp]; win++) {
            for (sfb = 0; sfb < icsInfo->maxSFB; sfb++) {
                /* dequantize one scalefactor band (not necessary if codebook is intensity or PNS)
                 * for zero codebook, still run dequantizer in case non-zero pulse data was added
                 */
                cb = (int)(sfbCodeBook[sfb]);
                width = sfbTab[sfb+1] - sfbTab[sfb];
                if (cb >= 0 && cb <= 11)
                    gbMask |= DequantBlock(coef, width, scaleFactors[sfb]);
                else if (cb == 13)
                    m_PSInfoBase->pnsUsed[ch] = 1;
                else if (cb == 14 || cb == 15)
                    m_PSInfoBase->intensityUsed[ch] = 1;    /* should only happen if ch == 1 */
                coef += width;
            }
            coef += (nSamps - sfbTab[icsInfo->maxSFB]);
        }
        sfbCodeBook += icsInfo->maxSFB;
        scaleFactors += icsInfo->maxSFB;
    }
    m_AACDecInfo->pnsUsed |= m_PSInfoBase->pnsUsed[ch];    /* set flag if PNS used for any channel */

    /* calculate number of guard bits in dequantized data */
    m_PSInfoBase->gbCurrent[ch] = CLZ(gbMask) - 1;

    return ERR_AAC_NONE;
}

/***********************************************************************************************************************
 * Function:    DeinterleaveShortBlocks
 *
 * Description: deinterleave transform coefficients in short blocks for one channel
 *
 * Inputs:      index of current channel
 *
 * Outputs:     deinterleaved coefficients (window groups into 8 separate windows)
 *
 * Return:      0 if successful, error code (< 0) if error
 *
 * Notes:       only necessary if deinterleaving not part of Huffman decoding
 **********************************************************************************************************************/
int DeinterleaveShortBlocks(int ch)
{
//    (void)aacDecInfo;
//    (void)ch;
    /* not used for this implementation - short block deinterleaving performed during Huffman decoding */
    return ERR_AAC_NONE;
}

/***********************************************************************************************************************
 * Function:    Get32BitVal
 *
 * Description: generate 32-bit unsigned random number
 *
 * Inputs:      last number calculated (seed, first time through)
 *
 * Outputs:     new number, saved in *last
 *
 * Return:      32-bit number, uniformly distributed between [0, 2^32)
 *
 * Notes:       uses simple linear congruential generator
 **********************************************************************************************************************/
unsigned int Get32BitVal(unsigned int *last)
{
    uint32_t r = *last;

    /* use same coefs as MPEG reference code (classic LCG)
     * use unsigned multiply to force reliable wraparound behavior in C (mod 2^32)
     */
    r = (1664525U * r) + 1013904223U;
    *last = r;

    return r;
}

/***********************************************************************************************************************
 * Function:    InvRootR
 *
 * Description: use Newton's method to solve for x = 1/sqrt(r)
 *
 * Inputs:      r in Q30 format, range = [0.25, 1] (normalize inputs to this range)
 *
 * Outputs:     none
 *
 * Return:      x = Q29, range = (1, 2)
 *
 * Notes:       guaranteed to converge and not overflow for any r in this range
 *
 *              xn+1  = xn - f(xn)/f'(xn)
 *              f(x)  = 1/sqrt(r) - x = 0 (find root)
 *                    = 1/x^2 - r
 *              f'(x) = -2/x^3
 *
 *              so xn+1 = xn/2 * (3 - r*xn^2)
 *
 *              NUM_ITER_INVSQRT = 3, maxDiff = 1.3747e-02
 *              NUM_ITER_INVSQRT = 4, maxDiff = 3.9832e-04
 **********************************************************************************************************************/
int InvRootR(int r)
{
    int i, xn, t;

    /* use linear equation for initial guess
     * x0 = -2*r + 3 (so x0 always >= correct answer in range [0.25, 1))
     * xn = Q29 (at every step)
     */
    xn = (MULSHIFT32(r, X0_COEF_2) << 2) + X0_OFF_2;

    for (i = 0; i < NUM_ITER_INVSQRT; i++) {
        t = MULSHIFT32(xn, xn);                    /* Q26 = Q29*Q29 */
        t = Q26_3 - (MULSHIFT32(r, t) << 2);    /* Q26 = Q26 - (Q31*Q26 << 1) */
        xn = MULSHIFT32(xn, t) << (6 - 1);        /* Q29 = (Q29*Q26 << 6), and -1 for division by 2 */
    }

    /* clip to range (1.0, 2.0)
     * (because of rounding, this can converge to xn slightly > 2.0 when r is near 0.25)
     */
    if (xn >> 30)
        xn = (1 << 30) - 1;

    return xn;
}

/***********************************************************************************************************************
 * Function:    ScaleNoiseVector
 *
 * Description: apply scaling to vector of noise coefficients for one scalefactor band
 *
 * Inputs:      unscaled coefficients
 *              number of coefficients in vector (one scalefactor band of coefs)
 *              scalefactor for this band (i.e. noise energy)
 *
 * Outputs:     nVals coefficients in Q(FBITS_OUT_DQ_OFF)
 *
 * Return:      guard bit mask (OR of abs value of all noise coefs)
 **********************************************************************************************************************/
int ScaleNoiseVector(int *coef, int nVals, int sf)
{

/* pow(2, i/4.0) for i = [0,1,2,3], format = Q30 */
static const int pow14[4] PROGMEM = {
    0x40000000, 0x4c1bf829, 0x5a82799a, 0x6ba27e65
};

    int i, c, spec, energy, sq, scalef, scalei, invSqrtEnergy, z, gbMask;

    energy = 0;
    for (i = 0; i < nVals; i++) {
        spec = coef[i];

        /* max nVals = max SFB width = 96, so energy can gain < 2^7 bits in accumulation */
        sq = (spec * spec) >> 8;        /* spec*spec range = (-2^30, 2^30) */
        energy += sq;
    }

    /* unless nVals == 1 (or the number generator is broken...), this should not happen */
    if (energy == 0)
        return 0;    /* coef[i] must = 0 for i = [0, nVals-1], so gbMask = 0 */

    /* pow(2, sf/4) * pow(2, FBITS_OUT_DQ_OFF) */
    scalef = pow14[sf & 0x3];
    scalei = (sf >> 2) + FBITS_OUT_DQ_OFF;

    /* energy has implied factor of 2^-8 since we shifted the accumulator
     * normalize energy to range [0.25, 1.0), calculate 1/sqrt(1), and denormalize
     *   i.e. divide input by 2^(30-z) and convert to Q30
     *        output of 1/sqrt(i) now has extra factor of 2^((30-z)/2)
     *        for energy > 0, z is an even number between 0 and 28
     * final scaling of invSqrtEnergy:
     *  2^(15 - z/2) to compensate for implicit 2^(30-z) factor in input
     *  +4 to compensate for implicit 2^-8 factor in input
     */
    z = CLZ(energy) - 2;                    /* energy has at least 2 leading zeros (see acc loop) */
    z &= 0xfffffffe;                        /* force even */
    invSqrtEnergy = InvRootR(energy << z);    /* energy << z must be in range [0x10000000, 0x40000000] */
    scalei -= (15 - z/2 + 4);                /* nInt = 1/sqrt(energy) in Q29 */

    /* normalize for final scaling */
    z = CLZ(invSqrtEnergy) - 1;
    invSqrtEnergy <<= z;
    scalei -= (z - 3 - 2);    /* -2 for scalef, z-3 for invSqrtEnergy */
    scalef = MULSHIFT32(scalef, invSqrtEnergy);    /* scalef (input) = Q30, invSqrtEnergy = Q29 * 2^z */
    gbMask = 0;

    if (scalei < 0) {
        scalei = -scalei;
        if (scalei > 31)
            scalei = 31;
        for (i = 0; i < nVals; i++) {
            c = MULSHIFT32(coef[i], scalef) >> scalei;
            gbMask |= FASTABS(c);
            coef[i] = c;
        }
    } else {
        /* for scalei <= 16, no clipping possible (coef[i] is < 2^15 before scaling)
         * for scalei > 16, just saturate exponent (rare)
         *   scalef is close to full-scale (since we normalized invSqrtEnergy)
         * remember, we are just producing noise here
         */
        if (scalei > 16)
            scalei = 16;
        for (i = 0; i < nVals; i++) {
            c = MULSHIFT32(coef[i] << scalei, scalef);
            coef[i] = c;
            gbMask |= FASTABS(c);
        }
    }

    return gbMask;
}

/***********************************************************************************************************************
 * Function:    GenerateNoiseVector
 *
 * Description: create vector of noise coefficients for one scalefactor band
 *
 * Inputs:      seed for number generator
 *              number of coefficients to generate
 *
 * Outputs:     buffer of nVals coefficients, range = [-2^15, 2^15)
 *              updated seed for number generator
 *
 * Return:      none
 **********************************************************************************************************************/
void GenerateNoiseVector(int *coef, int *last, int nVals)
{
    int i;

    for (i = 0; i < nVals; i++)
        coef[i] = ((int32_t)Get32BitVal((uint32_t *)last)) >> 16;
}

/***********************************************************************************************************************
 * Function:    CopyNoiseVector
 *
 * Description: copy vector of noise coefficients for one scalefactor band from L to R
 *
 * Inputs:      buffer of left coefficients
 *              number of coefficients to copy
 *
 * Outputs:     buffer of right coefficients
 *
 * Return:      none
 **********************************************************************************************************************/
void CopyNoiseVector(int *coefL, int *coefR, int nVals)
{
    int i;

    for (i = 0; i < nVals; i++)
        coefR[i] = coefL[i];
}

/***********************************************************************************************************************
 * Function:    PNS
 *
 * Description: apply perceptual noise substitution, if enabled (MPEG-4 only)
 *
 * Inputs:      index of current channel
 *
 * Outputs:     shaped noise in scalefactor bands where PNS is active
 *              updated minimum guard bit count for this channel
 *
 * Return:      0 if successful, -1 if error
 **********************************************************************************************************************/
int PNS(int ch)
{
    int gp, sfb, win, width, nSamps, gb, gbMask;
    int *coef;
    const uint16_t *sfbTab;
    uint8_t *sfbCodeBook;
    short *scaleFactors;
    int msMaskOffset, checkCorr, genNew;
    uint8_t msMask;
    uint8_t *msMaskPtr;
    ICSInfo_t *icsInfo;

    icsInfo = (ch == 1 && m_PSInfoBase->commonWin == 1) ? &(m_PSInfoBase->icsInfo[0]) : &(m_PSInfoBase->icsInfo[ch]);

    if (!m_PSInfoBase->pnsUsed[ch])
        return 0;

    if (icsInfo->winSequence == 2) {
        sfbTab = sfBandTabShort + sfBandTabShortOffset[m_PSInfoBase->sampRateIdx];
        nSamps = NSAMPS_SHORT;
    } else {
        sfbTab = sfBandTabLong + sfBandTabLongOffset[m_PSInfoBase->sampRateIdx];
        nSamps = NSAMPS_LONG;
    }
    coef = m_PSInfoBase->coef[ch];
    sfbCodeBook = m_PSInfoBase->sfbCodeBook[ch];
    scaleFactors = m_PSInfoBase->scaleFactors[ch];
    checkCorr = (m_AACDecInfo->currBlockID == AAC_ID_CPE && m_PSInfoBase->commonWin == 1 ? 1 : 0);

    gbMask = 0;
    for (gp = 0; gp < icsInfo->numWinGroup; gp++) {
        for (win = 0; win < icsInfo->winGroupLen[gp]; win++) {
            msMaskPtr = m_PSInfoBase->msMaskBits + ((gp*icsInfo->maxSFB) >> 3);
            msMaskOffset = ((gp*icsInfo->maxSFB) & 0x07);
            msMask = (*msMaskPtr++) >> msMaskOffset;

            for (sfb = 0; sfb < icsInfo->maxSFB; sfb++) {
                width = sfbTab[sfb+1] - sfbTab[sfb];
                if (sfbCodeBook[sfb] == 13) {
                    if (ch == 0) {
                        /* generate new vector, copy into ch 1 if it's possible that the channels will be correlated
                         * if ch 1 has PNS enabled for this SFB but it's uncorrelated (i.e. ms_used == 0),
                         *    the copied values will be overwritten when we process ch 1
                         */
                        GenerateNoiseVector(coef, &m_PSInfoBase->pnsLastVal, width);
                        if (checkCorr && m_PSInfoBase->sfbCodeBook[1][gp*icsInfo->maxSFB + sfb] == 13)
                            CopyNoiseVector(coef, m_PSInfoBase->coef[1] + (coef - m_PSInfoBase->coef[0]), width);
                    } else {
                        /* generate new vector if no correlation between channels */
                        genNew = 1;
                        if (checkCorr && m_PSInfoBase->sfbCodeBook[0][gp*icsInfo->maxSFB + sfb] == 13) {
                            if((m_PSInfoBase->msMaskPresent==1 && (msMask & 0x01)) || m_PSInfoBase->msMaskPresent == 2 )
                                genNew = 0;
                        }
                        if (genNew)
                            GenerateNoiseVector(coef, &m_PSInfoBase->pnsLastVal, width);
                    }
                    gbMask |= ScaleNoiseVector(coef, width, m_PSInfoBase->scaleFactors[ch][gp*icsInfo->maxSFB + sfb]);
                }
                coef += width;

                /* get next mask bit (should be branchless on ARM) */
                msMask >>= 1;
                if (++msMaskOffset == 8) {
                    msMask = *msMaskPtr++;
                    msMaskOffset = 0;
                }
            }
            coef += (nSamps - sfbTab[icsInfo->maxSFB]);
        }
        sfbCodeBook += icsInfo->maxSFB;
        scaleFactors += icsInfo->maxSFB;
    }

    /* update guard bit count if necessary */
    gb = CLZ(gbMask) - 1;
    if (m_PSInfoBase->gbCurrent[ch] > gb)
        m_PSInfoBase->gbCurrent[ch] = gb;

    return 0;
}

/***********************************************************************************************************************
 * Function:    GetSampRateIdx
 *
 * Description: get index of given sample rate
 *
 * Inputs:      sample rate (in Hz)
 *
 * Outputs:     none
 *
 * Return:      index of sample rate (table 1.15 in 14496-3:2001(E))
 *              -1 if sample rate not found in table
 **********************************************************************************************************************/
int GetSampRateIdx(int sampRate)
{
    int idx;

    for (idx = 0; idx < NUM_SAMPLE_RATES; idx++) {
        if (sampRate == sampRateTab[idx])
            return idx;
    }

    return -1;
}

/***********************************************************************************************************************
 * Function:    StereoProcessGroup
 *
 * Description: apply mid-side and intensity stereo to group of transform coefficients
 *
 * Inputs:      dequantized transform coefficients for both channels
 *              pointer to appropriate scalefactor band table
 *              mid-side mask enabled flag
 *              buffer with mid-side mask (one bit for each scalefactor band)
 *              bit offset into mid-side mask buffer
 *              max coded scalefactor band
 *              buffer of codebook indices for right channel
 *              buffer of scalefactors for right channel, range = [0, 256]
 *
 * Outputs:     updated transform coefficients in Q(FBITS_OUT_DQ_OFF)
 *              updated minimum guard bit count for both channels
 *
 * Return:      none
 *
 * Notes:       assume no guard bits in input
 *              gains 0 int bits
 **********************************************************************************************************************/
void StereoProcessGroup(int *coefL, int *coefR, const uint16_t *sfbTab,
                              int msMaskPres, uint8_t *msMaskPtr, int msMaskOffset, int maxSFB,
                              uint8_t *cbRight, short *sfRight, int *gbCurrent)
{
//fb
static const uint32_t pow14[2][4] PROGMEM = {
    { 0xc0000000, 0xb3e407d7, 0xa57d8666, 0x945d819b },
    { 0x40000000, 0x4c1bf829, 0x5a82799a, 0x6ba27e65 }
};

    int sfb, width, cbIdx, sf, cl, cr, scalef, scalei;
    int gbMaskL, gbMaskR;
    uint8_t msMask;

    msMask = (*msMaskPtr++) >> msMaskOffset;
    gbMaskL = 0;
    gbMaskR = 0;

    for (sfb = 0; sfb < maxSFB; sfb++) {
        width = sfbTab[sfb+1] - sfbTab[sfb];    /* assume >= 0 (see sfBandTabLong/sfBandTabShort) */
        cbIdx = cbRight[sfb];

        if (cbIdx == 14 || cbIdx == 15) {
            /* intensity stereo */
            if (msMaskPres == 1 && (msMask & 0x01))
                cbIdx ^= 0x01;                /* invert_intensity(): 14 becomes 15, or 15 becomes 14 */
            sf = -sfRight[sfb];                /* negative since we use identity 0.5^(x) = 2^(-x) (see spec) */
            cbIdx &= 0x01;                    /* choose - or + scale factor */
            scalef = pow14[cbIdx][sf & 0x03];
            scalei = (sf >> 2) + 2;            /* +2 to compensate for scalef = Q30 */

            if (scalei > 0) {
                if (scalei > 30)
                    scalei = 30;
                do {
                    cr = MULSHIFT32(*coefL++, scalef);
                    {int sign = (cr) >> 31; if (sign != (cr) >> (31-scalei))  {(cr) = sign ^ ((1 << (31-scalei)) - 1);}}
                    cr <<= scalei;
                    gbMaskR |= FASTABS(cr);
                    *coefR++ = cr;
                } while (--width);
            } else {
                scalei = -scalei;
                if (scalei > 31)
                    scalei = 31;
                do {
                    cr = MULSHIFT32(*coefL++, scalef) >> scalei;
                    gbMaskR |= FASTABS(cr);
                    *coefR++ = cr;
                } while (--width);
            }
        } else if ( cbIdx != 13 && ((msMaskPres == 1 && (msMask & 0x01)) || msMaskPres == 2) ) {
            /* mid-side stereo (assumes no GB in inputs) */
            do {
                cl = *coefL;
                cr = *coefR;

                if ( (FASTABS(cl) | FASTABS(cr)) >> 30 ) {
                    /* avoid overflow (rare) */
                    cl >>= 1;
                    sf = cl + (cr >> 1);
                    {int sign = (sf) >> 31; if (sign != (sf) >> (30))  {(sf) = sign ^ ((1 << (30)) - 1);}}
                    sf <<= 1;
                    cl = cl - (cr >> 1);
                    {int sign = (cl) >> 31; if (sign != (cl) >> (30))  {(cl) = sign ^ ((1 << (30)) - 1);}}
                    cl <<= 1;
                } else {
                    /* usual case */
                    sf = cl + cr;
                    cl -= cr;
                }

                *coefL++ = sf;
                gbMaskL |= FASTABS(sf);
                *coefR++ = cl;
                gbMaskR |= FASTABS(cl);
            } while (--width);

        } else {
            /* nothing to do */
            coefL += width;
            coefR += width;
        }

        /* get next mask bit (should be branchless on ARM) */
        msMask >>= 1;
        if (++msMaskOffset == 8) {
            msMask = *msMaskPtr++;
            msMaskOffset = 0;
        }
    }

    cl = CLZ(gbMaskL) - 1;
    if (gbCurrent[0] > cl)
        gbCurrent[0] = cl;

    cr = CLZ(gbMaskR) - 1;
    if (gbCurrent[1] > cr)
        gbCurrent[1] = cr;

    return;
}

/***********************************************************************************************************************
 * Function:    StereoProcess
 *
 * Description: apply mid-side and intensity stereo, if enabled
 *
 * Inputs:      none
 *
 * Outputs:     updated transform coefficients in Q(FBITS_OUT_DQ_OFF)
 *              updated minimum guard bit count for both channels
 *
 * Return:      0 if successful, -1 if error
 **********************************************************************************************************************/
int StereoProcess()
{
    ICSInfo_t *icsInfo;
    int gp, win, nSamps, msMaskOffset;
    int *coefL, *coefR;
    uint8_t *msMaskPtr;
    const uint16_t *sfbTab;


    /* mid-side and intensity stereo require common_window == 1 (see MPEG4 spec, Correction 2, 2004) */
    if (m_PSInfoBase->commonWin != 1 || m_AACDecInfo->currBlockID != AAC_ID_CPE)
        return 0;

    /* nothing to do */
    if (!m_PSInfoBase->msMaskPresent && !m_PSInfoBase->intensityUsed[1])
        return 0;

    icsInfo = &(m_PSInfoBase->icsInfo[0]);
    if (icsInfo->winSequence == 2) {
        sfbTab = sfBandTabShort + sfBandTabShortOffset[m_PSInfoBase->sampRateIdx];
        nSamps = NSAMPS_SHORT;
    } else {
        sfbTab = sfBandTabLong + sfBandTabLongOffset[m_PSInfoBase->sampRateIdx];
        nSamps = NSAMPS_LONG;
    }
    coefL = m_PSInfoBase->coef[0];
    coefR = m_PSInfoBase->coef[1];

    /* do fused mid-side/intensity processing for each block (one long or eight short) */
    msMaskOffset = 0;
    msMaskPtr = m_PSInfoBase->msMaskBits;
    for (gp = 0; gp < icsInfo->numWinGroup; gp++) {
        for (win = 0; win < icsInfo->winGroupLen[gp]; win++) {
            StereoProcessGroup(coefL, coefR, sfbTab, m_PSInfoBase->msMaskPresent,
                msMaskPtr, msMaskOffset, icsInfo->maxSFB, m_PSInfoBase->sfbCodeBook[1] + gp*icsInfo->maxSFB,
                m_PSInfoBase->scaleFactors[1] + gp*icsInfo->maxSFB, m_PSInfoBase->gbCurrent);
            coefL += nSamps;
            coefR += nSamps;
        }
        /* we use one bit per sfb, so there are maxSFB bits for each window group */
        msMaskPtr += (msMaskOffset + icsInfo->maxSFB) >> 3;
        msMaskOffset = (msMaskOffset + icsInfo->maxSFB) & 0x07;
    }

    ASSERT(coefL == m_PSInfoBase->coef[0] + 1024);
    ASSERT(coefR == m_PSInfoBase->coef[1] + 1024);

    return 0;
}

/***********************************************************************************************************************
 * Function:    RatioPowInv
 *
 * Description: use Taylor (MacLaurin) series expansion to calculate (a/b) ^ (1/c)
 *
 * Inputs:      a = [1, 64], b = [1, 64], c = [1, 64], a >= b
 *
 * Outputs:     none
 *
 * Return:      y = Q24, range ~= [0.015625, 64]
 **********************************************************************************************************************/
int RatioPowInv(int a, int b, int c)
{
    int lna, lnb, i, p, t, y;

    if (a < 1 || b < 1 || c < 1 || a > 64 || b > 64 || c > 64 || a < b)
        return 0;

    lna = MULSHIFT32(log2Tab[a], LOG2_EXP_INV) << 1;    /* ln(a), Q28 */
    lnb = MULSHIFT32(log2Tab[b], LOG2_EXP_INV) << 1;    /* ln(b), Q28 */
    p = (lna - lnb) / c;    /* Q28 */

    /* sum in Q24 */
    y = (1 << 24);
    t = p >> 4;        /* t = p^1 * 1/1! (Q24)*/
    y += t;

    for (i = 2; i <= NUM_TERMS_RPI; i++) {
        t = MULSHIFT32(invTab[i-1], t) << 2;
        t = MULSHIFT32(p, t) << 4;    /* t = p^i * 1/i! (Q24) */
        y += t;
    }

    return y;
}

/***********************************************************************************************************************
 * Function:    SqrtFix
 *
 * Description: use binary search to calculate sqrt(q)
 *
 * Inputs:      q = Q30
 *              number of fraction bits in input
 *
 * Outputs:     number of fraction bits in output
 *
 * Return:      lo = Q(fBitsOut)
 *
 * Notes:       absolute precision varies depending on fBitsIn
 *              normalizes input to range [0x200000000, 0x7fffffff] and takes
 *                floor(sqrt(input)), and sets fBitsOut appropriately
 **********************************************************************************************************************/
int SqrtFix(int q, int fBitsIn, int *fBitsOut)
{
    int z, lo, hi, mid;

    if (q <= 0) {
        *fBitsOut = fBitsIn;
        return 0;
    }

    /* force even fBitsIn */
    z = fBitsIn & 0x01;
    q >>= z;
    fBitsIn -= z;

    /* for max precision, normalize to [0x20000000, 0x7fffffff] */
    z = (CLZ(q) - 1);
    z >>= 1;
    q <<= (2*z);

    /* choose initial bounds */
    lo = 1;
    if (q >= 0x10000000)
        lo = 16384;    /* (int)sqrt(0x10000000) */
    hi = 46340;        /* (int)sqrt(0x7fffffff) */

    /* do binary search with 32x32->32 multiply test */
    do {
        mid = (lo + hi) >> 1;
        if (mid*mid > q)
            hi = mid - 1;
        else
            lo = mid + 1;
    } while (hi >= lo);
    lo--;

    *fBitsOut = ((fBitsIn + 2*z) >> 1);
    return lo;
}

/***********************************************************************************************************************
 * Function:    InvRNormalized
 *
 * Description: use Newton's method to solve for x = 1/r
 *
 * Inputs:      r = Q31, range = [0.5, 1) (normalize your inputs to this range)
 *
 * Outputs:     none
 *
 * Return:      x = Q29, range ~= [1.0, 2.0]
 *
 * Notes:       guaranteed to converge and not overflow for any r in [0.5, 1)
 *
 *              xn+1  = xn - f(xn)/f'(xn)
 *              f(x)  = 1/r - x = 0 (find root)
 *                    = 1/x - r
 *              f'(x) = -1/x^2
 *
 *              so xn+1 = xn - (1/xn - r) / (-1/xn^2)
 *                      = xn * (2 - r*xn)
 *
 *              NUM_ITER_IRN = 2, maxDiff = 6.2500e-02 (precision of about 4 bits)
 *              NUM_ITER_IRN = 3, maxDiff = 3.9063e-03 (precision of about 8 bits)
 *              NUM_ITER_IRN = 4, maxDiff = 1.5288e-05 (precision of about 16 bits)
 *              NUM_ITER_IRN = 5, maxDiff = 3.0034e-08 (precision of about 24 bits)
 **********************************************************************************************************************/
int InvRNormalized(int r)
{
    int i, xn, t;

    /* r =   [0.5, 1.0)
     * 1/r = (1.0, 2.0]
     *   so use 1.5 as initial guess
     */
    xn = Q28_15;

    /* xn = xn*(2.0 - r*xn) */
    for (i = NUM_ITER_IRN; i != 0; i--) {
        t = MULSHIFT32(r, xn);            /* Q31*Q29 = Q28 */
        t = Q28_2 - t;                    /* Q28 */
        xn = MULSHIFT32(xn, t) << 4;    /* Q29*Q28 << 4 = Q29 */
    }

    return xn;
}



/***********************************************************************************************************************
 * Function:    BitReverse32
 *
 * Description: Ken's fast in-place bit reverse
 *
 * Inputs:      buffer of 32 complex samples
 *
 * Outputs:     bit-reversed samples in same buffer
 *
 * Return:      none
***********************************************************************************************************************/
void BitReverse32(int *inout)
{
    int t;
    t=inout[2] ; inout[2]=inout[32]; inout[32]=t;
    t=inout[3] ; inout[3]=inout[33]; inout[33]=t;

    t=inout[4] ; inout[4]=inout[16]; inout[16]=t;
    t=inout[5] ; inout[5]=inout[17]; inout[17]=t;

    t=inout[6] ; inout[6]=inout[48]; inout[48]=t;
    t=inout[7] ; inout[7]=inout[49]; inout[49]=t;

    t=inout[10]; inout[10]=inout[40]; inout[40]=t;
    t=inout[11]; inout[11]=inout[41]; inout[41]=t;

    t=inout[12]; inout[12]=inout[24]; inout[24]=t;
    t=inout[13]; inout[13]=inout[25]; inout[25]=t;

    t=inout[14]; inout[14]=inout[56]; inout[56]=t;
    t=inout[15]; inout[15]=inout[57]; inout[57]=t;

    t=inout[18]; inout[18]=inout[36]; inout[36]=t;
    t=inout[19]; inout[19]=inout[37]; inout[37]=t;

    t=inout[22]; inout[22]=inout[52]; inout[52]=t;
    t=inout[23]; inout[23]=inout[53]; inout[53]=t;

    t=inout[26]; inout[26]=inout[44]; inout[44]=t;
    t=inout[27]; inout[27]=inout[45]; inout[45]=t;

    t=inout[30]; inout[30]=inout[60]; inout[60]=t;
    t=inout[31]; inout[31]=inout[61]; inout[61]=t;

    t=inout[38]; inout[38]=inout[50]; inout[50]=t;
    t=inout[39]; inout[39]=inout[51]; inout[51]=t;

    t=inout[46]; inout[46]=inout[58]; inout[58]=t;
    t=inout[47]; inout[47]=inout[59]; inout[59]=t;

}

/***********************************************************************************************************************
 * Function:    R8FirstPass32
 *
 * Description: radix-8 trivial pass for decimation-in-time FFT (log2(N) = 5)
 *
 * Inputs:      buffer of (bit-reversed) samples
 *
 * Outputs:     processed samples in same buffer
 *
 * Return:      none
 *
 * Notes:       assumes 3 guard bits, gains 1 integer bit
 *              guard bits out = guard bits in - 3 (if inputs are full scale)
 *                or guard bits in - 2 (if inputs bounded to +/- sqrt(2)/2)
 *              see scaling comments in fft.c for base AAC
 *              should compile with no stack spills on ARM (verify compiled output)
 *              current instruction count (per pass): 16 LDR, 16 STR, 4 SMULL, 61 ALU
 **********************************************************************************************************************/
void R8FirstPass32(int *r0)
{
    int r1, r2, r3, r4, r5, r6, r7;
    int r8, r9, r10, r11, r12, r14;

    /* number of passes = fft size / 8 = 32 / 8 = 4 */
    r1 = (32 >> 3);
    do {

        r2 = r0[8];
        r3 = r0[9];
        r4 = r0[10];
        r5 = r0[11];
        r6 = r0[12];
        r7 = r0[13];
        r8 = r0[14];
        r9 = r0[15];

        r10 = r2 + r4;
        r11 = r3 + r5;
        r12 = r6 + r8;
        r14 = r7 + r9;

        r2 -= r4;
        r3 -= r5;
        r6 -= r8;
        r7 -= r9;

        r4 = r2 - r7;
        r5 = r2 + r7;
        r8 = r3 - r6;
        r9 = r3 + r6;

        r2 = r4 - r9;
        r3 = r4 + r9;
        r6 = r5 - r8;
        r7 = r5 + r8;

        r2 = MULSHIFT32(SQRTHALF, r2);    /* can use r4, r5, r8, or r9 for constant and lo32 scratch reg */
        r3 = MULSHIFT32(SQRTHALF, r3);
        r6 = MULSHIFT32(SQRTHALF, r6);
        r7 = MULSHIFT32(SQRTHALF, r7);

        r4 = r10 + r12;
        r5 = r10 - r12;
        r8 = r11 + r14;
        r9 = r11 - r14;

        r10 = r0[0];
        r11 = r0[2];
        r12 = r0[4];
        r14 = r0[6];

        r10 += r11;
        r12 += r14;

        r4 >>= 1;
        r10 += r12;
        r4 += (r10 >> 1);
        r0[ 0] = r4;
        r4 -= (r10 >> 1);
        r4 = (r10 >> 1) - r4;
        r0[ 8] = r4;

        r9 >>= 1;
        r10 -= 2*r12;
        r4 = (r10 >> 1) + r9;
        r0[ 4] = r4;
        r4 = (r10 >> 1) - r9;
        r0[12] = r4;
        r10 += r12;

        r10 -= 2*r11;
        r12 -= 2*r14;

        r4 =  r0[1];
        r9 =  r0[3];
        r11 = r0[5];
        r14 = r0[7];

        r4 += r9;
        r11 += r14;

        r8 >>= 1;
        r4 += r11;
        r8 += (r4 >> 1);
        r0[ 1] = r8;
        r8 -= (r4 >> 1);
        r8 = (r4 >> 1) - r8;
        r0[ 9] = r8;

        r5 >>= 1;
        r4 -= 2*r11;
        r8 = (r4 >> 1) - r5;
        r0[ 5] = r8;
        r8 = (r4 >> 1) + r5;
        r0[13] = r8;
        r4 += r11;

        r4 -= 2*r9;
        r11 -= 2*r14;

        r9 = r10 - r11;
        r10 += r11;
        r14 = r4 + r12;
        r4 -= r12;

        r5 = (r10 >> 1) + r7;
        r8 = (r4 >> 1) - r6;
        r0[ 2] = r5;
        r0[ 3] = r8;

        r5 = (r9 >> 1) - r2;
        r8 = (r14 >> 1) - r3;
        r0[ 6] = r5;
        r0[ 7] = r8;

        r5 = (r10 >> 1) - r7;
        r8 = (r4 >> 1) + r6;
        r0[10] = r5;
        r0[11] = r8;

        r5 = (r9 >> 1) + r2;
        r8 = (r14 >> 1) + r3;
        r0[14] = r5;
        r0[15] = r8;

        r0 += 16;
        r1--;
    } while (r1 != 0);
}

/***********************************************************************************************************************
 * Function:    R4Core32
 *
 * Description: radix-4 pass for 32-point decimation-in-time FFT
 *
 * Inputs:      buffer of samples
 *
 * Outputs:     processed samples in same buffer
 *
 * Return:      none
 *
 * Notes:       gain 2 integer bits
 *              guard bits out = guard bits in - 1 (if inputs are full scale)
 *              see scaling comments in fft.c for base AAC
 *              uses 3-mul, 3-add butterflies instead of 4-mul, 2-add
 *              should compile with no stack spills on ARM (verify compiled output)
 *              current instruction count (per pass): 16 LDR, 16 STR, 4 SMULL, 61 ALU
 **********************************************************************************************************************/
void R4Core32(int *r0)
{
    int r2, r3, r4, r5, r6, r7;
    int r8, r9, r10, r12, r14;
    int *r1;

    r1 = (int *)twidTabOdd32;
    r10 = 8;
    do {
        /* can use r14 for lo32 scratch register in all MULSHIFT32 */
        r2 = r1[0];
        r3 = r1[1];
        r4 = r0[16];
        r5 = r0[17];
        r12 = r4 + r5;
        r12 = MULSHIFT32(r3, r12);
        r5  = MULSHIFT32(r2, r5) + r12;
        r2 += 2*r3;
        r4  = MULSHIFT32(r2, r4) - r12;

        r2 = r1[2];
        r3 = r1[3];
        r6 = r0[32];
        r7 = r0[33];
        r12 = r6 + r7;
        r12 = MULSHIFT32(r3, r12);
        r7  = MULSHIFT32(r2, r7) + r12;
        r2 += 2*r3;
        r6  = MULSHIFT32(r2, r6) - r12;

        r2 = r1[4];
        r3 = r1[5];
        r8 = r0[48];
        r9 = r0[49];
        r12 = r8 + r9;
        r12 = MULSHIFT32(r3, r12);
        r9  = MULSHIFT32(r2, r9) + r12;
        r2 += 2*r3;
        r8  = MULSHIFT32(r2, r8) - r12;

        r2 = r0[0];
        r3 = r0[1];

        r12 = r6 + r8;
        r8  = r6 - r8;
        r14 = r9 - r7;
        r9  = r9 + r7;

        r6 = (r2 >> 2) - r4;
        r7 = (r3 >> 2) - r5;
        r4 += (r2 >> 2);
        r5 += (r3 >> 2);

        r2 = r4 + r12;
        r3 = r5 + r9;
        r0[0] = r2;
        r0[1] = r3;
        r2 = r6 - r14;
        r3 = r7 - r8;
        r0[16] = r2;
        r0[17] = r3;
        r2 = r4 - r12;
        r3 = r5 - r9;
        r0[32] = r2;
        r0[33] = r3;
        r2 = r6 + r14;
        r3 = r7 + r8;
        r0[48] = r2;
        r0[49] = r3;

        r0 += 2;
        r1 += 6;
        r10--;
    } while (r10 != 0);
}

/***********************************************************************************************************************
 * Function:    FFT32C
 *
 * Description: Ken's very fast in-place radix-4 decimation-in-time FFT
 *
 * Inputs:      buffer of 32 complex samples (before bit-reversal)
 *
 * Outputs:     processed samples in same buffer
 *
 * Return:      none
 *
 * Notes:       assumes 3 guard bits in, gains 3 integer bits
 *              guard bits out = guard bits in - 2
 *              (guard bit analysis includes assumptions about steps immediately
 *               before and after, i.e. PreMul and PostMul for DCT)
 **********************************************************************************************************************/
void FFT32C(int *x)
{
    /* decimation in time */
    BitReverse32(x);

    /* 32-point complex FFT */
    R8FirstPass32(x);    /* gain 1 int bit,  lose 2 GB (making assumptions about input) */
    R4Core32(x);        /* gain 2 int bits, lose 0 GB (making assumptions about input) */
}

/***********************************************************************************************************************
 * Function:    CVKernel1
 *
 * Description: kernel of covariance matrix calculation for p01, p11, p12, p22
 *
 * Inputs:      buffer of low-freq samples, starting at time index = 0,
 *                freq index = patch subband
 *
 * Outputs:     64-bit accumulators for p01re, p01im, p12re, p12im, p11re, p22re
 *                stored in accBuf
 *
 * Return:      none
 *
 * Notes:       this is carefully written to be efficient on ARM
 *              use the assembly code version in sbrcov.s when building for ARM!
 **********************************************************************************************************************/
void CVKernel1(int *XBuf, int *accBuf)
{
    U64 p01re, p01im, p12re, p12im, p11re, p22re;
    int n, x0re, x0im, x1re, x1im;

    x0re = XBuf[0];
    x0im = XBuf[1];
    XBuf += (2*64);
    x1re = XBuf[0];
    x1im = XBuf[1];
    XBuf += (2*64);

    p01re.w64 = p01im.w64 = 0;
    p12re.w64 = p12im.w64 = 0;
    p11re.w64 = 0;
    p22re.w64 = 0;

    p12re.w64 = MADD64(p12re.w64,  x1re, x0re);
    p12re.w64 = MADD64(p12re.w64,  x1im, x0im);
    p12im.w64 = MADD64(p12im.w64,  x0re, x1im);
    p12im.w64 = MADD64(p12im.w64, -x0im, x1re);
    p22re.w64 = MADD64(p22re.w64,  x0re, x0re);
    p22re.w64 = MADD64(p22re.w64,  x0im, x0im);
    for (n = (NUM_TIME_SLOTS*SAMPLES_PER_SLOT + 6); n != 0; n--) {
        /* 4 input, 3*2 acc, 1 ptr, 1 loop counter = 12 registers (use same for x0im, -x0im) */
        x0re = x1re;
        x0im = x1im;
        x1re = XBuf[0];
        x1im = XBuf[1];

        p01re.w64 = MADD64(p01re.w64,  x1re, x0re);
        p01re.w64 = MADD64(p01re.w64,  x1im, x0im);
        p01im.w64 = MADD64(p01im.w64,  x0re, x1im);
        p01im.w64 = MADD64(p01im.w64, -x0im, x1re);
        p11re.w64 = MADD64(p11re.w64,  x0re, x0re);
        p11re.w64 = MADD64(p11re.w64,  x0im, x0im);

        XBuf += (2*64);
    }
    /* these can be derived by slight changes to account for boundary conditions */
    p12re.w64 += p01re.w64;
    p12re.w64 = MADD64(p12re.w64, x1re, -x0re);
    p12re.w64 = MADD64(p12re.w64, x1im, -x0im);
    p12im.w64 += p01im.w64;
    p12im.w64 = MADD64(p12im.w64, x0re, -x1im);
    p12im.w64 = MADD64(p12im.w64, x0im,  x1re);
    p22re.w64 += p11re.w64;
    p22re.w64 = MADD64(p22re.w64, x0re, -x0re);
    p22re.w64 = MADD64(p22re.w64, x0im, -x0im);

    accBuf[0]  = p01re.r.lo32;    accBuf[1]  = p01re.r.hi32;
    accBuf[2]  = p01im.r.lo32;    accBuf[3]  = p01im.r.hi32;
    accBuf[4]  = p11re.r.lo32;    accBuf[5]  = p11re.r.hi32;
    accBuf[6]  = p12re.r.lo32;    accBuf[7]  = p12re.r.hi32;
    accBuf[8]  = p12im.r.lo32;    accBuf[9]  = p12im.r.hi32;
    accBuf[10] = p22re.r.lo32;    accBuf[11] = p22re.r.hi32;
}

/***********************************************************************************************************************
 * Function:    CVKernel2
 *
 * Description: kernel of covariance matrix calculation for p02
 *
 * Inputs:      buffer of low-freq samples, starting at time index = 0,
 *                freq index = patch subband
 *
 * Outputs:     64-bit accumulators for p02re, p02im stored in accBuf
 *
 * Return:      none
 *
 * Notes:       this is carefully written to be efficient on ARM
 *              use the assembly code version in sbrcov.s when building for ARM!
 **********************************************************************************************************************/
void CVKernel2(int *XBuf, int *accBuf)
{
    U64 p02re, p02im;
    int n, x0re, x0im, x1re, x1im, x2re, x2im;

    p02re.w64 = p02im.w64 = 0;

    x0re = XBuf[0];
    x0im = XBuf[1];
    XBuf += (2*64);
    x1re = XBuf[0];
    x1im = XBuf[1];
    XBuf += (2*64);

    for (n = (NUM_TIME_SLOTS*SAMPLES_PER_SLOT + 6); n != 0; n--) {
        /* 6 input, 2*2 acc, 1 ptr, 1 loop counter = 12 registers (use same for x0im, -x0im) */
        x2re = XBuf[0];
        x2im = XBuf[1];

        p02re.w64 = MADD64(p02re.w64,  x2re, x0re);
        p02re.w64 = MADD64(p02re.w64,  x2im, x0im);
        p02im.w64 = MADD64(p02im.w64,  x0re, x2im);
        p02im.w64 = MADD64(p02im.w64, -x0im, x2re);

        x0re = x1re;
        x0im = x1im;
        x1re = x2re;
        x1im = x2im;
        XBuf += (2*64);
    }

    accBuf[0] = p02re.r.lo32;
    accBuf[1] = p02re.r.hi32;
    accBuf[2] = p02im.r.lo32;
    accBuf[3] = p02im.r.hi32;
}

/***********************************************************************************************************************
 * Function:    SetBitstreamPointer
 *
 * Description: initialize bitstream reader
 *
 * Inputs:      number of bytes in bitstream
 *              pointer to byte-aligned buffer of data to read from
 *
 * Outputs:     initialized bitstream info struct
 *
 * Return:      none
 **********************************************************************************************************************/
void SetBitstreamPointer(int nBytes, uint8_t *buf)
{
    /* init bitstream */
    m_aac_BitStreamInfo.bytePtr = buf;
    m_aac_BitStreamInfo.iCache = 0;        /* 4-byte uint32_t */
    m_aac_BitStreamInfo.cachedBits = 0;    /* i.e. zero bits in cache */
    m_aac_BitStreamInfo.nBytes = nBytes;
}

/***********************************************************************************************************************
 * Function:    RefillBitstreamCache
 *
 * Description: read new data from bitstream buffer into 32-bit cache
 *
 * Inputs:      none
 *
 * Outputs:     updated bitstream info struct
 *
 * Return:      none
 *
 * Notes:       only call when iCache is completely drained (resets bitOffset to 0)
 *              always loads 4 new bytes except when bsi->nBytes < 4 (end of buffer)
 *              stores data as big-endian in cache, regardless of machine endian-ness
 **********************************************************************************************************************/
//Optimized for REV16, REV32 (FB)
inline void RefillBitstreamCache()
{
    int nBytes = m_aac_BitStreamInfo.nBytes;
    if (nBytes >= 4) {
        /* optimize for common case, independent of machine endian-ness */
        m_aac_BitStreamInfo.iCache  = (*m_aac_BitStreamInfo.bytePtr++) << 24;
        m_aac_BitStreamInfo.iCache |= (*m_aac_BitStreamInfo.bytePtr++) << 16;
        m_aac_BitStreamInfo.iCache |= (*m_aac_BitStreamInfo.bytePtr++) <<  8;
        m_aac_BitStreamInfo.iCache |= (*m_aac_BitStreamInfo.bytePtr++);

        m_aac_BitStreamInfo.cachedBits = 32;
        m_aac_BitStreamInfo.nBytes -= 4;
    } else {
        m_aac_BitStreamInfo.iCache = 0;
        while (nBytes--) {
            m_aac_BitStreamInfo.iCache |= (*m_aac_BitStreamInfo.bytePtr++);
            m_aac_BitStreamInfo.iCache <<= 8;
        }
        m_aac_BitStreamInfo.iCache <<= ((3 - m_aac_BitStreamInfo.nBytes)*8);
        m_aac_BitStreamInfo.cachedBits = 8*m_aac_BitStreamInfo.nBytes;
        m_aac_BitStreamInfo.nBytes = 0;
    }
}

/***********************************************************************************************************************
 * Function:    GetBits
 *
 * Description: get bits from bitstream, advance bitstream pointer
 *
 * Inputs:      pointer to initialized aac_BitStreamInfo_t struct
 *              number of bits to get from bitstream
 *
 * Outputs:     updated bitstream info struct
 *
 * Return:      the next nBits bits of data from bitstream buffer
 *
 * Notes:       nBits must be in range [0, 31], nBits outside this range masked by 0x1f
 *              for speed, does not indicate error if you overrun bit buffer
 *              if nBits == 0, returns 0
 **********************************************************************************************************************/
unsigned int GetBits(int nBits)
{
    uint32_t data, lowBits;

    nBits &= 0x1f;                          /* nBits mod 32 to avoid unpredictable results like >> by negative amount */
    data = m_aac_BitStreamInfo.iCache >> (31 - nBits);        /* unsigned >> so zero-extend */
    data >>= 1;                                         /* do as >> 31, >> 1 so that nBits = 0 works okay (returns 0) */
    m_aac_BitStreamInfo.iCache <<= nBits;                    /* left-justify cache */
    m_aac_BitStreamInfo.cachedBits -= nBits;                 /* how many bits have we drawn from the cache so far */

    /* if we cross an int boundary, refill the cache */
    if (m_aac_BitStreamInfo.cachedBits < 0) {
        lowBits = -m_aac_BitStreamInfo.cachedBits;
        RefillBitstreamCache();
        data |= m_aac_BitStreamInfo.iCache >> (32 - lowBits);        /* get the low-order bits */

        m_aac_BitStreamInfo.cachedBits -= lowBits;            /* how many bits have we drawn from the cache so far */
        m_aac_BitStreamInfo.iCache <<= lowBits;            /* left-justify cache */
    }

    return data;
}

/***********************************************************************************************************************
 * Function:    GetBitsNoAdvance
 *
 * Description: get bits from bitstream, do not advance bitstream pointer
 *
 * Inputs:      pointer to initialized aac_BitStreamInfo_t struct
 *              number of bits to get from bitstream
 *
 * Outputs:     none (state of aac_BitStreamInfo_t struct left unchanged)
 *
 * Return:      the next nBits bits of data from bitstream buffer
 *
 * Notes:       nBits must be in range [0, 31], nBits outside this range masked by 0x1f
 *              for speed, does not indicate error if you overrun bit buffer
 *              if nBits == 0, returns 0
 **********************************************************************************************************************/
unsigned int GetBitsNoAdvance(int nBits)
{
    uint8_t *buf;
    uint32_t data, iCache;
    int32_t lowBits;

    nBits &= 0x1f;                          /* nBits mod 32 to avoid unpredictable results like >> by negative amount */
    data = m_aac_BitStreamInfo.iCache >> (31 - nBits);        /* unsigned >> so zero-extend */
    data >>= 1;                                         /* do as >> 31, >> 1 so that nBits = 0 works okay (returns 0) */
    lowBits = nBits - m_aac_BitStreamInfo.cachedBits;        /* how many bits do we have left to read */

    /* if we cross an int boundary, read next bytes in buffer */
    if (lowBits > 0) {
        iCache = 0;
        buf = m_aac_BitStreamInfo.bytePtr;
        while (lowBits > 0) {
            iCache <<= 8;
            if (buf < m_aac_BitStreamInfo.bytePtr + m_aac_BitStreamInfo.nBytes)
                iCache |= (uint32_t)*buf++;
            lowBits -= 8;
        }
        lowBits = -lowBits;
        data |= iCache >> lowBits;
    }

    return data;
}

/***********************************************************************************************************************
 * Function:    AdvanceBitstream
 *
 * Description: move bitstream pointer ahead
 *
 * Inputs:      number of bits to advance bitstream
 *
 * Outputs:     updated bitstream info struct
 *
 * Return:      none
 *
 * Notes:       generally used following GetBitsNoAdvance(bsi, maxBits)
 **********************************************************************************************************************/
void AdvanceBitstream(int nBits)
{
    nBits &= 0x1f;
    if (nBits > m_aac_BitStreamInfo.cachedBits) {
        nBits -= m_aac_BitStreamInfo.cachedBits;
        RefillBitstreamCache();
    }
    m_aac_BitStreamInfo.iCache <<= nBits;
    m_aac_BitStreamInfo.cachedBits -= nBits;
}

/***********************************************************************************************************************
 * Function:    CalcBitsUsed
 *
 * Description: calculate how many bits have been read from bitstream
 *
 * Inputs:      pointer to start of bitstream buffer
 *              bit offset into first byte of startBuf (0-7)
 *
 * Outputs:     none
 *
 * Return:      number of bits read from bitstream, as offset from startBuf:startOffset
 **********************************************************************************************************************/
int CalcBitsUsed(uint8_t *startBuf, int startOffset) {

    int bitsUsed;

    bitsUsed  = (m_aac_BitStreamInfo.bytePtr - startBuf) * 8;
    bitsUsed -= m_aac_BitStreamInfo.cachedBits;
    bitsUsed -= startOffset;

    return bitsUsed;
}
/***********************************************************************************************************************
 * Function:    ByteAlignBitstream
 *
 * Description: bump bitstream pointer to start of next byte
 *
 * Inputs:      none
 *
 * Outputs:     byte-aligned bitstream aac_BitStreamInfo_t struct
 *
 * Return:      none
 *
 * Notes:       if bitstream is already byte-aligned, do nothing
 **********************************************************************************************************************/
void ByteAlignBitstream(){

    int offset;

    offset = m_aac_BitStreamInfo.cachedBits & 0x07;
    AdvanceBitstream(offset);
}

#ifdef AAC_ENABLE_SBR

/**************************************************************************************
 * Function:    InitSBRState
 *
 * Description: initialize PSInfoSBR struct at start of stream or after flush
 *
 * Inputs:      valid AACDecInfo struct
 *
 * Outputs:     PSInfoSBR struct with proper initial state
 *
 * Return:      none
 **************************************************************************************/
void InitSBRState() {

    int i, ch;
    uint8_t *c;

    if (!m_PSInfoSBR)
        return;

    /* clear SBR state structure */
    c = (uint8_t *)m_PSInfoSBR;
    for (i = 0; i < (int)sizeof(m_PSInfoSBR); i++)
        *c++ = 0;

    /* initialize non-zero state variables */
    for (ch = 0; ch < AAC_MAX_NCHANS; ch++) {
        m_PSInfoSBR->sbrChan[ch].reset = 1;
        m_PSInfoSBR->sbrChan[ch].laPrev = -1;
    }
}
#endif

/***********************************************************************************************************************
 * Function:    DecodeSBRBitstream
 *
 * Description: decode sideband information for SBR
 *
 * Inputs:      base output channel (range = [0, nChans-1])
 *
 * Outputs:     initialized state structs (SBRHdr, SBRGrid, SBRFreq, SBRChan)
 *
 * Return:      0 if successful, error code (< 0) if error
 *
 * Notes:       SBR payload should be in aacDecInfo->fillBuf
 *              returns with no error if fill buffer is not an SBR extension block,
 *                or if current block is not a fill block (e.g. for LFE upsampling)
 **********************************************************************************************************************/
int DecodeSBRBitstream(int chBase) {

    int headerFlag;

    if(m_AACDecInfo->currBlockID != AAC_ID_FIL
            || (m_AACDecInfo->fillExtType != EXT_SBR_DATA && m_AACDecInfo->fillExtType != EXT_SBR_DATA_CRC))
        return ERR_AAC_NONE;

    SetBitstreamPointer(m_AACDecInfo->fillCount, m_AACDecInfo->fillBuf);
    if(GetBits(4) != (unsigned int) m_AACDecInfo->fillExtType) return ERR_AAC_SBR_BITSTREAM;

    if(m_AACDecInfo->fillExtType == EXT_SBR_DATA_CRC) m_PSInfoSBR->crcCheckWord = GetBits(10);

    headerFlag = GetBits(1);
    if(headerFlag) {
        /* get sample rate index for output sample rate (2x base rate) */
        m_PSInfoSBR->sampRateIdx = GetSampRateIdx(2 * m_AACDecInfo->sampRate);
        if(m_PSInfoSBR->sampRateIdx < 0 || m_PSInfoSBR->sampRateIdx >= NUM_SAMPLE_RATES)
            return ERR_AAC_SBR_BITSTREAM;
        else if(m_PSInfoSBR->sampRateIdx >= NUM_SAMPLE_RATES_SBR) return ERR_AAC_SBR_SINGLERATE_UNSUPPORTED;

        /* reset flag = 1 if header values changed */
        if(UnpackSBRHeader(&(m_PSInfoSBR->sbrHdr[chBase]))) m_PSInfoSBR->sbrChan[chBase].reset = 1;

        /* first valid SBR header should always trigger CalcFreqTables(), since psi->reset was set in InitSBR() */
        if(m_PSInfoSBR->sbrChan[chBase].reset)
            CalcFreqTables(&(m_PSInfoSBR->sbrHdr[chBase + 0]), &(m_PSInfoSBR->sbrFreq[chBase]),
                    m_PSInfoSBR->sampRateIdx);

        /* copy and reset state to right channel for CPE */
        if(m_AACDecInfo->prevBlockID == AAC_ID_CPE)
            m_PSInfoSBR->sbrChan[chBase + 1].reset = m_PSInfoSBR->sbrChan[chBase + 0].reset;
    }

    /* if no header has been received, upsample only */
    if(m_PSInfoSBR->sbrHdr[chBase].count == 0) return ERR_AAC_NONE;

    if(m_AACDecInfo->prevBlockID == AAC_ID_SCE) {
        UnpackSBRSingleChannel(chBase);
    }
    else if(m_AACDecInfo->prevBlockID == AAC_ID_CPE) {
        UnpackSBRChannelPair(chBase);
    }
    else {
        return ERR_AAC_SBR_BITSTREAM;
    }

    ByteAlignBitstream();

    return ERR_AAC_NONE;
}

#ifdef AAC_ENABLE_SBR

/***********************************************************************************************************************
 * Function:    DecodeSBRData
 *
 * Description: apply SBR to one frame of PCM data
 *
 * Inputs:      1024 samples of decoded 32-bit PCM, before SBR
 *              size of input PCM samples (must be 4 bytes)
 *              number of fraction bits in input PCM samples
 *              base output channel (range = [0, nChans-1])
 *              initialized state structs (SBRHdr, SBRGrid, SBRFreq, SBRChan)
 *
 * Outputs:     2048 samples of decoded 16-bit PCM, after SBR
 *
 * Return:      0 if successful, error code (< 0) if error
 **********************************************************************************************************************/
int DecodeSBRData(int chBase, short *outbuf) {

    int k, l, ch, chBlock, qmfaBands, qmfsBands;
    int upsampleOnly, gbIdx, gbMask;
    int *inbuf;
    short *outptr;

    SBRHeader *sbrHdr;
    SBRGrid *sbrGrid;
    SBRFreq *sbrFreq;
    SBRChan *sbrChan;

    /* same header and freq tables for both channels in CPE */
    sbrHdr = &(m_PSInfoSBR->sbrHdr[chBase]);
    sbrFreq = &(m_PSInfoSBR->sbrFreq[chBase]);

    /* upsample only if we haven't received an SBR header yet or if we have an LFE block */
    if(m_AACDecInfo->currBlockID == AAC_ID_LFE) {
        chBlock = 1;
        upsampleOnly = 1;
    }
    else if(m_AACDecInfo->currBlockID == AAC_ID_FIL) {
        if(m_AACDecInfo->prevBlockID == AAC_ID_SCE)
            chBlock = 1;
        else if(m_AACDecInfo->prevBlockID == AAC_ID_CPE)
            chBlock = 2;
        else
            return ERR_AAC_NONE;

        upsampleOnly = (sbrHdr->count == 0 ? 1 : 0);
        if(m_AACDecInfo->fillExtType != EXT_SBR_DATA && m_AACDecInfo->fillExtType != EXT_SBR_DATA_CRC)
            return ERR_AAC_NONE;
    }
    else {
        /* ignore non-SBR blocks */
        return ERR_AAC_NONE;
    }

    if(upsampleOnly) {
        sbrFreq->kStart = 32;
        sbrFreq->numQMFBands = 0;
    }

    for(ch = 0; ch < chBlock; ch++) {
        sbrGrid = &(m_PSInfoSBR->sbrGrid[chBase + ch]);
        sbrChan = &(m_PSInfoSBR->sbrChan[chBase + ch]);

        if(m_AACDecInfo->rawSampleBuf[ch] == 0 || m_AACDecInfo->rawSampleBytes != 4) return ERR_AAC_SBR_PCM_FORMAT;
        inbuf = (int*) m_AACDecInfo->rawSampleBuf[ch];
        outptr = outbuf + chBase + ch;

        /* restore delay buffers (could use ring buffer or keep in temp buffer for nChans == 1) */
        for(l = 0; l < HF_GEN; l++) {
            for(k = 0; k < 64; k++) {
                m_PSInfoSBR->XBuf[l][k][0] = m_PSInfoSBR->XBufDelay[chBase + ch][l][k][0];
                m_PSInfoSBR->XBuf[l][k][1] = m_PSInfoSBR->XBufDelay[chBase + ch][l][k][1];
            }
        }

        /* step 1 - analysis QMF */
        qmfaBands = sbrFreq->kStart;
        for(l = 0; l < 32; l++) {
            gbMask = QMFAnalysis(inbuf + l * 32, m_PSInfoSBR->delayQMFA[chBase + ch], m_PSInfoSBR->XBuf[l + HF_GEN][0],
                    m_AACDecInfo->rawSampleFBits, &(m_PSInfoSBR->delayIdxQMFA[chBase + ch]), qmfaBands);

            gbIdx = ((l + HF_GEN) >> 5) & 0x01;
            sbrChan->gbMask[gbIdx] |= gbMask; /* gbIdx = (0 if i < 32), (1 if i >= 32) */
        }

        if(upsampleOnly) {
            /* no SBR - just run synthesis QMF to upsample by 2x */
            qmfsBands = 32;
            for(l = 0; l < 32; l++) {
                /* step 4 - synthesis QMF */
                QMFSynthesis(m_PSInfoSBR->XBuf[l + HF_ADJ][0], m_PSInfoSBR->delayQMFS[chBase + ch],
                        &(m_PSInfoSBR->delayIdxQMFS[chBase + ch]), qmfsBands, outptr, m_AACDecInfo->nChans);
                outptr += 64 * m_AACDecInfo->nChans;
            }
        }
        else {
            /* if previous frame had lower SBR starting freq than current, zero out the synthesized QMF
             *   bands so they aren't used as sources for patching
             * after patch generation, restore from delay buffer
             * can only happen after header reset
             */
            for(k = sbrFreq->kStartPrev; k < sbrFreq->kStart; k++) {
                for(l = 0; l < sbrGrid->envTimeBorder[0] + HF_ADJ; l++) {
                    m_PSInfoSBR->XBuf[l][k][0] = 0;
                    m_PSInfoSBR->XBuf[l][k][1] = 0;
                }
            }

            /* step 2 - HF generation */
            GenerateHighFreq(sbrGrid, sbrFreq, sbrChan, ch);

            /* restore SBR bands that were cleared before patch generation (time slots 0, 1 no longer needed) */
            for(k = sbrFreq->kStartPrev; k < sbrFreq->kStart; k++) {
                for(l = HF_ADJ; l < sbrGrid->envTimeBorder[0] + HF_ADJ; l++) {
                    m_PSInfoSBR->XBuf[l][k][0] = m_PSInfoSBR->XBufDelay[chBase + ch][l][k][0];
                    m_PSInfoSBR->XBuf[l][k][1] = m_PSInfoSBR->XBufDelay[chBase + ch][l][k][1];
                }
            }

            /* step 3 - HF adjustment */
            AdjustHighFreq(sbrHdr, sbrGrid, sbrFreq, sbrChan, ch);

            /* step 4 - synthesis QMF */
            qmfsBands = sbrFreq->kStartPrev + sbrFreq->numQMFBandsPrev;
            for(l = 0; l < sbrGrid->envTimeBorder[0]; l++) {
                /* if new envelope starts mid-frame, use old settings until start of first envelope in this frame */
                QMFSynthesis(m_PSInfoSBR->XBuf[l + HF_ADJ][0], m_PSInfoSBR->delayQMFS[chBase + ch],
                        &(m_PSInfoSBR->delayIdxQMFS[chBase + ch]), qmfsBands, outptr, m_AACDecInfo->nChans);
                outptr += 64 * m_AACDecInfo->nChans;
            }

            qmfsBands = sbrFreq->kStart + sbrFreq->numQMFBands;
            for(; l < 32; l++) {
                /* use new settings for rest of frame (usually the entire frame, unless the first envelope starts mid-frame) */
                QMFSynthesis(m_PSInfoSBR->XBuf[l + HF_ADJ][0], m_PSInfoSBR->delayQMFS[chBase + ch],
                        &(m_PSInfoSBR->delayIdxQMFS[chBase + ch]), qmfsBands, outptr, m_AACDecInfo->nChans);
                outptr += 64 * m_AACDecInfo->nChans;
            }
        }

        /* save delay */
        for(l = 0; l < HF_GEN; l++) {
            for(k = 0; k < 64; k++) {
                m_PSInfoSBR->XBufDelay[chBase + ch][l][k][0] = m_PSInfoSBR->XBuf[l + 32][k][0];
                m_PSInfoSBR->XBufDelay[chBase + ch][l][k][1] = m_PSInfoSBR->XBuf[l + 32][k][1];
            }
        }
        sbrChan->gbMask[0] = sbrChan->gbMask[1];
        sbrChan->gbMask[1] = 0;

        if(sbrHdr->count > 0) sbrChan->reset = 0;
    }
    sbrFreq->kStartPrev = sbrFreq->kStart;
    sbrFreq->numQMFBandsPrev = sbrFreq->numQMFBands;

    if(m_AACDecInfo->nChans > 0 && (chBase + ch) == m_AACDecInfo->nChans) m_PSInfoSBR->frameCount++;

    return ERR_AAC_NONE;
}

#endif

/***********************************************************************************************************************
 * Function:    BubbleSort
 *
 * Description: in-place sort of uint8_ts
 *
 * Inputs:      buffer of elements to sort
 *              number of elements to sort
 *
 * Outputs:     sorted buffer
 *
 * Return:      none
 **********************************************************************************************************************/
void BubbleSort(uint8_t *v, int nItems) {

    int i;
    uint8_t t;

    while(nItems >= 2) {
        for(i = 0; i < nItems - 1; i++) {
            if(v[i + 1] < v[i]) {
                t = v[i + 1];
                v[i + 1] = v[i];
                v[i] = t;
            }
        }
        nItems--;
    }
}
/***********************************************************************************************************************
 * Function:    VMin
 *
 * Description: find smallest element in a buffer of uint8_ts
 *
 * Inputs:      buffer of elements to search
 *              number of elements to search
 *
 * Outputs:     none
 *
 * Return:      smallest element in buffer
 **********************************************************************************************************************/
uint8_t VMin(uint8_t *v, int nItems) {

    int i;
    uint8_t vMin;

    vMin = v[0];
    for(i = 1; i < nItems; i++) {
        if(v[i] < vMin) vMin = v[i];
    }
    return vMin;
}
/***********************************************************************************************************************
 * Function:    VMax
 *
 * Description: find largest element in a buffer of uint8_ts
 *
 * Inputs:      buffer of elements to search
 *              number of elements to search
 *
 * Outputs:     none
 *
 * Return:      largest element in buffer
 **********************************************************************************************************************/
uint8_t VMax(uint8_t *v, int nItems) {

    int i;
    uint8_t vMax;

    vMax = v[0];
    for(i = 1; i < nItems; i++) {
        if(v[i] > vMax) vMax = v[i];
    }
    return vMax;
}
/***********************************************************************************************************************
 * Function:    CalcFreqMasterScaleZero
 *
 * Description: calculate master frequency table when freqScale == 0
 *                (4.6.18.3.2.1, figure 4.39)
 *
 * Inputs:      alterScale flag
 *              index of first QMF subband in master freq table (k0)
 *              index of last QMF subband (k2)
 *
 * Outputs:     master frequency table
 *
 * Return:      number of bands in master frequency table
 *
 * Notes:       assumes k2 - k0 <= 48 and k2 >= k0 (4.6.18.3.6)
 **********************************************************************************************************************/
int CalcFreqMasterScaleZero(uint8_t *freqMaster, int alterScale, int k0, int k2) {

    int nMaster, k, nBands, k2Achieved, dk, vDk[64], k2Diff;

    if(alterScale) {
        dk = 2;
        nBands = 2 * ((k2 - k0 + 2) >> 2);
    }
    else {
        dk = 1;
        nBands = 2 * ((k2 - k0) >> 1);
    }

    if(nBands <= 0) return 0;

    k2Achieved = k0 + nBands * dk;
    k2Diff = k2 - k2Achieved;
    for(k = 0; k < nBands; k++)
        vDk[k] = dk;

    if(k2Diff > 0) {
        k = nBands - 1;
        while(k2Diff) {
            vDk[k]++;
            k--;
            k2Diff--;
        }
    }
    else if(k2Diff < 0) {
        k = 0;
        while(k2Diff) {
            vDk[k]--;
            k++;
            k2Diff++;
        }
    }

    nMaster = nBands;
    freqMaster[0] = k0;
    for(k = 1; k <= nBands; k++)
        freqMaster[k] = freqMaster[k - 1] + vDk[k - 1];

    return nMaster;
}

/* mBandTab[i] = temp1[i] / 2 */
static const int mBandTab[3] PROGMEM = {6, 5, 4};

/* invWarpTab[i] = 1.0 / temp2[i], Q30 (see 4.6.18.3.2.1) */
static const int invWarpTab[2] PROGMEM = {0x40000000, 0x313b13b1};

/***********************************************************************************************************************
 * Function:    CalcFreqMasterScale
 *
 * Description: calculate master frequency table when freqScale > 0
 *                (4.6.18.3.2.1, figure 4.39)
 *
 * Inputs:      alterScale flag
 *              freqScale flag
 *              index of first QMF subband in master freq table (k0)
 *              index of last QMF subband (k2)
 *
 * Outputs:     master frequency table
 *
 * Return:      number of bands in master frequency table
 *
 * Notes:       assumes k2 - k0 <= 48 and k2 >= k0 (4.6.18.3.6)
 **********************************************************************************************************************/
int CalcFreqMaster(uint8_t *freqMaster, int freqScale, int alterScale, int k0, int k2) {

    int bands, twoRegions, k, k1, t, vLast, vCurr, pCurr;
    int invWarp, nBands0, nBands1, change;
    uint8_t vDk1Min, vDk0Max;
    uint8_t *vDelta;

    if(freqScale < 1 || freqScale > 3) return -1;

    bands = mBandTab[freqScale - 1];
    invWarp = invWarpTab[alterScale];

    /* tested for all k0 = [5, 64], k2 = [k0, 64] */
    if(k2 * 10000 > 22449 * k0) {
        twoRegions = 1;
        k1 = 2 * k0;
    }
    else {
        twoRegions = 0;
        k1 = k2;
    }

    /* tested for all k0 = [5, 64], k1 = [k0, 64], freqScale = [1,3] */
    t = (log2Tab[k1] - log2Tab[k0]) >> 3; /* log2(k1/k0), Q28 to Q25 */
    nBands0 = 2 * (((bands * t) + (1 << 24)) >> 25); /* multiply by bands/2, round to nearest int (mBandTab has factor of 1/2 rolled in) */

    /* tested for all valid combinations of k0, k1, nBands (from sampRate, freqScale, alterScale)
     * roundoff error can be a problem with fixpt (e.g. pCurr = 12.499999 instead of 12.50003)
     *   because successive multiplication always undershoots a little bit, but this
     *   doesn't occur in any of the ratios we encounter from the valid k0/k1 bands in the spec
     */
    t = RatioPowInv(k1, k0, nBands0);
    pCurr = k0 << 24;
    vLast = k0;
    vDelta = freqMaster + 1; /* operate in-place */
    for(k = 0; k < nBands0; k++) {
        pCurr = MULSHIFT32(pCurr, t) << 8; /* keep in Q24 */
        vCurr = (pCurr + (1 << 23)) >> 24;
        vDelta[k] = (vCurr - vLast);
        vLast = vCurr;
    }

    /* sort the deltas and find max delta for first region */
    BubbleSort(vDelta, nBands0);
    vDk0Max = VMax(vDelta, nBands0);

    /* fill master frequency table with bands from first region */
    freqMaster[0] = k0;
    for(k = 1; k <= nBands0; k++)
        freqMaster[k] += freqMaster[k - 1];

    /* if only one region, then the table is complete */
    if(!twoRegions) return nBands0;

    /* tested for all k1 = [10, 64], k2 = [k0, 64], freqScale = [1,3] */
    t = (log2Tab[k2] - log2Tab[k1]) >> 3; /* log2(k1/k0), Q28 to Q25 */
    t = MULSHIFT32(bands * t, invWarp) << 2; /* multiply by bands/2, divide by warp factor, keep Q25 */
    nBands1 = 2 * ((t + (1 << 24)) >> 25); /* round to nearest int */

    /* see comments above for calculations in first region */
    t = RatioPowInv(k2, k1, nBands1);
    pCurr = k1 << 24;
    vLast = k1;
    vDelta = freqMaster + nBands0 + 1; /* operate in-place */
    for(k = 0; k < nBands1; k++) {
        pCurr = MULSHIFT32(pCurr, t) << 8; /* keep in Q24 */
        vCurr = (pCurr + (1 << 23)) >> 24;
        vDelta[k] = (vCurr - vLast);
        vLast = vCurr;
    }

    /* sort the deltas, adjusting first and last if the second region has smaller deltas than the first */
    vDk1Min = VMin(vDelta, nBands1);
    if(vDk1Min < vDk0Max) {
        BubbleSort(vDelta, nBands1);
        change = vDk0Max - vDelta[0];
        if(change > ((vDelta[nBands1 - 1] - vDelta[0]) >> 1)) change = ((vDelta[nBands1 - 1] - vDelta[0]) >> 1);
        vDelta[0] += change;
        vDelta[nBands1 - 1] -= change;
    }
    BubbleSort(vDelta, nBands1);

    /* fill master frequency table with bands from second region
     * Note: freqMaster[nBands0] = k1
     */
    for(k = 1; k <= nBands1; k++)
        freqMaster[k + nBands0] += freqMaster[k + nBands0 - 1];

    return (nBands0 + nBands1);
}
/***********************************************************************************************************************
 * Function:    CalcFreqHigh
 *
 * Description: calculate high resolution frequency table (4.6.18.3.2.2)
 *
 * Inputs:      master frequency table
 *              number of bands in master frequency table
 *              crossover band from header
 *
 * Outputs:     high resolution frequency table
 *
 * Return:      number of bands in high resolution frequency table
 **********************************************************************************************************************/
int CalcFreqHigh(uint8_t *freqHigh, uint8_t *freqMaster, int nMaster, int crossOverBand) {

    int k, nHigh;

    nHigh = nMaster - crossOverBand;

    for(k = 0; k <= nHigh; k++)
        freqHigh[k] = freqMaster[k + crossOverBand];

    return nHigh;
}
/***********************************************************************************************************************
 * Function:    CalcFreqLow
 *
 * Description: calculate low resolution frequency table (4.6.18.3.2.2)
 *
 * Inputs:      high resolution frequency table
 *              number of bands in high resolution frequency table
 *
 * Outputs:     low resolution frequency table
 *
 * Return:      number of bands in low resolution frequency table
 **********************************************************************************************************************/
int CalcFreqLow(uint8_t *freqLow, uint8_t *freqHigh, int nHigh) {

    int k, nLow, oddFlag;

    nLow = nHigh - (nHigh >> 1);
    freqLow[0] = freqHigh[0];
    oddFlag = nHigh & 0x01;

    for(k = 1; k <= nLow; k++)
        freqLow[k] = freqHigh[2 * k - oddFlag];

    return nLow;
}
/***********************************************************************************************************************
 * Function:    CalcFreqNoise
 *
 * Description: calculate noise floor frequency table (4.6.18.3.2.2)
 *
 * Inputs:      low resolution frequency table
 *              number of bands in low resolution frequency table
 *              index of starting QMF subband for SBR (kStart)
 *              index of last QMF subband (k2)
 *              number of noise bands
 *
 * Outputs:     noise floor frequency table
 *
 * Return:      number of bands in noise floor frequency table
 **********************************************************************************************************************/
int CalcFreqNoise(uint8_t *freqNoise, uint8_t *freqLow, int nLow, int kStart, int k2, int noiseBands) {

    int i, iLast, k, nQ, lTop, lBottom;

    lTop = log2Tab[k2];
    lBottom = log2Tab[kStart];
    nQ = noiseBands * ((lTop - lBottom) >> 2); /* Q28 to Q26, noiseBands = [0,3] */
    nQ = (nQ + (1 << 25)) >> 26;
    if(nQ < 1) nQ = 1;

    ASSERT(nQ <= MAX_NUM_NOISE_FLOOR_BANDS); /* required from 4.6.18.3.6 */

    iLast = 0;
    freqNoise[0] = freqLow[0];
    for(k = 1; k <= nQ; k++) {
        i = iLast + (nLow - iLast) / (nQ + 1 - k); /* truncating division */
        freqNoise[k] = freqLow[i];
        iLast = i;
    }

    return nQ;
}
/***********************************************************************************************************************
 * Function:    BuildPatches
 *
 * Description: build high frequency patches (4.6.18.6.3)
 *
 * Inputs:      master frequency table
 *              number of bands in low resolution frequency table
 *              index of first QMF subband in master freq table (k0)
 *              index of starting QMF subband for SBR (kStart)
 *              number of QMF bands in high resolution frequency table
 *              sample rate index
 *
 * Outputs:     starting subband for each patch
 *              number of subbands in each patch
 *
 * Return:      number of patches
 **********************************************************************************************************************/
int BuildPatches(uint8_t *patchNumSubbands, uint8_t *patchStartSubband, uint8_t *freqMaster, int nMaster, int k0,
        int kStart, int numQMFBands, int sampRateIdx) {

    int i, j, k;
    int msb, sb, usb, numPatches, goalSB, oddFlag;

    msb = k0;
    usb = kStart;
    numPatches = 0;
    goalSB = goalSBTab[sampRateIdx];

    if(nMaster == 0) {
        patchNumSubbands[0] = 0;
        patchStartSubband[0] = 0;
        return 0;
    }

    if(goalSB < kStart + numQMFBands) {
        k = 0;
        for(i = 0; freqMaster[i] < goalSB; i++)
            k = i + 1;
    }
    else {
        k = nMaster;
    }

    do {
        j = k + 1;
        do {
            j--;
            sb = freqMaster[j];
            oddFlag = (sb - 2 + k0) & 0x01;
        } while(sb > k0 - 1 + msb - oddFlag);

        patchNumSubbands[numPatches] = MAX(sb - usb, 0);
        patchStartSubband[numPatches] = k0 - oddFlag - patchNumSubbands[numPatches];

        /* from MPEG reference code - slightly different from spec */
        if((patchNumSubbands[numPatches] < 3) && (numPatches > 0)) break;

        if(patchNumSubbands[numPatches] > 0) {
            usb = sb;
            msb = sb;
            numPatches++;
        }
        else {
            msb = kStart;
        }

        if(freqMaster[k] - sb < 3) k = nMaster;

    } while(sb != (kStart + numQMFBands) && numPatches <= MAX_NUM_PATCHES);

    return numPatches;
}
/***********************************************************************************************************************
 * Function:    FindFreq
 *
 * Description: search buffer of uint8_ts for a specific value
 *
 * Inputs:      buffer of elements to search
 *              number of elements to search
 *              value to search for
 *
 * Outputs:     none
 *
 * Return:      non-zero if the value is found anywhere in the buffer, zero otherwise
 **********************************************************************************************************************/
int FindFreq(uint8_t *freq, int nFreq, uint8_t val) {

    int k;

    for(k = 0; k < nFreq; k++) {
        if(freq[k] == val) return 1;
    }

    return 0;
}
/***********************************************************************************************************************
 * Function:    RemoveFreq
 *
 * Description: remove one element from a buffer of uint8_ts
 *
 * Inputs:      buffer of elements
 *              number of elements
 *              index of element to remove
 *
 * Outputs:     new buffer of length nFreq-1
 *
 * Return:      none
 **********************************************************************************************************************/
void RemoveFreq(uint8_t *freq, int nFreq, int removeIdx) {

    int k;

    if(removeIdx >= nFreq) return;

    for(k = removeIdx; k < nFreq - 1; k++)
        freq[k] = freq[k + 1];
}
/***********************************************************************************************************************
 * Function:    CalcFreqLimiter
 *
 * Description: calculate limiter frequency table (4.6.18.3.2.3)
 *
 * Inputs:      number of subbands in each patch
 *              low resolution frequency table
 *              number of bands in low resolution frequency table
 *              index of starting QMF subband for SBR (kStart)
 *              number of limiter bands
 *              number of patches
 *
 * Outputs:     limiter frequency table
 *
 * Return:      number of bands in limiter frequency table
 **********************************************************************************************************************/
int CalcFreqLimiter(uint8_t *freqLimiter, uint8_t *patchNumSubbands, uint8_t *freqLow, int nLow, int kStart,
        int limiterBands, int numPatches) {

    int k, bands, nLimiter, nOctaves;
    int limBandsPerOctave[3] = { 120, 200, 300 }; /* [1.2, 2.0, 3.0] * 100 */
    uint8_t patchBorders[MAX_NUM_PATCHES + 1];

    /* simple case */
    if(limiterBands == 0) {
        freqLimiter[0] = freqLow[0] - kStart;
        freqLimiter[1] = freqLow[nLow] - kStart;
        return 1;
    }

    bands = limBandsPerOctave[limiterBands - 1];
    patchBorders[0] = kStart;

    /* from MPEG reference code - slightly different from spec (top border) */
    for(k = 1; k < numPatches; k++)
        patchBorders[k] = patchBorders[k - 1] + patchNumSubbands[k - 1];
    patchBorders[k] = freqLow[nLow];

    for(k = 0; k <= nLow; k++)
        freqLimiter[k] = freqLow[k];

    for(k = 1; k < numPatches; k++)
        freqLimiter[k + nLow] = patchBorders[k];

    k = 1;
    nLimiter = nLow + numPatches - 1;
    BubbleSort(freqLimiter, nLimiter + 1);

    while(k <= nLimiter) {
        nOctaves = log2Tab[freqLimiter[k]] - log2Tab[freqLimiter[k - 1]]; /* Q28 */
        nOctaves = (nOctaves >> 9) * bands; /* Q19, max bands = 300 < 2^9 */
        if(nOctaves < (49 << 19)) { /* compare with 0.49*100, in Q19 */
            if(freqLimiter[k] == freqLimiter[k - 1] || FindFreq(patchBorders, numPatches + 1, freqLimiter[k]) == 0) {
                RemoveFreq(freqLimiter, nLimiter + 1, k);
                nLimiter--;
            }
            else if(FindFreq(patchBorders, numPatches + 1, freqLimiter[k - 1]) == 0) {
                RemoveFreq(freqLimiter, nLimiter + 1, k - 1);
                nLimiter--;
            }
            else {
                k++;
            }
        }
        else {
            k++;
        }
    }

    /* store limiter boundaries as offsets from kStart */
    for(k = 0; k <= nLimiter; k++)
        freqLimiter[k] -= kStart;

    return nLimiter;
}
/***********************************************************************************************************************
 * Function:    CalcFreqTables
 *
 * Description: calulate master and derived frequency tables, and patches
 *
 * Inputs:      initialized SBRHeader struct for this SCE/CPE block
 *              initialized SBRFreq struct for this SCE/CPE block
 *              sample rate index of output sample rate (after SBR)
 *
 * Outputs:     master and derived frequency tables, and patches
 *
 * Return:      non-zero if error, zero otherwise
 **********************************************************************************************************************/
int CalcFreqTables(SBRHeader *sbrHdr, SBRFreq *sbrFreq, int sampRateIdx) {
    int k0, k2;

    k0 = k0Tab[sampRateIdx][sbrHdr->startFreq];

    if(sbrHdr->stopFreq == 14)
        k2 = 2 * k0;
    else if(sbrHdr->stopFreq == 15)
        k2 = 3 * k0;
    else
        k2 = k2Tab[sampRateIdx][sbrHdr->stopFreq];
    if(k2 > 64) k2 = 64;

    /* calculate master frequency table */
    if(sbrHdr->freqScale == 0)
        sbrFreq->nMaster = CalcFreqMasterScaleZero(sbrFreq->freqMaster, sbrHdr->alterScale, k0, k2);
    else
        sbrFreq->nMaster = CalcFreqMaster(sbrFreq->freqMaster, sbrHdr->freqScale, sbrHdr->alterScale, k0, k2);

    /* calculate high frequency table and related parameters */
    sbrFreq->nHigh = CalcFreqHigh(sbrFreq->freqHigh, sbrFreq->freqMaster, sbrFreq->nMaster, sbrHdr->crossOverBand);
    sbrFreq->numQMFBands = sbrFreq->freqHigh[sbrFreq->nHigh] - sbrFreq->freqHigh[0];
    sbrFreq->kStart = sbrFreq->freqHigh[0];

    /* calculate low frequency table */
    sbrFreq->nLow = CalcFreqLow(sbrFreq->freqLow, sbrFreq->freqHigh, sbrFreq->nHigh);

    /* calculate noise floor frequency table */
    sbrFreq->numNoiseFloorBands = CalcFreqNoise(sbrFreq->freqNoise, sbrFreq->freqLow, sbrFreq->nLow, sbrFreq->kStart,
            k2, sbrHdr->noiseBands);

    /* calculate limiter table */
    sbrFreq->numPatches = BuildPatches(sbrFreq->patchNumSubbands, sbrFreq->patchStartSubband, sbrFreq->freqMaster,
            sbrFreq->nMaster, k0, sbrFreq->kStart, sbrFreq->numQMFBands, sampRateIdx);
    sbrFreq->nLimiter = CalcFreqLimiter(sbrFreq->freqLimiter, sbrFreq->patchNumSubbands, sbrFreq->freqLow,
            sbrFreq->nLow, sbrFreq->kStart, sbrHdr->limiterBands, sbrFreq->numPatches);

    return 0;
}
/***********************************************************************************************************************
 * Function:    EstimateEnvelope
 *
 * Description: estimate power of generated HF QMF bands in one time-domain envelope
 *                (4.6.18.7.3)
 *
 * Inputs:      initialized PSInfoSBR struct
 *              initialized SBRHeader struct for this SCE/CPE block
 *              initialized SBRGrid struct for this channel
 *              initialized SBRFreq struct for this SCE/CPE block
 *              index of current envelope
 *
 * Outputs:     power of each QMF subband, stored as integer (Q0) * 2^N, N >= 0
 *
 * Return:      none
 **********************************************************************************************************************/
void EstimateEnvelope(SBRHeader *sbrHdr, SBRGrid *sbrGrid, SBRFreq *sbrFreq, int env) {

    int i, m, iStart, iEnd, xre, xim, nScale, expMax;
    int p, n, mStart, mEnd, invFact, t;
    int *XBuf;
    U64 eCurr;
    uint8_t *freqBandTab;

    /* estimate current envelope */
    iStart = sbrGrid->envTimeBorder[env] + HF_ADJ;
    iEnd = sbrGrid->envTimeBorder[env + 1] + HF_ADJ;
    if(sbrGrid->freqRes[env]) {
        n = sbrFreq->nHigh;
        freqBandTab = sbrFreq->freqHigh;
    }
    else {
        n = sbrFreq->nLow;
        freqBandTab = sbrFreq->freqLow;
    }

    /* ADS should inline MADD64 (smlal) properly, but check to make sure */
    expMax = 0;
    if(sbrHdr->interpFreq) {
        for(m = 0; m < sbrFreq->numQMFBands; m++) {
            eCurr.w64 = 0;
            XBuf = m_PSInfoSBR->XBuf[iStart][sbrFreq->kStart + m];
            for(i = iStart; i < iEnd; i++) {
                /* scale to int before calculating power (precision not critical, and avoids overflow) */
                xre = (*XBuf) >> FBITS_OUT_QMFA;
                XBuf += 1;
                xim = (*XBuf) >> FBITS_OUT_QMFA;
                XBuf += (2 * 64 - 1);
                eCurr.w64 = MADD64(eCurr.w64, xre, xre);
                eCurr.w64 = MADD64(eCurr.w64, xim, xim);
            }

            /* eCurr.w64 is now Q(64 - 2*FBITS_OUT_QMFA) (64-bit word)
             * if energy is too big to fit in 32-bit word (> 2^31) scale down by power of 2
             */
            nScale = 0;
            if(eCurr.r.hi32) {
                nScale = (32 - CLZ(eCurr.r.hi32)) + 1;
                t = (int) (eCurr.r.lo32 >> nScale); /* logical (unsigned) >> */
                t |= eCurr.r.hi32 << (32 - nScale);
            }
            else if(eCurr.r.lo32 >> 31) {
                nScale = 1;
                t = (int) (eCurr.r.lo32 >> nScale); /* logical (unsigned) >> */
            }
            else {
                t = (int) eCurr.r.lo32;
            }

            invFact = invBandTab[(iEnd - iStart) - 1];
            m_PSInfoSBR->eCurr[m] = MULSHIFT32(t, invFact);
            m_PSInfoSBR->eCurrExp[m] = nScale + 1; /* +1 for invFact = Q31 */
            if(m_PSInfoSBR->eCurrExp[m] > expMax) expMax = m_PSInfoSBR->eCurrExp[m];
        }
    }
    else {
        for(p = 0; p < n; p++) {
            mStart = freqBandTab[p];
            mEnd = freqBandTab[p + 1];
            eCurr.w64 = 0;
            for(i = iStart; i < iEnd; i++) {
                XBuf = m_PSInfoSBR->XBuf[i][mStart];
                for(m = mStart; m < mEnd; m++) {
                    xre = (*XBuf++) >> FBITS_OUT_QMFA;
                    xim = (*XBuf++) >> FBITS_OUT_QMFA;
                    eCurr.w64 = MADD64(eCurr.w64, xre, xre);
                    eCurr.w64 = MADD64(eCurr.w64, xim, xim);
                }
            }

            nScale = 0;
            if(eCurr.r.hi32) {
                nScale = (32 - CLZ(eCurr.r.hi32)) + 1;
                t = (int) (eCurr.r.lo32 >> nScale); /* logical (unsigned) >> */
                t |= eCurr.r.hi32 << (32 - nScale);
            }
            else if(eCurr.r.lo32 >> 31) {
                nScale = 1;
                t = (int) (eCurr.r.lo32 >> nScale); /* logical (unsigned) >> */
            }
            else {
                t = (int) eCurr.r.lo32;
            }

            invFact = invBandTab[(iEnd - iStart) - 1];
            invFact = MULSHIFT32(invBandTab[(mEnd - mStart) - 1], invFact) << 1;
            t = MULSHIFT32(t, invFact);

            for(m = mStart; m < mEnd; m++) {
                m_PSInfoSBR->eCurr[m - sbrFreq->kStart] = t;
                m_PSInfoSBR->eCurrExp[m - sbrFreq->kStart] = nScale + 1; /* +1 for invFact = Q31 */
            }
            if(m_PSInfoSBR->eCurrExp[mStart - sbrFreq->kStart] > expMax)
                expMax = m_PSInfoSBR->eCurrExp[mStart - sbrFreq->kStart];
        }
    }
    m_PSInfoSBR->eCurrExpMax = expMax;
}
/***********************************************************************************************************************
 * Function:    GetSMapped
 *
 * Description: calculate SMapped (4.6.18.7.2)
 *
 * Inputs:      initialized SBRGrid struct for this channel
 *              initialized SBRFreq struct for this SCE/CPE block
 *              initialized SBRChan struct for this channel
 *              index of current envelope
 *              index of current QMF band
 *              la flag for this envelope
 *
 * Outputs:     none
 *
 * Return:      1 if a sinusoid is present in this band, 0 if not
 **********************************************************************************************************************/
int GetSMapped(SBRGrid *sbrGrid, SBRFreq *sbrFreq, SBRChan *sbrChan, int env, int band, int la) {

    int bandStart, bandEnd, oddFlag, r;

    if (sbrGrid->freqRes[env]) {
        /* high resolution */
        bandStart = band;
        bandEnd = band+1;
    } else {
        /* low resolution (see CalcFreqLow() for mapping) */
        oddFlag = sbrFreq->nHigh & 0x01;
        bandStart = (band > 0 ? 2*band - oddFlag : 0);      /* starting index for freqLow[band] */
        bandEnd = 2*(band+1) - oddFlag;                     /* ending index for freqLow[band+1] */
    }

    /* sMapped = 1 if sIndexMapped == 1 for any frequency in this band */
    for (band = bandStart; band < bandEnd; band++) {
        if (sbrChan->addHarmonic[1][band]) {
            r = ((sbrFreq->freqHigh[band+1] + sbrFreq->freqHigh[band]) >> 1);
            if (env >= la || sbrChan->addHarmonic[0][r] == 1)
                return 1;
        }
    }
    return 0;
}

#define GBOOST_MAX  0x2830afd3  /* Q28, 1.584893192 squared */
#define ACC_SCALE   6

/* squared version of table in 4.6.18.7.5 */   /* Q30 (0x80000000 = sentinel for GMAX) */
static const uint32_t limGainTab[4] PROGMEM = {0x20138ca7, 0x40000000, 0x7fb27dce, 0x80000000};

/***********************************************************************************************************************
 * Function:    CalcMaxGain
 *
 * Description: calculate max gain in one limiter band (4.6.18.7.5)
 *
 * Inputs:      initialized SBRHeader struct for this SCE/CPE block
 *              initialized SBRGrid struct for this channel
 *              initialized SBRFreq struct for this SCE/CPE block
 *              index of current channel (0 for SCE, 0 or 1 for CPE)
 *              index of current envelope
 *              index of current limiter band
 *              number of fraction bits in dequantized envelope
 *                (max = Q(FBITS_OUT_DQ_ENV - 6) = Q23, can go negative)
 *
 * Outputs:     updated gainMax, gainMaxFBits, and sumEOrigMapped in PSInfoSBR struct
 *
 * Return:      none
 **********************************************************************************************************************/
void CalcMaxGain(SBRHeader *sbrHdr, SBRGrid *sbrGrid, SBRFreq *sbrFreq, int ch, int env, int lim, int fbitsDQ) {

    int m, mStart, mEnd, q, z, r;
    int sumEOrigMapped, sumECurr, gainMax, eOMGainMax, envBand;
    uint8_t eCurrExpMax;
    uint8_t *freqBandTab;

    mStart = sbrFreq->freqLimiter[lim]; /* these are offsets from kStart */
    mEnd = sbrFreq->freqLimiter[lim + 1];
    freqBandTab = (sbrGrid->freqRes[env] ? sbrFreq->freqHigh : sbrFreq->freqLow);

    /* calculate max gain to apply to signal in this limiter band */
    sumECurr = 0;
    sumEOrigMapped = 0;
    eCurrExpMax = m_PSInfoSBR->eCurrExpMax;
    eOMGainMax = m_PSInfoSBR->eOMGainMax;
    envBand = m_PSInfoSBR->envBand;
    for(m = mStart; m < mEnd; m++) {
        /* map current QMF band to appropriate envelope band */
        if(m == freqBandTab[envBand + 1] - sbrFreq->kStart) {
            envBand++;
            eOMGainMax = m_PSInfoSBR->envDataDequant[ch][env][envBand] >> ACC_SCALE; /* summing max 48 bands */
        }
        sumEOrigMapped += eOMGainMax;

        /* easy test for overflow on ARM */
        sumECurr += (m_PSInfoSBR->eCurr[m] >> (eCurrExpMax - m_PSInfoSBR->eCurrExp[m]));
        if(sumECurr >> 30) {
            sumECurr >>= 1;
            eCurrExpMax++;
        }
    }
    m_PSInfoSBR->eOMGainMax = eOMGainMax;
    m_PSInfoSBR->envBand = envBand;

    m_PSInfoSBR->gainMaxFBits = 30; /* Q30 tables */
    if(sumECurr == 0) {
        /* any non-zero numerator * 1/EPS_0 is > G_MAX */
        gainMax = (sumEOrigMapped == 0 ? (int) limGainTab[sbrHdr->limiterGains] : (int) 0x80000000);
    }
    else if(sumEOrigMapped == 0) {
        /* 1/(any non-zero denominator) * EPS_0 * limGainTab[x] is appx. 0 */
        gainMax = 0;
    }
    else {
        /* sumEOrigMapped = Q(fbitsDQ - ACC_SCALE), sumECurr = Q(-eCurrExpMax) */
        gainMax = limGainTab[sbrHdr->limiterGains];
        if(sbrHdr->limiterGains != 3) {
            q = MULSHIFT32(sumEOrigMapped, gainMax); /* Q(fbitsDQ - ACC_SCALE - 2), gainMax = Q30  */
            z = CLZ(sumECurr) - 1;
            r = InvRNormalized(sumECurr << z); /* in =  Q(z - eCurrExpMax), out = Q(29 + 31 - z + eCurrExpMax) */
            gainMax = MULSHIFT32(q, r); /* Q(29 + 31 - z + eCurrExpMax + fbitsDQ - ACC_SCALE - 2 - 32) */
            m_PSInfoSBR->gainMaxFBits = 26 - z + eCurrExpMax + fbitsDQ - ACC_SCALE;
        }
    }
    m_PSInfoSBR->sumEOrigMapped = sumEOrigMapped;
    m_PSInfoSBR->gainMax = gainMax;
}
/***********************************************************************************************************************
 * Function:    CalcNoiseDivFactors
 *
 * Description: calculate 1/(1+Q) and Q/(1+Q) (4.6.18.7.4; 4.6.18.7.5)
 *
 * Inputs:      dequantized noise floor scalefactor
 *
 * Outputs:     1/(1+Q) and Q/(1+Q), format = Q31
 *
 * Return:      none
 **********************************************************************************************************************/
void CalcNoiseDivFactors(int q, int *qp1Inv, int *qqp1Inv) {

    int z, qp1, t, s;

    /* 1 + Q_orig */
    qp1 = (q >> 1);
    qp1 += (1 << (FBITS_OUT_DQ_NOISE - 1)); /* >> 1 to avoid overflow when adding 1.0 */
    z = CLZ(qp1) - 1; /* z <= 31 - FBITS_OUT_DQ_NOISE */
    qp1 <<= z; /* Q(FBITS_OUT_DQ_NOISE + z) = Q31 * 2^-(31 - (FBITS_OUT_DQ_NOISE + z)) */
    t = InvRNormalized(qp1) << 1; /* Q30 * 2^(31 - (FBITS_OUT_DQ_NOISE + z)), guaranteed not to overflow */

    /* normalize to Q31 */
    s = (31 - (FBITS_OUT_DQ_NOISE - 1) - z - 1); /* clearly z >= 0, z <= (30 - (FBITS_OUT_DQ_NOISE - 1)) */
    *qp1Inv = (t >> s); /* s = [0, 31 - FBITS_OUT_DQ_NOISE] */
    *qqp1Inv = MULSHIFT32(t, q) << (32 - FBITS_OUT_DQ_NOISE - s);
}
/***********************************************************************************************************************
 * Function:    CalcComponentGains
 *
 * Description: calculate gain of envelope, sinusoids, and noise in one limiter band
 *                (4.6.18.7.5)
 *
 * Inputs:      initialized SBRHeader struct for this SCE/CPE block
 *              initialized SBRGrid struct for this channel
 *              initialized SBRFreq struct for this SCE/CPE block
 *              initialized SBRChan struct for this channel
 *              index of current channel (0 for SCE, 0 or 1 for CPE)
 *              index of current envelope
 *              index of current limiter band
 *              number of fraction bits in dequantized envelope
 *
 * Outputs:     gains for envelope, sinusoids and noise
 *              number of fraction bits for envelope gain
 *              sum of the total gain for each component in this band
 *              other updated state variables
 *
 * Return:      none
 **********************************************************************************************************************/
void CalcComponentGains(SBRGrid *sbrGrid, SBRFreq *sbrFreq, SBRChan *sbrChan, int ch, int env, int lim, int fbitsDQ) {

    int d, m, mStart, mEnd, q, qm, noiseFloor, sIndexMapped;
    int shift, eCurr, maxFlag, gainMax, gainMaxFBits;
    int gain, sm, z, r, fbitsGain, gainScale;
    uint8_t *freqBandTab;

    mStart = sbrFreq->freqLimiter[lim]; /* these are offsets from kStart */
    mEnd = sbrFreq->freqLimiter[lim + 1];

    gainMax = m_PSInfoSBR->gainMax;
    gainMaxFBits = m_PSInfoSBR->gainMaxFBits;

    d = (env == m_PSInfoSBR->la || env == sbrChan->laPrev ? 0 : 1);
    freqBandTab = (sbrGrid->freqRes[env] ? sbrFreq->freqHigh : sbrFreq->freqLow);

    /* figure out which noise floor this envelope is in (only 1 or 2 noise floors allowed) */
    noiseFloor = 0;
    if(sbrGrid->numNoiseFloors == 2 && sbrGrid->noiseTimeBorder[1] <= sbrGrid->envTimeBorder[env]) noiseFloor++;

    m_PSInfoSBR->sumECurrGLim = 0;
    m_PSInfoSBR->sumSM = 0;
    m_PSInfoSBR->sumQM = 0;
    /* calculate energy of noise to add in this limiter band */
    for(m = mStart; m < mEnd; m++) {
        if(m == sbrFreq->freqNoise[m_PSInfoSBR->noiseFloorBand + 1] - sbrFreq->kStart) {
            /* map current QMF band to appropriate noise floor band (NOTE: freqLimiter[0] == freqLow[0] = freqHigh[0]) */
            m_PSInfoSBR->noiseFloorBand++;
            CalcNoiseDivFactors(m_PSInfoSBR->noiseDataDequant[ch][noiseFloor][m_PSInfoSBR->noiseFloorBand],
                    &(m_PSInfoSBR->qp1Inv), &(m_PSInfoSBR->qqp1Inv));
        }
        if(m == sbrFreq->freqHigh[m_PSInfoSBR->highBand + 1] - sbrFreq->kStart) m_PSInfoSBR->highBand++;
        if(m == freqBandTab[m_PSInfoSBR->sBand + 1] - sbrFreq->kStart) {
            m_PSInfoSBR->sBand++;
            m_PSInfoSBR->sMapped = GetSMapped(sbrGrid, sbrFreq, sbrChan, env, m_PSInfoSBR->sBand, m_PSInfoSBR->la);
        }

        /* get sIndexMapped for this QMF subband */
        sIndexMapped = 0;
        r = ((sbrFreq->freqHigh[m_PSInfoSBR->highBand + 1] + sbrFreq->freqHigh[m_PSInfoSBR->highBand]) >> 1);
        if(m + sbrFreq->kStart == r) {
            /* r = center frequency, deltaStep = (env >= la || sIndexMapped'(r, numEnv'-1) == 1) */
            if(env >= m_PSInfoSBR->la || sbrChan->addHarmonic[0][r] == 1) sIndexMapped =
                    sbrChan->addHarmonic[1][m_PSInfoSBR->highBand];
        }

        /* save sine flags from last envelope in this frame:
         *   addHarmonic[0][0...63] = saved sine present flag from previous frame, for each QMF subband
         *   addHarmonic[1][0...nHigh-1] = addHarmonic bit from current frame, for each high-res frequency band
         * from MPEG reference code - slightly different from spec
         *   (sIndexMapped'(m,LE'-1) can still be 0 when numEnv == psi->la)
         */
        if(env == sbrGrid->numEnv - 1) {
            if(m + sbrFreq->kStart == r)
                sbrChan->addHarmonic[0][m + sbrFreq->kStart] = sbrChan->addHarmonic[1][m_PSInfoSBR->highBand];
            else
                sbrChan->addHarmonic[0][m + sbrFreq->kStart] = 0;
        }

        gain = m_PSInfoSBR->envDataDequant[ch][env][m_PSInfoSBR->sBand];
        qm = MULSHIFT32(gain, m_PSInfoSBR->qqp1Inv) << 1;
        sm = (sIndexMapped ? MULSHIFT32(gain, m_PSInfoSBR->qp1Inv) << 1 : 0);

        /* three cases: (sMapped == 0 && delta == 1), (sMapped == 0 && delta == 0), (sMapped == 1) */
        if(d == 1 && m_PSInfoSBR->sMapped == 0)
            gain = MULSHIFT32(m_PSInfoSBR->qp1Inv, gain) << 1;
        else if(m_PSInfoSBR->sMapped != 0) gain = MULSHIFT32(m_PSInfoSBR->qqp1Inv, gain) << 1;

        /* gain, qm, sm = Q(fbitsDQ), gainMax = Q(fbitsGainMax) */
        eCurr = m_PSInfoSBR->eCurr[m];
        if(eCurr) {
            z = CLZ(eCurr) - 1;
            r = InvRNormalized(eCurr << z); /* in = Q(z - eCurrExp), out = Q(29 + 31 - z + eCurrExp) */
            gainScale = MULSHIFT32(gain, r); /* out = Q(29 + 31 - z + eCurrExp + fbitsDQ - 32) */
            fbitsGain = 29 + 31 - z + m_PSInfoSBR->eCurrExp[m] + fbitsDQ - 32;
        }
        else {
            /* if eCurr == 0, then gain is unchanged (divide by EPS = 1) */
            gainScale = gain;
            fbitsGain = fbitsDQ;
        }

        /* see if gain for this band exceeds max gain */
        maxFlag = 0;
        if(gainMax != (int) 0x80000000) {
            if(fbitsGain >= gainMaxFBits) {
                shift = MIN(fbitsGain - gainMaxFBits, 31);
                maxFlag = ((gainScale >> shift) > gainMax ? 1 : 0);
            }
            else {
                shift = MIN(gainMaxFBits - fbitsGain, 31);
                maxFlag = (gainScale > (gainMax >> shift) ? 1 : 0);
            }
        }

        if(maxFlag) {
            /* gainScale > gainMax, calculate ratio with 32/16 division */
            q = 0;
            r = gainScale; /* guaranteed > 0, else maxFlag could not have been set */
            z = CLZ(r);
            if(z < 16) {
                q = 16 - z;
                r >>= q; /* out = Q(fbitsGain - q) */
            }

            z = CLZ(gainMax) - 1;
            r = (gainMax << z) / r; /* out = Q((fbitsGainMax + z) - (fbitsGain - q)) */
            q = (gainMaxFBits + z) - (fbitsGain - q); /* r = Q(q) */
            if(q > 30) {
                r >>= MIN(q - 30, 31);
            }
            else {
                z = MIN(30 - q, 30);
                r = CLIP_2N_SHIFT30(r, z); /* let r = Q30 since range = [0.0, 1.0) (clip to 0x3fffffff = 0.99999) */
            }

            qm = MULSHIFT32(qm, r) << 2;
            gain = MULSHIFT32(gain, r) << 2;
            m_PSInfoSBR->gLimBuf[m] = gainMax;
            m_PSInfoSBR->gLimFbits[m] = gainMaxFBits;
        }
        else {
            m_PSInfoSBR->gLimBuf[m] = gainScale;
            m_PSInfoSBR->gLimFbits[m] = fbitsGain;
        }

        /* sumSM, sumQM, sumECurrGLim = Q(fbitsDQ - ACC_SCALE) */
        m_PSInfoSBR->smBuf[m] = sm;
        m_PSInfoSBR->sumSM += (sm >> ACC_SCALE);

        m_PSInfoSBR->qmLimBuf[m] = qm;
        if(env != m_PSInfoSBR->la && env != sbrChan->laPrev && sm == 0) m_PSInfoSBR->sumQM += (qm >> ACC_SCALE);

        /* eCurr * gain^2 same as gain^2, before division by eCurr
         * (but note that gain != 0 even if eCurr == 0, since it's divided by eps)
         */
        if(eCurr) m_PSInfoSBR->sumECurrGLim += (gain >> ACC_SCALE);
    }
}
/***********************************************************************************************************************
 * Function:    ApplyBoost
 *
 * Description: calculate and apply boost factor for envelope, sinusoids, and noise
 *                in this limiter band (4.6.18.7.5)
 *
 * Inputs:      initialized SBRFreq struct for this SCE/CPE block
 *              index of current limiter band
 *              number of fraction bits in dequantized envelope
 *
 * Outputs:     envelope gain, sinusoids and noise after scaling by gBoost
 *              format = Q(FBITS_GLIM_BOOST) for envelope gain,
 *                     = Q(FBITS_QLIM_BOOST) for noise
 *                     = Q(FBITS_OUT_QMFA) for sinusoids
 *
 * Return:      none
 *
 * Notes:       after scaling, each component has at least 1 GB
 **********************************************************************************************************************/
void ApplyBoost(SBRFreq *sbrFreq, int lim, int fbitsDQ) {

    int m, mStart, mEnd, q, z, r;
    int sumEOrigMapped, gBoost;

    mStart = sbrFreq->freqLimiter[lim]; /* these are offsets from kStart */
    mEnd = sbrFreq->freqLimiter[lim + 1];

    sumEOrigMapped = m_PSInfoSBR->sumEOrigMapped >> 1;
    r = (m_PSInfoSBR->sumECurrGLim >> 1) + (m_PSInfoSBR->sumSM >> 1) + (m_PSInfoSBR->sumQM >> 1); /* 1 GB fine (sm and qm are mutually exclusive in acc) */
    if(r < (1 << (31 - 28))) {
        /* any non-zero numerator * 1/EPS_0 is > GBOOST_MAX
         * round very small r to zero to avoid scaling problems
         */
        gBoost = (sumEOrigMapped == 0 ? (1 << 28) : GBOOST_MAX);
        z = 0;
    }
    else if(sumEOrigMapped == 0) {
        /* 1/(any non-zero denominator) * EPS_0 is appx. 0 */
        gBoost = 0;
        z = 0;
    }
    else {
        /* numerator (sumEOrigMapped) and denominator (r) have same Q format (before << z) */
        z = CLZ(r) - 1; /* z = [0, 27] */
        r = InvRNormalized(r << z);
        gBoost = MULSHIFT32(sumEOrigMapped, r);
    }

    /* gBoost = Q(28 - z) */
    if(gBoost > (GBOOST_MAX >> z)) {
        gBoost = GBOOST_MAX;
        z = 0;
    }
    gBoost <<= z; /* gBoost = Q28, minimum 1 GB */

    /* convert gain, noise, sinusoids to fixed Q format, clipping if necessary
     *   (rare, usually only happens at very low bitrates, introduces slight
     *    distortion into final HF mapping, but should be inaudible)
     */
    for(m = mStart; m < mEnd; m++) {
        /* let gLimBoost = Q24, since in practice the max values are usually 16 to 20
         *   unless limiterGains == 3 (limiter off) and eCurr ~= 0 (i.e. huge gain, but only
         *   because the envelope has 0 power anyway)
         */
        q = MULSHIFT32(m_PSInfoSBR->gLimBuf[m], gBoost) << 2; /* Q(gLimFbits) * Q(28) --> Q(gLimFbits[m]-2) */
        r = SqrtFix(q, m_PSInfoSBR->gLimFbits[m] - 2, &z);
        z -= FBITS_GLIM_BOOST;
        if(z >= 0) {
            m_PSInfoSBR->gLimBoost[m] = r >> MIN(z, 31);
        }
        else {
            z = MIN(30, -z);
            r = CLIP_2N_SHIFT30(r, z);
            m_PSInfoSBR->gLimBoost[m] = r;
        }

        q = MULSHIFT32(m_PSInfoSBR->qmLimBuf[m], gBoost) << 2; /* Q(fbitsDQ) * Q(28) --> Q(fbitsDQ-2) */
        r = SqrtFix(q, fbitsDQ - 2, &z);
        z -= FBITS_QLIM_BOOST; /* << by 14, since integer sqrt of x < 2^16, and we want to leave 1 GB */
        if(z >= 0) {
            m_PSInfoSBR->qmLimBoost[m] = r >> MIN(31, z);
        }
        else {
            z = MIN(30, -z);
            r = CLIP_2N_SHIFT30(r, z);
            m_PSInfoSBR->qmLimBoost[m] = r;
        }

        q = MULSHIFT32(m_PSInfoSBR->smBuf[m], gBoost) << 2; /* Q(fbitsDQ) * Q(28) --> Q(fbitsDQ-2) */
        r = SqrtFix(q, fbitsDQ - 2, &z);
        z -= FBITS_OUT_QMFA; /* justify for adding to signal (xBuf) later */
        if(z >= 0) {
            m_PSInfoSBR->smBoost[m] = r >> MIN(31, z);
        }
        else {
            z = MIN(30, -z);
            r = CLIP_2N_SHIFT30(r, z);
            m_PSInfoSBR->smBoost[m] = r;
        }
    }
}
/***********************************************************************************************************************
 * Function:    CalcGain
 *
 * Description: calculate and apply proper gain to HF components in one envelope
 *                (4.6.18.7.5)
 *
 * Inputs:      initialized SBRHeader struct for this SCE/CPE block
 *              initialized SBRGrid struct for this channel
 *              initialized SBRFreq struct for this SCE/CPE block
 *              initialized SBRChan struct for this channel
 *              index of current channel (0 for SCE, 0 or 1 for CPE)
 *              index of current envelope
 *
 * Outputs:     envelope gain, sinusoids and noise after scaling
 *
 * Return:      none
 **********************************************************************************************************************/
void CalcGain(SBRHeader *sbrHdr, SBRGrid *sbrGrid, SBRFreq *sbrFreq, SBRChan *sbrChan, int ch, int env) {

    int lim, fbitsDQ;

    /* initialize to -1 so that mapping limiter bands to env/noise bands works right on first pass */
    m_PSInfoSBR->envBand = -1;
    m_PSInfoSBR->noiseFloorBand = -1;
    m_PSInfoSBR->sBand = -1;
    m_PSInfoSBR->highBand = -1;

    fbitsDQ = (FBITS_OUT_DQ_ENV - m_PSInfoSBR->envDataDequantScale[ch][env]); /* Q(29 - optional scalefactor) */
    for(lim = 0; lim < sbrFreq->nLimiter; lim++) {
        /* the QMF bands are divided into lim regions (consecutive, non-overlapping) */
        CalcMaxGain(sbrHdr, sbrGrid, sbrFreq, ch, env, lim, fbitsDQ);
        CalcComponentGains(sbrGrid, sbrFreq, sbrChan, ch, env, lim, fbitsDQ);
        ApplyBoost(sbrFreq, lim, fbitsDQ);
    }
}

/* hSmooth table from 4.7.18.7.6, format = Q31 */
static const int hSmoothCoef[MAX_NUM_SMOOTH_COEFS] PROGMEM = { 0x2aaaaaab, 0x2697a512, 0x1becfa68, 0x0ebdb043,
        0x04130598, };

/***********************************************************************************************************************
 * Function:    MapHF
 *
 * Description: map HF components to proper QMF bands, with optional gain smoothing
 *                filter (4.6.18.7.6)
 *
 * Inputs:      initialized SBRHeader struct for this SCE/CPE block
 *              initialized SBRGrid struct for this channel
 *              initialized SBRFreq struct for this SCE/CPE block
 *              initialized SBRChan struct for this channel
 *              index of current envelope
 *              reset flag (can be non-zero for first envelope only)
 *
 * Outputs:     complete reconstructed subband QMF samples for this envelope
 *
 * Return:      none
 *
 * Notes:       ensures that output has >= MIN_GBITS_IN_QMFS guard bits,
 *                so it's not necessary to check anything in the synth QMF
 **********************************************************************************************************************/
void MapHF(SBRHeader *sbrHdr, SBRGrid *sbrGrid, SBRFreq *sbrFreq, SBRChan *sbrChan, int env, int hfReset) {

    int noiseTabIndex, sinIndex, gainNoiseIndex, hSL;
    int i, iStart, iEnd, m, idx, j, s, n, smre, smim;
    int gFilt, qFilt, xre, xim, gbMask, gbIdx;
    int *XBuf;

    noiseTabIndex = sbrChan->noiseTabIndex;
    sinIndex = sbrChan->sinIndex;
    gainNoiseIndex = sbrChan->gainNoiseIndex; /* oldest entries in filter delay buffer */

    if(hfReset) noiseTabIndex = 2; /* starts at 1, double since complex */
    hSL = (sbrHdr->smoothMode ? 0 : 4);

    if(hfReset) {
        for(i = 0; i < hSL; i++) {
            for(m = 0; m < sbrFreq->numQMFBands; m++) {
                sbrChan->gTemp[gainNoiseIndex][m] = m_PSInfoSBR->gLimBoost[m];
                sbrChan->qTemp[gainNoiseIndex][m] = m_PSInfoSBR->qmLimBoost[m];
            }
            gainNoiseIndex++;
            if(gainNoiseIndex == MAX_NUM_SMOOTH_COEFS) gainNoiseIndex = 0;
        } ASSERT(env == 0); /* should only be reset when env == 0 */
    }

    iStart = sbrGrid->envTimeBorder[env];
    iEnd = sbrGrid->envTimeBorder[env + 1];
    for(i = iStart; i < iEnd; i++) {
        /* save new values in temp buffers (delay)
         * we only store MAX_NUM_SMOOTH_COEFS most recent values,
         *   so don't keep storing the same value over and over
         */
        if(i - iStart < MAX_NUM_SMOOTH_COEFS) {
            for(m = 0; m < sbrFreq->numQMFBands; m++) {
                sbrChan->gTemp[gainNoiseIndex][m] = m_PSInfoSBR->gLimBoost[m];
                sbrChan->qTemp[gainNoiseIndex][m] = m_PSInfoSBR->qmLimBoost[m];
            }
        }

        /* see 4.6.18.7.6 */
        XBuf = m_PSInfoSBR->XBuf[i + HF_ADJ][sbrFreq->kStart];
        gbMask = 0;
        for(m = 0; m < sbrFreq->numQMFBands; m++) {
            if(env == m_PSInfoSBR->la || env == sbrChan->laPrev) {
                /* no smoothing filter for gain, and qFilt = 0 (only need to do once) */
                if(i == iStart) {
                    m_PSInfoSBR->gFiltLast[m] = sbrChan->gTemp[gainNoiseIndex][m];
                    m_PSInfoSBR->qFiltLast[m] = 0;
                }
            }
            else if(hSL == 0) {
                /* no smoothing filter for gain, (only need to do once) */
                if(i == iStart) {
                    m_PSInfoSBR->gFiltLast[m] = sbrChan->gTemp[gainNoiseIndex][m];
                    m_PSInfoSBR->qFiltLast[m] = sbrChan->qTemp[gainNoiseIndex][m];
                }
            }
            else {
                /* apply smoothing filter to gain and noise (after MAX_NUM_SMOOTH_COEFS, it's always the same) */
                if(i - iStart < MAX_NUM_SMOOTH_COEFS) {
                    gFilt = 0;
                    qFilt = 0;
                    idx = gainNoiseIndex;
                    for(j = 0; j < MAX_NUM_SMOOTH_COEFS; j++) {
                        /* sum(abs(hSmoothCoef[j])) for all j < 1.0 */
                        gFilt += MULSHIFT32(sbrChan->gTemp[idx][m], hSmoothCoef[j]);
                        qFilt += MULSHIFT32(sbrChan->qTemp[idx][m], hSmoothCoef[j]);
                        idx--;
                        if(idx < 0) idx += MAX_NUM_SMOOTH_COEFS;
                    }
                    m_PSInfoSBR->gFiltLast[m] = gFilt << 1; /* restore to Q(FBITS_GLIM_BOOST) (gain of filter < 1.0, so no overflow) */
                    m_PSInfoSBR->qFiltLast[m] = qFilt << 1; /* restore to Q(FBITS_QLIM_BOOST) */
                }
            }

            if(m_PSInfoSBR->smBoost[m] != 0) {
                /* add scaled signal and sinusoid, don't add noise (qFilt = 0) */
                smre = m_PSInfoSBR->smBoost[m];
                smim = smre;

                /* sinIndex:  [0] xre += sm   [1] xim += sm*s   [2] xre -= sm   [3] xim -= sm*s  */
                s = (sinIndex >> 1); /* if 2 or 3, flip sign to subtract sm */
                s <<= 31;
                smre ^= (s >> 31);
                smre -= (s >> 31);
                s ^= ((m + sbrFreq->kStart) << 31);
                smim ^= (s >> 31);
                smim -= (s >> 31);

                /* if sinIndex == 0 or 2, smim = 0; if sinIndex == 1 or 3, smre = 0 */
                s = sinIndex << 31;
                smim &= (s >> 31);
                s ^= 0x80000000;
                smre &= (s >> 31);

                noiseTabIndex += 2; /* noise filtered by 0, but still need to bump index */
            }
            else {
                /* add scaled signal and scaled noise */
                qFilt = m_PSInfoSBR->qFiltLast[m];
                n = noiseTab[noiseTabIndex++];
                smre = MULSHIFT32(n, qFilt) >> (FBITS_QLIM_BOOST - 1 - FBITS_OUT_QMFA);

                n = noiseTab[noiseTabIndex++];
                smim = MULSHIFT32(n, qFilt) >> (FBITS_QLIM_BOOST - 1 - FBITS_OUT_QMFA);
            }
            noiseTabIndex &= 1023; /* 512 complex numbers */

            gFilt = m_PSInfoSBR->gFiltLast[m];
            xre = MULSHIFT32(gFilt, XBuf[0]);
            xim = MULSHIFT32(gFilt, XBuf[1]);
            xre = CLIP_2N_SHIFT30(xre, 32 - FBITS_GLIM_BOOST);
            xim = CLIP_2N_SHIFT30(xim, 32 - FBITS_GLIM_BOOST);

            xre += smre;
            *XBuf++ = xre;
            xim += smim;
            *XBuf++ = xim;

            gbMask |= FASTABS(xre);
            gbMask |= FASTABS(xim);
        }
        /* update circular buffer index */
        gainNoiseIndex++;
        if(gainNoiseIndex == MAX_NUM_SMOOTH_COEFS) gainNoiseIndex = 0;

        sinIndex++;
        sinIndex &= 3;

        /* ensure MIN_GBITS_IN_QMFS guard bits in output
         * almost never occurs in practice, but checking here makes synth QMF logic very simple
         */
        if(gbMask >> (31 - MIN_GBITS_IN_QMFS)) {
            XBuf = m_PSInfoSBR->XBuf[i + HF_ADJ][sbrFreq->kStart];
            for(m = 0; m < sbrFreq->numQMFBands; m++) {
                xre = XBuf[0];
                xim = XBuf[1];
                xre = CLIP_2N(xre, (31 - MIN_GBITS_IN_QMFS));
                xim = CLIP_2N(xim, (31 - MIN_GBITS_IN_QMFS));
                *XBuf++ = xre;
                *XBuf++ = xim;
            }
            gbMask = CLIP_2N(gbMask, (31 - MIN_GBITS_IN_QMFS));
        }
        gbIdx = ((i + HF_ADJ) >> 5) & 0x01;
        sbrChan->gbMask[gbIdx] |= gbMask;
    }
    sbrChan->noiseTabIndex = noiseTabIndex;
    sbrChan->sinIndex = sinIndex;
    sbrChan->gainNoiseIndex = gainNoiseIndex;
}
/***********************************************************************************************************************
 * Function:    AdjustHighFreq
 *
 * Description: adjust high frequencies and add noise and sinusoids (4.6.18.7)
 *
 * Inputs:      initialized SBRHeader struct for this SCE/CPE block
 *              initialized SBRGrid struct for this channel
 *              initialized SBRFreq struct for this SCE/CPE block
 *              initialized SBRChan struct for this channel
 *              index of current channel (0 for SCE, 0 or 1 for CPE)
 *
 * Outputs:     complete reconstructed subband QMF samples for this channel
 *
 * Return:      none
 **********************************************************************************************************************/
void AdjustHighFreq(SBRHeader *sbrHdr, SBRGrid *sbrGrid, SBRFreq *sbrFreq, SBRChan *sbrChan, int ch) {

    int i, env, hfReset;
    uint8_t frameClass, pointer;

    frameClass = sbrGrid->frameClass;
    pointer  = sbrGrid->pointer;

    /* derive la from table 4.159 */
    if ((frameClass == SBR_GRID_FIXVAR || frameClass == SBR_GRID_VARVAR) && pointer > 0)
        m_PSInfoSBR->la = sbrGrid->numEnv + 1 - pointer;
    else if (frameClass == SBR_GRID_VARFIX && pointer > 1)
        m_PSInfoSBR->la = pointer - 1;
    else
        m_PSInfoSBR->la = -1;

    /* for each envelope, estimate gain and adjust SBR QMF bands */
    hfReset = sbrChan->reset;
    for (env = 0; env < sbrGrid->numEnv; env++) {
        EstimateEnvelope(sbrHdr, sbrGrid, sbrFreq, env);
        CalcGain(sbrHdr, sbrGrid, sbrFreq, sbrChan, ch, env);
        MapHF(sbrHdr, sbrGrid, sbrFreq, sbrChan, env, hfReset);
        hfReset = 0;    /* only set for first envelope after header reset */
    }

    /* set saved sine flags to 0 for QMF bands outside of current frequency range */
    for (i = 0; i < sbrFreq->freqLimiter[0] + sbrFreq->kStart; i++)
        sbrChan->addHarmonic[0][i] = 0;
    for (i = sbrFreq->freqLimiter[sbrFreq->nLimiter] + sbrFreq->kStart; i < 64; i++)
        sbrChan->addHarmonic[0][i] = 0;
    sbrChan->addHarmonicFlag[0] = sbrChan->addHarmonicFlag[1];

    /* save la for next frame */
    if (m_PSInfoSBR->la == sbrGrid->numEnv)
        sbrChan->laPrev = 0;
    else
        sbrChan->laPrev = -1;
}
/***********************************************************************************************************************
 * Function:    CalcCovariance1
 *
 * Description: calculate covariance matrix for p01, p12, p11, p22 (4.6.18.6.2)
 *
 * Inputs:      buffer of low-freq samples, starting at time index 0,
 *                freq index = patch subband
 *
 * Outputs:     complex covariance elements p01re, p01im, p12re, p12im, p11re, p22re
 *                (p11im = p22im = 0)
 *              format = integer (Q0) * 2^N, with scalefactor N >= 0
 *
 * Return:      scalefactor N
 *
 * Notes:       outputs are normalized to have 1 GB (sign in at least top 2 bits)
 **********************************************************************************************************************/
int CalcCovariance1(int *XBuf, int *p01reN, int *p01imN, int *p12reN, int *p12imN, int *p11reN, int *p22reN) {

    int accBuf[2*6];
    int n, z, s, loShift, hiShift, gbMask;
    U64 p01re, p01im, p12re, p12im, p11re, p22re;

    CVKernel1(XBuf, accBuf);
    p01re.r.lo32 = accBuf[0];   p01re.r.hi32 = accBuf[1];
    p01im.r.lo32 = accBuf[2];   p01im.r.hi32 = accBuf[3];
    p11re.r.lo32 = accBuf[4];   p11re.r.hi32 = accBuf[5];
    p12re.r.lo32 = accBuf[6];   p12re.r.hi32 = accBuf[7];
    p12im.r.lo32 = accBuf[8];   p12im.r.hi32 = accBuf[9];
    p22re.r.lo32 = accBuf[10];  p22re.r.hi32 = accBuf[11];

    /* 64-bit accumulators now have 2*FBITS_OUT_QMFA fraction bits
     * want to scale them down to integers (32-bit signed, Q0)
     *   with scale factor of 2^n, n >= 0
     * leave 2 GB's for calculating determinant, so take top 30 non-zero bits
     */
    gbMask  = ((p01re.r.hi32) ^ (p01re.r.hi32 >> 31)) | ((p01im.r.hi32) ^ (p01im.r.hi32 >> 31));
    gbMask |= ((p12re.r.hi32) ^ (p12re.r.hi32 >> 31)) | ((p12im.r.hi32) ^ (p12im.r.hi32 >> 31));
    gbMask |= ((p11re.r.hi32) ^ (p11re.r.hi32 >> 31)) | ((p22re.r.hi32) ^ (p22re.r.hi32 >> 31));
    if (gbMask == 0) {
        s = p01re.r.hi32 >> 31; gbMask  = (p01re.r.lo32 ^ s) - s;
        s = p01im.r.hi32 >> 31; gbMask |= (p01im.r.lo32 ^ s) - s;
        s = p12re.r.hi32 >> 31; gbMask |= (p12re.r.lo32 ^ s) - s;
        s = p12im.r.hi32 >> 31; gbMask |= (p12im.r.lo32 ^ s) - s;
        s = p11re.r.hi32 >> 31; gbMask |= (p11re.r.lo32 ^ s) - s;
        s = p22re.r.hi32 >> 31; gbMask |= (p22re.r.lo32 ^ s) - s;
        z = 32 + CLZ(gbMask);
    } else {
        gbMask  = FASTABS(p01re.r.hi32) | FASTABS(p01im.r.hi32);
        gbMask |= FASTABS(p12re.r.hi32) | FASTABS(p12im.r.hi32);
        gbMask |= FASTABS(p11re.r.hi32) | FASTABS(p22re.r.hi32);
        z = CLZ(gbMask);
    }

    n = 64 - z; /* number of non-zero bits in bottom of 64-bit word */
    if (n <= 30) {
        loShift = (30 - n);
        *p01reN = p01re.r.lo32 << loShift;  *p01imN = p01im.r.lo32 << loShift;
        *p12reN = p12re.r.lo32 << loShift;  *p12imN = p12im.r.lo32 << loShift;
        *p11reN = p11re.r.lo32 << loShift;  *p22reN = p22re.r.lo32 << loShift;
        return -(loShift + 2*FBITS_OUT_QMFA);
    } else if (n < 32 + 30) {
        loShift = (n - 30);
        hiShift = 32 - loShift;
        *p01reN = (p01re.r.hi32 << hiShift) | (p01re.r.lo32 >> loShift);
        *p01imN = (p01im.r.hi32 << hiShift) | (p01im.r.lo32 >> loShift);
        *p12reN = (p12re.r.hi32 << hiShift) | (p12re.r.lo32 >> loShift);
        *p12imN = (p12im.r.hi32 << hiShift) | (p12im.r.lo32 >> loShift);
        *p11reN = (p11re.r.hi32 << hiShift) | (p11re.r.lo32 >> loShift);
        *p22reN = (p22re.r.hi32 << hiShift) | (p22re.r.lo32 >> loShift);
        return (loShift - 2*FBITS_OUT_QMFA);
    } else {
        hiShift = n - (32 + 30);
        *p01reN = p01re.r.hi32 >> hiShift;  *p01imN = p01im.r.hi32 >> hiShift;
        *p12reN = p12re.r.hi32 >> hiShift;  *p12imN = p12im.r.hi32 >> hiShift;
        *p11reN = p11re.r.hi32 >> hiShift;  *p22reN = p22re.r.hi32 >> hiShift;
        return (32 - 2*FBITS_OUT_QMFA - hiShift);
    }

    return 0;
}
/***********************************************************************************************************************
 * Function:    CalcCovariance2
 *
 * Description: calculate covariance matrix for p02 (4.6.18.6.2)
 *
 * Inputs:      buffer of low-freq samples, starting at time index = 0,
 *                freq index = patch subband
 *
 * Outputs:     complex covariance element p02re, p02im
 *              format = integer (Q0) * 2^N, with scalefactor N >= 0
 *
 * Return:      scalefactor N
 *
 * Notes:       outputs are normalized to have 1 GB (sign in at least top 2 bits)
 **********************************************************************************************************************/
int CalcCovariance2(int *XBuf, int *p02reN, int *p02imN) {

    U64 p02re, p02im;
    int n, z, s, loShift, hiShift, gbMask;
    int accBuf[2*2];

    CVKernel2(XBuf, accBuf);
    p02re.r.lo32 = accBuf[0];
    p02re.r.hi32 = accBuf[1];
    p02im.r.lo32 = accBuf[2];
    p02im.r.hi32 = accBuf[3];

    /* 64-bit accumulators now have 2*FBITS_OUT_QMFA fraction bits
     * want to scale them down to integers (32-bit signed, Q0)
     *   with scale factor of 2^n, n >= 0
     * leave 1 GB for calculating determinant, so take top 30 non-zero bits
     */
    gbMask  = ((p02re.r.hi32) ^ (p02re.r.hi32 >> 31)) | ((p02im.r.hi32) ^ (p02im.r.hi32 >> 31));
    if (gbMask == 0) {
        s = p02re.r.hi32 >> 31; gbMask  = (p02re.r.lo32 ^ s) - s;
        s = p02im.r.hi32 >> 31; gbMask |= (p02im.r.lo32 ^ s) - s;
        z = 32 + CLZ(gbMask);
    } else {
        gbMask  = FASTABS(p02re.r.hi32) | FASTABS(p02im.r.hi32);
        z = CLZ(gbMask);
    }
    n = 64 - z; /* number of non-zero bits in bottom of 64-bit word */

    if (n <= 30) {
        loShift = (30 - n);
        *p02reN = p02re.r.lo32 << loShift;
        *p02imN = p02im.r.lo32 << loShift;
        return -(loShift + 2*FBITS_OUT_QMFA);
    } else if (n < 32 + 30) {
        loShift = (n - 30);
        hiShift = 32 - loShift;
        *p02reN = (p02re.r.hi32 << hiShift) | (p02re.r.lo32 >> loShift);
        *p02imN = (p02im.r.hi32 << hiShift) | (p02im.r.lo32 >> loShift);
        return (loShift - 2*FBITS_OUT_QMFA);
    } else {
        hiShift = n - (32 + 30);
        *p02reN = p02re.r.hi32 >> hiShift;
        *p02imN = p02im.r.hi32 >> hiShift;
        return (32 - 2*FBITS_OUT_QMFA - hiShift);
    }

    return 0;
}
/***********************************************************************************************************************
 * Function:    CalcLPCoefs
 *
 * Description: calculate linear prediction coefficients for one subband (4.6.18.6.2)
 *
 * Inputs:      buffer of low-freq samples, starting at time index = 0,
 *                freq index = patch subband
 *              number of guard bits in input sample buffer
 *
 * Outputs:     complex LP coefficients a0re, a0im, a1re, a1im, format = Q29
 *
 * Return:      none
 *
 * Notes:       output coefficients (a0re, a0im, a1re, a1im) clipped to range (-4, 4)
 *              if the comples coefficients have magnitude >= 4.0, they are all
 *                set to 0 (see spec)
 **********************************************************************************************************************/
void CalcLPCoefs(int *XBuf, int *a0re, int *a0im, int *a1re, int *a1im, int gb) {

    int zFlag, n1, n2, nd, d, dInv, tre, tim;
    int p01re, p01im, p02re, p02im, p12re, p12im, p11re, p22re;

    /* pre-scale to avoid overflow - probably never happens in practice (see QMFA)
     *   max bit growth per accumulator = 38*2 = 76 mul-adds (X * X)
     *   using 64-bit MADD, so if X has n guard bits, X*X has 2n+1 guard bits
     *   gain 1 extra sign bit per multiply, so ensure ceil(log2(76/2) / 2) = 3 guard bits on inputs
     */
    if (gb < 3) {
        nd = 3 - gb;
        for (n1 = (NUM_TIME_SLOTS*SAMPLES_PER_SLOT + 6 + 2); n1 != 0; n1--) {
            XBuf[0] >>= nd; XBuf[1] >>= nd;
            XBuf += (2*64);
        }
        XBuf -= (2*64*(NUM_TIME_SLOTS*SAMPLES_PER_SLOT + 6 + 2));
    }

    /* calculate covariance elements */
    n1 = CalcCovariance1(XBuf, &p01re, &p01im, &p12re, &p12im, &p11re, &p22re);
    n2 = CalcCovariance2(XBuf, &p02re, &p02im);

    /* normalize everything to larger power of 2 scalefactor, call it n1 */
    if (n1 < n2) {
        nd = MIN(n2 - n1, 31);
        p01re >>= nd;   p01im >>= nd;
        p12re >>= nd;   p12im >>= nd;
        p11re >>= nd;   p22re >>= nd;
        n1 = n2;
    } else if (n1 > n2) {
        nd = MIN(n1 - n2, 31);
        p02re >>= nd;   p02im >>= nd;
    }

    /* calculate determinant of covariance matrix (at least 1 GB in pXX) */
    d = MULSHIFT32(p12re, p12re) + MULSHIFT32(p12im, p12im);
    d = MULSHIFT32(d, RELAX_COEF) << 1;
    d = MULSHIFT32(p11re, p22re) - d;
    ASSERT(d >= 0); /* should never be < 0 */

    zFlag = 0;
    *a0re = *a0im = 0;
    *a1re = *a1im = 0;
    if (d > 0) {
        /* input =   Q31  d    = Q(-2*n1 - 32 + nd) = Q31 * 2^(31 + 2*n1 + 32 - nd)
         * inverse = Q29  dInv = Q29 * 2^(-31 - 2*n1 - 32 + nd) = Q(29 + 31 + 2*n1 + 32 - nd)
         *
         * numerator has same Q format as d, since it's sum of normalized squares
         * so num * inverse = Q(-2*n1 - 32) * Q(29 + 31 + 2*n1 + 32 - nd)
         *                  = Q(29 + 31 - nd), drop low 32 in MULSHIFT32
         *                  = Q(29 + 31 - 32 - nd) = Q(28 - nd)
         */
        nd = CLZ(d) - 1;
        d <<= nd;
        dInv = InvRNormalized(d);

        /* 1 GB in pXX */
        tre = MULSHIFT32(p01re, p12re) - MULSHIFT32(p01im, p12im) - MULSHIFT32(p02re, p11re);
        tre = MULSHIFT32(tre, dInv);
        tim = MULSHIFT32(p01re, p12im) + MULSHIFT32(p01im, p12re) - MULSHIFT32(p02im, p11re);
        tim = MULSHIFT32(tim, dInv);

        /* if d is extremely small, just set coefs to 0 (would have poor precision anyway) */
        if (nd > 28 || (FASTABS(tre) >> (28 - nd)) >= 4 || (FASTABS(tim) >> (28 - nd)) >= 4) {
            zFlag = 1;
        } else {
            *a1re = tre << (FBITS_LPCOEFS - 28 + nd);   /* i.e. convert Q(28 - nd) to Q(29) */
            *a1im = tim << (FBITS_LPCOEFS - 28 + nd);
        }
    }

    if (p11re) {
        /* input =   Q31  p11re = Q(-n1 + nd) = Q31 * 2^(31 + n1 - nd)
         * inverse = Q29  dInv  = Q29 * 2^(-31 - n1 + nd) = Q(29 + 31 + n1 - nd)
         *
         * numerator is Q(-n1 - 3)
         * so num * inverse = Q(-n1 - 3) * Q(29 + 31 + n1 - nd)
         *                  = Q(29 + 31 - 3 - nd), drop low 32 in MULSHIFT32
         *                  = Q(29 + 31 - 3 - 32 - nd) = Q(25 - nd)
         */
        nd = CLZ(p11re) - 1;    /* assume positive */
        p11re <<= nd;
        dInv = InvRNormalized(p11re);

        /* a1re, a1im = Q29, so scaled by (n1 + 3) */
        tre = (p01re >> 3) + MULSHIFT32(p12re, *a1re) + MULSHIFT32(p12im, *a1im);
        tre = -MULSHIFT32(tre, dInv);
        tim = (p01im >> 3) - MULSHIFT32(p12im, *a1re) + MULSHIFT32(p12re, *a1im);
        tim = -MULSHIFT32(tim, dInv);

        if (nd > 25 || (FASTABS(tre) >> (25 - nd)) >= 4 || (FASTABS(tim) >> (25 - nd)) >= 4) {
            zFlag = 1;
        } else {
            *a0re = tre << (FBITS_LPCOEFS - 25 + nd);   /* i.e. convert Q(25 - nd) to Q(29) */
            *a0im = tim << (FBITS_LPCOEFS - 25 + nd);
        }
    }

    /* see 4.6.18.6.2 - if magnitude of a0 or a1 >= 4 then a0 = a1 = 0
     * i.e. a0re < 4, a0im < 4, a1re < 4, a1im < 4
     * Q29*Q29 = Q26
     */
    if (zFlag || MULSHIFT32(*a0re, *a0re) + MULSHIFT32(*a0im, *a0im) >= MAG_16 || MULSHIFT32(*a1re, *a1re) + MULSHIFT32(*a1im, *a1im) >= MAG_16) {
        *a0re = *a0im = 0;
        *a1re = *a1im = 0;
    }

    /* no need to clip - we never changed the XBuf data, just used it to calculate a0 and a1 */
    if (gb < 3) {
        nd = 3 - gb;
        for (n1 = (NUM_TIME_SLOTS*SAMPLES_PER_SLOT + 6 + 2); n1 != 0; n1--) {
            XBuf[0] <<= nd; XBuf[1] <<= nd;
            XBuf += (2*64);
        }
    }
}
/***********************************************************************************************************************
 * Function:    GenerateHighFreq
 *
 * Description: generate high frequencies with SBR (4.6.18.6)
 *
 * Inputs:      initialized SBRGrid struct for this channel
 *              initialized SBRFreq struct for this SCE/CPE block
 *              initialized SBRChan struct for this channel
 *              index of current channel (0 for SCE, 0 or 1 for CPE)
 *
 * Outputs:     new high frequency samples starting at frequency kStart
 *
 * Return:      none
 **********************************************************************************************************************/
void GenerateHighFreq(SBRGrid *sbrGrid, SBRFreq *sbrFreq, SBRChan *sbrChan, int ch) {

    int band, newBW, c, t, gb, gbMask, gbIdx;
    int currPatch, p, x, k, g, i, iStart, iEnd, bw, bwsq;
    int a0re, a0im, a1re, a1im;
    int x1re, x1im, x2re, x2im;
    int ACCre, ACCim;
    int *XBufLo, *XBufHi;
    (void) ch;

    /* calculate array of chirp factors */
    for (band = 0; band < sbrFreq->numNoiseFloorBands; band++) {
        c = sbrChan->chirpFact[band];   /* previous (bwArray') */
        newBW = newBWTab[sbrChan->invfMode[0][band]][sbrChan->invfMode[1][band]];

        /* weighted average of new and old (can't overflow - total gain = 1.0) */
        if (newBW < c)
            t = MULSHIFT32(newBW, 0x60000000) + MULSHIFT32(0x20000000, c);  /* new is smaller: 0.75*new + 0.25*old */
        else
            t = MULSHIFT32(newBW, 0x74000000) + MULSHIFT32(0x0c000000, c);  /* new is larger: 0.90625*new + 0.09375*old */
        t <<= 1;

        if (t < 0x02000000) /* below 0.015625, clip to 0 */
            t = 0;
        if (t > 0x7f800000) /* clip to 0.99609375 */
            t = 0x7f800000;

        /* save curr as prev for next time */
        sbrChan->chirpFact[band] = t;
        sbrChan->invfMode[0][band] = sbrChan->invfMode[1][band];
    }

    iStart = sbrGrid->envTimeBorder[0] + HF_ADJ;
    iEnd =   sbrGrid->envTimeBorder[sbrGrid->numEnv] + HF_ADJ;

    /* generate new high freqs from low freqs, patches, and chirp factors */
    k = sbrFreq->kStart;
    g = 0;
    bw = sbrChan->chirpFact[g];
    bwsq = MULSHIFT32(bw, bw) << 1;

    gbMask = (sbrChan->gbMask[0] | sbrChan->gbMask[1]); /* older 32 | newer 8 */
    gb = CLZ(gbMask) - 1;

    for (currPatch = 0; currPatch < sbrFreq->numPatches; currPatch++) {
        for (x = 0; x < sbrFreq->patchNumSubbands[currPatch]; x++) {
            /* map k to corresponding noise floor band */
            if (k >= sbrFreq->freqNoise[g+1]) {
                g++;
                bw = sbrChan->chirpFact[g];     /* Q31 */
                bwsq = MULSHIFT32(bw, bw) << 1; /* Q31 */
            }

            p = sbrFreq->patchStartSubband[currPatch] + x;  /* low QMF band */
            XBufHi = m_PSInfoSBR->XBuf[iStart][k];
            if (bw) {
                CalcLPCoefs(m_PSInfoSBR->XBuf[0][p], &a0re, &a0im, &a1re, &a1im, gb);

                a0re = MULSHIFT32(bw, a0re);    /* Q31 * Q29 = Q28 */
                a0im = MULSHIFT32(bw, a0im);
                a1re = MULSHIFT32(bwsq, a1re);
                a1im = MULSHIFT32(bwsq, a1im);

                XBufLo = m_PSInfoSBR->XBuf[iStart-2][p];

                x2re = XBufLo[0];   /* RE{XBuf[n-2]} */
                x2im = XBufLo[1];   /* IM{XBuf[n-2]} */
                XBufLo += (64*2);

                x1re = XBufLo[0];   /* RE{XBuf[n-1]} */
                x1im = XBufLo[1];   /* IM{XBuf[n-1]} */
                XBufLo += (64*2);

                for (i = iStart; i < iEnd; i++) {
                    /* a0re/im, a1re/im are Q28 with at least 1 GB,
                     *   so the summing for AACre/im is fine (1 GB in, plus 1 from MULSHIFT32)
                     */
                    ACCre = MULSHIFT32(x2re, a1re) - MULSHIFT32(x2im, a1im);
                    ACCim = MULSHIFT32(x2re, a1im) + MULSHIFT32(x2im, a1re);
                    x2re = x1re;
                    x2im = x1im;

                    ACCre += MULSHIFT32(x1re, a0re) - MULSHIFT32(x1im, a0im);
                    ACCim += MULSHIFT32(x1re, a0im) + MULSHIFT32(x1im, a0re);
                    x1re = XBufLo[0];   /* RE{XBuf[n]} */
                    x1im = XBufLo[1];   /* IM{XBuf[n]} */
                    XBufLo += (64*2);

                    /* lost 4 fbits when scaling by a0re/im, a1re/im (Q28) */
                    ACCre = CLIP_2N_SHIFT30(ACCre, 4);
                    ACCre += x1re;
                    ACCim = CLIP_2N_SHIFT30(ACCim, 4);
                    ACCim += x1im;

                    XBufHi[0] = ACCre;
                    XBufHi[1] = ACCim;
                    XBufHi += (64*2);

                    /* update guard bit masks */
                    gbMask  = FASTABS(ACCre);
                    gbMask |= FASTABS(ACCim);
                    gbIdx = (i >> 5) & 0x01;    /* 0 if i < 32, 1 if i >= 32 */
                    sbrChan->gbMask[gbIdx] |= gbMask;
                }
            } else {
                XBufLo = (int *)m_PSInfoSBR->XBuf[iStart][p];
                for (i = iStart; i < iEnd; i++) {
                    XBufHi[0] = XBufLo[0];
                    XBufHi[1] = XBufLo[1];
                    XBufLo += (64*2);
                    XBufHi += (64*2);
                }
            }
            k++;    /* high QMF band */
        }
    }
}
/***********************************************************************************************************************
 * Function:    DecodeHuffmanScalar
 *
 * Description: decode one Huffman symbol from bitstream
 *
 * Inputs:      pointers to Huffman table and info struct
 *              left-aligned bit buffer with >= huffTabInfo->maxBits bits
 *
 * Outputs:     decoded symbol in *val
 *
 * Return:      number of bits in symbol
 *
 * Notes:       assumes canonical Huffman codes:
 *                first CW always 0, we have "count" CW's of length "nBits" bits
 *                starting CW for codes of length nBits+1 =
 *                  (startCW[nBits] + count[nBits]) << 1
 *                if there are no codes at nBits, then we just keep << 1 each time
 *                  (since count[nBits] = 0)
 **********************************************************************************************************************/
int DecodeHuffmanScalar(const signed int *huffTab, const HuffInfo_t *huffTabInfo, unsigned int bitBuf,
        signed int *val) {

    unsigned int count, start, shift, t;
    const uint8_t *countPtr;
    const signed int /*short*/*map;

    map = huffTab + huffTabInfo->offset;
    countPtr = huffTabInfo->count;

    start = 0;
    count = 0;
    shift = 32;
    do {
        start += count;
        start <<= 1;
        map += count;
        count = *countPtr++;
        shift--;
        t = (bitBuf >> shift) - start;
    } while(t >= count);

    *val = (signed int) map[t];
    return (countPtr - huffTabInfo->count);
}
/***********************************************************************************************************************
 * Function:    DecodeOneSymbol
 *
 * Description: dequantize one Huffman symbol from bitstream,
 *                using table huffTabSBR[huffTabIndex]
 *
 * Inputs:      index of Huffman table
 *
 * Outputs:     bitstream advanced by number of bits in codeword
 *
 * Return:      one decoded symbol
 **********************************************************************************************************************/
int DecodeOneSymbol(int huffTabIndex) {

    int nBits, val;
    unsigned int bitBuf;
    const HuffInfo_t *hi;

    hi = &(huffTabSBRInfo[huffTabIndex]);

    bitBuf = GetBitsNoAdvance(hi->maxBits) << (32 - hi->maxBits);
    nBits = DecodeHuffmanScalar(huffTabSBR, hi, bitBuf, &val);
    AdvanceBitstream(nBits);

    return val;
}

/* [1.0, sqrt(2)], format = Q29 (one guard bit for decoupling) */
static const int envDQTab[2] PROGMEM = {0x20000000, 0x2d413ccc};

/***********************************************************************************************************************
 * Function:    DequantizeEnvelope
 *
 * Description: dequantize envelope scalefactors
 *
 * Inputs:      number of scalefactors to process
 *              amplitude resolution flag for this frame (0 or 1)
 *              quantized envelope scalefactors
 *
 * Outputs:     dequantized envelope scalefactors
 *
 * Return:      extra int bits in output (6 + expMax)
 *              in other words, output format = Q(FBITS_OUT_DQ_ENV - (6 + expMax))
 *
 * Notes:       dequantized scalefactors have at least 2 GB
 **********************************************************************************************************************/
int DequantizeEnvelope(int nBands, int ampRes, int8_t *envQuant, int *envDequant) {

    int exp, expMax, i, scalei;

    if(nBands <= 0) return 0;

    /* scan for largest dequant value (do separately from envelope decoding to keep code cleaner) */
    expMax = 0;
    for(i = 0; i < nBands; i++) {
        if(envQuant[i] > expMax) expMax = envQuant[i];
    }

    /* dequantized envelope gains
     *   envDequant = 64*2^(envQuant / alpha) = 2^(6 + envQuant / alpha)
     *     if ampRes == 0, alpha = 2 and range of envQuant = [0, 127]
     *     if ampRes == 1, alpha = 1 and range of envQuant = [0, 63]
     * also if coupling is on, envDequant is scaled by something in range [0, 2]
     * so range of envDequant = [2^6, 2^69] (no coupling), [2^6, 2^70] (with coupling)
     *
     * typical range (from observation) of envQuant/alpha = [0, 27] --> largest envQuant ~= 2^33
     * output: Q(29 - (6 + expMax))
     *
     * reference: 14496-3:2001(E)/4.6.18.3.5 and 14496-4:200X/FPDAM8/5.6.5.1.2.1.5
     */
    if(ampRes) {
        do {
            exp = *envQuant++;
            scalei = MIN(expMax - exp, 31);
            *envDequant++ = envDQTab[0] >> scalei;
        } while(--nBands);

        return (6 + expMax);
    }
    else {
        expMax >>= 1;
        do {
            exp = *envQuant++;
            scalei = MIN(expMax - (exp >> 1), 31);
            *envDequant++ = envDQTab[exp & 0x01] >> scalei;
        } while(--nBands);

        return (6 + expMax);
    }

}
/***********************************************************************************************************************
 * Function:    DequantizeNoise
 *
 * Description: dequantize noise scalefactors
 *
 * Inputs:      number of scalefactors to process
 *              quantized noise scalefactors
 *
 * Outputs:     dequantized noise scalefactors, format = Q(FBITS_OUT_DQ_NOISE)
 *
 * Return:      none
 *
 * Notes:       dequantized scalefactors have at least 2 GB
 **********************************************************************************************************************/
void DequantizeNoise(int nBands, int8_t *noiseQuant, int *noiseDequant) {

    int exp, scalei;

    if(nBands <= 0) return;

    /* dequantize noise floor gains (4.6.18.3.5):
     *   noiseDequant = 2^(NOISE_FLOOR_OFFSET - noiseQuant)
     *
     * range of noiseQuant = [0, 30] (see 4.6.18.3.6), NOISE_FLOOR_OFFSET = 6
     *   so range of noiseDequant = [2^-24, 2^6]
     */
    do {
        exp = *noiseQuant++;
        scalei = NOISE_FLOOR_OFFSET - exp + FBITS_OUT_DQ_NOISE; /* 6 + 24 - exp, exp = [0,30] */

        if(scalei < 0)
            *noiseDequant++ = 0;
        else if(scalei < 30)
            *noiseDequant++ = 1 << scalei;
        else
            *noiseDequant++ = 0x3fffffff; /* leave 2 GB */

    } while(--nBands);
}
/***********************************************************************************************************************
 * Function:    DecodeSBREnvelope
 *
 * Description: decode delta Huffman coded envelope scalefactors from bitstream
 *
 * Inputs:      initialized SBRGrid struct for this channel
 *              initialized SBRFreq struct for this SCE/CPE block
 *              initialized SBRChan struct for this channel
 *              index of current channel (0 for SCE, 0 or 1 for CPE)
 *
 * Outputs:     dequantized env scalefactors for left channel (before decoupling)
 *              dequantized env scalefactors for right channel (if coupling off)
 *                or raw decoded env scalefactors for right channel (if coupling on)
 *
 * Return:      none
 **********************************************************************************************************************/
void DecodeSBREnvelope(SBRGrid *sbrGrid, SBRFreq *sbrFreq, SBRChan *sbrChan, int ch) {

    int huffIndexTime, huffIndexFreq, env, envStartBits, band, nBands, sf, lastEnv;
    int freqRes, freqResPrev, dShift, i;

    if(m_PSInfoSBR->couplingFlag && ch) {
        dShift = 1;
        if(sbrGrid->ampResFrame) {
            huffIndexTime = HuffTabSBR_tEnv30b;
            huffIndexFreq = HuffTabSBR_fEnv30b;
            envStartBits = 5;
        }
        else {
            huffIndexTime = HuffTabSBR_tEnv15b;
            huffIndexFreq = HuffTabSBR_fEnv15b;
            envStartBits = 6;
        }
    }
    else {
        dShift = 0;
        if(sbrGrid->ampResFrame) {
            huffIndexTime = HuffTabSBR_tEnv30;
            huffIndexFreq = HuffTabSBR_fEnv30;
            envStartBits = 6;
        }
        else {
            huffIndexTime = HuffTabSBR_tEnv15;
            huffIndexFreq = HuffTabSBR_fEnv15;
            envStartBits = 7;
        }
    }

    /* range of envDataQuant[] = [0, 127] (see comments in DequantizeEnvelope() for reference) */
    for(env = 0; env < sbrGrid->numEnv; env++) {
        nBands = (sbrGrid->freqRes[env] ? sbrFreq->nHigh : sbrFreq->nLow);
        freqRes = (sbrGrid->freqRes[env]);
        freqResPrev = (env == 0 ? sbrGrid->freqResPrev : sbrGrid->freqRes[env - 1]);
        lastEnv = (env == 0 ? sbrGrid->numEnvPrev - 1 : env - 1);
        if(lastEnv < 0) lastEnv = 0; /* first frame */

        ASSERT(nBands <= MAX_QMF_BANDS);

        if(sbrChan->deltaFlagEnv[env] == 0) {
            /* delta coding in freq */
            sf = GetBits(envStartBits) << dShift;
            sbrChan->envDataQuant[env][0] = sf;
            for(band = 1; band < nBands; band++) {
                sf = DecodeOneSymbol(huffIndexFreq) << dShift;
                sbrChan->envDataQuant[env][band] = sf + sbrChan->envDataQuant[env][band - 1];
            }
        }
        else if(freqRes == freqResPrev) {
            /* delta coding in time - same freq resolution for both frames */
            for(band = 0; band < nBands; band++) {
                sf = DecodeOneSymbol(huffIndexTime) << dShift;
                sbrChan->envDataQuant[env][band] = sf + sbrChan->envDataQuant[lastEnv][band];
            }
        }
        else if(freqRes == 0 && freqResPrev == 1) {
            /* delta coding in time - low freq resolution for new frame, high freq resolution for old frame */
            for(band = 0; band < nBands; band++) {
                sf = DecodeOneSymbol(huffIndexTime) << dShift;
                sbrChan->envDataQuant[env][band] = sf;
                for(i = 0; i < sbrFreq->nHigh; i++) {
                    if(sbrFreq->freqHigh[i] == sbrFreq->freqLow[band]) {
                        sbrChan->envDataQuant[env][band] += sbrChan->envDataQuant[lastEnv][i];
                        break;
                    }
                }
            }
        }
        else if(freqRes == 1 && freqResPrev == 0) {
            /* delta coding in time - high freq resolution for new frame, low freq resolution for old frame */
            for(band = 0; band < nBands; band++) {
                sf = DecodeOneSymbol(huffIndexTime) << dShift;
                sbrChan->envDataQuant[env][band] = sf;
                for(i = 0; i < sbrFreq->nLow; i++) {
                    if(sbrFreq->freqLow[i] <= sbrFreq->freqHigh[band]
                            && sbrFreq->freqHigh[band] < sbrFreq->freqLow[i + 1]) {
                        sbrChan->envDataQuant[env][band] += sbrChan->envDataQuant[lastEnv][i];
                        break;
                    }
                }
            }
        }

        /* skip coupling channel */
        if(ch != 1 || m_PSInfoSBR->couplingFlag != 1)
            m_PSInfoSBR->envDataDequantScale[ch][env] = DequantizeEnvelope(nBands, sbrGrid->ampResFrame,
                    sbrChan->envDataQuant[env], m_PSInfoSBR->envDataDequant[ch][env]);
    }
    sbrGrid->numEnvPrev = sbrGrid->numEnv;
    sbrGrid->freqResPrev = sbrGrid->freqRes[sbrGrid->numEnv - 1];
}
/***********************************************************************************************************************
 * Function:    DecodeSBRNoise
 *
 * Description: decode delta Huffman coded noise scalefactors from bitstream
 *
 * Inputs:      initialized SBRGrid struct for this channel
 *              initialized SBRFreq struct for this SCE/CPE block
 *              initialized SBRChan struct for this channel
 *              index of current channel (0 for SCE, 0 or 1 for CPE)
 *
 * Outputs:     dequantized noise scalefactors for left channel (before decoupling)
 *              dequantized noise scalefactors for right channel (if coupling off)
 *                or raw decoded noise scalefactors for right channel (if coupling on)
 *
 * Return:      none
 **********************************************************************************************************************/
void DecodeSBRNoise(SBRGrid *sbrGrid, SBRFreq *sbrFreq, SBRChan *sbrChan, int ch) {

    int huffIndexTime, huffIndexFreq, noiseFloor, band, dShift, sf, lastNoiseFloor;

    if(m_PSInfoSBR->couplingFlag && ch) {
        dShift = 1;
        huffIndexTime = HuffTabSBR_tNoise30b;
        huffIndexFreq = HuffTabSBR_fNoise30b;
    }
    else {
        dShift = 0;
        huffIndexTime = HuffTabSBR_tNoise30;
        huffIndexFreq = HuffTabSBR_fNoise30;
    }

    for(noiseFloor = 0; noiseFloor < sbrGrid->numNoiseFloors; noiseFloor++) {
        lastNoiseFloor = (noiseFloor == 0 ? sbrGrid->numNoiseFloorsPrev - 1 : noiseFloor - 1);
        if(lastNoiseFloor < 0) lastNoiseFloor = 0; /* first frame */

        ASSERT(sbrFreq->numNoiseFloorBands <= MAX_QMF_BANDS);

        if(sbrChan->deltaFlagNoise[noiseFloor] == 0) {
            /* delta coding in freq */
            sbrChan->noiseDataQuant[noiseFloor][0] = GetBits(5) << dShift;
            for(band = 1; band < sbrFreq->numNoiseFloorBands; band++) {
                sf = DecodeOneSymbol(huffIndexFreq) << dShift;
                sbrChan->noiseDataQuant[noiseFloor][band] = sf + sbrChan->noiseDataQuant[noiseFloor][band - 1];
            }
        }
        else {
            /* delta coding in time */
            for(band = 0; band < sbrFreq->numNoiseFloorBands; band++) {
                sf = DecodeOneSymbol(huffIndexTime) << dShift;
                sbrChan->noiseDataQuant[noiseFloor][band] = sf + sbrChan->noiseDataQuant[lastNoiseFloor][band];
            }
        }

        /* skip coupling channel */
        if(ch != 1 || m_PSInfoSBR->couplingFlag != 1)
            DequantizeNoise(sbrFreq->numNoiseFloorBands, sbrChan->noiseDataQuant[noiseFloor],
                    m_PSInfoSBR->noiseDataDequant[ch][noiseFloor]);
    }
    sbrGrid->numNoiseFloorsPrev = sbrGrid->numNoiseFloors;
}

/* dqTabCouple[i] = 2 / (1 + 2^(12 - i)), format = Q30 */
static const int dqTabCouple[25] PROGMEM = {
    0x0007ff80, 0x000ffe00, 0x001ff802, 0x003fe010, 0x007f8080, 0x00fe03f8, 0x01f81f82, 0x03e0f83e,
    0x07878788, 0x0e38e38e, 0x1999999a, 0x2aaaaaab, 0x40000000, 0x55555555, 0x66666666, 0x71c71c72,
    0x78787878, 0x7c1f07c2, 0x7e07e07e, 0x7f01fc08, 0x7f807f80, 0x7fc01ff0, 0x7fe007fe, 0x7ff00200,
    0x7ff80080,
};

/***********************************************************************************************************************
 * Function:    UncoupleSBREnvelope
 *
 * Description: scale dequantized envelope scalefactors according to channel
 *                coupling rules
 *
 * Inputs:      initialized SBRGrid struct for this channel
 *              initialized SBRFreq struct for this SCE/CPE block
 *              initialized SBRChan struct for right channel including
 *                quantized envelope scalefactors
 *
 * Outputs:     dequantized envelope data for left channel (after decoupling)
 *              dequantized envelope data for right channel (after decoupling)
 *
 * Return:      none
 **********************************************************************************************************************/
void UncoupleSBREnvelope(SBRGrid *sbrGrid, SBRFreq *sbrFreq, SBRChan *sbrChanR) {

    int env, band, nBands, scalei, E_1;

    scalei = (sbrGrid->ampResFrame ? 0 : 1);
    for(env = 0; env < sbrGrid->numEnv; env++) {
        nBands = (sbrGrid->freqRes[env] ? sbrFreq->nHigh : sbrFreq->nLow);
        m_PSInfoSBR->envDataDequantScale[1][env] = m_PSInfoSBR->envDataDequantScale[0][env];
        for(band = 0; band < nBands; band++) {
            /* clip E_1 to [0, 24] (scalefactors approach 0 or 2) */
            E_1 = sbrChanR->envDataQuant[env][band] >> scalei;
            if(E_1 < 0) E_1 = 0;
            if(E_1 > 24) E_1 = 24;

            /* envDataDequant[0] has 1 GB, so << by 2 is okay */
            m_PSInfoSBR->envDataDequant[1][env][band] = MULSHIFT32(m_PSInfoSBR->envDataDequant[0][env][band],
                    dqTabCouple[24 - E_1]) << 2;
            m_PSInfoSBR->envDataDequant[0][env][band] = MULSHIFT32(m_PSInfoSBR->envDataDequant[0][env][band],
                    dqTabCouple[E_1]) << 2;
        }
    }
}
/***********************************************************************************************************************
 * Function:    UncoupleSBRNoise
 *
 * Description: scale dequantized noise floor scalefactors according to channel
 *                coupling rules
 *
 * Inputs:      initialized SBRGrid struct for this channel
 *              initialized SBRFreq struct for this SCE/CPE block
 *              initialized SBRChan struct for this channel including
 *                quantized noise scalefactors
 *
 * Outputs:     dequantized noise data for left channel (after decoupling)
 *              dequantized noise data for right channel (after decoupling)
 *
 * Return:      none
 **********************************************************************************************************************/
void UncoupleSBRNoise(SBRGrid *sbrGrid, SBRFreq *sbrFreq, SBRChan *sbrChanR) {

    int noiseFloor, band, Q_1;

    for (noiseFloor = 0; noiseFloor < sbrGrid->numNoiseFloors; noiseFloor++) {
        for (band = 0; band < sbrFreq->numNoiseFloorBands; band++) {
            /* Q_1 should be in range [0, 24] according to 4.6.18.3.6, but check to make sure */
            Q_1 = sbrChanR->noiseDataQuant[noiseFloor][band];
            if (Q_1 < 0)    Q_1 = 0;
            if (Q_1 > 24)   Q_1 = 24;

            /* noiseDataDequant[0] has 1 GB, so << by 2 is okay */
            m_PSInfoSBR->noiseDataDequant[1][noiseFloor][band] =
                    MULSHIFT32(m_PSInfoSBR->noiseDataDequant[0][noiseFloor][band], dqTabCouple[24 - Q_1]) << 2;
            m_PSInfoSBR->noiseDataDequant[0][noiseFloor][band] =
                    MULSHIFT32(m_PSInfoSBR->noiseDataDequant[0][noiseFloor][band], dqTabCouple[Q_1]) << 2;
        }
    }
}
/***********************************************************************************************************************
 * Function:    DecWindowOverlapNoClip
 *
 * Description: apply synthesis window, do overlap-add without clipping,
 *                for winSequence LONG-LONG
 *
 * Inputs:      input buffer (output of type-IV DCT)
 *              overlap buffer (saved from last time)
 *              window type (sin or KBD) for input buffer
 *              window type (sin or KBD) for overlap buffer
 *
 * Outputs:     one channel, one frame of 32-bit PCM, non-interleaved
 *
 * Return:      none
 *
 * Notes:       use this function when the decoded PCM is going to the SBR decoder
 **********************************************************************************************************************/
void DecWindowOverlapNoClip(int *buf0, int *over0, int *out0, int winTypeCurr, int winTypePrev) {

    int in, w0, w1, f0, f1;
    int *buf1, *over1, *out1;
    const int *wndPrev, *wndCurr;

    buf0 += (1024 >> 1);
    buf1  = buf0  - 1;
    out1  = out0 + 1024 - 1;
    over1 = over0 + 1024 - 1;

    wndPrev = (winTypePrev == 1 ? kbdWindow + kbdWindowOffset[1] : sinWindow + sinWindowOffset[1]);
    if (winTypeCurr == winTypePrev) {
        /* cut window loads in half since current and overlap sections use same symmetric window */
        do {
            w0 = *wndPrev++;
            w1 = *wndPrev++;
            in = *buf0++;

            f0 = MULSHIFT32(w0, in);
            f1 = MULSHIFT32(w1, in);

            in = *over0;
            *out0++ = in - f0;

            in = *over1;
            *out1-- = in + f1;

            in = *buf1--;
            *over1-- = MULSHIFT32(w0, in);
            *over0++ = MULSHIFT32(w1, in);
        } while (over0 < over1);
    } else {
        /* different windows for current and overlap parts - should still fit in registers on ARM w/o stack spill */
        wndCurr = (winTypeCurr == 1 ? kbdWindow + kbdWindowOffset[1] : sinWindow + sinWindowOffset[1]);
        do {
            w0 = *wndPrev++;
            w1 = *wndPrev++;
            in = *buf0++;

            f0 = MULSHIFT32(w0, in);
            f1 = MULSHIFT32(w1, in);

            in = *over0;
            *out0++ = in - f0;

            in = *over1;
            *out1-- = in + f1;

            w0 = *wndCurr++;
            w1 = *wndCurr++;
            in = *buf1--;

            *over1-- = MULSHIFT32(w0, in);
            *over0++ = MULSHIFT32(w1, in);
        } while (over0 < over1);
    }
}
/***********************************************************************************************************************
 * Function:    DecWindowOverlapLongStart
 *
 * Description: apply synthesis window, do overlap-add, without clipping
 *                for winSequence LONG-START
 *
 * Inputs:      input buffer (output of type-IV DCT)
 *              overlap buffer (saved from last time)
 *              window type (sin or KBD) for input buffer
 *              window type (sin or KBD) for overlap buffer
 *
 * Outputs:     one channel, one frame of 32-bit PCM, non-interleaved
 *
 * Return:      none
 *
 * Notes:       use this function when the decoded PCM is going to the SBR decoder
 **********************************************************************************************************************/
void DecWindowOverlapLongStartNoClip(int *buf0, int *over0, int *out0, int winTypeCurr, int winTypePrev) {

    int i,  in, w0, w1, f0, f1;
    int *buf1, *over1, *out1;
    const int *wndPrev, *wndCurr;

    buf0 += (1024 >> 1);
    buf1  = buf0  - 1;
    out1  = out0 + 1024 - 1;
    over1 = over0 + 1024 - 1;

    wndPrev = (winTypePrev == 1 ? kbdWindow + kbdWindowOffset[1] : sinWindow + sinWindowOffset[1]);
    i = 448;    /* 2 outputs, 2 overlaps per loop */
    do {
        w0 = *wndPrev++;
        w1 = *wndPrev++;
        in = *buf0++;

        f0 = MULSHIFT32(w0, in);
        f1 = MULSHIFT32(w1, in);

        in = *over0;
        *out0++ = in - f0;

        in = *over1;
        *out1-- = in + f1;

        in = *buf1--;

        *over1-- = 0;       /* Wn = 0 for n = (2047, 2046, ... 1600) */
        *over0++ = in >> 1; /* Wn = 1 for n = (1024, 1025, ... 1471) */
    } while (--i);

    wndCurr = (winTypeCurr == 1 ? kbdWindow + kbdWindowOffset[0] : sinWindow + sinWindowOffset[0]);

    /* do 64 more loops - 2 outputs, 2 overlaps per loop */
    do {
        w0 = *wndPrev++;
        w1 = *wndPrev++;
        in = *buf0++;

        f0 = MULSHIFT32(w0, in);
        f1 = MULSHIFT32(w1, in);

        in = *over0;
        *out0++ = in - f0;

        in = *over1;
        *out1-- = in + f1;

        w0 = *wndCurr++;    /* W[0], W[1], ... --> W[255], W[254], ... */
        w1 = *wndCurr++;    /* W[127], W[126], ... --> W[128], W[129], ... */
        in = *buf1--;

        *over1-- = MULSHIFT32(w0, in);  /* Wn = short window for n = (1599, 1598, ... , 1536) */
        *over0++ = MULSHIFT32(w1, in);  /* Wn = short window for n = (1472, 1473, ... , 1535) */
    } while (over0 < over1);
}
/***********************************************************************************************************************
 * Function:    DecWindowOverlapLongStop
 *
 * Description: apply synthesis window, do overlap-add, without clipping
 *                for winSequence LONG-STOP
 *
 * Inputs:      input buffer (output of type-IV DCT)
 *              overlap buffer (saved from last time)
 *              window type (sin or KBD) for input buffer
 *              window type (sin or KBD) for overlap buffer
 *
 * Outputs:     one channel, one frame of 32-bit PCM, non-interleaved
 *
 * Return:      none
 *
 * Notes:       use this function when the decoded PCM is going to the SBR decoder
 **********************************************************************************************************************/
void DecWindowOverlapLongStopNoClip(int *buf0, int *over0, int *out0, int winTypeCurr, int winTypePrev) {

    int i, in, w0, w1, f0, f1;
    int *buf1, *over1, *out1;
    const int *wndPrev, *wndCurr;

    buf0 += (1024 >> 1);
    buf1  = buf0  - 1;
    out1  = out0 + 1024 - 1;
    over1 = over0 + 1024 - 1;

    wndPrev = (winTypePrev == 1 ? kbdWindow + kbdWindowOffset[0] : sinWindow + sinWindowOffset[0]);
    wndCurr = (winTypeCurr == 1 ? kbdWindow + kbdWindowOffset[1] : sinWindow + sinWindowOffset[1]);

    i = 448;    /* 2 outputs, 2 overlaps per loop */
    do {
        /* Wn = 0 for n = (0, 1, ... 447) */
        /* Wn = 1 for n = (576, 577, ... 1023) */
        in = *buf0++;
        f1 = in >> 1;   /* scale since skipping multiply by Q31 */

        in = *over0;
        *out0++ = in;

        in = *over1;
        *out1-- = in + f1;

        w0 = *wndCurr++;
        w1 = *wndCurr++;
        in = *buf1--;

        *over1-- = MULSHIFT32(w0, in);
        *over0++ = MULSHIFT32(w1, in);
    } while (--i);

    /* do 64 more loops - 2 outputs, 2 overlaps per loop */
    do {
        w0 = *wndPrev++;    /* W[0], W[1], ...W[63] */
        w1 = *wndPrev++;    /* W[127], W[126], ... W[64] */
        in = *buf0++;

        f0 = MULSHIFT32(w0, in);
        f1 = MULSHIFT32(w1, in);

        in = *over0;
        *out0++ = in - f0;

        in = *over1;
        *out1-- = in + f1;

        w0 = *wndCurr++;
        w1 = *wndCurr++;
        in = *buf1--;

        *over1-- = MULSHIFT32(w0, in);
        *over0++ = MULSHIFT32(w1, in);
    } while (over0 < over1);
}
/***********************************************************************************************************************
 * Function:    DecWindowOverlapShort
 *
 * Description: apply synthesis window, do overlap-add, without clipping
 *                for winSequence EIGHT-SHORT (does all 8 short blocks)
 *
 * Inputs:      input buffer (output of type-IV DCT)
 *              overlap buffer (saved from last time)
 *              window type (sin or KBD) for input buffer
 *              window type (sin or KBD) for overlap buffer
 *
 * Outputs:     one channel, one frame of 32-bit PCM, non-interleaved
 *
 * Return:      none
 *
 * Notes:       use this function when the decoded PCM is going to the SBR decoder
 **********************************************************************************************************************/
void DecWindowOverlapShortNoClip(int *buf0, int *over0, int *out0, int winTypeCurr, int winTypePrev) {

    int i, in, w0, w1, f0, f1;
    int *buf1, *over1, *out1;
    const int *wndPrev, *wndCurr;

    wndPrev = (winTypePrev == 1 ? kbdWindow + kbdWindowOffset[0] : sinWindow + sinWindowOffset[0]);
    wndCurr = (winTypeCurr == 1 ? kbdWindow + kbdWindowOffset[0] : sinWindow + sinWindowOffset[0]);

    /* pcm[0-447] = 0 + overlap[0-447] */
    i = 448;
    do {
        f0 = *over0++;
        f1 = *over0++;
        *out0++ = f0;
        *out0++ = f1;
        i -= 2;
    } while (i);

    /* pcm[448-575] = Wp[0-127] * block0[0-127] + overlap[448-575] */
    out1  = out0 + (128 - 1);
    over1 = over0 + 128 - 1;
    buf0 += 64;
    buf1  = buf0  - 1;
    do {
        w0 = *wndPrev++;    /* W[0], W[1], ...W[63] */
        w1 = *wndPrev++;    /* W[127], W[126], ... W[64] */
        in = *buf0++;

        f0 = MULSHIFT32(w0, in);
        f1 = MULSHIFT32(w1, in);

        in = *over0;
        *out0++ = in - f0;

        in = *over1;
        *out1-- = in + f1;

        w0 = *wndCurr++;
        w1 = *wndCurr++;
        in = *buf1--;

        /* save over0/over1 for next short block, in the slots just vacated */
        *over1-- = MULSHIFT32(w0, in);
        *over0++ = MULSHIFT32(w1, in);
    } while (over0 < over1);

    /* pcm[576-703] = Wc[128-255] * block0[128-255] + Wc[0-127] * block1[0-127] + overlap[576-703]
     * pcm[704-831] = Wc[128-255] * block1[128-255] + Wc[0-127] * block2[0-127] + overlap[704-831]
     * pcm[832-959] = Wc[128-255] * block2[128-255] + Wc[0-127] * block3[0-127] + overlap[832-959]
     */
    for (i = 0; i < 3; i++) {
        out0 += 64;
        out1 = out0 + 128 - 1;
        over0 += 64;
        over1 = over0 + 128 - 1;
        buf0 += 64;
        buf1 = buf0 - 1;
        wndCurr -= 128;

        do {
            w0 = *wndCurr++;    /* W[0], W[1], ...W[63] */
            w1 = *wndCurr++;    /* W[127], W[126], ... W[64] */
            in = *buf0++;

            f0 = MULSHIFT32(w0, in);
            f1 = MULSHIFT32(w1, in);

            in  = *(over0 - 128);   /* from last short block */
            in += *(over0 + 0);     /* from last full frame */
            *out0++ = in - f0;

            in  = *(over1 - 128);   /* from last short block */
            in += *(over1 + 0);     /* from last full frame */
            *out1-- = in + f1;

            /* save over0/over1 for next short block, in the slots just vacated */
            in = *buf1--;
            *over1-- = MULSHIFT32(w0, in);
            *over0++ = MULSHIFT32(w1, in);
        } while (over0 < over1);
    }

    /* pcm[960-1023] = Wc[128-191] * block3[128-191] + Wc[0-63]   * block4[0-63] + overlap[960-1023]
     * over[0-63]    = Wc[192-255] * block3[192-255] + Wc[64-127] * block4[64-127]
     */
    out0 += 64;
    over0 -= 832;               /* points at overlap[64] */
    over1 = over0 + 128 - 1;    /* points at overlap[191] */
    buf0 += 64;
    buf1 = buf0 - 1;
    wndCurr -= 128;
    do {
        w0 = *wndCurr++;    /* W[0], W[1], ...W[63] */
        w1 = *wndCurr++;    /* W[127], W[126], ... W[64] */
        in = *buf0++;

        f0 = MULSHIFT32(w0, in);
        f1 = MULSHIFT32(w1, in);

        in  = *(over0 + 768);   /* from last short block */
        in += *(over0 + 896);   /* from last full frame */
        *out0++ = in - f0;

        in  = *(over1 + 768);   /* from last short block */
        *(over1 - 128) = in + f1;

        in = *buf1--;
        *over1-- = MULSHIFT32(w0, in);  /* save in overlap[128-191] */
        *over0++ = MULSHIFT32(w1, in);  /* save in overlap[64-127] */
    } while (over0 < over1);

    /* over0 now points at overlap[128] */

    /* over[64-191]   = Wc[128-255] * block4[128-255] + Wc[0-127] * block5[0-127]
     * over[192-319]  = Wc[128-255] * block5[128-255] + Wc[0-127] * block6[0-127]
     * over[320-447]  = Wc[128-255] * block6[128-255] + Wc[0-127] * block7[0-127]
     * over[448-576]  = Wc[128-255] * block7[128-255]
     */
    for (i = 0; i < 3; i++) {
        over0 += 64;
        over1 = over0 + 128 - 1;
        buf0 += 64;
        buf1 = buf0 - 1;
        wndCurr -= 128;
        do {
            w0 = *wndCurr++;    /* W[0], W[1], ...W[63] */
            w1 = *wndCurr++;    /* W[127], W[126], ... W[64] */
            in = *buf0++;

            f0 = MULSHIFT32(w0, in);
            f1 = MULSHIFT32(w1, in);

            /* from last short block */
            *(over0 - 128) -= f0;
            *(over1 - 128)+= f1;

            in = *buf1--;
            *over1-- = MULSHIFT32(w0, in);
            *over0++ = MULSHIFT32(w1, in);
        } while (over0 < over1);
    }

    /* over[576-1024] = 0 */
    i = 448;
    over0 += 64;
    do {
        *over0++ = 0;
        *over0++ = 0;
        *over0++ = 0;
        *over0++ = 0;
        i -= 4;
    } while (i);
}
/***********************************************************************************************************************
 * Function:    PreMultiply64
 *
 * Description: pre-twiddle stage of 64-point DCT-IV
 *
 * Inputs:      buffer of 64 samples
 *
 * Outputs:     processed samples in same buffer
 *
 * Return:      none
 *
 * Notes:       minimum 1 GB in, 2 GB out, gains 2 int bits
 *              gbOut = gbIn + 1
 *              output is limited to sqrt(2)/2 plus GB in full GB
 *              uses 3-mul, 3-add butterflies instead of 4-mul, 2-add
 **********************************************************************************************************************/
void PreMultiply64(int *zbuf1) {

    int i, ar1, ai1, ar2, ai2, z1, z2;
    int t, cms2, cps2a, sin2a, cps2b, sin2b;
    int *zbuf2;
    const int *csptr;

    zbuf2 = zbuf1 + 64 - 1;
    csptr = cos4sin4tab64;

    /* whole thing should fit in registers - verify that compiler does this */
    for (i = 64 >> 2; i != 0; i--) {
        /* cps2 = (cos+sin), sin2 = sin, cms2 = (cos-sin) */
        cps2a = *csptr++;
        sin2a = *csptr++;
        cps2b = *csptr++;
        sin2b = *csptr++;

        ar1 = *(zbuf1 + 0);
        ai2 = *(zbuf1 + 1);
        ai1 = *(zbuf2 + 0);
        ar2 = *(zbuf2 - 1);

        /* gain 2 ints bit from MULSHIFT32 by Q30
         * max per-sample gain (ignoring implicit scaling) = MAX(sin(angle)+cos(angle)) = 1.414
         * i.e. gain 1 GB since worst case is sin(angle) = cos(angle) = 0.707 (Q30), gain 2 from
         *   extra sign bits, and eat one in adding
         */
        t  = MULSHIFT32(sin2a, ar1 + ai1);
        z2 = MULSHIFT32(cps2a, ai1) - t;
        cms2 = cps2a - 2*sin2a;
        z1 = MULSHIFT32(cms2, ar1) + t;
        *zbuf1++ = z1;  /* cos*ar1 + sin*ai1 */
        *zbuf1++ = z2;  /* cos*ai1 - sin*ar1 */

        t  = MULSHIFT32(sin2b, ar2 + ai2);
        z2 = MULSHIFT32(cps2b, ai2) - t;
        cms2 = cps2b - 2*sin2b;
        z1 = MULSHIFT32(cms2, ar2) + t;
        *zbuf2-- = z2;  /* cos*ai2 - sin*ar2 */
        *zbuf2-- = z1;  /* cos*ar2 + sin*ai2 */
    }
}
/***********************************************************************************************************************
 * Function:    PostMultiply64
 *
 * Description: post-twiddle stage of 64-point type-IV DCT
 *
 * Inputs:      buffer of 64 samples
 *              number of output samples to calculate
 *
 * Outputs:     processed samples in same buffer
 *
 * Return:      none
 *
 * Notes:       minimum 1 GB in, 2 GB out, gains 2 int bits
 *              gbOut = gbIn + 1
 *              output is limited to sqrt(2)/2 plus GB in full GB
 *              nSampsOut is rounded up to next multiple of 4, since we calculate
 *                4 samples per loop
 **********************************************************************************************************************/
void PostMultiply64(int *fft1, int nSampsOut) {

    int i, ar1, ai1, ar2, ai2;
    int t, cms2, cps2, sin2;
    int *fft2;
    const int *csptr;

    csptr = cos1sin1tab64;
    fft2 = fft1 + 64 - 1;

    /* load coeffs for first pass
     * cps2 = (cos+sin)/2, sin2 = sin/2, cms2 = (cos-sin)/2
     */
    cps2 = *csptr++;
    sin2 = *csptr++;
    cms2 = cps2 - 2*sin2;

    for (i = (nSampsOut + 3) >> 2; i != 0; i--) {
        ar1 = *(fft1 + 0);
        ai1 = *(fft1 + 1);
        ar2 = *(fft2 - 1);
        ai2 = *(fft2 + 0);

        /* gain 2 int bits (multiplying by Q30), max gain = sqrt(2) */
        t = MULSHIFT32(sin2, ar1 + ai1);
        *fft2-- = t - MULSHIFT32(cps2, ai1);
        *fft1++ = t + MULSHIFT32(cms2, ar1);

        cps2 = *csptr++;
        sin2 = *csptr++;

        ai2 = -ai2;
        t = MULSHIFT32(sin2, ar2 + ai2);
        *fft2-- = t - MULSHIFT32(cps2, ai2);
        cms2 = cps2 - 2*sin2;
        *fft1++ = t + MULSHIFT32(cms2, ar2);
    }
}
/***********************************************************************************************************************
 * Function:    QMFAnalysisConv
 *
 * Description: convolution kernel for analysis QMF
 *
 * Inputs:      pointer to coefficient table, reordered for sequential access
 *              delay buffer of size 32*10 = 320 real-valued PCM samples
 *              index for delay ring buffer (range = [0, 9])
 *
 * Outputs:     64 consecutive 32-bit samples
 *
 * Return:      none
 *
 * Notes:       this is carefully written to be efficient on ARM
 *              use the assembly code version in sbrqmfak.s when building for ARM!
 **********************************************************************************************************************/
void QMFAnalysisConv(int *cTab, int *delay, int dIdx, int *uBuf) {

    int k, dOff;
    int *cPtr0, *cPtr1;
    U64 u64lo, u64hi;

    dOff = dIdx*32 + 31;
    cPtr0 = cTab;
    cPtr1 = cTab + 33*5 - 1;

    /* special first pass since we need to flip sign to create cTab[384], cTab[512] */
    u64lo.w64 = 0;
    u64hi.w64 = 0;
    u64lo.w64 = MADD64(u64lo.w64,  *cPtr0++,   delay[dOff]);    dOff -= 32; if (dOff < 0) {dOff += 320;}
    u64hi.w64 = MADD64(u64hi.w64,  *cPtr0++,   delay[dOff]);    dOff -= 32; if (dOff < 0) {dOff += 320;}
    u64lo.w64 = MADD64(u64lo.w64,  *cPtr0++,   delay[dOff]);    dOff -= 32; if (dOff < 0) {dOff += 320;}
    u64hi.w64 = MADD64(u64hi.w64,  *cPtr0++,   delay[dOff]);    dOff -= 32; if (dOff < 0) {dOff += 320;}
    u64lo.w64 = MADD64(u64lo.w64,  *cPtr0++,   delay[dOff]);    dOff -= 32; if (dOff < 0) {dOff += 320;}
    u64hi.w64 = MADD64(u64hi.w64,  *cPtr1--,   delay[dOff]);    dOff -= 32; if (dOff < 0) {dOff += 320;}
    u64lo.w64 = MADD64(u64lo.w64, -(*cPtr1--), delay[dOff]);    dOff -= 32; if (dOff < 0) {dOff += 320;}
    u64hi.w64 = MADD64(u64hi.w64,  *cPtr1--,   delay[dOff]);    dOff -= 32; if (dOff < 0) {dOff += 320;}
    u64lo.w64 = MADD64(u64lo.w64, -(*cPtr1--), delay[dOff]);    dOff -= 32; if (dOff < 0) {dOff += 320;}
    u64hi.w64 = MADD64(u64hi.w64,  *cPtr1--,   delay[dOff]);    dOff -= 32; if (dOff < 0) {dOff += 320;}

    uBuf[0]  = u64lo.r.hi32;
    uBuf[32] = u64hi.r.hi32;
    uBuf++;
    dOff--;

    /* max gain for any sample in uBuf, after scaling by cTab, ~= 0.99
     * so we can just sum the uBuf values with no overflow problems
     */
    for (k = 1; k <= 31; k++) {
        u64lo.w64 = 0;
        u64hi.w64 = 0;
        u64lo.w64 = MADD64(u64lo.w64, *cPtr0++, delay[dOff]);   dOff -= 32; if (dOff < 0) {dOff += 320;}
        u64hi.w64 = MADD64(u64hi.w64, *cPtr0++, delay[dOff]);   dOff -= 32; if (dOff < 0) {dOff += 320;}
        u64lo.w64 = MADD64(u64lo.w64, *cPtr0++, delay[dOff]);   dOff -= 32; if (dOff < 0) {dOff += 320;}
        u64hi.w64 = MADD64(u64hi.w64, *cPtr0++, delay[dOff]);   dOff -= 32; if (dOff < 0) {dOff += 320;}
        u64lo.w64 = MADD64(u64lo.w64, *cPtr0++, delay[dOff]);   dOff -= 32; if (dOff < 0) {dOff += 320;}
        u64hi.w64 = MADD64(u64hi.w64, *cPtr1--, delay[dOff]);   dOff -= 32; if (dOff < 0) {dOff += 320;}
        u64lo.w64 = MADD64(u64lo.w64, *cPtr1--, delay[dOff]);   dOff -= 32; if (dOff < 0) {dOff += 320;}
        u64hi.w64 = MADD64(u64hi.w64, *cPtr1--, delay[dOff]);   dOff -= 32; if (dOff < 0) {dOff += 320;}
        u64lo.w64 = MADD64(u64lo.w64, *cPtr1--, delay[dOff]);   dOff -= 32; if (dOff < 0) {dOff += 320;}
        u64hi.w64 = MADD64(u64hi.w64, *cPtr1--, delay[dOff]);   dOff -= 32; if (dOff < 0) {dOff += 320;}

        uBuf[0]  = u64lo.r.hi32;
        uBuf[32] = u64hi.r.hi32;
        uBuf++;
        dOff--;
    }
}
/***********************************************************************************************************************
 * Function:    QMFAnalysis
 *
 * Description: 32-subband analysis QMF (4.6.18.4.1)
 *
 * Inputs:      32 consecutive samples of decoded 32-bit PCM, format = Q(fBitsIn)
 *              delay buffer of size 32*10 = 320 PCM samples
 *              number of fraction bits in input PCM
 *              index for delay ring buffer (range = [0, 9])
 *              number of subbands to calculate (range = [0, 32])
 *
 * Outputs:     qmfaBands complex subband samples, format = Q(FBITS_OUT_QMFA)
 *              updated delay buffer
 *              updated delay index
 *
 * Return:      guard bit mask
 *
 * Notes:       output stored as RE{X0}, IM{X0}, RE{X1}, IM{X1}, ... RE{X31}, IM{X31}
 *              output stored in int buffer of size 64*2 = 128
 *                (zero-filled from XBuf[2*qmfaBands] to XBuf[127])
 **********************************************************************************************************************/
int QMFAnalysis(int *inbuf, int *delay, int *XBuf, int fBitsIn, int *delayIdx, int qmfaBands) {

    int n, y, shift, gbMask;
    int *delayPtr, *uBuf, *tBuf;

    /* use XBuf[128] as temp buffer for reordering */
    uBuf = XBuf;        /* first 64 samples */
    tBuf = XBuf + 64;   /* second 64 samples */

    /* overwrite oldest PCM with new PCM
     * delay[n] has 1 GB after shifting (either << or >>)
     */
    delayPtr = delay + (*delayIdx * 32);
    if (fBitsIn > FBITS_IN_QMFA) {
        shift = MIN(fBitsIn - FBITS_IN_QMFA, 31);
        for (n = 32; n != 0; n--) {
            y = (*inbuf) >> shift;
            inbuf++;
            *delayPtr++ = y;
        }
    } else {
        shift = MIN(FBITS_IN_QMFA - fBitsIn, 30);
        for (n = 32; n != 0; n--) {
            y = *inbuf++;
            y = CLIP_2N_SHIFT30(y, shift);
            *delayPtr++ = y;
        }
    }

    QMFAnalysisConv((int *)cTabA, delay, *delayIdx, uBuf);

    /* uBuf has at least 2 GB right now (1 from clipping to Q(FBITS_IN_QMFA), one from
     *   the scaling by cTab (MULSHIFT32(*delayPtr--, *cPtr++), with net gain of < 1.0)
     */
    tBuf[2*0 + 0] = uBuf[0];
    tBuf[2*0 + 1] = uBuf[1];
    for (n = 1; n < 31; n++) {
        tBuf[2*n + 0] = -uBuf[64-n];
        tBuf[2*n + 1] =  uBuf[n+1];
    }
    tBuf[2*31 + 1] =  uBuf[32];
    tBuf[2*31 + 0] = -uBuf[33];

    /* fast in-place DCT-IV - only need 2*qmfaBands output samples */
    PreMultiply64(tBuf);    /* 2 GB in, 3 GB out */
    FFT32C(tBuf);           /* 3 GB in, 1 GB out */
    PostMultiply64(tBuf, qmfaBands*2);  /* 1 GB in, 2 GB out */

    gbMask = 0;
    for (n = 0; n < qmfaBands; n++) {
        XBuf[2*n+0] =  tBuf[ n + 0];    /* implicit scaling of 2 in our output Q format */
        gbMask |= FASTABS(XBuf[2*n+0]);
        XBuf[2*n+1] = -tBuf[63 - n];
        gbMask |= FASTABS(XBuf[2*n+1]);
    }

    /* fill top section with zeros for HF generation */
    for (    ; n < 64; n++) {
        XBuf[2*n+0] = 0;
        XBuf[2*n+1] = 0;
    }

    *delayIdx = (*delayIdx == NUM_QMF_DELAY_BUFS - 1 ? 0 : *delayIdx + 1);

    /* minimum of 2 GB in output */
    return gbMask;
}
/***********************************************************************************************************************
 * Function:    QMFSynthesisConv
 *
 * Description: final convolution kernel for synthesis QMF
 *
 * Inputs:      pointer to coefficient table, reordered for sequential access
 *              delay buffer of size 64*10 = 640 complex samples (1280 ints)
 *              index for delay ring buffer (range = [0, 9])
 *              number of QMF subbands to process (range = [0, 64])
 *              number of channels
 *
 * Outputs:     64 consecutive 16-bit PCM samples, interleaved by factor of nChans
 *
 * Return:      none
 *
 * Notes:       this is carefully written to be efficient on ARM
 *              use the assembly code version in sbrqmfsk.s when building for ARM!
 **********************************************************************************************************************/
void QMFSynthesisConv(int *cPtr, int *delay, int dIdx, short *outbuf, int nChans) {

    int k, dOff0, dOff1;
    U64 sum64;

    dOff0 = (dIdx)*128;
    dOff1 = dOff0 - 1;
    if (dOff1 < 0)
        dOff1 += 1280;

    /* scaling note: total gain of coefs (cPtr[0]-cPtr[9] for any k) is < 2.0, so 1 GB in delay values is adequate */
    for (k = 0; k <= 63; k++) {
        sum64.w64 = 0;
        sum64.w64 = MADD64(sum64.w64, *cPtr++, delay[dOff0]);   dOff0 -= 256; if (dOff0 < 0) {dOff0 += 1280;}
        sum64.w64 = MADD64(sum64.w64, *cPtr++, delay[dOff1]);   dOff1 -= 256; if (dOff1 < 0) {dOff1 += 1280;}
        sum64.w64 = MADD64(sum64.w64, *cPtr++, delay[dOff0]);   dOff0 -= 256; if (dOff0 < 0) {dOff0 += 1280;}
        sum64.w64 = MADD64(sum64.w64, *cPtr++, delay[dOff1]);   dOff1 -= 256; if (dOff1 < 0) {dOff1 += 1280;}
        sum64.w64 = MADD64(sum64.w64, *cPtr++, delay[dOff0]);   dOff0 -= 256; if (dOff0 < 0) {dOff0 += 1280;}
        sum64.w64 = MADD64(sum64.w64, *cPtr++, delay[dOff1]);   dOff1 -= 256; if (dOff1 < 0) {dOff1 += 1280;}
        sum64.w64 = MADD64(sum64.w64, *cPtr++, delay[dOff0]);   dOff0 -= 256; if (dOff0 < 0) {dOff0 += 1280;}
        sum64.w64 = MADD64(sum64.w64, *cPtr++, delay[dOff1]);   dOff1 -= 256; if (dOff1 < 0) {dOff1 += 1280;}
        sum64.w64 = MADD64(sum64.w64, *cPtr++, delay[dOff0]);   dOff0 -= 256; if (dOff0 < 0) {dOff0 += 1280;}
        sum64.w64 = MADD64(sum64.w64, *cPtr++, delay[dOff1]);   dOff1 -= 256; if (dOff1 < 0) {dOff1 += 1280;}

        dOff0++;
        dOff1--;
        *outbuf = CLIPTOSHORT((sum64.r.hi32 + RND_VAL) >> FBITS_OUT_QMFS);
        outbuf += nChans;
    }
}
/***********************************************************************************************************************
 * Function:    QMFSynthesis
 *
 * Description: 64-subband synthesis QMF (4.6.18.4.2)
 *
 * Inputs:      64 consecutive complex subband QMF samples, format = Q(FBITS_IN_QMFS)
 *              delay buffer of size 64*10 = 640 complex samples (1280 ints)
 *              index for delay ring buffer (range = [0, 9])
 *              number of QMF subbands to process (range = [0, 64])
 *              number of channels
 *
 * Outputs:     64 consecutive 16-bit PCM samples, interleaved by factor of nChans
 *              updated delay buffer
 *              updated delay index
 *
 * Return:      none
 *
 * Notes:       assumes MIN_GBITS_IN_QMFS guard bits in input, either from
 *                QMFAnalysis (if upsampling only) or from MapHF (if SBR on)
 **********************************************************************************************************************/
void QMFSynthesis(int *inbuf, int *delay, int *delayIdx, int qmfsBands, short *outbuf, int nChans) {

    int n, a0, a1, b0, b1, dOff0, dOff1, dIdx;
    int *tBufLo, *tBufHi;

    dIdx = *delayIdx;
    tBufLo = delay + dIdx*128 + 0;
    tBufHi = delay + dIdx*128 + 127;

    /* reorder inputs to DCT-IV, only use first qmfsBands (complex) samples
     */
    for (n = 0; n < qmfsBands >> 1; n++) {
        a0 = *inbuf++;
        b0 = *inbuf++;
        a1 = *inbuf++;
        b1 = *inbuf++;
        *tBufLo++ = a0;
        *tBufLo++ = a1;
        *tBufHi-- = b0;
        *tBufHi-- = b1;
    }
    if (qmfsBands & 0x01) {
        a0 = *inbuf++;
        b0 = *inbuf++;
        *tBufLo++ = a0;
        *tBufHi-- = b0;
        *tBufLo++ = 0;
        *tBufHi-- = 0;
        n++;
    }
    for (     ; n < 32; n++) {
        *tBufLo++ = 0;
        *tBufHi-- = 0;
        *tBufLo++ = 0;
        *tBufHi-- = 0;
    }

    tBufLo = delay + dIdx*128 + 0;
    tBufHi = delay + dIdx*128 + 64;

    /* 2 GB in, 3 GB out */
    PreMultiply64(tBufLo);
    PreMultiply64(tBufHi);

    /* 3 GB in, 1 GB out */
    FFT32C(tBufLo);
    FFT32C(tBufHi);

    /* 1 GB in, 2 GB out */
    PostMultiply64(tBufLo, 64);
    PostMultiply64(tBufHi, 64);

    /* could fuse with PostMultiply64 to avoid separate pass */
    dOff0 = dIdx*128;
    dOff1 = dIdx*128 + 64;
    for (n = 32; n != 0; n--) {
        a0 =  (*tBufLo++);
        a1 =  (*tBufLo++);
        b0 =  (*tBufHi++);
        b1 = -(*tBufHi++);

        delay[dOff0++] = (b0 - a0);
        delay[dOff0++] = (b1 - a1);
        delay[dOff1++] = (b0 + a0);
        delay[dOff1++] = (b1 + a1);
    }

    QMFSynthesisConv((int *)cTabS, delay, dIdx, outbuf, nChans);

    *delayIdx = (*delayIdx == NUM_QMF_DELAY_BUFS - 1 ? 0 : *delayIdx + 1);
}
/***********************************************************************************************************************
 * Function:    UnpackSBRHeader
 *
 * Description: unpack SBR header (table 4.56)
 *
 * Inputs:      BitStreamInfo struct pointing to start of SBR header
 *
 * Outputs:     initialized SBRHeader struct for this SCE/CPE block
 *
 * Return:      non-zero if frame reset is triggered, zero otherwise
 **********************************************************************************************************************/
int UnpackSBRHeader(SBRHeader *sbrHdr) {

    SBRHeader sbrHdrPrev;

    /* save previous values so we know whether to reset decoder */
    sbrHdrPrev.startFreq =     sbrHdr->startFreq;
    sbrHdrPrev.stopFreq =      sbrHdr->stopFreq;
    sbrHdrPrev.freqScale =     sbrHdr->freqScale;
    sbrHdrPrev.alterScale =    sbrHdr->alterScale;
    sbrHdrPrev.crossOverBand = sbrHdr->crossOverBand;
    sbrHdrPrev.noiseBands =    sbrHdr->noiseBands;

    sbrHdr->ampRes =        GetBits(1);
    sbrHdr->startFreq =     GetBits(4);
    sbrHdr->stopFreq =      GetBits(4);
    sbrHdr->crossOverBand = GetBits(3);
    sbrHdr->resBitsHdr =    GetBits(2);
    sbrHdr->hdrExtra1 =     GetBits(1);
    sbrHdr->hdrExtra2 =     GetBits(1);

    if (sbrHdr->hdrExtra1) {
        sbrHdr->freqScale =    GetBits(2);
        sbrHdr->alterScale =   GetBits(1);
        sbrHdr->noiseBands =   GetBits(2);
    } else {
        /* defaults */
        sbrHdr->freqScale =    2;
        sbrHdr->alterScale =   1;
        sbrHdr->noiseBands =   2;
    }

    if (sbrHdr->hdrExtra2) {
        sbrHdr->limiterBands = GetBits(2);
        sbrHdr->limiterGains = GetBits(2);
        sbrHdr->interpFreq =   GetBits(1);
        sbrHdr->smoothMode =   GetBits(1);
    } else {
        /* defaults */
        sbrHdr->limiterBands = 2;
        sbrHdr->limiterGains = 2;
        sbrHdr->interpFreq =   1;
        sbrHdr->smoothMode =   1;
    }
    sbrHdr->count++;

    /* if any of these have changed from previous frame, reset the SBR module */
    if (sbrHdr->startFreq != sbrHdrPrev.startFreq || sbrHdr->stopFreq != sbrHdrPrev.stopFreq ||
        sbrHdr->freqScale != sbrHdrPrev.freqScale || sbrHdr->alterScale != sbrHdrPrev.alterScale ||
        sbrHdr->crossOverBand != sbrHdrPrev.crossOverBand || sbrHdr->noiseBands != sbrHdrPrev.noiseBands
        )
        return -1;
    else
        return 0;
}

/* cLog2[i] = ceil(log2(i)) (disregard i == 0) */
static const uint8_t cLog2[9] = {0, 0, 1, 2, 2, 3, 3, 3, 3};
/***********************************************************************************************************************
 * Function:    UnpackSBRGrid
 *
 * Description: unpack SBR grid (table 4.62)
 *
 * Inputs:      BitStreamInfo struct pointing to start of SBR grid
 *              initialized SBRHeader struct for this SCE/CPE block
 *
 * Outputs:     initialized SBRGrid struct for this channel
 *
 * Return:      none
 **********************************************************************************************************************/
void UnpackSBRGrid(SBRHeader *sbrHdr, SBRGrid *sbrGrid) {

    int numEnvRaw, env, rel, pBits, border, middleBorder = 0;
    uint8_t relBordLead[MAX_NUM_ENV], relBordTrail[MAX_NUM_ENV];
    uint8_t relBorder0[3], relBorder1[3], relBorder[3];
    uint8_t numRelBorder0, numRelBorder1, numRelBorder, numRelLead = 0, numRelTrail;
    uint8_t absBordLead = 0, absBordTrail = 0, absBorder;

    sbrGrid->ampResFrame = sbrHdr->ampRes;
    sbrGrid->frameClass = GetBits(2);
    switch(sbrGrid->frameClass){

        case SBR_GRID_FIXFIX:
            numEnvRaw = GetBits(2);
            sbrGrid->numEnv = (1 << numEnvRaw);
            if(sbrGrid->numEnv == 1) sbrGrid->ampResFrame = 0;

            ASSERT(sbrGrid->numEnv == 1 || sbrGrid->numEnv == 2 || sbrGrid->numEnv == 4);

            sbrGrid->freqRes[0] = GetBits(1);
            for(env = 1; env < sbrGrid->numEnv; env++)
                sbrGrid->freqRes[env] = sbrGrid->freqRes[0];

            absBordLead = 0;
            absBordTrail = NUM_TIME_SLOTS;
            numRelLead = sbrGrid->numEnv - 1;
            numRelTrail = 0;

            /* numEnv = 1, 2, or 4 */
            if(sbrGrid->numEnv == 1)
                border = NUM_TIME_SLOTS / 1;
            else if(sbrGrid->numEnv == 2)
                border = NUM_TIME_SLOTS / 2;
            else
                border = NUM_TIME_SLOTS / 4;

            for(rel = 0; rel < numRelLead; rel++)
                relBordLead[rel] = border;

            middleBorder = (sbrGrid->numEnv >> 1);

            break;

        case SBR_GRID_FIXVAR:
            absBorder = GetBits(2) + NUM_TIME_SLOTS;
            numRelBorder = GetBits(2);
            sbrGrid->numEnv = numRelBorder + 1;
            for(rel = 0; rel < numRelBorder; rel++)
                relBorder[rel] = 2 * GetBits(2) + 2;

            pBits = cLog2[sbrGrid->numEnv + 1];
            sbrGrid->pointer = GetBits(pBits);

            for(env = sbrGrid->numEnv - 1; env >= 0; env--)
                sbrGrid->freqRes[env] = GetBits(1);

            absBordLead = 0;
            absBordTrail = absBorder;
            numRelLead = 0;
            numRelTrail = numRelBorder;

            for(rel = 0; rel < numRelTrail; rel++)
                relBordTrail[rel] = relBorder[rel];

            if(sbrGrid->pointer > 1)
                middleBorder = sbrGrid->numEnv + 1 - sbrGrid->pointer;
            else
                middleBorder = sbrGrid->numEnv - 1;

            break;

        case SBR_GRID_VARFIX:
            absBorder = GetBits(2);
            numRelBorder = GetBits(2);
            sbrGrid->numEnv = numRelBorder + 1;
            for(rel = 0; rel < numRelBorder; rel++)
                relBorder[rel] = 2 * GetBits(2) + 2;

            pBits = cLog2[sbrGrid->numEnv + 1];
            sbrGrid->pointer = GetBits(pBits);

            for(env = 0; env < sbrGrid->numEnv; env++)
                sbrGrid->freqRes[env] = GetBits(1);

            absBordLead = absBorder;
            absBordTrail = NUM_TIME_SLOTS;
            numRelLead = numRelBorder;
            numRelTrail = 0;

            for(rel = 0; rel < numRelLead; rel++)
                relBordLead[rel] = relBorder[rel];

            if(sbrGrid->pointer == 0)
                middleBorder = 1;
            else if(sbrGrid->pointer == 1)
                middleBorder = sbrGrid->numEnv - 1;
            else
                middleBorder = sbrGrid->pointer - 1;

            break;

        case SBR_GRID_VARVAR:
            absBordLead = GetBits(2); /* absBorder0 */
            absBordTrail = GetBits(2) + NUM_TIME_SLOTS; /* absBorder1 */
            numRelBorder0 = GetBits(2);
            numRelBorder1 = GetBits(2);

            sbrGrid->numEnv = numRelBorder0 + numRelBorder1 + 1;
            ASSERT(sbrGrid->numEnv <= 5);

            for(rel = 0; rel < numRelBorder0; rel++)
                relBorder0[rel] = 2 * GetBits(2) + 2;

            for(rel = 0; rel < numRelBorder1; rel++)
                relBorder1[rel] = 2 * GetBits(2) + 2;

            pBits = cLog2[numRelBorder0 + numRelBorder1 + 2];
            sbrGrid->pointer = GetBits(pBits);

            for(env = 0; env < sbrGrid->numEnv; env++)
                sbrGrid->freqRes[env] = GetBits(1);

            numRelLead = numRelBorder0;
            numRelTrail = numRelBorder1;

            for(rel = 0; rel < numRelLead; rel++)
                relBordLead[rel] = relBorder0[rel];

            for(rel = 0; rel < numRelTrail; rel++)
                relBordTrail[rel] = relBorder1[rel];

            if(sbrGrid->pointer > 1)
                middleBorder = sbrGrid->numEnv + 1 - sbrGrid->pointer;
            else
                middleBorder = sbrGrid->numEnv - 1;

            break;
    }

    /* build time border vector */
    sbrGrid->envTimeBorder[0] = absBordLead * SAMPLES_PER_SLOT;

    rel = 0;
    border = absBordLead;
    for(env = 1; env <= numRelLead; env++) {
        border += relBordLead[rel++];
        sbrGrid->envTimeBorder[env] = border * SAMPLES_PER_SLOT;
    }

    rel = 0;
    border = absBordTrail;
    for(env = sbrGrid->numEnv - 1; env > numRelLead; env--) {
        border -= relBordTrail[rel++];
        sbrGrid->envTimeBorder[env] = border * SAMPLES_PER_SLOT;
    }

    sbrGrid->envTimeBorder[sbrGrid->numEnv] = absBordTrail * SAMPLES_PER_SLOT;

    if(sbrGrid->numEnv > 1) {
        sbrGrid->numNoiseFloors = 2;
        sbrGrid->noiseTimeBorder[0] = sbrGrid->envTimeBorder[0];
        sbrGrid->noiseTimeBorder[1] = sbrGrid->envTimeBorder[middleBorder];
        sbrGrid->noiseTimeBorder[2] = sbrGrid->envTimeBorder[sbrGrid->numEnv];
    }
    else {
        sbrGrid->numNoiseFloors = 1;
        sbrGrid->noiseTimeBorder[0] = sbrGrid->envTimeBorder[0];
        sbrGrid->noiseTimeBorder[1] = sbrGrid->envTimeBorder[1];
    }
}
/***********************************************************************************************************************
 * Function:    UnpackDeltaTimeFreq
 *
 * Description: unpack time/freq flags for delta coding of SBR envelopes (table 4.63)
 *
 * Inputs:      BitStreamInfo struct pointing to start of dt/df flags
 *              number of envelopes
 *              number of noise floors
 *
 * Outputs:     delta flags for envelope and noise floors
 *
 * Return:      none
 **********************************************************************************************************************/
void UnpackDeltaTimeFreq(int numEnv, uint8_t *deltaFlagEnv, int numNoiseFloors, uint8_t *deltaFlagNoise) {

    int env, noiseFloor;

    for (env = 0; env < numEnv; env++)
        deltaFlagEnv[env] = GetBits(1);

    for (noiseFloor = 0; noiseFloor < numNoiseFloors; noiseFloor++)
        deltaFlagNoise[noiseFloor] = GetBits(1);
}
/***********************************************************************************************************************
 * Function:    UnpackInverseFilterMode
 *
 * Description: unpack invf flags for chirp factor calculation (table 4.64)
 *
 * Inputs:      BitStreamInfo struct pointing to start of invf flags
 *              number of noise floor bands
 *
 * Outputs:     invf flags for noise floor bands
 *
 * Return:      none
 **********************************************************************************************************************/
void UnpackInverseFilterMode(int numNoiseFloorBands, uint8_t *mode) {

    int n;

    for (n = 0; n < numNoiseFloorBands; n++)
        mode[n] = GetBits(2);
}
/***********************************************************************************************************************
 * Function:    UnpackSinusoids
 *
 * Description: unpack sinusoid (harmonic) flags for each SBR subband (table 4.67)
 *
 * Inputs:      BitStreamInfo struct pointing to start of sinusoid flags
 *              number of high resolution SBR subbands (nHigh)
 *
 * Outputs:     sinusoid flags for each SBR subband, zero-filled above nHigh
 *
 * Return:      none
 **********************************************************************************************************************/
void UnpackSinusoids(int nHigh, int addHarmonicFlag, uint8_t *addHarmonic) {

    int n;

    n = 0;
    if(addHarmonicFlag) {
        for(; n < nHigh; n++)
            addHarmonic[n] = GetBits(1);
    }

    /* zero out unused bands */
    for(; n < MAX_QMF_BANDS; n++)
        addHarmonic[n] = 0;
}
/***********************************************************************************************************************
 * Function:    CopyCouplingGrid
 *
 * Description: copy grid parameters from left to right for channel coupling
 *
 * Inputs:      initialized SBRGrid struct for left channel
 *
 * Outputs:     initialized SBRGrid struct for right channel
 *
 * Return:      none
 **********************************************************************************************************************/
void CopyCouplingGrid(SBRGrid *sbrGridLeft, SBRGrid *sbrGridRight) {

    int env, noiseFloor;

    sbrGridRight->frameClass = sbrGridLeft->frameClass;
    sbrGridRight->ampResFrame = sbrGridLeft->ampResFrame;
    sbrGridRight->pointer = sbrGridLeft->pointer;

    sbrGridRight->numEnv = sbrGridLeft->numEnv;
    for(env = 0; env < sbrGridLeft->numEnv; env++) {
        sbrGridRight->envTimeBorder[env] = sbrGridLeft->envTimeBorder[env];
        sbrGridRight->freqRes[env] = sbrGridLeft->freqRes[env];
    }
    sbrGridRight->envTimeBorder[env] = sbrGridLeft->envTimeBorder[env]; /* borders are [0, numEnv] inclusive */

    sbrGridRight->numNoiseFloors = sbrGridLeft->numNoiseFloors;
    for(noiseFloor = 0; noiseFloor <= sbrGridLeft->numNoiseFloors; noiseFloor++)
        sbrGridRight->noiseTimeBorder[noiseFloor] = sbrGridLeft->noiseTimeBorder[noiseFloor];

    /* numEnvPrev, numNoiseFloorsPrev, freqResPrev are updated in DecodeSBREnvelope() and DecodeSBRNoise() */
}
/***********************************************************************************************************************
 * Function:    CopyCouplingInverseFilterMode
 *
 * Description: copy invf flags from left to right for channel coupling
 *
 * Inputs:      invf flags for left channel
 *              number of noise floor bands
 *
 * Outputs:     invf flags for right channel
 *
 * Return:      none
 **********************************************************************************************************************/
void CopyCouplingInverseFilterMode(int numNoiseFloorBands, uint8_t *modeLeft, uint8_t *modeRight) {

    int band;

    for(band = 0; band < numNoiseFloorBands; band++)
        modeRight[band] = modeLeft[band];
}
/***********************************************************************************************************************
 * Function:    UnpackSBRSingleChannel
 *
 * Description: unpack sideband info (grid, delta flags, invf flags, envelope and
 *                noise floor configuration, sinusoids) for a single channel
 *
 * Inputs:      BitStreamInfo struct pointing to start of sideband info
 *              initialized PSInfoSBR struct (after parsing SBR header and building
 *                frequency tables)
 *              base output channel (range = [0, nChans-1])
 *
 * Outputs:     updated PSInfoSBR struct (SBRGrid and SBRChan)
 *
 * Return:      none
 **********************************************************************************************************************/
void UnpackSBRSingleChannel(int chBase) {

    int bitsLeft;
    SBRHeader *sbrHdr = &(m_PSInfoSBR->sbrHdr[chBase]);
    SBRGrid *sbrGridL = &(m_PSInfoSBR->sbrGrid[chBase + 0]);
    SBRFreq *sbrFreq = &(m_PSInfoSBR->sbrFreq[chBase]);
    SBRChan *sbrChanL = &(m_PSInfoSBR->sbrChan[chBase + 0]);

    m_PSInfoSBR->dataExtra = GetBits(1);
    if(m_PSInfoSBR->dataExtra) m_PSInfoSBR->resBitsData = GetBits(4);

    UnpackSBRGrid(sbrHdr, sbrGridL);
    UnpackDeltaTimeFreq(sbrGridL->numEnv, sbrChanL->deltaFlagEnv, sbrGridL->numNoiseFloors, sbrChanL->deltaFlagNoise);
    UnpackInverseFilterMode(sbrFreq->numNoiseFloorBands, sbrChanL->invfMode[1]);

    DecodeSBREnvelope(sbrGridL, sbrFreq, sbrChanL, 0);
    DecodeSBRNoise(sbrGridL, sbrFreq, sbrChanL, 0);

    sbrChanL->addHarmonicFlag[1] = GetBits(1);
    UnpackSinusoids(sbrFreq->nHigh, sbrChanL->addHarmonicFlag[1], sbrChanL->addHarmonic[1]);

    m_PSInfoSBR->extendedDataPresent = GetBits(1);
    if(m_PSInfoSBR->extendedDataPresent) {
        m_PSInfoSBR->extendedDataSize = GetBits(4);
        if(m_PSInfoSBR->extendedDataSize == 15) m_PSInfoSBR->extendedDataSize += GetBits(8);

        bitsLeft = 8 * m_PSInfoSBR->extendedDataSize;

        /* get ID, unpack extension info, do whatever is necessary with it... */
        while(bitsLeft > 0) {
            GetBits(8);
            bitsLeft -= 8;
        }
    }
}
/***********************************************************************************************************************
 * Function:    UnpackSBRChannelPair
 *
 * Description: unpack sideband info (grid, delta flags, invf flags, envelope and
 *                noise floor configuration, sinusoids) for a channel pair
 *
 * Inputs:      base output channel (range = [0, nChans-1])
 *
 * Outputs:     updated PSInfoSBR struct (SBRGrid and SBRChan for both channels)
 *
 * Return:      none
 **********************************************************************************************************************/
void UnpackSBRChannelPair(int chBase) {

    int bitsLeft;
    SBRHeader *sbrHdr = &(m_PSInfoSBR->sbrHdr[chBase]);
    SBRGrid *sbrGridL = &(m_PSInfoSBR->sbrGrid[chBase + 0]), *sbrGridR = &(m_PSInfoSBR->sbrGrid[chBase + 1]);
    SBRFreq *sbrFreq = &(m_PSInfoSBR->sbrFreq[chBase]);
    SBRChan *sbrChanL = &(m_PSInfoSBR->sbrChan[chBase + 0]), *sbrChanR = &(m_PSInfoSBR->sbrChan[chBase + 1]);

    m_PSInfoSBR->dataExtra = GetBits(1);
    if(m_PSInfoSBR->dataExtra) {
        m_PSInfoSBR->resBitsData = GetBits(4);
        m_PSInfoSBR->resBitsData = GetBits(4);
    }

    m_PSInfoSBR->couplingFlag = GetBits(1);
    if(m_PSInfoSBR->couplingFlag) {
        UnpackSBRGrid(sbrHdr, sbrGridL);
        CopyCouplingGrid(sbrGridL, sbrGridR);

        UnpackDeltaTimeFreq(sbrGridL->numEnv, sbrChanL->deltaFlagEnv, sbrGridL->numNoiseFloors,
                sbrChanL->deltaFlagNoise);
        UnpackDeltaTimeFreq(sbrGridR->numEnv, sbrChanR->deltaFlagEnv, sbrGridR->numNoiseFloors,
                sbrChanR->deltaFlagNoise);

        UnpackInverseFilterMode(sbrFreq->numNoiseFloorBands, sbrChanL->invfMode[1]);
        CopyCouplingInverseFilterMode(sbrFreq->numNoiseFloorBands, sbrChanL->invfMode[1], sbrChanR->invfMode[1]);

        DecodeSBREnvelope(sbrGridL, sbrFreq, sbrChanL, 0);
        DecodeSBRNoise(sbrGridL, sbrFreq, sbrChanL, 0);
        DecodeSBREnvelope(sbrGridR, sbrFreq, sbrChanR, 1);
        DecodeSBRNoise(sbrGridR, sbrFreq, sbrChanR, 1);

        /* pass RIGHT sbrChan struct */
        UncoupleSBREnvelope(sbrGridL, sbrFreq, sbrChanR);
        UncoupleSBRNoise(sbrGridL, sbrFreq, sbrChanR);

    }
    else {
        UnpackSBRGrid(sbrHdr, sbrGridL);
        UnpackSBRGrid(sbrHdr, sbrGridR);
        UnpackDeltaTimeFreq(sbrGridL->numEnv, sbrChanL->deltaFlagEnv, sbrGridL->numNoiseFloors,
                sbrChanL->deltaFlagNoise);
        UnpackDeltaTimeFreq(sbrGridR->numEnv, sbrChanR->deltaFlagEnv, sbrGridR->numNoiseFloors,
                sbrChanR->deltaFlagNoise);
        UnpackInverseFilterMode(sbrFreq->numNoiseFloorBands, sbrChanL->invfMode[1]);
        UnpackInverseFilterMode(sbrFreq->numNoiseFloorBands, sbrChanR->invfMode[1]);

        DecodeSBREnvelope(sbrGridL, sbrFreq, sbrChanL, 0);
        DecodeSBREnvelope(sbrGridR, sbrFreq, sbrChanR, 1);
        DecodeSBRNoise(sbrGridL, sbrFreq, sbrChanL, 0);
        DecodeSBRNoise(sbrGridR, sbrFreq, sbrChanR, 1);
    }

    sbrChanL->addHarmonicFlag[1] = GetBits(1);
    UnpackSinusoids(sbrFreq->nHigh, sbrChanL->addHarmonicFlag[1], sbrChanL->addHarmonic[1]);

    sbrChanR->addHarmonicFlag[1] = GetBits(1);
    UnpackSinusoids(sbrFreq->nHigh, sbrChanR->addHarmonicFlag[1], sbrChanR->addHarmonic[1]);

    m_PSInfoSBR->extendedDataPresent = GetBits(1);
    if(m_PSInfoSBR->extendedDataPresent) {
        m_PSInfoSBR->extendedDataSize = GetBits(4);
        if(m_PSInfoSBR->extendedDataSize == 15) m_PSInfoSBR->extendedDataSize += GetBits(8);

        bitsLeft = 8 * m_PSInfoSBR->extendedDataSize;

        /* get ID, unpack extension info, do whatever is necessary with it... */
        while(bitsLeft > 0) {
            GetBits(8);
            bitsLeft -= 8;
        }
    }
}
