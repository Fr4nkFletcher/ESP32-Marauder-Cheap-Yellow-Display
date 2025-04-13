/*
 * flac_decoder.cpp
 * Java source code from https://www.nayuki.io/page/simple-flac-implementation
 * adapted to ESP32
 *
 * Created on: Jul 03,2020
 * Updated on: Jul 03,2021
 *
 * Author: Wolle
 *
 *
 */
#include "flac_decoder.h"
#include "vector"
using namespace std;


FLACFrameHeader_t   *FLACFrameHeader;
FLACMetadataBlock_t *FLACMetadataBlock;
FLACsubFramesBuff_t *FLACsubFramesBuff;

vector<int32_t>coefs;
const uint16_t outBuffSize = 2048;
uint16_t m_blockSize=0;
uint16_t m_blockSizeLeft = 0;
uint16_t m_validSamples = 0;
uint8_t  m_status = 0;
uint8_t* m_inptr;
int16_t  m_bytesAvail;
int16_t  m_bytesDecoded = 0;
float    m_compressionRatio = 0;
uint16_t m_rIndex=0;
uint64_t m_bitBuffer = 0;
uint8_t  m_bitBufferLen = 0;
bool     m_f_OggS_found = false;

//----------------------------------------------------------------------------------------------------------------------
//          FLAC INI SECTION
//----------------------------------------------------------------------------------------------------------------------
bool FLACDecoder_AllocateBuffers(void){
    if(psramFound()) {
        // PSRAM found, Buffer will be allocated in PSRAM
        if(!FLACFrameHeader)    {FLACFrameHeader   = (FLACFrameHeader_t*)    ps_malloc(sizeof(FLACFrameHeader_t));}
        if(!FLACMetadataBlock)  {FLACMetadataBlock = (FLACMetadataBlock_t*)  ps_malloc(sizeof(FLACMetadataBlock_t));}
        if(!FLACsubFramesBuff)  {FLACsubFramesBuff = (FLACsubFramesBuff_t*)  ps_malloc(sizeof(FLACsubFramesBuff_t));}
    }
    else {
        if(!FLACFrameHeader)    {FLACFrameHeader   = (FLACFrameHeader_t*)    malloc(sizeof(FLACFrameHeader_t));}
        if(!FLACMetadataBlock)  {FLACMetadataBlock = (FLACMetadataBlock_t*)  malloc(sizeof(FLACMetadataBlock_t));}
        if(!FLACsubFramesBuff)  {FLACsubFramesBuff = (FLACsubFramesBuff_t*)  malloc(sizeof(FLACsubFramesBuff_t));}
    }
    if(!FLACFrameHeader || !FLACMetadataBlock || !FLACsubFramesBuff ){
        log_e("not enough memory to allocate flacdecoder buffers");
        return false;
    }
    FLACDecoder_ClearBuffer();
    return true;
}
//----------------------------------------------------------------------------------------------------------------------
void FLACDecoder_ClearBuffer(){
    memset(FLACFrameHeader,   0, sizeof(FLACFrameHeader_t));
    memset(FLACMetadataBlock, 0, sizeof(FLACMetadataBlock_t));
    memset(FLACsubFramesBuff, 0, sizeof(FLACsubFramesBuff_t));
    m_status = DECODE_FRAME;
    return;
}
//----------------------------------------------------------------------------------------------------------------------
void FLACDecoder_FreeBuffers(){
    if(FLACFrameHeader)    {free(FLACFrameHeader);   FLACFrameHeader   = NULL;}
    if(FLACMetadataBlock)  {free(FLACMetadataBlock); FLACMetadataBlock = NULL;}
    if(FLACsubFramesBuff)  {free(FLACsubFramesBuff); FLACsubFramesBuff = NULL;}
}
//----------------------------------------------------------------------------------------------------------------------
//            B I T R E A D E R
//----------------------------------------------------------------------------------------------------------------------
uint32_t readUint(uint8_t nBits){
    while (m_bitBufferLen < nBits){
        uint8_t temp = *(m_inptr + m_rIndex);
        m_rIndex++;
        m_bytesAvail--;
        if(m_bytesAvail < 0) { log_i("error in bitreader"); }
        m_bitBuffer = (m_bitBuffer << 8) | temp;
        m_bitBufferLen += 8;
    }
    m_bitBufferLen -= nBits;
    uint32_t result = m_bitBuffer >> m_bitBufferLen;
    if (nBits < 32)
        result &= (1 << nBits) - 1;
    return result;
}

int32_t readSignedInt(int nBits){
    int32_t temp = readUint(nBits) << (32 - nBits);
    temp = temp >> (32 - nBits); // The C++ compiler uses the sign bit to fill vacated bit positions
    return temp;
}

int64_t readRiceSignedInt(uint8_t param){
    long val = 0;
    while (readUint(1) == 0)
        val++;
    val = (val << param) | readUint(param);
    return (val >> 1) ^ -(val & 1);
}

void alignToByte() {
    m_bitBufferLen -= m_bitBufferLen % 8;
}
//----------------------------------------------------------------------------------------------------------------------
//              F L A C - D E C O D E R
//----------------------------------------------------------------------------------------------------------------------
void FLACSetRawBlockParams(uint8_t Chans, uint32_t SampRate, uint8_t BPS, uint32_t tsis, uint32_t AuDaLength){
    FLACMetadataBlock->numChannels = Chans;
    FLACMetadataBlock->sampleRate = SampRate;
    FLACMetadataBlock->bitsPerSample = BPS;
    FLACMetadataBlock->totalSamples = tsis;  // total samples in stream
    FLACMetadataBlock->audioDataLength = AuDaLength;
}
//----------------------------------------------------------------------------------------------------------------------
void FLACDecoderReset(){ // set var to default
    m_status = DECODE_FRAME;
    m_bitBuffer = 0;
    m_bitBufferLen = 0;
}
//----------------------------------------------------------------------------------------------------------------------
int FLACFindSyncWord(unsigned char *buf, int nBytes) {
    int i;

    /* find byte-aligned syncword - need 13 matching bits */
    for (i = 0; i < nBytes - 1; i++) {
        if ((buf[i + 0] & 0xFF) == 0xFF  && (buf[i + 1] & 0xF8) == 0xF8) {
            FLACDecoderReset();
            return i;
        }
    }
    return -1;
}
//----------------------------------------------------------------------------------------------------------------------
int FLACFindOggSyncWord(unsigned char *buf, int nBytes){
    int i;

    /* find byte-aligned syncword - need 13 matching bits */
    for (i = 0; i < nBytes - 1; i++) {
        if ((buf[i + 0] & 0xFF) == 0xFF  && (buf[i + 1] & 0xF8) == 0xF8) {
            FLACDecoderReset();
            log_i("FLAC sync found");
            return i;
        }
    }
    /* find byte-aligned OGG Magic - OggS */
    for (i = 0; i < nBytes - 1; i++) {
        if ((buf[i + 0] == 'O') && (buf[i + 1] == 'g') && (buf[i + 2] == 'g') && (buf[i + 3] == 'S')) {
            FLACDecoderReset();
            log_i("OggS found");
            m_f_OggS_found = true;
            return i;
        }
    }
    return -1;
}
//----------------------------------------------------------------------------------------------------------------------
int FLACparseOggHeader(unsigned char *buf){
    uint8_t i = 0;
    uint8_t ssv = *(buf + i);                  // stream_structure_version
    (void)ssv;
    i++;
    uint8_t htf = *(buf + i);                  // header_type_flag
    (void)htf;
    i++;
    uint32_t tmp = 0;                         // absolute granule position
    for (int j = 0; j < 4; j++) {
        tmp += *(buf + j + i) << (4 -j - 1) * 8;
    }
    i += 4;
    uint64_t agp = (uint64_t) tmp << 32;
    for (int j = 0; j < 4; j++) {
        agp += *(buf + j + i) << (4 -j - 1) * 8;
    }
    i += 4;
    uint32_t ssnr = 0;                        // stream serial number
    for (int j = 0; j < 4; j++) {
        ssnr += *(buf + j + i) << (4 -j - 1) * 8;
    }
    i += 4;
    uint32_t psnr = 0;                        // page sequence no
    for (int j = 0; j < 4; j++) {
        psnr += *(buf + j + i) << (4 -j - 1) * 8;
    }
    i += 4;
    uint32_t pchk = 0;                        // page checksum
    for (int j = 0; j < 4; j++) {
        pchk += *(buf + j + i) << (4 -j - 1) * 8;
    }
    i += 4;
    uint8_t psegm = *(buf + i);
    i++;
    uint8_t psegmBuff[256];
    uint32_t pageLen = 0;
    for(uint8_t j = 0; j < psegm; j++){
        psegmBuff[j] = *(buf + i);
        pageLen += psegmBuff[j];
        i++;
    }
    return i;
}
//----------------------------------------------------------------------------------------------------------------------
int8_t FLACDecode(uint8_t *inbuf, int *bytesLeft, short *outbuf){

    if(m_f_OggS_found == true){
        m_f_OggS_found = false;
        *bytesLeft -= FLACparseOggHeader(inbuf);
        return ERR_FLAC_NONE;
    }

    if(m_status != OUT_SAMPLES){
        m_rIndex = 0;
        m_bytesAvail = (*bytesLeft);
        m_inptr = inbuf;
    }

    if(m_status == DECODE_FRAME){  // Read a ton of header fields, and ignore most of them

        if ((inbuf[0] == 'O') && (inbuf[1] == 'g') && (inbuf[2] == 'g') && (inbuf[3] == 'S')){
            *bytesLeft -= 4;
            m_f_OggS_found = true;
            return ERR_FLAC_NONE;
        }

        uint32_t temp = readUint(8);
        uint16_t sync = temp << 6 |readUint(6);
        if (sync != 0x3FFE){
            log_i("Sync code expected 0x3FFE but received %X", sync);
            return ERR_FLAC_SYNC_CODE_NOT_FOUND;
        }

        readUint(1);
        FLACFrameHeader->blockingStrategy = readUint(1);
        FLACFrameHeader->blockSizeCode = readUint(4);
        FLACFrameHeader->sampleRateCode = readUint(4);
        FLACFrameHeader->chanAsgn = readUint(4);
        FLACFrameHeader->sampleSizeCode = readUint(3);

        if(!FLACMetadataBlock->numChannels){
            if(FLACFrameHeader->chanAsgn == 0) FLACMetadataBlock->numChannels = 1;
            if(FLACFrameHeader->chanAsgn == 1) FLACMetadataBlock->numChannels = 2;
            if(FLACFrameHeader->chanAsgn > 7)  FLACMetadataBlock->numChannels = 2;
        }
        if(FLACMetadataBlock->numChannels < 1) return ERR_FLAC_UNKNOWN_CHANNEL_ASSIGNMENT;

        if(!FLACMetadataBlock->bitsPerSample){
            if(FLACFrameHeader->sampleSizeCode == 1) FLACMetadataBlock->bitsPerSample =  8;
            if(FLACFrameHeader->sampleSizeCode == 2) FLACMetadataBlock->bitsPerSample = 12;
            if(FLACFrameHeader->sampleSizeCode == 4) FLACMetadataBlock->bitsPerSample = 16;
            if(FLACFrameHeader->sampleSizeCode == 5) FLACMetadataBlock->bitsPerSample = 20;
            if(FLACFrameHeader->sampleSizeCode == 6) FLACMetadataBlock->bitsPerSample = 24;
        }
        if(FLACMetadataBlock->bitsPerSample > 16) return ERR_FLAC_BITS_PER_SAMPLE_TOO_BIG;
        if(FLACMetadataBlock->bitsPerSample < 8 ) return ERR_FLAG_BITS_PER_SAMPLE_UNKNOWN;

        if(!FLACMetadataBlock->sampleRate){
            if(FLACFrameHeader->sampleRateCode == 1)  FLACMetadataBlock->sampleRate =  88200;
            if(FLACFrameHeader->sampleRateCode == 2)  FLACMetadataBlock->sampleRate = 176400;
            if(FLACFrameHeader->sampleRateCode == 3)  FLACMetadataBlock->sampleRate = 192000;
            if(FLACFrameHeader->sampleRateCode == 4)  FLACMetadataBlock->sampleRate =   8000;
            if(FLACFrameHeader->sampleRateCode == 5)  FLACMetadataBlock->sampleRate =  16000;
            if(FLACFrameHeader->sampleRateCode == 6)  FLACMetadataBlock->sampleRate =  22050;
            if(FLACFrameHeader->sampleRateCode == 7)  FLACMetadataBlock->sampleRate =  24000;
            if(FLACFrameHeader->sampleRateCode == 8)  FLACMetadataBlock->sampleRate =  32000;
            if(FLACFrameHeader->sampleRateCode == 9)  FLACMetadataBlock->sampleRate =  44100;
            if(FLACFrameHeader->sampleRateCode == 10) FLACMetadataBlock->sampleRate =  48000;
            if(FLACFrameHeader->sampleRateCode == 11) FLACMetadataBlock->sampleRate =  96000;
        }

        readUint(1);
        temp = (readUint(8) << 24);
        temp = ~temp;

        uint32_t shift = 0x80000000; // Number of leading zeros
        int8_t count = 0;
        for(int i=0; i<32; i++){
            if((temp & shift) == 0) {count++; shift >>= 1;}
            else break;
        }
        count--;
        for (int i = 0; i < count; i++) readUint(8);
        m_blockSize = 0;

        if (FLACFrameHeader->blockSizeCode == 1)
            m_blockSize = 192;
        else if (2 <= FLACFrameHeader->blockSizeCode && FLACFrameHeader->blockSizeCode <= 5)
            m_blockSize = 576 << (FLACFrameHeader->blockSizeCode - 2);
        else if (FLACFrameHeader->blockSizeCode == 6)
            m_blockSize = readUint(8) + 1;
        else if (FLACFrameHeader->blockSizeCode == 7)
            m_blockSize = readUint(16) + 1;
        else if (8 <= FLACFrameHeader->blockSizeCode && FLACFrameHeader->blockSizeCode <= 15)
            m_blockSize = 256 << (FLACFrameHeader->blockSizeCode - 8);
        else{
            return ERR_FLAC_RESERVED_BLOCKSIZE_UNSUPPORTED;
        }

        if(m_blockSize > 8192){
            log_e("Error: blockSize too big");
            return ERR_FLAC_BLOCKSIZE_TOO_BIG;
        }

        if(FLACFrameHeader->sampleRateCode == 12)
            readUint(8);
        else if (FLACFrameHeader->sampleRateCode == 13 || FLACFrameHeader->sampleRateCode == 14){
            readUint(16);
        }
        readUint(8);
        m_status = DECODE_SUBFRAMES;
        *bytesLeft = m_bytesAvail;
        m_blockSizeLeft = m_blockSize;

        return ERR_FLAC_NONE;
    }

    if(m_status == DECODE_SUBFRAMES){

        // Decode each channel's subframe, then skip footer
        int ret = decodeSubframes();
        if(ret != 0) return ret;
        m_status = OUT_SAMPLES;
    }

    if(m_status == OUT_SAMPLES){  // Write the decoded samples
        // blocksize can be much greater than outbuff, so we can't stuff all in once
        // therefore we need often more than one loop (split outputblock into pieces)
        uint16_t blockSize;
        static uint16_t offset = 0;
        if(m_blockSize < outBuffSize + offset) blockSize = m_blockSize - offset;
        else blockSize = outBuffSize;


        for (int i = 0; i < blockSize; i++) {
            for (int j = 0; j < FLACMetadataBlock->numChannels; j++) {
                int val = FLACsubFramesBuff->samplesBuffer[j][i + offset];
                if (FLACMetadataBlock->bitsPerSample == 8) val += 128;
                outbuf[2*i+j] = val;
            }
        }

        m_validSamples = blockSize * FLACMetadataBlock->numChannels;
        offset += blockSize;

        if(offset != m_blockSize) return GIVE_NEXT_LOOP;
        offset = 0;
        if(offset > m_blockSize) { log_e("offset has a wrong value"); }
    }

    alignToByte();
    readUint(16);
    m_bytesDecoded = *bytesLeft - m_bytesAvail;
//    log_i("m_bytesDecoded %i", m_bytesDecoded);
//    m_compressionRatio = (float)m_bytesDecoded / (float)m_blockSize * FLACMetadataBlock->numChannels * (16/8);
//    log_i("m_compressionRatio % f", m_compressionRatio);
    *bytesLeft = m_bytesAvail;
    m_status = DECODE_FRAME;
    return ERR_FLAC_NONE;
}
//----------------------------------------------------------------------------------------------------------------------
uint16_t FLACGetOutputSamps(){
    int vs = m_validSamples;
    m_validSamples=0;
    return vs;
}
//----------------------------------------------------------------------------------------------------------------------
uint64_t FLACGetTotoalSamplesInStream(){
    return FLACMetadataBlock->totalSamples;
}
//----------------------------------------------------------------------------------------------------------------------
uint8_t FLACGetBitsPerSample(){
    return FLACMetadataBlock->bitsPerSample;
}
//----------------------------------------------------------------------------------------------------------------------
uint8_t FLACGetChannels(){
    return FLACMetadataBlock->numChannels;
}
//----------------------------------------------------------------------------------------------------------------------
uint32_t FLACGetSampRate(){
    return FLACMetadataBlock->sampleRate;
}
//----------------------------------------------------------------------------------------------------------------------
uint32_t FLACGetBitRate(){
    if(FLACMetadataBlock->totalSamples){
        float BitsPerSamp = (float)FLACMetadataBlock->audioDataLength / (float)FLACMetadataBlock->totalSamples * 8;
        return ((uint32_t)BitsPerSamp * FLACMetadataBlock->sampleRate);
    }
    return 0;
}
//----------------------------------------------------------------------------------------------------------------------
uint32_t FLACGetAudioFileDuration() {
    if(FLACGetSampRate()){
        uint32_t afd = FLACGetTotoalSamplesInStream()/ FLACGetSampRate(); // AudioFileDuration
        return afd;
    }
    return 0;
}
//----------------------------------------------------------------------------------------------------------------------
int8_t decodeSubframes(){
    if(FLACFrameHeader->chanAsgn <= 7) {
        for (int ch = 0; ch < FLACMetadataBlock->numChannels; ch++)
            decodeSubframe(FLACMetadataBlock->bitsPerSample, ch);
    }
    else if (8 <= FLACFrameHeader->chanAsgn && FLACFrameHeader->chanAsgn <= 10) {
        decodeSubframe(FLACMetadataBlock->bitsPerSample + (FLACFrameHeader->chanAsgn == 9 ? 1 : 0), 0);
        decodeSubframe(FLACMetadataBlock->bitsPerSample + (FLACFrameHeader->chanAsgn == 9 ? 0 : 1), 1);
        if(FLACFrameHeader->chanAsgn == 8) {
            for (int i = 0; i < m_blockSize; i++)
                FLACsubFramesBuff->samplesBuffer[1][i] = (
                        FLACsubFramesBuff->samplesBuffer[0][i] -
                        FLACsubFramesBuff->samplesBuffer[1][i]);
        }
        else if (FLACFrameHeader->chanAsgn == 9) {
            for (int i = 0; i < m_blockSize; i++)
                FLACsubFramesBuff->samplesBuffer[0][i] += FLACsubFramesBuff->samplesBuffer[1][i];
        }
        else if (FLACFrameHeader->chanAsgn == 10) {
            for (int i = 0; i < m_blockSize; i++) {
                long side =  FLACsubFramesBuff->samplesBuffer[1][i];
                long right = FLACsubFramesBuff->samplesBuffer[0][i] - (side >> 1);
                FLACsubFramesBuff->samplesBuffer[1][i] = right;
                FLACsubFramesBuff->samplesBuffer[0][i] = right + side;
            }
        }
        else {
            log_e("unknown channel assignment");
            return ERR_FLAC_UNKNOWN_CHANNEL_ASSIGNMENT;
        }
    }
    else{
        log_e("Reserved channel assignment");
        return ERR_FLAC_RESERVED_CHANNEL_ASSIGNMENT;
    }
    return ERR_FLAC_NONE;
}
//----------------------------------------------------------------------------------------------------------------------
int8_t decodeSubframe(uint8_t sampleDepth, uint8_t ch) {
    int8_t ret = 0;
    readUint(1);
    uint8_t type = readUint(6);
    int shift = readUint(1);
    if (shift == 1) {
        while (readUint(1) == 0)
            shift++;
    }
    sampleDepth -= shift;

    if(type == 0){  // Constant coding
        int16_t s= readSignedInt(sampleDepth);
        for(int i=0; i < m_blockSize; i++){
            FLACsubFramesBuff->samplesBuffer[ch][i] = s;
        }
    }
    else if (type == 1) {  // Verbatim coding
        for (int i = 0; i < m_blockSize; i++)
            FLACsubFramesBuff->samplesBuffer[ch][i] = readSignedInt(sampleDepth);
    }
    else if (8 <= type && type <= 12){
        ret = decodeFixedPredictionSubframe(type - 8, sampleDepth, ch);
        if(ret) return ret;
    }
    else if (32 <= type && type <= 63){
        ret = decodeLinearPredictiveCodingSubframe(type - 31, sampleDepth, ch);
        if(ret) return ret;
    }
    else{
        return ERR_FLAC_RESERVED_SUB_TYPE;
    }
    if(shift>0){
        for (int i = 0; i < m_blockSize; i++){
            FLACsubFramesBuff->samplesBuffer[ch][i] <<= shift;
        }
    }
    return ERR_FLAC_NONE;
}
//----------------------------------------------------------------------------------------------------------------------
int8_t decodeFixedPredictionSubframe(uint8_t predOrder, uint8_t sampleDepth, uint8_t ch) {
    uint8_t ret = 0;
    for(uint8_t i = 0; i < predOrder; i++)
        FLACsubFramesBuff->samplesBuffer[ch][i] = readSignedInt(sampleDepth);
    ret = decodeResiduals(predOrder, ch);
    if(ret) return ret;
    coefs.clear();
    if(predOrder == 0) coefs.resize(0);
    if(predOrder == 1) coefs.push_back(1);  // FIXED_PREDICTION_COEFFICIENTS
    if(predOrder == 2){coefs.push_back(2); coefs.push_back(-1);}
    if(predOrder == 3){coefs.push_back(3); coefs.push_back(-3); coefs.push_back(1);}
    if(predOrder == 4){coefs.push_back(4); coefs.push_back(-6); coefs.push_back(4); coefs.push_back(-1);}
    if(predOrder > 4) return ERR_FLAC_PREORDER_TOO_BIG; // Error: preorder > 4"
    restoreLinearPrediction(ch, 0);
    return ERR_FLAC_NONE;
}
//----------------------------------------------------------------------------------------------------------------------
int8_t decodeLinearPredictiveCodingSubframe(int lpcOrder, int sampleDepth, uint8_t ch){
    int8_t ret = 0;
    for (int i = 0; i < lpcOrder; i++)
        FLACsubFramesBuff->samplesBuffer[ch][i] = readSignedInt(sampleDepth);
    int precision = readUint(4) + 1;
    int shift = readSignedInt(5);
    coefs.resize(0);
    for (uint8_t i = 0; i < lpcOrder; i++)
        coefs.push_back(readSignedInt(precision));
    ret = decodeResiduals(lpcOrder, ch);
    if(ret) return ret;
    restoreLinearPrediction(ch, shift);
    return ERR_FLAC_NONE;
}
//----------------------------------------------------------------------------------------------------------------------
int8_t decodeResiduals(uint8_t warmup, uint8_t ch) {

    int method = readUint(2);
    if (method >= 2)
        return ERR_FLAC_RESERVED_RESIDUAL_CODING; // Reserved residual coding method
    uint8_t paramBits = method == 0 ? 4 : 5;
    int escapeParam = (method == 0 ? 0xF : 0x1F);
    int partitionOrder = readUint(4);

    int numPartitions = 1 << partitionOrder;
    if (m_blockSize % numPartitions != 0)
        return ERR_FLAC_WRONG_RICE_PARTITION_NR; //Error: Block size not divisible by number of Rice partitions
    int partitionSize = m_blockSize/ numPartitions;

    for (int i = 0; i < numPartitions; i++) {
        int start = i * partitionSize + (i == 0 ? warmup : 0);
        int end = (i + 1) * partitionSize;

        int param = readUint(paramBits);
        if (param < escapeParam) {
            for (int j = start; j < end; j++){
                FLACsubFramesBuff->samplesBuffer[ch][j] = readRiceSignedInt(param);
            }
        } else {
            int numBits = readUint(5);
            for (int j = start; j < end; j++){
                FLACsubFramesBuff->samplesBuffer[ch][j] = readSignedInt(numBits);
            }
        }
    }
    return ERR_FLAC_NONE;
}
//----------------------------------------------------------------------------------------------------------------------
void restoreLinearPrediction(uint8_t ch, uint8_t shift) {

    for (int i = coefs.size(); i < m_blockSize; i++) {
        int32_t sum = 0;
        for (int j = 0; j < coefs.size(); j++){
            sum += FLACsubFramesBuff->samplesBuffer[ch][i - 1 - j] * coefs[j];
        }
        FLACsubFramesBuff->samplesBuffer[ch][i] += (sum >> shift);
    }
}
//----------------------------------------------------------------------------------------------------------------------

