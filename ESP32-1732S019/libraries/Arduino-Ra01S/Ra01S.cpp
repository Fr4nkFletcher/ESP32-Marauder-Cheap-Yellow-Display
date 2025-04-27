#include "Arduino.h"
#include <SPI.h>
#include "Ra01S.h"


SX126x::SX126x(int spiSelect, int reset, int busy, int txen, int rxen)
{
  SX126x_SPI_SELECT = spiSelect;
  SX126x_RESET      = reset;
  SX126x_BUSY       = busy;
  SX126x_TXEN       = txen;
  SX126x_RXEN       = rxen;
  
  txActive          = false;
  debugPrint        = false;
  
  pinMode(SX126x_SPI_SELECT, OUTPUT);
  pinMode(SX126x_RESET, OUTPUT);
  pinMode(SX126x_BUSY, INPUT);
  if (SX126x_TXEN != -1) pinMode(SX126x_TXEN, OUTPUT);
  if (SX126x_RXEN != -1) pinMode(SX126x_RXEN, OUTPUT);

  //SPI.begin();
}


int16_t SX126x::begin(uint32_t frequencyInHz, int8_t txPowerInDbm, float tcxoVoltage, bool useRegulatorLDO) 
{
  Serial.println("begin");
  Serial.print("debugPrint=");
  Serial.println(debugPrint);
  Serial.print("SX126x_SPI_SELECT=");
  Serial.println(SX126x_SPI_SELECT);
  Serial.print("SX126x_RESET=");
  Serial.println(SX126x_RESET);
  Serial.print("SX126x_BUSY=");
  Serial.println(SX126x_BUSY);
  Serial.print("SX126x_TXEN=");
  Serial.println(SX126x_TXEN);
  Serial.print("SX126x_RXEN=");
  Serial.println(SX126x_RXEN);
  
  if ( txPowerInDbm > 22 )
    txPowerInDbm = 22;
  if ( txPowerInDbm < -3 )
    txPowerInDbm = -3;
  
  Reset();
  Serial.println("Reset");
  
  uint8_t wk[2];
  ReadRegister(SX126X_REG_LORA_SYNC_WORD_MSB, wk, 2); // 0x0740
  uint16_t syncWord = (wk[0] << 8) + wk[1];
  Serial.print("syncWord=0x");
  Serial.println(syncWord, HEX);
  if (syncWord != SX126X_SYNC_WORD_PUBLIC && syncWord != SX126X_SYNC_WORD_PRIVATE) {
    Serial.println("SX126x error, maybe no SPI connection");
    return ERR_INVALID_MODE;
  }

  Serial.println("SX126x installed");
  SetStandby(SX126X_STANDBY_RC);

  SetDio2AsRfSwitchCtrl(true);
  Serial.print("tcxoVoltage=");
  Serial.println(tcxoVoltage);
  // set TCXO control, if requested
  if(tcxoVoltage > 0.0) {
    SetDio3AsTcxoCtrl(tcxoVoltage, RADIO_TCXO_SETUP_TIME); // Configure the radio to use a TCXO controlled by DIO3
  }

  Calibrate(  SX126X_CALIBRATE_IMAGE_ON
                  | SX126X_CALIBRATE_ADC_BULK_P_ON
                  | SX126X_CALIBRATE_ADC_BULK_N_ON
                  | SX126X_CALIBRATE_ADC_PULSE_ON
                  | SX126X_CALIBRATE_PLL_ON
                  | SX126X_CALIBRATE_RC13M_ON
                  | SX126X_CALIBRATE_RC64K_ON
                  );

  Serial.print("useRegulatorLDO=");
  Serial.println(useRegulatorLDO);
  if (useRegulatorLDO) {
    SetRegulatorMode(SX126X_REGULATOR_LDO); // set regulator mode: LDO
  } else {
    SetRegulatorMode(SX126X_REGULATOR_DC_DC); // set regulator mode: DC-DC
  }

  SetBufferBaseAddress(0, 0);
#if 0
  // SX1261_TRANCEIVER
  SetPaConfig(0x06, 0x00, 0x01, 0x01); // PA Optimal Settings +15 dBm
  // SX1262_TRANCEIVER
  SetPaConfig(0x04, 0x07, 0x00, 0x01); // PA Optimal Settings +22 dBm
  // SX1268_TRANCEIVER
  SetPaConfig(0x04, 0x07, 0x00, 0x01); // PA Optimal Settings +22 dBm
#endif
  SetPaConfig(0x04, 0x07, 0x00, 0x01); // PA Optimal Settings +22 dBm
  SetOvercurrentProtection(60.0);  // current max 60mA for the whole device
  SetPowerConfig(txPowerInDbm, SX126X_PA_RAMP_200U); //0 fuer Empfaenger
  SetRfFrequency(frequencyInHz);
  return ERR_NONE;
}

void SX126x::FixInvertedIQ(uint8_t iqConfig)
{
  // fixes IQ configuration for inverted IQ
  // see SX1262/SX1268 datasheet, chapter 15 Known Limitations, section 15.4 for details
  // When exchanging LoRa packets with inverted IQ polarity, some packet losses may be observed for longer packets.
  // Workaround: Bit 2 at address 0x0736 must be set to:
  // ¡È0¡É when using inverted IQ polarity (see the SetPacketParam(...) command)
  // ¡È1¡É when using standard IQ polarity

  // read current IQ configuration
  uint8_t iqConfigCurrent = 0;
  ReadRegister(SX126X_REG_IQ_POLARITY_SETUP, &iqConfigCurrent, 1); // 0x0736

  // set correct IQ configuration
  if(iqConfig == SX126X_LORA_IQ_STANDARD) {
    iqConfigCurrent &= 0xFB;
  } else {
    iqConfigCurrent |= 0x04;
  }

  // update with the new value
  WriteRegister(SX126X_REG_IQ_POLARITY_SETUP, &iqConfigCurrent, 1); // 0x0736
}


void SX126x::LoRaConfig(uint8_t spreadingFactor, uint8_t bandwidth, uint8_t codingRate, uint16_t preambleLength, uint8_t payloadLen, bool crcOn, bool invertIrq) 
{

  SetStopRxTimerOnPreambleDetect(false);
  SetLoRaSymbNumTimeout(0); 
  SetPacketType(SX126X_PACKET_TYPE_LORA); // SX126x.ModulationParams.PacketType : MODEM_LORA
  uint8_t ldro = 0; // LowDataRateOptimize OFF
  SetModulationParams(spreadingFactor, bandwidth, codingRate, ldro);
  
  PacketParams[0] = (preambleLength >> 8) & 0xFF;
  PacketParams[1] = preambleLength;
  if ( payloadLen )
  {
    PacketParams[2] = 0x01; // Fixed length packet (implicit header)
    PacketParams[3] = payloadLen;
  }
  else
  {
    PacketParams[2] = 0x00; // Variable length packet (explicit header)
    PacketParams[3] = 0xFF;
  }

  if ( crcOn )
    PacketParams[4] = SX126X_LORA_IQ_INVERTED;
  else
    PacketParams[4] = SX126X_LORA_IQ_STANDARD;

  if ( invertIrq )
    PacketParams[5] = 0x01; // Inverted LoRa I and Q signals setup
  else
    PacketParams[5] = 0x00; // Standard LoRa I and Q signals setup

  // fixes IQ configuration for inverted IQ
  FixInvertedIQ(PacketParams[5]);

  WriteCommand(SX126X_CMD_SET_PACKET_PARAMS, PacketParams, 6); // 0x8C

#if 0
  SetDioIrqParams(SX126X_IRQ_ALL,  //all interrupts enabled
                  (SX126X_IRQ_RX_DONE | SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT), //interrupts on DIO1
                  SX126X_IRQ_NONE,  //interrupts on DIO2
                  SX126X_IRQ_NONE); //interrupts on DIO3
#endif
  // Do not use DIO interruptst
  SetDioIrqParams(SX126X_IRQ_ALL,   //all interrupts enabled
                  SX126X_IRQ_NONE,  //interrupts on DIO1
                  SX126X_IRQ_NONE,  //interrupts on DIO2
                  SX126X_IRQ_NONE); //interrupts on DIO3

  // Receive state no receive timeoout
  SetRx(0xFFFFFF);
}


void SX126x::DebugPrint(bool enable) 
{
  debugPrint = enable;
}


uint8_t SX126x::Receive(uint8_t *pData, uint16_t len) 
{
  uint8_t rxLen = 0;
  uint16_t irqRegs = GetIrqStatus();
  //uint8_t status = GetStatus();

  if( irqRegs & SX126X_IRQ_RX_DONE )
  {
    //ClearIrqStatus(SX126X_IRQ_RX_DONE);
    ClearIrqStatus(SX126X_IRQ_ALL);
    rxLen = ReadBuffer(pData, len);
  }
  
  return rxLen;
}


bool SX126x::Send(uint8_t *pData, uint8_t len, uint8_t mode)
{
  uint16_t irqStatus;
  bool rv = false;
  
  if ( txActive == false )
  {
    txActive = true;
    PacketParams[2] = 0x00; //Variable length packet (explicit header)
    PacketParams[3] = len;
    WriteCommand(SX126X_CMD_SET_PACKET_PARAMS, PacketParams, 6); // 0x8C

    //ClearIrqStatus(SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT);
    ClearIrqStatus(SX126X_IRQ_ALL);

    WriteBuffer(pData, len);
    SetTx(500);

    if ( mode & SX126x_TXMODE_SYNC )
    {
      irqStatus = GetIrqStatus();
      while ( (!(irqStatus & SX126X_IRQ_TX_DONE)) && (!(irqStatus & SX126X_IRQ_TIMEOUT)) )
      {
        delay(1);
        irqStatus = GetIrqStatus();
      }
      if (debugPrint) {
        Serial.print("irqStatus=");
        Serial.println(irqStatus, HEX);
        if (irqStatus & SX126X_IRQ_TX_DONE) {
          Serial.println("SX126X_IRQ_TX_DONE");
        }
        if (irqStatus & SX126X_IRQ_TIMEOUT) {
          Serial.println("SX126X_IRQ_TIMEOUT");
        }
      }
      txActive = false;

      SetRx(0xFFFFFF);

      if ( irqStatus & SX126X_IRQ_TX_DONE) {
        rv = true;
      }
    }
    else
    {
      rv = true;
    }
  }
  if (debugPrint) {
    Serial.print("Send rv=");
    Serial.println(rv, HEX);
  }
  return rv;
}


bool SX126x::ReceiveMode(void)
{
  uint16_t irq;
  bool rv = false;

  if ( txActive == false )
  {
    rv = true;
  }
  else
  {
    irq = GetIrqStatus();
    if ( irq & (SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT) )
    { 
      SetRx(0xFFFFFF);
      txActive = false;
      rv = true;
    }
  }

  return rv;
}


void SX126x::GetPacketStatus(int8_t *rssiPacket, int8_t *snrPacket)
{
  uint8_t buf[4];
  ReadCommand( SX126X_CMD_GET_PACKET_STATUS, buf, 4 ); // 0x14
  *rssiPacket = (buf[3] >> 1) * -1;
  ( buf[2] < 128 ) ? ( *snrPacket = buf[2] >> 2 ) : ( *snrPacket = ( ( buf[2] - 256 ) >> 2 ) );
}


void SX126x::SetTxPower(int8_t txPowerInDbm)
{
  SetPowerConfig(txPowerInDbm, SX126X_PA_RAMP_200U);
}


void SX126x::Reset(void)
{
  delay(10);
  digitalWrite(SX126x_RESET,0);
  delay(20);
  digitalWrite(SX126x_RESET,1);
  delay(10);
  // ensure BUSY is low (state meachine ready)
  WaitForIdle(BUSY_WAIT, "Reset", true);
}


void SX126x::Wakeup(void)
{
  GetStatus();
}


void SX126x::SetSleep(uint8_t mode)
{
  uint8_t data = mode;
  WriteCommand(SX126X_CMD_SET_SLEEP, &data, 1); // 0x84
}


void SX126x::SetStandby(uint8_t mode)
{
  uint8_t data = mode;
  WriteCommand(SX126X_CMD_SET_STANDBY, &data, 1); // 0x80
}


uint8_t SX126x::GetStatus(void)
{
  uint8_t rv;
  ReadCommand(SX126X_CMD_GET_STATUS, &rv, 1); // 0xC0
  return rv;
}


uint32_t SX126x::GetRandomNumber(void)
{
  uint8_t random[4];
  ReadRegister(SX126X_REG_RANDOM_NUMBER_0, random, 4);
  return *((uint32_t *)random);
}


void SX126x::SetDio3AsTcxoCtrl(float voltage, uint32_t delay)
{
  uint8_t buf[4];

  //buf[0] = tcxoVoltage & 0x07;
  if(fabs(voltage - 1.6) <= 0.001) {
    buf[0] = SX126X_DIO3_OUTPUT_1_6;
  } else if(fabs(voltage - 1.7) <= 0.001) {
    buf[0] = SX126X_DIO3_OUTPUT_1_7;
  } else if(fabs(voltage - 1.8) <= 0.001) {
    buf[0] = SX126X_DIO3_OUTPUT_1_8;
  } else if(fabs(voltage - 2.2) <= 0.001) {
    buf[0] = SX126X_DIO3_OUTPUT_2_2;
  } else if(fabs(voltage - 2.4) <= 0.001) {
    buf[0] = SX126X_DIO3_OUTPUT_2_4;
  } else if(fabs(voltage - 2.7) <= 0.001) {
    buf[0] = SX126X_DIO3_OUTPUT_2_7;
  } else if(fabs(voltage - 3.0) <= 0.001) {
    buf[0] = SX126X_DIO3_OUTPUT_3_0;
  } else {
    buf[0] = SX126X_DIO3_OUTPUT_3_3;
  }

  uint32_t delayValue = (float)delay / 15.625;
  buf[1] = ( uint8_t )( ( delayValue >> 16 ) & 0xFF );
  buf[2] = ( uint8_t )( ( delayValue >> 8 ) & 0xFF );
  buf[3] = ( uint8_t )( delayValue & 0xFF );

  WriteCommand(SX126X_CMD_SET_DIO3_AS_TCXO_CTRL, buf, 4); // 0x97
}


void SX126x::Calibrate(uint8_t calibParam)
{
  uint8_t data = calibParam;
  WriteCommand(SX126X_CMD_CALIBRATE, &data, 1); // 0x89
}


void SX126x::SetDio2AsRfSwitchCtrl(uint8_t enable)
{
  uint8_t data = enable;
  WriteCommand(SX126X_CMD_SET_DIO2_AS_RF_SWITCH_CTRL, &data, 1); // 0x9D
}


void SX126x::SetRfFrequency(uint32_t frequency)
{
  uint8_t buf[4];
  uint32_t freq = 0;

  CalibrateImage(frequency);

  freq = (uint32_t)((double)frequency / (double)FREQ_STEP);
  buf[0] = (uint8_t)((freq >> 24) & 0xFF);
  buf[1] = (uint8_t)((freq >> 16) & 0xFF);
  buf[2] = (uint8_t)((freq >> 8) & 0xFF);
  buf[3] = (uint8_t)(freq & 0xFF);
  WriteCommand(SX126X_CMD_SET_RF_FREQUENCY, buf, 4); // 0x86
}


void SX126x::CalibrateImage(uint32_t frequency)
{
  uint8_t calFreq[2];

  if( frequency> 900000000 )
  {
    calFreq[0] = 0xE1;
    calFreq[1] = 0xE9;
  }
  else if( frequency > 850000000 )
  {
    calFreq[0] = 0xD7;
    calFreq[1] = 0xDB;
  }
  else if( frequency > 770000000 )
  {
    calFreq[0] = 0xC1;
    calFreq[1] = 0xC5;
  }
  else if( frequency > 460000000 )
  {
    calFreq[0] = 0x75;
    calFreq[1] = 0x81;
  }
  else if( frequency > 425000000 )
  {
    calFreq[0] = 0x6B;
    calFreq[1] = 0x6F;
  }
  WriteCommand(SX126X_CMD_CALIBRATE_IMAGE, calFreq, 2); // 0x98
}


void SX126x::SetRegulatorMode(uint8_t mode)
{
  uint8_t data = mode;
  WriteCommand(SX126X_CMD_SET_REGULATOR_MODE, &data, 1); // 0x96
}


void SX126x::SetBufferBaseAddress(uint8_t txBaseAddress, uint8_t rxBaseAddress)
{
  uint8_t buf[2];

  buf[0] = txBaseAddress;
  buf[1] = rxBaseAddress;
  WriteCommand(SX126X_CMD_SET_BUFFER_BASE_ADDRESS, buf, 2); // 0x8F
}


void SX126x::SetPowerConfig(int8_t power, uint8_t rampTime)
{
  uint8_t buf[2];

  if( power > 22 )
  {
    power = 22;
  }
  else if( power < -3 )
  {
    power = -3;
  }
    
  buf[0] = power;
  buf[1] = ( uint8_t )rampTime;
  WriteCommand(SX126X_CMD_SET_TX_PARAMS, buf, 2); // 0x8E
}


void SX126x::SetPaConfig(uint8_t paDutyCycle, uint8_t hpMax, uint8_t deviceSel, uint8_t paLut)
{
  uint8_t buf[4];

  buf[0] = paDutyCycle;
  buf[1] = hpMax;
  buf[2] = deviceSel;
  buf[3] = paLut;
  WriteCommand(SX126X_CMD_SET_PA_CONFIG, buf, 4); // 0x95
}


void SX126x::SetOvercurrentProtection(float currentLimit)
{
  if((currentLimit >= 0.0) && (currentLimit <= 140.0)) {
    uint8_t buf[1];
    buf[0] = (uint8_t)(currentLimit / 2.5);
    WriteRegister(SX126X_REG_OCP_CONFIGURATION, buf, 1); // 0x08E7
  }
}


void SX126x::SetDioIrqParams
( uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask )
{
  uint8_t buf[8];

  buf[0] = (uint8_t)((irqMask >> 8) & 0x00FF);
  buf[1] = (uint8_t)(irqMask & 0x00FF);
  buf[2] = (uint8_t)((dio1Mask >> 8) & 0x00FF);
  buf[3] = (uint8_t)(dio1Mask & 0x00FF);
  buf[4] = (uint8_t)((dio2Mask >> 8) & 0x00FF);
  buf[5] = (uint8_t)(dio2Mask & 0x00FF);
  buf[6] = (uint8_t)((dio3Mask >> 8) & 0x00FF);
  buf[7] = (uint8_t)(dio3Mask & 0x00FF);
  WriteCommand(SX126X_CMD_SET_DIO_IRQ_PARAMS, buf, 8); // 0x08
}


void SX126x::SetStopRxTimerOnPreambleDetect( bool enable )
{
  uint8_t data = (uint8_t)enable;
  WriteCommand(SX126X_CMD_STOP_TIMER_ON_PREAMBLE, &data, 1); // 0x9F
}


void SX126x::SetLoRaSymbNumTimeout(uint8_t SymbNum)
{
  uint8_t data = SymbNum;
  WriteCommand(SX126X_CMD_SET_LORA_SYMB_NUM_TIMEOUT, &data, 1); // 0xA0
}


void SX126x::SetPacketType(uint8_t packetType)
{
  uint8_t data = packetType;
  WriteCommand(SX126X_CMD_SET_PACKET_TYPE, &data, 1); // 0x01
}


void SX126x::SetModulationParams(uint8_t spreadingFactor, uint8_t bandwidth, uint8_t codingRate, uint8_t lowDataRateOptimize)
{
  uint8_t data[4];
  //currently only LoRa supported
  data[0] = spreadingFactor;
  data[1] = bandwidth;
  data[2] = codingRate;
  data[3] = lowDataRateOptimize;
  WriteCommand(SX126X_CMD_SET_MODULATION_PARAMS, data, 4); // 0x8B
}


uint16_t SX126x::GetIrqStatus( void )
{
  uint8_t data[3];
  ReadCommand(SX126X_CMD_GET_IRQ_STATUS, data, 3); // 0x12
  return (data[1] << 8) | data[2];
}


void SX126x::ClearIrqStatus(uint16_t irq)
{
  uint8_t buf[2];

  buf[0] = (uint8_t)(((uint16_t)irq >> 8) & 0x00FF);
  buf[1] = (uint8_t)((uint16_t)irq & 0x00FF);
  WriteCommand(SX126X_CMD_CLEAR_IRQ_STATUS, buf, 2); // 0x02
}


void SX126x::SetRx(uint32_t timeout)
{
  if (debugPrint) {
    Serial.print("----- SetRx timeout=0x");
    Serial.println(timeout, HEX);
  }
  SetStandby(SX126X_STANDBY_RC);
  SetRxEnable();
  uint8_t buf[3];
  buf[0] = (uint8_t)((timeout >> 16) & 0xFF);
  buf[1] = (uint8_t)((timeout >> 8) & 0xFF);
  buf[2] = (uint8_t )(timeout & 0xFF);
  WriteCommand(SX126X_CMD_SET_RX, buf, 3); // 0x82

  for(int retry=0;retry<10;retry++) {
    if ((GetStatus() & 0x70) == 0x50) break;
    delay(1);
  }
  if ((GetStatus() & 0x70) != 0x50) {
    Serial.println("SetRx Illegal Status");
    while(1) {delay(1);}
  }
}


void SX126x::SetRxEnable(void)
{
  if (debugPrint) {
    Serial.print("SetRxEnable:SX126x_TXEN=");
    Serial.print(SX126x_TXEN);
    Serial.print(" SX126x_RXEN=");
    Serial.print(SX126x_RXEN);
    Serial.println();
  }
  if ((SX126x_TXEN != -1) && (SX126x_RXEN != -1)) {
    digitalWrite(SX126x_RXEN, HIGH);
    digitalWrite(SX126x_TXEN, LOW);
  }
}


void SX126x::SetTx(uint32_t timeoutInMs)
{
  if (debugPrint) {
    Serial.print("----- SetTx timeoutInMs=");
    Serial.println(timeoutInMs);
  }
  SetStandby(SX126X_STANDBY_RC);
  SetTxEnable();
  uint8_t buf[3];
  uint32_t tout = timeoutInMs;
  if (timeoutInMs != 0) {
    // Timeout duration = Timeout * 15.625 ¦Ìs
    uint32_t timeoutInUs = timeoutInMs * 1000;
    tout = (uint32_t)(timeoutInUs / 15.625);
  }
  if (debugPrint) {
    Serial.print("SetTx timeoutInMs=");
    Serial.print(timeoutInMs);
    Serial.print(" tout=");
    Serial.println(tout);
  }
  buf[0] = (uint8_t)((tout >> 16) & 0xFF);
  buf[1] = (uint8_t)((tout >> 8) & 0xFF);
  buf[2] = (uint8_t )(tout & 0xFF);
  WriteCommand(SX126X_CMD_SET_TX, buf, 3); // 0x83

  for(int retry=0;retry<10;retry++) {
    if ((GetStatus() & 0x70) == 0x60) break;
    delay(1);
  }
  if ((GetStatus() & 0x70) != 0x60) {
    Serial.println("SetTx Illegal Status");
    while(1) {delay(1);}
  }
}


void SX126x::SetTxEnable(void)
{
  if (debugPrint) {
    Serial.print("SetTxEnable:SX126x_TXEN=");
    Serial.print(SX126x_TXEN);
    Serial.print(" SX126x_RXEN=");
    Serial.print(SX126x_RXEN);
    Serial.println();
  }
  if ((SX126x_TXEN != -1) && (SX126x_RXEN != -1)){
    digitalWrite(SX126x_RXEN, LOW);
    digitalWrite(SX126x_TXEN, HIGH);
  }
}


uint8_t SX126x::GetRssiInst()
{
  uint8_t buf[2];
  ReadCommand( SX126X_CMD_GET_RSSI_INST, buf, 2 ); // 0x15
  return buf[1];
}


void SX126x::GetRxBufferStatus(uint8_t *payloadLength, uint8_t *rxStartBufferPointer)
{
  uint8_t buf[3];
  ReadCommand( SX126X_CMD_GET_RX_BUFFER_STATUS, buf, 3 ); // 0x13
  *payloadLength = buf[1];
  *rxStartBufferPointer = buf[2];
}


void SX126x::WaitForIdle(unsigned long timeout, char *text, bool stop)
{
  unsigned long start = millis();
  delayMicroseconds(1);
  while(digitalRead(SX126x_BUSY)) {
    delayMicroseconds(1);
    if(millis() - start >= timeout) {
      Serial.print("WaitForIdle [");
      Serial.print(text);
      Serial.print("] Timeout timeout=");
      Serial.println(timeout);
      if (stop) {
        while(1) {delay(1);}
      }
    }
  }
}



uint8_t SX126x::ReadBuffer(uint8_t *rxData, uint8_t maxLen)
{
  uint8_t offset = 0;
  uint8_t payloadLength = 0;
  GetRxBufferStatus(&payloadLength, &offset);
  if( payloadLength > maxLen )
  {
    Serial.println("ReadBuffer maxLen too small");
    return 0;
  }

  // ensure BUSY is low (state meachine ready)
  WaitForIdle(BUSY_WAIT, "start ReadBuffer", true);

  digitalWrite(SX126x_SPI_SELECT, LOW);
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
  SPI.transfer(SX126X_CMD_READ_BUFFER); // 0x1E
  SPI.transfer(offset);
  SPI.transfer(SX126X_CMD_NOP);
  for( uint16_t i = 0; i < payloadLength; i++ )
  {
    rxData[i] = SPI.transfer(SX126X_CMD_NOP);  
  }
  SPI.endTransaction();
  digitalWrite(SX126x_SPI_SELECT, HIGH);

  // wait for BUSY to go low
  WaitForIdle(BUSY_WAIT, "end ReadBuffer", false);

  return payloadLength;
}

void SX126x::WriteBuffer(uint8_t *txData, uint8_t txDataLen)
{
  // ensure BUSY is low (state meachine ready)
  WaitForIdle(BUSY_WAIT, "start WriteBuffer", true);

  digitalWrite(SX126x_SPI_SELECT, LOW);
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
  SPI.transfer(SX126X_CMD_WRITE_BUFFER); // 0x0E
  SPI.transfer(0); //offset in tx fifo
  for( uint16_t i = 0; i < txDataLen; i++ )
  { 
     SPI.transfer( txData[i]);  
  }
  SPI.endTransaction();
  digitalWrite(SX126x_SPI_SELECT, HIGH);

  // wait for BUSY to go low
  WaitForIdle(BUSY_WAIT, "end WriteBuffer", false);
}


void SX126x::WriteRegister(uint16_t reg, uint8_t* data, uint8_t numBytes, bool waitForBusy) {
  // ensure BUSY is low (state meachine ready)
  WaitForIdle(BUSY_WAIT, "start WriteRegister", true);

  // start transfer
  if(debugPrint) {
    Serial.print("WriteRegister: REG=0x");
    Serial.print(reg, HEX);
  }
  digitalWrite(SX126x_SPI_SELECT, LOW);
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));

  // send command byte
  SPI.transfer(SX126X_CMD_WRITE_REGISTER); // 0x0D
  SPI.transfer((reg & 0xFF00) >> 8);
  SPI.transfer(reg & 0xff);
  
  if(debugPrint) {
    Serial.print(" ");
    Serial.print(" DataOut: ");
  }
  for(uint8_t n = 0; n < numBytes; n++) {
    uint8_t in = SPI.transfer(data[n]);
    if(debugPrint) {
      Serial.print(data[n], HEX);
      Serial.print(" ");
    }
  }
  if(debugPrint)Serial.println();

  // stop transfer
  SPI.endTransaction();
  digitalWrite(SX126x_SPI_SELECT, HIGH);

  // wait for BUSY to go low
  if(waitForBusy) {
    WaitForIdle(BUSY_WAIT, "end WriteRegister", false);
  }
}


void SX126x::ReadRegister(uint16_t reg, uint8_t* data, uint8_t numBytes, bool waitForBusy) {
  // ensure BUSY is low (state meachine ready)
  WaitForIdle(BUSY_WAIT, "start ReadRegister", true);

  // start transfer
  if(debugPrint) {
    Serial.print("ReadRegister:  REG=0x");
    Serial.print(reg, HEX);
  }
  digitalWrite(SX126x_SPI_SELECT, LOW);
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));

  // send command byte
  SPI.transfer(SX126X_CMD_READ_REGISTER); // 0x1D
  SPI.transfer((reg & 0xFF00) >> 8);
  SPI.transfer(reg & 0xff);
  SPI.transfer(SX126X_CMD_NOP);

  if(debugPrint) {
    Serial.print(" ");
    Serial.print(" DataIn: ");
  }
  for(uint8_t n = 0; n < numBytes; n++) {
    data[n] = SPI.transfer(SX126X_CMD_NOP);
    if(debugPrint) {
      Serial.print(data[n], HEX);
      Serial.print(" ");
    }
  }
  if(debugPrint) Serial.println();

  // stop transfer
  SPI.endTransaction();
  digitalWrite(SX126x_SPI_SELECT, HIGH);

  // wait for BUSY to go low
  if(waitForBusy) {
    WaitForIdle(BUSY_WAIT, "end ReadRegister", false);
  }
}

// WriteCommand with retry for EBYTE
void SX126x::WriteCommand(uint8_t cmd, uint8_t* data, uint8_t numBytes, bool waitForBusy) {
  uint8_t status;
  for (int retry=1; retry<10; retry++) {
    status = WriteCommand2(cmd, data, numBytes,  waitForBusy);
    if (status == 0) break;
  }
  if (status != 0) {
    Serial.print("SPI Transaction error:");
    Serial.println(status);
    while(1) {delay(1);}
  }
}

uint8_t SX126x::WriteCommand2(uint8_t cmd, uint8_t* data, uint8_t numBytes, bool waitForBusy) {
  // ensure BUSY is low (state meachine ready)
  WaitForIdle(BUSY_WAIT, "start WriteCommand2", true);

  // start transfer
  digitalWrite(SX126x_SPI_SELECT, LOW);
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));

  // send command byte
  if(debugPrint) {
    Serial.print("WriteCommand:  CMD=0x");
    Serial.print(cmd, HEX);
    Serial.print(" ");
    Serial.print(" DataOut: ");
  }
  SPI.transfer(cmd);

  // variable to save error during SPI transfer
  uint8_t status = 0;

  // send/receive all bytes
  for(uint8_t n = 0; n < numBytes; n++) {
    uint8_t in = SPI.transfer(data[n]);
    if(debugPrint) {
      Serial.print(data[n], HEX);
      Serial.print("-->");
      Serial.print(in, HEX);
      Serial.print(" ");
    }

    // check status
    if(((in & 0b00001110) == SX126X_STATUS_CMD_TIMEOUT) ||
     ((in & 0b00001110) == SX126X_STATUS_CMD_INVALID) ||
     ((in & 0b00001110) == SX126X_STATUS_CMD_FAILED)) {
    status = in & 0b00001110;
    break;
    } else if(in == 0x00 || in == 0xFF) {
    status = SX126X_STATUS_SPI_FAILED;
    break;
    }
  }
  if(debugPrint) Serial.println();

  // stop transfer
  SPI.endTransaction();
  digitalWrite(SX126x_SPI_SELECT, HIGH);

  // wait for BUSY to go low
  if(waitForBusy) {
    WaitForIdle(BUSY_WAIT, "end WriteCommand2", false);
  }

  if (status != 0) {
    Serial.print("SPI Transaction error:");
    Serial.println(status);
    //while(1) {delay(1);}
  }
  return status;
}


void SX126x::ReadCommand(uint8_t cmd, uint8_t* data, uint8_t numBytes, bool waitForBusy) {
  // ensure BUSY is low (state meachine ready)
  WaitForIdle(BUSY_WAIT, "start ReadCommand", true);

  // start transfer
  digitalWrite(SX126x_SPI_SELECT, LOW);
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));

  // send command byte
  if(debugPrint) {
    Serial.print("ReadCommand:   CMD=0x");
    Serial.print(cmd, HEX);
  }
  SPI.transfer(cmd);

  if(debugPrint) {
    Serial.print(" ");
    Serial.print(" DataIn: ");
  }

  // send/receive all bytes
  for(uint8_t n = 0; n < numBytes; n++) {
    data[n] = SPI.transfer(SX126X_CMD_NOP);
    if(debugPrint) {
      Serial.print(data[n], HEX);
      Serial.print(" ");
    }
  }
  if(debugPrint) Serial.println();

  // stop transfer
  SPI.endTransaction();
  digitalWrite(SX126x_SPI_SELECT, HIGH);

  // wait for BUSY to go low
  if(waitForBusy) {
    WaitForIdle(BUSY_WAIT, "end ReadCommand", false);
  }
}
