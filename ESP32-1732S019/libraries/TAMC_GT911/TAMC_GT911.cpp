#include "Arduino.h"
#include <TAMC_GT911.h>
#include <Wire.h>

TAMC_GT911::TAMC_GT911(uint8_t _sda, uint8_t _scl, uint8_t _int, uint8_t _rst, uint16_t _width, uint16_t _height) :
  pinSda(_sda), pinScl(_scl), pinInt(_int), pinRst(_rst), width(_width), height(_height) {

}

void TAMC_GT911::begin(uint8_t _addr) {
  addr = _addr;
  Wire.begin(pinSda, pinScl);
  reset();
}
void TAMC_GT911::reset() {
  pinMode(pinInt, OUTPUT);
  pinMode(pinRst, OUTPUT);
  digitalWrite(pinInt, 0);
  digitalWrite(pinRst, 0);
  delay(10);
  digitalWrite(pinInt, addr==GT911_ADDR2);
  delay(1);
  digitalWrite(pinRst, 1);
  delay(5);
  digitalWrite(pinInt, 0);
  delay(50);
  pinMode(pinInt, INPUT);
  // attachInterrupt(pinInt, TAMC_GT911::onInterrupt, RISING);
  delay(50);
  readBlockData(configBuf, GT911_CONFIG_START, GT911_CONFIG_SIZE);
  setResolution(width, height);
}
void TAMC_GT911::calculateChecksum() {
  uint8_t checksum;
  for (uint8_t i=0; i<GT911_CONFIG_SIZE; i++) {
    checksum += configBuf[i];
  }
  checksum = (~checksum) + 1;
  configBuf[GT911_CONFIG_CHKSUM - GT911_CONFIG_START] = checksum;
}
// void ARDUINO_ISR_ATTR TAMC_GT911::onInterrupt() {
//   read();
//   TAMC_GT911::onRead();
// }
void TAMC_GT911::reflashConfig() {
  calculateChecksum();
  writeByteData(GT911_CONFIG_CHKSUM, configBuf[GT911_CONFIG_CHKSUM-GT911_CONFIG_START]);
  writeByteData(GT911_CONFIG_FRESH, 1);
}
void TAMC_GT911::setRotation(uint8_t rot) {
  rotation = rot;
}
void TAMC_GT911::setResolution(uint16_t _width, uint16_t _height) {
  configBuf[GT911_X_OUTPUT_MAX_LOW - GT911_CONFIG_START] = lowByte(_width);
  configBuf[GT911_X_OUTPUT_MAX_HIGH - GT911_CONFIG_START] = highByte(_width);
  configBuf[GT911_Y_OUTPUT_MAX_LOW - GT911_CONFIG_START] = lowByte(_height);
  configBuf[GT911_Y_OUTPUT_MAX_HIGH - GT911_CONFIG_START] = highByte(_height);
  reflashConfig();
}
// void TAMC_GT911::setOnRead(void (*isr)()) {
//   onRead = isr;
// }
void TAMC_GT911::read(void) {
  // Serial.println("TAMC_GT911::read");
  uint8_t data[7];
  uint8_t id;
  uint16_t x, y, size;

  uint8_t pointInfo = readByteData(GT911_POINT_INFO);
  uint8_t bufferStatus = pointInfo >> 7 & 1;
  uint8_t proximityValid = pointInfo >> 5 & 1;
  uint8_t haveKey = pointInfo >> 4 & 1;
  isLargeDetect = pointInfo >> 6 & 1;
  touches = pointInfo & 0xF;
  #ifdef DEBUG
  Serial.print("bufferStatus: ");Serial.println(bufferStatus);
  Serial.print("largeDetect: ");Serial.println(isLargeDetect);
  Serial.print("proximityValid: ");Serial.println(proximityValid);
  Serial.print("haveKey: ");Serial.println(haveKey);
  Serial.print("touches: ");Serial.println(touches);
  #endif
  isTouched = touches > 0;
  if (bufferStatus == 1 && isTouched) {
    for (uint8_t i=0; i<touches; i++) {
      readBlockData(data, GT911_POINT_1 + i * 8, 7);
      points[i] = readPoint(data);
    }
  }
  writeByteData(GT911_POINT_INFO, 0);
}
TP_Point TAMC_GT911::readPoint(uint8_t *data) {
  uint16_t temp;
  uint8_t id = data[0];
  uint16_t x = data[1] + (data[2] << 8);
  uint16_t y = data[3] + (data[4] << 8);
  uint16_t size = data[5] + (data[6] << 8);
  switch (rotation){
    case ROTATION_NORMAL:
      x = width - x;
      y = height - y;
      break;
    case ROTATION_LEFT:
      temp = x;
      x = width - y;
      y = temp;
      break;
    case ROTATION_INVERTED:
      x = x;
      y = y;
      break;
    case ROTATION_RIGHT:
      temp = x;
      x = y;
      y = height - temp;
      break;
    default:
      break;
  }
  return TP_Point(id, x, y, size);
}
void TAMC_GT911::writeByteData(uint16_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(highByte(reg));
  Wire.write(lowByte(reg));
  Wire.write(val);
  Wire.endTransmission();
}
uint8_t TAMC_GT911::readByteData(uint16_t reg) {
  uint8_t x;
  Wire.beginTransmission(addr);
  Wire.write(highByte(reg));
  Wire.write(lowByte(reg));
  Wire.endTransmission();
  Wire.requestFrom(addr, (uint8_t)1);
  x = Wire.read();
  return x;
}
void TAMC_GT911::writeBlockData(uint16_t reg, uint8_t *val, uint8_t size) {
  Wire.beginTransmission(addr);
  Wire.write(highByte(reg));
  Wire.write(lowByte(reg));
  // Wire.write(val, size);
  for (uint8_t i=0; i<size; i++) {
    Wire.write(val[i]);
  }
  Wire.endTransmission();
}
void TAMC_GT911::readBlockData(uint8_t *buf, uint16_t reg, uint8_t size) {
  Wire.beginTransmission(addr);
  Wire.write(highByte(reg));
  Wire.write(lowByte(reg));
  Wire.endTransmission();
  Wire.requestFrom(addr, size);
  for (uint8_t i=0; i<size; i++) {
    buf[i] = Wire.read();
  }
}
TP_Point::TP_Point(void) {
  id = x = y = size = 0;
}
TP_Point::TP_Point(uint8_t _id, uint16_t _x, uint16_t _y, uint16_t _size) {
  id = _id;
  x = _x;
  y = _y;
  size = _size;
}
bool TP_Point::operator==(TP_Point point) {
  return ((point.x == x) && (point.y == y) && (point.size == size));
}
bool TP_Point::operator!=(TP_Point point) {
  return ((point.x != x) || (point.y != y) || (point.size != size));
}
