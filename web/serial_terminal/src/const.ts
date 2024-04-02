import { toByteArray } from "./util";

export interface Logger {
  log(msg: string, ...args: any[]): void;
  error(msg: string, ...args: any[]): void;
  debug(msg: string, ...args: any[]): void;
}

export const baudRates = [
  115200, 128000, 153600, 230400, 460800, 921600, 1500000, 2000000,
];
export const FLASH_SIZES = {
  "512KB": 0x00,
  "256KB": 0x10,
  "1MB": 0x20,
  "2MB": 0x30,
  "4MB": 0x40,
  "2MB-c1": 0x50,
  "4MB-c1": 0x60,
  "8MB": 0x80,
  "16MB": 0x90,
};

export const ESP32_FLASH_SIZES = {
  "1MB": 0x00,
  "2MB": 0x10,
  "4MB": 0x20,
  "8MB": 0x30,
  "16MB": 0x40,
  "32MB": 0x50,
  "64MB": 0x60,
  "128MB": 0x70,
};

interface FlashSize {
  [key: number]: string;
}

export const DETECTED_FLASH_SIZES: FlashSize = {
  0x12: "256KB",
  0x13: "512KB",
  0x14: "1MB",
  0x15: "2MB",
  0x16: "4MB",
  0x17: "8MB",
  0x18: "16MB",
  0x19: "32MB",
  0x1a: "64MB",
};

export const FLASH_WRITE_SIZE = 0x400;
export const STUB_FLASH_WRITE_SIZE = 0x4000;
export const FLASH_SECTOR_SIZE = 0x1000; // Flash sector size, minimum unit of erase.
export const ESP_ROM_BAUD = 115200;
export const USB_JTAG_SERIAL_PID = 0x1001;

export const ESP8266_SPI_REG_BASE = 0x60000200;
export const ESP8266_BASEFUSEADDR = 0x3ff00050;
export const ESP8266_MACFUSEADDR = 0x3ff00050;
export const ESP8266_SPI_USR_OFFS = 0x1c;
export const ESP8266_SPI_USR1_OFFS = 0x20;
export const ESP8266_SPI_USR2_OFFS = 0x24;
export const ESP8266_SPI_MOSI_DLEN_OFFS = -1;
export const ESP8266_SPI_MISO_DLEN_OFFS = -1;
export const ESP8266_SPI_W0_OFFS = 0x40;
export const ESP8266_UART_DATE_REG_ADDR = 0x60000078;
export const ESP8266_BOOTLOADER_FLASH_OFFSET = 0x0;

export const ESP32_SPI_REG_BASE = 0x3ff42000;
export const ESP32_BASEFUSEADDR = 0x3ff5a000;
export const ESP32_MACFUSEADDR = 0x3ff5a000;
export const ESP32_SPI_USR_OFFS = 0x1c;
export const ESP32_SPI_USR1_OFFS = 0x20;
export const ESP32_SPI_USR2_OFFS = 0x24;
export const ESP32_SPI_MOSI_DLEN_OFFS = 0x28;
export const ESP32_SPI_MISO_DLEN_OFFS = 0x2c;
export const ESP32_SPI_W0_OFFS = 0x80;
export const ESP32_UART_DATE_REG_ADDR = 0x60000078;
export const ESP32_BOOTLOADER_FLASH_OFFSET = 0x1000;

export const ESP32S2_SPI_REG_BASE = 0x3f402000;
export const ESP32S2_BASEFUSEADDR = 0x3f41a000;
export const ESP32S2_MACFUSEADDR = 0x3f41a044;
export const ESP32S2_SPI_USR_OFFS = 0x18;
export const ESP32S2_SPI_USR1_OFFS = 0x1c;
export const ESP32S2_SPI_USR2_OFFS = 0x20;
export const ESP32S2_SPI_MOSI_DLEN_OFFS = 0x24;
export const ESP32S2_SPI_MISO_DLEN_OFFS = 0x28;
export const ESP32S2_SPI_W0_OFFS = 0x58;
export const ESP32S2_UART_DATE_REG_ADDR = 0x60000078;
export const ESP32S2_BOOTLOADER_FLASH_OFFSET = 0x1000;

export const ESP32S3_SPI_REG_BASE = 0x60002000;
export const ESP32S3_BASEFUSEADDR = 0x60007000;
export const ESP32S3_MACFUSEADDR = 0x60007000 + 0x044;
export const ESP32S3_SPI_USR_OFFS = 0x18;
export const ESP32S3_SPI_USR1_OFFS = 0x1c;
export const ESP32S3_SPI_USR2_OFFS = 0x20;
export const ESP32S3_SPI_MOSI_DLEN_OFFS = 0x24;
export const ESP32S3_SPI_MISO_DLEN_OFFS = 0x28;
export const ESP32S3_SPI_W0_OFFS = 0x58;
export const ESP32S3_UART_DATE_REG_ADDR = 0x60000080;
export const ESP32S3_BOOTLOADER_FLASH_OFFSET = 0x0;

export const ESP32C2_SPI_REG_BASE = 0x60002000;
export const ESP32C2_BASEFUSEADDR = 0x60008800;
export const ESP32C2_MACFUSEADDR = 0x60008800 + 0x044;
export const ESP32C2_SPI_USR_OFFS = 0x18;
export const ESP32C2_SPI_USR1_OFFS = 0x1c;
export const ESP32C2_SPI_USR2_OFFS = 0x20;
export const ESP32C2_SPI_MOSI_DLEN_OFFS = 0x24;
export const ESP32C2_SPI_MISO_DLEN_OFFS = 0x28;
export const ESP32C2_SPI_W0_OFFS = 0x58;
export const ESP32C2_UART_DATE_REG_ADDR = 0x6000007c;
export const ESP32C2_BOOTLOADER_FLASH_OFFSET = 0x0;

export const ESP32C3_SPI_REG_BASE = 0x60002000;
export const ESP32C3_BASEFUSEADDR = 0x60008800;
export const ESP32C3_MACFUSEADDR = 0x60008800 + 0x044;
export const ESP32C3_SPI_USR_OFFS = 0x18;
export const ESP32C3_SPI_USR1_OFFS = 0x1c;
export const ESP32C3_SPI_USR2_OFFS = 0x20;
export const ESP32C3_SPI_MOSI_DLEN_OFFS = 0x24;
export const ESP32C3_SPI_MISO_DLEN_OFFS = 0x28;
export const ESP32C3_SPI_W0_OFFS = 0x58;
export const ESP32C3_UART_DATE_REG_ADDR = 0x6000007c;
export const ESP32C3_BOOTLOADER_FLASH_OFFSET = 0x0;

export const ESP32C6_SPI_REG_BASE = 0x60003000;
export const ESP32C6_BASEFUSEADDR = 0x600b0800;
export const ESP32C6_MACFUSEADDR = 0x600b0800 + 0x044;
export const ESP32C6_SPI_USR_OFFS = 0x18;
export const ESP32C6_SPI_USR1_OFFS = 0x1c;
export const ESP32C6_SPI_USR2_OFFS = 0x20;
export const ESP32C6_SPI_MOSI_DLEN_OFFS = 0x24;
export const ESP32C6_SPI_MISO_DLEN_OFFS = 0x28;
export const ESP32C6_SPI_W0_OFFS = 0x58;
export const ESP32C6_UART_DATE_REG_ADDR = 0x6000007c;
export const ESP32C6_BOOTLOADER_FLASH_OFFSET = 0x0;

export const ESP32H2_SPI_REG_BASE = 0x60002000;
export const ESP32H2_BASEFUSEADDR = 0x6001a000;
export const ESP32H2_MACFUSEADDR = 0x6001a000 + 0x044;
export const ESP32H2_SPI_USR_OFFS = 0x18;
export const ESP32H2_SPI_USR1_OFFS = 0x1c;
export const ESP32H2_SPI_USR2_OFFS = 0x20;
export const ESP32H2_SPI_MOSI_DLEN_OFFS = 0x24;
export const ESP32H2_SPI_MISO_DLEN_OFFS = 0x28;
export const ESP32H2_SPI_W0_OFFS = 0x58;
export const ESP32H2_UART_DATE_REG_ADDR = 0x6000007c;
export const ESP32H2_BOOTLOADER_FLASH_OFFSET = 0x0;

export interface SpiFlashAddresses {
  regBase: number;
  baseFuse: number;
  macFuse: number;
  usrOffs: number;
  usr1Offs: number;
  usr2Offs: number;
  mosiDlenOffs: number;
  misoDlenOffs: number;
  w0Offs: number;
  uartDateReg: number;
  flashOffs: number;
}

export const SYNC_PACKET = toByteArray(
  "\x07\x07\x12 UUUUUUUUUUUUUUUUUUUUUUUUUUUUUUUU",
);
export const CHIP_DETECT_MAGIC_REG_ADDR = 0x40001000;
// These values for the families are made up; nothing that esptool uses.
export const CHIP_FAMILY_ESP8266 = 0x8266;
export const CHIP_FAMILY_ESP32 = 0x32;
export const CHIP_FAMILY_ESP32S2 = 0x3252;
export const CHIP_FAMILY_ESP32S3 = 0x3253;
export const CHIP_FAMILY_ESP32C2 = 0x32c2;
export const CHIP_FAMILY_ESP32C3 = 0x32c3;
export const CHIP_FAMILY_ESP32C6 = 0x32c6;
export const CHIP_FAMILY_ESP32H2 = 0x3272;
export type ChipFamily =
  | typeof CHIP_FAMILY_ESP8266
  | typeof CHIP_FAMILY_ESP32
  | typeof CHIP_FAMILY_ESP32S2
  | typeof CHIP_FAMILY_ESP32S3
  | typeof CHIP_FAMILY_ESP32C2
  | typeof CHIP_FAMILY_ESP32C3
  | typeof CHIP_FAMILY_ESP32C6
  | typeof CHIP_FAMILY_ESP32H2;

interface ChipInfo {
  [magicValue: number]: {
    name: string;
    family: ChipFamily;
  };
}

export const CHIP_DETECT_MAGIC_VALUES: ChipInfo = {
  0xfff0c101: { name: "ESP8266", family: CHIP_FAMILY_ESP8266 },
  0x00f01d83: { name: "ESP32", family: CHIP_FAMILY_ESP32 },
  0x000007c6: { name: "ESP32-S2", family: CHIP_FAMILY_ESP32S2 },
  0x9: { name: "ESP32-S3", family: CHIP_FAMILY_ESP32S3 },
  0xeb004136: { name: "ESP32-S3(beta2)", family: CHIP_FAMILY_ESP32S3 },
  0x6f51306f: { name: "ESP32-C2", family: CHIP_FAMILY_ESP32C2 },
  0x7c41a06f: { name: "ESP32-C2", family: CHIP_FAMILY_ESP32C2 },
  0x6921506f: { name: "ESP32-C3", family: CHIP_FAMILY_ESP32C3 },
  0x1b31506f: { name: "ESP32-C3", family: CHIP_FAMILY_ESP32C3 },
  0xd7b73e80: { name: "ESP32-H2", family: CHIP_FAMILY_ESP32H2 },
  0x0da1806f: { name: "ESP32-C6(beta)", family: CHIP_FAMILY_ESP32C6 },
  0x2ce0806f: { name: "ESP32-C6", family: CHIP_FAMILY_ESP32C6 },
};

// Commands supported by ESP8266 ROM bootloader
export const ESP_FLASH_BEGIN = 0x02;
export const ESP_FLASH_DATA = 0x03;
export const ESP_FLASH_END = 0x04;
export const ESP_MEM_BEGIN = 0x05;
export const ESP_MEM_END = 0x06;
export const ESP_MEM_DATA = 0x07;
export const ESP_SYNC = 0x08;
export const ESP_WRITE_REG = 0x09;
export const ESP_READ_REG = 0x0a;

export const ESP_ERASE_FLASH = 0xd0;
export const ESP_ERASE_REGION = 0xd1;

export const ESP_SPI_SET_PARAMS = 0x0b;
export const ESP_SPI_ATTACH = 0x0d;
export const ESP_CHANGE_BAUDRATE = 0x0f;
export const ESP_SPI_FLASH_MD5 = 0x13;
export const ESP_CHECKSUM_MAGIC = 0xef;
export const ESP_FLASH_DEFL_BEGIN = 0x10;
export const ESP_FLASH_DEFL_DATA = 0x11;
export const ESP_FLASH_DEFL_END = 0x12;

export const ROM_INVALID_RECV_MSG = 0x05;

export const USB_RAM_BLOCK = 0x800;
export const ESP_RAM_BLOCK = 0x1800;

// Timeouts
export const DEFAULT_TIMEOUT = 3000;
export const CHIP_ERASE_TIMEOUT = 600000; // timeout for full chip erase in ms
export const MAX_TIMEOUT = CHIP_ERASE_TIMEOUT * 2; // longest any command can run in ms
export const SYNC_TIMEOUT = 100; // timeout for syncing with bootloader in ms
export const ERASE_REGION_TIMEOUT_PER_MB = 30000; // timeout (per megabyte) for erasing a region in ms
export const MEM_END_ROM_TIMEOUT = 500;

/**
 * @name timeoutPerMb
 * Scales timeouts which are size-specific
 */
export const timeoutPerMb = (secondsPerMb: number, sizeBytes: number) => {
  let result = Math.floor(secondsPerMb * (sizeBytes / 0x1e6));
  if (result < DEFAULT_TIMEOUT) {
    return DEFAULT_TIMEOUT;
  }
  return result;
};

export const getSpiFlashAddresses = (
  chipFamily: ChipFamily,
): SpiFlashAddresses => {
  switch (chipFamily) {
    case CHIP_FAMILY_ESP32:
      return {
        regBase: ESP32_SPI_REG_BASE,
        baseFuse: ESP32_BASEFUSEADDR,
        macFuse: ESP32_MACFUSEADDR,
        usrOffs: ESP32_SPI_USR_OFFS,
        usr1Offs: ESP32_SPI_USR1_OFFS,
        usr2Offs: ESP32_SPI_USR2_OFFS,
        mosiDlenOffs: ESP32_SPI_MOSI_DLEN_OFFS,
        misoDlenOffs: ESP32_SPI_MISO_DLEN_OFFS,
        w0Offs: ESP32_SPI_W0_OFFS,
        uartDateReg: ESP32_UART_DATE_REG_ADDR,
        flashOffs: ESP32_BOOTLOADER_FLASH_OFFSET,
      };
    case CHIP_FAMILY_ESP32S2:
      return {
        regBase: ESP32S2_SPI_REG_BASE,
        baseFuse: ESP32S2_BASEFUSEADDR,
        macFuse: ESP32S2_MACFUSEADDR,
        usrOffs: ESP32S2_SPI_USR_OFFS,
        usr1Offs: ESP32S2_SPI_USR1_OFFS,
        usr2Offs: ESP32S2_SPI_USR2_OFFS,
        mosiDlenOffs: ESP32S2_SPI_MOSI_DLEN_OFFS,
        misoDlenOffs: ESP32S2_SPI_MISO_DLEN_OFFS,
        w0Offs: ESP32S2_SPI_W0_OFFS,
        uartDateReg: ESP32S2_UART_DATE_REG_ADDR,
        flashOffs: ESP32S2_BOOTLOADER_FLASH_OFFSET,
      };
    case CHIP_FAMILY_ESP32S3:
      return {
        regBase: ESP32S3_SPI_REG_BASE,
        usrOffs: ESP32S3_SPI_USR_OFFS,
        baseFuse: ESP32S3_BASEFUSEADDR,
        macFuse: ESP32S3_MACFUSEADDR,
        usr1Offs: ESP32S3_SPI_USR1_OFFS,
        usr2Offs: ESP32S3_SPI_USR2_OFFS,
        mosiDlenOffs: ESP32S3_SPI_MOSI_DLEN_OFFS,
        misoDlenOffs: ESP32S3_SPI_MISO_DLEN_OFFS,
        w0Offs: ESP32S3_SPI_W0_OFFS,
        uartDateReg: ESP32S3_UART_DATE_REG_ADDR,
        flashOffs: ESP32S3_BOOTLOADER_FLASH_OFFSET,
      };
    case CHIP_FAMILY_ESP8266:
      return {
        regBase: ESP8266_SPI_REG_BASE,
        usrOffs: ESP8266_SPI_USR_OFFS,
        baseFuse: ESP8266_BASEFUSEADDR,
        macFuse: ESP8266_MACFUSEADDR,
        usr1Offs: ESP8266_SPI_USR1_OFFS,
        usr2Offs: ESP8266_SPI_USR2_OFFS,
        mosiDlenOffs: ESP8266_SPI_MOSI_DLEN_OFFS,
        misoDlenOffs: ESP8266_SPI_MISO_DLEN_OFFS,
        w0Offs: ESP8266_SPI_W0_OFFS,
        uartDateReg: ESP8266_UART_DATE_REG_ADDR,
        flashOffs: ESP8266_BOOTLOADER_FLASH_OFFSET,
      };
    case CHIP_FAMILY_ESP32C2:
      return {
        regBase: ESP32C2_SPI_REG_BASE,
        baseFuse: ESP32C2_BASEFUSEADDR,
        macFuse: ESP32C2_MACFUSEADDR,
        usrOffs: ESP32C2_SPI_USR_OFFS,
        usr1Offs: ESP32C2_SPI_USR1_OFFS,
        usr2Offs: ESP32C2_SPI_USR2_OFFS,
        mosiDlenOffs: ESP32C2_SPI_MOSI_DLEN_OFFS,
        misoDlenOffs: ESP32C2_SPI_MISO_DLEN_OFFS,
        w0Offs: ESP32C2_SPI_W0_OFFS,
        uartDateReg: ESP32C2_UART_DATE_REG_ADDR,
        flashOffs: ESP32C2_BOOTLOADER_FLASH_OFFSET,
      };
    case CHIP_FAMILY_ESP32C3:
      return {
        regBase: ESP32C3_SPI_REG_BASE,
        baseFuse: ESP32C3_BASEFUSEADDR,
        macFuse: ESP32C3_MACFUSEADDR,
        usrOffs: ESP32C3_SPI_USR_OFFS,
        usr1Offs: ESP32C3_SPI_USR1_OFFS,
        usr2Offs: ESP32C3_SPI_USR2_OFFS,
        mosiDlenOffs: ESP32C3_SPI_MOSI_DLEN_OFFS,
        misoDlenOffs: ESP32C3_SPI_MISO_DLEN_OFFS,
        w0Offs: ESP32C3_SPI_W0_OFFS,
        uartDateReg: ESP32C3_UART_DATE_REG_ADDR,
        flashOffs: ESP32C3_BOOTLOADER_FLASH_OFFSET,
      };
    case CHIP_FAMILY_ESP32C6:
      return {
        regBase: ESP32C6_SPI_REG_BASE,
        baseFuse: ESP32C6_BASEFUSEADDR,
        macFuse: ESP32C6_MACFUSEADDR,
        usrOffs: ESP32C6_SPI_USR_OFFS,
        usr1Offs: ESP32C6_SPI_USR1_OFFS,
        usr2Offs: ESP32C6_SPI_USR2_OFFS,
        mosiDlenOffs: ESP32C6_SPI_MOSI_DLEN_OFFS,
        misoDlenOffs: ESP32C6_SPI_MISO_DLEN_OFFS,
        w0Offs: ESP32C6_SPI_W0_OFFS,
        uartDateReg: ESP32C6_UART_DATE_REG_ADDR,
        flashOffs: ESP32C6_BOOTLOADER_FLASH_OFFSET,
      };
    case CHIP_FAMILY_ESP32H2:
      return {
        regBase: ESP32H2_SPI_REG_BASE,
        baseFuse: ESP32H2_BASEFUSEADDR,
        macFuse: ESP32H2_MACFUSEADDR,
        usrOffs: ESP32H2_SPI_USR_OFFS,
        usr1Offs: ESP32H2_SPI_USR1_OFFS,
        usr2Offs: ESP32H2_SPI_USR2_OFFS,
        mosiDlenOffs: ESP32H2_SPI_MOSI_DLEN_OFFS,
        misoDlenOffs: ESP32H2_SPI_MISO_DLEN_OFFS,
        w0Offs: ESP32H2_SPI_W0_OFFS,
        uartDateReg: ESP32H2_UART_DATE_REG_ADDR,
        flashOffs: ESP32H2_BOOTLOADER_FLASH_OFFSET,
      };
    default:
      return {
        regBase: -1,
        baseFuse: -1,
        macFuse: -1,
        usrOffs: -1,
        usr1Offs: -1,
        usr2Offs: -1,
        mosiDlenOffs: -1,
        misoDlenOffs: -1,
        w0Offs: -1,
        uartDateReg: -1,
        flashOffs: -1,
      };
  }
};

export class SlipReadError extends Error {
  constructor(message: string) {
    super(message);
    this.name = "SlipReadError";
  }
}
