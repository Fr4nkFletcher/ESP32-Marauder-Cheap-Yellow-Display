import {
  CHIP_FAMILY_ESP32,
  CHIP_FAMILY_ESP32S2,
  CHIP_FAMILY_ESP32S3,
  CHIP_FAMILY_ESP32C2,
  CHIP_FAMILY_ESP32C3,
  CHIP_FAMILY_ESP32C6,
  CHIP_FAMILY_ESP32H2,
  CHIP_FAMILY_ESP8266,
  MAX_TIMEOUT,
  Logger,
  DEFAULT_TIMEOUT,
  ERASE_REGION_TIMEOUT_PER_MB,
  ESP_CHANGE_BAUDRATE,
  ESP_CHECKSUM_MAGIC,
  ESP_FLASH_BEGIN,
  ESP_FLASH_DATA,
  ESP_FLASH_END,
  ESP_MEM_BEGIN,
  ESP_MEM_DATA,
  ESP_MEM_END,
  ESP_READ_REG,
  ESP_WRITE_REG,
  ESP_SPI_ATTACH,
  ESP_SYNC,
  FLASH_SECTOR_SIZE,
  FLASH_WRITE_SIZE,
  STUB_FLASH_WRITE_SIZE,
  MEM_END_ROM_TIMEOUT,
  ROM_INVALID_RECV_MSG,
  SYNC_PACKET,
  SYNC_TIMEOUT,
  USB_RAM_BLOCK,
  ChipFamily,
  ESP_ERASE_FLASH,
  CHIP_ERASE_TIMEOUT,
  timeoutPerMb,
  ESP_ROM_BAUD,
  USB_JTAG_SERIAL_PID,
  ESP_FLASH_DEFL_BEGIN,
  ESP_FLASH_DEFL_DATA,
  ESP_FLASH_DEFL_END,
  getSpiFlashAddresses,
  SpiFlashAddresses,
  DETECTED_FLASH_SIZES,
  CHIP_DETECT_MAGIC_REG_ADDR,
  CHIP_DETECT_MAGIC_VALUES,
  SlipReadError,
} from "./const";
import { getStubCode } from "./stubs";
import { hexFormatter, sleep, slipEncode, toHex } from "./util";
// @ts-ignore
import { deflate } from "pako/dist/pako.esm.mjs";
import { pack, unpack } from "./struct";

export class ESPLoader extends EventTarget {
  chipFamily!: ChipFamily;
  chipName: string | null = null;
  _efuses = new Array(4).fill(0);
  _flashsize = 4 * 1024 * 1024;
  debug = false;
  IS_STUB = false;
  connected = true;
  flashSize: string | null = null;

  __inputBuffer?: number[];
  private _reader?: ReadableStreamDefaultReader<Uint8Array>;

  constructor(
    public port: SerialPort,
    public logger: Logger,
    private _parent?: ESPLoader,
  ) {
    super();
  }

  private get _inputBuffer(): number[] {
    return this._parent ? this._parent._inputBuffer : this.__inputBuffer!;
  }

  async initialize() {
    await this.hardReset(true);

    if (!this._parent) {
      this.__inputBuffer = [];
      // Don't await this promise so it doesn't block rest of method.
      this.readLoop();
    }
    await this.sync();

    // Determine chip family and name
    let chipMagicValue = await this.readRegister(CHIP_DETECT_MAGIC_REG_ADDR);
    let chip = CHIP_DETECT_MAGIC_VALUES[chipMagicValue >>> 0];
    if (chip === undefined) {
      throw new Error(
        `Unknown Chip: Hex: ${toHex(
          chipMagicValue >>> 0,
          8,
        ).toLowerCase()} Number: ${chipMagicValue}`,
      );
    }
    this.chipName = chip.name;
    this.chipFamily = chip.family;

    // Read the OTP data for this chip and store into this.efuses array
    let FlAddr = getSpiFlashAddresses(this.getChipFamily());
    let AddrMAC = FlAddr.macFuse;
    for (let i = 0; i < 4; i++) {
      this._efuses[i] = await this.readRegister(AddrMAC + 4 * i);
    }
    this.logger.log(`Chip type ${this.chipName}`);
    //this.logger.log("FLASHID");
  }

  /**
   * @name readLoop
   * Reads data from the input stream and places it in the inputBuffer
   */
  async readLoop() {
    if (this.debug) {
      this.logger.debug("Starting read loop");
    }

    this._reader = this.port.readable!.getReader();

    try {
      while (true) {
        const { value, done } = await this._reader.read();
        if (done) {
          this._reader.releaseLock();
          break;
        }
        if (!value || value.length === 0) {
          continue;
        }
        this._inputBuffer.push(...Array.from(value));
      }
    } catch (err) {
      console.error("Read loop got disconnected");
    }
    // Disconnected!
    this.connected = false;
    this.dispatchEvent(new Event("disconnect"));
    this.logger.debug("Finished read loop");
  }

  sleep(ms = 100) {
    return new Promise((resolve) => setTimeout(resolve, ms));
  }

  state_DTR = false;
  async setRTS(state: boolean) {
    await this.port.setSignals({ requestToSend: state });
    // # Work-around for adapters on Windows using the usbser.sys driver:
    // # generate a dummy change to DTR so that the set-control-line-state
    // # request is sent with the updated RTS state and the same DTR state
    // Referenced to esptool.py
    await this.setDTR(this.state_DTR);
  }

  async setDTR(state: boolean) {
    this.state_DTR = state;
    await this.port.setSignals({ dataTerminalReady: state });
  }

  async hardReset(bootloader = false) {
    this.logger.log("Try hard reset.");
    if (bootloader) {
      // enter flash mode
      if (this.port.getInfo().usbProductId === USB_JTAG_SERIAL_PID) {
        // esp32c3 esp32s3 etc. build-in USB serial.
        // when connect to computer direct via usb, using following signals
        // to enter flash mode automatically.
        await this.setDTR(false);
        await this.setRTS(false);
        await this.sleep(100);

        await this.setDTR(true);
        await this.setRTS(false);
        await this.sleep(100);

        await this.setRTS(true);
        await this.setDTR(false);
        await this.setRTS(true);

        await this.sleep(100);
        await this.setDTR(false);
        await this.setRTS(false);
      } else {
        // otherwise, esp chip should be connected to computer via usb-serial
        // bridge chip like ch340,CP2102 etc.
        // use normal way to enter flash mode.
        await this.setDTR(false);
        await this.setRTS(true);
        await this.sleep(100);
        await this.setDTR(true);
        await this.setRTS(false);
        await this.sleep(50);
        await this.setDTR(false);
      }
    } else {
      // just reset
      await this.setRTS(true); // EN->LOW
      await this.sleep(100);
      await this.setRTS(false);
    }
    await new Promise((resolve) => setTimeout(resolve, 1000));
  }

  /**
   * @name macAddr
   * The MAC address burned into the OTP memory of the ESP chip
   */
  macAddr() {
    let macAddr = new Array(6).fill(0);
    let mac0 = this._efuses[0];
    let mac1 = this._efuses[1];
    let mac2 = this._efuses[2];
    let mac3 = this._efuses[3];
    let oui;
    if (this.chipFamily == CHIP_FAMILY_ESP8266) {
      if (mac3 != 0) {
        oui = [(mac3 >> 16) & 0xff, (mac3 >> 8) & 0xff, mac3 & 0xff];
      } else if (((mac1 >> 16) & 0xff) == 0) {
        oui = [0x18, 0xfe, 0x34];
      } else if (((mac1 >> 16) & 0xff) == 1) {
        oui = [0xac, 0xd0, 0x74];
      } else {
        throw new Error("Couldnt determine OUI");
      }

      macAddr[0] = oui[0];
      macAddr[1] = oui[1];
      macAddr[2] = oui[2];
      macAddr[3] = (mac1 >> 8) & 0xff;
      macAddr[4] = mac1 & 0xff;
      macAddr[5] = (mac0 >> 24) & 0xff;
    } else if (this.chipFamily == CHIP_FAMILY_ESP32) {
      macAddr[0] = (mac2 >> 8) & 0xff;
      macAddr[1] = mac2 & 0xff;
      macAddr[2] = (mac1 >> 24) & 0xff;
      macAddr[3] = (mac1 >> 16) & 0xff;
      macAddr[4] = (mac1 >> 8) & 0xff;
      macAddr[5] = mac1 & 0xff;
    } else if (
      this.chipFamily == CHIP_FAMILY_ESP32S2 ||
      this.chipFamily == CHIP_FAMILY_ESP32S3 ||
      this.chipFamily == CHIP_FAMILY_ESP32C2 ||
      this.chipFamily == CHIP_FAMILY_ESP32C3 ||
      this.chipFamily == CHIP_FAMILY_ESP32C6 ||
      this.chipFamily == CHIP_FAMILY_ESP32H2
    ) {
      macAddr[0] = (mac1 >> 8) & 0xff;
      macAddr[1] = mac1 & 0xff;
      macAddr[2] = (mac0 >> 24) & 0xff;
      macAddr[3] = (mac0 >> 16) & 0xff;
      macAddr[4] = (mac0 >> 8) & 0xff;
      macAddr[5] = mac0 & 0xff;
    } else {
      throw new Error("Unknown chip family");
    }
    return macAddr;
  }

  async readRegister(reg: number) {
    if (this.debug) {
      this.logger.debug("Reading from Register " + toHex(reg, 8));
    }
    let packet = pack("<I", reg);
    await this.sendCommand(ESP_READ_REG, packet);
    let [val, _data] = await this.getResponse(ESP_READ_REG);
    return val;
  }

  /**
   * @name checkCommand
   * Send a command packet, check that the command succeeded and
   * return a tuple with the value and data.
   * See the ESP Serial Protocol for more details on what value/data are
   */
  async checkCommand(
    opcode: number,
    buffer: number[],
    checksum = 0,
    timeout = DEFAULT_TIMEOUT,
  ): Promise<[number, number[]]> {
    timeout = Math.min(timeout, MAX_TIMEOUT);
    await this.sendCommand(opcode, buffer, checksum);
    let [value, data] = await this.getResponse(opcode, timeout);

    if (data === null) {
      throw new Error("Didn't get enough status bytes");
    }

    let statusLen = 0;

    if (this.IS_STUB || this.chipFamily == CHIP_FAMILY_ESP8266) {
      statusLen = 2;
    } else if (
      [
        CHIP_FAMILY_ESP32,
        CHIP_FAMILY_ESP32S2,
        CHIP_FAMILY_ESP32S3,
        CHIP_FAMILY_ESP32C2,
        CHIP_FAMILY_ESP32C3,
        CHIP_FAMILY_ESP32C6,
        CHIP_FAMILY_ESP32H2,
      ].includes(this.chipFamily)
    ) {
      statusLen = 4;
    } else {
      if ([2, 4].includes(data.length)) {
        statusLen = data.length;
      }
    }

    if (data.length < statusLen) {
      throw new Error("Didn't get enough status bytes");
    }
    let status = data.slice(-statusLen, data.length);
    data = data.slice(0, -statusLen);
    if (this.debug) {
      this.logger.debug("status", status);
      this.logger.debug("value", value);
      this.logger.debug("data", data);
    }
    if (status[0] == 1) {
      if (status[1] == ROM_INVALID_RECV_MSG) {
        throw new Error("Invalid (unsupported) command " + toHex(opcode));
      } else {
        throw new Error("Command failure error code " + toHex(status[1]));
      }
    }

    return [value, data];
  }

  /**
   * @name sendCommand
   * Send a slip-encoded, checksummed command over the UART,
   * does not check response
   */
  async sendCommand(opcode: number, buffer: number[], checksum = 0) {
    let packet = slipEncode([
      ...pack("<BBHI", 0x00, opcode, buffer.length, checksum),
      ...buffer,
    ]);
    if (this.debug) {
      this.logger.debug(
        `Writing ${packet.length} byte${packet.length == 1 ? "" : "s"}:`,
        packet,
      );
    }
    await this.writeToStream(packet);
  }

  /**
   * @name readPacket
   * Generator to read SLIP packets from a serial port.
   * Yields one full SLIP packet at a time, raises exception on timeout or invalid data.
   * Designed to avoid too many calls to serial.read(1), which can bog
   * down on slow systems.
   */

  async readPacket(timeout: number): Promise<number[]> {
    let partialPacket: number[] | null = null;
    let inEscape = false;
    let readBytes: number[] = [];
    while (true) {
      let stamp = Date.now();
      readBytes = [];
      while (Date.now() - stamp < timeout) {
        if (this._inputBuffer.length > 0) {
          readBytes.push(this._inputBuffer.shift()!);
          break;
        } else {
          await sleep(10);
        }
      }
      if (readBytes.length == 0) {
        let waitingFor = partialPacket === null ? "header" : "content";
        throw new SlipReadError("Timed out waiting for packet " + waitingFor);
      }
      if (this.debug)
        this.logger.debug(
          "Read " + readBytes.length + " bytes: " + hexFormatter(readBytes),
        );
      for (let b of readBytes) {
        if (partialPacket === null) {
          // waiting for packet header
          if (b == 0xc0) {
            partialPacket = [];
          } else {
            if (this.debug) {
              this.logger.debug(
                "Read invalid data: " + hexFormatter(readBytes),
              );
              this.logger.debug(
                "Remaining data in serial buffer: " +
                  hexFormatter(this._inputBuffer),
              );
            }
            throw new SlipReadError(
              "Invalid head of packet (" + toHex(b) + ")",
            );
          }
        } else if (inEscape) {
          // part-way through escape sequence
          inEscape = false;
          if (b == 0xdc) {
            partialPacket.push(0xc0);
          } else if (b == 0xdd) {
            partialPacket.push(0xdb);
          } else {
            if (this.debug) {
              this.logger.debug(
                "Read invalid data: " + hexFormatter(readBytes),
              );
              this.logger.debug(
                "Remaining data in serial buffer: " +
                  hexFormatter(this._inputBuffer),
              );
            }
            throw new SlipReadError(
              "Invalid SLIP escape (0xdb, " + toHex(b) + ")",
            );
          }
        } else if (b == 0xdb) {
          // start of escape sequence
          inEscape = true;
        } else if (b == 0xc0) {
          // end of packet
          if (this.debug)
            this.logger.debug(
              "Received full packet: " + hexFormatter(partialPacket),
            );
          return partialPacket;
        } else {
          // normal byte in packet
          partialPacket.push(b);
        }
      }
    }
    throw new SlipReadError("Invalid state");
  }

  /**
   * @name getResponse
   * Read response data and decodes the slip packet, then parses
   * out the value/data and returns as a tuple of (value, data) where
   * each is a list of bytes
   */
  async getResponse(
    opcode: number,
    timeout = DEFAULT_TIMEOUT,
  ): Promise<[number, number[]]> {
    for (let i = 0; i < 100; i++) {
      const packet = await this.readPacket(timeout);

      if (packet.length < 8) {
        continue;
      }

      const [resp, opRet, _lenRet, val] = unpack("<BBHI", packet.slice(0, 8));
      if (resp != 1) {
        continue;
      }
      const data = packet.slice(8);
      if (opcode == null || opRet == opcode) {
        return [val, data];
      }
      if (data[0] != 0 && data[1] == ROM_INVALID_RECV_MSG) {
        this._inputBuffer.length = 0;
        throw new Error(`Invalid (unsupported) command ${toHex(opcode)}`);
      }
    }
    throw "Response doesn't match request";
  }

  /**
   * @name checksum
   * Calculate checksum of a blob, as it is defined by the ROM
   */
  checksum(data: number[], state = ESP_CHECKSUM_MAGIC) {
    for (let b of data) {
      state ^= b;
    }
    return state;
  }

  async setBaudrate(baud: number) {
    if (this.chipFamily == CHIP_FAMILY_ESP8266) {
      throw new Error("Changing baud rate is not supported on the ESP8266");
    }

    this.logger.log("Attempting to change baud rate to " + baud + "...");

    try {
      // Send ESP_ROM_BAUD(115200) as the old one if running STUB otherwise 0
      let buffer = pack("<II", baud, this.IS_STUB ? ESP_ROM_BAUD : 0);
      await this.checkCommand(ESP_CHANGE_BAUDRATE, buffer);
    } catch (e) {
      console.error(e);
      throw new Error(
        `Unable to change the baud rate to ${baud}: No response from set baud rate command.`,
      );
    }

    if (this._parent) {
      await this._parent.reconfigurePort(baud);
    } else {
      await this.reconfigurePort(baud);
    }
  }

  async reconfigurePort(baud: number) {
    try {
      // SerialPort does not allow to be reconfigured while open so we close and re-open
      // reader.cancel() causes the Promise returned by the read() operation running on
      // the readLoop to return immediately with { value: undefined, done: true } and thus
      // breaking the loop and exiting readLoop();
      await this._reader?.cancel();
      await this.port.close();

      // Reopen Port
      await this.port.open({ baudRate: baud });

      // Restart Readloop
      this.readLoop();

      this.logger.log(`Changed baud rate to ${baud}`);
    } catch (e) {
      console.error(e);
      throw new Error(`Unable to change the baud rate to ${baud}: ${e}`);
    }
  }

  /**
   * @name sync
   * Put into ROM bootload mode & attempt to synchronize with the
   * ESP ROM bootloader, we will retry a few times
   */
  async sync() {
    for (let i = 0; i < 5; i++) {
      this._inputBuffer.length = 0;
      let response = await this._sync();
      if (response) {
        await sleep(100);
        return true;
      }
      await sleep(100);
    }

    throw new Error("Couldn't sync to ESP. Try resetting.");
  }

  /**
   * @name _sync
   * Perform a soft-sync using AT sync packets, does not perform
   * any hardware resetting
   */
  async _sync() {
    await this.sendCommand(ESP_SYNC, SYNC_PACKET);
    for (let i = 0; i < 8; i++) {
      try {
        let [_reply, data] = await this.getResponse(ESP_SYNC, SYNC_TIMEOUT);
        if (data.length > 1 && data[0] == 0 && data[1] == 0) {
          return true;
        }
      } catch (err) {
        // If read packet fails.
      }
    }
    return false;
  }

  /**
   * @name getFlashWriteSize
   * Get the Flash write size based on the chip
   */
  getFlashWriteSize() {
    if (this.IS_STUB) {
      return STUB_FLASH_WRITE_SIZE;
    }
    return FLASH_WRITE_SIZE;
  }

  /**
   * @name flashData
   * Program a full, uncompressed binary file into SPI Flash at
   *   a given offset. If an ESP32 and md5 string is passed in, will also
   *   verify memory. ESP8266 does not have checksum memory verification in
   *   ROM
   */
  async flashData(
    binaryData: ArrayBuffer,
    updateProgress: (bytesWritten: number, totalBytes: number) => void,
    offset = 0,
    compress = false,
  ) {
    if (binaryData.byteLength >= 8) {
      // unpack the (potential) image header
      var header = Array.from(new Uint8Array(binaryData, 0, 4));
      let headerMagic = header[0];
      let headerFlashMode = header[2];
      let headerFlashSizeFreq = header[3];

      this.logger.log(
        `Image header, Magic=${toHex(headerMagic)}, FlashMode=${toHex(
          headerFlashMode,
        )}, FlashSizeFreq=${toHex(headerFlashSizeFreq)}`,
      );
    }

    let uncompressedFilesize = binaryData.byteLength;
    let compressedFilesize = 0;

    let dataToFlash;
    let timeout = DEFAULT_TIMEOUT;

    if (compress) {
      dataToFlash = deflate(new Uint8Array(binaryData), {
        level: 9,
      }).buffer;
      compressedFilesize = dataToFlash.byteLength;
      this.logger.log(
        `Writing data with filesize: ${uncompressedFilesize}. Compressed Size: ${compressedFilesize}`,
      );
      timeout = await this.flashDeflBegin(
        uncompressedFilesize,
        compressedFilesize,
        offset,
      );
    } else {
      this.logger.log(`Writing data with filesize: ${uncompressedFilesize}`);
      dataToFlash = binaryData;
      await this.flashBegin(uncompressedFilesize, offset);
    }

    let block = [];
    let seq = 0;
    let written = 0;
    let position = 0;
    let stamp = Date.now();
    let flashWriteSize = this.getFlashWriteSize();

    let filesize = compress ? compressedFilesize : uncompressedFilesize;

    while (filesize - position > 0) {
      if (this.debug) {
        this.logger.log(
          `Writing at ${toHex(offset + seq * flashWriteSize, 8)} `,
        );
      }
      if (filesize - position >= flashWriteSize) {
        block = Array.from(
          new Uint8Array(dataToFlash, position, flashWriteSize),
        );
      } else {
        // Pad the last block only if we are sending uncompressed data.
        block = Array.from(
          new Uint8Array(dataToFlash, position, filesize - position),
        );
        if (!compress) {
          block = block.concat(
            new Array(flashWriteSize - block.length).fill(0xff),
          );
        }
      }
      if (compress) {
        await this.flashDeflBlock(block, seq, timeout);
      } else {
        await this.flashBlock(block, seq);
      }
      seq += 1;
      // If using compression we update the progress with the proportional size of the block taking into account the compression ratio.
      // This way we report progress on the uncompressed size
      written += compress
        ? Math.round((block.length * uncompressedFilesize) / compressedFilesize)
        : block.length;
      position += flashWriteSize;
      updateProgress(
        Math.min(written, uncompressedFilesize),
        uncompressedFilesize,
      );
    }
    this.logger.log(
      "Took " + (Date.now() - stamp) + "ms to write " + filesize + " bytes",
    );

    // Only send flashF finish if running the stub because ir causes the ROM to exit and run user code
    if (this.IS_STUB) {
      await this.flashBegin(0, 0);
      if (compress) {
        await this.flashDeflFinish();
      } else {
        await this.flashFinish();
      }
    }
  }

  /**
   * @name flashBlock
   * Send one block of data to program into SPI Flash memory
   */
  async flashBlock(data: number[], seq: number, timeout = DEFAULT_TIMEOUT) {
    await this.checkCommand(
      ESP_FLASH_DATA,
      pack("<IIII", data.length, seq, 0, 0).concat(data),
      this.checksum(data),
      timeout,
    );
  }
  async flashDeflBlock(data: number[], seq: number, timeout = DEFAULT_TIMEOUT) {
    await this.checkCommand(
      ESP_FLASH_DEFL_DATA,
      pack("<IIII", data.length, seq, 0, 0).concat(data),
      this.checksum(data),
      timeout,
    );
  }

  /**
   * @name flashBegin
   * Prepare for flashing by attaching SPI chip and erasing the
   *   number of blocks requred.
   */
  async flashBegin(size = 0, offset = 0, encrypted = false) {
    let eraseSize;
    let buffer;
    let flashWriteSize = this.getFlashWriteSize();
    if (
      !this.IS_STUB &&
      [
        CHIP_FAMILY_ESP32,
        CHIP_FAMILY_ESP32S2,
        CHIP_FAMILY_ESP32S3,
        CHIP_FAMILY_ESP32C2,
        CHIP_FAMILY_ESP32C3,
        CHIP_FAMILY_ESP32C6,
        CHIP_FAMILY_ESP32H2,
      ].includes(this.chipFamily)
    ) {
      await this.checkCommand(ESP_SPI_ATTACH, new Array(8).fill(0));
    }
    let numBlocks = Math.floor((size + flashWriteSize - 1) / flashWriteSize);
    if (this.chipFamily == CHIP_FAMILY_ESP8266) {
      eraseSize = this.getEraseSize(offset, size);
    } else {
      eraseSize = size;
    }

    let timeout;
    if (this.IS_STUB) {
      timeout = DEFAULT_TIMEOUT;
    } else {
      timeout = timeoutPerMb(ERASE_REGION_TIMEOUT_PER_MB, size);
    }

    let stamp = Date.now();
    buffer = pack("<IIII", eraseSize, numBlocks, flashWriteSize, offset);
    if (
      this.chipFamily == CHIP_FAMILY_ESP32 ||
      this.chipFamily == CHIP_FAMILY_ESP32S2 ||
      this.chipFamily == CHIP_FAMILY_ESP32S3 ||
      this.chipFamily == CHIP_FAMILY_ESP32C2 ||
      this.chipFamily == CHIP_FAMILY_ESP32C3 ||
      this.chipFamily == CHIP_FAMILY_ESP32C6 ||
      this.chipFamily == CHIP_FAMILY_ESP32H2
    ) {
      buffer = buffer.concat(pack("<I", encrypted ? 1 : 0));
    }
    this.logger.log(
      "Erase size " +
        eraseSize +
        ", blocks " +
        numBlocks +
        ", block size " +
        toHex(flashWriteSize, 4) +
        ", offset " +
        toHex(offset, 4) +
        ", encrypted " +
        (encrypted ? "yes" : "no"),
    );
    await this.checkCommand(ESP_FLASH_BEGIN, buffer, 0, timeout);
    if (size != 0 && !this.IS_STUB) {
      this.logger.log(
        "Took " + (Date.now() - stamp) + "ms to erase " + numBlocks + " bytes",
      );
    }
    return numBlocks;
  }

  /**
   * @name flashDeflBegin
   *
   */

  async flashDeflBegin(
    size = 0,
    compressedSize = 0,
    offset = 0,
    encrypted = false,
  ) {
    // Start downloading compressed data to Flash (performs an erase)
    // Returns number of blocks to write.
    let flashWriteSize = this.getFlashWriteSize();
    let numBlocks = Math.floor(
      (compressedSize + flashWriteSize - 1) / flashWriteSize,
    );
    let eraseBlocks = Math.floor((size + flashWriteSize - 1) / flashWriteSize);
    let writeSize = 0;
    let timeout = 0;
    let buffer;

    if (this.IS_STUB) {
      writeSize = size; // stub expects number of bytes here, manages erasing internally
      timeout = timeoutPerMb(ERASE_REGION_TIMEOUT_PER_MB, writeSize); // ROM performs the erase up front
    } else {
      writeSize = eraseBlocks * flashWriteSize; // ROM expects rounded up to erase block size
      timeout = DEFAULT_TIMEOUT;
    }
    buffer = pack("<IIII", writeSize, numBlocks, flashWriteSize, offset);

    await this.checkCommand(ESP_FLASH_DEFL_BEGIN, buffer, 0, timeout);

    return timeout;
  }

  async flashFinish() {
    let buffer = pack("<I", 1);
    await this.checkCommand(ESP_FLASH_END, buffer);
  }

  async flashDeflFinish() {
    let buffer = pack("<I", 1);
    await this.checkCommand(ESP_FLASH_DEFL_END, buffer);
  }

  getBootloaderOffset() {
    let bootFlashOffs = getSpiFlashAddresses(this.getChipFamily());
    let BootldrFlashOffs = bootFlashOffs.flashOffs;
    return BootldrFlashOffs;
  }

  async flashId() {
    let SPIFLASH_RDID = 0x9f;
    let result = await this.runSpiFlashCommand(SPIFLASH_RDID, [], 24);
    return result;
  }

  getChipFamily() {
    return this._parent ? this._parent.chipFamily : this.chipFamily;
  }

  async writeRegister(
    address: number,
    value: number,
    mask = 0xffffffff,
    delayUs = 0,
    delayAfterUs = 0,
  ) {
    let buffer = pack("<IIII", address, value, mask, delayUs);
    if (delayAfterUs > 0) {
      // add a dummy write to a date register as an excuse to have a delay
      buffer.concat(
        pack(
          "<IIII",
          getSpiFlashAddresses(this.getChipFamily()).uartDateReg,
          0,
          0,
          delayAfterUs,
        ),
      );
    }
    await this.checkCommand(ESP_WRITE_REG, buffer);
  }

  async setDataLengths(
    spiAddresses: SpiFlashAddresses,
    mosiBits: number,
    misoBits: number,
  ) {
    if (spiAddresses.mosiDlenOffs != -1) {
      // ESP32/32S2/32S3/32C3 has a more sophisticated way to set up "user" commands
      let SPI_MOSI_DLEN_REG = spiAddresses.regBase + spiAddresses.mosiDlenOffs;
      let SPI_MISO_DLEN_REG = spiAddresses.regBase + spiAddresses.misoDlenOffs;
      if (mosiBits > 0) {
        await this.writeRegister(SPI_MOSI_DLEN_REG, mosiBits - 1);
      }
      if (misoBits > 0) {
        await this.writeRegister(SPI_MISO_DLEN_REG, misoBits - 1);
      }
    } else {
      let SPI_DATA_LEN_REG = spiAddresses.regBase + spiAddresses.usr1Offs;
      let SPI_MOSI_BITLEN_S = 17;
      let SPI_MISO_BITLEN_S = 8;
      let mosiMask = mosiBits == 0 ? 0 : mosiBits - 1;
      let misoMask = misoBits == 0 ? 0 : misoBits - 1;
      let value =
        (misoMask << SPI_MISO_BITLEN_S) | (mosiMask << SPI_MOSI_BITLEN_S);
      await this.writeRegister(SPI_DATA_LEN_REG, value);
    }
  }
  async waitDone(spiCmdReg: number, spiCmdUsr: number) {
    for (let i = 0; i < 10; i++) {
      let cmdValue = await this.readRegister(spiCmdReg);
      if ((cmdValue & spiCmdUsr) == 0) {
        return;
      }
    }
    throw Error("SPI command did not complete in time");
  }

  async runSpiFlashCommand(
    spiflashCommand: number,
    data: number[],
    readBits = 0,
  ) {
    // Run an arbitrary SPI flash command.

    // This function uses the "USR_COMMAND" functionality in the ESP
    // SPI hardware, rather than the precanned commands supported by
    // hardware. So the value of spiflash_command is an actual command
    // byte, sent over the wire.

    // After writing command byte, writes 'data' to MOSI and then
    // reads back 'read_bits' of reply on MISO. Result is a number.

    // SPI_USR register flags
    let SPI_USR_COMMAND = 1 << 31;
    let SPI_USR_MISO = 1 << 28;
    let SPI_USR_MOSI = 1 << 27;

    // SPI registers, base address differs ESP32* vs 8266
    let spiAddresses = getSpiFlashAddresses(this.getChipFamily());
    let base = spiAddresses.regBase;
    let SPI_CMD_REG = base;
    let SPI_USR_REG = base + spiAddresses.usrOffs;
    let SPI_USR2_REG = base + spiAddresses.usr2Offs;
    let SPI_W0_REG = base + spiAddresses.w0Offs;

    // SPI peripheral "command" bitmasks for SPI_CMD_REG
    let SPI_CMD_USR = 1 << 18;

    // shift values
    let SPI_USR2_COMMAND_LEN_SHIFT = 28;

    if (readBits > 32) {
      throw new Error(
        "Reading more than 32 bits back from a SPI flash operation is unsupported",
      );
    }
    if (data.length > 64) {
      throw new Error(
        "Writing more than 64 bytes of data with one SPI command is unsupported",
      );
    }

    let dataBits = data.length * 8;
    let oldSpiUsr = await this.readRegister(SPI_USR_REG);
    let oldSpiUsr2 = await this.readRegister(SPI_USR2_REG);

    let flags = SPI_USR_COMMAND;

    if (readBits > 0) {
      flags |= SPI_USR_MISO;
    }
    if (dataBits > 0) {
      flags |= SPI_USR_MOSI;
    }

    await this.setDataLengths(spiAddresses, dataBits, readBits);

    await this.writeRegister(SPI_USR_REG, flags);
    await this.writeRegister(
      SPI_USR2_REG,
      (7 << SPI_USR2_COMMAND_LEN_SHIFT) | spiflashCommand,
    );
    if (dataBits == 0) {
      await this.writeRegister(SPI_W0_REG, 0); // clear data register before we read it
    } else {
      data.concat(new Array(data.length % 4).fill(0x00)); // pad to 32-bit multiple

      let words = unpack("I".repeat(Math.floor(data.length / 4)), data);
      let nextReg = SPI_W0_REG;

      this.logger.debug(`Words Length: ${words.length}`);

      for (const word of words) {
        this.logger.debug(
          `Writing word ${toHex(word)} to register offset ${toHex(nextReg)}`,
        );
        await this.writeRegister(nextReg, word);
        nextReg += 4;
      }
    }
    await this.writeRegister(SPI_CMD_REG, SPI_CMD_USR);
    await this.waitDone(SPI_CMD_REG, SPI_CMD_USR);

    let status = await this.readRegister(SPI_W0_REG);
    // restore some SPI controller registers
    await this.writeRegister(SPI_USR_REG, oldSpiUsr);
    await this.writeRegister(SPI_USR2_REG, oldSpiUsr2);
    return status;
  }
  async detectFlashSize() {
    this.logger.log("Detecting Flash Size");

    let flashId = await this.flashId();
    let manufacturer = flashId & 0xff;
    let flashIdLowbyte = (flashId >> 16) & 0xff;

    this.logger.log(`FlashId: ${toHex(flashId)}`);
    this.logger.log(`Flash Manufacturer: ${manufacturer.toString(16)}`);
    this.logger.log(
      `Flash Device: ${((flashId >> 8) & 0xff).toString(
        16,
      )}${flashIdLowbyte.toString(16)}`,
    );

    this.flashSize = DETECTED_FLASH_SIZES[flashIdLowbyte];
    this.logger.log(`Auto-detected Flash size: ${this.flashSize}`);
  }

  /**
   * @name getEraseSize
   * Calculate an erase size given a specific size in bytes.
   *   Provides a workaround for the bootloader erase bug on ESP8266.
   */
  getEraseSize(offset: number, size: number) {
    let sectorsPerBlock = 16;
    let sectorSize = FLASH_SECTOR_SIZE;
    let numSectors = Math.floor((size + sectorSize - 1) / sectorSize);
    let startSector = Math.floor(offset / sectorSize);

    let headSectors = sectorsPerBlock - (startSector % sectorsPerBlock);
    if (numSectors < headSectors) {
      headSectors = numSectors;
    }

    if (numSectors < 2 * headSectors) {
      return Math.floor(((numSectors + 1) / 2) * sectorSize);
    }

    return (numSectors - headSectors) * sectorSize;
  }

  /**
   * @name memBegin (592)
   * Start downloading an application image to RAM
   */
  async memBegin(
    size: number,
    blocks: number,
    blocksize: number,
    offset: number,
  ) {
    return await this.checkCommand(
      ESP_MEM_BEGIN,
      pack("<IIII", size, blocks, blocksize, offset),
    );
  }

  /**
   * @name memBlock (609)
   * Send a block of an image to RAM
   */
  async memBlock(data: number[], seq: number) {
    return await this.checkCommand(
      ESP_MEM_DATA,
      pack("<IIII", data.length, seq, 0, 0).concat(data),
      this.checksum(data),
    );
  }

  /**
   * @name memFinish (615)
   * Leave download mode and run the application
   *
   * Sending ESP_MEM_END usually sends a correct response back, however sometimes
   * (with ROM loader) the executed code may reset the UART or change the baud rate
   * before the transmit FIFO is empty. So in these cases we set a short timeout and
   * ignore errors.
   */
  async memFinish(entrypoint = 0) {
    let timeout = this.IS_STUB ? DEFAULT_TIMEOUT : MEM_END_ROM_TIMEOUT;
    let data = pack("<II", entrypoint == 0 ? 1 : 0, entrypoint);
    return await this.checkCommand(ESP_MEM_END, data, 0, timeout);
  }

  async runStub(): Promise<EspStubLoader> {
    const stub: Record<string, any> = await getStubCode(this.chipFamily);

    // We're transferring over USB, right?
    let ramBlock = USB_RAM_BLOCK;

    // Upload
    this.logger.log("Uploading stub...");
    for (let field of ["text", "data"]) {
      if (Object.keys(stub).includes(field)) {
        let offset = stub[field + "_start"];
        let length = stub[field].length;
        let blocks = Math.floor((length + ramBlock - 1) / ramBlock);
        await this.memBegin(length, blocks, ramBlock, offset);
        for (let seq of Array(blocks).keys()) {
          let fromOffs = seq * ramBlock;
          let toOffs = fromOffs + ramBlock;
          if (toOffs > length) {
            toOffs = length;
          }
          await this.memBlock(stub[field].slice(fromOffs, toOffs), seq);
        }
      }
    }
    this.logger.log("Running stub...");
    await this.memFinish(stub["entry"]);

    let pChar: string;

    const p = await this.readPacket(500);
    pChar = String.fromCharCode(...p);

    if (pChar != "OHAI") {
      throw new Error("Failed to start stub. Unexpected response: " + pChar);
    }
    this.logger.log("Stub is now running...");
    const espStubLoader = new EspStubLoader(this.port, this.logger, this);

    // Try to autodetect the flash size as soon as the stub is running.
    await espStubLoader.detectFlashSize();

    return espStubLoader;
  }

  async writeToStream(data: number[]) {
    const writer = this.port.writable!.getWriter();
    await writer.write(new Uint8Array(data));
    try {
      writer.releaseLock();
    } catch (err) {
      console.error("Ignoring release lock error", err);
    }
  }

  async disconnect() {
    if (this._parent) {
      await this._parent.disconnect();
      return;
    }
    await this.port.writable!.getWriter().close();
    await new Promise((resolve) => {
      if (!this._reader) {
        resolve(undefined);
      }
      this.addEventListener("disconnect", resolve, { once: true });
      this._reader!.cancel();
    });
    this.connected = false;
  }
}

class EspStubLoader extends ESPLoader {
  /*
    The Stubloader has commands that run on the uploaded Stub Code in RAM
    rather than built in commands.
  */
  IS_STUB = true;

  /**
   * @name memBegin (592)
   * Start downloading an application image to RAM
   */
  async memBegin(
    size: number,
    blocks: number,
    blocksize: number,
    offset: number,
  ): Promise<any> {
    let stub = await getStubCode(this.chipFamily);
    let load_start = offset;
    let load_end = offset + size;
    console.log(load_start, load_end);
    console.log(
      stub.data_start,
      stub.data.length,
      stub.text_start,
      stub.text.length,
    );
    for (let [start, end] of [
      [stub.data_start, stub.data_start + stub.data.length],
      [stub.text_start, stub.text_start + stub.text.length],
    ]) {
      if (load_start < end && load_end > start) {
        throw new Error(
          "Software loader is resident at " +
            toHex(start, 8) +
            "-" +
            toHex(end, 8) +
            ". " +
            "Can't load binary at overlapping address range " +
            toHex(load_start, 8) +
            "-" +
            toHex(load_end, 8) +
            ". " +
            "Try changing the binary loading address.",
        );
      }
    }
  }

  /**
   * @name getEraseSize
   * depending on flash chip model the erase may take this long (maybe longer!)
   */
  async eraseFlash() {
    await this.checkCommand(ESP_ERASE_FLASH, [], 0, CHIP_ERASE_TIMEOUT);
  }
}
