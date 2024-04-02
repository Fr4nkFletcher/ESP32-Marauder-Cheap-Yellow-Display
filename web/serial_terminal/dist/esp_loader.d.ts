/// <reference types="w3c-web-serial" />
import { Logger, ChipFamily, SpiFlashAddresses } from "./const";
export declare class ESPLoader extends EventTarget {
    port: SerialPort;
    logger: Logger;
    private _parent?;
    chipFamily: ChipFamily;
    chipName: string | null;
    _efuses: any[];
    _flashsize: number;
    debug: boolean;
    IS_STUB: boolean;
    connected: boolean;
    flashSize: string | null;
    __inputBuffer?: number[];
    private _reader?;
    constructor(port: SerialPort, logger: Logger, _parent?: ESPLoader | undefined);
    private get _inputBuffer();
    initialize(): Promise<void>;
    /**
     * @name readLoop
     * Reads data from the input stream and places it in the inputBuffer
     */
    readLoop(): Promise<void>;
    sleep(ms?: number): Promise<unknown>;
    state_DTR: boolean;
    setRTS(state: boolean): Promise<void>;
    setDTR(state: boolean): Promise<void>;
    hardReset(bootloader?: boolean): Promise<void>;
    /**
     * @name macAddr
     * The MAC address burned into the OTP memory of the ESP chip
     */
    macAddr(): any[];
    readRegister(reg: number): Promise<number>;
    /**
     * @name checkCommand
     * Send a command packet, check that the command succeeded and
     * return a tuple with the value and data.
     * See the ESP Serial Protocol for more details on what value/data are
     */
    checkCommand(opcode: number, buffer: number[], checksum?: number, timeout?: number): Promise<[number, number[]]>;
    /**
     * @name sendCommand
     * Send a slip-encoded, checksummed command over the UART,
     * does not check response
     */
    sendCommand(opcode: number, buffer: number[], checksum?: number): Promise<void>;
    /**
     * @name readPacket
     * Generator to read SLIP packets from a serial port.
     * Yields one full SLIP packet at a time, raises exception on timeout or invalid data.
     * Designed to avoid too many calls to serial.read(1), which can bog
     * down on slow systems.
     */
    readPacket(timeout: number): Promise<number[]>;
    /**
     * @name getResponse
     * Read response data and decodes the slip packet, then parses
     * out the value/data and returns as a tuple of (value, data) where
     * each is a list of bytes
     */
    getResponse(opcode: number, timeout?: number): Promise<[number, number[]]>;
    /**
     * @name checksum
     * Calculate checksum of a blob, as it is defined by the ROM
     */
    checksum(data: number[], state?: number): number;
    setBaudrate(baud: number): Promise<void>;
    reconfigurePort(baud: number): Promise<void>;
    /**
     * @name sync
     * Put into ROM bootload mode & attempt to synchronize with the
     * ESP ROM bootloader, we will retry a few times
     */
    sync(): Promise<boolean>;
    /**
     * @name _sync
     * Perform a soft-sync using AT sync packets, does not perform
     * any hardware resetting
     */
    _sync(): Promise<boolean>;
    /**
     * @name getFlashWriteSize
     * Get the Flash write size based on the chip
     */
    getFlashWriteSize(): 1024 | 16384;
    /**
     * @name flashData
     * Program a full, uncompressed binary file into SPI Flash at
     *   a given offset. If an ESP32 and md5 string is passed in, will also
     *   verify memory. ESP8266 does not have checksum memory verification in
     *   ROM
     */
    flashData(binaryData: ArrayBuffer, updateProgress: (bytesWritten: number, totalBytes: number) => void, offset?: number, compress?: boolean): Promise<void>;
    /**
     * @name flashBlock
     * Send one block of data to program into SPI Flash memory
     */
    flashBlock(data: number[], seq: number, timeout?: number): Promise<void>;
    flashDeflBlock(data: number[], seq: number, timeout?: number): Promise<void>;
    /**
     * @name flashBegin
     * Prepare for flashing by attaching SPI chip and erasing the
     *   number of blocks requred.
     */
    flashBegin(size?: number, offset?: number, encrypted?: boolean): Promise<number>;
    /**
     * @name flashDeflBegin
     *
     */
    flashDeflBegin(size?: number, compressedSize?: number, offset?: number, encrypted?: boolean): Promise<number>;
    flashFinish(): Promise<void>;
    flashDeflFinish(): Promise<void>;
    getBootloaderOffset(): number;
    flashId(): Promise<number>;
    getChipFamily(): ChipFamily;
    writeRegister(address: number, value: number, mask?: number, delayUs?: number, delayAfterUs?: number): Promise<void>;
    setDataLengths(spiAddresses: SpiFlashAddresses, mosiBits: number, misoBits: number): Promise<void>;
    waitDone(spiCmdReg: number, spiCmdUsr: number): Promise<void>;
    runSpiFlashCommand(spiflashCommand: number, data: number[], readBits?: number): Promise<number>;
    detectFlashSize(): Promise<void>;
    /**
     * @name getEraseSize
     * Calculate an erase size given a specific size in bytes.
     *   Provides a workaround for the bootloader erase bug on ESP8266.
     */
    getEraseSize(offset: number, size: number): number;
    /**
     * @name memBegin (592)
     * Start downloading an application image to RAM
     */
    memBegin(size: number, blocks: number, blocksize: number, offset: number): Promise<[number, number[]]>;
    /**
     * @name memBlock (609)
     * Send a block of an image to RAM
     */
    memBlock(data: number[], seq: number): Promise<[number, number[]]>;
    /**
     * @name memFinish (615)
     * Leave download mode and run the application
     *
     * Sending ESP_MEM_END usually sends a correct response back, however sometimes
     * (with ROM loader) the executed code may reset the UART or change the baud rate
     * before the transmit FIFO is empty. So in these cases we set a short timeout and
     * ignore errors.
     */
    memFinish(entrypoint?: number): Promise<[number, number[]]>;
    runStub(): Promise<EspStubLoader>;
    writeToStream(data: number[]): Promise<void>;
    disconnect(): Promise<void>;
}
declare class EspStubLoader extends ESPLoader {
    IS_STUB: boolean;
    /**
     * @name memBegin (592)
     * Start downloading an application image to RAM
     */
    memBegin(size: number, blocks: number, blocksize: number, offset: number): Promise<any>;
    /**
     * @name getEraseSize
     * depending on flash chip model the erase may take this long (maybe longer!)
     */
    eraseFlash(): Promise<void>;
}
export {};
