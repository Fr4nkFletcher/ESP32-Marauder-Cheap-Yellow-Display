// Adafruit Internal Flash FatFs Format Example
// Author: Tony DiCola, Ha Thach
//
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// !!  NOTE: YOU WILL ERASE ALL DATA BY RUNNING THIS SKETCH!  !!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//
// Usage:
// - Modify the pins and type of fatfs object in the config
//   section below if necessary (usually not necessary).
// - Upload this sketch to your M0 express board.
// - Open the serial monitor at 115200 baud.  You should see a
//   prompt to confirm formatting.  If you don't see the prompt
//   close the serial monitor, press the board reset buttton,
//   wait a few seconds, then open the serial monitor again.
// - Type OK and enter to confirm the format when prompted.
// - Partitioning and formatting will take about 30-60 seconds.
//   Once formatted a message will be printed to notify you that
//   it is finished.
//
#include <SPI.h>
#include "SdFat.h"
#include "Adafruit_InternalFlash.h"

// Since SdFat doesn't fully support FAT12 such as format a new flash
// We will use Elm Cham's fatfs f_mkfs() to format. Note:
// - FM_FAT: create with MBR (63 sectors). For CircuitPython compatibility, this is preferred since CPY simulate MBR.
// However, disk size maybe need to be at least 192 sector (96KB)
// - FM_FAT | FM_SFD: create without MBR, save 63 sectors. Disk size can be 128 sector (64KB).
#include "ff.h"
#include "diskio.h"

// up to 11 characters
#define DISK_LABEL    "EXT FLASH"

// Start address and size should matches value in the CircuitPython (INTERNAL_FLASH_FILESYSTEM = 1)
// to make it easier to switch between Arduino and CircuitPython
#define INTERNAL_FLASH_FILESYSTEM_START_ADDR  (0x00040000 - 256 - 0 - INTERNAL_FLASH_FILESYSTEM_SIZE)
#define INTERNAL_FLASH_FILESYSTEM_SIZE        (64*1024)

Adafruit_InternalFlash flash(INTERNAL_FLASH_FILESYSTEM_START_ADDR, INTERNAL_FLASH_FILESYSTEM_SIZE);

// file system object from SdFat
FatFileSystem fatfs;

// Elm Cham's fatfs objects
FATFS elmchamFatfs;
uint8_t workbuf[4096]; // Working buffer for f_fdisk function.
  
void setup() {
  // Initialize serial port and wait for it to open before continuing.
  Serial.begin(115200);
  while (!Serial) delay(100);
  
  Serial.println("Adafruit Internal Flash FatFs Format Example");

  // Initialize flash library and check its chip ID.
  if (!flash.begin()) {
    Serial.println("Error, failed to initialize flash chip!");
    while(1) yield();
  }

  // Wait for user to send OK to continue.
  Serial.setTimeout(30000);  // Increase timeout to print message less frequently.
  do {
    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    Serial.println("This sketch will ERASE ALL DATA on the flash chip and format it with a new filesystem!");
    Serial.println("Type OK (all caps) and press enter to continue.");
    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  } while ( !Serial.find((char*) "OK"));

  // Call fatfs begin and passed flash object to initialize file system
  Serial.println("Creating and formatting FAT filesystem (this takes ~60 seconds)...");

  // Make filesystem: change FM_SFD (above note) to match your usage
  FRESULT r = f_mkfs("", FM_FAT | FM_SFD, 0, workbuf, sizeof(workbuf));
  if (r != FR_OK) {
    Serial.print("Error, f_mkfs failed with error code: "); Serial.println(r, DEC);
    while(1) yield();
  }

  // mount to set disk label
  r = f_mount(&elmchamFatfs, "0:", 1);
  if (r != FR_OK) {
    Serial.print("Error, f_mount failed with error code: "); Serial.println(r, DEC);
    while(1) yield();
  }

  // Setting label
  Serial.println("Setting disk label to: " DISK_LABEL);
  r = f_setlabel(DISK_LABEL);
  if (r != FR_OK) {
    Serial.print("Error, f_setlabel failed with error code: "); Serial.println(r, DEC);
    while(1) yield();
  }

  // unmount
  f_unmount("0:");

  // sync to make sure all data is written to flash
  flash.syncBlocks();
  
  Serial.println("Formatted flash!");

  // Check new filesystem
  if (!fatfs.begin(&flash)) {
    Serial.println("Error, failed to mount newly formatted filesystem!");
    while(1) delay(1);
  }

  // Done!
  Serial.println("Flash chip successfully formatted with new empty filesystem!");
}

void loop() {
  // Nothing to be done in the main loop.
}


//--------------------------------------------------------------------+
// fatfs diskio
//--------------------------------------------------------------------+
extern "C"
{

DSTATUS disk_status ( BYTE pdrv )
{
  (void) pdrv;
	return 0;
}

DSTATUS disk_initialize ( BYTE pdrv )
{
  (void) pdrv;
	return 0;
}

DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber to identify the drive */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Start sector in LBA */
	UINT count		/* Number of sectors to read */
)
{
  (void) pdrv;
	return flash.readBlocks(sector, buff, count) ? RES_OK : RES_ERROR;
}

DRESULT disk_write (
	BYTE pdrv,			/* Physical drive nmuber to identify the drive */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Start sector in LBA */
	UINT count			/* Number of sectors to write */
)
{
  (void) pdrv;
  return flash.writeBlocks(sector, buff, count) ? RES_OK : RES_ERROR;
}

DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
  (void) pdrv;

  switch ( cmd )
  {
    case CTRL_SYNC:
      flash.syncBlocks();
      return RES_OK;

    case GET_SECTOR_COUNT:
      *((DWORD*) buff) = flash.size()/512;
      return RES_OK;

    case GET_SECTOR_SIZE:
      *((WORD*) buff) = 512;
      return RES_OK;

    case GET_BLOCK_SIZE:
      *((DWORD*) buff) = 8;    // erase block size in units of sector size
      return RES_OK;

    default:
      return RES_PARERR;
  }
}

}
