# ESP_IDF_TouchDrv_Example

```
The current example only writes the CSTxxx series touch application. 
The other sensors have not been added, but the methods are the same.
```


### Configure the Project

Open the project configuration menu (`idf.py menuconfig`).


In the `SensorLib Example Configuration` menu:

* Select the PMU Type in the `SensorLib read and write methods` option，Choose according to the situation
    1. Implemented using built-in read and write methods (Read and write methods are provided internally by SensorLib, supporting high version esp-idf >= 5.0, and low version methods (< 5.0))
    2. Implemented using read and write callback methods (Implemented using externally provided methods, suitable for multiple platforms)
* In `Sensor SCL GPIO Num` select the clock pin to connect to the PMU,the default is 17
* In `Sensor SDA GPIO Num` select the data pin connected to the PMU,the default is 18
* Select the interrupt pin connected to the PMU in `Sensor Interrupt Pin`, the default is 16
* Select `Sensor reset Pin` , the default is 21
* `Master Frequency` The maximum communication frequency defaults to 100000HZ, and you can decide whether to change it according to the situation.

## How to Use Example

Before project configuration and build, be sure to set the correct chip target using `idf.py set-target <chip_name>`. default use **esp32s3**


### Build and Flash

Run `idf.py -p PORT flash monitor` to build, flash and monitor the project.

(To exit the serial monitor, type ``Ctrl-]``.)

See the [Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html) for full steps to configure and use ESP-IDF to build projects.

## Example Output

The output information is to configure the output voltage and enable status of the PMU

```
ESP-ROM:esp32s3-20210327
Build:Mar 27 2021
rst:0x15 (USB_UART_CHIP_RESET),boot:0xa (SPI_FAST_FLASH_BOOT)
Saved PC:0x403788f6
SPIWP:0xee
mode:DIO, clock div:1
load:0x3fce3810,len:0x178c
load:0x403c9700,len:0x4
load:0x403c9704,len:0xbfc
load:0x403cc700,len:0x2d64
entry 0x403c9900
I (26) boot: ESP-IDF v5.3-dev-1288-g5524b692ee 2nd stage bootloader
I (27) boot: compile time Jan 10 2024 22:49:17
I (27) boot: Multicore bootloader
I (31) boot: chip revision: v0.2
I (35) boot.esp32s3: Boot SPI Speed : 80MHz
I (40) boot.esp32s3: SPI Mode       : DIO
I (45) boot.esp32s3: SPI Flash Size : 2MB
I (49) boot: Enabling RNG early entropy source...
I (55) boot: Partition Table:
I (58) boot: ## Label            Usage          Type ST Offset   Length
I (66) boot:  0 nvs              WiFi data        01 02 00009000 00006000
I (73) boot:  1 phy_init         RF data          01 01 0000f000 00001000
I (81) boot:  2 factory          factory app      00 00 00010000 00100000
I (88) boot: End of partition table
I (92) esp_image: segment 0: paddr=00010020 vaddr=3c020020 size=0e0dch ( 57564) map
I (111) esp_image: segment 1: paddr=0001e104 vaddr=3fc93100 size=01f14h (  7956) load
I (113) esp_image: segment 2: paddr=00020020 vaddr=42000020 size=1acb0h (109744) map
I (137) esp_image: segment 3: paddr=0003acd8 vaddr=3fc95014 size=00994h (  2452) load
I (138) esp_image: segment 4: paddr=0003b674 vaddr=40374000 size=0f0e0h ( 61664) load
I (163) boot: Loaded app from partition at offset 0x10000
I (163) boot: Disabling RNG early entropy source...
I (163) cpu_start: Multicore app
I (176) cpu_start: Pro cpu start user code
I (176) cpu_start: cpu freq: 240000000 Hz
I (177) cpu_start: Application information:
I (177) cpu_start: Project name:     ESP_IDF_TouchDrv_Example
I (177) cpu_start: App version:      v0.1.4-6-gfeaf675-dirty
I (177) cpu_start: Compile time:     Jan 11 2024 14:01:31
I (177) cpu_start: ELF file SHA256:  3ce1527fc...
I (178) cpu_start: ESP-IDF:          v5.3-dev-1288-g5524b692ee
I (178) cpu_start: Min chip rev:     v0.0
I (178) cpu_start: Max chip rev:     v0.99 
I (178) cpu_start: Chip rev:         v0.2
I (178) heap_init: Initializing. RAM available for dynamic allocation:
I (179) heap_init: At 3FC96290 len 00053480 (333 KiB): RAM
I (179) heap_init: At 3FCE9710 len 00005724 (21 KiB): RAM
I (179) heap_init: At 3FCF0000 len 00008000 (32 KiB): DRAM
I (180) heap_init: At 600FE010 len 00001FD8 (7 KiB): RTCRAM
I (181) spi_flash: detected chip: winbond
I (181) spi_flash: flash io: dio
W (181) spi_flash: Detected size(16384k) larger than the size in the binary image header(2048k). Using the size in the binary image header.
W (181) i2c: This driver is an old driver, please migrate your application code to adapt `driver/i2c_master.h`
I (182) sleep: Configure to isolate all GPIO pins in sleep state
I (182) sleep: Enable automatic switching of GPIO sleep configuration
I (183) main_task: Started on CPU0
I (193) main_task: Calling app_main()
I (193) gpio: GPIO[16]| InputEn: 1| OutputEn: 0| OpenDrain: 0| Pullup: 1| Pulldown: 0| Intr:2 
I (193) I2C: Implemented using read and write callback methods (Use lower version < 5.0 API)
I (193) mian: I2C initialized successfully
I (193) TOUCH: ----touch_drv_init----
0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
10: -- -- -- -- -- 15 -- -- -- -- -- -- -- -- -- --
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
70: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
I (303) TOUCH: Find touch address 0x15
I (303) TOUCH: Find touch address 0x15
I (303) TOUCH: Implemented using built-in read and write methods (Use lower version < 5.0 API)
Using ESP-IDF Driver interface.
E (303) i2c: i2c driver install error
I (303) gpio: GPIO[21]| InputEn: 0| OutputEn: 1| OpenDrain: 0| Pullup: 0| Pulldown: 0| Intr:0 
I (303) gpio: GPIO[21]| InputEn: 0| OutputEn: 1| OpenDrain: 0| Pullup: 0| Pulldown: 0| Intr:0 
Chip ID:0xb5
Version :0x2
Touch type:CST816T
I (383) TOUCH: Init touch SUCCESS!
I (383) TOUCH: Some CST816 will automatically enter sleep,can use the touch.disableAutoSleep() to turn off
I (383) gpio: GPIO[21]| InputEn: 0| OutputEn: 1| OpenDrain: 0| Pullup: 0| Pulldown: 0| Intr:0 
I (513) mian: Run...
I (513) main_task: Returned from app_main()
X[0]:143  Y[0]:127 
X[0]:143  Y[0]:127
X[0]:142  Y[0]:127
X[0]:141  Y[0]:126
X[0]:138  Y[0]:125 
X[0]:135  Y[0]:123
X[0]:131  Y[0]:121
X[0]:126  Y[0]:119
X[0]:121  Y[0]:117
X[0]:84  Y[0]:146
X[0]:85  Y[0]:146
X[0]:86  Y[0]:145
X[0]:87  Y[0]:143
X[0]:88  Y[0]:140 
X[0]:89  Y[0]:136
X[0]:90  Y[0]:132
X[0]:91  Y[0]:127
X[0]:92  Y[0]:123
X[0]:93  Y[0]:119
X[0]:94  Y[0]:115
X[0]:95  Y[0]:111
I (513563) TOUCH: Touch Home button pressed!
I (513573) TOUCH: Touch Home button pressed!
I (513583) TOUCH: Touch Home button pressed!
I (513603) TOUCH: Touch Home button pressed!
I (513613) TOUCH: Touch Home button pressed!
I (513633) TOUCH: Touch Home button pressed!
I (513643) TOUCH: Touch Home button pressed!
```

## Build process example

Assuming you don't have esp-idf yet

```
mkdir -p ~/esp
cd ~/esp
git clone --recursive https://github.com/espressif/esp-idf.git
git clone https://github.com/lewisxhe/XPowersLib.git
cd esp-idf
./install.sh
. ./export.sh
cd ..
cd SensorLib/examples/ESP_IDF_Example

Configure SDA,SCL,INT,RST,Pin
...

idf.py menuconfig
idf.py build
idf.py -b 921600 flash
idf.py monitor

```