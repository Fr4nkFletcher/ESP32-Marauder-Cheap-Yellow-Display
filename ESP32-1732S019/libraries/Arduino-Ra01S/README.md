# Arduino-LoRa-Ra01S
This is an Arduino library for LoRa Communication using the radio transceiver chips [SX1262](https://www.semtech.com/products/wireless-rf/lora-core/sx1262) and [SX1268](https://www.semtech.com/products/wireless-rf/lora-core/sx1268).   

![ra01s_ra01sh](https://user-images.githubusercontent.com/6020549/161641357-a0fe292b-095e-440b-b8ae-24c58084a51d.JPG)


Ai-Thinker offers several LoRa modules.   
You can get these on AliExpress and eBay.   

|Model|Type|Interface/Core|Chip|Frequency|Foot-Pattern|IPEX-Antena|LoRa-WAN|
|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
|Ra-01|Tranceiver|SPI|SX1278|410-525Mhz|SMD16|No|No|
|Ra-02|Tranceiver|SPI|SX1278|410-525Mhz|SMD16|Yes|No|
|Ra-01H|Tranceiver|SPI|SX1276|803-930Mhz|SMD16|No|No|
|Ra-01S|Tranceiver|SPI|**SX1268**|410-525Mhz|SMD16|No|No|
|Ra-01SH|Tranceiver|SPI|**SX1262**|803-930Mhz|SMD16|Yes|No|
|Ra-01SC|Tranceiver|SPI|**LLCC68**|410-525Mhz|SMD16|Yes|No|
|Ra-06|MCU|ARM Cortex M0+|SX1278|410-525Mhz|SMD20|Yes|No|
|Ra-07|MCU|ARM Cortex M0+|ASR6501|410-525Mhz|SMD18|No|Yes|
|Ra-07H|MCU|ARM Cortex M0+|ASR6501|803-930Mhz|SMD18|No|Yes|
|Ra-08|MCU|ARM Cortex M4|ASR6601|410-525Mhz|SMD18|No|Yes|
|Ra-08H|MCU|ARM Cortex M4|ASR6601|803-930Mhz|SMD18|No|Yes|


# Option with SX1262/1268
LoRa modules with SX1262/1268 have several options.   

- Using TCXO(Temperature-Compensated Crystal Oscillator)   
SX1262/1268 can use the TCXO.   
If the TCXO is used, the XTB pin is not connected.   
However, the 6th pin (DIO3) of the SX1262/1268 can be used to power the TCXO.   
Explanation for TXCO and antenna control is [here](https://github.com/beegee-tokyo/SX126x-Arduino).   
Ra-01S / Ra-01SH does not use TCXO.   

- Power supply modes   
SX1262/1268 has two power supply modes.   
One is that only LDO used in all modes.   
Another is that DC_DC+LDO used for STBY_XOSC,FS, RX and TX modes.   
Explanation for LDO and DCDC selection is [here](https://github.com/beegee-tokyo/SX126x-Arduino).   
Ra-01S / Ra-01SH use only LDO in all modes.

- RF-Switching   
In general, use DIO2 to switch the RF-Switch.   
However, some tranceiver use an external gpio to switch the RF-Switch.   
Ra-01S / Ra-01SH use the SC70-6 integrated load switch to switch between RFO and RFI.   
Ra-01S / Ra-01SH use DIO2 to control this.   
DIO2 = 1, CTRL = 0, RFC to RF1  Tx Mode.   
DIO2 = 0, CTRL = 1, RFC to R21  Rx Mode.  

You need to look at the schematic to set these options properly, but it's very esoteric.   
The default settings for this library are for Ra-01S / Ra-01SH.    
__When using other than Ra-01S / Ra-01SH, you need to set them appropriately.__   

I created this library based on [this](https://github.com/tinytronix/SX126x).   
With this library, Ra-01S / Ra-01SH doesn't work.   

# About Ra-01SC   
Ra-01SC uses LLCC68.   
Ra-01SC is compatible with Ra-01S.   
However, there are the following restrictions:   
- BW is either 125KHz, 250KHz or 500Khz.   
- When BW is 125KHz, SF is in the range of 5-9.   
- When BW is 250KHz, SF is in the range of 5-10.   
- When BW is 500KHz, SF is in the range of 5-11.   

![ra01sc](https://user-images.githubusercontent.com/6020549/169180199-12a88938-1d6d-43c7-836d-f3c2081ac10d.JPG)

# Datasheet   
- RA-01S   
https://docs.ai-thinker.com/_media/lora/docs/ra-01s_specification.pdf

- RA-01SH   
https://docs.ai-thinker.com/_media/lora/docs/ra-01sh_specification.pdf

- RA-01SC(Chinese)   
https://img.iceasy.com/product/product/files/202108/8a8a8a1a7aec7b55017b2ef70a370953.pdf

# Foot pattern
RA-0x(SMD16) has the same foot pattern as ESP12.   
Therefore, a pitch conversion PCB for ESP12 can be used.   

![ra01s-3](https://user-images.githubusercontent.com/6020549/161641874-32a79d5f-dbae-42f1-a8cd-d0787c238a06.JPG)
![ra01s-2](https://user-images.githubusercontent.com/6020549/161641421-e720a7da-4889-4bd4-b2c6-1f3a28518cf8.JPG)

# Installation
Download this repo as zip. Then in the Arduino IDE go to Sketch->Add library->add .zip library.   

# Wiring
|Ra-01S/SH||UNO|MEGA|ESP8266|
|:-:|:-:|:-:|:-:|:-:|
|VCC|--|3.3V(*1)|3.3V|3.3V|
|GND|--|GND|GND|GND|
|SCK|--|D13(*2)|D52(*2)|IO14|
|MISO|--|D12|D50|IO12|
|MOSI|--|D11(*2)|D51(*2)|IO13|
|NSS|--|D5(*2)|D5(*2)|IO2|
|RST|--|D6(*2)|D6(*2)|IO0|
|BUSY|--|D7(*2)|D7(*2)|IO16|
|TXEN|--|N/C|N/C|N/C|
|RXEN|--|N/C|N/C|N/C|


(*1)   
UNO's 3.3V output can only supply 50mA.   
In addition, the output current capacity of UNO-compatible devices is smaller than that of official products.   
__So this module may not work normally when supplied from the on-board 3v3.__   

(*2)   
SX126x is not 5V tolerant.   
You need level shift from 5V to 3.3V.   
I used [this](https://www.ti.com/lit/ds/symlink/txs0108e.pdf?ts=1647593549503) for a level shift.   

# Using EBYTE Module

EBYTE offers several LoRa modules.   
You can get these on AliExpress and eBay.   

|Model|Interface|Chip|Frequency|Power|Foot-Patten|IPEX-Antena|LoRa-WAN|
|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
|E22-400M22S|SPI|SX1268|433/470Mhz|160mW|Standard|Yes|No|
|E22-400M30S|SPI|SX1268|433/470Mhz|1000mW|Standard|Yes|No|
|E22-400MM22S|SPI|SX1268|433/470Mhz|160mW|Small|No|No|
|E22-900M22S|SPI|SX1262|868/915Mhz|160mW|Standard|Yes|No|
|E22-900M30S|SPI|SX1262|868/915Mhz|1000mW|Standard|Yes|No|
|E22-900MM22S|SPI|SX1262|868/915Mhz|160mW|Small|No|No|

![EBYTE-1](https://user-images.githubusercontent.com/6020549/221339540-4330b34d-1aea-4911-87c5-607a5a3d4b57.JPG)
![EBYTE-2](https://user-images.githubusercontent.com/6020549/221339539-40fc9e6f-9224-4b0c-b222-efa5d9850560.JPG)

With this change it work.   


```
/*
SX126x  lora(5,               //Port-Pin Output: SPI select
             6,               //Port-Pin Output: Reset 
             7                //Port-Pin Input:  Busy
             );

int16_t ret = lora.begin(RF_FREQUENCY,              //frequency in Hz
                         TX_OUTPUT_POWER);          //tx power in dBm
*/

SX126x  lora(5,               //Port-Pin Output: SPI select
             6,               //Port-Pin Output: Reset 
             7                //Port-Pin Input:  Busy
             8                //Port-Pin Output: TXEN
             9                //Port-Pin Output: RXEN
             );

int16_t ret = lora.begin(RF_FREQUENCY,              //frequency in Hz
                         TX_OUTPUT_POWER,           //tx power in dBm
                         3.3,                       //use TCXO
                         true);                     //use TCXO
```


Two additional wires are required.   
|EBYTE||UNO|MEGA|ESP8266|
|:-:|:-:|:-:|:-:|:-:|
|TXEN|--|8(*3)|8(*3)|D4|
|RXEN|--|9(*3)|9(*3)|D5|

(*3)   
SX126x is not 5V tolerant.   
You need level shift from 5V to 3.3V.   

The pitch conversion base is [here](https://github.com/nopnop2002/esp-idf-sx126x/tree/main/ebyte-smd-pcb).   

SX1262 and LLCC68 are compatible, but for some reason they don't work.   
|Model|Interface|Chip|Frequency|Power|Foot-Patten|IPEX-Antena|LoRa-WAN|
|:-:|:-:|:-:|:-:|:-:|:-:|:-:|:-:|
|E220-400M22S|SPI|LLCC68|433/470Mhz|160mW|Standard|Yes|No|
|E220-400M30S|SPI|LLCC68|433/470Mhz|1000mW|Standard|Yes|No|
|E220-900M22S|SPI|LLCC68|868/915Mhz|160mW|Standard|Yes|No|
|E220-900M30S|SPI|LLCC68|868/915Mhz|1000mW|Standard|Yes|No|

# Software compatibility
This library can communicate with [RadioLib](https://github.com/jgromes/RadioLib).   
RadioLib require DIO1 connected in order to works.   

```
  // Set frequency: 866Mhz
  // Set bandwidth(BW): 125Khz
  // Set Spreading Factor(SF): 7
  // Set Error Cording Rate(CR): 4/5
  // Set SyncWord: 0x1424(Private Network)
  // Set Power: 10dBm
  // Set Preamble Length: 8
  // Configure the radio to NOT use a TCXO controlled by DIO3
  // Set regulator mode: DC-DC
  int state = radio.begin(866.0, 125.0, 7, 5, RADIOLIB_SX126X_SYNC_WORD_PRIVATE, 10, 8, 0.0, false);
```


# Limitation
- The SX126x chip implements FSK, but FSK is not supported in this library.   
- Interrupts is not supported in this library.   



# SX1262 and SX1278, SX1276 Comparison
[This](https://www.cdebyte.com/news/580) will be helpful.   



# Build ESP8266 with PlatformIO
```
$ git clone https://github.com/nopnop2002/Arduino-LoRa-Ra01S

$ cd Arduino-LoRa-Ra01S/example/Ra01S-RX/

$ pio init -b d1_mini

$ cp Ra01S-RX.ino src/

$ vi src/Ra01S-RX.ino
Disable AtMega
Enable ESP8266

$ vi platform.ini
[env:d1_mini]
platform = espressif8266
board = d1_mini
framework = arduino
lib_deps = https://github.com/nopnop2002/Arduino-LoRa-Ra01S --> Add this line

$ pio run -t upload && pio device monitor -b 115200
```

