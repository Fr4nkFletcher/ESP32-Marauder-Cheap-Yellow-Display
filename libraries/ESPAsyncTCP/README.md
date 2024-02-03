# ESPAsyncTCP 

### Async TCP Library for ESP8266 Arduino

For ESP32 look [HERE](https://github.com/dvarrel/AsyncTCP)

This is a fully asynchronous TCP library, aimed at enabling trouble-free, multi-connection network environment for Espressif's ESP8266 MCUs.

This library is the base for [ESPAsyncWebSrv](https://github.com/dvarrel/ESPAsyncWebSrv)

## AsyncClient and AsyncServer
The base classes on which everything else is built. They expose all possible scenarios, but are really raw and require more skills to use.

## AsyncPrinter
This class can be used to send data like any other ```Print``` interface (```Serial``` for example).
The object then can be used outside of the Async callbacks (the loop) and receive asynchronously data using ```onData```. The object can be checked if the underlying ```AsyncClient```is connected, or hook to the ```onDisconnect``` callback.

## AsyncTCPbuffer
This class is really similar to the ```AsyncPrinter```, but it differs in the fact that it can buffer some of the incoming data.

## SyncClient
It is exactly what it sounds like. This is a standard, blocking TCP Client, similar to the one included in ```ESP8266WiFi```

## Libraries and projects that use AsyncTCP
- [ESP Async Web Server](https://github.com/dvarrel/ESPAsyncWebSrv)
