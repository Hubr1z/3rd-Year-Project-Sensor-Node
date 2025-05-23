# mmWave Sensor Node Source Code

## Summary of the Code
This is 1 of the 2 source codes used in this mmWave for human crowdedness measurement 3rd year project. The source code in this repository is responsible for gathering data from the Seeed Studio XIAO 24GHz mmwave Human Static Presence Module to the microcontroller via the UART serial ports. The array of human value is passed through the rolling average and transmitted via Bluetooth Low Energy (BLE) to the BLE client which will process all the data coming in from the sensor nodes.

The source code currently accommodates for 2 sensor nodes each with their own UUID. Currently, the way to switch between which UUID to use and upload requires manual editting and is acknowledged that is prone to human error during switching but that is a problem for another day.

## How to Use
The mmWave source code is used on the ESP32C6 XIAO microcontroller using an arduino framework. This folder is self contained (assuming arduino framework used since it also includes the BLE libraries) and already imported libraries used are the XIAO mmWave libraries and the XIAO hardware serial libraries. A custom rolling average library is also in the library created by yours truly. It is recommended to use this folder on VSCode [PlatformIO](https://platformio.org) extension since it is the IDE used for this project complete with build and upload to the ESP32C6 microcontroller. Further guides on how to use PlaformIO with ESP32C6 found [here](https://wiki.seeedstudio.com/xiao_esp32c6_with_platform_io/). After setting up the IDE, just clone the repository to use.

Source code based on espressif arduino-esp32 BLE Server example code found [here](https://github.com/espressif/arduino-esp32/tree/master/libraries/BLE/examples/Server)
