# esp32_addon_bootloader
Bootloader code for updating the esp32_mouse_keyboard firmware via FABI/FLipMouse

Because we have to frequently change the Bluetooth code from the esp32_mouse_keyboard repository AND there is not enough space to keep 2 versions of this code, we need a small bootloader (this repo) for flashing a new image.

## Notes on esp32miniBT & Arduino Nano RP2040 Connect

Because this firmware is used for updating the ESP32 code on our own [esp32miniBT](https://github.com/asterics/esp32_mouse_keyboard) board (addon to FLipMouse, FLipPad and FABI) as well as on the Arduino Nano RP2040 Connect, there are some differences in the setup:

* The _partitions.csv_ is different, because a WROOM32 has 4MB flash, the uBlox Nina module on the Arduino only 2MB. If you need more space for your program, you can adjust the partitions.csv file to have a bigger ota partition.
* Pinning: we use GPIO27 for the red LED on the Arduino board, on the esp32miniBT it is GPIO5 in reversed polarity.
* Arduino: please load the esp32_addon_bootloader.ino file on the Arduino and connect __D2__ with __GND__ to reset the ESP32 into download mode. Then proceed with the Setup.
* ESP32-logging: sdkconfig.default disables all the esp-idf logging by default. If necessary, change in menuconfig.

Please select in `idf.py menuconfig` which board is used!

## Setup

This firmware is flashed as usual, with a serial-to-usb converter and the idf.py command:
```idf.py -b 115200 -p /dev/ttyUSB0 build flash```

(Baudrate can be higher, but the Arduino sketch for serial passthrough is setup to 115k2)

Afterwards, the esp32 addon should be mounted on a FLipMouse/FABI device. This device should support the `AT UG` command.

## Procedure

1. Open the FLipMouse / FABI serial port
2. If necessary, check if the communication works with `AT` (should return OK)
3. Send the `AT UG` command. This command sends the `$UG` command to the esp32_mouse_keyboard firmware, which marks this factory partition for boot and restarts.
5. The device switches to transparent UART mode (all data from the USB port is sent to the esp32 addon and vice versa) and changes the baud rate to 500kBaud/s
6. If there was already a flashed esp32_mouse_keyboard image, you should see OTA:starting; if not, OTA:write-error is received (this bootloader does not handle the $UG command)
7. Wait until `OTA:ready` is received, otherwise the update won't work.
8. Send the binary image to the USB serial port (it is possible to use the `update.py` script)
9. The firmware will be transmitted.
10. If successful, you should see the esp32_mouse_keyboard running and a status message `OTA:$FINISHED` (still with 500kBaud)
11. FLipMouse/FABI switches back automatically to 9600Baud/s and returns to normal functional state.

## update.py

Python 3 script for simply sending the binary image to a UART interface.
Please install `pyserial` library.
Usage:
```python3 update.py <COM port> <path to binary image>```

# Acknowledgements

Thanks to Espressif for the native_ota example which was used for this project.
Testing and adopting was done by Junaid Akhter Khan and Benjamin Aigner
