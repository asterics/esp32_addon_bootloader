# esp32_addon_bootloader
Bootloader code for updating the esp32_mouse_keyboard firmware via FABI/FLipMouse

Because we have to frequently change the Bluetooth code from the esp32_mouse_keyboard repository AND there is not enough space to keep 2 versions of this code, we need a small bootloader (this repo) for flashing a new image.

## Setup

This firmware is flashed as usual, with a serial-to-usb converter and the idf.py command:
```idf.py -p /dev/ttyUSB0 build flash```

Afterwards, the esp32 addon should be mounted on a FLipMouse/FABI device. This device should support the `AT UG` command.

## Procedure

1. Open the FLipMouse / FABI serial port
2. If necessary, check if the communication works with `AT` (should return OK)
3. Send the `AT UG` command. This command sends the `$UG` command to the esp32_mouse_keyboard firmware, which marks this factory partition for boot and restarts.
4. The device switches to transparent UART mode (all data from the USB port is sent to the esp32 addon and vice versa) and changes the baud rate to 500kBaud/s
5. Send the binary image to the USB serial port (it is possible to use the `update.py` script)
6. The firmware will be transmitted.
7. If successful, you should see the esp32_mouse_keyboard running and a status message `OTA:finished` (still with 500kBaud)
8. TODO: FLipMouse/FABI currently (19.4.2021) do NOT switch back automatically to 9600Baud/s

## update.py

Python 3 script for simply sending the binary image to a UART interface.
Please install `pyserial` library.
Usage:
```python3 update.py <COM port> <path to binary image>```

# Acknowledgements

Thanks to Espressif for the native_ota example which was used for this project.
Testing and adopting was done by Junaid Akhter Khan and Benjamin Aigner
