# ESP32-Guitar-MIDI-Controller
Code for ESP32-C3 SuperMini to turn it into a midi controller that sends midi commands via Bluetooth BLE and serial port (USB) compatible with linux ttymidi.
Functions:
- 9 push buttons (pull up) that send midi commands via the serial port (USB) and BLE when pressed (GPIO 0,1,2,3,5,6,7,9,10)
- 50ms debounce to prevent noise when pressing the buttons from generating spurious midi commands.
- 1 5k ohm fader for sending midi CC messages (GPIO 4)
- 1 10k Ohm expression pedal for sending midi CC messages (GPIO 20)
- Red LED lights up when turned on
- Blue LED built into the board (GPIO 8) flashes when the device is waiting for a BLE connection
- Blue LED stays on permanently when the BLE connection occurs
- Blue LED flashes when a midi command is sent
- When pressing the buttons connected to GPIO 2 and 3 for 2 seconds, BLE is disabled and the blue LED turns off. When pressed again, BLE turns on and returns to "waiting for connection" mode.

Libraries versions:
- ArduinoBLE v1.3.3
- NimBLE v1.4.3
- Esp32-BLE-midi de maxime v0.3.2

Board core version:
- ESP32 by Espressif Systems v2.0.18

TTYMIDI
- Download the ttymidi code
- unzip the contents
- In Linux terminal enter the folder and type:
   make
- Then type:
  sudo make install
- Finally, type the command (check if your device is in serial port /dev/ttyACM0 or another):
  ttymidi -s /dev/ttyACM0 -b 31250 -v
  
