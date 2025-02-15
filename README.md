
# NF jammmotors ESP32 motor controller

Controls multiple motors with several protocols remotely or by simple preset motion generators.


Supported motor drivers:
- Stepper motors ( step/dir interface)
- RC servo motors ( PPM interface )
- DC motors, LEDs etc. ( PWM interface )
- Xiomi CyberGear motors ( CAN bus interface )

Supported input protocols:
- Web UI ( HTTP / WebSocket interface )
- OSC ( MIDI CC and MIDI notes over OSC UDP interface )
- DMX (ArtNet over UDP interface )

Integrated motion control: 
- oscillatory movement generator
- random position movement generator
- inverse kinematic ( fixed configuration for 3 axis arm ) 

## build 

Using platformio:

`pio run -t upload`  # download dependencies, build and upload to ESP32 via USB

## connect hardware

Connect motor drivers to the following pins:
- Stepper motor to step:16 dir:17, enable:21, alarm:22 - ! TODO configured in code, no configuration in web interface possible as for now !
- RC servo motor - any pin, configure in Web interface.
- DC motor / LEDs - any pin, configure in Web interface.
- Xiaomi CyberGear motor - connect CAN interface chip to RX pin 4, TX pin 5. CAN ID configurable as "pin" in Web interface - usual factory setting on motor is 127.


## credits

Libraries used (from PlatformIO registry):
- ESP32 Arduino framework 
- ESP Async WebServer for providing the Web UI and WebSocket interface
- FastAccelStepper for smooth stepper control
- ESP32Servo for RC servo control
- ArtnetnodeWifi for ArtNet DMX interface
- MicroOSC for providing OSC interface
- littlefs for permanent settings and web app storage
- Xiaomi_CyberGear_Arduino for CyberGear motor control, patched and shipped within this Repo.


