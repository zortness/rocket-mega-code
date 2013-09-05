RocketMegaChibiAVR
==================

**Warning**
This is currently an unfinished piece of software. Do not try to fly with this!

Software for the [RocketMega R2](https://github.com/zortness/rocket-mega-shield) that functions
as a dual-deployment altimeter with telemetry and data logging.


Building
------------
This project can be built with the [Arduino IDE](http://arduino.cc/en/main/software), 
or something like [CodeBlocks for Arduino](http://www.arduinodev.com/codeblocks/).

This project currently depends on a few other open source libraries:
* [ChibiOS/RT for AVR](https://code.google.com/p/rtoslibs/)
* [Adafruit GPS Library](https://github.com/adafruit/Adafruit-GPS-Library)
* [Adafruit Unified Sensor Driver](https://github.com/adafruit/Adafruit_Sensor)
* [Adafruit BMP085 Library](https://github.com/adafruit/Adafruit-BMP085-Library)
* [Adafruit LSM303](https://github.com/adafruit/Adafruit_LSM303)
* [Adafruit L3GD20](https://github.com/adafruit/Adafruit_L3GD20)

The above libraries will have to be added to your Libraries folder in the Arduino IDE, 
or included in the compilation path for other IDEs.