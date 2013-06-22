KalmanSingleState
=================

A simple Kalman filter for a single sensor value.


Building
------------
This project can be built with the [Arduino IDE](http://arduino.cc/en/main/software), 
or something like [CodeBlocks for Arduino](http://www.arduinodev.com/codeblocks/).

This library will have to be added to your Libraries folder in the Arduino IDE, 
or included in the compilation path for other IDEs.

```c
#include: <KalmanSingleState.h>
//...

KalmanSingleState myFilter;
double rawValue = sensor.read();
myFilter.init(0.20001, 4.0001, 25.0, rawValue);
//...

myFilter.update(sensor.read());
Serial.print("Value:");
Serial.println(myFilter.getValue());
```
