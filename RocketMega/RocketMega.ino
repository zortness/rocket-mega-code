/**
* RocketMega Software
* This is a basic dual-deployment rocketry software package for the
* rocket-mega-board project and an Arduino Mega 2560 compatible board.
*/

/*
Copyright (c) 2013 Kurtis Kopf

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

#include <SoftwareSerial.h>
#include <Adafruit_GPS.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_L3GD20.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_LSM303.h>
#include "RocketMegaConstants.h"
#include <KalmanSingleState.h>

// prototypes
void error(uint8_t errno);
void readAccel();
void readAlt();
void readGyro();
void readLowAccel();
void logStatus();
void logStats();
void slowIntervals();
void fastIntervals();
void modeReady();
void modeAscent();
void modeApogee();
void modeDescent();
void modeDescent2();
void modeTouchdown();
void deployMain();
void deployDrogue();
void modePause();
void replay();
void startReplay(String fileName);

// variables
uint8_t mode = MODE_STARTUP;
uint8_t altInterval = ALTIMETER_INTERVAL_SLOW;
uint8_t gyroInterval = GYRO_INTERVAL_SLOW;
uint8_t logInterval = LOG_INTERVAL_SLOW;
uint8_t lowAccelInterval = LOW_ACCEL_INTERVAL_SLOW;
uint8_t highAccelInterval = HIGH_ACCEL_INTERVAL_SLOW;

Adafruit_GPS GPS(&Serial1);
File logFile;
String buffer = "";
String buffer1 = "";
boolean logSd = false;
boolean logRawGps = true;
Adafruit_L3GD20 gyro;
Adafruit_BMP085 bmp = Adafruit_BMP085(10085);
Adafruit_LSM303_Mag mag = Adafruit_LSM303_Mag(12345);
Adafruit_LSM303_Accel accel = Adafruit_LSM303_Accel(54321);
float sealevelPressure = DEFAULT_SEALEVEL_PRESSURE;
float startingAltitude = 0;
float maxAltitude = 0;
int ascentThreshold = DEFAULT_ASCENT_THRESHOLD;
int apogeeThreshold = DEFAULT_APOGEE_THRESHOLD;
int touchdownThreshold = DEFAULT_TOUCHDOWN_THRESHOLD;
float mainDeployAltitude = DEFAULT_MAIN_ALTITUDE;
unsigned long launchTime = 0;
unsigned long apogeeTime = 0;
unsigned long mainTime = 0;
unsigned long touchdownTime = 0;
unsigned long maxForce = 0;
float minTemperature = 0;
float maxTemperature = 0;
float minPressure = DEFAULT_SEALEVEL_PRESSURE; // we set this to the first reading anyway, but just to be safe
float maxPressure = 0;

// used for tracking times that relays are activated
unsigned long drogueDeployStart = 0;
unsigned long mainDeployStart = 0;

int xPin = ACCEL_X_PIN;
int yPin = ACCEL_Y_PIN;
boolean ledOn = false;
uint32_t counter = 0;
uint32_t printCount = 0;

// sensor values
KalmanSingleState xForce;
KalmanSingleState yForce;
KalmanSingleState altitude;
KalmanSingleState temperature;
KalmanSingleState pressure;
KalmanSingleState xRotation;
KalmanSingleState yRotation;
KalmanSingleState zRotation;
KalmanSingleState lxForce;
KalmanSingleState lyForce;
KalmanSingleState lzForce;
KalmanSingleState xMag;
KalmanSingleState yMag;
KalmanSingleState zMag;
KalmanSingleState orientation;

float latitude = 0;
float longitude = 0;

// replay settings
bool inReplay = false;
File replayFile;
unsigned long replayTimeOffset = 0;

/**
* Arduino setup.
* - Determine if we have an SD card available for logging
* - Initialize sensors, read each of them a few times to get a baseline for the Kalman filters
* - Initialize serial lines for GPS and XBee (main)
*/
void setup()
{
	pinMode(LED_PIN, OUTPUT);
	pinMode(RELAY_1_PIN, OUTPUT);
	pinMode(RELAY_2_PIN, OUTPUT);
	pinMode(SD_CHIP_SELECT, OUTPUT);
	Serial.begin(SERIAL_BAUD);

	Serial.println("Setting up SD Logging...");
	if (!SD.begin(SD_CHIP_SELECT))
	{
		Serial.println("ERROR: SD Card init failed!");
		logSd = false;
		//error(2);
	}
	else
	{
		logSd = true;
		Serial.println("Creating log file...");
		char filename[15];
		strcpy(filename, "ROCKET00.TXT");
		for (uint8_t i = 0; i < 100; i++)
		{
			filename[6] = '0' + i/10;
			filename[7] = '0' + i%10;
			// create if does not exist, do not open existing, write, sync after write
			if (! SD.exists(filename))
			{
				break;
			}
		}
		logFile = SD.open(filename, FILE_WRITE);
		if( ! logFile )
		{
			Serial.print("Couldnt create ");
			Serial.println(filename);
			logSd = false;
			//error(3);
		}
		else
		{
			Serial.print("Writing to ");
			Serial.println(filename);
			Serial.println("SD setup complete.");
		}
	}

	Serial.println("Setting up High-G Accelerometer...");
	for(int i = 0; i < 5; i++)
    {
        readAccel();
        delay(100);
    }
    Serial.print("x:"); Serial.print(xForce.getValue());
    Serial.print(", y:"); Serial.println(yForce.getValue());

    Serial.println("Setting up Altimeter...");
	if (!bmp.begin(BMP085_MODE_STANDARD))
	{
		Serial.println("Unable to initialize altimeter");
		error(5);
	}
	for (int i = 0; i < 5; i++)
    {
        readAlt();
        delay(100);
    }
	startingAltitude = altitude.getValue();
	minPressure = pressure.getValue();
	Serial.print("Starting altitude initially set at "); Serial.print(startingAltitude); Serial.println("m...");
	Serial.print("Starting temperature: "); Serial.print(temperature.getValue()); Serial.println('C');
	Serial.print("Starting pressure: "); Serial.print(pressure.getValue()); Serial.println("hPa");

	Serial.println("Setting up Low-G Accelerometer...");
	if (!mag.begin())
    {
        Serial.println("Unable to initialize Magnetometer");
		error(6);
    }
    if (!accel.begin())
    {
        Serial.println("Unable to initialize Low-G Accelerometer");
		error(6);
    }
    for (int i = 0; i < 5; i++)
    {
        readLowAccel();
        delay(100);
    }
    Serial.print("lx:"); Serial.print(lxForce.getValue());
    Serial.print(", ly:"); Serial.print(lyForce.getValue());
    Serial.print(", lz:"); Serial.println(lzForce.getValue());
    Serial.print("xm:"); Serial.print(xMag.getValue());
    Serial.print(", ym:"); Serial.print(yMag.getValue());
    Serial.print(", zm:"); Serial.println(zMag.getValue());

	Serial.println("Setting up Gyro...");
	if (!gyro.begin())
	{
		Serial.println("Unable to initialize gyro");
		error(4);
	}
	for (int i = 0; i < 5; i++)
    {
        readGyro();
        delay(100);
    }
    Serial.print("xr:"); Serial.print(xRotation.getValue());
    Serial.print(", yr:"); Serial.print(yRotation.getValue());
    Serial.print(", zr:"); Serial.println(zRotation.getValue());

	Serial.println("Setting up GPS...");
	Serial1.begin(GPS_BAUD);
	GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
	GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
	GPS.sendCommand(PGCMD_ANTENNA);
	Serial.println("Setup complete.");
	if (logSd)
	{
		logFile.println("===========Startup Complete============");
		logFile.flush();
	}

	slowIntervals();

	delay(1000);
	mode = MODE_READY;
}

/**
* Arduino loop.
* Read from sensors, detect state transitions, log data.
*/
void loop ()
{
    if (inReplay)
    {
        // recorded sensor data
        replay();
    }
    else if (mode != MODE_PAUSE)
    {
        // live sensor data
        if (counter % highAccelInterval == 0)
        {
            readAccel();
        }
        if (counter % altInterval == 0)
        {
            readAlt();
        }
        if (counter % gyroInterval == 0)
        {
            readGyro();
        }
        if (counter % lowAccelInterval == 0)
        {
            readLowAccel();
        }
    }

	// determine if we need to change state
	switch(mode)
	{
        case MODE_READY: modeReady(); break;
        case MODE_ASCENT: modeAscent(); break;
        case MODE_APOGEE: modeApogee(); break;
        case MODE_DESCENT: modeDescent(); break;
        case MODE_DESCENT2: modeDescent2(); break;
        case MODE_TOUCHDOWN: modeTouchdown(); break;
        case MODE_PAUSE: modePause(); break;
	}

	// check relays
	unsigned long t = millis();
	if (drogueDeployStart > 0 && (drogueDeployStart + RELAY_ON_TIME) < t)
	{
	    digitalWrite(RELAY_2_PIN, LOW);
	    drogueDeployStart = 0;
	}
	if (mainDeployStart > 0 && (mainDeployStart + RELAY_ON_TIME) < t)
    {
        digitalWrite(RELAY_1_PIN, LOW);
        mainDeployStart = 0;
    }

	// handle serial output on interval
	if (mode != MODE_PAUSE && counter % logInterval == 0)
	{
		logStatus();
		if (ledOn)
		{
			ledOn = false;
			digitalWrite(LED_PIN, LOW);
		}
		else
		{
			ledOn = true;
			digitalWrite(LED_PIN, HIGH);
		}
	}
	counter++;
}

/**
* This is used to read and replay sensor data from a log file for testing configuration.
*/
void replay()
{
    if (replayFile && replayFile.available())
    {
        // read current line from the opened file
        String line = replayFile.readStringUntil(10);
        if (line.startsWith("{"))
        {
            // current time
            unsigned long t = millis() - replayTimeOffset;
            unsigned long replayTime = 0;
            unsigned int sz = line.length();
            char buf[128];
            char lineBuf[sz];
            line.toCharArray(lineBuf, sz, 0);
            // move data into value holders
            // {m,t,alt,xF,yF,lxF,lyF,lzF,xR,yR,zR,xm,ym,zm,or,tm,pr,fix,lat,lon,spd,ang,glt,sat,dt}
            //  0 1 2   3  4  5   6   7   8  9  10 11 12 13 14 15 16 17  18  19  20  21  22  23  24
            unsigned int bufIndex = 0;
            unsigned int valIndex = 0;

            for (int i = 0; i < sz; i++)
            {
                if (lineBuf[i] == '{')
                {
                    // ignore
                }
                else if (lineBuf[i] == ',' || lineBuf[i] == '}')
                {
                    // end of value
                    buf[bufIndex] = '\0';
                    switch(valIndex)
                    {
                        // ignore mode
                        case 1:
                            replayTime = atoi(buf);
                            if (replayTime > t)
                            {
                                // wait for our clock to catch up
                                // this assumes that all log lines are in time order
                                delay(replayTime - t);
                            }
                            break;
                        case 2: altitude.x = atof(buf); break;
                        case 3: xForce.x = atof(buf); break;
                        case 4: yForce.x = atof(buf); break;
                        case 5: lxForce.x = atof(buf); break;
                        case 6: lyForce.x = atof(buf); break;
                        case 7: lzForce.x = atof(buf); break;
                        case 8: xRotation.x = atof(buf); break;
                        case 9: yRotation.x = atof(buf); break;
                        case 10: zRotation.x = atof(buf); break;
                        case 11: xMag.x = atof(buf); break;
                        case 12: yMag.x = atof(buf); break;
                        case 13: zMag.x = atof(buf); break;
                        case 14: orientation.x = atof(buf); break;
                        case 15: temperature.x = atof(buf); break;
                        case 16: pressure.x = atof(buf); break;
                        case 17: GPS.fix = atoi(buf); break;
                        case 18: latitude = atof(buf); break;
                        case 19: longitude = atof(buf); break;
                        case 20: GPS.speed = atof(buf); break;
                        case 21: GPS.angle = atof(buf); break;
                        case 22: GPS.altitude = atof(buf); break;
                        case 23: GPS.satellites = atoi(buf); break;
                        // ignore GPS date for now
                    }
                    valIndex ++;
                    bufIndex = 0;
                }
                else
                {
                    // value data, copy to line buf?
                    buf[bufIndex] = lineBuf[i];
                    bufIndex++;
                }
            }
            logInterval = 1;
        }
        else
        {
            logInterval = counter - 1;
        }
    }
    else
    {
        replayFile.close();
        Serial.println("====== REPLAY ENDED, Pausing ======");
        mode = MODE_PAUSE;
        inReplay = false;
        slowIntervals();
    }

    // if line time is > our time, delay millis
    // else play line now

    // if no file or data, end replay
}

/**
* Used to attempt to start a replay.
*/
void startReplay(String fileName)
{
    // open specified file
    char buf[fileName.length() + 1];
    fileName.toCharArray(buf, fileName.length() + 1, 0);
    replayFile = SD.open(buf);
    if (replayFile)
    {
        Serial.println("========== Starting Replay! ============");
        Serial.print("file: "); Serial.println(buf);
        replayTimeOffset = millis();
        mode = MODE_READY;
        inReplay = true;
        logInterval = 1;
    }
    else
    {
        Serial.println("!== ERROR ==!");
        Serial.print("Unable to read log file: "); Serial.println(buf);
        Serial.println("===== Pausing =====");
        mode = MODE_PAUSE;
        inReplay = false;
    }
}

/**
* Read from the High-G analog accelerometer (fast).
* This is a dual-axis, and we use a Kalman Filter to eliminate sensor noise.
*/
void readAccel()
{
	int xForceRaw = abs(analogRead(xPin) - ACCEL_X_CENTER);
	int yForceRaw = abs(analogRead(yPin) - ACCEL_Y_CENTER);

	if (!xForce.initialized)
    {
        xForce.init(HIGH_ACCEL_PROCESS_NOISE, HIGH_ACCEL_MEASURE_NOISE, HIGH_ACCEL_ERROR_COV, xForceRaw);
    }
    if (!yForce.initialized)
    {
        yForce.init(HIGH_ACCEL_PROCESS_NOISE, HIGH_ACCEL_MEASURE_NOISE, HIGH_ACCEL_ERROR_COV, yForceRaw);
    }
    xForce.update(xForceRaw);
    yForce.update(yForceRaw);
	if (xForce.getValue() > maxForce)
	{
		maxForce = xForce.getValue();
	}
	if (yForce.getValue()  > maxForce)
	{
		maxForce = yForce.getValue();
	}
}

/**
* Read from the i2c Altimeter
*/
void readAlt()
{
	sensors_event_t event;
	bmp.getEvent(&event);
	float rawPressure = event.pressure;
	float rawTemperature;
	bmp.getTemperature(&rawTemperature);
	float rawAltitude = bmp.pressureToAltitude(sealevelPressure, rawPressure, rawTemperature);
	//Serial.print("rawAltitude: "); Serial.println(rawAltitude);
	//float rawAltitude = 44330 * (1.0 - pow(rawPressure / sealevelPressure , 0.1903));

	if (!altitude.initialized)
    {
        altitude.init(ALTITUDE_PROCESS_NOISE, ALTITUDE_MEASURE_NOISE, ALTITUDE_ERROR_COV, rawAltitude);
        maxAltitude = rawAltitude;
    }
    altitude.update(rawAltitude);
    if (!pressure.initialized)
    {
        pressure.init(PRESSURE_PROCESS_NOISE, PRESSURE_MEASURE_NOISE, PRESSURE_ERROR_COV, rawPressure);
        minPressure = rawPressure;
        maxPressure = rawPressure;
    }
    pressure.update(rawPressure);
    if (!temperature.initialized)
    {
        temperature.init(TEMPERATURE_PROCESS_NOISE, TEMPERATURE_MEASURE_NOISE, TEMPERATURE_ERROR_COV, rawTemperature);
        minTemperature = rawTemperature;
        maxTemperature = rawTemperature;
    }
    temperature.update(rawTemperature);
	if (altitude.getValue() > maxAltitude)
	{
		maxAltitude = altitude.getValue();
	}
	if (temperature.getValue() < minTemperature)
	{
		minTemperature = temperature.getValue();
	}
	if (temperature.getValue() > maxTemperature)
	{
		maxTemperature = temperature.getValue();
	}
	if (pressure.getValue() < minPressure)
	{
		minPressure = pressure.getValue();
	}
	if (pressure.getValue() > maxPressure)
	{
		maxPressure = pressure.getValue();
	}
}

/**
* Read from i2c Gyro
*/
void readGyro()
{
	gyro.read();
	if (!xRotation.initialized)
    {
        xRotation.init(GYRO_PROCESS_NOISE, GYRO_MEASURE_NOISE, GYRO_ERROR_COV, gyro.data.x);
    }
    if (!yRotation.initialized)
    {
        yRotation.init(GYRO_PROCESS_NOISE, GYRO_MEASURE_NOISE, GYRO_ERROR_COV, gyro.data.y);
    }
    if (!zRotation.initialized)
    {
        zRotation.init(GYRO_PROCESS_NOISE, GYRO_MEASURE_NOISE, GYRO_ERROR_COV, gyro.data.y);
    }
	xRotation.update(gyro.data.x);
	yRotation.update(gyro.data.y);
	zRotation.update(gyro.data.z);
}

/**
* Read from the lower-G i2c Accelerometer/Compass
*/
void readLowAccel()
{
    sensors_event_t event;
    accel.getEvent(&event);

    if (!lxForce.initialized)
    {
        lxForce.init(LOW_ACCEL_PROCESS_NOISE, LOW_ACCEL_MEASURE_NOISE, LOW_ACCEL_ERROR_COV, event.acceleration.x);
    }
    if (!lyForce.initialized)
    {
        lyForce.init(LOW_ACCEL_PROCESS_NOISE, LOW_ACCEL_MEASURE_NOISE, LOW_ACCEL_ERROR_COV, event.acceleration.y);
    }
    if (!lzForce.initialized)
    {
        lzForce.init(LOW_ACCEL_PROCESS_NOISE, LOW_ACCEL_MEASURE_NOISE, LOW_ACCEL_ERROR_COV, event.acceleration.z);
    }
    lxForce.update(event.acceleration.x);
    lyForce.update(event.acceleration.y);
    lzForce.update(event.acceleration.z);

    mag.getEvent(&event);
    if (!xMag.initialized)
    {
        xMag.init(LOW_ACCEL_PROCESS_NOISE, LOW_ACCEL_MEASURE_NOISE, LOW_ACCEL_ERROR_COV, event.magnetic.x);
    }
    if (!yMag.initialized)
    {
        yMag.init(LOW_ACCEL_PROCESS_NOISE, LOW_ACCEL_MEASURE_NOISE, LOW_ACCEL_ERROR_COV, event.magnetic.y);
    }
    if (!zMag.initialized)
    {
        zMag.init(LOW_ACCEL_PROCESS_NOISE, LOW_ACCEL_MEASURE_NOISE, LOW_ACCEL_ERROR_COV, event.magnetic.z);
    }
    xMag.update(event.magnetic.x);
    yMag.update(event.magnetic.y);
    zMag.update(event.magnetic.z);
}

/**
* Set the sensor polling and logging intervals to "slow".
*/
void slowIntervals()
{
	altInterval = ALTIMETER_INTERVAL_SLOW;
	gyroInterval = GYRO_INTERVAL_SLOW;
	logInterval = LOG_INTERVAL_SLOW;
	lowAccelInterval = LOW_ACCEL_INTERVAL_SLOW;
	highAccelInterval = HIGH_ACCEL_INTERVAL_SLOW;
}

/**
* Set the sensor polling and logging intervals to "fast".
*/
void fastIntervals()
{
	altInterval = ALTIMETER_INTERVAL_FAST;
	gyroInterval = GYRO_INTERVAL_FAST;
	logInterval = LOG_INTERVAL_FAST;
	lowAccelInterval = LOW_ACCEL_INTERVAL_FAST;
	highAccelInterval = HIGH_ACCEL_INTERVAL_FAST;
}

/**
* Ready mode, looking for liftoff condition.
*/
void modeReady()
{
	if( xForce.getValue() > ascentThreshold
		|| yForce.getValue() > ascentThreshold )
	{
		mode = MODE_ASCENT;
		launchTime = millis();
		fastIntervals();
		Serial.println("=========== Launch Detected! ============");
		if (logSd)
		{
			logFile.println("=========== Launch Detected! ============");
			logFile.flush();
		}
		return;
	}
	// keep adjusting starting altitude (as sensor warms up, etc)
	startingAltitude = altitude.getValue();
}

/**
* Ascent mode, looking for apogee condition.
*/
void modeAscent()
{
	// constantly check accelerometer for levelling off or swing in direction of force
	// midpoint is 500 +/-50
	// check altimeter for drops?
	// switch to apogee mode

	if( xForce.getValue() < apogeeThreshold
		&& yForce.getValue() < apogeeThreshold )
	{
		mode = MODE_APOGEE;
		apogeeTime = millis();
		Serial.println("=========== Apogee Detected! ============");
		if (logSd)
		{
			logFile.println("=========== Apogee Detected! ============");
			logFile.flush();
		}
	}
}

/**
* Apogee mode, looking for descent.
*/
void modeApogee()
{
	// wait for altitude to start dropping?
	deployDrogue();
	mode = MODE_DESCENT;
	Serial.println("=========== Descending ============");
	if (logSd)
    {
        logFile.println("=========== Descending ============");
        logFile.flush();
    }
}

/**
* Descent mode, looking for main deployment condition.
*/
void modeDescent()
{
	// constantly check altimeter for main chute deployment altitude
	// check GPS altitude too?
	if (altitude.getValue() <= (startingAltitude + mainDeployAltitude))
	{
		mode = MODE_DESCENT2;
		mainTime = millis();
		Serial.println("=========== Main Deploy Altitude Detected! ============");
		if (logSd)
		{
			logFile.println("=========== Main Deploy Altitude Detected! ============");
			logFile.flush();
		}
		deployMain();
	}
}

/**
* Main chute deployed, looking for touchdown.
*/
void modeDescent2()
{
	// constantly check accelerometer for midpoint +/- 20?
	if ( xForce.getValue() < touchdownThreshold
		&& yForce.getValue() < touchdownThreshold )
	{
		// switch to touchdown
		mode = MODE_TOUCHDOWN;
		touchdownTime = millis();
		slowIntervals();
		Serial.println("=========== Touchdown! ============");
		if (logSd)
		{
			logFile.println("=========== Touchdown! ============");
			logFile.flush();
		}
		logStats();
		// stop logging to the SD at this point
		logSd = false;
	}
}

/**
* Touchdown complete, awaiting pickup.
*/
void modeTouchdown()
{
	// sparse logging
	// delay a few ms
	delay(100); // multiplied by the logInterval
}

/**
* Just hang around a bit.
*/
void modePause()
{
    delay(1000);
}


/**
* Deploy drogue chute
*/
void deployDrogue()
{
    digitalWrite(RELAY_2_PIN, HIGH);
    drogueDeployStart = millis();
    Serial.println("=========== Deploying Drogue ============");
	if (logSd)
    {
        logFile.println("=========== Deploying Drogue ============");
        logFile.flush();
    }
	//delay(1000);
	//digitalWrite(RELAY_2_PIN, LOW);
}

/**
* Deploy main chute
*/
void deployMain()
{
    digitalWrite(RELAY_1_PIN, HIGH);
    mainDeployStart = millis();
    Serial.println("=========== Deploying Main ============");
	if (logSd)
    {
        logFile.println("=========== Deploying Main ============");
        logFile.flush();
    }
    //delay(1000);
    //digitalWrite(RELAY_1_PIN, LOW);
}

/**
* Await any command input on the main serial line (from the XBee or console)
*/
void serialEvent()
{
	//read serial as a character
	char ser;
	while (Serial.available())
	{
		ser = Serial.read();
		if (ser == 10 || ser == ';') // return
		{
		    if (buffer.startsWith("deploy drogue"))
            {
                deployDrogue();
            }
            else if (buffer.startsWith("deploy main"))
            {
                deployMain();
            }
            else if (buffer.startsWith("deploy all"))
            {
                deployDrogue();
                deployMain();
            }
            else if (buffer.startsWith("set mode") && buffer.length() > 9)
            {
                const char tmp = buffer.charAt(9);
                mode = atoi(&tmp);
                Serial.print("===== MODE set to "); Serial.print(mode); Serial.println(" ===========");
            }
            else if (buffer.startsWith("pause"))
            {
                mode = MODE_PAUSE;
                Serial.print("===== PAUSED =======");
                Serial.print("Use 'resume', 'unpause', or 'set mode 1' to resume");
            }
            else if (buffer.startsWith("resume") || buffer.startsWith("unpause"))
            {
                mode = MODE_READY;
                Serial.print("===== RESUMING =======");
            }
            else if (buffer.startsWith("replay ") && buffer.length() > 6)
            {
                startReplay(buffer.substring(7, buffer.length()));
            }
            else
            {
                Serial.print("unknown command: ");
                Serial.println(buffer);
            }
			buffer = "";
		}
		else
		{
			buffer += ser;
		}
	}
}

/**
* Await any input from the GPS serial line.
*/
void serialEvent1()
{
	char ser;
	while (Serial1.available())
	{
		ser = Serial1.read();
		if (ser == 13)
		{
			// ignore
		}
		if (ser == 10)
		{
			// ignore in replay mode
			if (!inReplay)
            {
                uint8_t bufferSize = buffer1.length();
                char tmpBuffer[bufferSize];
                buffer1.toCharArray(tmpBuffer,buffer1.length());
                if (logSd && logRawGps)
                {
                    // immediately log raw GPS data to file
                    if (bufferSize > logFile.print(buffer1))
                    {
                        Serial.println("ERROR: Failed to write to SD card.");
                        logSd = false;
                    }
                    logFile.flush();
                }
                if (!buffer1.startsWith("$P"))
                {
                    if (GPS.parse(tmpBuffer))
                    {
                        float latDeg = (int)GPS.latitude / 100;
                        float latMin = fmod(GPS.latitude, 100.0);
                        latitude = latDeg + (latMin / 60.0);
                        if (GPS.lat == 'S' || GPS.lat == 's')
                        {
                            latitude *= -1.0;
                        }

                        float lonDeg = (int)GPS.longitude / 100;
                        float lonMin = fmod(GPS.longitude, 100.0);
                        longitude = lonDeg + (lonMin / 60.0);
                        if (GPS.lon == 'W' || GPS.lon == 'w')
                        {
                            longitude *= -1.0;
                        }
                    }
                    else if (logSd)
                    {
                        logFile.println("GPS_parse_error");
                        logFile.flush();
                    }
                }
            }
			buffer1 = "";
		}
		else
		{
			buffer1 += ser;
		}
	}
}

/**
* Log our current status, to MicroSD and Serial
*/
void logStatus()
{
	char alt[32];
	dtostrf(altitude.getValue(), 5, 2, alt);
	char prs[32];
	dtostrf(pressure.getValue(), 8, 2, prs);
	char tem[32];
	dtostrf(temperature.getValue(), 4, 2, tem);

	char xf[32];
	dtostrf(xForce.getValue(), 4, 2, xf);
	char yf[32];
	dtostrf(yForce.getValue(), 4, 2, yf);
	//char zf[32];
	//dtostrf(zForce.getValue(), 4, 2, zf);

	char lxf[32];
	dtostrf(lxForce.getValue(), 4, 2, lxf);
	char lyf[32];
	dtostrf(lyForce.getValue(), 4, 2, lyf);
	char lzf[32];
	dtostrf(lzForce.getValue(), 4, 2, lzf);
	char xm[32];
	dtostrf(xMag.getValue(), 4, 2, xm);
	char ym[32];
	dtostrf(yMag.getValue(), 4, 2, ym);
	char zm[32];
	dtostrf(zMag.getValue(), 4, 2, zm);
	char ori[32];
	dtostrf(orientation.getValue(), 4, 2, ori);

	char xr[32];
	dtostrf(xRotation.getValue(), 4, 2, xr);
	char yr[32];
	dtostrf(yRotation.getValue(), 4, 2, yr);
	char zr[32];
	dtostrf(zRotation.getValue(), 4, 2, zr);

	String printBuffer = "";

	if (printCount % PRINT_HEADER_INTERVAL == 0)
    {
        printBuffer += "[m,t,alt,xF,yF,lxF,lyF,lzF,xR,yR,zR,xm,ym,zm,or,tm,pr,fix,lat,lon,spd,ang,glt,sat,dt]\r\n";
    }
	printBuffer += '{';
	printBuffer += mode; printBuffer += ',';
	printBuffer += millis(); printBuffer += ',';
	printBuffer += alt; printBuffer += ',';
	printBuffer += xf; printBuffer += ',';
	printBuffer += yf; printBuffer += ',';
	//printBuffer += zf; printBuffer += ',';
	printBuffer += lxf; printBuffer += ',';
	printBuffer += lyf; printBuffer += ',';
	printBuffer += lzf; printBuffer += ',';
	printBuffer += xr; printBuffer += ',';
	printBuffer += yr; printBuffer += ',';
	printBuffer += zr; printBuffer += ',';
	printBuffer += xm; printBuffer += ',';
	printBuffer += ym; printBuffer += ',';
	printBuffer += zm; printBuffer += ',';
	printBuffer += ori; printBuffer += ',';
	printBuffer += tem; printBuffer += ',';
	printBuffer += prs; printBuffer += ',';
	printBuffer += (int)GPS.fix; //printBuffer += '}';

	if (GPS.fix)
	{
		char lat[32];
		dtostrf(latitude, 10, 6, lat);
		char lon[32];
		dtostrf(longitude, 10, 6, lon);
		char spd[32];
		dtostrf(GPS.speed, 6, 2, spd);
		char ang[32];
		dtostrf(GPS.angle, 6, 2, ang);
		char galt[32];
		dtostrf(GPS.altitude, 6, 2, galt);

		//printBuffer += "\r\nL:";
		printBuffer += ',';
		printBuffer += lat; printBuffer += ',';
		printBuffer += lon; printBuffer += ',';
		printBuffer += spd; printBuffer += ','; //printBuffer += "kn,";
		printBuffer += ang; printBuffer += ','; //printBuffer += "dg,";
		printBuffer += galt; printBuffer += ','; //printBuffer += "m,";
		printBuffer += (int)GPS.satellites; printBuffer += ','; //printBuffer += "sat,";
		printBuffer += GPS.year; printBuffer += '-';
		printBuffer += GPS.month; printBuffer += '-';
		printBuffer += GPS.day; printBuffer += '_';
		printBuffer += GPS.hour; printBuffer += ':';
		printBuffer += GPS.minute; printBuffer += ':';
		printBuffer += GPS.seconds; printBuffer += '.';
		printBuffer += GPS.milliseconds; //printBuffer += ';';
	}
	else
    {
        printBuffer += ",,,,,,";
    }
	printBuffer += '}';
	Serial.println(printBuffer);
	if (logSd)
	{
		logFile.println(printBuffer);
		logFile.flush();
	}
	printCount++;
}

/**
* Log our flight stats.
*/
void logStats()
{
	char stalt[32];
	dtostrf(startingAltitude, 5, 2, stalt);
	char maxalt[32];
	dtostrf(maxAltitude, 5, 2, maxalt);
	char maxtemp[32];
	dtostrf(maxTemperature, 5, 2, maxtemp);
	char mintemp[32];
	dtostrf(minTemperature, 5, 2, mintemp);
	char maxpress[32];
	dtostrf(maxPressure, 5, 2, maxpress);
	char minpress[32];
	dtostrf(minPressure, 5, 2, minpress);

	String printBuffer = "========= Flight Stats ============\r\n";
	printBuffer += "Starting Altitude: "; printBuffer += stalt; printBuffer += "\r\n";
	printBuffer += "Maximum Altitude: "; printBuffer += maxalt; printBuffer += "\r\n";
	printBuffer += "Maximum Temperature: "; printBuffer += maxtemp; printBuffer += "\r\n";
	printBuffer += "Minimum Temperature: "; printBuffer += mintemp; printBuffer += "\r\n";
	printBuffer += "Maximum Pressure: "; printBuffer += maxpress; printBuffer += "\r\n";
	printBuffer += "Minimum Pressure: "; printBuffer += minpress; printBuffer += "\r\n";
	printBuffer += "Maximum Force: "; printBuffer += maxForce; printBuffer += "\r\n";
	printBuffer += "Launch Time: "; printBuffer += launchTime; printBuffer += "\r\n";
	printBuffer += "Apogee Time: "; printBuffer += apogeeTime; printBuffer += "\r\n";
	printBuffer += "Main Time: "; printBuffer += mainTime; printBuffer += "\r\n";
	printBuffer += "Touchdown Time: "; printBuffer += touchdownTime; printBuffer += "\r\n";
	printBuffer += "Ascent Time: "; printBuffer += (apogeeTime - launchTime); printBuffer += "\r\n";
	printBuffer += "Descent Time: "; printBuffer += (touchdownTime - apogeeTime); printBuffer += "\r\n";
	printBuffer += "Flight Time: "; printBuffer += (touchdownTime - launchTime); printBuffer += "\r\n";
	Serial.println(printBuffer);
	if (logSd)
	{
		logFile.println(printBuffer);
		logFile.flush();
	}
}

/**
* Unrecoverable error, uh oh.
*/
void error(uint8_t errno)
{
	mode = MODE_ERROR;
	Serial.println("=========== ERROR: errno ============");
	while(1)
	{
		uint8_t i;
		for (i=0; i<errno; i++)
		{
			digitalWrite(LED_PIN, HIGH);
			delay(100);
			digitalWrite(LED_PIN, LOW);
			delay(100);
		}
		for (i=errno; i<10; i++)
		{
			delay(200);
		}
	}
}
