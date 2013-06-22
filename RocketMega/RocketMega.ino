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
#include <Adafruit_L3GD20.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>

// constants
#define SERIAL_BAUD 57600
#define GPS_BAUD 9600
#define LED_PIN 13
#define SD_CHIP_SELECT 48
#define SENSORS_DPS_TO_RADS (0.017453293F)
#define ACCEL_X_CENTER 500
#define ACCEL_Y_CENTER 500

#define LOG_INTERVAL_SLOW 2000
#define LOG_INTERVAL_FAST 1000
#define ALTIMETER_INTERVAL_SLOW 1000
#define ALTIMETER_INTERVAL_FAST 250
#define GYRO_INTERVAL_SLOW 1000
#define GYRO_INTERVAL_FAST 500

#define DEFAULT_ASCENT_THRESHOLD 50      // in distance from ACCEL_[X/Y]_CENTER where center is 1G
#define DEFAULT_APOGEE_THRESHOLD 10      // in distance from ACCEL_[X/Y]_CENTER where center is 1G
#define DEFAULT_TOUCHDOWN_THRESHOLD 2	 // in distance from ACCEL_[X/Y]_CENTER where center is 1G
#define DEFAULT_MAIN_ALTITUDE 300        // in meters (added to deck altitude)
#define DEFAULT_SEALEVEL_PRESSURE 101321 // in pascals, 101325

#define MODE_ERROR -1
#define MODE_STARTUP 0
#define MODE_READY 1
#define MODE_ASCENT 2
#define MODE_APOGEE 3
#define MODE_DESCENT 4
#define MODE_DESCENT2 5
#define MODE_TOUCHDOWN 6

typedef struct {
	double q;
	double r;
	double x;
	double p;
	double k;
} KalmanState;

// prototypes
void error(uint8_t errno);
void readAccel();
void readAlt();
void readGyro();
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
KalmanState kalmanInit(double q, double r, double p, double initialValue);
void kalmanUpdate(KalmanState* state, double measurement);

// variables
uint8_t mode = MODE_STARTUP;
uint8_t altInterval = ALTIMETER_INTERVAL_SLOW;
uint8_t gyroInterval = GYRO_INTERVAL_SLOW;
uint8_t logInterval = LOG_INTERVAL_SLOW;

Adafruit_GPS GPS(&Serial1);
File logFile;
String buffer = "";
String buffer1 = "";
boolean logSd = false;
boolean logRawGps = true;
Adafruit_L3GD20 gyro;
Adafruit_BMP085 bmp;
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

int xPin = A0;
int yPin = A1;
int xForce = 0;
int yForce = 0;
int xRotation = 0;
int yRotation = 0;
int zRotation = 0;
float temperature = 0;
float pressure = 0;
float altitude = 0;
boolean ledOn = false;
uint16_t timer = 0;

KalmanState xForceK;
KalmanState yForceK;

void setup()
{
	pinMode(LED_PIN, OUTPUT);
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

	Serial.println("Setting up Accelerometer...");
	xForceK = kalmanInit(0.500, 4.0, (double) ACCEL_X_CENTER, (double) analogRead(xPin));
	yForceK = kalmanInit(0.500, 4.0, (double) ACCEL_Y_CENTER, (double) analogRead(yPin));

	Serial.println("Setting up Gyro...");
	if (!gyro.begin())
	{
		Serial.println("Unable to initialize gyro");
		error(4);
	}

	Serial.println("Setting up Altimeter...");
	if (!bmp.begin(BMP085_ULTRALOWPOWER))
	{
		Serial.println("Unable to initialize altimeter");
		error(5);
	}
	readAlt();
	startingAltitude = altitude;
	minPressure = pressure;
	Serial.print("Starting altitude initially set at ");
	Serial.print(altitude);
	Serial.println("m...");

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

void loop ()
{
	// read accelerometer (every loop?)
	readAccel();
	// read altimeter on interval
	if (timer % altInterval == 0)
	{
		readAlt();
	}
	// read gyro on interval
	if (timer % gyroInterval == 0)
	{
		readGyro();
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
	}

	// handle serial output on interval
	if (timer % logInterval == 0)
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
	timer++;
}

// read the accelerometer values and store them
void readAccel()
{
	// these will need to be cleaned up with a Kalman filter
	xForce = analogRead(xPin);
	yForce = analogRead(yPin);
	kalmanUpdate(&xForceK, (double)xForce);
	kalmanUpdate(&yForceK, (double)yForce);
	if (abs(xForce) - ACCEL_X_CENTER > maxForce)
	{
		maxForce = abs(xForce) - ACCEL_X_CENTER;
	}
	if (abs(yForce) - ACCEL_Y_CENTER > maxForce)
	{
		maxForce = abs(yForce) - ACCEL_Y_CENTER;
	}
}

// read the altimeter values and store them
void readAlt()
{
	// these will need to be cleaned up with a Kalman filter
	temperature = bmp.readTemperature();
	altitude = bmp.readAltitude();
	pressure = bmp.readPressure();
	altitude = 44330 * (1.0 - pow(pressure/sealevelPressure,0.1903));
	if (altitude > maxAltitude)
	{
		maxAltitude = altitude;
	}
	if (temperature < minTemperature)
	{
		minTemperature = temperature;
	}
	if (temperature > maxTemperature)
	{
		maxTemperature = temperature;
	}
	if (pressure < minPressure)
	{
		minPressure = pressure;
	}
	if (pressure > maxPressure)
	{
		maxPressure = pressure;
	}
}

// read the gyro values and store them
void readGyro()
{
	gyro.read();
	xRotation = (int)gyro.data.x;
	yRotation = (int)gyro.data.y;
	zRotation = (int)gyro.data.z;
}

// set reading and logging rates at the slower intervals
void slowIntervals()
{
	altInterval = ALTIMETER_INTERVAL_SLOW;
	gyroInterval = GYRO_INTERVAL_SLOW;
	logInterval = LOG_INTERVAL_SLOW;
}

// set reading and logging rates at the faster intervals
void fastIntervals()
{
	altInterval = ALTIMETER_INTERVAL_FAST;
	gyroInterval = GYRO_INTERVAL_FAST;
	logInterval = LOG_INTERVAL_FAST;
}

// Ready and waiting for liftoff
void modeReady()
{
	if( abs(xForce - ACCEL_X_CENTER) > ascentThreshold
		|| abs(yForce - ACCEL_Y_CENTER) > ascentThreshold )
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
	startingAltitude = altitude;
}

// Going up, waiting for apogee
void modeAscent()
{
	// constantly check accelerometer for levelling off or swing in direction of force
	// midpoint is 500 +/-50
	// check altimeter for drops?
	// switch to apogee mode
	mode = MODE_APOGEE;

	if( abs(xForce - ACCEL_X_CENTER) < apogeeThreshold
		&& abs(yForce - ACCEL_Y_CENTER) < apogeeThreshold )
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

// Hang time at apogee for a second or so
void modeApogee()
{
	// wait for altitude to start dropping?
	// TODO: set Drogue Relay toggle to HIGH
	mode = MODE_DESCENT;
}

// descent under drogue chute, waiting for altitude
void modeDescent()
{
	// constantly check altimeter for main chute deployment altitude
	// check GPS altitude too?
	if (altitude <= (startingAltitude + mainDeployAltitude))
	{
		// TODO: set Main Relay toggle to HIGH
		mode = MODE_DESCENT2;
		mainTime = millis();
		Serial.println("=========== Main Deploy Altitude Detected! ============");
		if (logSd)
		{
			logFile.println("=========== Main Deploy Altitude Detected! ============");
			logFile.flush();
		}
	}
}

// under full chute, waiting for touchdown on the ground
void modeDescent2()
{
	// constantly check accelerometer for midpoint +/- 20?
	if ( abs(xForce - ACCEL_X_CENTER) < touchdownThreshold
		&& abs(yForce - ACCEL_Y_CENTER) < touchdownThreshold )
	{
		// switch to touchdown
		mode = MODE_TOUCHDOWN;
		touchdownTime = millis();
		slowIntervals();
		// TODO: turn off Drogue and Main relays!
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

// on the ground, just broadcast location and stuff
void modeTouchdown()
{
	// sparse logging
	// delay a few ms
	delay(100); // multiplied by the logInterval
}

// main serial line
void serialEvent()
{
	//read serial as a character
	char ser;
	while (Serial.available())
	{
		ser = Serial.read();
		if (ser == 10 || ser == ';') // return
		{
			Serial.print("line: ");
			Serial.println(buffer);
			buffer = "";
		}
		else
		{
			buffer += ser;
		}
	}
}

// GPS serial line
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
			//Serial.println(buffer1);
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
			if (!buffer1.startsWith("$P") && !GPS.parse(tmpBuffer) && logSd)
			{
				logFile.println("GPS_parse_error");
				logFile.flush();
			}
			buffer1 = "";
		}
		else
		{
			buffer1 += ser;
		}
	}
}

void logStatus()
{
	char alt[32];
	dtostrf(altitude, 5, 2, alt);
	char prs[32];
	dtostrf(pressure, 8, 2, prs);
	char tem[32];
	dtostrf(temperature, 4, 2, tem);

	char xf[32];
	dtostrf(xForceK.x, 4, 2, xf);
	char yf[32];
	dtostrf(yForceK.x, 4, 2, yf);

	String printBuffer = "S:";
	printBuffer += mode; printBuffer += ",";
	printBuffer += millis(); printBuffer += ',';
	printBuffer += alt; printBuffer += "m,";
	printBuffer += xForce; printBuffer += "x,";
		printBuffer += '['; printBuffer += xf; printBuffer += "x],";
	printBuffer += yForce; printBuffer += "y,";
		printBuffer += '['; printBuffer += yf; printBuffer += "y],";
	printBuffer += xRotation; printBuffer += "x,";
	printBuffer += yRotation; printBuffer += "y,";
	printBuffer += zRotation; printBuffer += "z,";
	printBuffer += tem; printBuffer += "C,";
	printBuffer += prs; printBuffer += "pa,";
	printBuffer += (int)GPS.fix; printBuffer += ";";

	if (GPS.fix)
	{
		float latDeg = (int)GPS.latitude / 100;
		float latMin = fmod(GPS.latitude, 100.0);
		float realLat = latDeg + (latMin / 60.0);
		if (GPS.lat == 'S' || GPS.lat == 's')
		{
			realLat *= -1.0;
		}

		float lonDeg = (int)GPS.longitude / 100;
		float lonMin = fmod(GPS.longitude, 100.0);
		float realLon = lonDeg + (lonMin / 60.0);
		if (GPS.lon == 'W' || GPS.lon == 'w')
		{
			realLon *= -1.0;
		}

		char lat[32];
		dtostrf(realLat, 10, 6, lat);
		char lon[32];
		dtostrf(realLon, 10, 6, lon);
		char spd[32];
		dtostrf(GPS.speed, 6, 2, spd);
		char ang[32];
		dtostrf(GPS.angle, 6, 2, ang);
		char galt[32];
		dtostrf(GPS.altitude, 6, 2, galt);

		printBuffer += "\r\nL:";
		printBuffer += lat; printBuffer += GPS.lat; printBuffer += ',';
		printBuffer += lon; printBuffer += GPS.lon; printBuffer += ',';
		printBuffer += spd; printBuffer += "kn,";
		printBuffer += ang; printBuffer += "dg,";
		printBuffer += galt; printBuffer += "m,";
		printBuffer += (int)GPS.satellites; printBuffer += "sat,";
		printBuffer += GPS.year; printBuffer += '-';
		printBuffer += GPS.month; printBuffer += '-';
		printBuffer += GPS.day; printBuffer += '_';
		printBuffer += GPS.hour; printBuffer += ':';
		printBuffer += GPS.minute; printBuffer += ':';
		printBuffer += GPS.seconds; printBuffer += '.';
		printBuffer += GPS.milliseconds; printBuffer += ';';
	}
	Serial.println(printBuffer);
	if (logSd)
	{
		logFile.println(printBuffer);
		logFile.flush();
	}
}

// log interesting stats of the flight
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

void error(uint8_t errno)
{
	mode = MODE_ERROR;
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

KalmanState kalmanInit(double q, double r, double p, double initialValue)
{
	KalmanState result;
	result.q = q;
	result.r = r;
	result.p = p;
	result.x = initialValue;
	return result;
}

void kalmanUpdate(KalmanState* state, double measurement)
{
	state->p = state->p + state->q;
	state->k = state->p / (state->p + state->r);
	state->x = state->x + state->k * (measurement - state->x);
	state->p = (1 - state->k) * state->p;
}
