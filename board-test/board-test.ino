/**
* Test the RocketMega R2 board functionality.
*/

#define DEBUG_SERIAL Serial
#define DEBUG_BAUD 57600
#define TELEM_SERIAL Serial1
#define TELEM_BAUD 57600
#define GPS_BAUD 9600
#define GPS_SERIAL Serial2

#define SD_CHIP_SELECT 48
//#define SPI_MISO_PIN 50
//#define SPI_MOSI_PIN 51
//#define SPI_SCLK_PIN 52

#define ACCEL_X_PIN A8
#define ACCEL_Y_PIN A9
#define ACCEL_Z_PIN A10
// taken from ADXL377 datasheet (can be between 5.8 and 7.2, 6.5 is typical)
#define ACCEL_MV_TO_G 6.5

#define ACCEL_X_CENTER_OFFSET 0
#define ACCEL_Y_CENTER_OFFSET 0
#define ACCEL_Z_CENTER_OFFSET 0

#define PYRO_1_PIN 42
#define PYRO_2_PIN 43
#define PYRO_3_PIN 44
#define PYRO_4_PIN 45
#define PYRO_1_CONT_PIN A11
#define PYRO_2_CONT_PIN A12
#define PYRO_3_CONT_PIN A13
#define PYRO_4_CONT_PIN A14
#define PYRO_ON_TIME 1000 // in milliseconds

#define VOLTAGE_PIN A15

#if defined(__AVR_ATmega168__) ||defined(__AVR_ATmega168P__) ||defined(__AVR_ATmega328P__)
  #define VOLTAGE_REF 5.0
  #define V_PER_STEP 0.0048828125 // vref / adc_max
  #define MV_PER_STEP 4.8828125
  #define ACCEL_X_CENTER 337
  #define ACCEL_Y_CENTER 337
  #define ACCEL_Z_CENTER 337
  // for 10-bit ADC, range is 0-1023
  // ((3.3/5.0) * 1023) / 2 = 337.59 -> center point
  // 200/337.59 = 0.59243761002991 G's per step
  #define G_PER_STEP 0.59243761

// Mega 1280 & 2560 (5v)
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  #define VOLTAGE_REF 5.0
  #define V_PER_STEP 0.0048828125
  #define MV_PER_STEP 4.8828125
  #define ACCEL_X_CENTER 337
  #define ACCEL_Y_CENTER 337
  #define ACCEL_Z_CENTER 337
  #define G_PER_STEP 0.59243761

// Due and others (3.3v)
#else
  #define VOLTAGE_REF 3.3
  #define V_PER_STEP 0.00322265625
  #define MV_PER_STEP 3.22265625
  #define ACCEL_X_CENTER 512
  #define ACCEL_Y_CENTER 512
  #define ACCEL_Z_CENTER 512
  // for 10-bit ADC (default setting), range is 0-1023
  // 1024 / 2 = 512 -> center point
  // 200/512 = 0.390625 G's per step
  #define G_PER_STEP 0.390625

#endif


#define LED_PIN 13
#define BUZZER_PIN 8

#define DEFAULT_SEALEVEL_PRESSURE 1013.21 // in h pascals, 101325

#include <Arduino.h>
#include <Adafruit_GPS.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_L3GD20.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_LSM303.h>


HardwareSerial DebugSerial = DEBUG_SERIAL;
HardwareSerial TelemSerial = TELEM_SERIAL;
HardwareSerial GpsSerial = GPS_SERIAL;
Adafruit_GPS GPS(&GpsSerial);
Adafruit_L3GD20 gyro;
Adafruit_BMP085 bmp = Adafruit_BMP085(10085);
Adafruit_LSM303_Mag mag = Adafruit_LSM303_Mag(12345);
Adafruit_LSM303_Accel accel = Adafruit_LSM303_Accel(54321);
float sealevelPressure = DEFAULT_SEALEVEL_PRESSURE;
String buffer1 = "";

int pyroPin = PYRO_1_PIN;


void setupSerials()
{
    DebugSerial.begin(DEBUG_BAUD);
    TelemSerial.begin(TELEM_BAUD);
    GpsSerial.begin(GPS_BAUD);
    DebugSerial.println("INFO: Serials configured");
}

void setupPins()
{
    pinMode(LED_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
	pinMode(PYRO_1_PIN, OUTPUT);
	pinMode(PYRO_2_PIN, OUTPUT);
	pinMode(PYRO_3_PIN, OUTPUT);
	pinMode(PYRO_4_PIN, OUTPUT);
	pinMode(SD_CHIP_SELECT, OUTPUT);
	DebugSerial.println("INFO: Pins configured");
}

void setupSd()
{
    if (!SD.begin(SD_CHIP_SELECT))
	{
		DebugSerial.println("ERROR: SD Card init failed!");
		return;
	}
	DebugSerial.println("INFO: SD Card initialized");
}

void testSd()
{
    char filename[15];
    strcpy(filename, "TEST.TXT");
    File logFile = SD.open(filename, FILE_WRITE);
    if( ! logFile )
    {
        DebugSerial.println("ERROR: Unable to create/open file TEST.TXT");
    }
    else
    {
        DebugSerial.println("INFO: Opened TEST.TXT");
        logFile.println("Test data");
        logFile.println(millis());
        logFile.close();
        DebugSerial.println("INFO: Wrote test data");
    }
}

void setupHighGAccel()
{
    // nothing to do during setup
}

void testHighGAccel()
{
    float xVal = (float)(analogRead(ACCEL_X_PIN) - ACCEL_X_CENTER);
    xVal *= MV_PER_STEP;
    xVal /= ACCEL_MV_TO_G;
    float yVal = (float)(analogRead(ACCEL_Y_PIN) - ACCEL_Y_CENTER);
    yVal *= MV_PER_STEP;
    yVal /= ACCEL_MV_TO_G;
    float zVal = (float)(analogRead(ACCEL_Z_PIN) - ACCEL_Z_CENTER);
    zVal *= MV_PER_STEP;
    zVal /= ACCEL_MV_TO_G;
    DebugSerial.print("INFO: Analog Accel (");
    DebugSerial.print(xVal);
    DebugSerial.print(",");
    DebugSerial.print(yVal);
    DebugSerial.print(",");
    DebugSerial.print(zVal);
    DebugSerial.println(")");
}

void setupAltimeter()
{
    if (!bmp.begin(BMP085_MODE_STANDARD))
	{
		DebugSerial.println("ERROR: Unable to initialize altimeter");
		return;
	}
	DebugSerial.println("INFO: Altimeter initialized");
}

void testAltimeter()
{
    sensors_event_t event;
	bmp.getEvent(&event);
	float rawPressure = event.pressure;
	float rawTemperature;
	bmp.getTemperature(&rawTemperature);
	float rawAltitude = bmp.pressureToAltitude(sealevelPressure, rawPressure, rawTemperature);

	DebugSerial.print("INFO: Pressure ");
	DebugSerial.println(rawPressure);
	DebugSerial.print("INFO: Temperature ");
	DebugSerial.println(rawTemperature);
	DebugSerial.print("INFO: Altitude ");
	DebugSerial.println(rawAltitude);
}

void setupLowGAccel()
{
    if (!mag.begin())
    {
        DebugSerial.println("ERROR: Unable to initialize Magnetometer");
        return;
    }
    if (!accel.begin())
    {
        DebugSerial.println("ERROR: Unable to initialize Low-G Accelerometer");
        return;
    }
    DebugSerial.println("INFO: Low-G Accelerometer and Magnetometer initialized");
}

void testLowGAccel()
{
    sensors_event_t event;
    accel.getEvent(&event);
    DebugSerial.print("INFO: LowGAccel (");
    DebugSerial.print(event.acceleration.x);
    DebugSerial.print(",");
    DebugSerial.print(event.acceleration.y);
    DebugSerial.print(",");
    DebugSerial.print(event.acceleration.z);
    DebugSerial.println(")");

    mag.getEvent(&event);
    DebugSerial.print("INFO: Magnetometer (");
    DebugSerial.print(event.magnetic.x);
    DebugSerial.print(",");
    DebugSerial.print(event.magnetic.y);
    DebugSerial.print(",");
    DebugSerial.print(event.magnetic.z);
    DebugSerial.println(")");
}

void setupGyro()
{
    if (!gyro.begin())
	{
		DebugSerial.println("ERROR: Unable to initialize gyro");
		return;
	}
	DebugSerial.println("INFO: Gyro initialized");
}

void testGyro()
{
    gyro.read();
    DebugSerial.print("INFO: Gyro (");
    DebugSerial.print(gyro.data.x);
    DebugSerial.print(",");
    DebugSerial.print(gyro.data.y);
    DebugSerial.print(",");
    DebugSerial.print(gyro.data.z);
    DebugSerial.println(")");
}

void setupGps()
{
	GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
	GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
	GPS.sendCommand(PGCMD_ANTENNA);
	DebugSerial.println("INFO: GPS initialized");
}

void testGps()
{
    // this is automatially handled by serial2 event
}

void testContinuity()
{
    int cont1 = analogRead(PYRO_1_CONT_PIN);
    int cont2 = analogRead(PYRO_2_CONT_PIN);
    int cont3 = analogRead(PYRO_3_CONT_PIN);
    int cont4 = analogRead(PYRO_4_CONT_PIN);
    DebugSerial.print("INFO: Pyro Continuity (");
    DebugSerial.print(cont1 > 0 ? "X" : "_");
    DebugSerial.print(",");
    DebugSerial.print(cont2 > 0 ? "X" : "_");
    DebugSerial.print(",");
    DebugSerial.print(cont3 > 0 ? "X" : "_");
    DebugSerial.print(",");
    DebugSerial.print(cont4 > 0 ? "X" : "_");
    DebugSerial.println(")");
}

void testVoltage()
{
    double vraw = (double)analogRead(VOLTAGE_PIN);

    DebugSerial.print("INFO: Raw voltage read ");
    DebugSerial.println(vraw);
    double vout = vraw * V_PER_STEP;

    DebugSerial.print("INFO: Voltage at pin ");
    DebugSerial.println(vout);

    // voltage divider equation: Vout = (Rb / (Ra + Rb)) * Vin
    // Ra = 620k
    // Rb = 100k
    //double Ra = 620000.0; // ohms
    //double Rb = 100000.0; // ohms
    double factor = 0.13888888;
    double voltage = vout / factor;

    DebugSerial.print("INFO: Actual Input Voltage ");
    DebugSerial.println(voltage);
}

void testBuzzer()
{
    //tone(BUZZER_PIN, 1915, 250);
}

void setup()
{
	setupSerials();
	setupPins();
    //setupHighGAccel();
    //setupAltimeter();
    //setupLowGAccel();
    //setupGyro();
    //setupGps();
    setupSd();
    DebugSerial.println("INFO: Setup complete");
    delay(1000);
}

void loop()
{
	DebugSerial.println("INFO: Testing...");
	//TelemSerial.println("Testing telemetry...");
	digitalWrite(LED_PIN, HIGH);   // set the LED on

    testSd();
    //testHighGAccel();
    //testAltimeter();
    //testLowGAccel();
    //testGyro();
    //testGps();
    //testContinuity();
    //testVoltage();
    //testBuzzer();

    //DebugSerial.print("INFO: Pyro Pin ");
    //DebugSerial.println(pyroPin);
	//digitalWrite(pyroPin, HIGH);
	//delay(1000);              // wait for a second
	//digitalWrite(LED_PIN, LOW);    // set the LED off
	//digitalWrite(pyroPin, LOW);

	//pyroPin++;
	//if (pyroPin > PYRO_4_PIN)
    //{
    //    pyroPin = PYRO_1_PIN;
    //}

    delay(25000);
}


void serialEvent2()
{
	char ser;
	while (GpsSerial.available())
	{
		ser = GpsSerial.read();
		if (ser == 13)
		{
			// ignore
		}
		if (ser == 10)
		{
			DebugSerial.println(buffer1);
			buffer1 = "";
		}
		else
		{
			buffer1 += ser;
		}
	}
}

