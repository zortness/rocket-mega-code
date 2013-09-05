/**
***** WARNING *****
* THIS IS UNFINISHED
*
*
* For the RocketMega R2 board on top of an Arduino Mega compatable AVR board.
*/

#define __DEBUG true
#define __RAW false
#define __TELEM_RAW_GPS false
#define __LOG_RAW_GPS false

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

#define ACCEL_MV_TO_G 6.5 // taken from ADXL377 datasheet (can be between 5.8 and 7.2, 6.5 is typical)

#define ACCEL_ALPHA 0.75
#define ACCEL_SAMPLE_RATE 0.1 // in seconds, ie: 10Hz (or 1000ms * rate = sleep time)
#define ALT_ALPHA 0.50
#define ALT_SAMPLE_RATE 0.1
#define LGACC_ALPHA 0.50
#define LGACC_SAMPLE_RATE 0.1
#define MG_ALPHA 0.50
#define GYRO_ALPHA 0.50
#define GYRO_SAMPLE_RATE 0.1
#define GPS_SAMPLE_RATE 1
#define ANALOG_SAMPLE_RATE 1
#define DECISION_SAMPLE_RATE 0.1
#define TELEM_RATE_SLOW 5
#define TELEM_RATE_MED 2.5
#define TELEM_RATE_FAST 0.5

#define RATE_TO_MS(A)   (int)(1000.0 * A)

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
// voltage divider equation: Vout = (Rb / (Ra + Rb)) * Vin
// Ra = 620k
// Rb = 100k
#define VOLTAGE_DIVIDER 0.13888888

#if defined(__AVR_ATmega168__) ||defined(__AVR_ATmega168P__) ||defined(__AVR_ATmega328P__)
  #define VOLTAGE_REF 5.0
  #define V_PER_STEP 0.0048828125 // vref / adc_max
  #define MV_PER_STEP 4.8828125

// Mega 1280 & 2560 (5v)
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  #define VOLTAGE_REF 5.0
  #define V_PER_STEP 0.0048828125
  #define MV_PER_STEP 4.8828125

// Due and others (3.3v)
#else
  #define VOLTAGE_REF 3.3
  #define V_PER_STEP 0.00322265625
  #define MV_PER_STEP 3.22265625

#endif


#define LED_PIN 13
#define BUZZER_PIN 8

#define DEFAULT_SEALEVEL_PRESSURE 1013.21 // in h pascals, 101325

// errors and warnings
#define ERROR_SD 1
#define WARN_SD 1
#define ERROR_ALT 2
#define ERROR_LG_ACC 3
#define ERROR_MAG 4
#define ERROR_GYRO 5

#define ERROR_BEEP_HZ 1850
#define ERROR_BEEP_DUR 350
#define ERROR_BEEP_DEL 300
#define WARN_BEEP_HZ 1915
#define WARN_BEEP_DUR 250
#define WARN_BEEP_DEL 300


// includes
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_GPS.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_L3GD20.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_LSM303.h>
#include <ChibiOS_AVR.h>

// simple container for our progressive filter values
typedef struct {
    float rawVal;
    float filteredVal;
} filter_val_t;

HardwareSerial *DebugSerial = &DEBUG_SERIAL;
HardwareSerial *TelemSerial = &TELEM_SERIAL;
HardwareSerial *GpsSerial = &GPS_SERIAL;
File logFile;
Adafruit_GPS GPS(GpsSerial); // gets read by printing thread
Adafruit_L3GD20 gyro;
Adafruit_BMP085 bmp = Adafruit_BMP085(10085);
Adafruit_LSM303_Mag mag = Adafruit_LSM303_Mag(12345);
Adafruit_LSM303_Accel accel = Adafruit_LSM303_Accel(54321);
float sealevelPressure = DEFAULT_SEALEVEL_PRESSURE;

// volatile stuff for sensor values
volatile filter_val_t hg_x;
volatile filter_val_t hg_y;
volatile filter_val_t hg_z;
volatile filter_val_t lg_x;
volatile filter_val_t lg_y;
volatile filter_val_t lg_z;
volatile filter_val_t mag_x;
volatile filter_val_t mag_y;
volatile filter_val_t mag_z;
volatile filter_val_t gyro_x;
volatile filter_val_t gyro_y;
volatile filter_val_t gyro_z;
volatile filter_val_t temperature;
volatile filter_val_t pressure;
volatile filter_val_t altitude;

// additional volatile vals
volatile int cont1 = 0;
volatile int cont2 = 0;
volatile int cont3 = 0;
volatile int cont4 = 0;
volatile float voltage = 0.0;

float latitude = 0.0;
float longitude = 0.0;

int gpsBufferLoc = 0;
char gpsBuffer[256] = "";

void logStatus();


/**
* Linear version of a complimentary filter.
* Combines new reading from a sensor with last value.
*/
float linearCompFilter(volatile filter_val_t *val, float rawVal, float alpha, float dt)
{
    val->rawVal = rawVal;
    float lastVal = val->filteredVal;
    float diff = lastVal - rawVal;
    // complementary filter
    val->filteredVal = (alpha *                         // high pass filter
                            (lastVal + (diff * dt))     // integration of difference over time
                        ) +
                        ((1.0-alpha) * rawVal);         // low pass filter
    return val->filteredVal;
}

/**
* Beep out a warning.
*/
void beepWarning(int pulses)
{
    for(int i = 0; i < pulses; i++)
    {
        digitalWrite(LED_PIN, HIGH);
        //tone(BUZZER_PIN, WARN_BEEP_HZ, WARN_BEEP_DUR);
        delay(WARN_BEEP_DEL);
        digitalWrite(LED_PIN, LOW);
        delay(WARN_BEEP_DEL);
    }
}

/**
* Beep out an error.
* Option to halt execution.
* Interrupts disabled.
*/
void beepError(int pulses, bool halt)
{
    if (halt)
    {
        noInterrupts();
    }
    while(true)
    {
        for(int i = 0; i < pulses; i++)
        {
            digitalWrite(LED_PIN, HIGH);
            //tone(BUZZER_PIN, ERROR_BEEP_HZ, ERROR_BEEP_DUR);
            delay(ERROR_BEEP_DEL);
            digitalWrite(LED_PIN, LOW);
            delay(ERROR_BEEP_DEL);
        }
        if (!halt)
        {
            return;
        }
        else
        {
            delay(5000);
        }
    }
}

void setupSerials()
{
    TelemSerial->begin(TELEM_BAUD);
    GpsSerial->begin(GPS_BAUD);
    DebugSerial->begin(DEBUG_BAUD);
    #if __DEBUG
    DebugSerial->println("INFO: Serials configured");
    #endif
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
	#if __DEBUG
    DebugSerial->println("INFO: Pins configured");
    #endif
}

void setupSd()
{
    if (!SD.begin(SD_CHIP_SELECT))
	{
		TelemSerial->println("ERROR: SD Card init failed!");
		return beepError(ERROR_SD, true);
	}
	else
	{
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
		    TelemSerial->println("Unable to write to SD");
		    beepWarning(WARN_SD);
		}
		else
		{
		    #if __DEBUG
			DebugSerial->print("Writing to ");
			DebugSerial->println(filename);
			#endif
		}
	}
	#if __DEBUG
	DebugSerial->println("INFO: SD Card initialized");
	#endif
}

void setupHighGAccel()
{
    hg_x.filteredVal = 0;
    hg_y.filteredVal = 0;
    hg_z.filteredVal = 0;
    // seed filter data
    //hg_x.filteredVal = (float)analogRead(ACCEL_X_PIN);
    //hg_y.filteredVal = (float)analogRead(ACCEL_Y_PIN);
    //hg_z.filteredVal = (float)analogRead(ACCEL_Z_PIN);
}

void readHighGAccel()
{
    // x
    float sample = (float)analogRead(ACCEL_X_PIN);
    sample *= MV_PER_STEP; // now in mV
    sample /= ACCEL_MV_TO_G; // now in G's
    linearCompFilter(&hg_x, sample, ACCEL_ALPHA, ACCEL_SAMPLE_RATE);
    // y
    sample = (float)analogRead(ACCEL_Y_PIN);
    sample *= MV_PER_STEP;
    sample /= ACCEL_MV_TO_G;
    linearCompFilter(&hg_y, sample, ACCEL_ALPHA, ACCEL_SAMPLE_RATE);
    // z
    sample = (float)analogRead(ACCEL_Z_PIN);
    sample *= MV_PER_STEP;
    sample /= ACCEL_MV_TO_G;
    linearCompFilter(&hg_z, sample, ACCEL_ALPHA, ACCEL_SAMPLE_RATE);
}

void setupAltimeter()
{
    if (!bmp.begin(BMP085_MODE_ULTRALOWPOWER))
	{
		DebugSerial->println("ERROR: Unable to initialize altimeter");
		beepError(ERROR_ALT, true);
	}
	pressure.filteredVal = 0;
	temperature.filteredVal = 0;
	altitude.filteredVal = 0;
	#if __DEBUG
	DebugSerial->println("INFO: Altimeter initialized");
	#endif
}

void readAltimeter()
{
    sensors_event_t event;
	bmp.getEvent(&event);
	float rawTemperature;
	bmp.getTemperature(&rawTemperature);
    linearCompFilter(&pressure, event.pressure, ALT_ALPHA, ALT_SAMPLE_RATE);
    linearCompFilter(&temperature, rawTemperature, ALT_ALPHA, ALT_SAMPLE_RATE);
    linearCompFilter(&altitude, bmp.pressureToAltitude(sealevelPressure, event.pressure, rawTemperature),
                     ALT_ALPHA, ALT_SAMPLE_RATE);
}

void setupLowGAccel()
{
    if (!accel.begin())
    {
        TelemSerial->println("ERROR: Unable to initialize Low-G Accelerometer");
        beepError(ERROR_LG_ACC, true);
        return;
    }
    if (!mag.begin())
    {
        TelemSerial->println("ERROR: Unable to initialize Magnetometer");
        beepError(ERROR_MAG, true);
        return;
    }
    lg_x.filteredVal = 0;
    lg_y.filteredVal = 0;
    lg_z.filteredVal = 0;
    mag_x.filteredVal = 0;
    mag_y.filteredVal = 0;
    mag_z.filteredVal = 0;
    #if __DEBUG
    DebugSerial->println("INFO: Low-G Accelerometer and Magnetometer initialized");
    #endif
}

void readLowGAccel()
{
    sensors_event_t event;
    accel.getEvent(&event);
    linearCompFilter(&lg_x, event.acceleration.x, LGACC_ALPHA, LGACC_SAMPLE_RATE);
    linearCompFilter(&lg_y, event.acceleration.y, LGACC_ALPHA, LGACC_SAMPLE_RATE);
    linearCompFilter(&lg_z, event.acceleration.z, LGACC_ALPHA, LGACC_SAMPLE_RATE);
    mag.getEvent(&event);
    linearCompFilter(&mag_x, event.magnetic.x, LGACC_ALPHA, LGACC_SAMPLE_RATE);
    linearCompFilter(&mag_y, event.magnetic.y, LGACC_ALPHA, LGACC_SAMPLE_RATE);
    linearCompFilter(&mag_z, event.magnetic.z, LGACC_ALPHA, LGACC_SAMPLE_RATE);
}

void setupGyro()
{
    if (!gyro.begin())
	{
		TelemSerial->println("ERROR: Unable to initialize gyro");
		beepError(ERROR_GYRO, true);
		return;
	}
	gyro_x.filteredVal = 0;
	gyro_y.filteredVal = 0;
	gyro_z.filteredVal = 0;
	#if __DEBUG
	DebugSerial->println("INFO: Gyro initialized");
	#endif
}

void readGyro()
{
    gyro.read();
    linearCompFilter(&gyro_x, gyro.data.x, GYRO_ALPHA, GYRO_SAMPLE_RATE);
    linearCompFilter(&gyro_y, gyro.data.y, GYRO_ALPHA, GYRO_SAMPLE_RATE);
    linearCompFilter(&gyro_z, gyro.data.z, GYRO_ALPHA, GYRO_SAMPLE_RATE);
}

void setupGps()
{
	GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
	GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
	GPS.sendCommand(PGCMD_ANTENNA);
	#if __DEBUG
	DebugSerial->println("INFO: GPS initialized");
	#endif
}

void testContinuity()
{
    cont1 = analogRead(PYRO_1_CONT_PIN);
    cont2 = analogRead(PYRO_2_CONT_PIN);
    cont3 = analogRead(PYRO_3_CONT_PIN);
    cont4 = analogRead(PYRO_4_CONT_PIN);
}

void testVoltage()
{
    float v = (float)analogRead(VOLTAGE_PIN);
    v *= V_PER_STEP;
    v /= VOLTAGE_DIVIDER;
    voltage = v;
}

/**
* Read/parse data from the GPS serial line.
* Side-effect of updating the GPS object.
*/
static WORKING_AREA(gpsThreadWa, 128);
static msg_t GpsThread(void *arg)
{
    char ser;
    while(true)
    {
        while (GpsSerial->available())
        {
            ser = GpsSerial->read();
            if (ser == 13)
            {
                // ignore
            }
            if (ser == 10)
            {
                gpsBuffer[gpsBufferLoc] = '\0';
                #if __DEBUG
                //DebugSerial->print(gpsBuffer);
                #endif
                if (GPS.parse(gpsBuffer))
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
                gpsBufferLoc = 0;
            }
            else
            {
                gpsBuffer[gpsBufferLoc] = ser;
                gpsBufferLoc++;
            }
        }
        // wait 1 second from the last parse, timing is not critical, so it doesn't matter if we drift a bit
        chThdSleepUntil(chTimeNow() + MS2ST( RATE_TO_MS(GPS_SAMPLE_RATE) ));
    }
}

/**
* Read from the 200G accelerometer.
* Updates all associated values (hg_).
*/
static WORKING_AREA(hgAccelThreadWa, 32);
static msg_t HighGAccelThread(void *arg)
{
    systime_t time = chTimeNow();
    while(true)
    {
        time += MS2ST( RATE_TO_MS(ACCEL_SAMPLE_RATE));
        readHighGAccel();
        chThdSleepUntil(time);
    }
}

/**
* Read from the 16G accelerometer + magnometer.
* Updates all associated values (lg_ and mag_)
*/
static WORKING_AREA(lgAccelThreadWa, 128);
static msg_t LowGAccelThread(void *arg)
{
    systime_t time = chTimeNow();
    while(true)
    {
        time += MS2ST( RATE_TO_MS(LGACC_SAMPLE_RATE));
        readLowGAccel();
        chThdSleepUntil(time);
    }
}

/**
* Read from the Gyro.
* Updates all associated values (gyro_)
*/
static WORKING_AREA(gyroThreadWa, 128);
static msg_t GyroThread(void *arg)
{
    systime_t time = chTimeNow();
    while(true)
    {
        time += MS2ST( RATE_TO_MS(GYRO_SAMPLE_RATE));
        readGyro();
        chThdSleepUntil(time);
    }
}

/**
* Read from the Altimeter.
* Updates all associated values (altitude, pressure, temperature)
*/
static WORKING_AREA(altimeterThreadWa, 160);
static msg_t AltimeterThread(void *arg)
{
    systime_t time = chTimeNow();
    while(true)
    {
        time += MS2ST(RATE_TO_MS(ALT_SAMPLE_RATE));
        readAltimeter();
        chThdSleepUntil(time);
    }
}

/**
* Read analog values for continuity, voltage.
* Updates all associated values (cont_, voltage)
*/
static WORKING_AREA(analogThreadWa, 128);
static msg_t AnalogThread(void *arg)
{
    systime_t time = chTimeNow();
    while(true)
    {
        time += MS2ST(RATE_TO_MS(ANALOG_SAMPLE_RATE));
        readAltimeter();
        chThdSleepUntil(time);
    }
}

/**
* Decision making thread. This is what controls our current state.
*/
static WORKING_AREA(decisionThreadWa, 128);
static msg_t DecisionThread(void *arg)
{
    systime_t time = chTimeNow();
    while(true)
    {
        time += MS2ST(RATE_TO_MS(ANALOG_SAMPLE_RATE));
        TelemSerial->println("making decisions");
        chThdSleepUntil(time);
    }
}

/**
* Main thread... starts everything else and dumps data.
*/
void mainThread()
{
    systime_t time = chTimeNow();
    // start all threads
    chThdCreateStatic(hgAccelThreadWa, sizeof(hgAccelThreadWa), NORMALPRIO, HighGAccelThread, NULL); // high g
    delay(10);
    chThdCreateStatic(lgAccelThreadWa, sizeof(lgAccelThreadWa), NORMALPRIO, LowGAccelThread, NULL); // low g
    delay(10);
    chThdCreateStatic(gyroThreadWa, sizeof(gyroThreadWa), NORMALPRIO, GyroThread, NULL); // gyro
    delay(10);
    chThdCreateStatic(altimeterThreadWa, sizeof(altimeterThreadWa), NORMALPRIO, AltimeterThread, NULL); // alt
    delay(10);
    chThdCreateStatic(gpsThreadWa, sizeof(gpsThreadWa), NORMALPRIO, GpsThread, NULL); // gps
    delay(10);
    chThdCreateStatic(analogThreadWa, sizeof(analogThreadWa), NORMALPRIO, AnalogThread, NULL); // analog

    // let sensor threads run for a few seconds to stabilize
    chThdSleepUntil(time += MS2ST(2500));
    // start decision thread
    chThdCreateStatic(decisionThreadWa, sizeof(decisionThreadWa), HIGHPRIO, DecisionThread, NULL);

    while(true)
    {
        // send telemetry data
        // write log data to sd file


        logStatus();

        // timing is not critical, so it doesn't matter if we drift a bit
        chThdSleepUntil(chTimeNow() + MS2ST( RATE_TO_MS(GPS_SAMPLE_RATE) ));
    }
}

void setup()
{
	setupSerials();
	setupPins();
    setupHighGAccel();
    setupAltimeter();
    setupLowGAccel();
    setupGyro();
    setupGps();
    #if __DEBUG
    DebugSerial->println("INFO: Setup complete");
    #endif
    delay(250);
    chBegin(mainThread);
    while(true){}
}


void loop()
{
	// not used
}

void logStatus()
{
    #if __DEBUG
    DebugSerial->print("Unused Stack: hga:");
    DebugSerial->print(chUnusedStack(hgAccelThreadWa, sizeof(hgAccelThreadWa)));
    DebugSerial->print(", lga:");
    DebugSerial->print(chUnusedStack(lgAccelThreadWa, sizeof(lgAccelThreadWa)));
    DebugSerial->print(", gyro:");
    DebugSerial->print(chUnusedStack(gyroThreadWa, sizeof(gyroThreadWa)));
    DebugSerial->print(", alt:");
    DebugSerial->print(chUnusedStack(altimeterThreadWa, sizeof(altimeterThreadWa)));
    DebugSerial->print(", gps:");
    DebugSerial->print(chUnusedStack(gpsThreadWa, sizeof(gpsThreadWa)));
    DebugSerial->print(", ana:");
    DebugSerial->print(chUnusedStack(analogThreadWa, sizeof(analogThreadWa)));
    DebugSerial->print(", dec:");
    DebugSerial->print(chUnusedStack(decisionThreadWa, sizeof(decisionThreadWa)));
    DebugSerial->print(", main:");
    DebugSerial->println(chUnusedHeapMain());
    #endif

    long t = millis();
	char alt[16];
	dtostrf(altitude.filteredVal, 5, 2, alt);
	char prs[16];
	dtostrf(pressure.filteredVal, 8, 2, prs);
	char tem[16];
	dtostrf(temperature.filteredVal, 4, 2, tem);

	char hgx[16];
	dtostrf(hg_x.filteredVal, 4, 2, hgx);
	char hgy[16];
	dtostrf(hg_y.filteredVal, 4, 2, hgy);
	char hgz[16];
	dtostrf(hg_z.filteredVal, 4, 2, hgz);

	char lgx[16];
	dtostrf(lg_x.filteredVal, 4, 2, lgx);
	char lgy[16];
	dtostrf(lg_y.filteredVal, 4, 2, lgy);
	char lgz[16];
	dtostrf(lg_z.filteredVal, 4, 2, lgz);

	char magx[16];
	dtostrf(mag_x.filteredVal, 4, 2, magx);
	char magy[16];
	dtostrf(mag_y.filteredVal, 4, 2, magy);
	char magz[16];
	dtostrf(mag_z.filteredVal, 4, 2, magz);

	char gyrox[16];
	dtostrf(gyro_x.filteredVal, 4, 2, gyrox);
	char gyroy[16];
	dtostrf(gyro_y.filteredVal, 4, 2, gyroy);
	char gyroz[16];
	dtostrf(gyro_z.filteredVal, 4, 2, gyroz);

	char vlt[16];
	dtostrf(voltage, 4, 2, vlt);

	String printBuffer = "$T{";

	//printBuffer += mode; printBuffer += ',';
	//printBuffer += t; printBuffer += ',';
	printBuffer += alt; printBuffer += ',';
	printBuffer += hgx; printBuffer += ',';
	printBuffer += hgy; printBuffer += ',';
	printBuffer += hgz; printBuffer += ',';
	printBuffer += lgx; printBuffer += ',';
	printBuffer += lgy; printBuffer += ',';
	printBuffer += lgz; printBuffer += ',';
	printBuffer += magx; printBuffer += ',';
	printBuffer += magy; printBuffer += ',';
	printBuffer += magz; printBuffer += ',';
	printBuffer += gyrox; printBuffer += ',';
	printBuffer += gyroy; printBuffer += ',';
	printBuffer += gyroz; printBuffer += ',';
	printBuffer += tem; printBuffer += ',';
	printBuffer += prs; printBuffer += ',';
	printBuffer += cont1; printBuffer += ',';
	printBuffer += cont2; printBuffer += ',';
	printBuffer += cont3; printBuffer += ',';
	printBuffer += cont4; printBuffer += ',';
	printBuffer += vlt; printBuffer += ',';
	printBuffer += (int)GPS.fix; //printBuffer += '}';

	if (GPS.fix)
	{
		char lat[16];
		dtostrf(latitude, 10, 6, lat);
		char lon[16];
		dtostrf(longitude, 10, 6, lon);
		char spd[16];
		dtostrf(GPS.speed, 6, 2, spd);
		char ang[16];
		dtostrf(GPS.angle, 6, 2, ang);
		char galt[16];
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

	// raw values
	#if __RAW
    printBuffer += "\r\n$R{";
    char ralt[16];
    dtostrf(altitude.rawVal, 5, 2, ralt);
    char rprs[16];
    dtostrf(pressure.rawVal, 8, 2, rprs);
    char rtem[16];
    dtostrf(temperature.rawVal, 4, 2, rtem);

    char rxf[16];
    dtostrf(hg_x.rawVal, 4, 2, rxf);
    char ryf[16];
    dtostrf(hg_y.rawVal, 4, 2, ryf);
    char rzf[32];
    dtostrf(hg_z.rawVal, 4, 2, rzf);

    char rlxf[16];
    dtostrf(lg_x.rawVal, 4, 2, rlxf);
    char rlyf[16];
    dtostrf(lg_y.rawVal, 4, 2, rlyf);
    char rlzf[16];
    dtostrf(lg_z.rawVal, 4, 2, rlzf);

    char rxm[16];
    dtostrf(mag_x.rawVal, 4, 2, rxm);
    char rym[16];
    dtostrf(mag_y.rawVal, 4, 2, rym);
    char rzm[16];
    dtostrf(mag_z.rawVal, 4, 2, rzm);

    char rxr[16];
    dtostrf(gyro_x.rawVal, 4, 2, rxr);
    char ryr[16];
    dtostrf(gyro_y.rawVal, 4, 2, ryr);
    char rzr[16];
    dtostrf(gyro_z.rawVal, 4, 2, rzr);

    //printBuffer += mode; printBuffer += ',';
    //printBuffer += t; printBuffer += ',';
    printBuffer += ralt; printBuffer += ',';
    printBuffer += rxf; printBuffer += ',';
    printBuffer += ryf; printBuffer += ',';
    printBuffer += rzf; printBuffer += ',';
    printBuffer += rlxf; printBuffer += ',';
    printBuffer += rlyf; printBuffer += ',';
    printBuffer += rlzf; printBuffer += ',';
    printBuffer += rxr; printBuffer += ',';
    printBuffer += ryr; printBuffer += ',';
    printBuffer += rzr; printBuffer += ',';
    printBuffer += rxm; printBuffer += ',';
    printBuffer += rym; printBuffer += ',';
    printBuffer += rzm; printBuffer += ',';
    printBuffer += rtem; printBuffer += ',';
    printBuffer += rprs;

    printBuffer += '}';
    #endif

    #if __DEBUG
    DebugSerial->println(printBuffer);
    #endif
    TelemSerial->println(printBuffer);

    // TODO: log to file as well
}
