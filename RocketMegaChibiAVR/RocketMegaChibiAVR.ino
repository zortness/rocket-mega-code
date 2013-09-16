/**
***** WARNING *****
* THIS IS UNFINISHED
*
*
* For the RocketMega R2 board on top of an Arduino Mega compatable AVR board.
*/

#define __DEBUG true
#define __DEBUG_STACK true
#define __DEBUG_GPS false
#define __RAW true
#define __TELEM_RAW_GPS false
#define __LOG_RAW_GPS false

#define DEBUG_SERIAL Serial
#define DEBUG_BAUD 57600
#define TELEM_SERIAL Serial1
#define TELEM_BAUD 57600
#define GPS_BAUD 9600
#define GPS_SERIAL Serial2
#define GPS_BUFFER_SIZE 256

#define SD_CHIP_SELECT 48
//#define SPI_MISO_PIN 50
//#define SPI_MOSI_PIN 51
//#define SPI_SCLK_PIN 52

#define ACCEL_X_PIN A8
#define ACCEL_Y_PIN A9
#define ACCEL_Z_PIN A10
// taken from ADXL377 datasheet (can be between 5.8 and 7.2, 6.5 is typical)
#define ACCEL_MV_TO_G 6.5F


#define ACCEL_ALPHA 0.95F
#define ACCEL_SAMPLE_RATE 0.1F // in seconds, ie: 10Hz (or 1000ms * rate = sleep time)
#define ALT_ALPHA 0.50F
#define ALT_SAMPLE_RATE 0.1F
#define LGACC_ALPHA 0.50F
#define LGACC_SAMPLE_RATE 0.1F
#define MG_ALPHA 0.50F
#define GYRO_ALPHA 0.50F
#define GYRO_SAMPLE_RATE 0.1F
#define GPS_SAMPLE_RATE 0.2F
#define ANALOG_SAMPLE_RATE 2.0F
#define DECISION_SAMPLE_RATE 0.1F
#define TELEM_RATE_SLOW 5.0F
#define TELEM_RATE_MED 1.0F
#define TELEM_RATE_FAST 0.5F

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
  #define VOLTAGE_REF 5.0F
  #define V_PER_STEP 0.0048828125F // vref / adc_max
  #define MV_PER_STEP 4.8828125F
  #define ACCEL_X_CENTER 337
  #define ACCEL_Y_CENTER 337
  #define ACCEL_Z_CENTER 337
  // for 10-bit ADC, range is 0-1023
  // ((3.3/5.0) * 1023) / 2 = 337.59 -> center point
  // 200/337.59 = 0.59243761002991 G's per step
  #define G_PER_STEP 0.59243761F

// Mega 1280 & 2560 (5v)
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  #define VOLTAGE_REF 5.0F
  #define V_PER_STEP 0.0048828125F
  #define MV_PER_STEP 4.8828125F
  #define ACCEL_X_CENTER 337
  #define ACCEL_Y_CENTER 337
  #define ACCEL_Z_CENTER 337
  #define G_PER_STEP 0.59243761F

// Due and others (3.3v)
#else
  #define VOLTAGE_REF 3.3F
  #define V_PER_STEP 0.00322265625F
  #define MV_PER_STEP 3.22265625F
  #define ACCEL_X_CENTER 512
  #define ACCEL_Y_CENTER 512
  #define ACCEL_Z_CENTER 512
  // for 10-bit ADC (default setting), range is 0-1023
  // 1024 / 2 = 512 -> center point
  // 200/512 = 0.390625 G's per step
  #define G_PER_STEP 0.390625F
#endif


#define LED_PIN 13
#define BUZZER_PIN 8

#define DEFAULT_ASCENT_THRESHOLD 45.0F          // in m/s^2
#define DEFAULT_ALT_ASCENT_THRESHOLD 25.0F      // in meters from starting altitude
#define DEFAULT_ALT_VEL_THRESHOLD 20.0F         // in meters / second
#define DEFAULT_APOGEE_THRESHOLD 15.0F          // in m/s^2
#define DEFAULT_ALT_VEL_APOGEE_THRESHOLD -5.0F  // in meters / second
#define DEFAULT_TOUCHDOWN_THRESHOLD 15.0F       // in m/s^2
#define DEFAULT_MAIN_ALTITUDE 300.0F            // in meters (added to deck altitude)
#define DEFAULT_SEALEVEL_PRESSURE 1013.21F      // in h pascals, 101325

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

#define MODE_ERROR 999
#define MODE_STARTUP 0
#define MODE_READY 1
#define MODE_ASCENT 2
#define MODE_APOGEE 3
#define MODE_DESCENT 4
#define MODE_DESCENT2 5
#define MODE_TOUCHDOWN 6
#define MODE_PAUSE 7


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
boolean logSd = false;
Adafruit_GPS GPS(GpsSerial); // gets read by printing thread
Adafruit_L3GD20 gyro;
Adafruit_BMP085 bmp = Adafruit_BMP085(10085);
Adafruit_LSM303_Mag mag = Adafruit_LSM303_Mag(12345);
Adafruit_LSM303_Accel accel = Adafruit_LSM303_Accel(54321);

// volatile stuff for sensor values
volatile filter_val_t hg_x;             // in G's (* ~9.8 m/s^2)
volatile filter_val_t hg_y;             // in G's
volatile filter_val_t hg_z;             // in G's
volatile filter_val_t lg_x;             // in G's
volatile filter_val_t lg_y;             // in G's
volatile filter_val_t lg_z;             // in G's
volatile filter_val_t mag_x;            // in micro Tesla (uT)
volatile filter_val_t mag_y;            // in micro Tesla (uT)
volatile filter_val_t mag_z;            // in micro Tesla (uT)
volatile filter_val_t gyro_x;           // angular degrees per second (d/s)
volatile filter_val_t gyro_y;           // angular degrees per second (d/s)
volatile filter_val_t gyro_z;           // angular degrees per second (d/s)
volatile filter_val_t temperature;      // in celcius (C)
volatile filter_val_t pressure;         // in hectapascals (hPa)
volatile filter_val_t altitude;         // in meters (m)
volatile filter_val_t alt_vel;          // in meters per second (m/s), based on delta altitude

// additional volatile vals
volatile int cont1 = 0;
volatile int cont2 = 0;
volatile int cont3 = 0;
volatile int cont4 = 0;
volatile float voltage = 0.0F;

float latitude = 0.0F;
float longitude = 0.0F;

int gpsBufferLoc = 0;
char gpsBuffer[GPS_BUFFER_SIZE];

static String printBuffer;

float sealevelPressure = DEFAULT_SEALEVEL_PRESSURE;
float startingAltitude = 0.0F;
float maxAltitude = -1000.0F;
float maxVelocity = 0.0F;
float ascentThreshold = DEFAULT_ASCENT_THRESHOLD;
float ascentAltThreshold = DEFAULT_ALT_ASCENT_THRESHOLD;
float ascentVelThreshold = DEFAULT_ALT_VEL_THRESHOLD;
float apogeeThreshold = DEFAULT_APOGEE_THRESHOLD;
float apogeeVelThreshold = DEFAULT_ALT_VEL_APOGEE_THRESHOLD;
float touchdownThreshold = DEFAULT_TOUCHDOWN_THRESHOLD;
float mainDeployAltitude = DEFAULT_MAIN_ALTITUDE;
unsigned long launchTime = 0L;
unsigned long apogeeTime = 0L;
unsigned long mainTime = 0L;
unsigned long touchdownTime = 0L;
float maxForce = 0.0F;
float minTemperature = 0.0F;
float maxTemperature = 0.0F;
float minPressure = DEFAULT_SEALEVEL_PRESSURE; // we set this to the first reading anyway, but just to be safe
float maxPressure = 0.0F;
bool statsOut = false;

// used for tracking times that relays are activated
unsigned long drogueDeployStart = 0;
unsigned long mainDeployStart = 0;

volatile uint8_t mode = MODE_STARTUP;

void logStatus();
void logStatusTd();
void logStats();
void modeReady();
void modeAscent();
void modeApogee();
void modeDescent();
void modeDescent2();
void modeTouchdown();
void deployMain();
void deployDrogue();

/**
* Linear version of a complimentary filter.
* Combines new reading from a sensor with last value.
*/
float linearCompFilter(volatile filter_val_t *val, float rawVal, float alpha, float dt)
{
    val->rawVal = rawVal;
    float lastVal = val->filteredVal;
    float diff = rawVal - lastVal;
    // complementary filter
    // .95 * (10 + (3 * 0.1)) = .95 * 10.3 = 9.785
    // .05 * 13 = .65
    // 9.785 + .65 = 10.435
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
            mode = MODE_ERROR;
            delay(5000);
        }
    }
}

/**
* Set up serial lines.
*/
void setupSerials()
{
    TelemSerial->begin(TELEM_BAUD);
    GpsSerial->begin(GPS_BAUD);
    DebugSerial->begin(DEBUG_BAUD);
    #if __DEBUG
    DebugSerial->println("INFO: Serials configured");
    #endif
}

/**
* Set up our GPIO pins.
*/
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

/**
* Set up our SD card, create file for this run.
*/
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
		    TelemSerial->println("ERROR: Unable to write to SD");
		    #if __DEBUG
			DebugSerial->println("ERROR: Unable to write to SD");
			#endif
		    beepWarning(WARN_SD);
		    return;
		}
		else
		{
		    logSd = true;
		    #if __DEBUG
			DebugSerial->print("INFO: Writing to ");
			DebugSerial->println(filename);
			#endif
		}
	}
	#if __DEBUG
	DebugSerial->println("INFO: SD Card initialized");
	#endif
}

/**
* Set up the High G Analog Accelerometer.
*/
void setupHighGAccel()
{
    hg_x.filteredVal = 0;
    hg_y.filteredVal = 0;
    hg_z.filteredVal = 0;
}

/**
* Sample from the High G Accelerometer.
*/
void readHighGAccel()
{
    // x
    float sample = (float)(analogRead(ACCEL_X_PIN) - ACCEL_X_CENTER);
    sample *= G_PER_STEP;
    linearCompFilter(&hg_x, sample, ACCEL_ALPHA, ACCEL_SAMPLE_RATE);
    if (abs(hg_x.filteredVal) > maxForce)
    {
        maxForce = abs(hg_x.filteredVal);
    }
    // y
    sample = (float)(analogRead(ACCEL_Y_PIN) - ACCEL_Y_CENTER);
    sample *= G_PER_STEP;
    linearCompFilter(&hg_y, sample, ACCEL_ALPHA, ACCEL_SAMPLE_RATE);
    if (abs(hg_y.filteredVal) > maxForce)
    {
        maxForce = abs(hg_y.filteredVal);
    }
    // z
    sample = (float)(analogRead(ACCEL_Z_PIN) - ACCEL_Z_CENTER);
    sample *= G_PER_STEP;
    linearCompFilter(&hg_z, sample, ACCEL_ALPHA, ACCEL_SAMPLE_RATE);
    if (abs(hg_z.filteredVal) > maxForce)
    {
        maxForce = abs(hg_z.filteredVal);
    }
}

/**
* Set up Barometric Pressure / Temperature / Altimeter.
*/
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

/**
* Sample from Altimeter module.
*/
void readAltimeter()
{
    sensors_event_t event;
	bmp.getEvent(&event);
	float rawTemperature;
	bmp.getTemperature(&rawTemperature);
    linearCompFilter(&pressure, event.pressure, ALT_ALPHA, ALT_SAMPLE_RATE);
    linearCompFilter(&temperature, rawTemperature, ALT_ALPHA, ALT_SAMPLE_RATE);
    float lastAlt = altitude.filteredVal;
    linearCompFilter(&altitude, bmp.pressureToAltitude(sealevelPressure, event.pressure, rawTemperature),
                     ALT_ALPHA, ALT_SAMPLE_RATE);
    if (altitude.filteredVal > maxAltitude)
    {
        maxAltitude = altitude.filteredVal;
    }
    if (pressure.filteredVal > maxPressure)
    {
        maxPressure = pressure.filteredVal;
    }
    if (pressure.filteredVal < minPressure)
    {
        minPressure = pressure.filteredVal;
    }
    if (temperature.filteredVal > maxTemperature)
    {
        maxTemperature = temperature.filteredVal;
    }
    if (temperature.filteredVal < minTemperature)
    {
        minTemperature = temperature.filteredVal;
    }
    if (lastAlt != 0)
    {
        linearCompFilter(&alt_vel, ((altitude.filteredVal - lastAlt) / ALT_SAMPLE_RATE),
                     ALT_ALPHA, ALT_SAMPLE_RATE);
        if (alt_vel.filteredVal > maxVelocity)
        {
            maxVelocity = alt_vel.filteredVal;
        }
    }
}

/**
* Set up the Low G Accelerometer and Magnetometer (shared module).
*/
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

/**
* Sample from the Low G Accelerometer and Magnetometer.
*/
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

/**
* Set up the Gyro.
*/
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

/**
* Read from the Gryo.
*/
void readGyro()
{
    gyro.read();
    linearCompFilter(&gyro_x, gyro.data.x, GYRO_ALPHA, GYRO_SAMPLE_RATE);
    linearCompFilter(&gyro_y, gyro.data.y, GYRO_ALPHA, GYRO_SAMPLE_RATE);
    linearCompFilter(&gyro_z, gyro.data.z, GYRO_ALPHA, GYRO_SAMPLE_RATE);
}

/**
* Set up the GPS.
*/
void setupGps()
{
	GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
	GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
	GPS.sendCommand(PGCMD_ANTENNA);
	#if __DEBUG
	DebugSerial->println("INFO: GPS initialized");
	#endif
}

/**
* Read from the GPS serial buffer.
*/
void readGps()
{
    char ser;
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
            #if __DEBUG_GPS
            DebugSerial->print(gpsBuffer);
            #endif
            #if __LOG_RAW_GPS
            if (logSd)
            {
                logFile.print(gpsBuffer);
            }
            #endif // __LOG_RAW_GPS
            #if __TELEM_RAW_GPS
            TelemSerial->print(gpsBuffer);
            #endif // __TELEM_RAW_GPS
            // ignore $PGTOP messages (device specific)
            //if (!gpsBuffer[1] == 'P')
            //{
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
                else
                {
                    #if __DEBUG
                    //DebugSerial->println("ERROR: error parsing GPS sentence");
                    #endif
                }
            //}
            gpsBufferLoc = 0;
        }
        else if (gpsBufferLoc >= GPS_BUFFER_SIZE)
        {
            #if __DEBUG
            DebugSerial->println("ERROR: GPS buffer overrun");
            #endif
            gpsBuffer[GPS_BUFFER_SIZE - 1] = '\0';
            gpsBufferLoc = 0;
        }
        else
        {
            gpsBuffer[gpsBufferLoc] = ser;
            gpsBufferLoc++;
        }
    }
}

/**
* Test the analog continuity lines for voltage.
*/
void testContinuity()
{
    cont1 = analogRead(PYRO_1_CONT_PIN);
    cont2 = analogRead(PYRO_2_CONT_PIN);
    cont3 = analogRead(PYRO_3_CONT_PIN);
    cont4 = analogRead(PYRO_4_CONT_PIN);
}

/**
* Test our input voltage analog line (at about 6:1).
*/
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
static WORKING_AREA(gpsThreadWa, 32);
static msg_t GpsThread(void *arg)
{
    systime_t time = chTimeNow();
    while(true)
    {
        time += MS2ST( RATE_TO_MS(GPS_SAMPLE_RATE));
        readGps();
        chThdSleepUntil(time);
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
static WORKING_AREA(gyroThreadWa, 64);
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
static WORKING_AREA(analogThreadWa, 32);
static msg_t AnalogThread(void *arg)
{
    systime_t time = chTimeNow();
    while(true)
    {
        time += MS2ST(RATE_TO_MS(ANALOG_SAMPLE_RATE));
        testContinuity();
        testVoltage();
        chThdSleepUntil(time);
    }
}

/**
* Decision thread. Runs at High Priority. Makes decisions.
*/
static WORKING_AREA(decisionThreadWa, 256);
static msg_t DecisionThread(void *arg)
{
    systime_t time = chTimeNow();
    while(true)
    {
        time += MS2ST(RATE_TO_MS(DECISION_SAMPLE_RATE));

        switch(mode)
        {
            case MODE_READY: modeReady(); break;
            case MODE_ASCENT: modeAscent(); break;
            case MODE_APOGEE: modeApogee(); break;
            case MODE_DESCENT: modeDescent(); break;
            case MODE_DESCENT2: modeDescent2(); break;
            case MODE_TOUCHDOWN: modeTouchdown(); break;
        }
        chThdSleepUntil(time);
    }
}

/**
* Main thread... starts everything else and sends out telemtry.
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
    chThdSleepUntil(time += MS2ST(5000));

    startingAltitude = altitude.filteredVal;

    #if __DEBUG
    DebugSerial->print("Starting altitude: ");
    DebugSerial->println(altitude.filteredVal);
    #endif
    TelemSerial->print("Starting altitude: ");
    TelemSerial->println(altitude.filteredVal);

    // TODO: beep continuities, etc?

    mode = MODE_READY;

    // start decision thread
    chThdCreateStatic(decisionThreadWa, sizeof(decisionThreadWa), HIGHPRIO, DecisionThread, NULL);

    time = chTimeNow();
    while(true)
    {
        switch(mode)
        {
            case MODE_READY:
                time += MS2ST(RATE_TO_MS(TELEM_RATE_MED));
                break;
            case MODE_ASCENT:
                time += MS2ST(RATE_TO_MS(TELEM_RATE_FAST));
                break;
            case MODE_APOGEE:
            case MODE_DESCENT:
            case MODE_DESCENT2:
                time += MS2ST(RATE_TO_MS(TELEM_RATE_MED));
                break;
            case MODE_TOUCHDOWN:
                time += MS2ST(RATE_TO_MS(TELEM_RATE_SLOW));
                break;
            default:
                time += MS2ST(RATE_TO_MS(TELEM_RATE_SLOW));
                break;
        }

        // check relays
        unsigned long t = millis();
        if (drogueDeployStart > 0 && (drogueDeployStart + PYRO_ON_TIME) < t)
        {
            digitalWrite(PYRO_2_PIN, LOW);
            drogueDeployStart = 0;
        }
        if (mainDeployStart > 0 && (mainDeployStart + PYRO_ON_TIME) < t)
        {
            digitalWrite(PYRO_1_PIN, LOW);
            mainDeployStart = 0;
        }

        // toggle LED
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));

        if (mode == MODE_TOUCHDOWN)
        {
            logStatusTd();
        }
        else
        {
            logStatus();
        }
        chThdSleepUntil(time);
    }
}

/**
* Setup everything.
* Part of the Arduino library.
*/
void setup()
{
	setupSerials();
	setupPins();
    setupHighGAccel();
    setupAltimeter();
    setupLowGAccel();
    setupGyro();
    setupGps();
    setupSd();

    printBuffer.reserve(400);

    #if __DEBUG
    DebugSerial->println("INFO: Setup complete");
    #endif
    delay(250);
    chBegin(mainThread);
    while(true){}
}
void loop(){/* not used, part of Arduino library, obsolete with ChibiOS */}



/**
* Ready mode, looking for liftoff condition.
*/
void modeReady()
{
    // using the more sensitive (and less noisy) low G digital accelerometer
    // also testing the starting altitude vs current altitude
	if( lg_x.filteredVal > ascentThreshold
		|| lg_y.filteredVal > ascentThreshold
		|| lg_z.filteredVal > ascentThreshold
		|| altitude.filteredVal > startingAltitude + ascentAltThreshold
		|| alt_vel.filteredVal > ascentVelThreshold)
	{
		mode = MODE_ASCENT;
		launchTime = millis();
		String msg = "INFO: Launch Detected ";
		if( lg_x.filteredVal > ascentThreshold ) msg += "X";
		if( lg_y.filteredVal > ascentThreshold ) msg += "Y";
		if( lg_z.filteredVal > ascentThreshold ) msg += "Z";
		if( altitude.filteredVal > startingAltitude + ascentAltThreshold ) msg += "ALT";
		if( alt_vel.filteredVal > ascentVelThreshold ) msg += "VEL";
		#if __DEBUG
        DebugSerial->println(msg);
        #endif
		TelemSerial->println(msg);
		if (logSd)
		{
			logFile.println(msg);
			logFile.flush();
		}
		return;
	}
}

/**
* Ascent mode, looking for apogee condition.
*/
void modeAscent()
{
	// constantly check accelerometer for levelling off or swing in direction of force
	// check altimeter for drops
	// switch to apogee mode

	if( (lg_x.filteredVal < apogeeThreshold
		&& lg_y.filteredVal < apogeeThreshold
		&& lg_z.filteredVal < apogeeThreshold)
        ||
        alt_vel.filteredVal < apogeeVelThreshold
		)
	{
		mode = MODE_APOGEE;
		apogeeTime = millis();
		String msg = "INFO: Apogee Detected";
		if( lg_x.filteredVal < apogeeThreshold ) msg += "X";
		if( lg_y.filteredVal < apogeeThreshold ) msg += "Y";
		if( lg_z.filteredVal < apogeeThreshold ) msg += "Z";
		if( alt_vel.filteredVal < apogeeVelThreshold ) msg += "VEL";
		#if __DEBUG
        DebugSerial->println(msg);
        #endif
		TelemSerial->println(msg);
		if (logSd)
		{
			logFile.println(msg);
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
	String msg = "INFO: Descending";
	#if __DEBUG
    DebugSerial->println(msg);
    #endif
    TelemSerial->println(msg);
    if (logSd)
    {
        logFile.println(msg);
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
	if (altitude.filteredVal <= (startingAltitude + mainDeployAltitude))
	{
		mode = MODE_DESCENT2;
		mainTime = millis();
		deployMain();
		String msg = "INFO: Main Deploy";
        #if __DEBUG
        DebugSerial->println(msg);
        #endif
        TelemSerial->println(msg);
        if (logSd)
        {
            logFile.println(msg);
            logFile.flush();
        }
	}
}

/**
* Main chute deployed, looking for touchdown.
*/
void modeDescent2()
{
	if (lg_x.filteredVal < touchdownThreshold
		&& lg_y.filteredVal < touchdownThreshold
		&& lg_z.filteredVal < touchdownThreshold
		&& alt_vel.filteredVal < 0.25F )
	{
		// switch to touchdown
		mode = MODE_TOUCHDOWN;
		touchdownTime = millis();
		String msg = "INFO: Touchdown";
        #if __DEBUG
        DebugSerial->println(msg);
        #endif
        TelemSerial->println(msg);
        if (logSd)
        {
            logFile.println(msg);
            logFile.flush();
        }
	}
}

/**
* Touchdown complete, awaiting pickup.
*/
void modeTouchdown()
{
    // low power mode?
}

/**
* Deploy drogue chute
*/
void deployDrogue()
{
    digitalWrite(PYRO_2_PIN, HIGH);
    drogueDeployStart = millis();
}

/**
* Deploy main chute
*/
void deployMain()
{
    digitalWrite(PYRO_1_PIN, HIGH);
    mainDeployStart = millis();
}

/**
* Log everything out to Telemetry, Debug, and uSD.
*/
void logStatus()
{
    #if __DEBUG_STACK
    DebugSerial->print("$STK, hga:");
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

	char alt[8];
	dtostrf(altitude.filteredVal, 5, 2, alt);
	char vel[8];
	dtostrf(alt_vel.filteredVal, 5, 2, vel);
	char prs[10];
	dtostrf(pressure.filteredVal, 7, 2, prs);
	char tem[8];
	dtostrf(temperature.filteredVal, 4, 2, tem);

	char hgx[8];
	dtostrf(hg_x.filteredVal, 4, 2, hgx);
	char hgy[8];
	dtostrf(hg_y.filteredVal, 4, 2, hgy);
	char hgz[8];
	dtostrf(hg_z.filteredVal, 4, 2, hgz);

	char lgx[8];
	dtostrf(lg_x.filteredVal, 4, 2, lgx);
	char lgy[8];
	dtostrf(lg_y.filteredVal, 4, 2, lgy);
	char lgz[8];
	dtostrf(lg_z.filteredVal, 4, 2, lgz);

	char magx[8];
	dtostrf(mag_x.filteredVal, 4, 2, magx);
	char magy[8];
	dtostrf(mag_y.filteredVal, 4, 2, magy);
	char magz[8];
	dtostrf(mag_z.filteredVal, 4, 2, magz);

	char gyrox[8];
	dtostrf(gyro_x.filteredVal, 4, 2, gyrox);
	char gyroy[8];
	dtostrf(gyro_y.filteredVal, 4, 2, gyroy);
	char gyroz[8];
	dtostrf(gyro_z.filteredVal, 4, 2, gyroz);

	char vlt[8];
	dtostrf(voltage, 4, 2, vlt);

	printBuffer = "$T,";

	printBuffer += mode; printBuffer += ',';
	printBuffer += alt; printBuffer += ',';
	printBuffer += vel; printBuffer += ',';
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
	printBuffer += (int)GPS.fix;

	if (GPS.fix)
	{
		char lat[16];
		dtostrf(latitude, 9, 6, lat);
		char lon[16];
		dtostrf(longitude, 9, 6, lon);
		char spd[16];
		dtostrf(GPS.speed, 3, 2, spd);
		char ang[16];
		dtostrf(GPS.angle, 3, 2, ang);
		char galt[16];
		dtostrf(GPS.altitude, 3, 2, galt);

		//printBuffer += "\r\nL:";
		printBuffer += ',';
		printBuffer += lat; printBuffer += ',';
		printBuffer += lon; printBuffer += ',';
		printBuffer += spd; printBuffer += ','; // knots //printBuffer += "kn,";
		printBuffer += ang; printBuffer += ','; // degrees
		printBuffer += galt; printBuffer += ','; // meters
		printBuffer += (int)GPS.satellites; printBuffer += ',';
		printBuffer += GPS.year; printBuffer += '-';
		printBuffer += GPS.month; printBuffer += '-';
		printBuffer += GPS.day; printBuffer += '_';
		printBuffer += GPS.hour; printBuffer += ':';
		printBuffer += GPS.minute; printBuffer += ':';
		printBuffer += GPS.seconds; printBuffer += '.';
		printBuffer += GPS.milliseconds;
		printBuffer += "\r\n";
	}
	else
    {
        printBuffer += ",,,,,,\r\n";
    }

	// raw values
	#if __RAW
    printBuffer += "$R,";
    char ralt[8];
    dtostrf(altitude.rawVal, 5, 2, ralt);
    char rvel[8];
    dtostrf(alt_vel.rawVal, 5, 2, rvel);
    char rprs[10];
    dtostrf(pressure.rawVal, 7, 2, rprs);
    char rtem[8];
    dtostrf(temperature.rawVal, 4, 2, rtem);

    char rxf[8];
    dtostrf(hg_x.rawVal, 4, 2, rxf);
    char ryf[8];
    dtostrf(hg_y.rawVal, 4, 2, ryf);
    char rzf[8];
    dtostrf(hg_z.rawVal, 4, 2, rzf);

    char rlxf[8];
    dtostrf(lg_x.rawVal, 4, 2, rlxf);
    char rlyf[8];
    dtostrf(lg_y.rawVal, 4, 2, rlyf);
    char rlzf[8];
    dtostrf(lg_z.rawVal, 4, 2, rlzf);

    char rxm[8];
    dtostrf(mag_x.rawVal, 4, 2, rxm);
    char rym[8];
    dtostrf(mag_y.rawVal, 4, 2, rym);
    char rzm[8];
    dtostrf(mag_z.rawVal, 4, 2, rzm);

    char rxr[8];
    dtostrf(gyro_x.rawVal, 4, 2, rxr);
    char ryr[8];
    dtostrf(gyro_y.rawVal, 4, 2, ryr);
    char rzr[8];
    dtostrf(gyro_z.rawVal, 4, 2, rzr);

    printBuffer += ralt; printBuffer += ',';
    printBuffer += rvel; printBuffer += ',';
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
    printBuffer += "\r\n";

    #endif

    #if __DEBUG
    DebugSerial->print(printBuffer);
    #endif
    TelemSerial->print(printBuffer);

    if (logSd)
    {
        if (logFile.print(printBuffer) < 0)
        {
            #if __DEBUG
            DebugSerial->println("ERROR: Failed to write to SD card.");
            #endif
            logSd = false;
            logFile.close();
        }
        else
        {
            logFile.flush();
        }
    }
}


/**
* Log our flight stats (after touchdown).
*/
void logStats()
{
	char stalt[16];
	dtostrf(startingAltitude, 5, 2, stalt);
	char maxalt[16];
	dtostrf(maxAltitude, 5, 2, maxalt);
	char maxtemp[8];
	dtostrf(maxTemperature, 5, 2, maxtemp);
	char mintemp[8];
	dtostrf(minTemperature, 5, 2, mintemp);
	char maxpress[16];
	dtostrf(maxPressure, 5, 2, maxpress);
	char minpress[8];
	dtostrf(minPressure, 5, 2, minpress);
	char maxf[16];
	dtostrf(maxForce, 5, 2, maxf);
	char maxvel[16];
	dtostrf(maxVelocity, 5, 2, maxvel);

	printBuffer = "========= Flight Stats ============\r\n";
	printBuffer += "Starting Altitude: "; printBuffer += stalt; printBuffer += "\r\n";
	printBuffer += "Maximum Altitude: "; printBuffer += maxalt; printBuffer += "\r\n";
	printBuffer += "Maximum Force: "; printBuffer += maxf; printBuffer += "\r\n";
	printBuffer += "Maximum Velocity: "; printBuffer += maxvel; printBuffer += "\r\n";
	printBuffer += "Maximum Temperature: "; printBuffer += maxtemp; printBuffer += "\r\n";
	printBuffer += "Minimum Temperature: "; printBuffer += mintemp; printBuffer += "\r\n";
	printBuffer += "Maximum Pressure: "; printBuffer += maxpress; printBuffer += "\r\n";
	printBuffer += "Minimum Pressure: "; printBuffer += minpress; printBuffer += "\r\n";
	printBuffer += "Launch Time: "; printBuffer += launchTime; printBuffer += "\r\n";
	printBuffer += "Apogee Time: "; printBuffer += apogeeTime; printBuffer += "\r\n";
	printBuffer += "Main Time: "; printBuffer += mainTime; printBuffer += "\r\n";
	printBuffer += "Touchdown Time: "; printBuffer += touchdownTime; printBuffer += "\r\n";
	printBuffer += "Ascent Time: "; printBuffer += (apogeeTime - launchTime); printBuffer += "\r\n";
	printBuffer += "Descent Time: "; printBuffer += (touchdownTime - apogeeTime); printBuffer += "\r\n";
	printBuffer += "Flight Time: "; printBuffer += (touchdownTime - launchTime); printBuffer += "\r\n";

	#if __DEBUG
    DebugSerial->print(printBuffer);
    #endif
    TelemSerial->print(printBuffer);
    if (logSd)
    {
        logFile.print(printBuffer);
        logFile.flush();
    }
}

/**
* Log our status, after touchdown.
*/
void logStatusTd()
{
    if (!statsOut)
    {
        logStats(); // just do this once
        statsOut = true;
    }
    printBuffer = "INFO: Location: ";
    // just output our current position on telemetry?
    if (GPS.fix)
	{
		char lat[16];
		dtostrf(latitude, 10, 6, lat);
		char lon[16];
		dtostrf(longitude, 10, 6, lon);
		char galt[16];
		dtostrf(GPS.altitude, 6, 2, galt);

		printBuffer += lat; printBuffer += ',';
		printBuffer += lon; printBuffer += ',';
		printBuffer += galt; printBuffer += ','; // meters
	}
	else
    {
        printBuffer += "No Fix!";
    }
    #if __DEBUG
    DebugSerial->println(printBuffer);
    #endif
    TelemSerial->println(printBuffer);
}
