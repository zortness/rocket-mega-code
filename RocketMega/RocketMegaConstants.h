#ifndef ROCKETMEGACONSTANTS_H_INCLUDED
#define ROCKETMEGACONSTANTS_H_INCLUDED


// constants
#define SERIAL_BAUD 57600
#define GPS_BAUD 9600
#define LED_PIN 13
#define SD_CHIP_SELECT 48
#define SENSORS_DPS_TO_RADS (0.017453293F)
#define ACCEL_X_CENTER 500
#define ACCEL_Y_CENTER 500
#define RELAY_1_PIN 42
#define RELAY_2_PIN 43
#define RELAY_ON_TIME 1000 // in milliseconds

// delays, these are NOT in milliseconds, they are in loop cycles!
#define LOG_INTERVAL_SLOW 4000
#define LOG_INTERVAL_FAST 1000
#define ALTIMETER_INTERVAL_SLOW 1000
#define ALTIMETER_INTERVAL_FAST 250
#define GYRO_INTERVAL_SLOW 1000
#define GYRO_INTERVAL_FAST 500
#define HIGH_ACCEL_INTERVAL_SLOW 10
#define HIGH_ACCEL_INTERVAL_FAST 1
#define LOW_ACCEL_INTERVAL_SLOW 1000
#define LOW_ACCEL_INTERVAL_FAST 500
#define PRINT_HEADER_INTERVAL 50

#define DEFAULT_ASCENT_THRESHOLD 50      // in distance from ACCEL_[X/Y]_CENTER where center is 1G
#define DEFAULT_APOGEE_THRESHOLD 10      // in distance from ACCEL_[X/Y]_CENTER where center is 1G
#define DEFAULT_TOUCHDOWN_THRESHOLD 2	 // in distance from ACCEL_[X/Y]_CENTER where center is 1G
#define DEFAULT_MAIN_ALTITUDE 300        // in meters (added to deck altitude)
#define DEFAULT_SEALEVEL_PRESSURE 1013.21 // in h pascals, 101325

#define MODE_ERROR 999
#define MODE_STARTUP 0
#define MODE_READY 1
#define MODE_ASCENT 2
#define MODE_APOGEE 3
#define MODE_DESCENT 4
#define MODE_DESCENT2 5
#define MODE_TOUCHDOWN 6
#define MODE_PAUSE 7

// kalman filter values
#define HIGH_ACCEL_PROCESS_NOISE 0.20001
#define HIGH_ACCEL_MEASURE_NOISE 4.0001
#define HIGH_ACCEL_ERROR_COV 25.0
#define LOW_ACCEL_PROCESS_NOISE 0.20001
#define LOW_ACCEL_MEASURE_NOISE 4.0001
#define LOW_ACCEL_ERROR_COV 25.0
#define ALTITUDE_PROCESS_NOISE 0.20001
#define ALTITUDE_MEASURE_NOISE 4.0001
#define ALTITUDE_ERROR_COV 25.0
#define PRESSURE_PROCESS_NOISE 0.20001
#define PRESSURE_MEASURE_NOISE 4.0001
#define PRESSURE_ERROR_COV 25.0
#define TEMPERATURE_PROCESS_NOISE 0.20001
#define TEMPERATURE_MEASURE_NOISE 4.0001
#define TEMPERATURE_ERROR_COV 25.0
#define GYRO_PROCESS_NOISE 0.20001
#define GYRO_MEASURE_NOISE 4.0001
#define GYRO_ERROR_COV 25.0

// 168 and 328 Arduinos (5v)
#if defined(__AVR_ATmega168__) ||defined(__AVR_ATmega168P__) ||defined(__AVR_ATmega328P__)
  #define CHIPSET ATmega_168_168P_328P
  #define ACCEL_X_PIN A2
  #define ACCEL_Y_PIN A3

// Mega 1280 & 2560 (5v)
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  #define CHIPSET ATmega_1280_2560
  #define ACCEL_X_PIN A2
  #define ACCEL_Y_PIN A3

// Due and others (3v)
#else
  #define CHIPSET unknown
  #define ACCEL_X_PIN A0
  #define ACCEL_Y_PIN A1

#endif


#endif // ROCKETMEGACONSTANTS_H_INCLUDED
