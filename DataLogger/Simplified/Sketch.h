/*
 * Sketch.h
 *
 * Created: 12/28/2017 4:51:46 PM
 *  Author: DGrul
 */ 


#ifndef SKETCH_H_
#define SKETCH_H_

/************************************
*			Include					*
************************************/
#include <Wire.h>
#include <Math.h>

// Accel, Gyro, Mag
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>

// GPS
#include <TinyGPS++.h>

// Nissan Consult
#include "Consult.h"
#include "ConsultConversionFunctions.h"
#include "ConsultRegister.h"
#include "ConsultErrorCode.h"

/************************************
*			Definitions				*
************************************/
// Serial Definitions
#define DEBUG			Serial
#define SDCARD			Serial1
#define GPS_PORT        Serial2
#define NISSAN_PORT     Serial3

// Baud Rate Definitions
#define GPS_STARTBAUDRATE 9600
#define NEW_BAUDRATE    115200

#define CSVHEADING "\"Time (s)\",\"Latitude\",\"Longitude\",\"Altitude (m)\",\"Heading\",\"X\",\"Y\",\"Speed (MPH)\",\"Engine Speed (RPM)\",\"Accelerator Pedal\",\"Break Pedal\""

/************************************
*			Function Prototypes		*
************************************/
// Main system
void setup(void );
void loop(void );

// Initialize systems
void initializeSensors();
void initializeGPS();
void initializeNissanConsult();
void initializeOpenLog();

// Get data
void getHeadingDirection(sensors_event_t magnetometer, sensors_vec_t orientations);
void getSensorData(sensors_event_t accelerometer, sensors_event_t gyroscope, sensors_vec_t orientations);
void getGPSData();
void getNissanData();

// Peripherals
void printToSDCard(void);
void gotoCommandMode(void);
void createFile(char *);
boolean checkSensors(void);
void initializeECU(Consult Class);
void checkGPSReady(void);
void sendGPSCommand(uint8_t* Command, uint8_t sizeOfArray);
void displayInfo(void);
void resetOpenLog(void);

#endif /* SKETCH_H_ */