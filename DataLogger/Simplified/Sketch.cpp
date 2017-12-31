﻿/*Begining of Auto generated code by Atmel studio */
#include <Arduino.h>
/*End of auto generated code by Atmel studio */

// Include
#include "Sketch.h"

/************************************
*			Global Objects			*
************************************/
//  Accelerometer, Gyroscope and Magnetometer
Adafruit_10DOF                  dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified   accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified     mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified         gyro  = Adafruit_L3GD20_Unified(20);
sensors_event_t accel_event;
sensors_event_t gyro_event;
sensors_event_t mag_event;
sensors_vec_t   orientation;

// GPS
TinyGPSPlus gps;

// Nissan Consult
Consult myConsult = Consult();

/************************************
*			Global Variables		*
************************************/
//  Accelerometer, Gyroscope and Magnetometer
double xAngle = 0, yAngle = 0;
double firstTime, previousTime;
double XAccelForce, YAccelForce;
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
double Heading;

// GPS
double Latitude = 0.000000;
double Longitude = 0.000000;
double Altitude = 0.000000;
uint16_t Year;
uint8_t Month;
uint8_t Day;
uint8_t Hour;
uint8_t Minute;
uint8_t refreshRateCmd[10] = {0xA0, 0xA1, 0x00, 0x03, 0x0E, 0x14, 0x00, 0x1A, 0x0D, 0x0A};    // 20 Hz Refresh Rate
uint8_t baudRateCmd[11] = {0xA0, 0xA1, 0x00, 0x04, 0x05, 0x00, 0x05, 0x00, 0x00, 0x0D, 0x0A}; // 115200 Baud Rate

// Nissan Consult
boolean initialized = false;
int VehicleSpeed = 0;
int Tachometer = 0;
int ThrottlePosition = 0;
int BrakePosition = 0;

/************************************
*			The Brain				*
************************************/
// Set-up the Arduino
void setup(void)
{
	// Begin serial port
	DEBUG.begin(NEW_BAUDRATE);

	// Initialize systems
	initializeSensors();		// Initialize Accel, Gyr, and Mag
	initializeGPS();			// Initialize GPS and wait till reads location
	initializeNissanConsult();	// Initialize Nissan Consult
	
	// Print CSV file heading
	SDCARD.println(CSVHEADING);
	return;
}

// The main loop
void loop(void)
{
	// Get data
	getGPSData();											// Get GPS data
	getSensorData(accel_event, gyro_event, orientation);	// Get angles
	getHeadingDirection(mag_event, orientation);			// Get heading
	getNissanData();										// Get Nissan data

	// Print Info
	printToSDCard();	// Print data to SD Card
  
	return;
}

/************************************
*			Functions				*
************************************/
// Checks if accel, gyro, and mag are connected and ready
void initializeSensors()
{
	gyro.enableAutoRange(true);	// Enable Auto-Ranging with Gyroscope
	while(!checkSensors()) {}	//Initialize the Sensors
		
	previousTime = millis();	// For Gyroscope Reading
	
	return;
}

// Sets GPS to 115200 baud and 20 Hz refresh rate then waits
// until it sees location
void initializeGPS()
{
	// Start GPS Serial Port
	GPS_PORT.begin(GPS_STARTBAUDRATE);
	
	// Update Venus838FLPx baud rate to 115200
	sendGPSCommand(baudRateCmd, sizeof(baudRateCmd));
	GPS_PORT.begin(NEW_BAUDRATE);
	
	// Send Update Refresh Rate Command
	sendGPSCommand(refreshRateCmd, sizeof(refreshRateCmd));

	// Wait till navigation reads location
	checkGPSReady();	
	
	return;
}

// Sets Consult serial port and initializes CPU
void initializeNissanConsult()
{
	// Get Consult ready for use
	myConsult.setSerial(&NISSAN_PORT);  // Set Consult to serial 3
	myConsult.setMetric(false);			// Set Consult to MPH and Fahrenheit
	NISSAN_PORT.flush();
	
	// Initialize Consult
	initializeECU(myConsult);
		
	return;
}

// Initialize Sensors
boolean checkSensors()
{
  if(!accel.begin())
  {
    DEBUG.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    return false;
  }
  if (!gyro.begin())
  {
    DEBUG.println("Ooops, no L3GD20 detected ... Check your wiring!");
    return false;
  }
  if(!mag.begin())
  {
    DEBUG.println("Ooops, no LSM303 detected ... Check your wiring!");
    return false;
  }
  return true;
}

// Send initialization command to Nissan ECU
void initializeECU(Consult Class)
                             {
	while(initialized == false)
	{
		DEBUG.println("Trying to initialize Nissan Consult");
		if( Class.initEcu() ) initialized = true;
	}
}

// Wait till GPS finds location
void checkGPSReady()
{
	// Initial variables
	bool Ready = false;
	
	// Wait for GPS to find location
	while(!Ready)
	{
		while (GPS_PORT.available() > 0)
		{
			gps.encode(GPS_PORT.read()); // Get GPS Data
		}
		if(gps.location.lat() > 0) Ready = true;
	}

	// Fill date and time
	Year = gps.date.year();
	Month = gps.date.month();
	Day = gps.date.day();
	Hour = gps.time.hour();
	Minute = gps.time.minute();
	
	return;
}

// Sends commands to the GPS
void sendGPSCommand(uint8_t* Command, uint8_t sizeOfArray)
{
	GPS_PORT.write(Command, sizeOfArray);
	GPS_PORT.flush();
	delay(10);
	
	return;
}

// Get data from GPS
void getGPSData()
{
	// Read data
	while (GPS_PORT.available() > 0)
	{
		gps.encode( GPS_PORT.read() );
	}
	
	// Store it in variable
	Latitude = gps.location.lat();
	Longitude = gps.location.lng();
	Altitude = gps.altitude.meters();
	
	return;
}

// Get heading direction from magnetometer
void getHeadingDirection(sensors_event_t magnetometer, sensors_vec_t orientations)
{
  mag.getEvent(&magnetometer);
  if (dof.magGetOrientation(SENSOR_AXIS_Z, &magnetometer, &orientations))
  {
    Heading = orientations.heading;
  }
}

// Gets acceleration & gyroscope data and runs it through complementary filter
void getSensorData(sensors_event_t accelerometer,sensors_event_t gyroscope, sensors_vec_t orientations)
{
	// Label variables needed
	double dT, xGyrData, yGyrData, xAccel, yAccel;

	// Get sensor information
	gyro.getEvent(&gyroscope);		// Get Gyroscope Information
	accel.getEvent(&accelerometer);	// Get Accelerometer Information
  
	// Get accelerometer angles (degrees)
	dof.accelGetOrientation(&accelerometer,&orientations);	  // Turn acceleration data into angles
	xAccel = orientations.roll;              // Get the Roll from the accelerometer
	yAccel = orientations.pitch;             // Get the Pitch from the accelerometer

	// Get gyroscope angles (degrees per second)
    firstTime = millis();
    xGyrData = gyroscope.gyro.x * 57.2957;		// X-Axis in Degrees per Second
    yGyrData = - gyroscope.gyro.y * 57.2957;	// Y-Axis in Degrees per Second
    dT = (firstTime - previousTime) / 1000;		// Finding the time passed since last reading
    previousTime = firstTime;

	// Use complementary filter formula
	xAngle = 0.90 * (xAngle + (xGyrData * dT)) + 0.10 * xAccel;
	yAngle = 0.90 * (yAngle + (yGyrData * dT)) + 0.10 * yAccel;

	// Get g-forces
	XAccelForce = accelerometer.acceleration.x*cos(yAngle*0.01745) - accelerometer.acceleration.z*sin(yAngle*0.01745);
	XAccelForce = XAccelForce * 0.101972;
	YAccelForce = accelerometer.acceleration.y*cos(xAngle*0.01745) - accelerometer.acceleration.z*sin(xAngle*0.01745);
	YAccelForce = YAccelForce * 0.101972;
	
	return;
}

// Reads vehicle speed, tachometer and throttle position
void getNissanData()
{
	// Set initial variables and structures
	int NumOfRegisters = 3; // Declare Number of Registers
	ConsultRegister myRegisters[NumOfRegisters]; // Create Registers Class
	
	// Set Conversion Functions
	myRegisters[0] = ConsultRegister("Speed", ECU_REGISTER_VEHICLE_SPEED, ECU_REGISTER_NULL, &ConsultConversionFunctions::convertVehicleSpeed);
	myRegisters[1] = ConsultRegister("Tach", ECU_REGISTER_TACH_MSB, ECU_REGISTER_TACH_LSB, &ConsultConversionFunctions::convertTachometer);
	myRegisters[2] = ConsultRegister("Throttle Position", ECU_REGISTER_THROTTLE_POSITION, ECU_REGISTER_NULL, &ConsultConversionFunctions::convertThrottlePosition);

	// If able to start stream, read data
	if (myConsult.startEcuStream(myRegisters, NumOfRegisters)) 
	{
		if(myConsult.readEcuStream(myRegisters, NumOfRegisters))
		{
			VehicleSpeed = myRegisters[0].getValue();
			Tachometer = myRegisters[1].getValue();
			ThrottlePosition = myRegisters[2].getValue();
		}
		myConsult.stopEcuStream();
	}
	
	// Else, do nothing
	return;
}

// Print data in CSV format
void printToSDCard()
{
	// Get current time since start
	double Time = (double)millis() / 1000.00;
	
	// Print information
	SDCARD.print(Time,3);			SDCARD.print(",");	// Time (s)
	SDCARD.print(Latitude,6);		SDCARD.print(",");	// Latitude
	SDCARD.print(Longitude,6);		SDCARD.print(",");	// Longitude
	SDCARD.print(Altitude,2);		SDCARD.print(",");	// Altitude (m)
	SDCARD.print(Heading);			SDCARD.print(",");	// Heading
	SDCARD.print(XAccelForce,3);	SDCARD.print(",");	// X (g)
	SDCARD.print(YAccelForce,3);	SDCARD.print(",");	// Y (g)
	SDCARD.print(VehicleSpeed);		SDCARD.print(",");	// Vehicle Speed (mph)
	SDCARD.print(Tachometer);		SDCARD.print(",");	// Tachometer (rpm)
	SDCARD.print(ThrottlePosition);	SDCARD.print(",");	// Throttle Position
	SDCARD.print(BrakePosition);	SDCARD.println(",");	// Brake Position
	
	return;
}
