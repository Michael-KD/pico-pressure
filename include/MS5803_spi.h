/*	MS5803
 * 	An Arduino library for the Measurement Specialties MS5803 family
 * 	of pressure sensors. This library uses I2C to communicate with the
 * 	MS5803 using the Wire library from Arduino.
 *	
 *	http://github.com/millerlp 
 * 	
 * 	Copyright Luke Miller, April 1 2014
 *	Changed by Dale DeJager to support multiple sensors
 *	on March 17, 2019
 */


#ifndef __MS_5803__
#define __MS_5803__

#include <Arduino.h>
#include <SPI.h>

class MS_5803 {
public:
	// Constructor for the class. 
	// The argument is the desired oversampling resolution, which has 
	// values of 256, 512, 1024, 2048, 4096
	// Second argument is the I2C address of the unit: either 0x76 or 0x77
    MS_5803(uint16_t Resolution = 512, uint8_t csPin = PIN_SPI1_SS, uint8_t SensorNumber = 1);
    // Initialize the sensor 
    boolean initializeMS_5803(boolean Verbose = true);
    // Reset the sensor
    void resetSensor();
    // Read the sensor
    void readSensor();
    //*********************************************************************
    // Additional methods to extract temperature, pressure (mbar), and the 
    // D1,D2 values after readSensor() has been called
    
    // Return temperature in degrees Celsius.
    float temperature() const       {return tempC;}  
    // Return pressure in mbar.
    float pressure() const          {return mbar;}
//    // Return temperature in degress Fahrenheit.
//    float temperatureF() const		{return tempF;}
//    // Return pressure in psi (absolute)
//    float psia() const				{return psiAbs;}
//    // Return pressure in psi (gauge)
//    float psig() const				{return psiGauge;}
//    // Return pressure in inHg
//    float inHg() const				{return inHgPress;}
//    // Return pressure in mmHg
//    float mmHg() const				{return mmHgPress;}
    // Return the D1 and D2 values, mostly for troubleshooting
    unsigned long D1val() const 	{return D1raw;}
    unsigned long D2val() const		{return D2raw;}
    
    
private:
    
    float mbar; // Store pressure in mbar. 
    float tempC; // Store temperature in degrees Celsius
//    float tempF; // Store temperature in degrees Fahrenheit
//    float psiAbs; // Store pressure in pounds per square inch, absolute
//    float psiGauge; // Store gauge pressure in pounds per square inch (psi)
//    float inHgPress;	// Store pressure in inches of mercury
//    float mmHgPress;	// Store pressure in mm of mercury
    unsigned long D1raw;	// Store D1 value
    unsigned long D2raw;	// Store D2 value
    int32_t mbarInt; // pressure in mbar, initially as a signed long integer
    // Check data integrity with CRC4
    unsigned char MS_5803_CRC(unsigned int n_prom[]); 
    // Handles commands to the sensor.
    unsigned long MS_5803_ADC(char commandADC);
    // Oversampling resolution
    uint16_t _Resolution;
    unsigned int sensorCoeffs[8]; //Coefficients
    uint8_t _address; // I2C Address
    uint8_t _SensorNumber; // Sensor number
    uint8_t _csPin; // Chip select pin for SPI
    SPISettings _spiSettings; // SPI settings
    
    // timer for last temp read
    unsigned long lastTempRead = 0;
};

#endif 