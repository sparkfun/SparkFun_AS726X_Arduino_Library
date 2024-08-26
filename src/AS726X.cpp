#include "AS726X.h"
#include "Arduino.h"
//Sets up the sensor for constant read
//Returns the sensor version (AS7262 or AS7263)

AS726X::AS726X()
{
	
}

bool AS726X::begin(TwoWire &wirePort, uint8_t gain, uint8_t measurementMode)
{
	_i2cPort = &wirePort;
	_sensorVersion = virtualReadRegister(AS726x_HW_VERSION);

	//HW version for AS7262, AS7263 and AS7261
	if (_sensorVersion != SENSORTYPE_AS7261 && 
		_sensorVersion != SENSORTYPE_AS7262 && 
		_sensorVersion != SENSORTYPE_AS7263) 
	{
		return false;
	}

	//Set to 12.5mA (minimum)
	if(setBulbCurrent(0b00)) return false;

	if(disableBulb()) return false; //Turn off to avoid heating the sensor

	if(setIndicatorCurrent(0b11)) return false; //Set to 8mA (maximum)

	if(disableIndicator()) return false; //Turn off lights to save power

	if(setIntegrationTime(50)) return false; //50 * 2.8ms = 140ms. 0 to 255 is valid.

	//If you use Mode 2 or 3 (all the colors) then integration time is double.
	//140*2 = 280ms between readings.

	if(setGain(gain)) return false; //Set gain to 64x

	if(setMeasurementMode(measurementMode)) return false; //One-shot mode

	return true;
}

uint8_t AS726X::getVersion()
{
	return _sensorVersion;
}

//Sets the measurement mode
//Mode 0: Continuous reading of VBGY (7262) / STUV (7263)
//Mode 1: Continuous reading of GYOR (7262) / RTUX (7263)
//Mode 2: Continuous reading of all channels (power-on default)
//Mode 3: One-shot reading of all channels
int AS726X::setMeasurementMode(uint8_t mode)
{
	if (mode > 0b11) mode = 0b11;

	//Read, mask/set, write
	uint8_t value = virtualReadRegister(AS726x_CONTROL_SETUP); //Read
	value &= 0b11110011; //Clear BANK bits
	value |= (mode << 2); //Set BANK bits with user's choice
	return virtualWriteRegister(AS726x_CONTROL_SETUP, value); //Write
}

uint8_t AS726X::getMeasurementMode()
{
	uint8_t value = virtualReadRegister(AS726x_CONTROL_SETUP); //Read
	return (value & 0b00001100); //Isolate BANK bits.
}

//Sets the gain value
//Gain 0: 1x (power-on default)
//Gain 1: 3.7x
//Gain 2: 16x
//Gain 3: 64x
int AS726X::setGain(uint8_t gain)
{
	if (gain > 0b11) gain = 0b11;

	//Read, mask/set, write
	uint8_t value = virtualReadRegister(AS726x_CONTROL_SETUP); //Read
	value &= 0b11001111; //Clear GAIN bits
	value |= (gain << 4); //Set GAIN bits with user's choice
	return virtualWriteRegister(AS726x_CONTROL_SETUP, value); //Write
}

uint8_t AS726X::getGain()
{
	uint8_t value = virtualReadRegister(AS726x_CONTROL_SETUP); //Read
	return (value & 0b00110000); //Isolate GAIN bits.
}

//Sets the integration value
//Give this function a uint8_t from 0 to 255.
//Time will be 2.8ms * [integration value]
int AS726X::setIntegrationTime(uint8_t integrationValue)
{
	return virtualWriteRegister(AS726x_INT_T, integrationValue); //Write
}

uint8_t AS726X::getIntegrationTime()
{
	uint8_t value = virtualReadRegister(AS726x_INT_T); //Read
	return value;
}

int AS726X::enableInterrupt()
{
	//Read, mask/set, write
	uint8_t value = virtualReadRegister(AS726x_CONTROL_SETUP); //Read
	value |= 0b01000000; //Set INT bit
	return virtualWriteRegister(AS726x_CONTROL_SETUP, value); //Write
}

//Disables the interrupt pin
int AS726X::disableInterrupt()
{
	//Read, mask/set, write
	uint8_t value = virtualReadRegister(AS726x_CONTROL_SETUP); //Read
	value &= 0b10111111; //Clear INT bit
	return virtualWriteRegister(AS726x_CONTROL_SETUP, value); //Write
}

//Tells IC to take measurements and polls for data ready flag
int AS726X::takeMeasurements()
{
	//Clear DATA_RDY flag when using Mode 3
	if(clearDataAvailable()) return -1;

	 //Goto mode 3 for one shot measurement of all channels
	if(setMeasurementMode(3)) return -1;

	uint32_t timeout = millis() + TIMEOUT;

	//Wait for data to be ready
	while (dataAvailable() == false)
	{
		delay(POLLING_DELAY);
		if(millis() > timeout) return -1;
	}

	//Readings can now be accessed via getViolet(), getBlue(), etc
	return 0;
}

//Turns on bulb, takes measurements, turns off bulb
int AS726X::takeMeasurementsWithBulb()
{
	//enableIndicator(); //Tell the world we are taking a reading. 
	//The indicator LED is red and may corrupt the readings

	if(enableBulb()) return -1; //Turn on bulb to take measurement

	if(takeMeasurements()) return -1;

	if(disableBulb()) return -1; //Turn off bulb to avoid heating sensor
	//disableIndicator();

	return 0;
}

//Get the various color readings
int AS726X::getViolet() { return(getChannel(AS7262_V)); }
int AS726X::getBlue() { return(getChannel(AS7262_B)); }
int AS726X::getGreen() { return(getChannel(AS7262_G)); }
int AS726X::getYellow() { return(getChannel(AS7262_Y)); }
int AS726X::getOrange() { return(getChannel(AS7262_O)); }
int AS726X::getRed() { return(getChannel(AS7262_R)); }

//Get the various NIR readings
int AS726X::getR() { return(getChannel(AS7263_R)); }
int AS726X::getS() { return(getChannel(AS7263_S)); }
int AS726X::getT() { return(getChannel(AS7263_T)); }
int AS726X::getU() { return(getChannel(AS7263_U)); }
int AS726X::getV() { return(getChannel(AS7263_V)); }
int AS726X::getW() { return(getChannel(AS7263_W)); }

//Get the various CIE readings
int AS726X::getX() { return(getChannel(AS7261_X)); }
int AS726X::getY() { return(getChannel(AS7261_Y)); }
int AS726X::getZ() { return(getChannel(AS7261_Z)); }
int AS726X::getNir() { return(getChannel(AS7261_NIR)); }
int AS726X::getDark() { return(getChannel(AS7261_DARK)); }
int AS726X::getClear() { return(getChannel(AS7261_CLEAR)); }

//A the 16-bit value stored in a given channel registerReturns 
int AS726X::getChannel(uint8_t channelRegister)
{
	int colorData = virtualReadRegister(channelRegister) << 8; //High uint8_t
	colorData |= virtualReadRegister(channelRegister + 1); //Low uint8_t
	return(colorData);
}

//Returns the various calibration data
float AS726X::getCalibratedViolet() { return(getCalibratedValue(AS7262_V_CAL)); }
float AS726X::getCalibratedBlue() { return(getCalibratedValue(AS7262_B_CAL)); }
float AS726X::getCalibratedGreen() { return(getCalibratedValue(AS7262_G_CAL)); }
float AS726X::getCalibratedYellow() { return(getCalibratedValue(AS7262_Y_CAL)); }
float AS726X::getCalibratedOrange() { return(getCalibratedValue(AS7262_O_CAL)); }
float AS726X::getCalibratedRed() { return(getCalibratedValue(AS7262_R_CAL)); }

float AS726X::getCalibratedR() { return(getCalibratedValue(AS7263_R_CAL)); }
float AS726X::getCalibratedS() { return(getCalibratedValue(AS7263_S_CAL)); }
float AS726X::getCalibratedT() { return(getCalibratedValue(AS7263_T_CAL)); }
float AS726X::getCalibratedU() { return(getCalibratedValue(AS7263_U_CAL)); }
float AS726X::getCalibratedV() { return(getCalibratedValue(AS7263_V_CAL)); }
float AS726X::getCalibratedW() { return(getCalibratedValue(AS7263_W_CAL)); }

float AS726X::getCalibratedX() { return(getCalibratedValue(AS7261_X_CAL)); }
float AS726X::getCalibratedY() { return(getCalibratedValue(AS7261_Y_CAL)); }
float AS726X::getCalibratedZ() { return(getCalibratedValue(AS7261_Z_CAL)); }
float AS726X::getCalibratedX1931() { return(getCalibratedValue(AS7261_X1931_CAL)); }
float AS726X::getCalibratedY1931() { return(getCalibratedValue(AS7261_Y1931_CAL)); }
float AS726X::getCalibratedUPri1976() { return(getCalibratedValue(AS7261_UPRI_CAL)); }
float AS726X::getCalibratedVPri1976() { return(getCalibratedValue(AS7261_VPRI_CAL)); }
float AS726X::getCalibratedU1976() { return(getCalibratedValue(AS7261_U_CAL)); }
float AS726X::getCalibratedV1976() { return(getCalibratedValue(AS7261_V_CAL)); }
float AS726X::getCalibratedDUV1976() { return(getCalibratedValue(AS7261_DUV_CAL)); }
int AS726X::getCalibratedLux()  { return(getChannel(AS7261_LUX_CAL)); }
int AS726X::getCalibratedCCT() { return(getChannel(AS7261_CCT_CAL)); }

//Given an address, read four uint8_ts and return the floating point calibrated value
float AS726X::getCalibratedValue(uint8_t calAddress)
{
	uint8_t b0, b1, b2, b3;
	b0 = virtualReadRegister(calAddress + 0);
	b1 = virtualReadRegister(calAddress + 1);
	b2 = virtualReadRegister(calAddress + 2);
	b3 = virtualReadRegister(calAddress + 3);

	//Channel calibrated values are stored big-endian
	uint32_t calBytes = 0;
	calBytes |= ((uint32_t)b0 << (8 * 3));
	calBytes |= ((uint32_t)b1 << (8 * 2));
	calBytes |= ((uint32_t)b2 << (8 * 1));
	calBytes |= ((uint32_t)b3 << (8 * 0));

	return (convertBytesToFloat(calBytes));
}

//Given 4 uint8_ts returns the floating point value
float AS726X::convertBytesToFloat(uint32_t myLong)
{
	float myFloat;
	memcpy(&myFloat, &myLong, 4); //Copy uint8_ts into a float
	return (myFloat);
}

//Checks to see if DRDY flag is set in the control setup register
bool AS726X::dataAvailable()
{
	uint8_t value = virtualReadRegister(AS726x_CONTROL_SETUP);
	return (value & (1 << 1)); //Bit 1 is DATA_RDY
}

//Clears the DRDY flag
//Normally this should clear when data registers are read
int AS726X::clearDataAvailable()
{
	uint8_t value = virtualReadRegister(AS726x_CONTROL_SETUP);
	value &= ~(1 << 1); //Set the DATA_RDY bit
	return virtualWriteRegister(AS726x_CONTROL_SETUP, value);
}

//Enable the onboard indicator LED
int AS726X::enableIndicator()
{
	//Read, mask/set, write
	uint8_t value = virtualReadRegister(AS726x_LED_CONTROL);
	value |= (1 << 0); //Set the bit
	return virtualWriteRegister(AS726x_LED_CONTROL, value);
}

//Disable the onboard indicator LED
int AS726X::disableIndicator()
{
	//Read, mask/set, write
	uint8_t value = virtualReadRegister(AS726x_LED_CONTROL);
	value &= ~(1 << 0); //Clear the bit
	return virtualWriteRegister(AS726x_LED_CONTROL, value);
}

//Set the current limit of onboard LED. Default is max 8mA = 0b11.
int AS726X::setIndicatorCurrent(uint8_t current)
{
	if (current > 0b11) current = 0b11;
	//Read, mask/set, write
	uint8_t value = virtualReadRegister(AS726x_LED_CONTROL); //Read
	value &= 0b11111001; //Clear ICL_IND bits
	value |= (current << 1); //Set ICL_IND bits with user's choice
	return virtualWriteRegister(AS726x_LED_CONTROL, value); //Write
}

//Enable the onboard 5700k or external incandescent bulb
int AS726X::enableBulb()
{
	//Read, mask/set, write
	uint8_t value = virtualReadRegister(AS726x_LED_CONTROL);
	value |= (1 << 3); //Set the bit
	return virtualWriteRegister(AS726x_LED_CONTROL, value);
}

//Disable the onboard 5700k or external incandescent bulb
int AS726X::disableBulb()
{
	//Read, mask/set, write
	uint8_t value = virtualReadRegister(AS726x_LED_CONTROL);
	value &= ~(1 << 3); //Clear the bit
	return virtualWriteRegister(AS726x_LED_CONTROL, value);
}

//Set the current limit of bulb/LED.
//Current 0: 12.5mA
//Current 1: 25mA
//Current 2: 50mA
//Current 3: 100mA
int AS726X::setBulbCurrent(uint8_t current)
{
	if (current > 0b11) current = 0b11; //Limit to two bits

										//Read, mask/set, write
	uint8_t value = virtualReadRegister(AS726x_LED_CONTROL); //Read
	value &= 0b11001111; //Clear ICL_DRV bits
	value |= (current << 4); //Set ICL_DRV bits with user's choice
	return virtualWriteRegister(AS726x_LED_CONTROL, value); //Write
}

//Returns the temperature in C
//Pretty inaccurate: +/-8.5C
uint8_t AS726X::getTemperature()
{
	return (virtualReadRegister(AS726x_DEVICE_TEMP));
}

//Convert to F if needed
float AS726X::getTemperatureF()
{
	float temperatureF = getTemperature();
	temperatureF = temperatureF * 1.8 + 32.0;
	return (temperatureF);
}

//Does a soft reset
//Give sensor at least 1000ms to reset
int AS726X::softReset()
{
	//Read, mask/set, write
	uint8_t value = virtualReadRegister(AS726x_CONTROL_SETUP); //Read
	value |= (1 << 7); //Set RST bit
	return virtualWriteRegister(AS726x_CONTROL_SETUP, value); //Write
}

//Read a virtual register from the AS726x
uint8_t AS726X::virtualReadRegister(uint8_t virtualAddr)
{
	uint8_t status;
	uint8_t retries = 0;

	//Do a prelim check of the read register
	status = readRegister(AS72XX_SLAVE_STATUS_REG);
	if ((status & AS72XX_SLAVE_RX_VALID) != 0) //There is data to be read
	{
		//Serial.println("Premptive read");
		uint8_t incoming = readRegister(AS72XX_SLAVE_READ_REG); //Read the uint8_t but do nothing with it
	}

	//Wait for WRITE flag to clear
	while (1)
	{
		status = readRegister(AS72XX_SLAVE_STATUS_REG);
		if (status == 0xFF) return status;
		if ((status & AS72XX_SLAVE_TX_VALID) == 0) break; // If TX bit is clear, it is ok to write
		delay(POLLING_DELAY);
		if(retries++ > MAX_RETRIES) return 0xFF;
	}

	// Send the virtual register address (bit 7 should be 0 to indicate we are reading a register).
	if(writeRegister(AS72XX_SLAVE_WRITE_REG, virtualAddr)) return 0xFF;

	retries = 0;

	//Wait for READ flag to be set
	while (1)
	{
		status = readRegister(AS72XX_SLAVE_STATUS_REG);
		if (status == 0xFF) return status;
		if ((status & AS72XX_SLAVE_RX_VALID) != 0) break; // Read data is ready.
		delay(POLLING_DELAY);
		if(retries++ > MAX_RETRIES) return 0xFF;
	}

	uint8_t incoming = readRegister(AS72XX_SLAVE_READ_REG);
	return (incoming);
}

//Write to a virtual register in the AS726x
int AS726X::virtualWriteRegister(uint8_t virtualAddr, uint8_t dataToWrite)
{
	uint8_t status;
	uint8_t retries = 0;

	//Wait for WRITE register to be empty
	while (1)
	{
		status = readRegister(AS72XX_SLAVE_STATUS_REG);
		if (status == 0xFF) return -1;
		if ((status & AS72XX_SLAVE_TX_VALID) == 0) break; // No inbound TX pending at slave. Okay to write now.
		delay(POLLING_DELAY);
		if(retries++ > MAX_RETRIES) return -1;
	}

	// Send the virtual register address (setting bit 7 to indicate we are writing to a register).
	writeRegister(AS72XX_SLAVE_WRITE_REG, (virtualAddr | 0x80));

	retries = 0;

	//Wait for WRITE register to be empty
	while (1)
	{
		status = readRegister(AS72XX_SLAVE_STATUS_REG);
		if (status == 0xFF) return -1;
		if ((status & AS72XX_SLAVE_TX_VALID) == 0) break; // No inbound TX pending at slave. Okay to write now.
		delay(POLLING_DELAY);
		if(retries++ > MAX_RETRIES) return -1;
	}

	// Send the data to complete the operation.
	writeRegister(AS72XX_SLAVE_WRITE_REG, dataToWrite);

	return 0;
}

//Reads from a give location from the AS726x
uint8_t AS726X::readRegister(uint8_t addr)
{
	uint8_t err = 0xFF;

	_i2cPort->beginTransmission(AS726X_ADDR);
	if(_i2cPort->write(addr) == 0) return err;
	if(_i2cPort->endTransmission()) return err;

	if(_i2cPort->requestFrom(AS726X_ADDR, 1) == 0) return err;
	if (_i2cPort->available()) {
		return (_i2cPort->read());
	}
	else {
		Serial.println("I2C Error");
		return err; //Error
	}

	return 0;
}

//Write a value to a spot in the AS726x
int AS726X::writeRegister(uint8_t addr, uint8_t val)
{
	uint8_t err = 0xFF;

	_i2cPort->beginTransmission(AS726X_ADDR);
	if(_i2cPort->write(addr) == 0) return (int)err;
	if(_i2cPort->write(val) == 0) return (int)err;
	if(_i2cPort->endTransmission()) return (int)err;

	return 0;
}