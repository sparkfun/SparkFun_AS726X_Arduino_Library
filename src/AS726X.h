/*
 Name:		AS726X.h
 Created:	7/11/2017 12:06:22 PM
 Author:	andrew.england
 Editor:	http://www.visualmicro.com
*/

#ifndef _AS726X_h
#define _AS726X_h
#include "Arduino.h"
#include "Wire.h"
class AS726X {
public:
	AS726X();
	bool begin(TwoWire &wirePort = Wire, uint8_t gain = 3, uint8_t measurementMode = 3);
	void takeMeasurements();
	uint8_t getVersion();
	void takeMeasurementsWithBulb();
	uint8_t getTemperature();
	float getTemperatureF();
	void setMeasurementMode(uint8_t mode);
	uint8_t getMeasurementMode();
	bool dataAvailable();
	void enableIndicator();
	void disableIndicator();
	void setIndicatorCurrent(uint8_t current);
	void enableBulb();
	void disableBulb();
	void setBulbCurrent(uint8_t current);
	void softReset();
	void setGain(uint8_t gain);
	uint8_t getGain();
	void setIntegrationTime(uint8_t integrationValue);
	uint8_t getIntegrationTime();
	void enableInterrupt();
	void disableInterrupt();

	//Get the various color readings
	int getViolet();
	int getBlue();
	int getGreen();
	int getYellow();
	int getOrange();
	int getRed();

	//Get the various NIR readings
	int getR();
	int getS();
	int getT();
	int getU();
	int getV();
	int getW();

	//get X, Y, Z, NIR, Dark, Clear readings.
	int getX();
	int getY();
	int getZ();
	int getNir();
	int getDark();
	int getClear();

	//Returns the various calibration data
	float getCalibratedViolet();
	float getCalibratedBlue();
	float getCalibratedGreen();
	float getCalibratedYellow();
	float getCalibratedOrange();
	float getCalibratedRed();

	float getCalibratedR();
	float getCalibratedS();
	float getCalibratedT();
	float getCalibratedU();
	float getCalibratedV();
	float getCalibratedW();

	float getCalibratedX();
	float getCalibratedY();
	float getCalibratedZ();
	float getCalibratedX1931();
	float getCalibratedY1931();
	float getCalibratedUPri1976();
	float getCalibratedVPri1976();
	float getCalibratedU1976();
	float getCalibratedV1976();
	float getCalibratedDUV1976();
	int getCalibratedLux();
	int getCalibratedCCT();

private:
	TwoWire *_i2cPort;
	int getChannel(uint8_t channelRegister);
	float getCalibratedValue(uint8_t calAddress);
	float convertBytesToFloat(uint32_t myLong);
	void clearDataAvailable();
	uint8_t virtualReadRegister(uint8_t virtualAddr);
	void virtualWriteRegister(uint8_t virtualAddr, uint8_t dataToWrite);
	void writeRegister(uint8_t addr, uint8_t val);
	uint8_t readRegister(uint8_t addr);

#define AS726X_ADDR 0x49 //7-bit unshifted default I2C Address

	//Register addresses
#define AS726x_DEVICE_TYPE 0x00
#define AS726x_HW_VERSION 0x01
#define AS726x_CONTROL_SETUP 0x04
#define AS726x_INT_T 0x05
#define AS726x_DEVICE_TEMP 0x06
#define AS726x_LED_CONTROL 0x07

#define AS72XX_SLAVE_STATUS_REG 0x00
#define AS72XX_SLAVE_WRITE_REG 0x01
#define AS72XX_SLAVE_READ_REG 0x02

	//The same register locations are shared between the AS7262 and AS7263, they're just called something different
	//AS7262 Registers
#define AS7262_V 0x08
#define AS7262_B 0x0A
#define AS7262_G 0x0C
#define AS7262_Y 0x0E
#define AS7262_O 0x10
#define AS7262_R 0x12
#define AS7262_V_CAL 0x14
#define AS7262_B_CAL 0x18
#define AS7262_G_CAL 0x1C
#define AS7262_Y_CAL 0x20
#define AS7262_O_CAL 0x24
#define AS7262_R_CAL 0x28

	//AS7263 Registers
#define AS7263_R 0x08
#define AS7263_S 0x0A
#define AS7263_T 0x0C
#define AS7263_U 0x0E
#define AS7263_V 0x10
#define AS7263_W 0x12
#define AS7263_R_CAL 0x14
#define AS7263_S_CAL 0x18
#define AS7263_T_CAL 0x1C
#define AS7263_U_CAL 0x20
#define AS7263_V_CAL 0x24
#define AS7263_W_CAL 0x28

	//AS7261 Registers
#define AS7261_X 0x08 //16b
#define AS7261_Y 0x0A //16b
#define AS7261_Z 0x0C //16b
#define AS7261_NIR 0x0E //16b
#define AS7261_DARK 0x10 //16b
#define AS7261_CLEAR 0x12 //16b
#define AS7261_X_CAL 0x14
#define AS7261_Y_CAL 0x18
#define AS7261_Z_CAL 0x1C
#define AS7261_X1931_CAL 0x20
#define AS7261_Y1931_CAL 0x24
#define AS7261_UPRI_CAL 0x28
#define AS7261_VPRI_CAL 0x2C
#define AS7261_U_CAL 0x30
#define AS7261_V_CAL 0x34
#define AS7261_DUV_CAL 0x38
#define AS7261_LUX_CAL 0x3C //16b
#define AS7261_CCT_CAL 0x3E //16b

#define AS72XX_SLAVE_TX_VALID 0x02
#define AS72XX_SLAVE_RX_VALID 0x01

#define SENSORTYPE_AS7262 0x3E
#define SENSORTYPE_AS7263 0x3F
#define SENSORTYPE_AS7261 0x40

#define POLLING_DELAY 5 //Amount of ms to wait between checking for virtual register changes

	uint8_t _sensorVersion = 0;
};

#endif