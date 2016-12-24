/* 
* HeaterItem.h
*
* Created: 28.01.2016 15:32:58
* Author: Ivanov
*/


#ifndef __HEATERITEM_H__
#define __HEATERITEM_H__

#include "Arduino.h"

#define ADDR_LEN 3

class HeaterItem
{
//variables
public:
	boolean isEnabled;
	byte address[ADDR_LEN];
	byte sensorAddress[8];
	byte port;
	byte pin;
	boolean isAuto;
	uint16_t powerConsumption;
	boolean isOn;
	boolean wantsOn;
	byte priority;
	boolean isConnected;
	boolean actualState;
protected:
private:
	float temperature = 0;
	float targetTemperature = 0;
	float delta = 0;
	float temperatureAdjust = 0;

//functions
public:
	HeaterItem();
	~HeaterItem();
	bool operator>( const HeaterItem &c );
	//HeaterItem& operator=( const HeaterItem &c );
	void setTemperature(float temp);
	float getTemperature();
	void getTemperatureBytes(byte *array);
	void setTargetTemperature(float temp);
	float getTargetTemperature();
	void setTemperatureAdjust(float temp);
	float getTemperatureAdjust();
	void getTemperatureAdjustBytes(byte *array);
	float getDelta();
	void getAddressString(char* str, uint8_t* len);
protected:
private:
	HeaterItem( const HeaterItem &c );
	void byteToHexStr(const byte* value, uint8_t size, char* str, uint8_t* len);
}; //HeaterItem

#endif //__HEATERITEM_H__
