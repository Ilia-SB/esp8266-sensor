/* 
* HeaterItem.cpp
*
* Created: 28.01.2016 15:32:57
* Author: Ivanov
*/


#include "HeaterItem.h"

// default constructor
HeaterItem::HeaterItem()
{
} //HeaterItem

// default destructor
HeaterItem::~HeaterItem()
{
} //~HeaterItem

bool HeaterItem::operator>( const HeaterItem &c ) {
	if(priority < c.priority) {
		return true;
	}
	if(priority > c.priority) {
		return false;
	}
	
	//if priority == c.priority
	if (delta > c.delta) {
		return true;
	}
	//otherwise return false;
	return false;
}

void HeaterItem::setTemperature(float temp) {
	temperature = temp;
	delta = targetTemperature - getTemperature();
}

float HeaterItem::getTemperature() {
	return temperature + temperatureAdjust;
}

void HeaterItem::getTemperatureBytes(byte *array) {
	float temp = getTemperature();
	if (temp < 0) {
		array[0] = 1;
	} else {
		array[0] = 0;
	}
	array[1] = temp;
	array[2] = temp*100 - array[1]*100;
}

void HeaterItem::setTargetTemperature(float temp) {
	targetTemperature = temp;
	delta = targetTemperature - getTemperature();
}

float HeaterItem::getTargetTemperature() {
	return targetTemperature;
}

void HeaterItem::setTemperatureAdjust(float temp) {
	temperatureAdjust = temp;
	delta = targetTemperature - getTemperature();
}

float HeaterItem::getTemperatureAdjust() {
	return temperatureAdjust;
}

void HeaterItem::getTemperatureAdjustBytes(byte *array) {
	if (temperatureAdjust < 0) {
		array[0] = 1;
	} else {
		array[0] = 0;
	}
	array[1] = abs(temperatureAdjust);
	array[2] = abs(temperatureAdjust)*100 - array[1]*100;
}

float HeaterItem::getDelta() {
	return delta;
}

void HeaterItem::getAddressString(char* str, uint8_t* len) {
	byteToHexStr(address, sizeof(address), str, len);
}

void HeaterItem::byteToHexStr(const byte* value, uint8_t size, char* str, uint8_t* len) {
	uint8_t t;
	for (int i=0; i<size; i++) {
		t = value[i] / 16;
		if (t < 10) {
			*str = 48 + t;
		} else {
			*str = 55 + t;
		}
		str++;
		
		t = value[i] % 16;
		if (t<10) {
			*str = 48 + t;
		} else {
			*str = 55 + t;
		}
		str++;
	}
	
	*str = '\0';
	*len = size * 2;
}