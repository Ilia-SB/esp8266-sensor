/* 
* Settings.h
*
* Created: 20.12.2016 22:46:56
* Author: Ivanov
*/


#ifndef __SETTINGS_H__
#define __SETTINGS_H__

#include <stdint.h>
#include <string.h>
#include <Arduino.h>
#include "debug_print.h"

class Settings
{
//variables
public:
	struct __attribute__((__packed__)) StructSettings {
		char address[7];
		char ssid[13];
		char password[13];
		char mqttHost[16];
		int mqttPort;
		char mqttUser[13];
		char mqttPassword[13];
	};
	static StructSettings settings;
protected:
private:

//functions
public:
	Settings();
	~Settings();
	static void setAddress(String* address);
	static void setSsid(String* ssid);
	static void setPassword(String* password);
	static void setMqttHost(String* mqttHost);
	static void setMqttUser(String* mqttUser);
	static void setMqttPassword(String* mqttPassword);
	static void* getSettingsPointer();
	static uint8_t getSettingsSize();
protected:
private:
	Settings( const Settings &c );
	Settings& operator=( const Settings &c );

}; //Settings

#endif //__SETTINGS_H__
