/* 
* Settings.cpp
*
* Created: 20.12.2016 22:46:56
* Author: Ivanov
*/


#include "Settings.h"

Settings::StructSettings Settings::settings;
char buffer[13];

// default constructor
Settings::Settings()
{
} //Settings

// default destructor
Settings::~Settings()
{
} //~Settings

void fillProperty(char* dest, String* src) {
	memset(buffer, '\0', 13);
	src->toCharArray(buffer, 13);
	memcpy(dest, buffer, src->length());
}

void Settings::setAddress(String* address) {
	memset(buffer, '\0', 13);
	address->toCharArray(buffer, 7);
	memcpy(settings.address, buffer, 7);
}

void Settings::setSsid(String* ssid) {
	fillProperty(settings.ssid, ssid);
}

void Settings::setHostName(String* hostname) {
	fillProperty(settings.hostname, hostname);
}

void Settings::setPassword(String* password) {
	fillProperty(settings.password, password);
}

void Settings::setMqttHost(String* mqttHost) {
	fillProperty(settings.mqttHost, mqttHost);
}

void Settings::setMqttUser(String* mqttUser) {
	fillProperty(settings.mqttUser, mqttUser);
}

void Settings::setMqttPassword(String* mqttPassword) {
	fillProperty(settings.mqttPassword, mqttPassword);
}

void* Settings::getSettingsPointer() {
	return &settings;
}

uint8_t Settings::getSettingsSize() {
	return sizeof(settings);
}