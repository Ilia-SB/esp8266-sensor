// mqtt_interface.h

#ifndef _MQTT_INTERFACE_h
#define _MQTT_INTERFACE_h

const char* mqttStatusesTopic = "ehome/heating/statuses/";
const char* mqttCommandsTopic = "ehome/heating/commands/#";

const char* presenceItem = "presence";
const char* tempItem = "temp";
const char* isAutoItem = "isauto";
const char* isOnItem = "ison";
const char* isEnabledItem = "isenabled";
const char* targetTempItem = "targettemp";
const char* hysteresisItem = "hysteresis";
const char* temperatureAdjustItem = "tempadjust";
const char* eraseSettingsItem = "erasesettings";
const char* eepromErrorItem = "eeprom_error";
const char* temperatureReadErrorItem = "temp_read_error";

const char* ON = "ON";

#endif

