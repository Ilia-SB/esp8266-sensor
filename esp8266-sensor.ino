#include <EEPROM.h>
#include <OneWire.h>
#include <PubSubClient.h>
#include <DallasTemperature.h>
#include <Ticker.h>
#include <ESP8266HTTPUpdateServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266WebServer.h>
#include <WiFiClient.h>
#include <ESP8266WiFi.h>
#include "FS.h"
#include "debug_print.h"
#include "HeaterItem.h"
#include "config.h"
#include "mqtt_interface.h"
#include "eepromAddr.h"
#include "Settings.h"
/*
extern "C" {
	#include "user_interface.h"
}
*/


#define LED			2
#define ONE_WIRE	5
#define RELAY		4

static PROGMEM prog_uint32_t crc_table[16] = {0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac, 0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
	0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c, 0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c};
	
char SETTINGS_HTML[2048];
	
Settings sett;
File configFile, settingsFile;
#define CONFIG_BUFFER_LEN 19
uint8_t configBuffer[CONFIG_BUFFER_LEN+1];

OneWire oneWire(ONE_WIRE);
DallasTemperature sensor(&oneWire);

float temp;
float hysteresis;
unsigned long settingsServerStartTime;

bool flagProcessHeater = false;
bool flagSetEepromError = false, flagClearEepromError = false;
bool flagWifiConnected = false, flagWifiIsConnecting = false;
bool flagNetworkServicesInitialized = false;
bool flagMqttIsConnected = false, flagMqttIsConnecting = false;
bool flagMqttTryConnect = false;
bool flagSettingsServerStarted = false;

#define MAX_COMMAND_LEN 15

ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;

HeaterItem heater;

WiFiEventHandler wifiConnectHandler, wifiDisconnectHandler;
WiFiClient wifiClient;
PubSubClient mqttClient;

Ticker blinker;
Ticker ticker;
Ticker configSaver;
Ticker mqttConnector;

void blink(void);
void requestTemp(void);
void getTemp(void);
void publishMessage(float);

void blink() {
	int state = digitalRead(LED);
	digitalWrite(LED, !state);
}

unsigned long elapsedSince(unsigned long start) {
		return (unsigned long)(millis() - start);
}

void requestTemp() {
	digitalWrite(LED, LOW);
	sensor.requestTemperatures();
	ticker.attach(1, getTemp);
}

void getTemp() {
	digitalWrite(LED, HIGH);
	uint8_t addr[8];
	sensor.getAddress(addr, 0);
	heater.setTemperature(sensor.getTempC(addr));
	DebugPrintln(heater.getTemperature());
	flagProcessHeater = true;
	ticker.attach(30, requestTemp);
}

void publishMessageF(const char* item, const char* addr, float value, bool persist) {
	if (!mqttClient.connected()) {
		DebugPrintln("Mqtt not connected.");
		return;
	}
	char topic[100];
	char payload[10];
	sprintf(topic, "%sitem_%s_%s", mqttStatusesTopic, addr, item);
	if (value < 0) {
		if (value >= -10) {
			dtostrf(value, 5, 2, payload);			
		} else {
			dtostrf(value, 6, 2, payload);
		}
	} else {
		if (value < 10) {
			dtostrf(value, 4, 2, payload);	
		} else {
				dtostrf(value, 5, 2, payload);
		}
	}

	mqttClient.publish(topic, payload, persist);
}

void publishMessageI(const char* item, const char* addr, int value, bool persist) {
	if (!mqttClient.connected()) {
		DebugPrintln("Mqtt not connected.");
		return;
	}
	char topic[100];
	char payload[10];
	sprintf(topic, "%sitem_%s_%s", mqttStatusesTopic, addr, item);
	sprintf(payload, "%d", value);
	mqttClient.publish(topic, payload, persist);
}

void publishMessageB(const char* item, const char* addr, bool state, bool persist) {
	if (!mqttClient.connected()) {
		DebugPrintln("Mqtt not connected.");
		return;
	}
	char topic[100];
	char payload[4];
	sprintf(topic, "%sitem_%s_%s", mqttStatusesTopic, addr, item);
	
	if (state) {
		memcpy(payload, "ON", 3);
	} else {
		memcpy(payload, "OFF", 4);
	}

	mqttClient.publish(topic, payload, persist);
}

void publishMessageS(const char* addr, const char* text, bool persist) {
	if (!mqttClient.connected()) {
		DebugPrintln("Mqtt not connected.");
		return;
	}
	char topic[100];
	char payload[256];
	sprintf(topic, "%s%s", mqttDebugTopic, addr);
	
	sprintf (payload, "%s", text);

	mqttClient.publish(topic, payload, persist);
}

void messageReceived(char* topic, unsigned char* pld, unsigned int pldLength) {
	char itemAddr[ADDR_LEN*2+1];
	char command[MAX_COMMAND_LEN+1];
	char payload[10];
	
	if (!parseMessage(topic, command, itemAddr)) {
		return;
	}

	//check if the command is for this unit
	if (strcmp(sett.settings.address, itemAddr)) {
		return; //not for this unit
	}
	
	//populate payload
	memcpy(payload, pld, pldLength);
	payload[pldLength] = '\0';	
	
	executeCommand(command, payload);
}

void executeCommand(const char* command, const char* payload) {
	DebugPrintf("executeCommand(): %s | %s\n", command, payload);
	if (!strcmp(command, isAutoItem)) {
		DebugPrintln("isAuto");
		heater.isAuto = getBoolPayload(payload);

		if (!heater.isAuto) {
			heater.isOn = false;
		}
		publishMessageB(isAutoItem, sett.settings.address, heater.isAuto, true);
		publishMessageB(isOnItem, sett.settings.address, heater.isOn, false);
	} else
	if (!strcmp(command, isEnabledItem)) {
		DebugPrintln("isEnabled");
		heater.isEnabled = getBoolPayload(payload);
		if (!heater.isEnabled) {
			heater.isAuto = false;
			heater.isOn = false;
		}
		publishMessageB(isEnabledItem, sett.settings.address, heater.isEnabled, true);
		publishMessageB(isAutoItem, sett.settings.address, heater.isAuto, true);
		publishMessageB(isOnItem, sett.settings.address, heater.isOn, false);
	} else
	if (!strcmp(command, isOnItem)) {
		DebugPrintln("isOn");
		if (!heater.isAuto) {
			heater.isOn = getBoolPayload(payload);
		}
		publishMessageB(isOnItem, sett.settings.address, heater.isOn, false);
	} else
	if (!strcmp(command, targetTempItem)) {
		DebugPrintln("setTargetTemp");
		float temp = strtod(payload, nullptr);
		heater.setTargetTemperature(temp);
		publishMessageI(targetTempItem, sett.settings.address, (int)heater.getTargetTemperature(), true);
	} else
	if (!strcmp(command, hysteresisItem)) {
		DebugPrintln("setHysteresis");
		hysteresis = strtod(payload, nullptr);
		publishMessageF(hysteresisItem, sett.settings.address, hysteresis, true);
	} else
	if (!strcmp(command, temperatureAdjustItem)) {
		DebugPrintln("setTempAdjust");
		float tempAdjust = strtod(payload, nullptr);
		heater.setTemperatureAdjust(tempAdjust);
		publishMessageF(temperatureAdjustItem, sett.settings.address, heater.getTemperatureAdjust(), true);
	} else {
		DebugPrintln("Unknown");
		return;
	}
	
	
	configSaver.detach();
	configSaver.attach(5, configSaverWorker);
	flagProcessHeater = true;
}

void configSaverWorker() {
	saveConfig(&heater, CONFIG_BUFFER_LEN);
	configSaver.detach();
}

uint8_t serialize(HeaterItem* item, uint8_t* buffer) {
	float tmp;
	uint8_t* start = buffer;
	*buffer++ = item->isEnabled;
	*buffer++ = item->isAuto;
	*buffer++ = item->isOn;
	tmp = item->getTargetTemperature();
	memcpy(buffer, &tmp, sizeof tmp);
	buffer += sizeof tmp;
	tmp = item->getTemperatureAdjust();
	memcpy(buffer, &tmp, sizeof tmp);
	buffer += sizeof tmp;
	memcpy(buffer, &hysteresis, sizeof hysteresis);
	buffer += sizeof hysteresis;
	return buffer - start;
}

void deserialize(uint8_t* buffer, HeaterItem* item) {
	float tmp;
	item->isEnabled = *buffer++;
	item->isAuto = *buffer++;
	item->isOn = *buffer++;
	memcpy(&tmp, buffer, sizeof tmp);
	buffer += sizeof tmp;
	item->setTargetTemperature(tmp);
	memcpy(&tmp, buffer, sizeof tmp);
	buffer += sizeof tmp;
	item->setTemperatureAdjust(tmp);
	memcpy(&hysteresis, buffer, sizeof hysteresis);
}

bool getBoolPayload(const char* payload) {
	if (!strcmp(payload, ON)) {
		return true;
	} else {
		return false;
	}
}

void mqttConnect() {
	if (!mqttClient.connected() && flagWifiConnected) {
		DebugPrintln("Connecting to mqtt server");
		flagMqttTryConnect = true;
		//mqttClient.connect(sett.settings.address);
	}
}


bool parseMessage(const char* topic, char* command, char* item) {
	//find last slash in topic
	char tempItem[MAX_COMMAND_LEN + ADDR_LEN * 2 + 6 + 1]; //item_xxxxxx_MAX_COMMAND_LEN
	uint8_t slashPosition = strlen(topic) + 1;
	for (uint8_t i=strlen(topic)-1; i-->0;) {
		if(topic[i] == '/') {
			slashPosition = i;
			break;
		} 
	}
	if (slashPosition == strlen(topic) + 1) {
		return false;
	}
	
	//copy everything after slash to tempItem
	memcpy(tempItem, topic + slashPosition + 1, strlen(topic) - slashPosition);
	//find underscores
	uint8_t _positions[MAX_COMMAND_LEN];
	uint8_t _found = 0;
	for (uint8_t i=strlen(tempItem)-1;i-->0;) {
		if(tempItem[i] == '_') {
			_positions[_found] = i;
			_found++;
			if (_found == 2) {
				break;
			}
		}
	}
	
	if (_found != 2) {
		return false;
	}
	
	//copy command and item to corresponding buffers
	uint8_t commandLen = strlen(tempItem) - _positions[0];
	memcpy(command, tempItem + _positions[0] + 1, commandLen);
	command[commandLen] = '\0';
	uint8_t itemLen = _positions[0] - _positions[1] - 1;
	memcpy(item, tempItem + _positions[1] + 1, itemLen);
	item[itemLen] = '\0';
	return true;
}

void loadConfig() {
	DebugPrintln("Loading config");
	configFile = SPIFFS.open("/cfg", "r");
	if (!configFile) {
		DebugPrintln("Config file missing. Creating");
		createConfigFile();
	}
	configFile.readBytes((char*)configBuffer, CONFIG_BUFFER_LEN);
	configFile.close();
	for(int i=0; i<CONFIG_BUFFER_LEN;i++) {
		DebugPrint(configBuffer[i]);DebugPrint(" ");
	}
	DebugPrintln(" ");
	unsigned long loaded_crc;
	memcpy(&loaded_crc, configBuffer + CONFIG_BUFFER_LEN - sizeof(loaded_crc), sizeof(loaded_crc));
	unsigned long crc = crc_byte(configBuffer, CONFIG_BUFFER_LEN - sizeof(crc));
	if (loaded_crc == crc) {
		deserialize(configBuffer, &heater);
		flagClearEepromError = true;
	} else {
		DebugPrintln("CRC error loading config");
		flagSetEepromError = true;
		createConfigFile();
	}
	#ifdef DEBUG
		char tempadj[7], targettemp[7], hyst[7];
		dtostrf(heater.getTemperatureAdjust(), 6, 2, tempadj);
		dtostrf(heater.getTargetTemperature(), 6, 2, targettemp);
		dtostrf(hysteresis, 6, 2, hyst);
		DebugPrintln("Loaded config:");
		DebugPrint("isEnabled: "); DebugPrintln(heater.isEnabled);
		DebugPrint("isAuto: "); DebugPrintln(heater.isAuto);
		DebugPrint("isOn: "); DebugPrintln(heater.isOn);
		DebugPrintf("targetTemp: %s\n", targettemp);
		DebugPrintf("tempAdjust: %s\n", tempadj);
		DebugPrintf("hysteresis: %s\n", hyst);
	#endif
	
	//TODO: crc and eeprom error reporting
}

void startSettingsServer() {
	sprintf(SETTINGS_HTML,
		"<!DOCTYPE HTML>"
		"<html>"
		"<head>"
		"<meta http-equiv=\"Content-Type\" content=\"text/html; charset=UTF-8\">"
		"<title>Settings</title>"
		"</head>"
		"<body>"
		"<form method=\"post\" action=\"/savesettings\">"
		"<h2>Settings</h2>"
		"<p></p>"
		"</div>"
		"<ul >"
		"<li>"
		"<label>Unit address </label>"
		"<input type=\"text\" name=\"addr\" maxlength=\"6\" value=\"%s\"/>"
		"</li>"
		"<li>"
		"<label>WiFi SSID </label>"
		"<input type=\"text\" name=\"ssid\" maxlength=\"31\" value=\"%s\"/>"
		"</li>"
		"<li>"
		"<label>WiFi password </label>"
		"<input type=\"password\" name=\"password\" maxlength=\"31\" value=\"%s\"/>"
		"</li>"
		"<li>"
		"<label>WiFi hostname </label>"
		"<input type=\"text\" name=\"hostname\" maxlength=\"31\" value=\"%s\"/>"
		"</li>"
		"<li>"
		"<label>Mqtt host </label>"
		"<input type=\"text\" name=\"mqttHost\" maxlength=\"31\" value=\"%s\"/>"
		"</li>"
		"<li>"
		"<label>Mqtt PORT </label>"
		"<input type=\"text\" name=\"mqttPort\" maxlength=\"5\" value=\"%d\"/>"
		"</li>"
		"<li>"
		"<label>Mqtt user </label>"
		"<input type=\"text\" name=\"mqttUser\" maxlength=\"31\" value=\"%s\"/>"
		"</li>"
		"<li>"
		"<label>Mqtt password </label>"
		"<input maxlength=\"31\" name=\"mqttPassword\" value=\"%s\"/>"
		"</li>"
		"<li>"
		"<input type=\"submit\" name=\"submit\" value=\"Submit\" />"
		"</li>"
		"</ul>"
		"</form>"
		"</body>"
		"</html>",
	sett.settings.address, sett.settings.ssid, sett.settings.password, sett.settings.hostname, sett.settings.mqttHost, sett.settings.mqttPort, sett.settings.mqttUser, sett.settings.mqttPassword);
	
	DebugPrintln("Starting settings server.");
	DebugPrintln(SETTINGS_HTML);
	httpServer.on("/", handleRoot);
	httpServer.on("/savesettings", handleSaveSettings);
}

void startWifiAP() {
	DebugPrintln("Starting WiFi access point.");
	const char *ssid = "Sensor";
	WiFi.persistent(false);
	WiFi.mode(WIFI_STA);
	WiFi.softAP(ssid);
}

void loadSettings() {
	DebugPrintln("Loading settings");
	settingsFile = SPIFFS.open("/settings", "r");
	if (!settingsFile) {
		DebugPrintln("Settings file missing. Going into settings mode.");
		startWifiAP();
		startSettingsServer();
		httpServer.begin();
		while(true) {
			httpServer.handleClient();
			yield();
		}
		return;
	}
	byte settingsBuffer[sett.getSettingsSize() + sizeof(unsigned long)];
	memset(settingsBuffer, '\0', 100);
	DebugPrintf("Reading %d bytes.\n", sett.getSettingsSize());
	uint8_t dataLen = sett.getSettingsSize();
	uint8_t read = settingsFile.readBytes((char*)settingsBuffer, dataLen);
	DebugPrintf("Read: %d bytes\n", read);
	for(uint8_t i=0; i<dataLen; i++) {
		DebugPrint(settingsBuffer[i]);
	}
	DebugPrintln();
	unsigned long loaded_crc;
	settingsFile.readBytes((char*)&loaded_crc, sizeof(loaded_crc));
	settingsFile.close();
	unsigned long crc = crc_byte(settingsBuffer, sett.getSettingsSize());
	if (loaded_crc != crc) {
		DebugPrint(loaded_crc); DebugPrint("<>"); DebugPrintln(crc);
		DebugPrintln("CRC error loading settings. Going into settings mode.");
		flagSetEepromError = true;
		startWifiAP();
		startSettingsServer();
		httpServer.begin();
		while(true) {
			httpServer.handleClient();
			yield();
		}
		return;
	}
	memcpy(sett.getSettingsPointer(), settingsBuffer, sett.getSettingsSize());
	DebugPrintln("Loaded settings:");
	DebugPrintf("Unit address: %s\r\n", sett.settings.address);
	DebugPrintf("WiFi SSID: %s\r\n", sett.settings.ssid);
	DebugPrintf("WiFi password: %s\r\n", sett.settings.password);
	DebugPrintf("WiFi hostname: %s\r\n", sett.settings.hostname);
	DebugPrintf("Mqtt host: %s\r\n", sett.settings.mqttHost);
	DebugPrintf("Mqtt port: %d\r\n", sett.settings.mqttPort);
	DebugPrintf("Mqtt user: %s\r\n", sett.settings.mqttUser);
	DebugPrintf("Mqtt password: %s\r\n", sett.settings.mqttPassword);
}

void handleRoot() {
	DebugPrintln("Displaying settings page.");
	httpServer.send(200, "text/html", SETTINGS_HTML);
}

void handleSaveSettings() {
	DebugPrintln("Checking settings file.");
	settingsFile= SPIFFS.open("/settings", "w");
	if (!settingsFile) {
		DebugPrintln("Settings file creation failed. Formatting SPIFFS");
	}
	settingsFile.close();
	String s;
	s=httpServer.arg("addr");
	sett.setAddress(&s);
	s=httpServer.arg("ssid");
	sett.setSsid(&s);
	s=httpServer.arg("password");
	sett.setPassword(&s);
	s=httpServer.arg("hostname");
	sett.setHostName(&s);
	s=httpServer.arg("mqttHost");
	sett.setMqttHost(&s);
	sett.settings.mqttPort = httpServer.arg("mqttPort").toInt();
	s=httpServer.arg("mqttUser");
	sett.setMqttUser(&s);
	s=httpServer.arg("mqttPassword");
	sett.setMqttPassword(&s);
	saveSettings(&sett);
	httpServer.send(200, "text/html", "Rebooting...");
	SPIFFS.end();
	ESP.restart();
}

void createConfigFile() {
	heater.isEnabled = true;
	heater.isOn = false;
	heater.isAuto = true;
	heater.setTargetTemperature(5);
	heater.setTemperatureAdjust(0);
	hysteresis = 3;
	
	saveConfig(&heater, CONFIG_BUFFER_LEN);
}

void saveConfig(HeaterItem* item, size_t len) {
	uint8_t dataLen = serialize(item, configBuffer);
	unsigned long crc = crc_byte(configBuffer, dataLen);
	memcpy(configBuffer + dataLen, &crc, sizeof crc);
	configBuffer[CONFIG_BUFFER_LEN] = '\0';
	DebugPrintln("Saving config.");
	for(int i=0; i<CONFIG_BUFFER_LEN;i++) {
		DebugPrint(configBuffer[i]);DebugPrint(" ");
	}
	DebugPrintln(" ");
	configFile = SPIFFS.open("/cfg", "w");
	if (!configFile) {
		DebugPrintln("Config file creation failed.");
		return;
	} else {
		uint8_t written = configFile.write(configBuffer, len);
		if (written == len) {
			DebugPrintln("Done.");
		} else {
			DebugPrintln("Writing config failed.");
		}
		DebugPrintf("Written %d bytes.\n", written);
		configFile.close();
	}
}

void saveSettings(const Settings* s) {
	DebugPrintln("Saving settings.");
	byte settingsBuffer[sett.getSettingsSize() + sizeof(unsigned long)];
	uint8_t dataLen = s->getSettingsSize();
	memcpy(settingsBuffer, s->getSettingsPointer(), dataLen);
	DebugPrintf("Settings size: %d\n", dataLen);
	DebugPrintf("Settings pointer: %p\n", s->getSettingsPointer());
	DebugPrint("Raw settings: ");
	for (uint8_t i=0; i<dataLen; i++) {
		DebugPrint(settingsBuffer[i]);
	}
	DebugPrintln();
	unsigned long crc = crc_byte(settingsBuffer, dataLen);
	DebugPrintf("CRC: %lu\n", crc);
	memcpy(settingsBuffer + dataLen, &crc, sizeof(crc));
	dataLen += sizeof(crc);
	DebugPrintln();
	DebugPrintln("Opening settings file.");
	settingsFile = SPIFFS.open("/settings", "w");
	if (!settingsFile) {
		DebugPrintln("Settings file creation failed.");
		return;
	} else {
		DebugPrintln("Writing settings.");
		uint8_t written = settingsFile.write(settingsBuffer, dataLen);
		if (written == dataLen) {
			DebugPrintln("Done.");
		} else {
			DebugPrintln("Writing config failed.");
		}
		DebugPrintf("Written %d bytes.\n", written);
		settingsFile.close();
	}
}

unsigned long crc_update(unsigned long crc, byte data)
{
	byte tbl_idx;
	tbl_idx = crc ^ (data >> (0 * 4));
	crc = pgm_read_dword_near(crc_table + (tbl_idx & 0x0f)) ^ (crc >> 4);
	tbl_idx = crc ^ (data >> (1 * 4));
	crc = pgm_read_dword_near(crc_table + (tbl_idx & 0x0f)) ^ (crc >> 4);
	return crc;
}

unsigned long crc_byte(byte *b, int len)
{
	unsigned long crc = ~0L;
	uint8_t i;

	for (i = 0 ; i < len ; i++)
	{
		crc = crc_update(crc, *b++);
	}
	crc = ~crc;
	return crc;
}

void onWIFIConnected(const WiFiEventStationModeGotIP& event) {
	//DebugPrintln("WIFI connection event");
	flagWifiConnected = true;
	flagWifiIsConnecting = false;
}

void onWIFIDisconnected(const WiFiEventStationModeDisconnected& event) {
	//DebugPrintln("WIFI disconnection event");
	flagWifiConnected = false;
	flagNetworkServicesInitialized = false;
	flagMqttIsConnected = false;
	flagMqttIsConnecting = false;
	blinker.attach_ms(50, blink);
}

void setup()
{
	pinMode(LED, OUTPUT);
	digitalWrite(LED, HIGH);
	pinMode(RELAY, OUTPUT);
	digitalWrite(RELAY, LOW);

	#ifdef DEBUG
		Serial.begin(115200);
	#endif
	DebugPrintln();
	DebugPrintln("Starting...");
	FSInfo fsInfo;
	SPIFFS.begin();
		
	loadSettings();
	loadConfig();
	DebugPrintf("This unit's address is: %s\n", sett.settings.address);
	DebugPrintf("Setting WIFI hostname to: %s\n", sett.settings.hostname);
	
	WiFi.persistent(false);
	wifiConnectHandler = WiFi.onStationModeGotIP(&onWIFIConnected);
	wifiDisconnectHandler = WiFi.onStationModeDisconnected(&onWIFIDisconnected);
	WiFi.mode(WIFI_STA);
	
	mqttClient.setClient(wifiClient);
	mqttClient.setServer(sett.settings.mqttHost, sett.settings.mqttPort);
	mqttClient.setCallback(messageReceived);
	
	sensor.begin();
	sensor.setResolution(12);

	digitalWrite(LED, LOW);
	
	requestTemp();
}

void loop()
{
	yield();
	if (flagMqttTryConnect) {
		flagMqttTryConnect = false;
		char topic[100];
		char message[4] = "OFF";
		sprintf(topic, "%sitem_%s_%s", mqttStatusesTopic, sett.settings.address, presenceItem);

		mqttClient.connect(sett.settings.address, sett.settings.mqttUser, sett.settings.mqttPassword, topic, 0, true, message);
	}
	
	if (flagWifiConnected) {
		if (!flagNetworkServicesInitialized) {
			DebugPrintln("Wifi connection established.");
			DebugPrintln("Starting network services.");
			
			MDNS.begin(host);
				
			httpUpdater.setup(&httpServer, update_path, update_username, update_password);
			startSettingsServer();
			httpServer.begin();
				
			MDNS.addService("http", "tcp", 80);
			publishMessageS(sett.settings.address, "Network services started", false);
			DebugPrintf("HTTPUpdateServer ready! Open http://%s.local%s in your browser and login with username '%s' and password '%s'\n", host, update_path, update_username, update_password);
				
			flagNetworkServicesInitialized = true;
			blinker.detach();
		} else { //If wifi is connected and network services initialized
			httpServer.handleClient();
		}
		
		if(!flagMqttIsConnected) {
			if (!flagMqttIsConnecting) {
				DebugPrintln("Starting mqtt connection.");
				blinker.attach_ms(100, blink);
				mqttConnector.attach(1, mqttConnect);
				flagMqttIsConnecting = true;
			}
			
			if (mqttClient.connected()) {
				DebugPrintln("Mqtt connection established.");
				mqttConnector.detach();
				blinker.detach();
				mqttClient.subscribe(mqttCommandsTopic, 1);
				publishMessageB(presenceItem, sett.settings.address, true, true);
				flagMqttIsConnected = true;
				flagMqttIsConnecting = false;
				
				publishMessageF(temperatureAdjustItem, sett.settings.address, heater.getTemperatureAdjust(), true);
				publishMessageI(targetTempItem, sett.settings.address, (int)heater.getTargetTemperature(), true);
				publishMessageF(hysteresisItem, sett.settings.address, hysteresis, true);
				publishMessageB(isEnabledItem, sett.settings.address, heater.isEnabled, true);
				publishMessageB(isAutoItem, sett.settings.address, heater.isAuto, true);
			}
		} else { //If mqtt is connected
			if (!mqttClient.connected()) { //Detect mqtt disconnect
				DebugPrintln("Mqtt disconnected.");
				flagMqttIsConnected = false;
				flagMqttIsConnecting = false;
				return;
			}
			
			mqttClient.loop();
			
			if (flagSetEepromError) {
				flagSetEepromError = false;
				publishMessageB(eepromErrorItem, sett.settings.address, true, true);
			}
			
			if (flagClearEepromError) {
				flagClearEepromError = false;
				publishMessageB(eepromErrorItem, sett.settings.address, false, true);
			}						
		}
	} else { //If wifi is not connected
		if (!flagWifiIsConnecting) { //and is not connecting
			DebugPrintln("Starting connection to WiFi network.");
			//wifi_station_set_hostname(sett.settings.hostname);
			WiFi.hostname(sett.settings.hostname);
			WiFi.begin(sett.settings.ssid, sett.settings.password);
			blinker.attach_ms(50, blink);
			flagWifiIsConnecting = true;
		} else { //Wifi is connecting. Check status
			if (WiFi.status() == WL_NO_SSID_AVAIL) { //No network
				if (!flagSettingsServerStarted) { //if settings server not started
					DebugPrintln("No WiFi network. Going into settings mode for 1 minute.");
					startWifiAP();
					startSettingsServer();
					httpServer.begin();
					settingsServerStartTime = millis();
					flagSettingsServerStarted = true;
				} else { //if settings server is already running
					if (elapsedSince(settingsServerStartTime) < 60000) { //if less than one minute passed
						httpServer.handleClient();
					} else { //one minute has passed
						if (WiFi.softAPgetStationNum() == 0) { //if no clients connected we can close settings server and check for available network again
							DebugPrintln("Retrying connection to WiFi network.");
							httpServer.close();
							flagSettingsServerStarted = false;
							DebugPrintln("Starting wifi connection.");
							WiFi.hostname(sett.settings.hostname);
							WiFi.begin(sett.settings.ssid, sett.settings.password);
							blinker.attach_ms(50, blink);
							flagWifiIsConnecting = true;
						} else { //clients connected, process them
							httpServer.handleClient();
						}
					}
				}
			}
		}
	}
	
	if (flagProcessHeater) {
		flagProcessHeater = false;
		
		if (heater.isEnabled) {
			
			//AUTO MODE
			if (heater.isAuto) {
				if (heater.getTemperature() > heater.getTargetTemperature() + hysteresis) {
					digitalWrite(RELAY, LOW);
					heater.isOn = false;
				}
				if (heater.getTemperature() < heater.getTargetTemperature()) {
					digitalWrite(RELAY, HIGH);
					heater.isOn = true;
				}
			} else {
			
			//MANUAL MODE
				if (heater.isOn) {
					digitalWrite(RELAY, HIGH);
				} else {
					digitalWrite(RELAY, LOW);
				}
			}
			
			if(flagWifiConnected && flagMqttIsConnected) {
				publishMessageF(tempItem, sett.settings.address, heater.getTemperature(), false);
				publishMessageB(isOnItem, sett.settings.address, heater.isOn, false);
			}
		}
	}
}
