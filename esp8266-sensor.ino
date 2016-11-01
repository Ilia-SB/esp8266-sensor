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

#define LED			2
#define ONE_WIRE	5
#define RELAY		4

File configFile;
#define CONFIG_BUFFER_LEN 15
uint8_t configBuffer[CONFIG_BUFFER_LEN];

OneWire oneWire(ONE_WIRE);
DallasTemperature sensor(&oneWire);

float temp;
float hysteresis;

bool flagReportTemp = false;

#define ADDR_LEN 3
const byte address[ADDR_LEN] = {0x00,0x00,0x0E};
char addressStr[ADDR_LEN * 2 + 1];

#define MAX_COMMAND_LEN 15

ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;

HeaterItem heater;

WiFiClient wifiClient;
PubSubClient mqttClient;

Ticker ticker;

void blink(void);
void requestTemp(void);
void getTemp(void);
void publishMessage(float);

void blink() {
	int state = digitalRead(LED);
	digitalWrite(LED, !state);
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
	temp = sensor.getTempC(addr);
	DebugPrintln(temp);
	flagReportTemp = true;
	ticker.attach(30, requestTemp);
}

void publishMessage(float temperature) {
	if (!mqttClient.connected()) {
		DebugPrintln("Disconnected");
		return;
	}
	char topic[100];
	char payload[10];
	char addr[ADDR_LEN*2 + 1];
	uint8_t len = 0;
	heater.getAddressString(addr, &len);
	sprintf(topic, "%sitem_%s_%s", mqttStatusesTopic, addr, tempItem);
	dtostrf(temperature, 6, 2, payload);
	bool result = mqttClient.publish(topic, payload, false);
}

void publishMessageEepromError(bool state) {
	if (!mqttClient.connected()) {
		DebugPrintln("Disconnected");
		return;
	}
	char topic[100];
	char payload[4];
	
	if (state) {
		memcpy(payload, "ON", 3);
	} else {
		memcpy(payload, "OFF", 4);
	}

	sprintf(topic, "%s%s", mqttStatusesTopic, eepromErrorItem);
	bool result = mqttClient.publish(topic, payload, false);
}

void messageReceived(char* topic, unsigned char* pld, unsigned int pldLength) {
	char itemAddr[ADDR_LEN*2+1];
	char command[MAX_COMMAND_LEN+1];
	char payload[10];
	
	if (!parseMessage(topic, command, itemAddr)) {
		return;
	}
	
	//check if command is for this module
	//if (!)
	
	//populate payload
	memcpy(payload, pld, pldLength);
	payload[pldLength] = '\0';
	
	//check if the command is for this unit
	if (!strcmp(addressStr, itemAddr)) {
		return; //not for this unit
	}
	
	executeCommand(command, payload);
}

void executeCommand(const char* command, const char* payload) {
	if (strcmp(command, isAutoItem)) {
		heater.isAuto = getBoolPayload(payload);
		if (!heater.isAuto) {
			heater.isOn = false;
		}
	} else
	if (strcmp(command, isEnabledItem)) {
		heater.isEnabled = getBoolPayload(payload);
	} else
	if (strcmp(command, isOnItem)) {
		if (!heater.isAuto) {
			heater.isOn = getBoolPayload(payload);
		}
	} else
	if (strcmp(command, targetTempItem)) {
		float temp = strtod(payload, nullptr);
		heater.setTargetTemperature(temp);
	} else
	if (strcmp(command, hysteresisItem)) {
		hysteresis = strtod(payload, nullptr);
	} else
	if (strcmp(command, temperatureAdjustItem)) {
		float tempAdjust = strtod(payload, nullptr);
		heater.setTemperatureAdjust(tempAdjust);
	} else {
		return;
	}

}

void serialize(HeaterItem* item, uint8_t* buffer) {
	float tmp;
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
	if (strcmp(payload, ON)) {
		return true;
	} else {
		return false;
	}
}

void mqttConnect() {
	DebugPrintln("Connecting to mqtt server");
	while (!mqttClient.connect(addressStr)) {
		DebugPrint(".");
		delay(1000);
	}
	mqttClient.subscribe(mqttCommandsTopic);
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
	for (uint8_t i=strlen(tempItem)-1;i++>0;) {
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
	uint8_t commandLen = strlen(targetTempItem) - _positions[0];
	memcpy(command, topic + _positions[0] + 1, commandLen);
	command[commandLen] = '\0';
	uint8_t itemLen = _positions[0] - _positions[1] - 1;
	memcpy(item, topic + _positions[1] + 1, itemLen);
	item[itemLen] = '\0';
	
	return true;
}

void loadConfig() {
	configFile = SPIFFS.open("/cfg", "r");
	if (!configFile) {
		DebugPrintln("Config file missing. Creating");
		createConfigFile();
	}
	configFile.readBytes((char*)configBuffer, CONFIG_BUFFER_LEN);
	configFile.close();
	deserialize(configBuffer, &heater);
	
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
}

void createConfigFile() {
	heater.isEnabled = false;
	heater.isOn = false;
	heater.isAuto = false;
	heater.setTargetTemperature(5);
	heater.setTemperatureAdjust(0);
	hysteresis = 1;
	
	serialize(&heater, configBuffer);
	saveConfig(configBuffer, CONFIG_BUFFER_LEN);
}

void saveConfig(uint8_t* buffer, size_t len) {
	configFile = SPIFFS.open("/cfg", "w");
	if (!configFile) {
		DebugPrintln("Config file creation failed.");
		return;
	} else {
		uint8_t written = configFile.write(buffer, len);
		if (written == len) {
			DebugPrintln("Done.");
		} else {
			DebugPrintln("Writing config failed.");
		}
		DebugPrintf("Written %d bytes.", written);
		configFile.close();
	}
}

void setup()
{
	pinMode(LED, OUTPUT);
	digitalWrite(LED, HIGH);
	pinMode(RELAY, OUTPUT);
	digitalWrite(RELAY, LOW);

	memcpy(heater.address, address, ADDR_LEN);
	uint8_t len;
	heater.getAddressString(addressStr, &len);
	
	#ifdef DEBUG
		Serial.begin(115200);
	#endif
	DebugPrintln();
	DebugPrintln("Starting...");
	
	FSInfo fsInfo;
	SPIFFS.begin();
	if (!SPIFFS.info(fsInfo)) {
		DebugPrintln("File system not formatted. Formatting...");
		if (SPIFFS.format()) {
			DebugPrintln("Done.");
		} else {
			DebugPrintln("Failed.");
		}
	}
	
	loadConfig();
	
	ticker.attach_ms(50, blink);
	
	WiFi.persistent(false);
	WiFi.mode(WIFI_AP_STA);
	WiFi.begin(ssid, password);
	
	while(WiFi.waitForConnectResult() != WL_CONNECTED){
	  WiFi.begin(ssid, password);
	  DebugPrintln("WiFi failed, retrying.");
	}
	
	ticker.detach();
	ticker.attach_ms(100, blink);
	
	MDNS.begin(host);
	
	httpUpdater.setup(&httpServer, update_path, update_username, update_password);
	httpServer.begin();
	
	MDNS.addService("http", "tcp", 80);
	DebugPrintf("HTTPUpdateServer ready! Open http://%s.local%s in your browser and login with username '%s' and password '%s'\n", host, update_path, update_username, update_password);
	
	mqttClient.setClient(wifiClient);
	mqttClient.setServer(mqttHost, mqttPort);
	mqttClient.setCallback(messageReceived);
	mqttConnect();
	
	sensor.begin();
	sensor.setResolution(12);
	
	ticker.detach();
	digitalWrite(LED, HIGH);
	
	requestTemp();
}

void loop()
{
	httpServer.handleClient();
	
	if(!mqttClient.connected()) {
		mqttConnect();
	}
	
	mqttClient.loop();
	
	if (flagReportTemp) {
		flagReportTemp = false;
		publishMessage(temp);
	}
}
