#include "eepromAddr.h"
#include <OneWire.h>
#include <PubSubClient.h>
#include <DallasTemperature.h>
#include <Ticker.h>
#include <ESP8266HTTPUpdateServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266WebServer.h>
#include <WiFiClient.h>
#include <ESP8266WiFi.h>
#include "debug_print.h"
#include "HeaterItem.h"
#include "config.h"
#include "mqtt_interface.h"

#define LED			2
#define ONE_WIRE	5
#define RELAY		4

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
		Serial.println("Disconnected");
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
		eepromWriteItem(IS_AUTO);
	} else
	if (strcmp(command, isEnabledItem)) {
		heater.isEnabled = getBoolPayload(payload);
		eepromWriteItem(IS_ENABLED);
	} else
	if (strcmp(command, isOnItem)) {
		if (!heater.isAuto) {
			heater.isOn = getBoolPayload(payload);
			eepromWriteItem(IS_ON);
		}
	} else
	if (strcmp(command, targetTempItem)) {
		float temp = strtod(payload, nullptr);
		heater.setTargetTemperature(temp);
		eepromWriteItem(TARGET_TEMP);
	} else
	if (strcmp(command, hysteresisItem)) {
		hysteresis = strtod(payload, nullptr);
		eepromWriteItem(HYSTERESIS);
	} else
	if (strcmp(command, temperatureAdjustItem)) {
		float tempAdjust = strtod(payload, nullptr);
		heater.setTemperatureAdjust(tempAdjust);
		eepromWriteItem(TEMP_ADJUST);
	} else {
		return;
	}

}

void eepromWriteItem(uint8_t addr) {
	
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

	ticker.attach_ms(50, blink);
	
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
