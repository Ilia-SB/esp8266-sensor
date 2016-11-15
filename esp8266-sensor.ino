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

static PROGMEM prog_uint32_t crc_table[16] = {0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac, 0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
	0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c, 0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c};

File configFile;
#define CONFIG_BUFFER_LEN 19
uint8_t configBuffer[CONFIG_BUFFER_LEN+1];

OneWire oneWire(ONE_WIRE);
DallasTemperature sensor(&oneWire);

float temp;
float hysteresis;

bool flagProcessHeater = false;
bool flagSetEepromError = false, flagClearEepromError = false;
bool flagWifiConnected = false, flagWifiIsConnecting = false;
bool flagNetworkServicesInitialized = false;
bool flagMqttIsConnected = false, flagMqttIsConnecting = false;
bool flagMqttTryConnect = false;

#define ADDR_LEN 3
const byte address[ADDR_LEN] = {0x00,0x00,0x11};
char addressStr[ADDR_LEN * 2 + 1];

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

void messageReceived(char* topic, unsigned char* pld, unsigned int pldLength) {
	char itemAddr[ADDR_LEN*2+1];
	char command[MAX_COMMAND_LEN+1];
	char payload[10];
	
	if (!parseMessage(topic, command, itemAddr)) {
		return;
	}

	//check if the command is for this unit
	if (strcmp(addressStr, itemAddr)) {
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
		publishMessageB(isAutoItem, addressStr, heater.isAuto, true);
		publishMessageB(isOnItem, addressStr, heater.isOn, false);
	} else
	if (!strcmp(command, isEnabledItem)) {
		DebugPrintln("isEnabled");
		heater.isEnabled = getBoolPayload(payload);
		if (!heater.isEnabled) {
			heater.isAuto = false;
			heater.isOn = false;
		}
		publishMessageB(isEnabledItem, addressStr, heater.isEnabled, true);
		publishMessageB(isAutoItem, addressStr, heater.isAuto, true);
		publishMessageB(isOnItem, addressStr, heater.isOn, false);
	} else
	if (!strcmp(command, isOnItem)) {
		DebugPrintln("isOn");
		if (!heater.isAuto) {
			heater.isOn = getBoolPayload(payload);
		}
		publishMessageB(isOnItem, addressStr, heater.isOn, false);
	} else
	if (!strcmp(command, targetTempItem)) {
		DebugPrintln("setTargetTemp");
		float temp = strtod(payload, nullptr);
		heater.setTargetTemperature(temp);
		publishMessageI(targetTempItem, addressStr, (int)heater.getTargetTemperature(), true);
	} else
	if (!strcmp(command, hysteresisItem)) {
		DebugPrintln("setHysteresis");
		hysteresis = strtod(payload, nullptr);
		publishMessageF(hysteresisItem, addressStr, hysteresis, true);
	} else
	if (!strcmp(command, temperatureAdjustItem)) {
		DebugPrintln("setTempAdjust");
		float tempAdjust = strtod(payload, nullptr);
		heater.setTemperatureAdjust(tempAdjust);
		publishMessageF(temperatureAdjustItem, addressStr, heater.getTemperatureAdjust(), true);
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
		//mqttClient.connect(addressStr);
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
	memcpy(&loaded_crc, configBuffer + CONFIG_BUFFER_LEN - sizeof loaded_crc, sizeof loaded_crc);
	unsigned long crc = crc_byte(configBuffer, CONFIG_BUFFER_LEN - sizeof crc);
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
	DebugPrintln("WIFI connection event");
	flagWifiConnected = true;
	flagWifiIsConnecting = false;
}

void onWIFIDisconnected(const WiFiEventStationModeDisconnected& event) {
	DebugPrintln("WIFI disconnection event");
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

	memcpy(heater.address, address, ADDR_LEN);
	uint8_t len;
	heater.getAddressString(addressStr, &len);
	
	#ifdef DEBUG
		Serial.begin(115200);
	#endif
	DebugPrintln();
	DebugPrintln("Starting...");
	DebugPrintf("This unit's address is: %s\n", addressStr);	
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
	
	WiFi.persistent(false);
	wifiConnectHandler = WiFi.onStationModeGotIP(&onWIFIConnected);
	wifiDisconnectHandler = WiFi.onStationModeDisconnected(&onWIFIDisconnected);
	WiFi.mode(WIFI_AP_STA);
	
	mqttClient.setClient(wifiClient);
	mqttClient.setServer(mqttHost, mqttPort);
	mqttClient.setCallback(messageReceived);
	
	sensor.begin();
	sensor.setResolution(12);

	digitalWrite(LED, LOW);
	
	requestTemp();
}

void loop()
{
	if (flagMqttTryConnect) {
		flagMqttTryConnect = false;
		char topic[100];
		char message[4] = "OFF";
		sprintf(topic, "%sitem_%s_%s", mqttStatusesTopic, addressStr, presenceItem);

		mqttClient.connect(addressStr, mqttUser, mqttPassword, topic, 0, true, message);
	}
	
	if (flagWifiConnected) {
		if (!flagNetworkServicesInitialized) {
			DebugPrintln("Wifi connection established.");
			DebugPrintln("Starting network services.");
			
			MDNS.begin(host);
				
			httpUpdater.setup(&httpServer, update_path, update_username, update_password);
			httpServer.begin();
				
			MDNS.addService("http", "tcp", 80);
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
				mqttClient.subscribe(mqttCommandsTopic);
				publishMessageB(presenceItem, addressStr, true, true);
				flagMqttIsConnected = true;
				flagMqttIsConnecting = false;
				
				publishMessageF(temperatureAdjustItem, addressStr, heater.getTemperatureAdjust(), true);
				publishMessageI(targetTempItem, addressStr, (int)heater.getTargetTemperature(), true);
				publishMessageF(hysteresisItem, addressStr, hysteresis, true);
				publishMessageB(isEnabledItem, addressStr, heater.isEnabled, true);
				publishMessageB(isAutoItem, addressStr, heater.isAuto, true);
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
				publishMessageB(eepromErrorItem, addressStr, true, true);
			}
			
			if (flagClearEepromError) {
				flagClearEepromError = false;
				publishMessageB(eepromErrorItem, addressStr, false, true);
			}						
		}
	} else { //If wifi is not connected
		if (!flagWifiIsConnecting) {
			DebugPrintln("Starting wifi connection.");
			WiFi.begin(ssid, password);
			blinker.attach_ms(50, blink);
			flagWifiIsConnecting = true;
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
				publishMessageF(tempItem, addressStr, heater.getTemperature(), false);
				publishMessageB(isOnItem, addressStr, heater.isOn, false);
			}
		}
	}
}
