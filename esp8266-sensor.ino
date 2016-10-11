#include <MQTTClient.h>
#include <DallasTemperature.h>
#include <Ticker.h>
#include <ESP8266HTTPUpdateServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266WebServer.h>
#include <WiFiClient.h>
#include <ESP8266WiFi.h>
#include "debug_print.h"
#include "HeaterItem.h"

#define LED		2
#define ONE_WIRE 4

OneWire oneWire(ONE_WIRE);
DallasTemperature sensor(&oneWire);

float temp;

bool flagReportTemp = false;

#define ADDR_LEN 3
const byte address[] = {0x00,0x00,0x0E};

const char* host = "esp8266-webupdate";
const char* update_path = "/firmware";
const char* update_username = "admin";
const char* update_password = "admin";
const char* ssid = "MCR";
const char* password = "5158088515";

const char* mqttHost = "192.168.1.3";
const char* mqttStatusesTopic = "/ehome/heating/statuses/";
const char* tempItem = "temp";

ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;

HeaterItem heater;

WiFiClient wifiClient;
MQTTClient mqttClient;

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
	char topic[100];
	char payload[10];
	char addr[ADDR_LEN*2 + 1];
	uint8_t len = 0;
	heater.getAddressString(addr, &len);
	sprintf(topic, "%sitem_%s_%s", mqttStatusesTopic, addr, tempItem);
	dtostrf(temperature, 4, 2, payload);
	mqttClient.publish(topic, payload);
}

void messageReceived(String topic, String payload, char * bytes, unsigned int length) {
	Serial.print("incoming: ");
	Serial.print(topic);
	Serial.print(" - ");
	Serial.print(payload);
	Serial.println();
}

void setup()
{
	pinMode(LED, OUTPUT);
	digitalWrite(LED, HIGH);
	
	memcpy(heater.address, address, ADDR_LEN);
	
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
	
	mqttClient.begin(mqttHost, wifiClient);
	
	while (!mqttClient.connect("esp8266", "1", "1")) {
		DebugPrint(".");
		delay(1000);
	}
	
	sensor.begin();
	sensor.setResolution(12);
	
	ticker.detach();
	digitalWrite(LED, HIGH);
	
	requestTemp();
}

void loop()
{
	httpServer.handleClient();
	
	if (flagReportTemp) {
		flagReportTemp = false;
		publishMessage(temp);
	}
}
