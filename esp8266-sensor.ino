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

#define LED		2
#define ONE_WIRE 4

OneWire oneWire(ONE_WIRE);
DallasTemperature sensor(&oneWire);

float temp;

bool flagReportTemp = false;

#define ADDR_LEN 3
const byte address[ADDR_LEN] = {0x00,0x00,0x0E};
char addressStr[ADDR_LEN * 2 + 1];


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
	//Serial.printf(topic, "%s: %s\n", topic, payload);
	Serial.println(topic);
	Serial.println(payload);
	boolean result = mqttClient.publish(topic, payload, false);
	Serial.println(result);
}

void messageReceived(char* topic, unsigned char* payload, unsigned int length) {

}

void mqttConnect() {
	while (!mqttClient.connect(addressStr)) {
		DebugPrint(".");
		delay(1000);
	}
	mqttClient.subscribe(m
}

void setup()
{
	pinMode(LED, OUTPUT);
	digitalWrite(LED, HIGH);
	
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
	} else {
		mqttClient.loop();
	}
	
	if (flagReportTemp) {
		flagReportTemp = false;
		publishMessage(temp);
	}
}
