#include "config.h"
#include <PubSubClient.h>
#include <AutoConnect.h>
#include <WiFi.h>
#include <string.h>
#include <Wire.h>
#include "esp_system.h"

#include "DHT.h"
#include "SDS011.h"
#include "ccs811.h"


#define DHT_PIN 19
#define CCS_NWAKE_PIN 23

#define DS_CO2_PIN 5

typedef struct value_s {
	float pm25;
	float pm10;
	float temp;
	float rh;
	uint32_t eco2;
	uint32_t etvoc;
	uint16_t errstat;
	uint16_t raw;
	uint32_t co2;
};
/********************************* PRIVATE FUNCTION PROTOTYPES **********************/

void connectWifi();
void disconnectWifi();
void connectMqtt();
void disconnectMqtt();
uint8_t getConnDetails(char* mac, char* wifiSSID);
void hibernate();
void delayedHibernate(void *parameter);
void setup();
void loop();
void activate_sds();
uint8_t read_dht22(DHT unit, float* temp, float* rh);
void read_ccs811(CCS811 sensor, value_s* data_s, uint8_t read_count);
uint8_t read_sds011(SDS011 sensor, float *pm25, float *pm10, uint8_t count);
uint8_t read_ds_co2(uint8_t pin, uint32_t *co2, uint8_t count);
uint8_t publish_mqtt(value_s* data_s, char* mac, char* wifi_ssid);
void init_wtdg(void);

/********************************* PRIVATE VARIABLES ********************************/
AutoConnect portal;

WiFiClient espClient;
PubSubClient client(espClient);
HardwareSerial port(2);

DHT dht(DHT_PIN, DHT22);
SDS011 sds;
CCS811 ccs811(CCS_NWAKE_PIN);

TaskHandle_t hibernateTaskHandle = NULL;
int bootCount;

char macAddr[18];
char wifiSSID[32];
hw_timer_t *timer = NULL;

/************************************************************************************/
// RESET WATCHDOG FUNCTIONS
void IRAM_ATTR resetModule(){
	ets_printf("reboot\n");
	esp_restart();
}

void init_wtdg(void) {
	timer = timerBegin(0, 80, true); //timer 0, div 80
    timerAttachInterrupt(timer, &resetModule, true);
	timerAlarmWrite(timer, 600000000, false); //set time in us
}

void connectWifi() {
  Serial.println("Starting AutoConnect Wifi portal..");
  AutoConnectConfig conf;
  conf.title = "ESP32 AQI Sensor";
  conf.title = "ESP32 AQI Sensor";
  conf.apid = "ESP32 AQI";
  conf.psk = "ilmavaiva";
  conf.autoReconnect = true;
  
  portal.config(conf);
  portal.begin();

  while (WiFi.status() != WL_CONNECTED) {
	portal.handleClient();
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("");
}

void disconnectWifi() {
  WiFi.disconnect(true);
  Serial.println("WiFi disonnected");
}

void connectMqtt() {
  Serial.println("Connecting to MQTT...");
  client.setServer(MQTT_HOST, MQTT_PORT);

  while (!client.connected()) {
    if (!client.connect(MQTT_CLIENTID, MQTT_USERNAME, MQTT_PASSWORD)) {
      Serial.print("MQTT connection failed:");
      Serial.print(client.state());
      Serial.println("Retrying...");
      delay(MQTT_RETRY_WAIT);
    }
  }

  Serial.println("MQTT connected");
  Serial.println("");
}

void disconnectMqtt() {
  client.disconnect();
  Serial.println("MQTT disconnected");
}

uint8_t getConnDetails(char* mac, char* wifiSSID)
{
	uint8_t macAddr[6];
	WiFi.macAddress(macAddr);
	sprintf(mac, "%02X:%02X:%02X:%02X:%02X:%02X",
	macAddr[0],
	macAddr[1],
	macAddr[2],
	macAddr[3],
	macAddr[4],
	macAddr[5]);

	snprintf(wifiSSID, 32, "%s", WiFi.SSID());

	return 0; //FIXME Should return error codes in case something fails
}

void hibernate() {
  esp_sleep_enable_timer_wakeup(SLEEP_DURATION * 1000000ll);
  Serial.println("Going to sleep now.");
  delay(100);
  esp_deep_sleep_start();
}

void delayedHibernate(void *parameter) {
  delay(EMERGENCY_HIBERNATE*1000); // delay for five minutes
  Serial.println("Something got stuck, entering emergency hibernate...");
  hibernate();
}

void setup() {
	// all action is done when device is woken up
	Serial.begin(115200);
	sds.begin(&port);
	dht.begin();
	Wire.begin();
	ccs811.set_i2cdelay(50);
	pinMode(DS_CO2_PIN, INPUT);
	delay(1000);

	// increase boot count
	bootCount++;

	// create a hibernate task in case something gets stuck
#if SLEEP_DURATION
	xTaskCreate(delayedHibernate, "hibernate", 4096, NULL, 1, &hibernateTaskHandle);
#endif

	// connect to Wifi or start as AP
	connectWifi();

	sds.wakeup();


	value_s values;

	if( !ccs811.begin() ) Serial.println("setup: CCS811 begin FAILED");
	if( !ccs811.start(CCS811_MODE_1SEC) ) Serial.println("setup: CCS811 start FAILED");
	delay(SDS_WARMUP_TIME * 1000);
	
	read_dht22(dht, &(values.temp), &(values.rh));

	read_ccs811(ccs811, &values, CCS811_READ_COUNT);


	read_sds011(sds, &(values.pm25), &(values.pm10), SDS_READ_COUNT);
	read_ds_co2(DS_CO2_PIN, &(values.co2), 1);

	// connect to mqtt server
	connectMqtt();



	getConnDetails(macAddr, wifiSSID);
	publish_mqtt(&values, macAddr, wifiSSID);

#if SLEEP_DURATION
	// disconnect wifi and mqtt
	disconnectWifi();
	disconnectMqtt();
	sds.sleep();

	// delete emergency hibernate task
	vTaskDelete(hibernateTaskHandle);

	// go to sleep now
	hibernate();
#endif
	init_wtdg();
}

void loop() {
#if SLEEP_DURATION
  /// we're not doing anything in the loop, only on device wakeup
  delay(10000);

#else
  	timerWrite(timer, 0);
  	if(WiFi.status() != WL_CONNECTED) {
		WiFi.reconnect();
	}
	while(!client.connected()) {
		if (!client.connect(MQTT_CLIENTID, MQTT_USERNAME, MQTT_PASSWORD)) {
			Serial.print("MQTT connection failed:");
			Serial.print(client.state());
			Serial.println("Retrying...");
			delay(MQTT_RETRY_WAIT);
		}
	}
  

	value_s values;
	
	read_dht22(dht, &(values.temp), &(values.rh));
	read_ccs811(ccs811, &values, 1);
	read_sds011(sds, &(values.pm25), &(values.pm10), 1);
	read_ds_co2(DS_CO2_PIN, &(values.co2), 1);

	publish_mqtt(&values, macAddr, wifiSSID); //TODO Assumes WiFi doesn't change without restarting
  	delay(3000);
#endif

}


void activate_sds() {
	//TODO Implement me!
	return;
}

uint8_t read_dht22(DHT unit, float* temp, float* rh) {
	*temp = unit.readTemperature();
	*rh = unit.readHumidity();
	if ( isnan(*temp) || isnan(*rh)) {
		Serial.println("[ERROR] Please check the DHT sensor !");
		return 1;
	}

	return 0;
}

void read_ccs811(CCS811 sensor, value_s* data_s, uint8_t count) {
	// Pass environmental data from DHT22 to CCS811
	uint16_t t_data, h_data;
	h_data = 2*data_s->rh;
	double integer_t = (double)data_s->temp;
	float fractional_t = modf(integer_t, &integer_t);
	uint16_t temp_high = (((uint16_t)integer_t+ 25) << 9);
	uint16_t temp_low = ((uint16_t)(fractional_t / 0.001953125) & 0x1FF);

	t_data = (temp_high | temp_low);

	/*
	if(!ccs811.set_envdata(t_data, h_data)) {
		Serial.println("CCS811: set_envdata returned error");
	}
	*/

	uint8_t samples = 0;
	data_s->eco2 = 0;
	data_s->etvoc = 0;

	uint16_t eco2, etvoc;

	//FIXME Add a timeout
	
	while(samples < count) {
		ccs811.read(&eco2, &etvoc, &(data_s->errstat), &(data_s->raw));

		// Print measurement results based on status
		if( data_s->errstat==CCS811_ERRSTAT_OK ) { 
			data_s->eco2 += eco2;
			data_s->etvoc += etvoc;
			samples++;
		} else if( data_s->errstat==CCS811_ERRSTAT_OK_NODATA ) {
			Serial.println("CCS811: waiting for (new) data");
		} else if( data_s->errstat & CCS811_ERRSTAT_I2CFAIL ) { 
			Serial.println("CCS811: I2C error");
		} else {
			Serial.print("CCS811: errstat="); Serial.print(data_s->errstat,HEX); 
			Serial.print("="); Serial.println( ccs811.errstat_str(data_s->errstat) ); 
		}
		delay(1000);
	}
	
	if( count ) {
		data_s->eco2 = data_s->eco2 / count;
		data_s->etvoc = data_s->etvoc / count;
	}
	return;
}

uint8_t publish_mqtt(value_s* data_s, char* mac, char* wifi_ssid) {
	String baseTopic = MQTT_BASE_TOPIC + "/" + mac + "/";

	Serial.print("PM2.5 ");
	Serial.print(data_s->pm25);
	Serial.print("\tPM10 ");
	Serial.print(data_s->pm10);
	Serial.print("\tTemp ");
	Serial.print(data_s->temp);
	Serial.print("\tHum ");
	Serial.print(data_s->rh);
	Serial.print("\teCO2 ");
	Serial.print(data_s->eco2);
	Serial.print("\tCO2 ");
	Serial.print(data_s->co2);
	Serial.print("\teTVOC ");
	Serial.println(data_s->etvoc);

	char buffer[128];
	snprintf(buffer, 128, "{\"data\":{\"value\":%f, \"unit\": \"ug/m^3\"}, \"meta\":{\"wifi\":\"%s\"}}", data_s->pm25, wifi_ssid);
	client.publish((baseTopic + "pm2.5").c_str(), buffer); 

	snprintf(buffer, 128, "{\"data\":{\"value\":%f, \"unit\": \"ug/m^3\"}, \"meta\":{\"wifi\":\"%s\"}}", data_s->pm10, wifi_ssid);
	client.publish((baseTopic + "pm10").c_str(), buffer);

	snprintf(buffer, 128, "{\"data\":{\"value\":%f, \"unit\": \"C\"}, \"meta\":{\"wifi\":\"%s\"}}", data_s->temp, wifi_ssid);
	client.publish((baseTopic + "temperature").c_str(), buffer);
	
	snprintf(buffer, 128, "{\"data\":{\"value\":%f, \"unit\": \"\%\"}, \"meta\":{\"wifi\":\"%s\"}}", data_s->rh, wifi_ssid);
	client.publish((baseTopic + "humidity").c_str(), buffer);
	
	snprintf(buffer, 128, "{\"data\":{\"value\":%d, \"unit\": \"ppm\"}, \"meta\":{\"wifi\":\"%s\"}}", data_s->eco2, wifi_ssid);
	client.publish((baseTopic + "eco2").c_str(), buffer);
	
	snprintf(buffer, 128, "{\"data\":{\"value\":%d, \"unit\": \"ppm\"}, \"meta\":{\"wifi\":\"%s\"}}", data_s->co2, wifi_ssid);
	client.publish((baseTopic + "co2").c_str(), buffer);

	snprintf(buffer, 128, "{\"data\":{\"value\":%d, \"unit\": \"ppb\"}, \"meta\":{\"wifi\":\"%s\"}}", data_s->etvoc, wifi_ssid);
	client.publish((baseTopic + "etvoc").c_str(), buffer);

	delay(1000); // Needed to allow the WiFi peripheral to flush the data. Otherwise might go to sleep before TX is complete

	return 0;
}

uint8_t read_sds011(SDS011 sensor, float *pm25, float *pm10, uint8_t count) {
	// FIXME Add a timeout
	float temp25, temp10;
	*pm25 = 0;
	*pm10 = 0;
	uint8_t samples = 0;
	for(int i = 0; i< count; i++) {
		uint8_t err = sds.read(&temp25, &temp10);
		if(!err) {
			*pm25 += temp25;
			*pm10 += temp10;
			samples++;
		}
		delay(SDS_SAMPLE_TIME);
	}
	if (samples > 0) {
		// Save average values
		*pm25 = *pm25/samples;
		*pm10 = *pm10/samples;
		/*//DEBUG
			Serial.print("SDS011 readings: pm2.5:\t");
			Serial.print(*pm25);
			Serial.print("\tpm10:\t");
			Serial.println(*pm10);
		//DEBUG
		*/
		return 0;
	}
	else
		return 1;

}

uint8_t read_ds_co2(uint8_t pin, uint32_t *co2, uint8_t count) {
	// FIXME Add a timeout
	uint32_t temp_co2;
	*co2 = 0;
	uint8_t samples = 0;
	for(int i = 0; i< count; i++) {
		uint32_t duration = pulseIn(pin, HIGH, 1200000);
		if(duration) {
			*co2 += duration/200;
			samples++;
		}
	}
	if (samples > 0) {
		// Save average values
		*co2 = *co2/samples;
		return 0;
	}
	else
		return 1;

}

uint8_t get_rx_checksum(uint8_t *msg) {
	// Only the data bits are used
	uint8_t ret = 0;
	for(int i=2; i<8; i++)
		ret += msg[i];
	return ret;
}
