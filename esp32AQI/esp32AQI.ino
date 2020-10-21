#include "config.h"
#include <PubSubClient.h>
#include <AutoConnect.h>
#include <WiFi.h>
#include <string.h>
#include <Wire.h>
#include "esp_system.h"

// TODO: Replace commenting out with #ifdef macros for convenient sensor selection
//#include "DHT.h"
//#include "SDS011.h"

#include "ccs811.h"
#include "mh_z14a.h"
#include "Adafruit_BMP280.h"
#include "ClosedCube_HDC1080.h"
#include "APDS9930.h"


//#define DHT_PIN 19
#define CCS_NWAKE_PIN 23

#define TEMP_OFFSET 3.5

//#define DS_CO2_PIN 5

typedef struct value_s {
	float pm25;
	float pm10;
	float temp;
	float rh;
	float pressure;
	uint32_t eco2;
	uint32_t etvoc;
	uint16_t errstat;
	uint16_t raw;
	uint32_t co2;
	float light;
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
//uint8_t read_dht22(DHT unit, float* temp, float* rh);

void activate_sensors(void);
void read_sensors(value_s* data_s);
void deactivate_sensors();

void read_ccs811(CCS811 sensor, value_s* data_s, uint8_t read_count);
//uint8_t read_sds011(SDS011 sensor, float *pm25, float *pm10, uint8_t count);
//uint8_t read_ds_co2(uint8_t pin, uint32_t *co2, uint8_t count);

uint8_t publish_mqtt(value_s* data_s, char* mac, char* wifi_ssid);
void init_wtdg(void);

/********************************* PRIVATE VARIABLES ********************************/
AutoConnect portal;

WiFiClient espClient;
PubSubClient client(espClient);
HardwareSerial port(2);

/************ SENSORS ***********/
//DHT dht(DHT_PIN, DHT22);
//SDS011 sds;
CCS811 ccs811(CCS_NWAKE_PIN);
MH_Z14A mh_z14a;
Adafruit_BMP280 bmp280;
ClosedCube_HDC1080 HDC1080;
APDS9930 apds = APDS9930();



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
	timerAlarmEnable(timer);
}

void connectWifi() {
  Serial.println(F("Starting AutoConnect Wifi portal.."));
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

  Serial.println(F("\nWiFi connected"));
}

void disconnectWifi() {
  WiFi.disconnect(true);
  Serial.println(F("WiFi disonnected"));
}

void connectMqtt() {
  Serial.println(F("Connecting to MQTT..."));
  client.setServer(MQTT_HOST, MQTT_PORT);

  while (!client.connected()) {
    if (!client.connect(MQTT_CLIENTID, MQTT_USERNAME, MQTT_PASSWORD)) {
      Serial.print(F("MQTT connection failed:"));
      Serial.print(client.state());
      Serial.println(F("Retrying..."));
      delay(MQTT_RETRY_WAIT);
    }
  }

  Serial.println(F("MQTT connected"));
}

void disconnectMqtt() {
  client.disconnect();
  Serial.println(F("MQTT disconnected"));
}

uint8_t getConnDetails(char* mac, char* wifiSSID)
{
	uint8_t macAddr[6];
	WiFi.macAddress(macAddr);
	sprintf(mac, "%2X:%2X:%2X:%2X:%2X:%2X",
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
  Serial.println(F("Going to sleep now."));
  delay(100);
  esp_deep_sleep_start();
}

void delayedHibernate(void *parameter) {
  delay(EMERGENCY_HIBERNATE*1000); // delay for five minutes
  Serial.println(F("Something got stuck, entering emergency hibernate..."));
  hibernate();
}

void setup() {
	// all action is done when device is woken up
	Serial.begin(115200);


	// increase boot count
	bootCount++;

	// create a hibernate task in case something gets stuck
#if SLEEP_DURATION
	xTaskCreate(delayedHibernate, "hibernate", 4096, NULL, 1, &hibernateTaskHandle);
#endif
	
	activate_sensors();
	delay(1000);

	// connect to Wifi or start as AP
	connectWifi();

	delay(SDS_WARMUP_TIME * 1000);

	value_s values;
	read_sensors(&values);

	// connect to mqtt server
	connectMqtt();

	getConnDetails(macAddr, wifiSSID);
	publish_mqtt(&values, macAddr, wifiSSID);

#if SLEEP_DURATION
	// disconnect wifi and mqtt
	disconnectWifi();
	disconnectMqtt();
	deactivate_sensors();

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
  	while(WiFi.status() != WL_CONNECTED) {
		WiFi.reconnect();
		delay(10000);
	}
	while(!client.connected()) {
		if (!client.connect(MQTT_CLIENTID, MQTT_USERNAME, MQTT_PASSWORD)) {
			Serial.print(F("MQTT connection failed:"));
			Serial.print(client.state());
			Serial.println(F("Retrying..."));
			delay(MQTT_RETRY_WAIT);
		}
	}
  

	value_s values;
	
	read_sensors(&values);

	publish_mqtt(&values, macAddr, wifiSSID); //TODO Assumes WiFi doesn't change without restarting
  	delay(5000);
#endif

}

void activate_sensors(void) {
// UNUSED IN THIS BUILD
	//sds.begin(&port);
	//sds.wakeup();
	//dht.begin();
	//pinMode(DS_CO2_PIN, INPUT);

// CCS811
	Wire.begin();
	ccs811.set_i2cdelay(50);

	if( !ccs811.begin() ) Serial.println(F("setup: CCS811 begin FAILED"));
	if( !ccs811.start(CCS811_MODE_1SEC) ) Serial.println(F("setup: CCS811 start FAILED"));

// BMP280
	if (!bmp280.begin(0x76)) {
		Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
		while (true);
	}

	bmp280.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
					Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
					Adafruit_BMP280::SAMPLING_X8,    /* Pressure oversampling */
					Adafruit_BMP280::FILTER_X16,      /* Filtering. */
					Adafruit_BMP280::STANDBY_MS_2000); /* Standby time. */

// HDC1080
	HDC1080.begin(0x40);
	HDC1080_Registers reg = HDC1080.readRegister();
	reg.Heater = 0;
	HDC1080.writeRegister(reg);

// MH-Z14A
	mh_z14a.begin(&port);


// APDS9930
	if ( !apds.init() ) {
		Serial.println(F("Something went wrong during APDS-9930 init!"));
	}

	// Set ambient light gain to x64, for best accuraccy in the dark
	if ( !apds.setAmbientLightGain(3) ) {
		Serial.println(F("Something went wrong during AGAIN setting!"));
	}


  // Start running the APDS-9930 light sensor (no interrupts)
	if ( !apds.enableLightSensor(false) ) {
		Serial.println(F("Something went wrong during light sensor init!"));
	}
}

void read_sensors(value_s* data_s) {

	//read_dht22(dht, &(values.temp), &(values.rh));
	//read_sds011(sds, &(values.pm25), &(values.pm10), 1);
	//read_ds_co2(DS_CO2_PIN, &(values.co2), 1);
	
	float temp_temp = HDC1080.readTemperature();

	// To prevent false readings at boot
	while(HDC1080.readTemperature() > 100)
		delay(300);

	data_s->temp = HDC1080.readTemperature() - TEMP_OFFSET;
	data_s->rh = HDC1080.readHumidity();

	ccs811.set_envdata(data_s->temp, data_s->rh);
	data_s->pressure = bmp280.readPressure();
	mh_z14a.read( &(data_s->co2) );
	read_ccs811(ccs811, data_s, 1);
	if (  !apds.readAmbientLightLux(data_s->light) )
		Serial.println(F("Error reading light values"));

}

uint8_t publish_mqtt(value_s* data_s, char* mac, char* wifi_ssid) {
	String baseTopic = MQTT_BASE_TOPIC + "/" + mac + "/";
/*
	Serial.print("PM2.5 ");
	Serial.print(data_s->pm25);
	Serial.print("\tPM10 ");
	Serial.print(data_s->pm10);
*/
	Serial.print(F("\tTemp "));
	Serial.print(data_s->temp);
	Serial.print(F("\tHum "));
	Serial.print(data_s->rh);
	Serial.print(F("\tPressure "));
	Serial.print(data_s->pressure);
	Serial.print(F("\teCO2 "));
	Serial.print(data_s->eco2);
	Serial.print(F("\tCO2 "));
	Serial.print(data_s->co2);
	Serial.print(F("\teTVOC "));
	Serial.print(data_s->etvoc);
	Serial.print(F("\tLight "));
	Serial.println(data_s->light);

	char buffer[128];
	/*
	snprintf(buffer, 128, "{\"data\":{\"value\":%f, \"unit\": \"ug/m^3\"}, \"meta\":{\"wifi\":\"%s\"}}", data_s->pm25, wifi_ssid);
	client.publish((baseTopic + "pm2.5").c_str(), buffer); 

	snprintf(buffer, 128, "{\"data\":{\"value\":%f, \"unit\": \"ug/m^3\"}, \"meta\":{\"wifi\":\"%s\"}}", data_s->pm10, wifi_ssid);
	client.publish((baseTopic + "pm10").c_str(), buffer);
	*/

	snprintf(buffer, 128, "{\"data\":{\"value\":%f, \"unit\": \"C\"}, \"meta\":{\"wifi\":\"%s\"}}", data_s->temp, wifi_ssid);
	client.publish((baseTopic + "temperature").c_str(), buffer);
	
	snprintf(buffer, 128, "{\"data\":{\"value\":%f, \"unit\": \"\%\"}, \"meta\":{\"wifi\":\"%s\"}}", data_s->rh, wifi_ssid);
	client.publish((baseTopic + "humidity").c_str(), buffer);

	snprintf(buffer, 128, "{\"data\":{\"value\":%f, \"unit\": \"Pa\"}, \"meta\":{\"wifi\":\"%s\"}}", data_s->pressure, wifi_ssid);
	client.publish((baseTopic + "pressure").c_str(), buffer);
	
	snprintf(buffer, 128, "{\"data\":{\"value\":%d, \"unit\": \"ppm\"}, \"meta\":{\"wifi\":\"%s\"}}", data_s->eco2, wifi_ssid);
	client.publish((baseTopic + "eco2").c_str(), buffer);
	
	snprintf(buffer, 128, "{\"data\":{\"value\":%d, \"unit\": \"ppm\"}, \"meta\":{\"wifi\":\"%s\"}}", data_s->co2, wifi_ssid);
	client.publish((baseTopic + "co2").c_str(), buffer);

	snprintf(buffer, 128, "{\"data\":{\"value\":%d, \"unit\": \"ppb\"}, \"meta\":{\"wifi\":\"%s\"}}", data_s->etvoc, wifi_ssid);
	client.publish((baseTopic + "etvoc").c_str(), buffer);

	snprintf(buffer, 128, "{\"data\":{\"value\":%f, \"unit\": \"lux\"}, \"meta\":{\"wifi\":\"%s\"}}", data_s->light, wifi_ssid);
	client.publish((baseTopic + "light").c_str(), buffer);

	delay(1000); // Needed to allow the WiFi peripheral to flush the data. Otherwise might go to sleep before TX is complete

	return 0;
}

void deactivate_sensors() {
	//TODO Implement me!
	//sds.sleep();
	return;
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

	
	if(!ccs811.set_envdata(data_s->temp, data_s->rh)) {
		Serial.println(F("CCS811: set_envdata returned error"));
	}
	

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
			Serial.println(F("CCS811: waiting for (new) data"));
		} else if( data_s->errstat & CCS811_ERRSTAT_I2CFAIL ) { 
			Serial.println(F("CCS811: I2C error"));
		} else {
			Serial.print(F("CCS811: errstat=")); Serial.print(data_s->errstat,HEX); 
			Serial.print(F("=")); Serial.println( ccs811.errstat_str(data_s->errstat) ); 
		}
		delay(1000);
	}
	
	if( count ) {
		data_s->eco2 = data_s->eco2 / count;
		data_s->etvoc = data_s->etvoc / count;
	}
	return;
}

/*
uint8_t read_dht22(DHT unit, float* temp, float* rh) {
	*temp = unit.readTemperature();
	*rh = unit.readHumidity();
	if ( isnan(*temp) || isnan(*rh)) {
		Serial.println("[ERROR] Please check the DHT sensor !");
		return 1;
	}

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
		//DEBUG
			//Serial.print("SDS011 readings: pm2.5:\t");
			//Serial.print(*pm25);
			//Serial.print("\tpm10:\t");
			//Serial.println(*pm10);
		//DEBUG
		
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

*/

uint8_t get_rx_checksum(uint8_t *msg) {
	// Only the data bits are used
	uint8_t ret = 0;
	for(int i=2; i<8; i++)
		ret += msg[i];
	return ret;
}
