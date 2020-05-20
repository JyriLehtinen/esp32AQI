(Based on https://github.com/sidddy/flora)


## Stack requirements
Hardware:
 - ESP32
 - SDS011
 - CCS811
 - DHT22

Software:
- MQTT server

Arduino Libraries (not complete)
 - https://github.com/Hieromon/PageBuilder
 - https://github.com/Hieromon/AutoConnect
 - https://github.com/adafruit/DHT-sensor-library
 - https://github.com/ricki-z/SDS011/
 - https://github.com/maarten-pennings/CCS811

## Setup instructions

1) Copy config.h.example into config.h and update seetings according to your environment:
- MQTT Server address and credentials

2) Update the CCS811 firmware according to the library instructions

3) Open ino sketch in Arduino, compile & upload. 

4) If not connected to Wifi, ESP32 launches as AP *SSID "ESP32 AQI Sensor"**
Connect to the Wifi and input **password "ilmavaiva"**
From here you can search for available networks and input credentials. You only need to do this once, ESP32 will remember the network(s).

## Configuration

- SLEEP_DURATION - how long should the device sleep between wakeups?  0 to disable sleep (Recommended as SDS and CCS require warmup at boot).
- EMERGENCY_HIBERNATE - how long after wakeup should the device forcefully go to sleep (e.g. when something gets stuck)?

- SDS_SAMPLE_TIME 	- How often the SDS is read?
- SDS_WARMUP_TIME	- How long the warmup should be?
- SDS_READ_COUNT  	- How many reads to do in a single call?
- CCS811_READ_COUNT - How many reads to do in a single call?

## Sketch size issues

The sketch does not fit into the default arduino parition size of around 1.3MB. You'll need to change your default parition table and increase maximum build size to at least 1.6MB.
On Arduino IDE this can be achieved using "Tools -> Partition Scheme -> No OTA"
