/**
 * @file main.cpp  Bluetooth LE controlled air fan
 *
 * Connects to cadience and speed profile. The speed of the fan is coupled
 * to the speed of the smart bike trainer.
 *
 * Based on Smart Fan (Bike Trainer)
 * https://github.com/jmlopezdona/smartfan-esp32
 *
 * Copyright (C) 2019 Christian Spielberger
 */

#ifndef PROJECT_VER
#define PROJECT_VER "???"
#endif

#ifndef BUILDNR
#define BUILDNR "???"
#endif

#define TAG "blefan"


#include <Arduino.h>
#include <BLEAddress.h>
#include <BLEDevice.h>
#include <BLEUUID.h>
#include <BLERemoteCharacteristic.h>
#include <WiFiMulti.h>
#include <WiFiClient.h>
#include <HTTPUpdate.h>
#ifdef USE_PWM
#include "driver/mcpwm.h"
#endif

#define LEDPIN 32
#define FANPIN 17
#define ZCPIN   4

#define STACK_SIZE 1024

// The remote service we wish to connect to.
static BLEUUID deviceUUID("00001818-0000-1000-8000-00805f9b34fb");
static BLEUUID cscService("00001816-0000-1000-8000-00805f9b34fb");
// The characteristic of the remote service we are interested in.
static BLEUUID charUUID("00002a5b-0000-1000-8000-00805f9b34fb");

#define CONFIG_ESP_WIFI_SSID "OpenWrt-chris"
#define CONFIG_ESP_WIFI_PASSWORD "zippezappe538"
#define CONFIG_ESP_UPDATE_URL "http://cspiel.at/share/blefan.bin"

BLEAdvertisedDevice gAdvertisedDevice;
static boolean doConnect = false;
static boolean connected = false;
static BLEClient* pClient = 0;
static BLERemoteService* pRemoteService = 0;
static BLERemoteCharacteristic* pRemoteCharacteristic = 0;
static TaskHandle_t led_thread;
//static TaskHandle_t pwm_thread;

bool ledstate = false;
long ledperiod = 1000;
bool manual = false;

#ifdef USE_PWM
	float pwm_duty_prev = 0.;
#endif

static volatile unsigned char pwm_duty = 0;
static volatile unsigned char zero_cnt = 0;
bool pwm_set = false;

long prev_led = 0;
static long start_time = 0;

double t0 = 0, t1 = 0;
uint32_t revs0 = 0, revs1 = 0;

int wheel_size = 2000; // in mm

String command;

WiFiMulti WiFiMulti;

static void ble_handler(
	BLERemoteCharacteristic* pBLERemoteCharacteristic,
	uint8_t* pData,
	size_t length,
	bool isNotify) {

	if (manual)
		return;

//    ESP_LOGI(TAG, "Notify length %u",length);

	uint8_t flags;
	memcpy(&flags, pData, sizeof(flags));
	int offset = (int) sizeof(flags);

	// if speed present can be combo sensor or speed sensor only
	if (flags & 1 ) {
		uint32_t revs;
		memcpy(&revs, pData + offset, sizeof(revs));
		offset += sizeof(revs);

		uint16_t t;
		memcpy(&t, pData + offset, sizeof(t));
		offset += sizeof(t);

		t0 = t1;
		t1 = (double) t / 1024.0;

		revs0 = revs1;
		revs1 = revs;

		if (t0 && t1) {
			double dt = t1 - t0;
			double speed;

			if (dt == 0) {
				speed = 0;
			} else {
				// rollover
				if (dt < 0)
					dt = t1 + (((double)0xFFFF/1024.0) - t0);

				double drev = revs1 - revs0;
				speed = drev * wheel_size * 0.001 / dt * 3.6;
			}

			ESP_LOGI(TAG, "Speed: %lf", speed);
//            ESP_LOGI(TAG, "t=%u t0=%lf t1=%lf revs=%u revs0=%u revs1=%u",
//                    t, t0, t1, revs, revs0, revs1);

			ledperiod = speed > 1. ? (int) (1000. / speed) : 1000;
			pwm_duty = speed < 4. ? 0 :
				(speed < 40. ? (unsigned char) (speed * 2.5) : 100);

			ESP_LOGI(TAG, "pwm_duty=%u", pwm_duty);
		}
	}

}

bool connectToServer() {
	ESP_LOGI(TAG, "Forming a connection to %s",
			gAdvertisedDevice.getAddress().toString().c_str());

	pClient = BLEDevice::createClient();
	ESP_LOGI(TAG, " - Created client");

	// Connect to the remote BLE Server.
	if (!pClient->connect(&gAdvertisedDevice)) {
	  ESP_LOGW(TAG, "Failed to connect to server %s",
			  gAdvertisedDevice.getAddress().toString().c_str());
	  return false;
	}

	ESP_LOGI(TAG, " - STEP 2 - Connected to server");
	ledperiod = 600;

	// Obtain a reference to the service we are after in the remote BLE server.
	pRemoteService = pClient->getService(cscService);
	if (!pRemoteService) {
	  ESP_LOGW(TAG, "Failed to find our service UUID: %s",
			  cscService.toString().c_str());
	  return false;
	}

	ESP_LOGI(TAG, " - STEP 3 - Found our service");
	ledperiod = 400;

	// Obtain a reference to the characteristic in the service of the remote BLE server.
	pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
	if (pRemoteCharacteristic == nullptr) {
	  ESP_LOGI(TAG, "Failed to find our characteristic UUID: %s",
			  charUUID.toString().c_str());
	  return false;
	}
	ESP_LOGI(TAG, " - STEP 4 - Found our characteristic");

	// Read the value of the characteristic.
	std::string value = pRemoteCharacteristic->readValue();
	ESP_LOGI(TAG, "The characteristic value was: %s", value.c_str());

	pRemoteCharacteristic->registerForNotify(ble_handler);
	ledperiod = 200;

	return true;
}


/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
	/**
	 * Called for each advertising BLE server.
	 */
	void onResult(BLEAdvertisedDevice advertisedDevice) {
		ESP_LOGI(TAG, "BLE Advertised Device found: %s",
				advertisedDevice.toString().c_str());

		// We have found a device, let us now see if it contains the service we are looking for.
		if (advertisedDevice.haveServiceUUID() &&
				advertisedDevice.getServiceUUID().equals(deviceUUID)) {
			ESP_LOGI(TAG, "STEP 1 - Yessss! Found our device!  address: %s "
					"address type: %d",
					advertisedDevice.getAddress().toString().c_str(),
					advertisedDevice.getAddressType());
			advertisedDevice.getScan()->stop();

			gAdvertisedDevice = advertisedDevice;
			doConnect = true;
			digitalWrite(FANPIN, 0);
		}
	}
};

// Every zero crossing : (50Hz)-> 10ms (1/2 Cycle)
// 10ms = 1000 * 10 us
void IRAM_ATTR zero_cross_int()
{
//    if (pwm_duty > 0) {
//        digitalWrite(FANPIN, HIGH);
//        if (pwm_duty < 90) {
//            unsigned int ontime = 100 * pwm_duty;    // 1000 * 10 / 100 * pwm_duty
//            delayMicroseconds(ontime);    // Off cycle
//            digitalWrite(FANPIN, LOW);
//        }
//    }
	if (pwm_duty > 0) {
		unsigned int offtime = 100 * (50 - pwm_duty/2);
		delayMicroseconds(offtime);    // Off cycle
		digitalWrite(FANPIN, HIGH);
		if (pwm_duty < 90) {
			delayMicroseconds(20);
			digitalWrite(FANPIN, LOW);
		}
	}
}

struct qrow {
	unsigned char h;
	unsigned char l;
};

static struct qrow qtable[] = {
	{0, 20}, // 0%
	{2, 18}, // 10%
	{2, 8},  // 20%
	{6, 14}, // 30%
	{4, 6},  // 40%
	{4, 4},  // 50%
	{6, 4},  // 60%
	{14, 6}, // 70%
	{8, 2},  // 80%
	{18, 2}, // 90%
	{20, 0}, // 100%
};

void IRAM_ATTR zero_cross_int2()
{
	unsigned char i = (pwm_duty + 5) / 10;
	if (i > 10)
		i = 10;

	if (i == 0) {
		if (zero_cnt != 255) {
			digitalWrite(FANPIN, LOW);
			zero_cnt = 255;
		}
	} else if (i < 10) {
		if (zero_cnt == 255)
			zero_cnt = 0;

		if (zero_cnt == 0) {
			digitalWrite(FANPIN, HIGH);
			pwm_set = true;
		} else if (zero_cnt == qtable[i].h) {
			digitalWrite(FANPIN, LOW);
			pwm_set = false;
			zero_cnt = qtable[i].l;
		}

		if (pwm_set)
			zero_cnt++;
		else
			zero_cnt--;
	} else if (i >= 10) {
		if (zero_cnt != 255) {
			digitalWrite(FANPIN, HIGH);
			zero_cnt = 255;
		}
	}
}


void initGpios() {
	pinMode(LEDPIN, OUTPUT);
	pinMode(FANPIN, OUTPUT);

#ifdef USE_PWM
	mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, FANPIN);
	mcpwm_config_t pwm_config;
	pwm_config.frequency = 10000;
	pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
	pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
	pwm_config.counter_mode = MCPWM_UP_COUNTER;
	pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
	mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
#else
	pinMode(ZCPIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(ZCPIN), zero_cross_int, RISING);
#endif

	digitalWrite(LEDPIN, HIGH);
	digitalWrite(FANPIN, HIGH);
}


void toggleLED()
{
	ledstate = !ledstate;
	digitalWrite(LEDPIN, ledstate);
}


void updateStatusLed()
{
	if (!ledperiod) {
	} else if ((millis() - prev_led) > ledperiod) {
		prev_led = millis();
		toggleLED();
	}

}


void updatePWM()
{
#ifdef USE_PWM
	if (pwm_duty != pwm_duty_prev) {
		pwm_duty_prev = pwm_duty;
		mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, pwm_duty);
		mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
	}
#endif
}


void setLED(bool state)
{
	digitalWrite(LEDPIN, state);
	ledperiod = 0;
}


static void scan_complete_h (BLEScanResults result)
{
	ESP_LOGI(TAG, "Scan complete.");
	digitalWrite(FANPIN, LOW);
	pwm_set = false;
}


void initBLE()
{
	BLEDevice::init("");
	BLEScan* pBLEScan = BLEDevice::getScan();
	pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
	pBLEScan->setActiveScan(true);
	pBLEScan->start(10, scan_complete_h);

	ESP_LOGI(TAG, "Leaving initBLE.");
}


void checkConnection()
{
	if (doConnect) {
		setLED(false);
		if (connectToServer()) {
			ESP_LOGI(TAG, "We are now connected to the BLE Server.");
			connected = true;
			digitalWrite(FANPIN, LOW);
			pwm_set = false;
		} else {
			ESP_LOGI(TAG, "We have failed to connect to the server.");
		}
		initGpios();
		doConnect = false;
	}
}


void run_led(void* arg)
{
	while (1) {
		updateStatusLed();

		vTaskDelay(10 / portTICK_PERIOD_MS);
	}
}


void setup(void) {
	Serial.begin(115200);
	start_time = millis();

	ESP_LOGI(TAG,
			"\n"
			"########################################################\n"
			"Starting version " PROJECT_VER " (Build " BUILDNR ") ...\n"
			"########################################################\n");

	ESP_LOGI(TAG, "Starting Arduino BLE Client application...");

	xTaskCreate(run_led, "LED", STACK_SIZE, NULL, 10, &led_thread);
	initBLE();
	enableLoopWDT();
}


static bool _update = false;

void do_command()
{
	if (command == "update") {
		delete pRemoteCharacteristic;
		pRemoteCharacteristic = 0;

		WiFi.mode(WIFI_STA);
		WiFiMulti.addAP(CONFIG_ESP_WIFI_SSID, CONFIG_ESP_WIFI_PASSWORD);
		WiFiMulti.run(100);
		_update = true;
	} else if (command == "reboot") {
		esp_restart();
	} else {
		Serial.printf("Menu:\n");
		Serial.printf("  update  ... start OTA update\n");
		Serial.printf("  reboot  ... reboot esp32\n");
		Serial.printf("\n");
		Serial.printf("command: |%s|", command.c_str());
	}

	command.clear();
}

void checkUpdate()
{
	if (_update) {
		if (WiFi.status() == WL_CONNECTED) {
			_update = false;
			WiFiClient client;
			ESP_LOGI(TAG, "Http ota update started...");
			HTTPUpdateResult ret = httpUpdate.update(client, CONFIG_ESP_UPDATE_URL);
			ESP_LOGI(TAG, "Http ota update done.");
			switch (ret) {
				case HTTP_UPDATE_FAILED:
					Serial.printf("HTTP_UPDATE_FAILED Error (%d): %s\n", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
					break;

				case HTTP_UPDATE_NO_UPDATES:
					Serial.println("HTTP_UPDATE_NO_UPDATES");
					break;

				case HTTP_UPDATE_OK:
					Serial.println("HTTP_UPDATE_OK");
					break;
			}
		}
	}
}

void loop()
{
	char c;

	checkConnection();
	checkUpdate();

	vTaskDelay(10 / portTICK_PERIOD_MS);

	c = Serial.read();
	if (c > 0x1f && c < 0x7f)
		Serial.printf("%c", (char) c);

	switch (c) {
		case '+':
			manual = true;
			if (pwm_duty < 100)
				pwm_duty += 1;
			ESP_LOGI(TAG, "Inc pwm_duty: %u", pwm_duty);
			break;
		case '-':
			manual = true;
			if (pwm_duty > 0)
				pwm_duty -= 1;
			ESP_LOGI(TAG, "Dec pwm_duty: %u", pwm_duty);
			break;
		default:
			if (c >= 'a' && c <= 'z')
				command += c;
			else if (c == 10 || c == 13) {
				do_command();
			}
			break;
	}

	static unsigned char zero_cnt_prev = 0;
	if (zero_cnt_prev != zero_cnt) {
		Serial.printf("c=%u s=%u\n", zero_cnt, pwm_set);
		zero_cnt_prev = zero_cnt;
	}

}

