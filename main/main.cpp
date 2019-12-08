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

#define LEDPIN 32
#define FANPIN 17

#define STACK_SIZE 4*1024

// The remote service we wish to connect to.
static BLEUUID deviceUUID("00001818-0000-1000-8000-00805f9b34fb");
static BLEUUID cscService("00001816-0000-1000-8000-00805f9b34fb");
// The characteristic of the remote service we are interested in.
static BLEUUID charUUID("00002a5b-0000-1000-8000-00805f9b34fb");

BLEAdvertisedDevice gAdvertisedDevice;
static boolean doConnect = false;
static boolean connected = false;
static BLERemoteCharacteristic* pRemoteCharacteristic;
static TaskHandle_t led_thread;

bool ledstate = false;
long ledperiod = 1000;
long prev_led = 0;
static long start_time = 0;

double t0 = 0, t1 = 0;
uint32_t revs0 = 0, revs1 = 0;

int wheel_size = 2000; // in mm

static void ble_handler(
	BLERemoteCharacteristic* pBLERemoteCharacteristic,
	uint8_t* pData,
	size_t length,
	bool isNotify) {

	ESP_LOGI(TAG, "Notify length %u",length);

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
			ESP_LOGI(TAG, "t=%u t0=%lf t1=%lf revs=%u revs0=%u revs1=%u",
					t, t0, t1, revs, revs0, revs1);

			ledperiod = speed > 0.1 ? (int) (1000. / speed) : 1000;
		}
	}

}

bool connectToServer() {
	ESP_LOGI(TAG, "Forming a connection to %s",
			gAdvertisedDevice.getAddress().toString().c_str());

	BLEClient* pClient = BLEDevice::createClient();
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
	BLERemoteService* pRemoteService = pClient->getService(cscService);
	if (pRemoteService == nullptr) {
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


void initGpios() {
	pinMode(LEDPIN, OUTPUT);
	pinMode(FANPIN, OUTPUT);

	digitalWrite(LEDPIN, 1);
	digitalWrite(FANPIN, 1);
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

void setLED(bool state)
{
	digitalWrite(LEDPIN, state);
	ledperiod = 0;
}


static void scan_complete_h (BLEScanResults result)
{
	ESP_LOGI(TAG, "Scan complete.");
	digitalWrite(FANPIN, 0);
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
			digitalWrite(FANPIN, 0);
		} else {
			ESP_LOGI(TAG, "We have failed to connect to the server.");
		}
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

	initGpios();

	ESP_LOGI(TAG, "Starting Arduino BLE Client application...");

	xTaskCreate(run_led, "LED", STACK_SIZE, NULL, 10, &led_thread);
	initBLE();
	enableLoopWDT();
}

void loop()
{
	checkConnection();
	vTaskDelay(10 / portTICK_PERIOD_MS);
}

