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

#define LEDPIN 34
#define FANPIN 17

// The remote service we wish to connect to.
static BLEUUID deviceUUID("00001818-0000-1000-8000-00805f9b34fb");
static BLEUUID cscService("00001816-0000-1000-8000-00805f9b34fb");
// The characteristic of the remote service we are interested in.
static BLEUUID charUUID("00002a5b-0000-1000-8000-00805f9b34fb");

static BLEAddress *pServerAddress;
static boolean doConnect = false;
static boolean connected = false;
static BLERemoteCharacteristic* pRemoteCharacteristic;

bool ledstate = false;
long ledperiod = 1000;
long prev_led = 0;
static long start_time = 0;

double t0, t1;
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
	int offset(sizeof(flags));

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

			// rollover
			if (dt < 0)
				dt = t1 + (((double)0xFFFF/1024.0) - t0);

			double drev = revs1 - revs0;
			double speed = drev * wheel_size * 0.001 / dt * 3.6;

			ESP_LOGI(TAG, "Speed: %lf", speed);

			ledperiod = speed > 0.1 ? (int) (1000. / speed) : 1000;
		}
	}

}

bool connectToServer(BLEAddress pAddress) {
	ESP_LOGI(TAG, "Forming a connection to %s", pAddress.toString().c_str());

	BLEClient* pClient = BLEDevice::createClient();
	ESP_LOGI(TAG, " - Created client");

	// Connect to the remove BLE Server.
	if (!pClient->connect(pAddress)) {
	  ESP_LOGW(TAG, "Failed to connect to server %s",
			  pAddress.toString().c_str());
	  return false;
	}

	ESP_LOGI(TAG, " - STEP 2 - Connected to server");

	// Obtain a reference to the service we are after in the remote BLE server.
	BLERemoteService* pRemoteService = pClient->getService(cscService);
	if (pRemoteService == nullptr) {
	  ESP_LOGW(TAG, "Failed to find our service UUID: %s",
			  cscService.toString().c_str());
	  return false;
	}
	ESP_LOGI(TAG, " - STEP 3 - Found our service");


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
		return;
		if (advertisedDevice.haveServiceUUID() &&
				advertisedDevice.getServiceUUID().equals(deviceUUID)) {
			ESP_LOGI(TAG, "STEP 1 - Yessss! Found our device!  address: %s",
					advertisedDevice.getAddress().toString().c_str());
			advertisedDevice.getScan()->stop();

			pServerAddress = new BLEAddress(advertisedDevice.getAddress());
			doConnect = true;
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
		ESP_LOGI(TAG, "LED=%d", ledstate);
	}

}

void setLED(bool state)
{
	digitalWrite(LEDPIN, state);
	ledperiod = 0;
}

void initBLE()
{
	BLEDevice::init("");
	BLEScan* pBLEScan = BLEDevice::getScan();
	pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
	pBLEScan->setActiveScan(true);
	pBLEScan->start(10);

	ESP_LOGI(TAG, "Leaving initBLE.");
}


void checkConnection()
{
	if (doConnect) {
		setLED(false);
		if (connectToServer(*pServerAddress)) {
			ESP_LOGI(TAG, "We are now connected to the BLE Server.");
			connected = true;
		} else {
			ESP_LOGI(TAG, "We have failed to connect to the server.");
		}
		doConnect = false;
	}
}

void setup(void) {
	Serial.begin(115200);
	start_time = millis();

	ESP_LOGI(TAG,"\n##########################################################\n"\
			"Starting version " PROJECT_VER " (Build " BUILDNR ") ...\n"\
			"##########################################################\n");

	initGpios();

	ESP_LOGI(TAG, "Starting Arduino BLE Client application...");

	initBLE();
}


void loop()
{
	updateStatusLed();

	checkConnection();
}

