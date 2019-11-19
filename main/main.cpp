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
#define PROJECT_VER
#endif

#ifndef BUILDNR
#define BUILDNR "???"
#endif

#define TAG "blefan"


#include "Arduino.h"
#include "BLEDevice.h"

#define LEDPIN 34
#define FANPIN 17

// The remote service we wish to connect to.
static BLEUUID serviceUUID("00001816-0000-1000-8000-00805f9b34fb");
// The characteristic of the remote service we are interested in.
static BLEUUID charUUID("00002a5b-0000-1000-8000-00805f9b34fb");

static BLEAddress *pServerAddress;
static boolean doConnect = false;
static boolean connected = false;
static BLERemoteCharacteristic* pRemoteCharacteristic;

bool ledstate = false;
long ledperiod = 1000;
long prev_led = -1;

double t0, t1;
uint32_t revs0 = 0, revs1 = 0;

int wheel_size = 2000; // in mm

static void ble_handler(
	BLERemoteCharacteristic* pBLERemoteCharacteristic,
	uint8_t* pData,
	size_t length,
	bool isNotify) {


	Serial.print("Notify callback for characteristic ");
	Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
	Serial.print(" of data length ");
	Serial.print(length);
	Serial.print(" data: ");
	for (int i = 0; i < length; i++) {
		Serial.print(pData[i]);
	}
	ESP_LOGI(TAG, );

	uint8_t flags;
	memcpy(&flags, pData, sizeof(flags));
	int offset(sizeof(flags));

	time_t abs_time = time(NULL);

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
			speed = drev * wheel_size * 0.001 / dt * 3.6;

			Serial.print("Speed: ");
			ESP_LOGI(TAG, speed);

			ledperiod = speed > 0.1 ? (int) (1000. / speed) : 1000;
		}

	}

}

bool connectToServer(BLEAddress pAddress) {
	Serial.print("Forming a connection to ");
	ESP_LOGI(TAG, pAddress.toString().c_str());

	BLEClient* pClient = BLEDevice::createClient();
	ESP_LOGI(TAG, " - Created client");

	// Connect to the remove BLE Server.
	pClient->connect(pAddress);
	ESP_LOGI(TAG, " - Connected to server");

	// Obtain a reference to the service we are after in the remote BLE server.
	BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
	if (pRemoteService == nullptr) {
	  Serial.print("Failed to find our service UUID: ");
	  ESP_LOGI(TAG, serviceUUID.toString().c_str());
	  return false;
	}
	ESP_LOGI(TAG, " - Found our service");


	// Obtain a reference to the characteristic in the service of the remote BLE server.
	pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
	if (pRemoteCharacteristic == nullptr) {
	  ESP_LOGI(TAG, "Failed to find our characteristic UUID: %s",
			  charUUID.toString().c_str());
	  return false;
	}
	ESP_LOGI(TAG, " - Found our characteristic");

	// Read the value of the characteristic.
	std::string value = pRemoteCharacteristic->readValue();
	Serial.print("The characteristic value was: ");
	ESP_LOGI(TAG, value.c_str());

	pRemoteCharacteristic->registerForNotify(ble_handler);
}


/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
	/**
	 * Called for each advertising BLE server.
	 */
	void onResult(BLEAdvertisedDevice advertisedDevice) {
		Serial.print("BLE Advertised Device found: ");
		ESP_LOGI(TAG, advertisedDevice.toString().c_str());

		// We have found a device, let us now see if it contains the service we are looking for.
		if (advertisedDevice.haveServiceUUID() && advertisedDevice.getServiceUUID().equals(serviceUUID)) {
			Serial.print("Found our device!  address: ");
			advertisedDevice.getScan()->stop();

			pServerAddress = new BLEAddress(advertisedDevice.getAddress());
			doConnect = true;
		}
	}
};


void initGpios() {
	pinMode(LEDPIN, OUTPUT);
	pinMode(FANPIN, OUTPUT);

	digitalWrite(LEDPIN, 0);
	digitalWrite(FANPIN, 0);
}

void updateStatusLed()
{
	bool state = ledstate;

	if (!ledperiod) {
	} else if ((millis()-prev_led) > ledperiod) {
		state = !ledstate;
		prev_led = millis();
		digitalWrite(ONBOARDLED_PIN, state);
		ledstate = state;
	}

}

void initBLE()
{
	BLEDevice::init("");
	BLEScan* pBLEScan = BLEDevice::getScan();
	pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
	pBLEScan->setActiveScan(true);
	pBLEScan->start(30);
}

void checkConnection()
{
	if (doConnect == true) {
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

