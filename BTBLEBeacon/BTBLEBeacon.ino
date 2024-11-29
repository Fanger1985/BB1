#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#define BEACON_UUID "12345678-1234-1234-1234-123456789abc" // Unique ID for the beacon
#define BEACON_MAJOR 1
#define BEACON_MINOR 1
#define TX_POWER -59 // Calibrated signal strength at 1 meter

void setup() {
  Serial.begin(115200);

  // Initialize BLE
  BLEDevice::init("FollowMeBeacon");
  BLEServer *pServer = BLEDevice::createServer();

  // Set up iBeacon
  BLEAdvertisementData oAdvertisementData;
  BLEAdvertisementData oScanResponseData;

  oAdvertisementData.setFlags(0x04); // BR_EDR_NOT_SUPPORTED
  oAdvertisementData.setCompleteServices(BLEUUID(BEACON_UUID));

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->setAdvertisementData(oAdvertisementData);
  pAdvertising->setScanResponseData(oScanResponseData);

  // Start advertising
  pAdvertising->start();
  Serial.println("Beacon is now broadcasting.");
}

void loop() {
  // Nothing to do here; the beacon just broadcasts
}
