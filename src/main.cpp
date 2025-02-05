//Sensor Node Code
#if defined(ARDUINO_SEEED_XIAO_NRF52840_SENSE) || defined(ARDUINO_SEEED_XIAO_NRF52840)
#error "XIAO nRF52840 please use the non-mbed-enable version."
#endif

//Server Directives
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

#define bleServerName1 "XIAOESP32C6_BLE_SENDER1"
#define SERVICE_UUID_NODE1        "1de92c84-6aad-4260-9be2-126613bdb0c0"
#define CHARACTERISTIC_UUID_NODE1 "1de92c84-6aad-4260-9be2-126613bdb0c1"

#define bleServerName2 "XIAOESP32C6_BLE_SENDER2"
#define SERVICE_UUID_NODE2        "4f056d6b-d747-42bf-87d3-275649d82520"
#define CHARACTERISTIC_UUID_NODE2 "4f056d6b-d747-42bf-87d3-275649d82521"

//mmWave Directives
#include <SoftwareSerial.h>
#include <mmwave_for_xiao.h>
#include <rolling_average.h>

//BLE Globals
BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
uint8_t movingPowerA[8], staticPowerA[8];

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };
  
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

// Define the SoftwareSerial object, D2 as RX, D3 as TX, connect to the serial port of the mmwave sensor
SoftwareSerial COMSerial(D2, D3);

// Creates a global Serial object for printing debugging information
#define ShowSerial Serial

// Initialising the radar configuration
// Seeed_HSP24 xiao_config(COMSerial, ShowSerial);
Seeed_HSP24 xiao_config(COMSerial);

Seeed_HSP24::RadarStatus radarStatus;
Seeed_HSP24::AskStatus askStatus;

RollingAVG staticPower;
RollingAVG movingPower;

// Parsing the acquired radar status
static const char* targetStatusToString(Seeed_HSP24::TargetStatus status) {
  switch (status) {
    case Seeed_HSP24::TargetStatus::NoTarget:
      return "NoTarget";
    case Seeed_HSP24::TargetStatus::MovingTarget:
      return "MovingTarget";
    case Seeed_HSP24::TargetStatus::StaticTarget:
      return "StaticTarget";
    case Seeed_HSP24::TargetStatus::BothTargets:
      return "BothTargets";
    default:
      return "Unknown";
  }
}

void setup() {
  //clear array values
  memset(movingPowerA, 0, sizeof(movingPowerA));
  //mmWave Connections
  COMSerial.begin(9600);
  ShowSerial.begin(9600);
  delay(500);

  ShowSerial.println("Programme Starting!");

  xiao_config.enableEngineeringModel();

  // //setting gate detection threshold
  // for(int i = 0; i < 9; i++){
  // xiao_config.setGatePower(i, 50, 50);
  // }

  //BLE Connections
  Serial.begin(115200);
  Serial.println("Starting BLE work!");

  BLEDevice::init(bleServerName2);
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID_NODE2);
  pServer->setCallbacks(new MyServerCallbacks());
  pCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_NODE2, BLECharacteristic::PROPERTY_NOTIFY);

  pService->start();
  // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // this still is working for backward compatibility
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID_NODE2);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("Characteristic defined! Now you can read it in your phone!");
}

void loop() {

  if (deviceConnected) {
  
    int retryCount = 0;
    const int MAX_RETRIES = 10;  // Maximum number of retries to prevent infinite loops

  //Get radar status
    do {
      radarStatus = xiao_config.getStatus();
      staticPower.get_samples(radarStatus.radarStaticPower.staticGate);
      staticPower.rolling_avg();
      movingPower.get_samples(radarStatus.radarMovePower.moveGate);
      movingPower.rolling_avg();
      retryCount++;
    } while (radarStatus.targetStatus == Seeed_HSP24::TargetStatus::ErrorFrame && retryCount < MAX_RETRIES);

  if (radarStatus.targetStatus != Seeed_HSP24::TargetStatus::ErrorFrame) {
  //Put values into array
  for(uint8_t i = 0; i < 8; i++) {
    movingPowerA[i] = movingPower.avg_value(i);
  }
  pCharacteristic->setValue((uint8_t *)&movingPowerA, sizeof(movingPowerA));
  pCharacteristic->notify();
  }
  else{
    BLEDevice::startAdvertising();
  }
  delay(1000);
}





//Back up codes for debug:

  // //Parses radar status and prints results from debug serial port
  //     ShowSerial.print("Status: " + String(targetStatusToString(radarStatus.targetStatus)) + "  ----   ");
  //     ShowSerial.println("Distance: " + String(radarStatus.distance) + "  Mode: " + String(radarStatus.radarMode));

  //     if (radarStatus.radarMode == 1) {
  //       ShowSerial.print("Move:");
  //       for (int i = 0; i < 8; i++) {
  //         ShowSerial.print(" " + String(movingPower.avg_value(i)) + ",");
  //       }
  //       ShowSerial.println("");
  //       ShowSerial.print("Static:");
  //       for (int i = 0; i < 8; i++) {
  //         ShowSerial.print(" " + String(staticPower.avg_value(i)) + ",");
  //       }
  //       ShowSerial.println("");
  //       ShowSerial.println("Photosensitive: " + String(radarStatus.photosensitive));
  //     }
  //   }

