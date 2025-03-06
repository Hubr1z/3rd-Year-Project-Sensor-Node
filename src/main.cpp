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
#include <HardwareSerial.h>
#include <mmwave_for_xiao.h>
#include <rolling_average.h>

#include "esp_timer.h"
//BLE Globals
BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
uint8_t movingPowerA[8], staticPowerA[8];
bool lowBattery = false;


class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };
  
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

// Define the SoftwareSerial object, D2 as RX, D3 as TX, connect to the serial port of the mmwave sensor
// SoftwareSerial RadarSerial(D2, D3);
HardwareSerial RadarSerial(0); // Alias for UART0 (Serial0)

// Creates a global Serial object for printing debugging information
#define ShowSerial Serial

// Initialising the radar configuration
// Seeed_HSP24 xiao_config(COMSerial, ShowSerial);
Seeed_HSP24 xiao_config(RadarSerial);

Seeed_HSP24::RadarStatus radarStatus;
Seeed_HSP24::AskStatus askStatus;

// RollingAVG staticPower;
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

void ISR(void *arg){
  uint32_t Vbatt = 0;
  for(int i = 0; i < 16; i++) {
    Vbatt = Vbatt + analogReadMilliVolts(A0); // ADC with correction   
  }
  float Vbattf = 2 * Vbatt / 16 / 1000.0;     // attenuation ratio 1/2, mV --> V
  Serial.println(Vbattf, 3);

  if (Vbattf < 3.5){
    lowBattery = true;
    if (digitalRead (LED_BUILTIN))
      digitalWrite(LED_BUILTIN, LOW);
    else
      digitalWrite(LED_BUILTIN, HIGH);
  }
  else
    lowBattery = false;
};

void setup() {
  pinMode(A0, INPUT);         // ADC
  pinMode(LED_BUILTIN, OUTPUT);
  //LED Blink Ticker
  esp_timer_handle_t handle;
  esp_timer_create_args_t timerISR = {
    .callback = ISR,        //!< Function to call when timer expires
    .arg = nullptr,                          //!< Argument to pass to the callback
    .dispatch_method = ESP_TIMER_TASK,   //!< Call the callback from task or from ISR
    .name = "Timer1",               //!< Timer name, used in esp_timer_dump function
    .skip_unhandled_events = true,     //!< Skip unhandled events for periodic timers
  };
  esp_timer_create(&timerISR, &handle);
  esp_timer_start_periodic(handle, 1000000);


  //clear array values
  memset(movingPowerA, 0, sizeof(movingPowerA));
  //mmWave Connections
  RadarSerial.begin(115200, SERIAL_8N1, D7, D6);
  ShowSerial.begin(115200);
  delay(500);

  ShowSerial.println("Programme Starting!");

  xiao_config.enableEngineeringModel();

  // //setting gate detection threshold
  // for(int i = 0; i < 9; i++){
  // xiao_config.setGatePower(i, 50, 50);
  // }

  //BLE Connections

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
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P18);
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN, ESP_PWR_LVL_P18);
  Serial.println("Characteristic defined! Now you can read it in your phone!");
}

void loop() {
if (1){
  if (deviceConnected) {
  
    int retryCount = 0;
    const int MAX_RETRIES = 10;  // Maximum number of retries to prevent infinite loops

  //Get radar status
    do {
      radarStatus = xiao_config.getStatus();

      // staticPower.get_sample_set(radarStatus.radarStaticPower.staticGate);
      // staticPower.rolling_avg();

      movingPower.sample_acquisition(radarStatus.radarMovePower.moveGate);

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
  }
  else{
  BLEDevice::startAdvertising();
  }
}
else
BLEDevice::stopAdvertising();
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


