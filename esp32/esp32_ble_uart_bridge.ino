/*
 * ESP32-C3 BLE UART 브릿지
 * STM32 (USART1) <-> ESP32-C3 (BLE) <-> 맥북 (LightBlue)
 *
 * 보드: ESP32C3 Dev Module (Arduino IDE)
 *
 * 배선:
 *   STM32 PB6  (USART1_TX) -> ESP32 GPIO4 (RX)
 *   STM32 PA10 (USART1_RX) <- ESP32 GPIO3 (TX)
 *   GND   공통
 *   3V3   STM32 -> ESP32 (USB 케이블 없이 전원 공급)
 *
 * 주의: GPIO0/GPIO1은 부트스트랩 핀이라 UART로 쓰면 불안정
 *       (부팅 시 노이즈성 신호) -> GPIO3/GPIO4로 최종 확정
 *
 * 2026.07.27
 */

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

BLECharacteristic *pTxCharacteristic;
HardwareSerial MySerial(1);  // UART1 사용

class MyCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String rxValue = pCharacteristic->getValue();
    if (rxValue.length() > 0) {
      Serial.print("BLE received: ");
      Serial.println(rxValue);
      MySerial.write((uint8_t*)rxValue.c_str(), rxValue.length());
      Serial.println("Sent to UART");
    }
  }
};

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting BLE...");

  MySerial.begin(115200, SERIAL_8N1, 4, 3);  // RX=GPIO4, TX=GPIO3

  BLEDevice::init("ESP32_UART");
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);

  pTxCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_TX,
                        BLECharacteristic::PROPERTY_NOTIFY);
  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_RX,
                        BLECharacteristic::PROPERTY_WRITE);
  pRxCharacteristic->setCallbacks(new MyCallbacks());

  pService->start();
  pServer->getAdvertising()->start();

  Serial.println("BLE advertising started");
}

void loop() {
  if (MySerial.available()) {
    String data = "";
    while (MySerial.available()) {
      data += (char)MySerial.read();   // readString()은 타임아웃 누적 문제로
                                        // 즉시 바이트 단위 읽기로 변경함
    }
    Serial.print("UART received: ");
    Serial.println(data);
    pTxCharacteristic->setValue(data.c_str());
    pTxCharacteristic->notify();
  }
}
