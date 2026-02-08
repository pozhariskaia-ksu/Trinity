#include <WiFi.h>
#include "ESP32_NOW.h"
#include <ESP32Servo.h>

#define LED_BUILTIN 2
#define ESPNOW_WIFI_CHANNEL 6

#define H_FIX 16
#define V_FIX 1

int minUs = 550;
//int minUs = 500;
int maxUs = 2450;
//int maxUs = 2500;

int servo1Pin = 15;
int servo2Pin = 16;

typedef struct struct_servo_payload {
  short h;
  short v;
  short l;
  unsigned int s; 
} struct_servo_payload;


struct_servo_payload servoPayload;


uint8_t ctrtlAddress[] = {0xCC, 0xDB, 0xA7, 0x12, 0x65, 0xC8};



esp_now_peer_info_t peerInfo;

Servo servoH;
Servo servoV;

// функция обратного вызова. будет выполнен при подтверждении доставыки пакета
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) 
{
  Serial.print("[?] Last Packet Send Status (REM->CTRL):\t");

  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// функция обратного вызова. будет выполнен приполучении данных с удаленой стороны
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&servoPayload, incomingData, sizeof(servoPayload));
  
  Serial.printf("[i] H=%d, V=%d, L=%d\n", servoPayload.h, servoPayload.v, servoPayload.l);

  servoH.write(servoPayload.h + H_FIX);
  servoV.write(servoPayload.v + V_FIX);

  if(servoPayload.l == 1)
    digitalWrite(LED_BUILTIN, HIGH);
  else
    digitalWrite(LED_BUILTIN, LOW);


  // Отправка пакета по протаколу ESP-NOW
  esp_err_t result = esp_now_send(ctrtlAddress, (uint8_t *) &servoPayload, sizeof(servoPayload));
 // проверка на отправку  
  if (result == ESP_OK) {
    Serial.println("[+] Sending confirmed");
  }
  else {
    Serial.println("[e] Sending error");
  } 
}

ESP32PWM pwm;

void setup() {
  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);


  servoH.setPeriodHertz(50);      // Стандарт 50hz сервак
  servoV.setPeriodHertz(50);      // Стандарт 50hz сервак
  servoH.attach(servo1Pin, minUs, maxUs);
  servoV.attach(servo2Pin, minUs, maxUs);
  
  Serial.begin(115200);


  // Инициализация вай фай модуля
  WiFi.mode(WIFI_STA);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);
  while (!WiFi.STA.started()) {
    delay(100);
  }

  Serial.println("[i] CubeSatKsu (Remote)");
  Serial.println("[i] Current Wi-Fi parameters:");
  Serial.println("[i]   Mode: STA");
  Serial.println("[i]   MAC Address: " + WiFi.macAddress());
  Serial.printf("[i]   Channel: %d\n", ESPNOW_WIFI_CHANNEL);

  Serial.print("[i] ESP-NOW Init ...");
  ESP_NOW.begin();
  Serial.println(" done!");

  Serial.printf("[i] ESP-NOW version: %d, max data length: %d\n", ESP_NOW.getVersion(), ESP_NOW.getMaxDataLen());
  


  pinMode (LED_BUILTIN, OUTPUT);

  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));  
  


//корректировка сервомашинок програмно
    servoH.write(90 + H_FIX);
    servoV.write(90 + V_FIX);  

    digitalWrite(LED_BUILTIN, HIGH);


   // Структура с информацией об удаленном узле 

  esp_now_register_send_cb(esp_now_send_cb_t(OnDataSent));

  // заполняем структуру peerInfo
  memcpy(peerInfo.peer_addr, ctrtlAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;// Передаем пакеты без шифрования
  
  // Добавляем удаленное подключение (установление дружественных отношений с приёмником)        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }    
}


void loop() {
  delay(1000);
}
