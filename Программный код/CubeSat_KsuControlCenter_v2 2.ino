#include <WiFi.h>
#include "ESP32_NOW.h"
#include <SimpleKeypad.h>

#define LED_BUILTIN 2 // выбор ножки светодиода на плате

#define ESPNOW_WIFI_CHANNEL 6 // выбор канала радиомодуля

#define START_ANGLE 50 // Начальный угол
#define MAX_ANGLE 130 // Конченый угол
#define SCAN_ANGLE 80 //Угол сканирования
#define MAN_STEP 10 // угол ручной корректировки

// размеры клавиатуры
#define KP_ROWS 4
#define KP_COLS 4

// задержка между командами
#define LASER_DELAY 3000

// пины подключения (по порядку штекера)
byte colPins[KP_COLS] = {5, 17, 16, 4};
byte rowPins[KP_ROWS] = {3, 21, 19, 18};

// массив имён кнопок
char keys[KP_ROWS][KP_COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}     
};


//uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // Широрковещательный адресс который получат все устройства на данном канале
uint8_t broadcastAddress[] = {0x70, 0xB8, 0xF6, 0x5D, 0x06, 0xAC}; // адрес приемника




// Создаем собственный тип данных 
typedef struct struct_servo_payload {
  short h; // Данные горизонтальные 
  short v; // Данные вертикаль
  short l; // Лазер 
  unsigned int s = 0; // порядковый номер пакета
} struct_servo_payload;


struct_servo_payload sp; //Создаем глобальную переменную где хранятся данные двух сервоприводов(углы) значение лазера 

esp_now_peer_info_t peerInfo; // Структура с информацией об удаленном узле 

// создаём клавиатуру
SimpleKeypad pad((char*)keys, rowPins, colPins, KP_ROWS, KP_COLS);

// функция обратного вызова. будет выполнен при подтверждении доставыки пакета
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) 
{
  Serial.print("[?] Last Packet Send Status:\t");

  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");

  digitalWrite(LED_BUILTIN, LOW); // выключаем светодиод 
}

// функция обратного вызова. будет выполнен приполучении данных с удаленой стороны
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {

  struct_servo_payload ask;
  
  memcpy(&ask, incomingData, sizeof(ask)); //запрись полученных данных в локальную переменную типа struct_servo_payload для их отображения
  
  Serial.printf("[A] H=%d, V=%d, L=%d, S=%d\n", ask.h, ask.v, ask.l, ask.s);
}


void jobLineH(int msDelay) // Рисует горизонтальную линию
{
  sp.v = 90; // 
  sp.l = 1;
  
  for(int i=0;i<8;i++)
  {
    sp.h = START_ANGLE + SCAN_ANGLE/8 * i;

    sendPayload(sp);

    delay(msDelay);
  }
}

void jobLineV(int msDelay)// Рисует вертикальную линию
{
  sp.h = 90;
  sp.l = 1;
  
  for(int i=0;i<8;i++)
  {
    sp.v = START_ANGLE + SCAN_ANGLE/8 * i;

    sendPayload(sp);

    delay(msDelay);
  }
}

void jobLineLDRU(int msDelay)// Рисует линию  из LD в RU
{
  sp.l = 1;
  
  for(int i=0;i<8;i++)
  {
    sp.v = START_ANGLE + SCAN_ANGLE/8 * i;

    sp.h = START_ANGLE + SCAN_ANGLE/8 * i;
    
    sendPayload(sp);

    delay(msDelay);
  }
}

void jobLineLURD(int msDelay)// Рисует линию  из LD в RU
{
  sp.l = 1;
  
  for(int i=0;i<8;i++)
  {
    sp.v = MAX_ANGLE - SCAN_ANGLE/8 * i;

    sp.h = START_ANGLE + SCAN_ANGLE/8 * i;
    
    sendPayload(sp);

    delay(msDelay);
  }
}

void jobSet0() // Установка позиции обеих сервомашин в 90 градусов
{
  sp.h = 90;
  sp.v = 90;
  sp.l = 1;
  
  sendPayload(sp);
}

void jobSetL(short laserOn) // Включить лазер
{
  sp.l = laserOn;
  
  sendPayload(sp);
}

void jobJobU() // Лазер вверх
{
  if(sp.v < MAX_ANGLE)
    sp.v += MAN_STEP;
  
  sendPayload(sp);
}

void jobJobR() // Лазер вправо
{
  if(sp.h < MAX_ANGLE)
    sp.h += MAN_STEP;
  
  sendPayload(sp);
}

void jobJobD() //Лазер вниз
{
  if(sp.v > START_ANGLE)
    sp.v -= MAN_STEP;
  
  sendPayload(sp);
}

void jobJobL() // Лазер влево
{
  if(sp.h > START_ANGLE)
    sp.h -= MAN_STEP;
  
  sendPayload(sp);
}

void jobJobLU() // Лазер влево вверх
{
  if(sp.v < MAX_ANGLE)
    sp.v += MAN_STEP;
  
  if(sp.h > START_ANGLE)
    sp.h -= MAN_STEP;

  sendPayload(sp);
}

void jobJobLD() // Лазер влево вниз
{
  if(sp.v > START_ANGLE)
    sp.v -= MAN_STEP;
  
   if(sp.h > START_ANGLE)
    sp.h -= MAN_STEP;

  sendPayload(sp);
}

void jobJobRU() // Лазер вправо вверх
{
  if(sp.v < MAX_ANGLE )
    sp.v += MAN_STEP;
  
  if(sp.h < MAX_ANGLE )
    sp.h += MAN_STEP;

  sendPayload(sp);
}

void jobJobRD() // Лазер вправо вниз
{
  if(sp.v > START_ANGLE )
    sp.v -= MAN_STEP;
  
  if(sp.h < MAX_ANGLE )
    sp.h += MAN_STEP;

  sendPayload(sp);
}



void sendPayload(struct_servo_payload payload)
{
  digitalWrite(LED_BUILTIN, HIGH); // включение светодиода на плате
  
  Serial.printf("[+] Send data H=%d, V=%d, L=%d, S=%d \n", payload.h, payload.v, payload.l, payload.s);
  
  // Отправка пакета по протаколу ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &payload, sizeof(payload));
 // проверка на отправку  
  if (result == ESP_OK) {
    Serial.println("[+] Sending confirmed");
  }
  else {
    Serial.println("[e] Sending error");
  }    

  sp.s++;
}

void setup() {
  Serial.begin(115200);// установка скорости серийного порта


  // Инициализация радиомодуля
  WiFi.mode(WIFI_STA);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);
  while (!WiFi.STA.started()) {
    delay(100);
  }

  // Блок вывода информации о радиомодуле в отладочную  консоль
  Serial.println("[i] CubeSatKsu (Control Center)");
  Serial.println("[i] Current Wi-Fi parameters:");
  Serial.println("[i]   Mode: STA");
  Serial.println("[i]   MAC Address: " + WiFi.macAddress());
  Serial.printf("[i]   Channel: %d\n", ESPNOW_WIFI_CHANNEL);

  Serial.print("[i] ESP-NOW Init ...");
  ESP_NOW.begin(); // перевод радиомудуля в режим работы ESP NOW
  Serial.println(" done!");

  // вывод информации о версии перотокола ESP NOW о максимальной передаваемой информации в одном пакете
  Serial.printf("[i] ESP-NOW version: %d, max data length: %d\n", ESP_NOW.getVersion(), ESP_NOW.getMaxDataLen());
  
  pinMode (LED_BUILTIN, OUTPUT);// перевод порта(ножки вывода) GPIO в режим вывода

// регистрация функции обратного вызова при подтверждении отправки(доставки?) пакета
  esp_now_register_send_cb(esp_now_send_cb_t(OnDataSent));

  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  
  // заполняем структуру peerInfo
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;// Передаем пакеты без шифрования
  
  // Добавляем удаленное подключение (установление дружественных отношений с приёмником)        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  
   jobSet0();// сервомашинки в 0
}

//Основной цикл программы выполняется постоянно
void loop() {
  


  char key = pad.getKey(); // Считали значение с клавиатуры

  if (key) {
    Serial.println(key);

    switch (key) 
    {
    case '0': // key == '0' === 48
      jobSet0();
      break;
    case '3':
      jobJobLU(); // перемещение вверх влево
      break;
    case '2': // перемещение вверх
      jobJobU();
      break;
    case '1':
      jobJobRU(); // перемещение вверх вправо
      break;
    case '6':
      jobJobL(); // перемещение влево
      break; 
    case '5':// полноя демонстрация всех циклов
      jobLineH(LASER_DELAY);  
      jobSet0();
      delay(LASER_DELAY);
      jobLineV(LASER_DELAY);
      jobSet0();
      delay(LASER_DELAY);
      jobLineLDRU(LASER_DELAY);  
      jobSet0();
      delay(LASER_DELAY);
      jobLineLURD(LASER_DELAY);
      jobSet0();
      break;   

    case '4':
      jobJobR(); // перемещение вправо 
      break; 
    case '9':
      jobJobLD(); // перемещение влево вниз
      break;     
    case '8': // перемещение вниз
      jobJobD();
      break;
    case '7':
      jobJobRD(); // перемещение вправо вниз
      break;
    case 'A': // перемещение лазера по горизонтали
      jobLineH(LASER_DELAY);  
      jobSet0();
      break;
    case 'B': // перемещение лазера по вертикали
      jobLineV(LASER_DELAY);
      jobSet0();
      break;
    case 'C': // перемещение из низа лева в вверх право
      jobLineLDRU(LASER_DELAY);  
      jobSet0();
      break;
    case 'D': // перемещение из верха лево в низ право
      jobLineLURD(LASER_DELAY);
      jobSet0();
      break;   
    case '*': // Включение лазера
      jobSetL(1);
      break;
    case '#': // Выключение лазера
      jobSetL(0);
      break;          
    }
  }
  
}
