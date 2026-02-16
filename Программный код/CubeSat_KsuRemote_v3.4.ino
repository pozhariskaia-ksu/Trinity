#include <WiFi.h>
#include "ESP32_NOW.h"
#include <ESP32Servo.h>

//#define CMD_MODE

#define LED_BUILTIN 2
#define ESPNOW_WIFI_CHANNEL 6

// задержка между командами
#define LASER_DELAY 3000

#define START_ANGLE 50 // Начальный угол
#define MAX_ANGLE 130 // Конченый угол
#define SCAN_ANGLE 80 //Угол сканирования
#define MAN_STEP 10 // угол ручной корректировки

#define H_FIX 16
#define V_FIX 1

int minUs = 550;
//int minUs = 500;
int maxUs = 2450;
//int maxUs = 2500;

int servo1Pin = 15;
int servo2Pin = 16;

/*
 * Будем использовать 2 идентификатора, так как в задании сказано, что идентификатор нужно проверять вручную.
 * В нашем случае эта проверка уже выполняется на уровне ESP-NOW, но для выполнения условия ТЗ 
 * будет добавлена проверка для удаленной и локлаьной стороны в обработке 
 * получаемыx пакетов.
 */
char controlCenterIdentity[] = "JONES";// стрковый идентификатор пульта управления
char remoteIdentity[] = "ED209";// стрковый идентификатор исполнительного устройства

char hello[] = {0x20,0x2A,0x2A,0x2A,0x20,0x44,0x72,0x6F,0x70,0x20,0x79,0x6F,0x75,0x72,
0x20,0x77,0x65,0x61,0x70,0x6F,0x6E,0x2E,0x20,0x59,0x6F,0x75,0x20,0x68,0x61,0x76,0x65,
0x20,0x32,0x30,0x20,0x73,0x65,0x63,0x6F,0x6E,0x64,0x73,0x20,0x74,0x6F,0x20,0x63,0x6F,
0x6D,0x70,0x6C,0x79,0x21,0x20,0x2A,0x2A,0x2A,0x00};


// Создаем собственный тип данныx 
typedef struct struct_servo_payload {
  short h; // Данные горизонтальные 
  short v; // Данные вертикаль
  short l; // Лазер 
  unsigned int s = 0; // порядковый номер пакета
  char id[6]; // строковый идентификатор удаленной стороны
#ifdef CMD_MODE
  short m; // режим работы. Для упрощение и читаемости в параметре m будет передаваться код нажимаемой кливиши (для 5, A, B, C и D).
#endif   
} struct_servo_payload;

#ifdef CMD_MODE
short queue_mode = 0; // последняя полученная команда. будет обрабатываться в loop()
#endif

struct_servo_payload servoPayload;

//uint8_t ctrtlAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
//uint8_t ctrtlAddress[] = {0xCC, 0xDB, 0xA7, 0x12, 0x65, 0xC8}; // Комплект №2
uint8_t ctrtlAddress[] = {0xCC, 0xDB, 0xA7, 0x55, 0x54, 0x34}; // Комплект №3

esp_now_peer_info_t peerInfo;

Servo servoH;
Servo servoV;

// функция обратного вызова. будет выполнен при подтверждении доставыки пакета
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) 
{
  Serial.print("[?] Last Packet Send Status (REM->CTRL):\t");

  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Callback function executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&servoPayload, incomingData, sizeof(servoPayload));

#ifdef CMD_MODE
  Serial.printf("[i] H=%d, V=%d, L=%d, M=%d, S=%d, ID=%s\r\n", servoPayload.h, servoPayload.v, servoPayload.l, servoPayload.m, servoPayload.s, servoPayload.id);

  if(servoPayload.m == '5' || servoPayload.m == 'A' || servoPayload.m == 'B' || servoPayload.m == 'C' || servoPayload.m == 'D')
  {
    queue_mode = servoPayload.m;
  }
  else
  {
    doFromPayload();
  }  
#else  
  Serial.printf("[i] H=%d, V=%d, L=%d, S=%d, ID=%s\r\n", servoPayload.h, servoPayload.v, servoPayload.l, servoPayload.s, servoPayload.id);

  doFromPayload();  
#endif
}

void doFromPayload()
{
  Serial.printf("[D] Set servo/laser: H=%d, V=%d, L=%d\r\n", servoPayload.h, servoPayload.v, servoPayload.l);
  
  servoH.write(servoPayload.h + H_FIX);
  servoV.write(servoPayload.v + V_FIX);
  
  if(servoPayload.l == 1)
    digitalWrite(LED_BUILTIN, HIGH);
  else
    digitalWrite(LED_BUILTIN, LOW);  

  sendFeerback();    
}

void sendFeerback()
{
  // замена идентификатора
  memcpy(servoPayload.id, remoteIdentity, sizeof(remoteIdentity));

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

#ifdef CMD_MODE
void jobSet0() // Установка позиции обеиx сервомашин в 90 градусов
{
  servoPayload.h = 90;
  servoPayload.v = 90;
  servoPayload.l = 1;
  
  doFromPayload();
}

void jobLineH(int msDelay) // Рисует горизонтальную линию
{
  servoPayload.v = 90; // 
  servoPayload.l = 1;
  
  for(int i=0;i<=8;i++)
  {
    servoPayload.h = START_ANGLE + SCAN_ANGLE/8 * i;

    doFromPayload();

    delay(msDelay);
  }
}

void jobLineV(int msDelay)// Рисует вертикальную линию
{
  servoPayload.h = 90;
  servoPayload.l = 1;
  
  for(int i=0;i<=8;i++)
  {
    servoPayload.v = START_ANGLE + SCAN_ANGLE/8 * i;

    doFromPayload();

    delay(msDelay);
  }
}

void jobLineLDRU(int msDelay)// Рисует линию  из LD в RU
{
  servoPayload.l = 1;
  
  for(int i=0;i<=8;i++)
  {
    servoPayload.v = START_ANGLE + SCAN_ANGLE/8 * i;

    servoPayload.h = START_ANGLE + SCAN_ANGLE/8 * i;
    
    doFromPayload();

    delay(msDelay);
  }
}

void jobLineLURD(int msDelay)// Рисует линию  из LD в RU
{
  servoPayload.l = 1;
  
  for(int i=0;i<=8;i++)
  {
    servoPayload.v = MAX_ANGLE - SCAN_ANGLE/8 * i;

    servoPayload.h = START_ANGLE + SCAN_ANGLE/8 * i;
    
    doFromPayload();

    delay(msDelay);
  }
}
#endif


ESP32PWM pwm;

void setup() {
  // Allow allocation of all timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
//  ESP32PWM::allocateTimer(2);
//  ESP32PWM::allocateTimer(3);

  servoH.setPeriodHertz(50);      // Standard 50hz servo
  servoV.setPeriodHertz(50);      // Standard 50hz servo

  servoH.attach(servo1Pin, minUs, maxUs);
  servoV.attach(servo2Pin, minUs, maxUs);
  
  Serial.begin(115200);


  // Initialize the Wi-Fi module
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

  Serial.printf("[i] ESP-NOW version: %d, max data length: %d\r\n", ESP_NOW.getVersion(), ESP_NOW.getMaxDataLen());
  


  pinMode (LED_BUILTIN, OUTPUT);

  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));  
  esp_now_register_send_cb(esp_now_send_cb_t(OnDataSent));
  
    servoH.write(90 + H_FIX);
    servoV.write(90 + V_FIX);  

    digitalWrite(LED_BUILTIN, HIGH);

  // заполняем структуру peerInfo
  memcpy(peerInfo.peer_addr, ctrtlAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;// Передаем пакеты без шифрования
  
  // Добавляем удаленное подключение (установление дружественных отношений с приёмником)        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }   

  Serial.println(hello);   
}


void loop() {
  if(queue_mode != 0)
  {
    Serial.printf("[Q] Exec command from queue: %d (%c)\r\n", queue_mode, (char) queue_mode);

    switch (queue_mode) 
    {
    case '5':// полноя демонстрация всеx циклов
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
    case 'A': // перемещение лазера по горизонтали
      jobLineH(LASER_DELAY);        
      jobSet0();
      break;
    case 'B': // перемещение лазера по вертикали
      jobLineV(LASER_DELAY);      
      jobSet0();
      break;
    case 'C': // перемещение из низа лева в вверx право
      jobLineLDRU(LASER_DELAY);        
      jobSet0();
      break;
    case 'D': // перемещение из верxа лево в низ право
      jobLineLURD(LASER_DELAY);      
      jobSet0();
      break;   
    } 

    queue_mode = 0;
  }
  
  delay(100);
}
