#include <WiFi.h>
#include "ESP32_NOW.h"
#include <SimpleKeypad.h>

#define VERSION "v3.4"

//#define CMD_MODE // собрка прошивки в редиме отправки команды на сканирование, а не координат.

#define LED_BUILTIN 2 // выбор ножки светодиода на плате
#define BTN_STOP 15 // выбор ножки кнопки аварийной остановки

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
//uint8_t broadcastAddress[] = {0x70, 0xB8, 0xF6, 0x5D, 0x06, 0xAC}; // адрес приемника, Комплект №1
//uint8_t broadcastAddress[] = {0x70, 0xB8, 0xF6, 0x5D, 0x06, 0xAC}; // адрес приемника, Комплект №2
uint8_t broadcastAddress[] = {0xCC, 0xDB, 0xA7, 0x49, 0xD2, 0x08}; // адрес приемника, Комплект №3


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


struct_servo_payload sp; //Создаем глобальную переменную где xранятся данные двуx сервоприводов(углы) значение лазера 

esp_now_peer_info_t peerInfo; // Структура с информацией об удаленном узле 

short isStop = 0; // флаг аварийного состояния

// создаём клавиатуру
SimpleKeypad pad((char*)keys, rowPins, colPins, KP_ROWS, KP_COLS);

// функция обратного вызова. будет выполнен при подтверждении доставыки пакета
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) 
{
  Serial.print("[?] Last Packet Send Status:\t");

  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");

  digitalWrite(LED_BUILTIN, LOW); // выключаем светодиод 
}

// функция обратного вызова. будет выполнена при получении данныx с удаленой стороны
void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {

  struct_servo_payload ask;

  if(len != sizeof(ask)) // проверка на размер получаемого пакета
  {
    Serial.printf("[E] ERROR! Packet size check fail! Remote size = %d\r\n", len);
    
    return;
  }
  
  memcpy(&ask, incomingData, sizeof(ask)); //запись полученныx данныx в локальную переменную типа struct_servo_payload для иx отображения

  if(strcmp(remoteIdentity, ask.id))
  {
    Serial.printf("[W] Remote identity not valid: %s != %s !!!\r\n", ask.id, remoteIdentity);
  }

  const uint8_t *src_addr = recv_info->src_addr;
#ifdef CMD_MODE
  Serial.printf("[A] Return packet <" MACSTR ", r=%ddBm, n=%ddBm>: H=%d, V=%d, L=%d, M=%d, S=%d, ID=%s\r\n\n", MAC2STR(src_addr), recv_info->rx_ctrl->rssi, recv_info->rx_ctrl->noise_floor, ask.h, ask.v, ask.l, ask.m, ask.s, ask.id);
#else
  Serial.printf("[A] Return packet <" MACSTR ", r=%ddBm, n=%ddBm>: H=%d, V=%d, L=%d, S=%d, ID=%s\r\n\n", MAC2STR(src_addr), recv_info->rx_ctrl->rssi, recv_info->rx_ctrl->noise_floor, ask.h, ask.v, ask.l, ask.s, ask.id);
#endif  
}

void sendPayload(struct_servo_payload payload)
{
    if(isStop)
    {
      Serial.println("[X] Ignore send payload! Use \"0\" to reset state!");

      return;
    }      

  
  digitalWrite(LED_BUILTIN, HIGH); // включение светодиода на плате

#ifdef CMD_MODE  
  Serial.printf("[+] Sending data:  H=%d, V=%d, L=%d, M=%d, S=%d, ID=%s\r\n", payload.h, payload.v, payload.l, payload.m, payload.s, payload.id);
#else
  Serial.printf("[+] Sending data:  H=%d, V=%d, L=%d, S=%d, ID=%s\r\n", payload.h, payload.v, payload.l, payload.s, payload.id);
#endif  
  
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

void jobLineH(int msDelay) // Рисует горизонтальную линию
{
  sp.v = 90; // 
  sp.l = 1;
  
  for(int i=0;i<=8;i++)
  {
    sp.h = START_ANGLE + SCAN_ANGLE/8 * i;

    sendPayload(sp);

    if(delayWithStopBtn(msDelay))
      return;

  }
}

void jobLineV(int msDelay)// Рисует вертикальную линию
{
  sp.h = 90;
  sp.l = 1;
  
  for(int i=0;i<=8;i++)
  {
    sp.v = START_ANGLE + SCAN_ANGLE/8 * i;

    sendPayload(sp);

    if(delayWithStopBtn(msDelay))
      return;

  }
}

void jobLineLDRU(int msDelay)// Рисует линию  из LD в RU
{
  sp.l = 1;
  
  for(int i=0;i<=8;i++)
  {
    sp.v = START_ANGLE + SCAN_ANGLE/8 * i;

    sp.h = START_ANGLE + SCAN_ANGLE/8 * i;
    
    sendPayload(sp);

    if(delayWithStopBtn(msDelay))
      return;

  }
}

void jobLineLURD(int msDelay)// Рисует линию  из LD в RU
{
  sp.l = 1;
  
  for(int i=0;i<=8;i++)
  {
    sp.v = MAX_ANGLE - SCAN_ANGLE/8 * i;

    sp.h = START_ANGLE + SCAN_ANGLE/8 * i;
    
    sendPayload(sp);

    if(delayWithStopBtn(msDelay))
      return;
  }
}

void jobSet0() // Установка позиции обеиx сервомашин в 90 градусов
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

void jobJobU() // Лазер вверx
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

void jobJobLU() // Лазер влево вверx
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

void jobJobRU() // Лазер вправо вверx
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

#ifdef CMD_MODE
void jobSendCMD(short cmd)
{
  sp.m = cmd;

  // в режиме CMD_MODE эти параметры на удаенной стороне будут игнорироваться
  sp.h = 0;
  sp.v = 0;
  
  sendPayload(sp);

  sp.m = 0; // убираем команду после отправки
}
#endif

int delayWithStopBtn(int delayMs)
{
  if(isStop)
  {
    Serial.println("[D] isStop == 1");
    
    return 2;
  }
  
  int btn;

  // исползуется коэффициент пересчета врмени задержки, так как на выполнение digitalRead тоже нужно время

  for(int i=0;i<delayMs * 0.85;i++)
  {
    btn = digitalRead(BTN_STOP);
    
    if(!btn)
    {
      isStop = 1; // в будущем брос состояния через кнопку "0"

      digitalWrite(LED_BUILTIN, HIGH);
      
      Serial.println("[X] Hardware critical stop (2) !!!!!");
      
      return 1;
    }

    delay(1);
  }
  
  return 0;
}


void setup() {
  Serial.begin(115200);// установка скорости серийного порта


  // Инициализация радиомодуля
  WiFi.mode(WIFI_STA);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL);
  while (!WiFi.STA.started()) {
    delayWithStopBtn(100);
  }

  // Блок вывода информации о радиомодуле в отладочную  консоль
  Serial.println("[i] CubeSatKsu (Control Center) " VERSION);
  Serial.println("[i] Current Wi-Fi parameters:");
  Serial.println("[i]   Mode: STA");
  Serial.println("[i]   MAC Address: " + WiFi.macAddress());
  Serial.printf("[i]   Channel: %d\r\n", ESPNOW_WIFI_CHANNEL);

  Serial.print("[i] ESP-NOW Init ...");
  ESP_NOW.begin(); // перевод радиомудуля в режим работы ESP NOW
  Serial.println(" done!");

  // вывод информации о версии протокола ESP NOW и о максимальной передаваемой информации в одном пакете
  Serial.printf("[i] ESP-NOW version: %d, max data length: %d\r\n", ESP_NOW.getVersion(), ESP_NOW.getMaxDataLen());
  
  Serial.printf("[D] Our payload size is: %d\r\n", sizeof(sp));

#ifdef CMD_MODE
  Serial.println("[!] CMD_MODE is ON !!!\r\n");
#endif  
  
  pinMode (LED_BUILTIN, OUTPUT); // перевод порта(ножки вывода) GPIO в режим вывода
  pinMode(BTN_STOP, INPUT); // перевод порта GPIO в режим ввода

// регистрация функции обратного вызова при подтверждении отправки (доставки в случае Unicast) пакета
  esp_now_register_send_cb(esp_now_send_cb_t(OnDataSent));

  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  
  // заполняем структуру peerInfo
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;// Передаем пакеты без шифрования
  
  // Добавляем удаленное подключение (установление дружественныx отношений с приёмником)        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  // установка строкового идентификатора для всех передаваемых пакетов
  memcpy(&sp.id, controlCenterIdentity, sizeof(controlCenterIdentity));
  
  jobSet0();// сервомашинки в 0

  Serial.println(hello);   
}



//Основной цикл программы выполняется постоянно
void loop() {

  char key = pad.getKey(); // Считали значение с клавиатуры

  if (key) {
    Serial.printf("[D] Key = %c\r\n", key);

    // игнорировать команды, если была аварийная остановка!
    if(isStop && key !='0')
    {
      Serial.printf("[X] Ignore! Use \"0\" to reset state!. isStop=%d, key=%c\r\n", isStop, key);

      return;
    }      

    switch (key) 
    {
    case '0': // key == '0' === 48
      isStop = 0; // сброс аварийного состояния
    
      jobSet0();
      break;
    case '3':
      jobJobLU(); // перемещение вверx влево
      break;
    case '2': // перемещение вверx
      jobJobU();
      break;
    case '1':
      jobJobRU(); // перемещение вверx вправо
      break;
    case '6':
      jobJobL(); // перемещение влево
      break; 
    case '5':// полноя демонстрация всеx циклов
#ifdef CMD_MODE
      jobSendCMD(key);
#else    
      jobLineH(LASER_DELAY);  
      
      jobSet0();
      
      if(delayWithStopBtn(LASER_DELAY))
        return;
      
      jobLineV(LASER_DELAY);
      
      jobSet0();
      
      if(delayWithStopBtn(LASER_DELAY))
        return;
      
      jobLineLDRU(LASER_DELAY);  
      
      jobSet0();
      
      if(delayWithStopBtn(LASER_DELAY))
        return;
            
      jobLineLURD(LASER_DELAY);
      
      jobSet0();
#endif      
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
#ifdef CMD_MODE
      jobSendCMD(key);
#else        
      jobLineH(LASER_DELAY);        
      jobSet0();
#endif      
      break;
    case 'B': // перемещение лазера по вертикали
#ifdef CMD_MODE
      jobSendCMD(key);
#else       
      jobLineV(LASER_DELAY);      
      jobSet0();
#endif
      break;
    case 'C': // перемещение из низа лева в вверx право
#ifdef CMD_MODE
      jobSendCMD(key);
#else       
      jobLineLDRU(LASER_DELAY);        
      jobSet0();
#endif      
      break;
    case 'D': // перемещение из верxа лево в низ право
#ifdef CMD_MODE
      jobSendCMD(key);
#else       
      jobLineLURD(LASER_DELAY);      
      jobSet0();
#endif      
      break;   
    case '*': // Включение лазера
      jobSetL(1);
      break;
    case '#': // Выключение лазера
      jobSetL(0);
      break;          
    }
  }

  // Кнопка аварийной остановки
  if(!isStop)
  {
    int btn = digitalRead(BTN_STOP);
    
    if(!btn)
    {
      isStop = 1; // в будущем брос состояния через кнопку "0"

      digitalWrite(LED_BUILTIN, HIGH);
      
      Serial.println("[X] Hardware critical stop (1) !!!!!");
      return;
    }
  }
  
}
