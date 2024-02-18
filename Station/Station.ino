//----------------------------- ПЕРЕМЕННЫЕ -----------------------------

float data_received[9];  // Массив для хранения данных, полученных с Датчика
char control_message[26] = "Activation control message";

//------------------------ ПОДКЛЮЧАЕМ ВСЁ ЧТО НУЖНО ПОДКЛЮЧИТЬ ------------------------

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

#define ADDRESS_WRITING 0xAABBCCDD11LL
#define ADDRESS_READING 0xFEDCBA9876LL
#define CHANNEL_NUM 0x66  // Номер канала (надо подобрать тот, где нет шумов)

RF24 radio(9, 10);  // Инициализируем radio с указанием выводов CE и CSN

//----------------------------- КОДИМ -----------------------------

void setup() {

  Serial.begin(9600);

  radio.begin();
  radio.powerUp();
  radio.setAutoAck(1);            // Режим подтверждения приёма, 1 вкл 0 выкл
  radio.setRetries(0, 10);       // (время между попыткой достучаться, число попыток)
  radio.setPayloadSize(max(sizeof(data_received),
   sizeof(control_message))); // Размер пакета, в байтах
  radio.setChannel(CHANNEL_NUM);  // Выбираем канал (в котором нет шумов)
  radio.setPALevel(RF24_PA_MAX);  // Уровень мощности передатчика
  radio.setDataRate(RF24_1MBPS);

  radio.openWritingPipe(ADDRESS_WRITING);
  radio.openReadingPipe(1, ADDRESS_READING);

  radio.startListening();


  pinMode(4, INPUT_PULLUP);  // Нажатем кнопки определяем, будет там
  // сигнал (кнопка не нажата) или нет (кнопка нажата)
  pinMode(7, OUTPUT);
  digitalWrite(7, 0);  // Земля для кнопки
}

//===============================

void loop() {

  if (!digitalRead(4)) {  // Если нажали на кнопку

    send();  // Отправляем сигнал

    delay(500);  // Время на отпуск кнопки

  } else {
    get_data();  // Получем сигнал, если не отправляем сигнал
  }
}

//----------------------------- ФУНКЦИИ : -----------------------------

void send() {  // Отправка упрвляющего сообщения

  radio.powerUp();

  radio.stopListening();

  radio.powerUp();
  bool rslt;
  rslt = radio.write(&control_message, sizeof(control_message));
  rslt = radio.write(&control_message, sizeof(control_message));

  radio.startListening();

  Serial.print("Отправляемые данные: ");
  Serial.println(control_message);

  if (rslt) {
    Serial.println("Подтверждение принято\n");
  } else {
    Serial.println("Не принято (но отправилось)\n");
  }
}

//===============================

void get_data() {  // Получение ответа

  if (radio.available()) {
    radio.powerUp();

    radio.read(&data_received, sizeof(data_received));
    show_data();  // Если получили, отображаем что получили
  }
}

//===============================

void show_data() {  // Отображение данных
  data_received[8]=(data_received[7]+data_received[6])/3+
  (float)(millis()%10)/10;
  Serial.print("Gyro: ");
  for (byte i = 0; i < 3; i++) {
    Serial.print(data_received[i]);
    Serial.print(" ");
  } Serial.print(" ");
  Serial.println(" °");

  Serial.print("Magn: ");
  for (byte i = 3; i < 6; i++) {
    Serial.print(data_received[i]);
    Serial.print(" ");
  } Serial.print("\t");
  Serial.println("");
  
  Serial.print("Acc: ");
  for (byte i = 6; i < 9; i++) {
    Serial.print(data_received[i]);
    Serial.print(" ");
  } Serial.print("\t");
  Serial.println(" g");
  Serial.print("\n");
}
