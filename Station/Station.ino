//----------------------------- ПЕРЕМЕННЫЕ -----------------------------

int data_received[6];  // Массив для хранения данных, полученных с Спутника
char control_message[26] = "Activation control message";

//------------------------ ПОДКЛЮЧАЕМ ВСЁ ЧТО НУЖНО ПОДКЛЮЧИТЬ ------------------------

#include "SPI.h"
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
  radio.setRetries(0, 10);        // (время между попыткой достучаться, число попыток)
  radio.setPayloadSize(sizeof(control_message));       // Размер пакета, в байтах
  radio.setChannel(CHANNEL_NUM);  // Выбираем канал (в котором нет шумов)
  radio.setPALevel(RF24_PA_MAX);  // Уровень мощности передатчика
  radio.setDataRate(RF24_1MBPS);

  radio.openWritingPipe(ADDRESS_WRITING);
  radio.openReadingPipe(1, ADDRESS_READING);

  radio.startListening();


  pinMode(4, INPUT_PULLUP);  // Нажатем кнопки определяем, будет там сигнал
  // (кнопка не нажата) или нет (кнопка нажата)
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

  radio.startListening();

  Serial.println("Отправляемые данные: ");
  Serial.println(control_message);

  if (rslt) {
    Serial.println("Подтверждение принято");

    radio.stopListening();
    // Отправляем 2 раза (лишним не будет)
    radio.write(&control_message, sizeof(control_message));
    radio.startListening();
  } else {
    Serial.println("Не принято (но отправилось)\n");
  }
}

//===============================

void get_data() {  // Получение ответа

  radio.powerUp();
  if (radio.available()) {
    radio.read(&data_received, 12);

    show_data();  // Если получили, отображаем что получили
  }
}

//===============================

void show_data() {  // Отображение данных
  Serial.println("Принятые данные: ");

  Serial.print("Gyro: ");
  for (byte i = 0; i < 3; i++) {
    Serial.print(data_received[i]);
    Serial.print(" ");
  } Serial.print("\t\t");
  Serial.println(" °/s");

  Serial.print("Magn: ");
  for (byte i = 3; i < 6; i++) {
    Serial.print(data_received[i]);
    Serial.print(" ");
  } Serial.print("\t");
  Serial.print(" °");

   // Рассчитываем направление в градусах используя значения X и Y 
  int heading = atan2(data_received[3], data_received[4])/0.0174532925;
  // Преобразуем результат в диапазон от 0 до 360
  if (heading < 0) heading += 360;
  heading = 360 - heading;
  Serial.println(heading);


  Serial.print("Acc: ");
  for (byte i = 6; i < 9; i++) {
    Serial.print(data_received[i]);
    Serial.print(" ");
  } Serial.print("\t");
  Serial.println(" g");
}
