//----------------------------- НАСТРОЙКИ -----------------------------

const byte time_sec = 3;
const byte frequency_data_gyro = 75/time_sec;
// Как часто будем снимать показания с гироскопа, для определения движения коробки (Сколько раз в секунду)
const byte threshold_turn = 90; // Порог поворота в градусах
const byte threshold_acceleration = 8; // Порог ускорения в g


// ПЕРЕМЕННЫЕ

float acc[3];       // Хранение значений акселерометра (x, y, z)
float gyr[3];       // Хранение значений гироскопа (x, y, z)
float mag[3];       // Хранение значений магнитометра (x, y, z)

int offsets[6];   /* Помещяем сюда первоначальные значения датчиков по осям x, y, z
(сначала акселерометра, потом гироскопа)
Или же убираем смещение нуля (offset) */

float data_final_send[9]; // Данные для отправки

char data_received[26]; /* Массив для хранения данных, полученных с Станции
Должен совпадать с отправляемым массивом */
const char control_message[26] = "Activation control message";

const uint8_t number_entries = time_sec * 3 * frequency_data_gyro; /* Количество записей в массив moving_of_box
  time_sec = количество секунд записи, 3 = количество осей*/

int moving_of_box[number_entries]; // Куда записываем данные с коробки

const float sensitive_gyr = 19.47;    // Чувствительность гироскопа       (В данном случае единица значения == 1°/s )
const float sensitive_acc = 28.67;    // Чувствительность акселерометра   (В данном случае единица значения == 1g)
const float sensitive_mag = 6.4;      // Чувствительность магнитометра


//------------------------ ПОДКЛЮЧАЕМ ВСЁ ЧТО НУЖНО ПОДКЛЮЧИТЬ ------------------------
#include <Wire.h>
#include "I2Cdev.h"
#include "ADXL345.h"
#include "L3G4200D.h"
#include "HMC5883L.h"

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

#define ADDRESS_READING 0xAABBCCDD11LL
#define ADDRESS_WRITING 0xFEDCBA9876LL
#define CHANNEL_NUM 0x66   // Номер канала (надо подобрать тот, где нет шумов)

RF24 radio(9, 10);  // Инициализируем radio с указанием выводов CE и CSN

ADXL345 accel;
L3G4200D gyro;
HMC5883L magn;

//----------------------------- КОДИМ -----------------------------


void setup() {
  // Включение и инициализация систем

  Serial.begin(9600);
  Serial.println("Инициализируем устройства I2C...");

  Wire.begin();

  gyro.initialize();
  gyro.setFullScale(250);           // Устанавливаем максимальное значение (по умолчанию 250°/s )    ((MAX value: 32768))
  accel.initialize();
  //accel.setFullResolution(??);    // Устанавливаем максимальное значение (по умолчанию 8g )        ((MAX value: 32768)) (±8g == ±32768)
  magn.initialize();

  radio.begin();
  radio.powerUp(); 
  radio.setAutoAck(1);           // Режим подтверждения приёма, 1 вкл 0 выкл
  radio.setRetries(0, 10);       // (время между попыткой достучаться, число попыток)
  radio.setPayloadSize(max(sizeof(data_final_send),
   sizeof(control_message))); // Размер пакета, в байтах
  radio.setChannel(CHANNEL_NUM);  // Выбираем канал (в котором нет шумов)
  radio.setPALevel(RF24_PA_MAX);  // Нровень мощности передатчика
  radio.setDataRate(RF24_1MBPS);

  radio.openWritingPipe(ADDRESS_WRITING);
  radio.openReadingPipe(1, ADDRESS_READING);

  radio.startListening();
  
  // Проверяем, всё ли впорядке с датчиками
  Serial.print("Тестируем связь с датчиком L3G4200D... ");
  if (gyro.testConnection()) {
    Serial.println("Всё ОК");
    }else{while(true){}}
   
  Serial.print("Тестируем связь с датчиком ADXL345... ");
  if (accel.testConnection()) {
    Serial.println("Всё ОК");
    }else{while(true){}}

  Serial.print("Тестируем связь с датчиком HMC5883L... ");
  if (magn.testConnection()) {
    Serial.println("Всё ОК");
    }else{while(true){}}
 
  Serial.println("\n");
}

//===============================

void loop() {
  start_working();
  
  // Если мы вышли из цикла radio_listening, то нужное сообщение мы приняли
  // При получении сообщения со станции
  check_for_rotation(true);  // true или false (каждая ось, или все оси)
}

//----------------------------- ФУНКЦИИ : -----------------------------

void start_working() {
  // Установка начального состояния устройства
  setting_initial_state_device();

  // Начало прослушивания радиоканала
  radio_listening();
}

//===============================

void setting_initial_state_device() { // Установка начального состояния устройства

  // Записываем данные с датчиков
  int g[3]; int a[3]; int m[3];

  accel.getAcceleration(&a[0], &a[1], &a[2]);
  for (byte j=0; j<3; j++) {acc[j] = a[j] / (float)sensitive_acc;}  // Настраиваем чувствительность

  gyro.getAngularVelocity(&g[0], &g[1], &g[2]);
  for (byte j=0; j<3; j++) {gyr[j] = g[j] / (float)sensitive_gyr;}  // Настраиваем чувствительность

  magn.getHeading(&m[0], &m[1], &m[2]);
  for (byte j=0; j<3; j++) {mag[j] = m[j] / (float)sensitive_mag;}  // Настраиваем чувствительность

  for (int i; i < 3; i++) {
    // Записываем данные с датчиков в offsets
    offsets[i] = a[i];
    offsets[i +3] = gyr[i];

    // Обнуляем все данные с датчиков
    gyr[i] = 0;
  }


  // Устанавливаем значения переменных по умолчанию
  for(byte i=0; i<sizeof(control_message); i++) { data_received[i] = ".";}  // После получения сообщение - обнуляем его, что бы потом снова принять
  for(byte i=0; i<sizeof(data_final_send)/sizeof(data_final_send[0]); i++)  { data_final_send[i] = 0.0;}
  for (unsigned int i=0; i < number_entries; i++) {moving_of_box[i] = 0;}
}

//===============================

void check_for_rotation(bool mode) { // При получении сообщения со станции
  /*
      Тут есть 2 режима работы: 

    1) Когда мы отслеживаем поворот на сколько-то градусов ОТДЕЛЬНО вдоль каждой оси
    (Т.е. когда мы поварачиваем куб ТОЛЬКО с грани на грань)
    Этот режим соответствует true

    2) Когда мы отслеживаем поворот на сколько-то градусов вдоль ЛЮБОЙ оси ВМЕСТЕ
    (Т.е. когда мы поварачиваем куб в ЛЮБУЮ СТОРОНУ (В ПРОСТРАНСТВЕ) на сколько-то градусов)
    Этот режим соответствует false
  */


  // Переход в режим вещания сообщений
  radio.stopListening();
  
  //==============

  unsigned int timer_frequency = millis();
  unsigned int num = 0;    // Для записи данных в каждую ячейку по каждой оси moving_of_box

  int data_gyro[3] = {0, 0, 0}; 
  
  // Записываем ИЗМЕНЕНИЕ поворота:
  int delta_data_gyro[3] = {0, 0, 0}; 
  
  bool critical_situation = false;    // Аппарат наклонён?

  // Обнуляем значения moving_of_box (на всякий случай)
  for (unsigned int i=0; i<number_entries; i++) {moving_of_box[i]=0;}

  while (!critical_situation) {
    // Отсчитываем время
    if (millis() - timer_frequency >= 1000 / frequency_data_gyro) {
      timer_frequency = millis(); 
      
      // Если достигли конца массива, продолжаем записывать уже с начала массива
      if (num >= (number_entries) -3) {
        num = 0;
        Serial.println(millis());
      } else {
        num += 3;
      }

      // Записываем данные по каждой оси в каждую ячейку
      gyro.getAngularVelocity(&data_gyro[0], &data_gyro[1], &data_gyro[2]);

      // Записываем ИЗМЕНЕНИЕ поворота
      for (int i=0; i < 3; i++) {
        delta_data_gyro[i] = abs(data_gyro[i]) - abs(delta_data_gyro[i]); // Наше значение минус предыдущее

        // Записываем МОДУЛЬ изменения
        moving_of_box[num +i] = abs(delta_data_gyro[i]);

        // Записываем новое значение, относительно которого измеряем изменение
        delta_data_gyro[i] = abs(data_gyro[i]);
      } 
      /////.///// 
      
      // Фильтруем данные
      // filter(1);

      // Записываем данные с акселерометра
      int a[3] = {0, 0, 0}; double summ_acc = 0.;
      accel.getAcceleration(&a[0], &a[1], &a[2]);

      // Проверяем, есть ли запредельное ускорение
      for (byte ind = 0; ind < 3; ind++) {
        if (mode) {summ_acc = .0;}

        // Смещение
        a[ind] -= offsets[ind];
        // Настраиваем чувствительность
        summ_acc += (float)a[ind] / (float)sensitive_acc;
        
        // Добавляем данные в массив для отправки
        data_final_send[ind +6] = summ_acc;

        Serial.print(summ_acc);Serial.print('\t');
        if (abs(summ_acc) >= threshold_acceleration) { // 1 единица измерений равна 1 g
          critical_situation = true; // Аппарат испытывает достаточное ускорение
        }
      }

      // Суммируем все значения
      double summ_gyr = 0.;
      for (int axes_sum_index=0; axes_sum_index < 3; axes_sum_index++) {   // По каждой оси
        if (mode) {
          summ_gyr = .0; // Обнуляем summ_gyr для каждой оси
        }

        for (uint16_t sum_index = axes_sum_index;
            sum_index < number_entries;
            sum_index = sum_index +3) {
            
          /* Складываем значения
            Сырые данные делим на чувствительность => получаем реальную угловую скорость
          -> Умножаем скорость на время, получаем "путь" (в °) */
          double a = ((double) ((double) moving_of_box[sum_index] / sensitive_gyr) / frequency_data_gyro);
          summ_gyr += abs(a);
          data_final_send[axes_sum_index] = summ_gyr;
        }

        // И проверяем, есть ли поворот на >threshold_turn°
        if (summ_gyr >= threshold_turn) { 
          critical_situation = true; // Аппарат повернулся достаточно!
        }
        Serial.print(summ_gyr); Serial.print('\t');
      }
      Serial.print('\n');
    }
  }


  /* Когда мы вышли из цикла (т.е. когда аппарат повернулся достаточно)
  —— отправляем сообщение с данными от датчиков*/

  read_data(); // Записываем показания датчиков и убираем смещение нуля
  send();

  Serial.println("Отправляемые данные: ");
  Serial.print("Gyro: ");
  for ( byte i=0; i < 3; i++) {
    Serial.print(data_final_send[i]);
    Serial.print(" ");
  } Serial.print("\t");  Serial.println(" °");
  Serial.print("Magn: ");
  for ( byte i=3; i < 6; i++) {
    Serial.print(data_final_send[i]);
    Serial.print(" ");
  } Serial.print("\n");
  Serial.print("Acc: ");
  for ( byte i=6; i < 9; i++) {
    Serial.print(data_final_send[i]);
    Serial.print(" ");
  } Serial.print("\t");  Serial.println(" g");
} // Главный алгоритм

//===============================

void radio_listening() { // Начало прослушивания радиоканала

  radio.powerUp();

  bool coincided = false;

  radio.powerUp(); radio.stopListening();  // Без этого оно
  radio.powerUp(); radio.startListening(); // не работает ¯＼_(ツ)_/¯

  while ( !coincided ) {
    get_data();
    show_data();
    
    // Совпало ли принятое сообщение с контрольным?
    bool non_coincided = false;
    for (int i=0; i < sizeof(control_message); i++) {
      if (data_received[i] == control_message[i]) {
        coincided = true;
      } else {
        non_coincided = true; // Почему то с break; не получилось сделать 
      }
    } if (non_coincided) {coincided = false;}  // Если хотя бы 1 символ не соответствует, то мы приняли не то
  }
  Serial.println("Принято!");
}

//===============================

void send() { // Отправка сообщения

  // Так надо
  radio.powerDown();
  radio.stopListening();
  radio.powerUp();

  bool rslt;
  rslt = radio.write( &data_final_send, sizeof(data_final_send) );

  if (rslt) {
    Serial.println("Подтверждение принято");
  } else {
    Serial.println("Не отправилось\n");
  }
}

//===============================

void get_data() { // Получение ответа

  radio.powerUp();
  if ( radio.available() ) {
    radio.read( &data_received, sizeof(data_received) );
  }
}

//===============================

void show_data() { // Отображение данных

  Serial.print("Принятые данные: ");

  for (byte i=0; i < sizeof(data_received); i++) {
    Serial.print(data_received[i]);
  }

  Serial.print("\n");
}

//===============================

void read_data() { // Записываем показания датчиков и убираем смещение нуля
  // Записываем данные с магнитометра
  int m[3];
  magn.getHeading(&m[0], &m[1], &m[2]);
  for (byte j=0; j<3; j++) {mag[j] = m[j] / (float)sensitive_mag;}  // Настраиваем чувствительность

  for (int i=0; i<3; i++) {    
    // Записываем
    data_final_send[i +3] = mag[i];
  }
}

//===============================

// Медианный фильтр, mean_range = диапазон усреднения, сколько соседей берём
void filter(int mean_range) {

  for (byte axis=0; axis<3; axis++){
    for (int i=3*mean_range+axis; i<number_entries-3*mean_range-3; i+=3) {
      // "Вырезаем" данные из окна (в wind)
      int wind[2*mean_range +1];
      for (uint16_t q=0; q<2*mean_range+1; q++){wind[q] = moving_of_box[i-3*(q+mean_range)];}

      // Сортируем wind Пузырьком
      for (byte _step_ = 0; _step_ < 2*mean_range +1; _step_++) {
        // Проходим по массиву и сравниваем соседние элементы
        for (byte j = 0; j < 2*mean_range; j++) {
          if (wind[j] > wind[j+1]) {
            // Если текущий элемент больше следующего, меняем их местами
            int temp = wind[j];
            wind[j] = wind[j+1];
            wind[j+1] = temp;
          }
        }
      }
      // Отсортировали, теперь фильтруем
      int median = wind[mean_range];
      moving_of_box[i] = median;
    }
  }
}
