//----------------------------- ПЕРЕМЕННЫЕ -----------------------------

float acc[3];       // Хранение значений акселерометра (x, y, z)
float gyr[3];       // Хранение значений гироскопа (x, y, z)
float mag[3];       // Хранение значений магнитометра (x, y, z)

int offsets[6];   /* Помещяем сюда первоначальные значения датчиков по осям x, y, z
(сначала акселерометра, потом гироскопа)
Или же убираем смещение нуля (offset) */

float data_for_final_send[9]; // Данные для отправки

char data_received[26]; /* Массив для хранения данных, полученных с Станции
Должен совпадать с отправляемым массивом */
const char control_message[26] = "Activation control message";

const uint8_t frequency_data_of_gyro = 25; /* Как часто будем снимать показания с гироскопа, для определения движения коробки
(Сколько раз в секунду)*/

const uint8_t number_entries = 3 * 3 * frequency_data_of_gyro; /* Количество записей в массив moving_of_box
  3 = количество секунд записи, 3 = количество осей*/

int moving_of_box[number_entries]; // Куда записываем данные с коробки

const float sensitive_gyr = 19.47;    // Чувствительность гироскопа       (В данном случае единица значения == 1°/s )
const float sensitive_acc = 28.67;    // Чувствительность акселерометра   ¡В данном случае единица значения == 0.01g!
const float sensitive_mag = 6.4;      // Чувствительность магнитометра


//------------------------ ПОДКЛЮЧАЕМ ВСЁ ЧТО НУЖНО ПОДКЛЮЧИТЬ ------------------------
#include <Wire.h>
#include "I2Cdev.h"
#include "ADXL345.h"
#include "L3G4200D.h"
#include "BMP085.h"
#include "HMC5883L.h"

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

#define ADDRESS_WRITING 0xFEDCBA9876LL
#define ADDRESS_READING 0xAABBCCDD11LL
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
  radio.setAutoAck(1);            // Режим подтверждения приёма, 1 вкл 0 выкл
  radio.setRetries(0, 10);        // (время между попыткой достучаться, число попыток)
  radio.setPayloadSize(sizeof(control_message));  // Размер пакета, в байтах
  radio.setChannel(CHANNEL_NUM);  // Выбираем канал (в котором нет шумов)
  radio.setPALevel(RF24_PA_MAX);  // Нровень мощности передатчика
  radio.setDataRate(RF24_1MBPS);
  radio.openWritingPipe(ADDRESS_WRITING);
  radio.openReadingPipe(1, ADDRESS_READING);
  radio.startListening();

  // Проверяем, всё ли впорядке с датчиками
  Serial.print("Тестируем связь с датчиком L3G4200D... ");
  Serial.println(gyro.testConnection() ? "Всё ОК" : "Что-то не так");
  Serial.print("Тестируем связь с датчиком ADXL345... ");
  Serial.println(accel.testConnection() ? "Всё ОК" : "Что-то не так");
  Serial.print("Тестируем связь с датчиком HMC5883L... ");
  Serial.println(magn.testConnection() ? "Всё ОК" : "Что-то не так");
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
  for(byte i=0; i<9; i++)  { data_for_final_send[i] = 0.0;}
  for (unsigned int i=0; i < number_entries; i++) {moving_of_box[i] = 0;}
}

//===============================

void check_for_rotation(bool mode) { // При получении сообщения со станции
  /*
      Тут есть 2 режима работы: 

    1) Когда мы отслеживаем поворот на 80° ОТДЕЛЬНО вдоль каждой оси
    (Т.е. когда мы поварачиваем куб ТОЛЬКО с грани на грань)
    Этот режим соответствует true

    2) Когда мы отслеживаем поворот на 80° вдоль ЛЮБОЙ оси ВМЕСТЕ
    (Т.е. когда мы поварачиваем куб в ЛЮБУЮ СТОРОНУ (В ПРОСТРАНСТВЕ) на 80°)
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
  
  bool apparatus_tilted = false;    // Аппарат наклонён?

  int moving_of_box[number_entries];

  // Обнуляем значения moving_of_box (на всякий случай)
  for (unsigned int i=0; i<number_entries; i++) {moving_of_box[i]=0;}

  while (!apparatus_tilted) {
    // Отсчитываем время
    if (millis() - timer_frequency >= 1000 / frequency_data_of_gyro) {
      timer_frequency = millis(); 
      
      //..../// Если достигли конца массива, продолжаем записывать уже с начала массива
      if (num >= (number_entries) -3) {
        num = 0;
      } else {
        num += 3;
      }
      //....//

      // for (int i=0; i<3; i++){
      //   for (unsigned int j=i; j<number_entries; j+=3 ) {
      //     Serial.print((float)moving_of_box[j] / sensitive_gyr / frequency_data_of_gyro );
      //     Serial.print(" ");
      //     }
      //     Serial.print("\n");
      // } Serial.print("\n");

      /////.///// Записываем данные по каждой оси в каждую ячейку

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
      
      // Усредняем
      filter(1);

      // Записываем данные с акселерометра
      int a[3] = {0, 0, 0}; double summ_acc = 0.;
      accel.getAcceleration(&a[0], &a[1], &a[2]);

      // Проверяем, есть ли запредельное ускорение
      for (byte ind=0; ind<3; ind++) {
        if (mode) {summ_acc = .0;}

        // Смещение
        a[ind] -= offsets[ind];
        // Настраиваем чувствительность
        summ_acc += (float)a[ind] / (float)sensitive_acc;
        
        if (summ_acc >= 8) { // 1 единица измерений равна 1 g
          apparatus_tilted = true; // Аппарат испытывает достаточное ускорение
        } Serial.print(summ_acc);Serial.print('\t');
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
          double a = ((double) ((double) moving_of_box[sum_index] / sensitive_gyr) / frequency_data_of_gyro);
          summ_gyr += abs(a);
        }

        // И проверяем, есть ли поворот на >90°
        if (summ_gyr >= 90) { 
          apparatus_tilted = true; // Аппарат повернулся достаточно!
        }
        Serial.print(summ_gyr); Serial.print('\t');
      } Serial.print('\n');
    }
  }


  /* Когда мы вышли из цикла (т.е. когда аппарат повернулся достаточно)
  —— отправляем сообщение с данными от датчиков*/

  read_data(); // Записываем показания датчиков и убираем смещение нуля
  for (byte i=0; i<3; i++) {  // Отправляем 3 раза (с минимальным интервалом)
    send();

    Serial.println("Отправляемые данные: ");
    Serial.print("Gyro: ");
    for ( byte i=0; i < 3; i++) {
      Serial.print(data_for_final_send[i]);
      Serial.print(" ");
    } Serial.print("\t");  Serial.println(" °/s");
    Serial.print("Magn: ");
    for ( byte i=3; i < 6; i++) {
      Serial.print(data_for_final_send[i]);
      Serial.print(" ");
    } Serial.print("\t");  Serial.println(" °");
    Serial.print("Acc: ");
    for ( byte i=6; i < 9; i++) {
      Serial.print(data_for_final_send[i]);
      Serial.print(" ");
    } Serial.print("\t");  Serial.println(" g");
  }
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

  radio.powerUp();

  radio.stopListening();

  radio.powerUp();

  bool rslt;
  rslt = radio.write( &data_for_final_send, sizeof(data_for_final_send) * sizeof(data_for_final_send[0]) );

  radio.startListening();

  Serial.print("\t");
  if (rslt) {
    Serial.println("Подтверждение принято");
  } else {
    Serial.println("Не отправилось");
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
  
  // Записываем данные с датчиков
  int g[3]; int a[3]; int m[3];

  accel.getAcceleration(&a[0], &a[1], &a[2]);
  for (byte j=0; j<3; j++) {acc[j] = a[j] / (float)sensitive_acc;}  // Настраиваем чувствительность

  gyro.getAngularVelocity(&g[0], &g[1], &g[2]);
  for (byte j=0; j<3; j++) {gyr[j] = g[j] / (float)sensitive_gyr;}  // Настраиваем чувствительность

  magn.getHeading(&m[0], &m[1], &m[2]);
  for (byte j=0; j<3; j++) {mag[j] = m[j] / (float)sensitive_mag;}  // Настраиваем чувствительность

  for (int i=0; i<3; i++) {
    // Убираем смещение нуля
    acc[i] -= offsets[i +0]; // Измеряем в том числе статическое ускорение
    gyr[i] -= offsets[i +3]; 
    // mag[i] -= offsets[i +6];  Нету смещений
    
    // Записываем
    data_for_final_send[i +0] = gyr[i];
    data_for_final_send[i +3] = mag[i];
    data_for_final_send[i +6] = acc[i];
  }
}

//===============================

// Медианный фильтр, mean_range = диапазон усреднения, сколько соседей берём
void filter(int mean_range) {
  int new_list[number_entries];

  // Убираем краюшки
  for (int i=0; i<mean_range; i++) {new_list[i] = moving_of_box[i];}
  for (int i=number_entries-mean_range-1; i<number_entries; i++) {new_list[i] = moving_of_box[i];}
  
  for (int i=mean_range; i<number_entries-mean_range; i++) {
    int wind[2*mean_range +1];
    for (int q=i; q<i+2*mean_range +1; q++){wind[q] = moving_of_box[q];}

    // Сортируем Пузырьком

    for (int i = 0; i < 2*mean_range +1; i++) {
      // Проходим по массиву и сравниваем соседние элементы
      for (int j = 0; j < 2*mean_range; j++) {
        if (wind[j] > wind[j+1]) {
          // Если текущий элемент больше следующего, меняем их местами
          int temp = wind[j];
          wind[j] = wind[j+1];
          wind[j+1] = temp;
          free(temp);
        }
      }
    }

  }
  *moving_of_box = *new_list;
}
