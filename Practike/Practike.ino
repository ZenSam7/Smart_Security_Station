// ПРАКТИКА ПЕРЕД ОЛИМПИАДОЙ

#include <Servo.h>
Servo servo;

#define servo_pin 2
#define uk_pin A0


int angle; unsigned int l;
void setup() {
  Serial.begin(9600);
  pinMode(uk_pin, INPUT); 
  servo.attach(servo_pin);

  pinMode(13, OUTPUT);
}

void loop() {
  l = analogRead(uk_pin);
  float lenght = 32 * pow((5.0 / 1024.0) * l, -1.10);

  // Уменьшаем угол, если обнаружили объект
  lenght > 20 ? angle++ : angle--;
  angle = clamp(angle, 0, 180);
  servo.write(angle);

  Serial.print(lenght); Serial.print("\t\t"); Serial.println(angle);
}

// Двигаем серво на нужный угол
int _angle_servo_now;
void servo_move(Servo servo, int angle, uint8_t speed) {
  if (_angle_servo_now < angle) {
    for (int i=_angle_servo_now; i<angle; i++) {
      servo.write(i);
      delay(speed);
    }
  } else {
    for (int i=_angle_servo_now; i>angle; i--) {
      servo.write(i);
      delay(speed);
    }
  }
  _angle_servo_now = angle;
}


// Идём прямо (ограничивая мощьность ШИМом)
void go_straight(uint8_t left_motor_vcc, uint8_t right_motor_vcc,
                 uint64_t time, uint8_t precent_shim) {
  uint64_t timer = millis();
  while ((timer + time) > millis()) {
    ShIM(left_motor_vcc, precent_shim);
    ShIM(right_motor_vcc, precent_shim);
  }
}


// Ограничиваем var снизу и сверху
int clamp(int var, int buttom, int upper) {
  if (var > upper) {var = upper;}
  else if (var < buttom) {var = buttom;}
  return var;
}


uint8_t _shim;
// precent - процент заполнения ШИМ от 0 до 100
void ShIM(uint8_t pin, uint8_t precent) {
  if (_shim < precent) {
      digitalWrite(pin, 1);
  } else {
    digitalWrite(pin, 0);
  }
  _shim++;
  _shim = _shim%100;
}
