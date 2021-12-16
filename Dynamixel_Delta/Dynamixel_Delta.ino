// https://habr.com/ru/post/390281/
// https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/

#include <Dynamixel2Arduino.h>  // Подключение библиотеки Dynamixel

#define DEBUG_SERIAL Serial // Установка константы, отвечающей за последовательный порт, подключаемый к компьютеру
#define DXL_SERIAL Serial3 // OpenCM9.04 EXP Board's DXL port Serial. (To use the DXL port on the OpenCM 9.04 board, you must use Serial1 for Serial. And because of the OpenCM 9.04 driver code, you must call Serial1.setDxlMode(true); before dxl.begin();.)

#define DXL_DIR_PIN 22 // Инициализация переменной, отвечащей за номер пина, подключенного к информационному пину приводов манипулятора
#define DXL_PROTOCOL_VERSION 1.0 // Инициализация переменной, отвечащей за протокол передачи данных от OpenCM9.04 к приводам
#define JOINT_N 3 // Количество приводов

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN); // Инициализация указателя на команды из библиотеки Dynamixel

// Размеры робота
#define E_DELTA 91.9 // Сторона подвижной платформы
#define F_DELTA 159.349 // Сторона неподвижного основания
#define RE_DELTA 212 // Длина нижнего плеча рычага
#define RF_DELTA 104.495 // Длина верхнего плеча рычага

// Тригонометрические константы
#define SQRT3 sqrt(3.0)
#define SIN120 SQRT3 / 2.0
#define COS120 -0.5
#define TAN60 SQRT3
#define SIN30 0.5
#define TAN30 1 / SQRT3

void setup() {
  DEBUG_SERIAL.begin(57600); // Установка скорости обмена данными по последовательному порту компьютера
  while(!DEBUG_SERIAL); // Ждём, пока монитор порта не откроется
  DEBUG_SERIAL.println("Setup...");
  dxl.begin(1000000); // Установка скорости обмена данными по последовательному порту манипулятора
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION); // Выбор протокола обмена данными
  delay(1000); // Ждём, чтобы моторы успели подключиться
  for (int i = 1; i <= JOINT_N; i++) { // Цикл для перебора всех приводов
    while (true) {
      if(dxl.ping(i) == true) { // Проверка отвечает ли мотор
        DEBUG_SERIAL.print("Dynamixel with ID "); DEBUG_SERIAL.print(i); DEBUG_SERIAL.print(" found, model "); DEBUG_SERIAL.print(dxl.getModelNumber(i)); DEBUG_SERIAL.println(".");
        break;
      } else {
        DEBUG_SERIAL.print("Dynamixel with ID "); DEBUG_SERIAL.print(i); DEBUG_SERIAL.print(" not found!"); DEBUG_SERIAL.println(" Wait...");
        delay(500);
      }
    }
    dxl.torqueOff(i); // Отключение блокировки привода
    dxl.setOperatingMode(i, OP_POSITION); // Установка режима работы привода в качестве шарнира
    delay(10);
  }
  DEBUG_SERIAL.println("Start...");
  delay(100);
  // Занять среднюю позицию
  for (int i = 1; i <= JOINT_N; i++) {
    MoveMotor(i, 50, 410);
  }
  // Ждём, чтобы все приводы заняли позицию
  WaitMotorsTakeGoalPosition(410, 410, 410);
  delay(1000);
  int* motPos = new int[4];
  motPos = Delta_IK(0, 0, 0);
  DEBUG_SERIAL.println(motPos[0]);
}

void loop() {
  for (int i = 1; i <= JOINT_N; i++) {
    MoveMotor(i, 50, 410);
  }
  // Ждём, чтобы все приводы заняли позицию
  WaitMotorsTakeGoalPosition(410, 410, 410);

  for (int i = 1; i <= JOINT_N; i++) {
    MoveMotor(i, 50, 512);
  }
  // Ждём, чтобы все приводы заняли позицию
  WaitMotorsTakeGoalPosition(512, 512, 512);
}

void WaitMotorsTakeGoalPosition(int posMotor1, int posMotor2, int posMotor3) {
  while (dxl.getPresentPosition(1) != posMotor1 && dxl.getPresentPosition(2) != posMotor2 && dxl.getPresentPosition(3) != posMotor3) { delay(5); }
}

void MoveMotor(int motorId, int speed, int goalPos) {
  dxl.setGoalVelocity(motorId, speed); // Задание целевой скорости
  dxl.setGoalPosition(motorId, goalPos); // Задание целевого положения
}

// Прямая кинематика: (theta1, theta2, theta3) -> (x0, y0, z0)
// Возвращаемый статус: 0 = OK, -1 = несуществующая позиция
int* Delta_FK(float theta1, float theta2, float theta3, float &x0, float &y0, float &z0) {
  int status = 0;
  
  float t = (F_DELTA - E_DELTA) * TAN30 / 2;
  float dtr = PI / (float)180.0;

  theta1 *= dtr;
  theta2 *= dtr;
  theta3 *= dtr;

  float y1 = -(t + RF_DELTA * cos(theta1));
  float z1 = -RF_DELTA * sin(theta1);

  float y2 = (t + RF_DELTA * cos(theta2)) * SIN30;
  float x2 = y2 * TAN60;
  float z2 = -RF_DELTA * sin(theta2);

  float y3 = (t + RF_DELTA * cos(theta3)) * SIN30;
  float x3 = -y3 * TAN60;
  float z3 = -RF_DELTA * sin(theta3);

  float dnm = (y2 - y1) * x3 - (y3 - y1) * x2;

  float w1 = y1 * y1 + z1 * z1;
  float w2 = x2 * x2 + y2 * y2 + z2*z2;
  float w3 = x3 * x3 + y3 * y3 + z3 * z3;

  // x = (a1*z + b1) / dnm
  float a1 = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1);
  float b1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) / 2.0;

  // y = (a2*z + b2) / dnm;
  float a2 = -(z2 - z1) * x3 + (z3 - z1) * x2;
  float b2 = ((w2 - w1) * x3 - (w3 - w1) * x2) / 2.0;

  // a*z^2 + b*z + c = 0
  float a = a1 * a1 + a2 * a2 + dnm * dnm;
  float b = 2 * (a1 * b1 + a2 * (b2 - y1 * dnm) - z1 * dnm * dnm);
  float c = (b2 - y1 * dnm) * (b2 - y1 * dnm) + b1 * b1 + dnm * dnm * (z1 * z1 - RE_DELTA * RE_DELTA);

  // Дискриминант
  float d = pow(b, 2) - (float)4.0 * a * c;
  if (d < 0) {
    //return -1; // Несуществующая позиция
    status = -1;
  }

  z0 = -(float)0.5 * (b + sqrt(d)) / a;
  x0 = (a1 * z0 + b1) / dnm;
  y0 = (a2 * z0 + b2) / dnm;
  
  int *return_array = new int[4];
  return_array[0] = 0; // Ошибки нет
  return_array[1] = x0;
  return_array[2] = y0;
  return_array[3] = z0;
  return return_array;
}

// Вспомогательная функция обратной кинематики, расчет угла theta1 (в плоскости YZ)
int Delta_calcAngleYZ(float x0, float y0, float z0, float &theta) {
    float y1 = -0.5 * 0.57735 * F_DELTA; // f/2 * tg 30
    y0 -= 0.5 * 0.57735 * E_DELTA; // Сдвигаем центр к краю
    // z = a + b * y
    float a = (x0 * x0 + y0 * y0 + z0 * z0 + RF_DELTA * RF_DELTA - RE_DELTA * RE_DELTA - y1 * y1) / (2 * z0);
    float b = (y1 - y0) / z0;
    // Дискриминант
    float d = -(a + b * y1) * (a + b * y1) + RF_DELTA * (b * b * RF_DELTA + RF_DELTA);
    if (d < 0) return -1; // Несуществующая точка
    float yj = (y1 - a * b - sqrt(d)) / (b * b + 1); //Выбираем внешнюю точку
    float zj = a + b * yj;
    theta = 180.0 * atan(-zj / (y1 - yj)) / PI + ((yj > y1) ? 180.0 : 0.0);
    return 0;
}

// Обратная кинематика: (x0, y0, z0) -> (theta1, theta2, theta3)
// Возвращаемый статус: 0 = OK, -1 = несуществующая позиция
int* Delta_IK(float x0, float y0, float z0) { // Delta_IK(float x0, float y0, float z0, float &theta1, float &theta2, float &theta3)
    float theta1, theta2, theta3 = 0;
    int status = Delta_calcAngleYZ(x0, y0, z0, theta1);
    if (status == 0) status = Delta_calcAngleYZ(x0 * COS120 + y0 * SIN120, y0 * COS120 - x0 * SIN120, z0, theta2); // Rotate coords to +120 deg
    if (status == 0) status = Delta_calcAngleYZ(x0 * COS120 - y0 * SIN120, y0 * COS120 + x0 * SIN120, z0, theta3); // Rotate coords to -120 deg
    int *return_array = new int[4];
    return_array[0] = status;
    //return_array[1] = x;
    //return_array[2] = y;
    //return_array[3] = z;
    return return_array;
}
