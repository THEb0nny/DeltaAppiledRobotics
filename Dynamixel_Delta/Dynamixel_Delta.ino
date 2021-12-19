// https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/
// https://habr.com/ru/post/390281/

// https://habr.com/ru/post/580970/
// https://habr.com/ru/post/583190/

#include <Dynamixel2Arduino.h>  // Подключение библиотеки Dynamixel

#define DEBUG_SERIAL Serial // Установка константы, отвечающей за последовательный порт, подключаемый к компьютеру
#define DXL_SERIAL Serial3 // OpenCM9.04 EXP Board's DXL port Serial. (To use the DXL port on the OpenCM 9.04 board, you must use Serial1 for Serial. And because of the OpenCM 9.04 driver code, you must call Serial1.setDxlMode(true); before dxl.begin();.)

#define DXL_DIR_PIN 22 // Инициализация переменной, отвечащей за номер пина, подключенного к информационному пину приводов манипулятора
#define DXL_PROTOCOL_VERSION 1.0 // Инициализация переменной, отвечащей за протокол передачи данных от OpenCM9.04 к приводам
#define JOINT_N 3 // Количество приводов

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN); // Инициализация указателя на команды из библиотеки Dynamixel

// Тригонометрические константы
#define SQRT3 sqrt(3.0)
#define SIN120 sin(radians(120));
#define SIN240 sin(radians(240));
#define COS120 cos(radians(120));
#define COS240 cos(radians(240));

// Размеры робота в мм
#define F_BASE_PLATFORM_DELTA 159.349 // Длина стороны верхнего неподвижного основания
#define OQ F_BASE_PLATFORM_DELTA * SQRT3 / 6 // Радиус окружности осей шарниров

#define R_E_DELTA 212 // Длина рычага
#define R_F_DELTA 104.495 // Длина верхнего плеча рычага

#define R_L_DELTA 212 // Длина рычага
#define R_R_DELTA 104.495 // Длина штанги

#define F_PLATFORM_DELTA 91.8 // Длина стороны подвижной платформы
#define VM F_PLATFORM_DELTA * SQRT3 / 6 // Радиус окружности осей рычагов

void setup() {
  Serial.print(radians(45));
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

int* Delta_IK(int X_V, int Y_V, int Z_V) {
  // Расчёт координат точки V в системах координат, повёрнутых на 120° и 240° по часовой стрелке относительно основной
  float X_V_120 = X_V * COS120 - Y_V * SIN120;
  float Y_V_120 = X_V * SIN120 + Y_V * COS120;
  float X_V_240 = X_V * COS240 - Y_V * SIN240;
  float Y_V_240 = X_V * SIN240 + Y_V * COS240;

  float theta1 = Calc_Theta(X_V, Y_V, Z_V);
  float theta2 = Calc_Theta(X_V_120, Y_V_120, Z_V);
  float theta3 = Calc_Theta(X_V_240, Y_V_240, Z_V);

  int *ik_theta = new int[3];
  ik_theta[1] = theta1;
  ik_theta[2] = theta2;
  ik_theta[3] = theta3;
  return ik_theta;
}

float Calc_Theta(int X_V, int Y_V, int Z_V) {
  float y_M = -VM + Y_V;
  float y_Q = -OQ;
  // Первый метод вычисления
  float sigma = pow(R_L_DELTA, 2) - pow(R_R_DELTA, 2) + pow(X_V, 2) + pow(y_M, 2) - pow(y_Q, 2) + pow(Z_V, 2);
  float a = pow(2 * y_M - 2 * y_Q, 2) / (4 * pow(Z_V, 2)) + 1;
  float b = - 2 * y_Q - ((2 * y_M - 2 * y_Q) * sigma) / (2 * pow(Z_V, 2));
  float c = pow(sigma, 2) / (4 * pow(Z_V, 2)) - pow(R_L_DELTA, 2) + pow(y_Q, 2);
  float y_L = (-b - sqrt(pow(b, 2) - 4 * a * c)) / (2 * a);
  float z_L = (-2 * y_L * y_M + 2 * y_L * y_Q + sigma) / (2 * Z_V);
  float theta = 180 + atan(radians((-z_L) / (y_Q - y_L)));
  // Второй метод вычисления
  //float NL = sqrt(pow(R_R_DELTA, 2) - pow(X_V, 2));
  //float const_1 = y_M - y_Q;
  //float NQ = sqrt(pow(const_1, 2) + pow(Z_V, 2));
  //float theta = 360 - acos(radians((pow(R_L_DELTA, 2) + pow(NQ, 2) - pow(NL, 2)) / (2 * R_L_DELTA * NQ))) - acos(radians(const_1 / NQ));
  return theta;
}

float* Delta_FK(float theta1, int theta2, int theta3) {
  // Расчёт координат концов рычагов
  float x_L1 = 0;
  float y_L1 = -OQ - R_L_DELTA * cos(radians(theta1 - 180));
  float z_L1 = -R_L_DELTA * sin(radians(theta2 - 180));
  float z_L2 = -R_L_DELTA * sin(radians(theta2 - 180));
  float z_L3 = -R_L_DELTA * sin(radians(theta3 - 180));
  float y_L2S = -OQ - R_L_DELTA * cos(radians(theta2 - 180));
  float y_L3S = -OQ - R_L_DELTA * cos(radians(theta3 - 180));
  float x_L2 = y_L2S * sin(radians(120));
  float y_L2 = y_L2S * cos(radians(120));
  float x_L3 = y_L3S * sin(radians(120));
  float y_L3 = y_L2S * cos(radians(120));
  // Расчёт координат центров сфер (сдвинутых концов рычагов)
  float x_P1 = x_L1;
  float y_P1 = y_L1 + VM;
  float z_P1 = z_L1;
  float x_P2 = x_L2 + VM * cos(radians(30));
  float y_P2 = y_L2 - VM * sin(radians(30));
  float z_P2 = z_L2;
  float x_P3 = x_L3 - VM * cos(radians(30));
  float y_P3 = y_L3 - VM * sin(radians(30));
  float z_P3 = z_L3;
  // Расчёт точки пересечения сфер
  float w1 = pow(R_R_DELTA, 2) - pow(x_P1, 2) - pow(y_P1, 2) - pow(z_P1, 2);
  float w2 = pow(R_R_DELTA, 2) - pow(x_P2, 2) - pow(y_P2, 2) - pow(z_P2, 2);
  float w3 = pow(R_R_DELTA, 2) - pow(x_P3, 2) - pow(y_P3, 2) - pow(z_P3, 2);
  float a1 = x_P2 - x_P1;
  float a2 = x_P3 - x_P1;
  float b1 = y_P2 - y_P1;
  float b2 = y_P3 - y_P1;
  float c1 = z_P2 - z_P1;
  float c2 = z_P3 - z_P1;
  float d1 = (w1 - w2) / 2;
  float d2 = (w1 - w3) / 2;
  float e1 = (b1 * c2 - b2 * c1) / (a1 * b2 - a2 * b1);
  float f1 = -(b1 * d2 - b2 * d1) / (a1 * b2 - a2 * b1);
  float e2 = -(a1 * c2 - a2 * c1) / (a1 * b2 - a2 * b1);
  float f2 = (a1 * d2 - a2 * d1) / (a1 * b2 - a2 * b1);
  float a_KU = pow(e1, 2) + pow(e2, 2) + 1;
  float b_KU = 2 * e1 * (f1 - x_P1) - 2 * z_P1 + 2 * e2 * (f2 - y_P1);
  float c_KU = pow(z_P1, 2) + pow(f1 - x_P1, 2) + pow(f2 - y_P1, 2) - pow(R_R_DELTA, 2);
  float z_V = ((-b_KU - sqrt(pow(b_KU, 2) - 4 * a_KU * c_KU))) / (2 * a_KU);
  float x_V = e1 * z_V + f1;
  float y_V = e2 * z_V + f2;
  //float L1[3] = {x_L1, y_L1, z_L1};
  //float L2[3] = {x_L2, y_L2, z_L2};
  //float L3[3] = {x_L3, y_L3, z_L3};
  float V[3] = {x_V, y_V, z_V}; 
}
