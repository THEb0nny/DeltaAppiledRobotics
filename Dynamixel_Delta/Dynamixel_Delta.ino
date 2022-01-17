// https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/
// https://emanual.robotis.com/docs/en/parts/controller/opencm904/

// https://habr.com/ru/post/580970/
// https://habr.com/ru/post/583190/
// https://vk.com/id317251061

#include <Dynamixel2Arduino.h>  // Подключение библиотеки Dynamixel

#define DEBUG_SERIAL Serial // Установка константы, отвечающей за последовательный порт, подключаемый к компьютеру
#define DXL_SERIAL Serial3 // OpenCM9.04 EXP Board's DXL port Serial. (To use the DXL port on the OpenCM 9.04 board, you must use Serial1 for Serial. And because of the OpenCM 9.04 driver code, you must call Serial1.setDxlMode(true); before dxl.begin();.)

#define DXL_DIR_PIN 22 // Инициализация переменной, отвечащей за номер пина, подключенного к информационному пину приводов манипулятора
#define DXL_PROTOCOL_VERSION 1.0 // Инициализация переменной, отвечащей за протокол передачи данных от OpenCM9.04 к приводам
#define JOINT_N 3 // Количество приводов дельты
#define DYNAMIXEL_GOAL_POS_ERROR 5 // Погрешность позиции для динимикселей

#define EXP_BOARD_BUTTON1_PIN D16 // Пин кнопки 1 на плате расширения
#define EXP_BOARD_BUTTON2_PIN D17 // Пин кнопки 2 на плате расширения
#define EXP_BOARD_LED1_PIN D18 // Пин светодиода 1 на плате расширения
#define EXP_BOARD_LED2_PIN D19 // Пин светодиода 2 на плате расширения
#define EXP_BOARD_LED3_PIN D20 // Пин светодиода 3 на плате расширения

#define SOLENOID_RELAY_PIN D10 // Пин управления реле соленоида

// Тригонометрические константы
#define SQRT3 sqrtf(3.0)
#define COS120 cos(radians(120))
#define SIN120 sin(radians(120))
#define COS240 cos(radians(240))
#define SIN240 sin(radians(240))

// Для светодиодов на плате расширения, которые от земли
#define LED_HIGH LOW
#define LED_LOW HIGH

// Размеры робота в мм
#define F_BASE_PLATFORM_DELTA 158.483 // Длина стороны верхнего неподвижного основания
#define OQ F_BASE_PLATFORM_DELTA * SQRT3 / 6 // Радиус окружности осей шарниров

#define R_L_DELTA 100 // Длина рычага
#define R_R_DELTA 212 // Длина штанги

#define F_PLATFORM_DELTA 55.426 // Длина стороны подвижной платформы
#define VM F_PLATFORM_DELTA * SQRT3 / 6 // Радиус окружности осей рычагов

#define OFFSET_V_IN_PLATFORM_HEIGHT 10 // Размер высоты платформы для смещения точки V

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN); // Инициализация указателя на команды из библиотеки Dynamixel

void setup() {
  DEBUG_SERIAL.begin(57600); // Установка скорости обмена данными по последовательному порту компьютера
  pinMode(EXP_BOARD_BUTTON1_PIN, INPUT_PULLDOWN); // Установка режима кнопки 1 на плате расширения
  pinMode(EXP_BOARD_BUTTON2_PIN, INPUT_PULLDOWN); // Установка режима кнопки 2 на плате расширения
  pinMode(EXP_BOARD_LED1_PIN, OUTPUT); // Установка режима пина светодиода 1 на плате расширения
  pinMode(EXP_BOARD_LED2_PIN, OUTPUT); // Установка режима пина светодиода 2 на плате расширения
  pinMode(EXP_BOARD_LED3_PIN, OUTPUT); // Установка режима пина светодиода 3 на плате расширения
  pinMode(SOLENOID_RELAY_PIN, OUTPUT); // Управление реле с помощью соленоида
  PneumaticSuctionCupState(false); // Не захватывать при старте
  digitalWrite(EXP_BOARD_LED1_PIN, LED_LOW); // Выключаем светодиод 1 на плате расширения
  digitalWrite(EXP_BOARD_LED2_PIN, LED_LOW); // Выключаем светодиод 2 на плате расширения
  digitalWrite(EXP_BOARD_LED3_PIN, LED_LOW); // Выключаем светодиод 3 на плате расширения
  //
  /*while (1) {
    digitalWrite(SOLENOID_RELAY_PIN, LOW);
    delay(2000);
    digitalWrite(SOLENOID_RELAY_PIN, HIGH);
    delay(2000);
  }*/
  //
  //while(!DEBUG_SERIAL); // Ждём, пока монитор порта не откроется
  while(digitalRead(EXP_BOARD_BUTTON1_PIN) == 0); // Ждём, пока не будет нажата кнопка 1 на плате расширения
  DEBUG_SERIAL.println("Setup...");
  dxl.begin(1000000); // Установка скорости обмена данными по последовательному порту манипулятора
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION); // Выбор протокола обмена данными
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
    dxl.torqueOff(i); // Отключение блокировки привода, чтобы установить режим работы!
    bool setDinamixelOperationMode = dxl.setOperatingMode(i, OP_POSITION); // Установка режима работы привода в качестве шарнира
    if (!setDinamixelOperationMode) {
      DEBUG_SERIAL.print("Dynamixel wgith ID "); DEBUG_SERIAL.print(i); DEBUG_SERIAL.println("mode not set!");
    }
    delay(10);
  }
  DEBUG_SERIAL.println("Start..."); DEBUG_SERIAL.println();
  delay(500);
  // Занять среднюю позицию
  for (int i = 1; i <= JOINT_N; i++) {
    MoveMotorToGoal(i, 50, 410);
  }
  // Ждём, чтобы все приводы заняли позицию
  WaitMotorsTakeGoalPos(410, 410, 410);
  delay(500);
}

void loop() {
  float* motPos = new float[3];
  motPos = Delta_IK(0, 50, -180);
  DEBUG_SERIAL.print("NeedMotorPos: "); DEBUG_SERIAL.print(motPos[0]); DEBUG_SERIAL.print(", "); DEBUG_SERIAL.print(motPos[1]); DEBUG_SERIAL.print(", "); DEBUG_SERIAL.println(motPos[2]);
  motPos[0] = ConvertDegreesToGoalPos(motPos[0]);
  motPos[1] = ConvertDegreesToGoalPos(motPos[1]);
  motPos[2] = ConvertDegreesToGoalPos(motPos[2]);
  DEBUG_SERIAL.print("NeedGoalPos: "); DEBUG_SERIAL.print(motPos[0]); DEBUG_SERIAL.print(", "); DEBUG_SERIAL.print(motPos[1]); DEBUG_SERIAL.print(", "); DEBUG_SERIAL.println(motPos[2]);
  MoveMotorToGoal(1, 50, motPos[0]);
  MoveMotorToGoal(2, 50, motPos[1]);
  MoveMotorToGoal(3, 50, motPos[2]);
  WaitMotorsTakeGoalPos(motPos[0], motPos[1], motPos[2]);
  delay(500);
  DEBUG_SERIAL.println();

  motPos = Delta_IK(50, -30, -180);
  DEBUG_SERIAL.print("NeedMotorPos: "); DEBUG_SERIAL.print(motPos[0]); DEBUG_SERIAL.print(", "); DEBUG_SERIAL.print(motPos[1]); DEBUG_SERIAL.print(", "); DEBUG_SERIAL.println(motPos[2]);
  motPos[0] = ConvertDegreesToGoalPos(motPos[0]);
  motPos[1] = ConvertDegreesToGoalPos(motPos[1]);
  motPos[2] = ConvertDegreesToGoalPos(motPos[2]);
  DEBUG_SERIAL.print("NeedGoalPos: "); DEBUG_SERIAL.print(motPos[0]); DEBUG_SERIAL.print(", "); DEBUG_SERIAL.print(motPos[1]); DEBUG_SERIAL.print(", "); DEBUG_SERIAL.println(motPos[2]);
  MoveMotorToGoal(1, 50, motPos[0]);
  MoveMotorToGoal(2, 50, motPos[1]);
  MoveMotorToGoal(3, 50, motPos[2]);
  WaitMotorsTakeGoalPos(motPos[0], motPos[1], motPos[2]);
  delay(500);
  DEBUG_SERIAL.println();

  motPos = Delta_IK(-50, -30, -180);
  DEBUG_SERIAL.print("NeedMotorPos: "); DEBUG_SERIAL.print(motPos[0]); DEBUG_SERIAL.print(", "); DEBUG_SERIAL.print(motPos[1]); DEBUG_SERIAL.print(", "); DEBUG_SERIAL.println(motPos[2]);
  motPos[0] = ConvertDegreesToGoalPos(motPos[0]);
  motPos[1] = ConvertDegreesToGoalPos(motPos[1]);
  motPos[2] = ConvertDegreesToGoalPos(motPos[2]);
  DEBUG_SERIAL.print("NeedGoalPos: "); DEBUG_SERIAL.print(motPos[0]); DEBUG_SERIAL.print(", "); DEBUG_SERIAL.print(motPos[1]); DEBUG_SERIAL.print(", "); DEBUG_SERIAL.println(motPos[2]);
  MoveMotorToGoal(1, 50, motPos[0]);
  MoveMotorToGoal(2, 50, motPos[1]);
  MoveMotorToGoal(3, 50, motPos[2]);
  WaitMotorsTakeGoalPos(motPos[0], motPos[1], motPos[2]);
  delay(500);
  DEBUG_SERIAL.println();
}

int ConvertDegreesToGoalPos(float deg) {
  // 30° - мертвая зона диномикселя
  deg = constrain(deg, 30, 300); // Ограничиваем входное значение, где 30° - это начальный градус слева и 300°
  float goalPos = map(deg, 330, 30, 1023, 0);
  return goalPos;
}

void WaitMotorsTakeGoalPos(int posMotor1, int posMotor2, int posMotor3) { 
  while (true) {
    //DEBUG_SERIAL.print("Motors position: "); DEBUG_SERIAL.print(dxl.getPresentPosition(1)); DEBUG_SERIAL.print(", "); DEBUG_SERIAL.print(dxl.getPresentPosition(2)); DEBUG_SERIAL.print(", "); DEBUG_SERIAL.println(dxl.getPresentPosition(3));
    if ((posMotor1 - DYNAMIXEL_GOAL_POS_ERROR <= dxl.getPresentPosition(1) && dxl.getPresentPosition(1) <= posMotor1 + DYNAMIXEL_GOAL_POS_ERROR) && (posMotor2 - DYNAMIXEL_GOAL_POS_ERROR <= dxl.getPresentPosition(2) && dxl.getPresentPosition(2) <= posMotor2 + DYNAMIXEL_GOAL_POS_ERROR) && (posMotor3 - DYNAMIXEL_GOAL_POS_ERROR <= dxl.getPresentPosition(3) && dxl.getPresentPosition(3) <= posMotor3 + DYNAMIXEL_GOAL_POS_ERROR)) {
      break;
    }
    //delay(500);
  }
  /*while (!(dxl.getPresentPosition(1) == posMotor1 && dxl.getPresentPosition(2) == posMotor2 && dxl.getPresentPosition(3) == posMotor3)) {
    DEBUG_SERIAL.print("Motors position: "); DEBUG_SERIAL.print(dxl.getPresentPosition(1)); DEBUG_SERIAL.print(", "); DEBUG_SERIAL.print(dxl.getPresentPosition(2)); DEBUG_SERIAL.print(", "); DEBUG_SERIAL.println(dxl.getPresentPosition(3));
    delay(100);
  }*/
  DEBUG_SERIAL.print("Motors performed position: "); DEBUG_SERIAL.print(dxl.getPresentPosition(1)); DEBUG_SERIAL.print(", "); DEBUG_SERIAL.print(dxl.getPresentPosition(2)); DEBUG_SERIAL.print(", "); DEBUG_SERIAL.println(dxl.getPresentPosition(3));
}

void MoveMotorToGoal(int motorId, int speed, int goalPos) {
  dxl.setGoalVelocity(motorId, speed); // Задание целевой скорости
  dxl.setGoalPosition(motorId, goalPos); // Задание целевого положения
}

float* Delta_IK(float X_V, float Y_V, float Z_V) {
  Z_V += OFFSET_V_IN_PLATFORM_HEIGHT; // Учитываем высоту платформы
  // Расчёт координат точки V в системах координат, повёрнутых на 120° и 240° по часовой стрелке относительно основной
  float X_V_120 = X_V * COS120 - Y_V * SIN120;
  float Y_V_120 = X_V * SIN120 + Y_V * COS120;
  float X_V_240 = X_V * COS240 - Y_V * SIN240;
  float Y_V_240 = X_V * SIN240 + Y_V * COS240;
  float *ik_theta = new float[3];
  ik_theta[0] = Calc_Theta(X_V, Y_V, Z_V);
  ik_theta[1] = Calc_Theta(X_V_120, Y_V_120, Z_V);
  ik_theta[2] = Calc_Theta(X_V_240, Y_V_240, Z_V);
  return ik_theta;
}

float Calc_Theta(int X_V, int Y_V, int Z_V) {
  float y_M = -VM + Y_V;
  float y_Q = -OQ;
  // Первый метод вычисления
  /*
  float sigma = pow(R_L_DELTA, 2) - pow(R_R_DELTA, 2) + pow(X_V, 2) + pow(y_M, 2) - pow(y_Q, 2) + pow(Z_V, 2);
  float a = pow(2 * y_M - 2 * y_Q, 2) / (4 * pow(Z_V, 2)) + 1;
  float b = - 2 * y_Q - ((2 * y_M - 2 * y_Q) * sigma) / (2 * pow(Z_V, 2));
  float c = pow(sigma, 2) / (4 * pow(Z_V, 2)) - pow(R_L_DELTA, 2) + pow(y_Q, 2);
  float y_L = (-b - sqrt(pow(b, 2) - 4 * a * c)) / (2 * a);
  float z_L = (-2 * y_L * y_M + 2 * y_L * y_Q + sigma) / (2 * Z_V);
  float theta = 180 + degrees(atan((-z_L) / (y_Q - y_L)));
  */
  // Второй метод вычисления
  float NL = sqrtf(pow(R_R_DELTA, 2) - pow(X_V, 2));
  float const_1 = y_M - y_Q;
  float NQ = sqrtf(pow(const_1, 2) + pow(Z_V, 2));
  float theta = 360 - degrees(acos((pow(R_L_DELTA, 2) + pow(NQ, 2) - pow(NL, 2)) / (2 * R_L_DELTA * NQ))) - degrees(acos(const_1 / NQ));
  return theta;
}

float* Delta_FK(float theta1, int theta2, int theta3) {
  // Расчёт координат концов рычагов
  float x_L1 = 0;
  float y_L1 = -OQ - R_L_DELTA * cos(radians(theta1 - 180));
  float z_L1 = -R_L_DELTA * sin(radians(theta1 - 180));
  float z_L2 = -R_L_DELTA * sin(radians(theta2 - 180));
  float z_L3 = -R_L_DELTA * sin(radians(theta3 - 180));
  float y_L2S = -OQ - R_L_DELTA * cos(radians(theta2 - 180));
  float y_L3S = -OQ - R_L_DELTA * cos(radians(theta3 - 180));
  float x_L2 = y_L2S * sin(radians(120));
  float y_L2 = y_L2S * cos(radians(120));
  float x_L3 = y_L3S * sin(radians(240));
  float y_L3 = y_L2S * cos(radians(240));
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
  float z_V = ((-b_KU - sqrtf(pow(b_KU, 2) - 4 * a_KU * c_KU))) / (2 * a_KU);
  float x_V = e1 * z_V + f1;
  float y_V = e2 * z_V + f2;
  //float L1[3] = {x_L1, y_L1, z_L1};
  //float L2[3] = {x_L2, y_L2, z_L2};
  //float L3[3] = {x_L3, y_L3, z_L3};
  float *fk_V = new float[3];
  fk_V[0] = x_V, fk_V[1] = y_V, fk_V[2] = z_V;
  return fk_V;
}

// Функция для управления пневматическим захватом
void PneumaticSuctionCupState(bool isCapture) {
  if (isCapture) digitalWrite(SOLENOID_RELAY_PIN, HIGH);
  else digitalWrite(SOLENOID_RELAY_PIN, LOW);
}
