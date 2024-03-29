// https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/
// https://emanual.robotis.com/docs/en/parts/controller/opencm904/
// https://emanual.robotis.com/docs/en/popup/arduino_api/readControlTableItem/

// https://habr.com/ru/post/580970/
// https://habr.com/ru/post/583190/
// https://vk.com/id317251061

#include <Dynamixel2Arduino.h>  // Подключение библиотеки Dynamixel
#include "GyverTimer.h"

#define DEBUG_LEVEL 2 // Уровень дебага

#define MAX_INPUT_VAL_IN_MANUAL_CONTROL 4 // Максимальное количество значений введёных в Serial 

#define DEBUG_SERIAL Serial // Установка константы, отвечающей за последовательный порт, подключаемый к компьютеру
#define DXL_SERIAL Serial3 // OpenCM9.04 EXP Board's DXL port Serial. (To use the DXL port on the OpenCM 9.04 board, you must use Serial1 for Serial. And because of the OpenCM 9.04 driver code, you must call Serial1.setDxlMode(true); before dxl.begin();.)

#define DXL_DIR_PIN 22 // Инициализация переменной, отвечащей за номер пина, подключенного к информационному пину приводов манипулятора
#define DXL_PROTOCOL_VERSION 1.0 // Инициализация переменной, отвечащей за протокол передачи данных от OpenCM9.04 к приводам

#define JOINT_N 3 // Количество приводов дельты
#define DYNAMIXEL_GOAL_POS_ERROR 3 // Погрешность позиции для динимикселей
#define MAX_TIME_PERFORMED_POS 3500 // Максимальное время для занятия ошибка, защита

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
#define F_PLATFORM_DELTA 62.354 // Длина стороны подвижной платформы
#define VM F_PLATFORM_DELTA * SQRT3 / 6 // Радиус окружности осей рычагов
#define OFFSET_V_IN_PLATFORM_HEIGHT 10 // Размер высоты платформы для смещения точки V
#define MIN_Z -123 // Минимальная высота работы
#define MAX_Z -165 // Максимальная высота работы

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN); // Инициализация указателя на команды из библиотеки Dynamixel
GTimer servosWorksMaxTimeTimer(MS); // Инициализация таймера защиты максимального времени цикла ожидания занятия позиции сервопривода
GTimer serialPrintTimer(MS);

using namespace ControlTableItem; // Пространство имён для обращения к параметрам диномикселей

byte workMode = 1; // Режим управления

void setup() {
  DEBUG_SERIAL.begin(57600); // Установка скорости обмена данными по последовательному порту компьютера
  PneumaticSuctionCupState(false, 0); // Не захватывать при старте
  pinMode(EXP_BOARD_BUTTON1_PIN, INPUT_PULLDOWN); // Установка режима кнопки 1 на плате расширения
  pinMode(EXP_BOARD_BUTTON2_PIN, INPUT_PULLDOWN); // Установка режима кнопки 2 на плате расширения
  pinMode(EXP_BOARD_LED1_PIN, OUTPUT); // Установка режима пина светодиода 1 на плате расширения
  pinMode(EXP_BOARD_LED2_PIN, OUTPUT); // Установка режима пина светодиода 2 на плате расширения
  pinMode(EXP_BOARD_LED3_PIN, OUTPUT); // Установка режима пина светодиода 3 на плате расширения
  pinMode(SOLENOID_RELAY_PIN, OUTPUT); // Управление реле с помощью соленоида
  digitalWrite(EXP_BOARD_LED1_PIN, LED_LOW); // Выключаем светодиод 1 на плате расширения
  digitalWrite(EXP_BOARD_LED2_PIN, LED_LOW); // Выключаем светодиод 2 на плате расширения
  digitalWrite(EXP_BOARD_LED3_PIN, LED_LOW); // Выключаем светодиод 3 на плате расширения
  dxl.begin(1000000); // Установка скорости обмена данными по последовательному порту манипулятора
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION); // Выбор протокола обмена данными
  DEBUG_SERIAL.println("Wait press Btn1 or Btn2...");
  while(true) {
    if (digitalRead(EXP_BOARD_BUTTON1_PIN) == 1) { // Автоматический режим демонтрации, кнопка 1 на плате расширения
      workMode = 1;
      break;
    }
    if (digitalRead(EXP_BOARD_BUTTON2_PIN) == 1) { // Режим управления, Кнопка 2 на плате расширения
      workMode = 2;
      break;
    }
  }
  DEBUG_SERIAL.println("Setup...");
  serialPrintTimer.setInterval(500); // Установить таймер печати
  serialPrintTimer.reset(); // Спросить таймер печати
  for (byte i = 0; i < JOINT_N; i++) { // Цикл для перебора всех приводов
    while (true) {
      if(dxl.ping(i + 1) == true) { // Проверка отвечает ли мотор
        DEBUG_SERIAL.print("Dynamixel with ID "); DEBUG_SERIAL.print(i + 1); DEBUG_SERIAL.print(" found, model "); DEBUG_SERIAL.print(dxl.getModelNumber(i + 1)); DEBUG_SERIAL.println(".");
        break;
      } else {
        if (serialPrintTimer.isReady()) {
          DEBUG_SERIAL.print("Dynamixel with ID "); DEBUG_SERIAL.print(i + 1); DEBUG_SERIAL.print(" not found!"); DEBUG_SERIAL.println(" Wait...");
        }
        delay(10);
      }
    }
    while (true) { // Установить режим работы
      dxl.torqueOff(i + 1); // Отключение крутящего момента, чтобы установить режим работы!
      bool setDinamixelOperationMode = dxl.setOperatingMode(i + 1, OP_POSITION); // Установка режима работы привода в качестве шарнира
      if (!setDinamixelOperationMode && serialPrintTimer.isReady()) { // Если режим мотора не установился
        DEBUG_SERIAL.print("Dynamixel with ID "); DEBUG_SERIAL.print(i + 1); DEBUG_SERIAL.println(" mode not set!");
      } else break; // Режим установился, можно выйти из цикла
      delay(10);
    }
    dxl.torqueOn(i + 1); // Включение крутящего момента
  }
  DEBUG_SERIAL.print("Start... Work mode is "); DEBUG_SERIAL.println(workMode);
  SetAllServosSpeed(50); // Установить всем сервоприводам скорость
  DeltaMoveToPos(0, 0, MIN_Z, true); // Занять начальную позицию
}

void loop() {
  if (workMode == 1) {
    bool suctionCupState = false; // Состояние для автоматического режима демонстрации
    while (true) {
      //// 1 pos
      DeltaMoveToPos(0, 45, -125, true); // Занять позицию сверху
      DeltaMoveToPos(0, 45, -140, true); // Занять позицию, опустившись
      if (!suctionCupState) PneumaticSuctionCupState(true, 500); // Захватить, если присоска не работала
      else PneumaticSuctionCupState(false, 500); // Отпустить, если присоска до этого захватила
      suctionCupState = !suctionCupState; // Поменять состояние пневматического захвата
      DeltaMoveToPos(0, 45, -125, true); // Занять позицию сверху
      delay(200);
  
      //// 2 pos
      DeltaMoveToPos(-45, -45, -125, true); // Занять позицию сверху
      DeltaMoveToPos(-45, -45, -140, true); // Занять позицию, опустившись
      if (!suctionCupState) PneumaticSuctionCupState(true, 500); // Захватить, если присоска не работала
      else PneumaticSuctionCupState(false, 500); // Отпустить, если присоска до этого захватила
      suctionCupState = !suctionCupState; // Поменять состояние пневматического захвата
      DeltaMoveToPos(-45, -45, -125, true); // Занять позицию сверху
      delay(200);
  
      //// 3 pos
      DeltaMoveToPos(45, -45, -125, true); // Занять позицию сверху
      DeltaMoveToPos(45, -45, -140, true); // Занять позицию, опустившись
      if (!suctionCupState) PneumaticSuctionCupState(true, 500); // Захватить, если присоска не работала
      else PneumaticSuctionCupState(false, 500); // Отпустить, если присоска до этого захватила
      suctionCupState = !suctionCupState; // Поменять состояние пневматического захвата
      DeltaMoveToPos(45, -45, -125, true); // Занять начальную позицию
      delay(200);
    }
  } else if (workMode == 2) {
    ManualControl();
  }
}

// Команда на перемещение
void DeltaMoveToPos(float x, float y, float z, bool waitPerformedPos) {
  //x = constrain(x, -300, 300); // Ограничения по x
  //y = constrain(y, -300, 300); // Ограничения по y
  z = constrain(z, MAX_Z, MIN_Z); // Ограничения по z
  float* servosDegPos = Delta_IK(x, y, z);
  float* servosGoalPos = new float[3];
  if (DEBUG_LEVEL >= 1) {
    DEBUG_SERIAL.print("Target servo mot deg: ");
    for (byte i = 0; i < JOINT_N; i++) {
      DEBUG_SERIAL.print(servosDegPos[i]);
      if (i < JOINT_N - 1) DEBUG_SERIAL.print(", ");
      else DEBUG_SERIAL.println();
    }
  }
  if (DEBUG_LEVEL >= 1) DEBUG_SERIAL.print("Target servo mot pos: ");
  for (byte i = 0; i < JOINT_N; i++) {
    servosGoalPos[i] = ConvertDegreesToGoalPos(servosDegPos[i]);
    if (DEBUG_LEVEL >= 1) {
      DEBUG_SERIAL.print(servosGoalPos[i]);
      if (i < JOINT_N - 1) DEBUG_SERIAL.print(", ");
      else DEBUG_SERIAL.println();
    }
  }
  MoveServosToPos(servosGoalPos, waitPerformedPos);
  // Удаляем массивы для очистки памяти
  delete[] servosDegPos;
  delete[] servosGoalPos;
}

// Расчёт обратной кинематики
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

// Вспомогательнафя функция расчёта углов сервоприводов обратной кинематики
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

// Расчёт прямой кинематики
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
void PneumaticSuctionCupState(bool isCapture, int delayTime) {
  if (isCapture) digitalWrite(SOLENOID_RELAY_PIN, HIGH);
  else digitalWrite(SOLENOID_RELAY_PIN, LOW);
  delay(delayTime);
}

void ManualControl() {
  int x = 0, y = 0, z = MIN_Z;
  int xOld = x, yOld = y, zOld = z;
  bool control = true;
  DEBUG_SERIAL.print("x: "); DEBUG_SERIAL.print(x); DEBUG_SERIAL.print(" y: "); DEBUG_SERIAL.print(y); DEBUG_SERIAL.print(" z: "); DEBUG_SERIAL.println(z);
  while (control) {
    String inputValues[MAX_INPUT_VAL_IN_MANUAL_CONTROL]; // Массив входящей строки
    String key[MAX_INPUT_VAL_IN_MANUAL_CONTROL]; // Массив ключей
    int values[MAX_INPUT_VAL_IN_MANUAL_CONTROL]; // Массив значений
    if (Serial.available() > 2) { // Если есть доступные данные
      // Встроенная функция readStringUntil будет читать все данные, пришедшие в UART до специального символа — '\n' (перенос строки).
      // Он появляется в паре с '\r' (возврат каретки) при передаче данных функцией Serial.println().
      // Эти символы удобно передавать для разделения команд, но не очень удобно обрабатывать. Удаляем их функцией trim().
      String inputStr = Serial.readStringUntil('\n');
      inputStr.trim(); // Чистим символы
      char strBuffer[99]; // Создаём пустой массив символов
      inputStr.toCharArray(strBuffer, 99); // Перевести строку в массив символов последующего разделения по пробелам
      // Считываем x и y разделённых пробелом, а также Z
      for (byte i = 0; i < MAX_INPUT_VAL_IN_MANUAL_CONTROL; i++) {
        inputValues[i] = (i == 0 ? String(strtok(strBuffer, " ")) : String(strtok(NULL, " ")));
        inputValues[i].replace(" ", ""); // Убрать возможные пробелы между символами
        if (DEBUG_LEVEL >= 2) {
          if (inputValues[i] != "") {
            if (i > 0) Serial.print(", ");
            Serial.print(inputValues[i]);
          }
          if (i == MAX_INPUT_VAL_IN_MANUAL_CONTROL - 1) Serial.println();
        }
      }
      for (byte i = 0; i < MAX_INPUT_VAL_IN_MANUAL_CONTROL; i++) {
        if (inputValues[i] == "") continue; // Если значение пустое, то перейти на следующий шаг цикла
        String inputValue = inputValues[i]; // Записываем в строку обрезанную часть пробелами
        byte separatorIndexTmp = inputValue.indexOf("="); // Значение, где мы нашли позицию знака равенства
        byte separatorIndex = (separatorIndexTmp != 255 ? separatorIndexTmp : inputValue.length()); // Окончательная позиция для разделения
        key[i] = inputValue.substring(0, separatorIndex); // Записываем ключ с начала строки до знака равно
        values[i] = (inputValue.substring(separatorIndex + 1, inputValue.length())).toInt(); // Записываем значение с начала цифры до конца строки
        if (key[i] == "x") {
          x = values[i];
        } else if (key[i] == "y") {
          y = values[i];
        } else if (key[i] == "z") {
          z = values[i];
        } else if (key[i] == "s") {
          SetAllServosSpeed(values[i]);
        } else if (key[i] == "sc") {
          if (values[i] == 1) PneumaticSuctionCupState(true, 0);
          else if (values[i] == 0) PneumaticSuctionCupState(false, 0);
        } else if (key[i] == "break") {
          Serial.println(key[i]);
          control = false;
          break;
        }
        if (key[i].length() > 0) {
          Serial.print(key[i]); Serial.print(" = "); Serial.println(values[i]); // Печать ключ и значение, если ключ существует
        }
        if (x != xOld || y != yOld || z != zOld) {
          DeltaMoveToPos(x, y, z, true);
          if (x != xOld) xOld = x;
          if (y != yOld) yOld = y;
          if (z != zOld) zOld = z;
        }
      }
    }
  }
}

int ConvertDegreesToGoalPos(int degPos) {
  // 30, 330 - физически мертвые зоны диномикселя
  // Динамиксель команду 1023 - не выполняет, соотвественно 511 средняя позиция, а не как пишет документация 512
  degPos = constrain(degPos, 30, 300); // Ограничиваем входное значение, где 30° - это начальный градус слева и 300°
  int goalPos = map(degPos, 300, 30, 1022, 0);
  if (DEBUG_LEVEL >= 2) {
    DEBUG_SERIAL.print("inputDegPos: "); DEBUG_SERIAL.print(degPos); DEBUG_SERIAL.print(", "); DEBUG_SERIAL.print("goalPos: "); DEBUG_SERIAL.println(goalPos);
  }
  return goalPos;
}

// Установить скорость сервоприводу
void SetServoSpeed(int servoId, int speed) {
  dxl.setGoalVelocity(servoId, speed); // Задание целевой скорости
}

// Установить скорость сервоприводам
void SetServosSpeed(float *servosSpeed) {
  for (byte i = 0; i < JOINT_N; i++) {
    bool status = dxl.setGoalVelocity(i + 1, servosSpeed[i]); // Задание целевой скорости
    //if (status) DEBUG_SERIAL.println();
  }
}

// Установить скорость всем сервоприводам
void SetAllServosSpeed(int speed) {
  for (byte i = 0; i < JOINT_N; i++) {
    dxl.setGoalVelocity(i + 1, speed); // Задание целевой скорости
  }
}

// Сервоприводу занять позицию
void MoveServoToPos(int servoId, int pos) {
  dxl.setGoalPosition(servoId, pos); // Задание целевого положения
}

// Сервоприводам занять позиции
void MoveServosToPos(float *servosTargetPos, bool waitPerformedPos) {
  if (DEBUG_LEVEL >= 1) DEBUG_SERIAL.print("Target servos pos: ");
  for (byte i = 0; i < JOINT_N; i++) {
    if (servosTargetPos[i] != -1 && servosTargetPos[i] < 1023) { // Пропустить шаг цикла, если значение для него с массива 0-е
      bool status = dxl.setGoalPosition(i + 1, servosTargetPos[i]); // Задание целевого положения
      //if (status) DEBUG_SERIAL.println();
      DEBUG_SERIAL.print(servosTargetPos[i]);
    } else DEBUG_SERIAL.print("null");
    if (i < JOINT_N - 1) DEBUG_SERIAL.print(", ");
    else DEBUG_SERIAL.println();
  }
  if (waitPerformedPos) WaitServosPosPerformed(); // Если нужно, ждать занятия сервоприводами позиции
}

// Получить от серво его угол
int GetServoPos(int servoId) {
  return dxl.getPresentPosition(servoId);
}

// Получить значения углов с сервоприводов
int* GetServosPos() {
  int *pos = new int[JOINT_N];
  for (byte i = 0; i < JOINT_N; i++) {
    pos[i] = dxl.getPresentPosition(i + 1);
  }
  return pos;
}

// Получить значения о движения сервопривода
bool GetServoMoving(byte servoId) {
  return dxl.readControlTableItem(MOVING, servoId);
}

// Получить значения о движения моторов
bool* GetServosMoving() {
  bool *movingStates = new bool[JOINT_N];
  for (byte i = 0; i < JOINT_N; i++) {
    movingStates[i] = dxl.readControlTableItem(MOVING, i + 1);
  }
  return movingStates;
}

// Получить целевое значение с сервопривода
int GetServoTargetPos(byte servoId) {
  return dxl.readControlTableItem(GOAL_POSITION, servoId);
}

// Получить целевые значения с сервоприводов
int* GetServosTargetPos() {
  int* targetPos = new int[JOINT_N];
  for (byte i = 0; i < JOINT_N; i++) {
    targetPos[i] = dxl.readControlTableItem(GOAL_POSITION, i + 1);
  }
  return targetPos;
}

// Ждать пока сервомоторы не займут позиции
void WaitServosPosPerformed() {
  int* presentServosPos = new int[JOINT_N];
  int* targetServosPos = GetServosTargetPos(); // Получить целевые позиции с сервоприводов
  servosWorksMaxTimeTimer.setTimeout(MAX_TIME_PERFORMED_POS); // Установка времени таймера защиты по максимальному времени, запуск таймера
  servosWorksMaxTimeTimer.reset();
  serialPrintTimer.setInterval(100); // Установить таймер печати
  serialPrintTimer.reset(); // Спросить таймер печати
  if (DEBUG_LEVEL >= 1) DEBUG_SERIAL.println("Current servos position: ");
  while (true) {
    int* presentServosPos = GetServosPos(); // Прочитать значения с диномикселей
    if (DEBUG_LEVEL >= 1 && serialPrintTimer.isReady()) {
      for (byte i = 0; i < JOINT_N; i++) { // Вывод текущих положений с сервоприводов
        DEBUG_SERIAL.print(presentServosPos[i]);
        if (i < JOINT_N - 1) DEBUG_SERIAL.print(", ");
        else DEBUG_SERIAL.println();
      }
    }
    bool* isMoving = GetServosMoving(); // Прочитать значения о состоянии движений
    if (DEBUG_LEVEL >= 2 && serialPrintTimer.isReady()) {
      for (byte i = 0; i < JOINT_N; i++) { // Вывод значений о движении сервоприводов
        DEBUG_SERIAL.print(isMoving[i]); // 1 - движение, 0 - движения нет
        if (i < JOINT_N - 1) DEBUG_SERIAL.print(", ");
        else DEBUG_SERIAL.println();
      }
    }
    /*
    // Проверяем, допустима ли ошибка
    bool* servosIsPerformed = new bool[JOINT_N]; // Массив состояния о занятии позиции сервоприводами
    for (byte i = 0; i < JOINT_N; i++) { // Проверяем условие и записываем в массив для каждого отдельного серво
      servosIsPerformed[i] = (targetServosPos[i] - DYNAMIXEL_GOAL_POS_ERROR <= presentServosPos[i] && presentServosPos[i] <= targetServosPos[i] + DYNAMIXEL_GOAL_POS_ERROR);
    }
    */
    // Если все условия выполнились по серво или превышено максимальное время по таймеру, то выйти из цикла
    // (servosIsPerformed[0] && servosIsPerformed[1] && servosIsPerformed[2]) && 
    if ((isMoving[0] == 0 && isMoving[1] == 0 && isMoving[2] == 0) || servosWorksMaxTimeTimer.isReady()) {
      if (DEBUG_LEVEL >= 1) {
        DEBUG_SERIAL.print("Motors performed position: ");
        for (byte i = 0; i < JOINT_N; i++) {
          DEBUG_SERIAL.print(presentServosPos[i]);
          if (i < JOINT_N - 1) DEBUG_SERIAL.print(", ");
          else DEBUG_SERIAL.println();
        }
      }
      // Удалить массивы перед выходом из цикла для очистки памяти
      delete[] isMoving;
      delete[] presentServosPos;
      //delete[] targetServosPos;
      //delete[] servosIsPerformed;
      break;
    }
    // Удаляем массивы для очистки памяти
    delete[] presentServosPos;
    delete[] isMoving;
    //delete[] targetServosPos;
    //delete[] servosIsPerformed;
    delay(50);
  }
  serialPrintTimer.stop(); // Остановить таймер печати
}
