//--------------------- НАСТРОЙКИ ----------------------
#define CH_NUM 0x60   // номер канала (должен совпадать с передатчиком)

//--------------------- НАСТРОЙКИ ----------------------

//--------------------- ДЛЯ РАЗРАБОТЧИКОВ -----------------------
// УРОВЕНЬ МОЩНОСТИ ПЕРЕДАТЧИКА
// На выбор RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
#define SIG_POWER RF24_PA_MAX

// СКОРОСТЬ ОБМЕНА
// На выбор RF24_2MBPS, RF24_1MBPS, RF24_250KBPS
// должна быть одинакова на приёмнике и передатчике!
// при самой низкой скорости имеем самую высокую чувствительность и дальность!!
// ВНИМАНИЕ!!! enableAckPayload НЕ РАБОТАЕТ НА СКОРОСТИ 250 kbps!
#define SIG_SPEED RF24_1MBPS
//--------------------- ДЛЯ РАЗРАБОТЧИКОВ -----------------------

//--------------------- БИБЛИОТЕКИ ----------------------
#include <Servo.h>
Servo servoUp;
Servo servoGutter;
Servo servoUltras;
Servo servoCamera;
int angle = 60;      // Переменная,в которой хранится положение сервопривода
int interval = 30;    // Интервал времени, через который будет производиться поворот сервопривода на один градус
unsigned long lastTimeCheck;  // Время последнего движения сервопривода, в мс от начала работы программы


#include "Ultrasonic.h"
#define RANGERPIN   16
Ultrasonic ultrasonic(RANGERPIN);
int distance;
int angle1 = 60, angle2 = 86, angle3 = 120;


#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
//RF24 radio(9, 10);   // "создать" модуль на пинах 9 и 10 для НАНО/УНО
RF24 radio(49, 53); // для МЕГИ
//--------------------- БИБЛИОТЕКИ ----------------------

//--------------------- ПЕРЕМЕННЫЕ ----------------------
byte pipeNo;
byte address[][6] = {"1Node", "2Node", "3Node", "4Node", "5Node", "6Node"}; // возможные номера труб

int recieved_data[10];   // массив принятых данных
int telemetry[10];       // массив данных телеметрии (то что шлём на передатчик)

int trackPosition = 0, gutterPosition = 2, move=0;   
bool viewFlag = false, backFlag = false, switchON = false, flagCleaner = false, Stop = false;             
bool discharge = false, flag = true, done = true, aval = false, firstStart = true, turn = true;
bool snowMode = false, iceMode = false; //Состояние режимов уборки(false - выкл; true - вкл)
unsigned long backTime, time1, time2, time3, speedtime, gutter1, gutter2, turn1;
unsigned long track, view, view1;
int ver[5] = {6, 6, 6, 6, 6}, ver1[5], proc;

const int runCleaner = 14;   // номер пина реле моторов пылесоса и щеток

//Переменные для хранения дистанций 3 положений желоба
int rightDistance, leftDistance, middleDistance;

//Переменная, показывающая состояние MessangeBox
int MessangeBox_Status;

//Переменная, показывающая состояния двигателей и датчиков
int Transmit_move;

//Переменная хранящая угол поворота сервы
int pos=86;

int speed;


char batLOW = "Батарея разряжена!";
char incline = "Подъем критический! Не рекомендуется его преодолевать.";
char slip = "Подъем скользкий! Попробуйте заехать на него задним ходом, посыпая его песком.";
char let = "Впереди препятствие! Необходимо его объехать.";
//--------------------- ПЕРЕМЕННЫЕ ----------------------

//------------------- ПОДКЛЮЧЕНИЕ IMU -------------------
// библиотека для работы I²C
#include <Wire.h>
// библиотека для работы с модулями IMU
#include <TroykaIMU.h>
// множитель фильтра
Madgwick filter;
#define BETA 0.22
Accelerometer accel;
Gyroscope gyro;

// Переменные для данных с гироскопа и акселерометра
float gx, gy, gz, ax, ay, az;
 
// Переменные для хранения самолётных углов ориентации
float yaw, pitch, roll;

// Переменная для хранения частоты выборок фильтра
float sampleRate = 100;
 
//------------------- ПОДКЛЮЧЕНИЕ IMU -------------------

//------------------- ПОДКЛЮЧЕНИЕ GPS -------------------
#include <TinyGPS.h>

float lat,lon;
TinyGPS gps;
//------------------- ПОДКЛЮЧЕНИЕ GPS -------------------

//------------------- ПОДКЛЮЧЕНИЕ BAT -------------------
// пин для считывания напряжения
int pin_read = A8;
// максимальный заряд аккумулятора
float max_v = 12.6; 
// минимальный заряд аккумулятора
float min_v = 8.25; 
//------------------- ПОДКЛЮЧЕНИЕ BAT -------------------

//----------------- ПОДКЛЮЧЕНИЕ МОТОРОВ -----------------
#define SPEED_1      5 
#define DIR_1        24

#define SPEED_2      6
#define DIR_2        7

#define SPEED_3      3
#define DIR_3        2

#define SPEED_4      9
#define DIR_4        8
//----------------- ПОДКЛЮЧЕНИЕ МОТОРОВ -----------------
void setup() {
  Serial.begin(9600);
  
  radio.begin();
  // На выбор RF24_2MBPS, RF24_1MBPS, RF24_250KBPS
  radio.setDataRate( RF24_250KBPS );
  radio.setChannel(0x60);
  // На выбор RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
  radio.setPALevel(RF24_PA_MIN); 
  radio.setRetries(5,5);
  radio.openReadingPipe(1,0xABCDABCD71LL);
  radio.startListening();
  
  gps_setup();
  imu_setup();
  analogReference(INTERNAL1V1);
  servoUp.attach(44);
  servoUp.write(150);
  servoGutter.attach(10);
  servoUltras.attach(46);
  servoCamera.attach(45);
  servoUltras.write(86);
  servoCamera.write(0); //Угол на который надо будет возможно поставить другой

//----------------- ПОДКЛЮЧЕНИЕ МОТОРОВ -----------------
  pinMode(runCleaner, OUTPUT);
  for (int i = 2; i < 10; i++) {     
    pinMode(i, OUTPUT);
  }
    pinMode(DIR_1, OUTPUT);
//----------------- ПОДКЛЮЧЕНИЕ МОТОРОВ -----------------
}

void loop(void) {
  lastTimeCheck=millis();
  //distance = ultrasonic.MeasureInCentimeters();
  gps_loop();
  imu_loop();

  
  if(millis() - view > 5000) {
    view = millis();
    viewFlag = true;
  }

  if (viewFlag == true){ // проверка заполнения пакета
     //standby_mode(); //Остановка двигателей движения
     servoUltras.write(angle3);
     delay(500);
     leftDistance = ultrasonic.MeasureInCentimeters();
     
     servoUltras.write(angle1);
     delay(500);
     rightDistance = ultrasonic.MeasureInCentimeters();
          
     servoUltras.write(angle2);
     delay(500);
     middleDistance = ultrasonic.MeasureInCentimeters();
     
     viewFlag = false;
  }
//////////////////////////////////////////////////////////////////////////////
/////// Заряд батареи
//////////////////////////////////////////////////////////////////////////////
  float Vbat = (analogRead(pin_read) * 1.1) / 1023;
  float del = 0.0885; 
  float Vin = Vbat / del;
  // уровень заряда в процентах
  int proc = ((Vin - min_v) / (max_v - min_v)) * 100;
  // вывод данных в монитор порта
  //Serial.println(Vin);
  //Serial.println(proc);
//////////////////////////////////////////////////////////////////////////////
/////// Скорость
//////////////////////////////////////////////////////////////////////////////
  float x, y, z;
  int b = 0.5;
  int c = 2;
  accel.readAXYZ(&x, &y, &z);
  speed = sqrt((pow(x, c) + pow(y, c)));
//////////////////////////////////////////////////////////////////////////////  
//////// Чтение информации с nrf24L01
//////////////////////////////////////////////////////////////////////////////

      telemetry[0] = proc; //Заряд  proc
      telemetry[1] = speed; //Скорость
      telemetry[2] = round(pitch); //Угол наклона
      telemetry[4] = lat; //Широта
      telemetry[5] = lon; //Долгота
      telemetry[6] = Transmit_move; //Состояние датчиков
      telemetry[7] = MessangeBox_Status; //Состояние всплывающих окон
      if (radio.available(0xABCDABCD71LL)>0) {
        radio.read( &recieved_data, sizeof(recieved_data));
        radio.stopListening();
        radio.openWritingPipe(0xABCDABCD71LL);
        radio.write(telemetry, sizeof(telemetry));
        radio.openReadingPipe(1,0xABCDABCD71LL);
        radio.startListening();
        work();
      }

  Serial.print(proc);
  Serial.print(" ");
  Serial.print(speed);
  Serial.print(" ");
  Serial.print(round(pitch));
  Serial.print(" ");
  Serial.print(lat);
  Serial.print(" ");
  Serial.print(lon);
  Serial.print("\t");

  Serial.print(recieved_data[0]);
  Serial.print("  ");
  Serial.print(recieved_data[1]);
  Serial.print("  ");
  Serial.print(recieved_data[2]);
  Serial.print("  ");
  Serial.print(recieved_data[3]);
  Serial.print("  ");
  Serial.print(recieved_data[4]);
  Serial.print("\t");

  Serial.print(leftDistance);
  Serial.print("  ");
  Serial.print(middleDistance);
  Serial.print("  ");
  Serial.print(rightDistance);
  Serial.print("  ");
  Serial.print(angle1);
  Serial.print("  ");
  Serial.print(angle2);
  Serial.print("  ");
  Serial.print(angle3);
  Serial.print("\t");
  Serial.print(Stop);
  Serial.print("\t");
  Serial.println(MessangeBox_Status); //Transmit_move
  
}
void work() { 
  if (trackPosition == 0) {
    flagCleaner = true;
  }
  else {
    flagCleaner = false;
  }
  
  //Управление двигателями движения
  if(recieved_data[0] == 1) {
    if (flagCleaner == true) {
      if (Stop == false) {
        move=1;    
        servoCamera.write(0);
        digitalWrite(DIR_1, LOW);
        analogWrite(SPEED_1, 200);
        digitalWrite(DIR_2, LOW);
        analogWrite(SPEED_2, 200);
      } else {
        move=0;
        standby_mode(); //Остановка двигателей движения
      }
    }
  } else if (recieved_data[0] == 2) {
      move=2;
      servoCamera.write(150);
      digitalWrite(DIR_1, HIGH);
      analogWrite(SPEED_1, 255);
      digitalWrite(DIR_2, HIGH);
      analogWrite(SPEED_2, 255);
  } else if (recieved_data[0] == 3) {
    if (flagCleaner == true) {
      move=3;
      servoCamera.write(0);
      digitalWrite(DIR_1, HIGH);
      analogWrite(SPEED_1, 255);
      digitalWrite(DIR_2, LOW);
      analogWrite(SPEED_2, 255);
    }
  } else if (recieved_data[0] == 4) {
    if (flagCleaner == true) {
      move=4;
      servoCamera.write(0);
      digitalWrite(DIR_1, LOW);
      analogWrite(SPEED_1, 255);
      digitalWrite(DIR_2, HIGH);
      analogWrite(SPEED_2, 255);
    }
  } else if (recieved_data[0] == 0) {
    move=0;
    standby_mode(); //Остановка двигателей движения
  }
  
  //Управление шнеками
  if (move==1 or move==3 or move==4) {
    if (recieved_data[1] == 1) {
      digitalWrite(runCleaner, HIGH);
    } else if (recieved_data[1] == 0) {
      digitalWrite(runCleaner, LOW);
    }
  } else Transmit_move=1; //Шнеки не должны включаться
  
  //Упраление посыпкой
  if (move==1 or move==2 or move==3 or move==4) {
    if (recieved_data[2] == 1) {
      digitalWrite(DIR_3, LOW);
      analogWrite(SPEED_3, 255);
    } else if (recieved_data[2] == 0) {
      analogWrite(SPEED_3, 0);
    }
  } else Transmit_move=2; //Посыпка не должна работать

  
  //Защита от "Дурака"
  if (recieved_data[1] == 1 and recieved_data[0] == 2) { //Шнеки не крутяться при заднем ходу
    standby_mode();
    digitalWrite(runCleaner, LOW);
  }
  
  //Вращение желоба только через программу
    if (recieved_data[3] == 1) {
      gutter_left();
    } else if (recieved_data[3] == 2) {
      gutter_forward();
    } else if (recieved_data[3] == 3) {
      gutter_right();
    }

    if (move==0 or move==2) {
      if(recieved_data[4] == 0) {     //Преодоление бордюра
        servoUp.write(105);
        trackPosition = 1;
      } else if (recieved_data[4] == 1) {
        servoUp.write(150);
        trackPosition = 0;
      }
    }
    
  //Вращение желоба в автоматическом режиме 
  if (gutterPosition == 1) {
    if (angle2 == 86 and middleDistance <= 50) {
      //move=0;
      //standby_mode();
      Stop = true;
      Transmit_move=5;
      //gutter_forward();
      Serial.println("предет впереди");
    } else {
      Stop = false;
      if (angle3 == 120 and leftDistance <= 50) {
        move=0;
        standby_mode();
        //Stop = false;
        Transmit_move=5;
        //gutter_forward();
        Serial.println("предет слева < 50см");
      } else if (angle3 == 120 and leftDistance > 50 and leftDistance < 100) {
        //Просто опускаем козырек в нижнее положение
        //gutter_left();
        Serial.println("предет слева > 50см");
      } else if (angle3 == 120 and leftDistance >= 100) {
        //Все нормально! Едем дальше!!!
        //gutter_left();
        Serial.println("предет слева в далеке");
      }
    }
  }

  if (gutterPosition == 2) {
    if (angle2 == 86 and middleDistance <= 50) {
      //move=0;
      //standby_mode();
      Stop = true;
      Transmit_move=5;
      //gutter_forward();
      Serial.println("предет впереди");
    } else if (angle2 == 86 and middleDistance > 50) Stop = false;
  }

  if (gutterPosition == 3) {
    if (angle2 == 86 and middleDistance <= 50) {
      move=0;
      standby_mode();
      gutter_forward();
      Serial.println("предет впереди");
    } else {
      if (angle1 == 60 and rightDistance <= 50) {
        move=0;
        standby_mode();
        gutter_forward();
        Serial.println("предет справа < 50см");
      } else if (angle1 == 60 and rightDistance > 50 and rightDistance < 100) {
        //Просто опускаем козырек в нижнее положение
        //gutter_right();
        Serial.println("предет справа > 50см");
      } else if (angle1 == 60 and rightDistance >= 100) {
        //Все нормально! Едем дальше!!!
        //gutter_right();
        Serial.println("предет справа в далеке");
      }
    }
  }

  //Всплывающие окна
  /*if(proc <= 20) {
    MessangeBox_Status=1;
  } else*/ if (round(pitch) > 15) {
    MessangeBox_Status=2;
  }
  else if (move==1 or move==3 or move==4) {
    if (speed<=1) {
      if (round(pitch) > 3) {
        MessangeBox_Status=3;
      }
    }
  } else if (Transmit_move==5) {
    MessangeBox_Status=4;
  } else {
    MessangeBox_Status=5;
  }
}
//------------------------Положение гусениц----------------------------------
void radar() {
  if (angle < 120) {
    if ((millis()-lastTimeCheck)>= interval) {
      angle++;
      servoUltras.write(angle);
      lastTimeCheck= millis();
    }
  } else if (angle > 60)  {
    if ((millis()-lastTimeCheck)>= interval) {
      angle--;
      servoUltras.write(angle);
      lastTimeCheck= millis();
    }   
  }
}
//------------------------Положение гусениц----------------------------------

//---------------------Левое положение желоба--------------------------------
void gutter_left() {
  if (gutterPosition == 2) {
    gutter1 = millis();
    do{
      digitalWrite(DIR_4, HIGH);
      analogWrite(SPEED_4, 100);
      delay(10);
      if ((millis() - gutter1 >= 2000)) flag = false;
    } while(flag);
     flag = true;
     analogWrite(SPEED_4, 0);
  } else if (gutterPosition == 3) {
    gutter2 = millis();
    do{
      digitalWrite(DIR_4, HIGH);
      analogWrite(SPEED_4, 100);
      delay(10);
      if ((millis() - gutter2 >= 4000)) flag = false;
    } while(flag);
     flag = true;
     analogWrite(SPEED_4, 0);
    } else {
      analogWrite(SPEED_4, 0);
    }
    //Transmit_move=4;
    gutterPosition = 1;
}
//---------------------Левое положение желоба--------------------------------

//---------------------Прямое положение желоба-------------------------------
void gutter_forward() {
  if (gutterPosition == 1) {
    gutter1 = millis();
    do{
      digitalWrite(DIR_4, LOW);
      analogWrite(SPEED_4, 100);
      delay(10);
      if ((millis() - gutter1 >= 2000)) flag = false;
    } while(flag);
     flag = true;
     analogWrite(SPEED_4, 0);
  } else if (gutterPosition == 3) {
    gutter1 = millis();
    do{
      digitalWrite(DIR_4, HIGH);
      analogWrite(SPEED_4, 100);
      delay(10);
      if ((millis() - gutter1 >= 2000)) flag = false;
    } while(flag);
     flag = true;
     analogWrite(SPEED_4, 0);
    } else {
      analogWrite(SPEED_4, 0);
  }
        gutterPosition = 2;
        //Transmit_move=5;
        //Сыпем снег под себя. Выставляем минимальное значение козырька
        servoGutter.write(50);
}
//---------------------Прямое положение желоба-------------------------------

//---------------------Правое положение желоба-------------------------------
void gutter_right() {
  if (gutterPosition == 2) {
    gutter1 = millis();
    do{
      digitalWrite(DIR_4, LOW);
      analogWrite(SPEED_4, 100);
      delay(10);
      if ((millis() - gutter1 >= 2000)) flag = false;
    } while(flag);
     flag = true;
     analogWrite(SPEED_4, 0);
  } else if (gutterPosition == 1) {
    gutter2 = millis();
    do{
      digitalWrite(DIR_4, LOW);
      analogWrite(SPEED_4, 100);
      delay(10);
      if ((millis() - gutter2 >= 4000)) flag = false;
    } while(flag);
     flag = true;
     analogWrite(SPEED_4, 0);
    } else {
      analogWrite(SPEED_4, 0);
  }
        //Transmit_move=6;
        gutterPosition = 3;
}
//---------------------Правое положение желоба-------------------------------

//---------------------Просмотр с левой стороны------------------------------
void left_view() {
  pos = 120;
  servoUltras.write(pos);
  if (pos == 120 and distance <= 50) {
    move=0;
    standby_mode();
    gutter_forward();
  } else if (pos == 120 and distance > 50 and distance < 100) {
    //Просто опускаем козырек в нижнее положение
    
  } else if (pos == 120 and distance >= 100) {
    //Все нормально! Едем дальше!!!
  }
}
//---------------------Просмотр с левой стороны------------------------------

//---------------------Просмотр с правой стороны-----------------------------
void right_view() {
  pos = 60;
  servoUltras.write(pos);
  if (pos == 60 and distance <= 50) {
    move=0;
    standby_mode();
    gutter_forward();
  } else if (pos == 60 and distance > 50 and distance < 100) {
    //Просто опускаем козырек в нижнее положение
    
  } else if (pos == 60 and distance >= 100) {
    //Все нормально! Едем дальше!!!
  }
}
//---------------------Просмотр с правой стороны-----------------------------

//---------------------Просмотр с средней стороны----------------------------
void middle_view() {
  pos = 86;
  servoUltras.write(pos);
  if (pos == 86 and distance <= 50) {
    move=0;
    standby_mode();
  }
}
//---------------------Просмотр с средней стороны----------------------------

/////////
void standby_mode() {
    servoCamera.write(150);   //Угол на который надо будет возможно поставить другой
    analogWrite(SPEED_1, 0);  //Отключение всех моторов. Ждем команд
    analogWrite(SPEED_2, 0);
    analogWrite(SPEED_3, 0);
    analogWrite(SPEED_4, 0);
    digitalWrite(runCleaner, LOW);
}
/////////

/////////////////////////////////////////////////////////////////////////////
/////Функции датчиков
/////////////////////////////////////////////////////////////////////////////
void imu_setup() {
  accel.begin();
  //accel.setRange(RANGE_2G);
  gyro.begin();
  // Инициализируем фильтр
    filter.begin();
}

void gps_setup() {
  Serial1.begin(9600); // connect gps sensor  
}

void imu_loop() {
  // Запоминаем текущее время
    unsigned long startMillis = millis();
    // Считываем данные с акселерометра в единицах G
    accel.readAccelerationGXYZ(ax, ay, az);
    // Считываем данные с гироскопа в радианах в секунду
    gyro.readRotationRadXYZ(gx, gy, gz);
    // Устанавливаем частоту фильтра
    filter.setFrequency(sampleRate);
    // Обновляем входные данные в фильтр
    filter.update(gx, gy, gz, ax, ay, az);
 
    // Получаем из фильтра углы: yaw, pitch и roll 
    yaw = filter.getYawDeg();
    pitch = filter.getPitchDeg();
    roll = filter.getRollDeg();
 
    // Вычисляем затраченное время на обработку данных
    unsigned long deltaMillis = millis() - startMillis;
    // Вычисляем частоту обработки фильтра
    sampleRate = 1000 / deltaMillis;
}

void gps_loop() {
   while(Serial1.available()){ // check for gps data
  if(gps.encode(Serial1.read()))// encode gps data
  {
  gps.f_get_position(&lat,&lon); // get latitude and longitude
  }
  }
}
