//--------------------- НАСТРОЙКИ ----------------------
#define CH_NUM 0x60   // номер канала (должен совпадать с приёмником)

int MessangeBox_Status, Transmit_move;
/* 
String batLOW = "Батарея разряжена!";
String incline = "Подъем критический! Не рекомендуется его преодолевать.";
String slip = "Подъем скользкий! Попробуйте заехать на него задним ходом, посыпая его песком.";
String let = "Впереди препятствие! Необходимо его объехать.";
*/
String batLOW = "Battery low!";
String incline = "The rise is critical! It is not recommended to overcome it.";
String slip = "The climb is slippery! Try to drive it in reverse, sprinkling it with sand.";
String let = "There's an obstacle ahead! You need to go around it.";
String OK = "OK";
String str;
//--------------------- НАСТРОЙКИ ----------------------

//--------------------- ДЛЯ РАЗРАБОТЧИКОВ -----------------------
// УРОВЕНЬ МОЩНОСТИ ПЕРЕДАТЧИКА
// На выбор RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
#define SIG_POWER RF24_PA_LOW

// СКОРОСТЬ ОБМЕНА
// На выбор RF24_2MBPS, RF24_1MBPS, RF24_250KBPS
// должна быть одинакова на приёмнике и передатчике!
// при самой низкой скорости имеем самую высокую чувствительность и дальность!!
// ВНИМАНИЕ!!! enableAckPayload НЕ РАБОТАЕТ НА СКОРОСТИ 250 kbps!
#define SIG_SPEED RF24_1MBPS
//--------------------- ДЛЯ РАЗРАБОТЧИКОВ -----------------------

//--------------------- БИБЛИОТЕКИ ----------------------
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
RF24 radio(9, 10); // "создать" модуль на пинах 9 и 10 Для Уно
//RF24 radio(9, 53); // для Меги
//--------------------- БИБЛИОТЕКИ ----------------------

//--------------------- ПЕРЕМЕННЫЕ ----------------------
byte address[][6] = {"1Node", "2Node", "3Node", "4Node", "5Node", "6Node"}; // возможные номера труб

int transmit_data[10];          // массив пересылаемых данных
int telemetry[10];              // массив принятых от приёмника данных телеметрии
byte rssi;
int trnsmtd_pack = 1, failed_pack;
unsigned long RSSI_timer, time1, time2;

int sig, bat, inChar, Stop, gutterMode, cleanSnow, cleanIce, trackMode, forward, turn, trPack, prStatus, strNum, TStr, b;
int lat, lon, speed, deg;
//--------------------- ПЕРЕМЕННЫЕ ----------------------

void setup() {
  Serial.begin(115200); // открываем порт для связи с ПК
  
  radio.begin();
  // На выбор RF24_2MBPS, RF24_1MBPS, RF24_250KBPS
  radio.setDataRate( RF24_250KBPS );
  radio.setChannel(0x60);
  // На выбор RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
  radio.setPALevel(RF24_PA_MIN);
  radio.setRetries(5,5);
}

void loop() {
    radio.openWritingPipe(0xABCDABCD71LL);
    radio.write(transmit_data, sizeof(transmit_data));
    radio.openReadingPipe(1,0xABCDABCD71LL);
    radio.startListening();
    delay(10);
    if (radio.available(0xABCDABCD71LL)>0) {
      radio.read( &telemetry, sizeof(telemetry));
      trPack++;
    }
    radio.stopListening();
    
  if(Serial.available() != 0) {
    char c = (char)Serial.read();
    /*if(c == '1') gutterMode=1;       //Положение желоба
    else if (c == '2') gutterMode=2;
    else if (c == '3') gutterMode=3;
    else if (c == '4') cleanSnow=1; //Управление режимами уборки снега и льда
    else if (c == '5') cleanSnow=0;
    else if (c == '6') cleanIce=1;
    else if (c == '7') cleanIce=0;
    else if (c == '8') trackMode=1; //Позиция гусениц
    else if (c == '9') trackMode=2;*/

    
    if(c == 'w') forward=1;
    else if (c == 's') forward=2;
    else if (c == 'a') forward=3;
    else if (c == 'd') forward=4;
    else if (c == 'f') cleanSnow=1; //forward=5
    else if (c == 'g') cleanIce=1;
    else if (c == '2') trackMode=1;
    else if (c == '4') gutterMode=1;
    else if (c == '5') gutterMode=2;
    else if (c == '6') gutterMode=3;
    else if (c == 'e') forward=0;
    else if (c == '0') cleanSnow=0;
    else if (c == '1') cleanIce=0;
    else if (c == '3') trackMode=0;
    
  }

  transmit_data[0] = forward;
  transmit_data[1] = cleanSnow;
  transmit_data[2] = cleanIce;
  transmit_data[3] = gutterMode;
  transmit_data[4] = trackMode;

//Вывод данных в разные строки
  Serial.println(bat); //Батарея  bat
  Serial.println(sig); //Качество связи
  Serial.println(telemetry[1]); //Скорость
  Serial.println(telemetry[2]); //Угол наклона
  Serial.println(lat); //Широта
  Serial.println(lon); //Долгота
  Serial.println(str);
  Serial.println(Transmit_move); //Режим работы робота
  //Serial.println(turn);
  
  
  Transmit_move = telemetry[6];
  MessangeBox_Status = telemetry[7];

  if (MessangeBox_Status==1) {
    str = batLOW;
  } else if (MessangeBox_Status==2) {
    str = incline;
  } else if (MessangeBox_Status==3) {
    str = slip;
  } else if (MessangeBox_Status==4) {
    str = let;
  } else if (MessangeBox_Status==5) {
    str = OK;
  }

  if (millis() - time2 >= 2000) {
    bat = telemetry[0];
    time2 = millis();
  }
  if (millis() - time1 >= 1000) {
    sig = trPack;
    trPack = 0;
    time1 = millis();
  }
}
