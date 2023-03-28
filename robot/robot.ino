#include <Wire.h>  //библиотека для I2C интерфейса
#include <math.h>

#include "WiFi.h"
#include "AsyncUDP.h"
#include "ESPmDNS.h"
#include <GyverPortal.h>  //@@@@@@@@@@@

#include <Adafruit_PWMServoDriver.h>                          // библиотека для моторной платы
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x70);  // адрес платы

#include "Adafruit_APDS9960.h"
Adafruit_APDS9960 apds9960;

// Определяем количество плат
#define NBOARDS 17

// Определяем номер этой платы
const unsigned int NUM = 15;
bool on_stan = false;
bool go_stan = false;
bool go_base = false;

// Определяем структуру данных для обмена

/*
   У всех плат проекта должна быть одинаковая
   структура! Для удобства её можно записать в
   отдельный .h файл
*/

struct multidata {
  /* Номер платы (необходим для быстрого доступа по индексу
    в массиве структур) */
  uint8_t num;
  /* В структуру можно добавлять элементы
    например, ip-адрес текущей платы:*/
  IPAddress boardIP;

  String nameTeam;
  bool dina_Base;  //sam
  bool dina_TS;    //sam
  bool dina_St;
  bool dina_R;  //sam
  bool alertTeam;
  GPcolor color_Res;  //@@@@@@@@@@
  byte vol_s_colb;
  byte vol_en_colb;
};

// Массив структур для обмена
multidata data[NBOARDS]{ 0 };

/* Определяем имена для mDNS */
// для ведущей платы
const char* master_host = "esp32master";
// приставка имени ведомой платы
const char* slave_host = "esp32slave";

// Определяем название и пароль точки доступа
const char* SSID = "TP-Link_4F90";
const char* PASSWORD = "00608268";

// Определяем порт
const uint16_t PORT = 8888;

// Создаём объект UDP соединения
AsyncUDP udp;

bool arm;

// Определяем callback функцию обработки пакета
void parsePacket(AsyncUDPPacket packet) {
  const multidata* tmp = (multidata*)packet.data();

  // Вычисляем размер данных
  const size_t len = packet.length() / sizeof(data[0]);

  // Если адрес данных не равен нулю и размер данных больше нуля...
  if (tmp != nullptr && len > 0) {

    // Проходим по элементам массива
    for (size_t i = 0; i < len; i++) {

      // Если это не ESP на которой выполняется этот скетч
      if (i != NUM) {
        // Обновляем данные массива структур
        data[i].num = tmp[i].num;
        data[i].boardIP = tmp[i].boardIP;
        // Записываем данные станции
        data[i].nameTeam = tmp[i].nameTeam;
        data[i].dina_Base = tmp[i].dina_Base;
        data[i].dina_TS = tmp[i].dina_TS;
        data[i].dina_St = tmp[i].dina_St;
        data[i].dina_R = tmp[i].dina_R;
        data[i].alertTeam = tmp[i].alertTeam;
        data[i].color_Res = tmp[i].color_Res;  //@@@@@@@@
        data[i].vol_s_colb = tmp[i].vol_s_colb;
        data[i].vol_en_colb = tmp[i].vol_en_colb;
      } else {
        // Записываем данные станции
        data[i].num = tmp[i].num;
        data[i].boardIP = tmp[i].boardIP;
        data[i].nameTeam = tmp[i].nameTeam;
        data[i].dina_Base = tmp[i].dina_Base;
        data[i].dina_TS = tmp[i].dina_TS;
        data[i].dina_St = tmp[7].dina_St;
        data[i].dina_R = tmp[7].dina_R;
        data[i].alertTeam = tmp[i].alertTeam;
        data[i].color_Res = tmp[i].color_Res;  //@@@@@@@@
        data[i].vol_s_colb = tmp[i].vol_s_colb;
        data[i].vol_en_colb = tmp[i].vol_en_colb;
        if(on_stan!=data[i].dina_St  ){
          if(data[i].dina_St){go_stan = true; go_base = false; on_stan=true;}//на станцию в путь
          else{go_stan = false;go_base = true; on_stan=false;}//на базу в путь
        }
      }
    }
  }
}




#define sensor_addr 0x3F  // Переключатели адреса в положении "OFF"


#define I2C_HUB_ADDR 0x70  // настройки I2C для платы MGB-I2C63EN
#define EN_MASK 0x08
#define DEF_CHANNEL 0x00
#define MAX_CHANNEL 0x08


uint16_t red_data = 0;
uint16_t green_data = 0;
uint16_t blue_data = 0;
uint16_t clear_data = 0;
uint16_t prox_data = 0;

static volatile int p00 = 0;
static volatile int p01 = 0;
static volatile int p02 = 0;
static volatile int p03 = 0;
static volatile int p04 = 0;
static volatile int p05 = 0;
static volatile int p06 = 0;
static volatile int p07 = 0;
static volatile int p08 = 0;
static volatile int p09 = 0;
static volatile int p10 = 0;
static volatile int p11 = 0;
static volatile int p12 = 0;
static volatile int p13 = 0;
static volatile int p14 = 0;
static volatile int p15 = 0;
static volatile int p16 = 0;
static volatile int p17 = 0;
static volatile int p18 = 0;

/*
  I2C порт 0x07 - выводы GP16 (SDA), GP17 (SCL) //tobot
  I2C порт 0x06 - выводы GP4 (SDA), GP13 (SCL)
  I2C порт 0x05 - выводы GP14 (SDA), GP15 (SCL) LINE!!!
  I2C порт 0x04 - выводы GP5 (SDA), GP23 (SCL)
  I2C порт 0x03 - выводы GP18 (SDA), GP19 (SCL) Color!!!!!
*/

void vv() {  // вывод данных с датчика линии
  Serial.print(p00);
  Serial.print(" ");
  Serial.print(p01);
  Serial.print(" ");
  Serial.print(p02);
  Serial.print(" ");
  Serial.print(p03);
  Serial.print(" ");
  Serial.print(p04);
  Serial.print(" ");
  Serial.print(p05);
  Serial.print(" ");
  Serial.print(p06);
  Serial.print(" ");
  Serial.print(p07);
  Serial.print(" ");
  Serial.print(p08);
  Serial.print(" ");
  Serial.print(p09);
  Serial.print(" ");
  Serial.print(p10);
  Serial.print(" ");
  Serial.print(p11);
  Serial.print(" ");
  Serial.print(p12);
  Serial.print(" ");
  Serial.print(p13);
  Serial.print(" ");
  Serial.print(p14);
  Serial.print(" ");
  Serial.print(p15);
  Serial.print(" ");
  Serial.print(p16);
  Serial.print(" ");
  Serial.print(p17);
  Serial.print(" ");
  Serial.print(p18);
  Serial.println();
}
void setup() {
  // Инициализация последовательного порта
  Serial.begin(115200);
  // Инициализация драйвера
  Wire.begin();
  Wire.setClock(100000L);
  delay(100);
  pwm.begin();
  // Частота (Гц)
  pwm.setPWMFreq(100);
  // Все порты выключены
  pwm.setPWM(8, 0, 4096);
  pwm.setPWM(9, 0, 4096);
  pwm.setPWM(10, 0, 4096);
  pwm.setPWM(11, 0, 4096);

  // Добавляем номер этой платы в массив структур
  data[NUM].num = NUM;
  data[NUM].dina_Base = true;
  // Инициируем WiFi
  WiFi.begin(SSID, PASSWORD);
  // Ждём подключения WiFi
  Serial.print("Подключаем к WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(100);
  }
  Serial.println();

  // Записываем адрес текущей платы в элемент структуры
  data[NUM].boardIP = WiFi.localIP();

  // Инициируем mDNS с именем "esp32slave" + номер платы
  if (!MDNS.begin(String(slave_host + NUM).c_str())) {
    Serial.println("не получилось инициировать mDNS");
  }

  // Узнаём IP адрес платы с UDP сервером
  IPAddress server = MDNS.queryHost(master_host);

  // Если удалось подключиться по UDP
  if (udp.connect(server, PORT)) {

    Serial.println("UDP подключён");
  }
  // вызываем callback функцию при получении пакета
  udp.onPacket(parsePacket);

  setBusChannel(0x03);  // 6ой канал
  if (!apds9960.begin()) {
    Serial.println("Failed to initialize device!");
  }
  // Инициализация режимов работы датчика
  apds9960.enableColor(true);
  apds9960.enableProximity(false);
  setBusChannel(0x05);  // 5ой канал
  init_sensor();
  Serial.println(" device!");
}


void povorv() {
  motorA_setpower(0x14, true);
  motorB_setpower(0x14, true);
  delay(1500);
  while (!(p06 < 1450 && p05 < 1450 && p15 < 1450 && p16 < 1450 && (p11 > 1450 || p10 > 1450 || p09 > 1450))) {
    setBusChannel(0x05);
    poll_sensor();
    motorA_setpower(0x14, true);
    motorB_setpower(0x14, true);
  }
  motorA_setpower(0x00, true);
  motorB_setpower(0x00, true);
}

void povorv2() {
  motorA_setpower(0x12, true);
  motorB_setpower(0x12, true);
  delay(1200);
  while (!(p06 < 1200 && p05 < 1200 && p15 < 1200 && p16 < 1200 && (p11 > 1450 || p10 > 1450 || p09 > 1450))) {
    setBusChannel(0x05);
    poll_sensor();
    motorA_setpower(0x12, true);
    motorB_setpower(0x12, true);
  }
  motorA_setpower(0x00, true);
  motorB_setpower(0x00, true);
}


void povorvn() {
  //motorA_setpower(0x14, false);
  motorB_setpower(0x14, false);
  delay(1000);
  while (!(p15 < 1350)) {
    setBusChannel(0x05);
    poll_sensor();
    motorA_setpower(0x14, false);
    motorB_setpower(0x14, false);
  }
  delay(200);
  motorA_setpower(0x00, true);
  motorB_setpower(0x00, true);
}

void loop() {
//motorA_setpower(0x12, false);
 // motorB_setpower(0x12, false);
 if(on_stan && !go_stan && !go_base){
   data[NUM].dina_Base = false;
  data[NUM].dina_TS = true;
  data[7].alerTeam = false;
 }
 else{
   if(!on_stan && !go_stan && !go_base){
   data[NUM].dina_Base = true;
   data[NUM].dina_TS = false;
   }
   else{
   data[NUM].dina_Base = false;
   data[NUM].dina_TS = false;
   }
   
   }
  udp.broadcastTo((uint8_t*)&data[NUM], sizeof(data[0]), PORT);
  if(go_base){
    to_base();
    go_base=false;
  }
  if(go_stan){
    to_stanc();
    go_stan=false;
  }
  //Serial.println(colr());
  //povorvn();
  //to_base();
  //delay(3000);
  //dvign();
  //to_stanc();
  //dvig(false);
//  delay(3000);
  //to_base();
  //delay(3000);
  delay(100);
  //Serial.println(colr());
  //setBusChannel(0x07);
  //vih();
}

uint8_t colr() {  //1 - zel, 2 - kras, 3-nichego,4 - dark
  uint8_t cl = 3;
  setBusChannel(0x03);
  while (!apds9960.colorDataReady()) {
    delay(5);
  }
  apds9960.getColorData(&red_data, &green_data, &blue_data, &clear_data);
  // Serial.println("RED   = " + String(red_data));
  // Serial.println("GREEN = " + String(green_data));
  // Serial.println("BLUE  = " + String(blue_data));
  // Serial.println("CLEAR = " + String(clear_data));
  // Serial.println("PROX  = " + String(prox_data));
  if (clear_data < 30 && green_data < 15 && red_data < 15 && blue_data < 15) {
    cl = 4;
  } else {
    if (clear_data < 180) {
      if (red_data > green_data && red_data > blue_data && abs(blue_data - green_data) < 4 && clear_data > 50) {
        cl = 2;
      } else {
        if (green_data > red_data && green_data > blue_data && blue_data > 30 && red_data < 49) {
          cl = 1;
        }
      }
    }
  }
  return (cl);
}

void dvig(bool bz) {  //движение по линии и стоп на определённом цвете: true - до базы(стоп на зелёном), false - до станции(стоп на красном)
  uint8_t nccl = 0;
  if (!bz) {
    nccl = 2;
  } else {
    nccl = 1;
  }

  while (colr() != nccl) {
    setBusChannel(0x05);
    poll_sensor();
    if (p06 > 1250 or p05 > 1250) {
      motorA_setpower(0x08, true);
      motorB_setpower(0x12, false);
    } else {
      if (p15 > 1250 or p16 > 1250) {
        motorA_setpower(0x12, true);
        motorB_setpower(0x08, false);
      }

      else {
        motorA_setpower(0x12, true);
        motorB_setpower(0x12, false);
      }
    }
  }
  motorA_setpower(0x12, true);
        motorB_setpower(0x12, false);
  delay(400);
  motorA_setpower(0x00, true);
  motorB_setpower(0x00, false);
}

void dvign() {  //движение по линии и стоп на определённом цвете: true - до базы(стоп на зелёном), false - до станции(стоп на красном)
  while (colr() != 4 || p16<1350 ) {
    Serial.println(colr());
    setBusChannel(0x05);
    poll_sensor();
    if (p06 > 1250 or p05 > 1250) {
      motorA_setpower(0x08, true);
      motorB_setpower(0x12, false);
    } else {
      if (p15 > 1250 or p16 > 1250) {
        motorA_setpower(0x12, true);
        motorB_setpower(0x08, false);
      }

      else {
        motorA_setpower(0x12, true);
        motorB_setpower(0x12, false);
      }
    }
  }
  motorA_setpower(0x12, false);
  motorB_setpower(0x12, true);
  delay(100);
  motorA_setpower(0x00, true);
  motorB_setpower(0x00, false);
}

void vih() {  //"умный выход из парковки
  setBusChannel(0x05);
  Serial.println("RTRTR");
  motorA_setpower(0x12, false);
  motorB_setpower(0x12, false);
  delay(300);
  while (!(p06 < 1350 && p05 < 1350 && p16 < 1350 && p17 < 1350 && (p11 > 1350 || p10 > 1350 || p09 > 1350))) {
    poll_sensor();
  }
  while (p03 < 1450 && p17 < 1450) {
    vv();
    Serial.println(colr());
    setBusChannel(0x05);
    poll_sensor();
    if (p06 > 1250 or p05 > 1250) {
      motorA_setpower(0x08, true);
      motorB_setpower(0x12, false);
    } else {
      if (p15 > 1250 or p16 > 1250) {
        motorA_setpower(0x12, true);
        motorB_setpower(0x08, false);
      }

      else {
        motorA_setpower(0x12, true);
        motorB_setpower(0x12, false);
      }
    }
  }
  //motorA_setpower(0x00, true);
  //motorB_setpower(0x00, true);
  /*motorA_setpower(0x12, true);
  motorB_setpower(0x12, false);
  while (p03 < 1450 && p17 < 1450) {
    vv();
    setBusChannel(0x05);
    poll_sensor();
  }*/
  motorA_setpower(0x00, false);
  motorB_setpower(0x00, true);
  Serial.println("Failed to initialize device!");
  delay(400);
  while (!(p06 < 1350 && p05 < 1350 && p16 < 1350 && p17 < 1350 && (p11 > 1350 || p10 > 1350 || p09 > 1350))) {
    setBusChannel(0x05);
    poll_sensor();
    motorA_setpower(0x12, true);
    motorB_setpower(0x0, true);
  }
  motorA_setpower(0x00, true);
  motorB_setpower(0x00, true);
}
void to_stanc() {
  vih();
  dvig(false);
  motorA_setpower(0x00, true);
  motorB_setpower(0x00, false);
}
void to_base() {  //ДВИЖЕНИИЕ ДО БАЗЫ ДОДЕЛАТЬ
  povorv2();
  dvig(true);
  povorvn();
  dvign();

}


// Мощность мотора "A" от -100% до +100% (от знака зависит направление вращения)
void motorA_setpower(float pwr, bool invert) {
  // Проверка, инвертирован ли мотор
  if (invert) {
    pwr = -pwr;
  }
  // Проверка диапазонов
  if (pwr < -100) {
    pwr = -100;
  }
  if (pwr > 100) {
    pwr = 100;
  }
  int pwmvalue = fabs(pwr) * 40.95;
  if (pwr < 0) {
    pwm.setPWM(10, 0, 4096);
    pwm.setPWM(11, 0, pwmvalue);
  } else {
    pwm.setPWM(11, 0, 4096);
    pwm.setPWM(10, 0, pwmvalue);
  }
}

// Мощность мотора "B" от -100% до +100% (от знака зависит направление вращения)
void motorB_setpower(float pwr, bool invert) {
  // Проверка, инвертирован ли мотор
  if (invert) {
    pwr = -pwr;
  }
  // Проверка диапазонов
  if (pwr < -100) {
    pwr = -100;
  }
  if (pwr > 100) {
    pwr = 100;
  }
  int pwmvalue = fabs(pwr) * 40.95;
  if (pwr < 0) {
    pwm.setPWM(8, 0, 4096);
    pwm.setPWM(9, 0, pwmvalue);
  } else {
    pwm.setPWM(9, 0, 4096);
    pwm.setPWM(8, 0, pwmvalue);
  }
}

bool setBusChannel(uint8_t i2c_channel)  // смена I2C порта
{
  if (i2c_channel >= MAX_CHANNEL) {
    return false;
  } else {
    Wire.beginTransmission(I2C_HUB_ADDR);
    Wire.write(i2c_channel | EN_MASK);
    Wire.endTransmission();
    return true;
  }
}
void init_sensor() {
  Wire.beginTransmission(sensor_addr);
  Wire.write(0x10);        // Регистр настройки всей микросхемы
  Wire.write(0b00000000);  // Нормальный режим работы
  Wire.write(0b01001111);  // АЦП в непрерывном режиме, 200 ksps, встроенный ИОН для ЦАП
  Wire.endTransmission();
  delay(1000);
  Wire.beginTransmission(sensor_addr);
  Wire.write(0x20);        // Регистр настройки порта 0 (подключен к оптическому сенсору)
  Wire.write(0b00000000);  // Сброс конфигурации порта
  Wire.write(0b00000000);
  Wire.endTransmission();
  Wire.beginTransmission(sensor_addr);
  Wire.write(0x21);        // Регистр настройки порта 1 (подключен к оптическому сенсору)
  Wire.write(0b00000000);  // Сброс конфигурации порта
  Wire.write(0b00000000);
  Wire.endTransmission();
  Wire.beginTransmission(sensor_addr);
  Wire.write(0x22);        // Регистр настройки порта 2 (подключен к оптическому сенсору)
  Wire.write(0b00000000);  // Сброс конфигурации порта
  Wire.write(0b00000000);
  Wire.endTransmission();
  Wire.beginTransmission(sensor_addr);
  Wire.write(0x23);        // Регистр настройки порта 3 (подключен к оптическому сенсору)
  Wire.write(0b00000000);  // Сброс конфигурации порта
  Wire.write(0b00000000);
  Wire.endTransmission();
  Wire.beginTransmission(sensor_addr);
  Wire.write(0x24);        // Регистр настройки порта 4 (подключен к оптическому сенсору)
  Wire.write(0b00000000);  // Сброс конфигурации порта
  Wire.write(0b00000000);
  Wire.endTransmission();
  Wire.beginTransmission(sensor_addr);
  Wire.write(0x25);        // Регистр настройки порта 5 (подключен к оптическому сенсору)
  Wire.write(0b00000000);  // Сброс конфигурации порта
  Wire.write(0b00000000);
  Wire.endTransmission();
  Wire.beginTransmission(sensor_addr);
  Wire.write(0x26);        // Регистр настройки порта 6 (подключен к оптическому сенсору)
  Wire.write(0b00000000);  // Сброс конфигурации порта
  Wire.write(0b00000000);
  Wire.endTransmission();
  Wire.beginTransmission(sensor_addr);
  Wire.write(0x27);        // Регистр настройки порта 7 (подключен к оптическому сенсору)
  Wire.write(0b00000000);  // Сброс конфигурации порта
  Wire.write(0b00000000);
  Wire.endTransmission();
  Wire.beginTransmission(sensor_addr);
  Wire.write(0x28);        // Регистр настройки порта 8 (подключен к оптическому сенсору)
  Wire.write(0b00000000);  // Сброс конфигурации порта
  Wire.write(0b00000000);
  Wire.endTransmission();
  Wire.beginTransmission(sensor_addr);
  Wire.write(0x29);        // Регистр настройки порта 9 (подключен к оптическому сенсору)
  Wire.write(0b00000000);  // Сброс конфигурации порта
  Wire.write(0b00000000);
  Wire.endTransmission();
  Wire.beginTransmission(sensor_addr);
  Wire.write(0x2A);        // Регистр настройки порта 10 (подключен к оптическому сенсору)
  Wire.write(0b00000000);  // Сброс конфигурации порта
  Wire.write(0b00000000);
  Wire.endTransmission();
  Wire.beginTransmission(sensor_addr);
  Wire.write(0x2B);        // Регистр настройки порта 11 (подключен к оптическому сенсору)
  Wire.write(0b00000000);  // Сброс конфигурации порта
  Wire.write(0b00000000);
  Wire.endTransmission();
  Wire.beginTransmission(sensor_addr);
  Wire.write(0x2C);        // Регистр настройки порта 12 (подключен к оптическому сенсору)
  Wire.write(0b00000000);  // Сброс конфигурации порта
  Wire.write(0b00000000);
  Wire.endTransmission();
  Wire.beginTransmission(sensor_addr);
  Wire.write(0x2D);        // Регистр настройки порта 13 (подключен к оптическому сенсору)
  Wire.write(0b00000000);  // Сброс конфигурации порта
  Wire.write(0b00000000);
  Wire.endTransmission();
  Wire.beginTransmission(sensor_addr);
  Wire.write(0x2E);        // Регистр настройки порта 14 (подключен к оптическому сенсору)
  Wire.write(0b00000000);  // Сброс конфигурации порта
  Wire.write(0b00000000);
  Wire.endTransmission();
  Wire.beginTransmission(sensor_addr);
  Wire.write(0x2F);        // Регистр настройки порта 15 (подключен к оптическому сенсору)
  Wire.write(0b00000000);  // Сброс конфигурации порта
  Wire.write(0b00000000);
  Wire.endTransmission();
  Wire.beginTransmission(sensor_addr);
  Wire.write(0x30);        // Регистр настройки порта 16 (подключен к оптическому сенсору)
  Wire.write(0b00000000);  // Сброс конфигурации порта
  Wire.write(0b00000000);
  Wire.endTransmission();
  Wire.beginTransmission(sensor_addr);
  Wire.write(0x31);        // Регистр настройки порта 17 (подключен к оптическому сенсору)
  Wire.write(0b00000000);  // Сброс конфигурации порта
  Wire.write(0b00000000);
  Wire.endTransmission();
  Wire.beginTransmission(sensor_addr);
  Wire.write(0x32);        // Регистр настройки порта 18 (подключен к оптическому сенсору)
  Wire.write(0b00000000);  // Сброс конфигурации порта
  Wire.write(0b00000000);
  Wire.endTransmission();
  delay(1000);
  Wire.beginTransmission(sensor_addr);
  Wire.write(0x20);        // Регистр настройки порта 0 (подключен к оптическому сенсору)
  Wire.write(0b01110001);  // Диапазон входного напряжения 0 ... 10 В, встроенный ИОН, порт в режиме входа АЦП
  Wire.write(0b11100000);  // Порт не ассоциирован с другим портом, количество выборок АЦП - 128
  Wire.endTransmission();
  Wire.beginTransmission(sensor_addr);
  Wire.write(0x21);        // Регистр настройки порта 1 (подключен к оптическому сенсору)
  Wire.write(0b01110001);  // Диапазон входного напряжения 0 ... 10 В, встроенный ИОН, порт в режиме входа АЦП
  Wire.write(0b11100000);  // Порт не ассоциирован с другим портом, количество выборок АЦП - 128
  Wire.endTransmission();
  Wire.beginTransmission(sensor_addr);
  Wire.write(0x22);        // Регистр настройки порта 2 (подключен к оптическому сенсору)
  Wire.write(0b01110001);  // Диапазон входного напряжения 0 ... 10 В, встроенный ИОН, порт в режиме входа АЦП
  Wire.write(0b11100000);  // Порт не ассоциирован с другим портом, количество выборок АЦП - 128
  Wire.endTransmission();
  Wire.beginTransmission(sensor_addr);
  Wire.write(0x23);        // Регистр настройки порта 3 (подключен к оптическому сенсору)
  Wire.write(0b01110001);  // Диапазон входного напряжения 0 ... 10 В, встроенный ИОН, порт в режиме входа АЦП
  Wire.write(0b11100000);  // Порт не ассоциирован с другим портом, количество выборок АЦП - 128
  Wire.endTransmission();
  Wire.beginTransmission(sensor_addr);
  Wire.write(0x24);        // Регистр настройки порта 4 (подключен к оптическому сенсору)
  Wire.write(0b01110001);  // Диапазон входного напряжения 0 ... 10 В, встроенный ИОН, порт в режиме входа АЦП
  Wire.write(0b11100000);  // Порт не ассоциирован с другим портом, количество выборок АЦП - 128
  Wire.endTransmission();
  Wire.beginTransmission(sensor_addr);
  Wire.write(0x25);        // Регистр настройки порта 5 (подключен к оптическому сенсору)
  Wire.write(0b01110001);  // Диапазон входного напряжения 0 ... 10 В, встроенный ИОН, порт в режиме входа АЦП
  Wire.write(0b11100000);  // Порт не ассоциирован с другим портом, количество выборок АЦП - 128
  Wire.endTransmission();
  Wire.beginTransmission(sensor_addr);
  Wire.write(0x26);        // Регистр настройки порта 6 (подключен к оптическому сенсору)
  Wire.write(0b01110001);  // Диапазон входного напряжения 0 ... 10 В, встроенный ИОН, порт в режиме входа АЦП
  Wire.write(0b11100000);  // Порт не ассоциирован с другим портом, количество выборок АЦП - 128
  Wire.endTransmission();
  Wire.beginTransmission(sensor_addr);
  Wire.write(0x27);        // Регистр настройки порта 7 (подключен к оптическому сенсору)
  Wire.write(0b01110001);  // Диапазон входного напряжения 0 ... 10 В, встроенный ИОН, порт в режиме входа АЦП
  Wire.write(0b11100000);  // Порт не ассоциирован с другим портом, количество выборок АЦП - 128
  Wire.endTransmission();
  Wire.beginTransmission(sensor_addr);
  Wire.write(0x28);        // Регистр настройки порта 8 (подключен к оптическому сенсору)
  Wire.write(0b01110001);  // Диапазон входного напряжения 0 ... 10 В, встроенный ИОН, порт в режиме входа АЦП
  Wire.write(0b11100000);  // Порт не ассоциирован с другим портом, количество выборок АЦП - 128
  Wire.endTransmission();
  Wire.beginTransmission(sensor_addr);
  Wire.write(0x29);        // Регистр настройки порта 9 (подключен к оптическому сенсору)
  Wire.write(0b01110001);  // Диапазон входного напряжения 0 ... 10 В, встроенный ИОН, порт в режиме входа АЦП
  Wire.write(0b11100000);  // Порт не ассоциирован с другим портом, количество выборок АЦП - 128
  Wire.endTransmission();
  Wire.beginTransmission(sensor_addr);
  Wire.write(0x2A);        // Регистр настройки порта 10 (подключен к оптическому сенсору)
  Wire.write(0b01110001);  // Диапазон входного напряжения 0 ... 10 В, встроенный ИОН, порт в режиме входа АЦП
  Wire.write(0b11100000);  // Порт не ассоциирован с другим портом, количество выборок АЦП - 128
  Wire.endTransmission();
  Wire.beginTransmission(sensor_addr);
  Wire.write(0x2B);        // Регистр настройки порта 11 (подключен к оптическому сенсору)
  Wire.write(0b01110001);  // Диапазон входного напряжения 0 ... 10 В, встроенный ИОН, порт в режиме входа АЦП
  Wire.write(0b11100000);  // Порт не ассоциирован с другим портом, количество выборок АЦП - 128
  Wire.endTransmission();
  Wire.beginTransmission(sensor_addr);
  Wire.write(0x2C);        // Регистр настройки порта 12 (подключен к оптическому сенсору)
  Wire.write(0b01110001);  // Диапазон входного напряжения 0 ... 10 В, встроенный ИОН, порт в режиме входа АЦП
  Wire.write(0b11100000);  // Порт не ассоциирован с другим портом, количество выборок АЦП - 128
  Wire.endTransmission();
  Wire.beginTransmission(sensor_addr);
  Wire.write(0x2D);        // Регистр настройки порта 13 (подключен к оптическому сенсору)
  Wire.write(0b01110001);  // Диапазон входного напряжения 0 ... 10 В, встроенный ИОН, порт в режиме входа АЦП
  Wire.write(0b11100000);  // Порт не ассоциирован с другим портом, количество выборок АЦП - 128
  Wire.endTransmission();
  Wire.beginTransmission(sensor_addr);
  Wire.write(0x2E);        // Регистр настройки порта 14 (подключен к оптическому сенсору)
  Wire.write(0b01110001);  // Диапазон входного напряжения 0 ... 10 В, встроенный ИОН, порт в режиме входа АЦП
  Wire.write(0b11100000);  // Порт не ассоциирован с другим портом, количество выборок АЦП - 128
  Wire.endTransmission();
  Wire.beginTransmission(sensor_addr);
  Wire.write(0x2F);        // Регистр настройки порта 15 (подключен к оптическому сенсору)
  Wire.write(0b01110001);  // Диапазон входного напряжения 0 ... 10 В, встроенный ИОН, порт в режиме входа АЦП
  Wire.write(0b11100000);  // Порт не ассоциирован с другим портом, количество выборок АЦП - 128
  Wire.endTransmission();
  Wire.beginTransmission(sensor_addr);
  Wire.write(0x30);        // Регистр настройки порта 16 (подключен к оптическому сенсору)
  Wire.write(0b01110001);  // Диапазон входного напряжения 0 ... 10 В, встроенный ИОН, порт в режиме входа АЦП
  Wire.write(0b11100000);  // Порт не ассоциирован с другим портом, количество выборок АЦП - 128
  Wire.endTransmission();
  Wire.beginTransmission(sensor_addr);
  Wire.write(0x31);        // Регистр настройки порта 17 (подключен к оптическому сенсору)
  Wire.write(0b01110001);  // Диапазон входного напряжения 0 ... 10 В, встроенный ИОН, порт в режиме входа АЦП
  Wire.write(0b11100000);  // Порт не ассоциирован с другим портом, количество выборок АЦП - 128
  Wire.endTransmission();
  Wire.beginTransmission(sensor_addr);
  Wire.write(0x32);        // Регистр настройки порта 18 (подключен к оптическому сенсору)
  Wire.write(0b01111001);  // Диапазон входного напряжения 0 ... 10 В, встроенный ИОН, порт в режиме входа АЦП
  Wire.write(0b11100000);  // Порт не ассоциирован с другим портом, количество выборок АЦП - 128
  Wire.endTransmission();
  delay(1000);
  // Отладка регистров
  /*
    int a = 0;
    int b = 0;
    Wire.beginTransmission(sensor_addr);
    Wire.write(0x45); // Регистр данных АЦП
    Wire.endTransmission();
    Wire.requestFrom(sensor_addr, 2);
    Serial.println(Wire.available());
    a = Wire.read();
    b = Wire.read();
    Serial.println(String(a, 2));
    Serial.println(String(b, 2));
  */
}

// Получение данных с датчика
void poll_sensor() {
  setBusChannel(0x05);
  int adc_sensor_data[38] = { 0 };
  Wire.beginTransmission(sensor_addr);
  Wire.write(0x40);  // Регистр данных АЦП
  Wire.endTransmission();
  Wire.requestFrom(sensor_addr, 10);
  if (Wire.available() == 10) {
    adc_sensor_data[0] = Wire.read();  // ADC00
    adc_sensor_data[1] = Wire.read();
    adc_sensor_data[2] = Wire.read();  // ADC01
    adc_sensor_data[3] = Wire.read();
    adc_sensor_data[4] = Wire.read();  // ADC02
    adc_sensor_data[5] = Wire.read();
    adc_sensor_data[6] = Wire.read();  // ADC03
    adc_sensor_data[7] = Wire.read();
    adc_sensor_data[8] = Wire.read();  // ADC04
    adc_sensor_data[9] = Wire.read();
  }
  Wire.beginTransmission(sensor_addr);
  Wire.write(0x45);  // Регистр данных АЦП
  Wire.endTransmission();
  Wire.requestFrom(sensor_addr, 10);
  if (Wire.available() == 10) {
    adc_sensor_data[10] = Wire.read();  // ADC05
    adc_sensor_data[11] = Wire.read();
    adc_sensor_data[12] = Wire.read();  // ADC06
    adc_sensor_data[13] = Wire.read();
    adc_sensor_data[14] = Wire.read();  // ADC07
    adc_sensor_data[15] = Wire.read();
    adc_sensor_data[16] = Wire.read();  // ADC08
    adc_sensor_data[17] = Wire.read();
    adc_sensor_data[18] = Wire.read();  // ADC09
    adc_sensor_data[19] = Wire.read();
  }
  Wire.beginTransmission(sensor_addr);
  Wire.write(0x4A);  // Регистр данных АЦП
  Wire.endTransmission();
  Wire.requestFrom(sensor_addr, 10);
  if (Wire.available() == 10) {
    adc_sensor_data[20] = Wire.read();  // ADC10
    adc_sensor_data[21] = Wire.read();
    adc_sensor_data[22] = Wire.read();  // ADC11
    adc_sensor_data[23] = Wire.read();
    adc_sensor_data[24] = Wire.read();  // ADC12
    adc_sensor_data[25] = Wire.read();
    adc_sensor_data[26] = Wire.read();  // ADC13
    adc_sensor_data[27] = Wire.read();
    adc_sensor_data[28] = Wire.read();  // ADC14
    adc_sensor_data[29] = Wire.read();
  }
  Wire.beginTransmission(sensor_addr);
  Wire.write(0x4F);  // Регистр данных АЦП
  Wire.endTransmission();
  Wire.requestFrom(sensor_addr, 8);
  if (Wire.available() == 8) {
    adc_sensor_data[30] = Wire.read();  // ADC15
    adc_sensor_data[31] = Wire.read();
    adc_sensor_data[32] = Wire.read();  // ADC16
    adc_sensor_data[33] = Wire.read();
    adc_sensor_data[34] = Wire.read();  // ADC17
    adc_sensor_data[35] = Wire.read();
    adc_sensor_data[36] = Wire.read();  // ADC18
    adc_sensor_data[37] = Wire.read();
  }
  p00 = adc_sensor_data[36] * 256 + adc_sensor_data[37];
  p01 = adc_sensor_data[34] * 256 + adc_sensor_data[35];
  p02 = adc_sensor_data[32] * 256 + adc_sensor_data[33];
  p03 = adc_sensor_data[30] * 256 + adc_sensor_data[31];
  p04 = adc_sensor_data[28] * 256 + adc_sensor_data[29];
  p05 = adc_sensor_data[26] * 256 + adc_sensor_data[27];
  p06 = adc_sensor_data[24] * 256 + adc_sensor_data[25];
  p07 = adc_sensor_data[22] * 256 + adc_sensor_data[23];
  p08 = adc_sensor_data[20] * 256 + adc_sensor_data[21];
  p09 = adc_sensor_data[18] * 256 + adc_sensor_data[19];
  p10 = adc_sensor_data[16] * 256 + adc_sensor_data[17];
  p11 = adc_sensor_data[14] * 256 + adc_sensor_data[15];
  p12 = adc_sensor_data[12] * 256 + adc_sensor_data[13];
  p13 = adc_sensor_data[10] * 256 + adc_sensor_data[11];
  p14 = adc_sensor_data[8] * 256 + adc_sensor_data[9];
  p15 = adc_sensor_data[6] * 256 + adc_sensor_data[7];
  p16 = adc_sensor_data[4] * 256 + adc_sensor_data[5];
  p17 = adc_sensor_data[2] * 256 + adc_sensor_data[3];
  p18 = adc_sensor_data[0] * 256 + adc_sensor_data[1];
}