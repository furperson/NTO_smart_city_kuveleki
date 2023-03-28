#include <Wire.h>
#include <VL53L0X.h>
#include <BH1750.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "mcp3021.h"
#include "SparkFun_SGP30_Arduino_Library.h"
#include "Adafruit_APDS9960.h"

unsigned long time_r = 0;

// Подключаем библиотеки для вифи:
#include "WiFi.h"
#include "PCA9536.h"
#include "AsyncUDP.h"
#include "ESPmDNS.h"
#include <GyverPortal.h>
GyverPortal portal;



uint8_t adcDeviceId = 0x4D;  // 0b00000001 // 0b00000101 (также попробуйте просканировать адрес: https://github.com/MAKblC/Codes/tree/master/I2C%20scanner)

MCP3021 mcp3021;         //датчик протечки воды
Adafruit_BME280 bme280;  //датчик температуры, влажности и давления
BH1750 lightMeter;       //датчик освещения
PCA9536 pca9536;         //реле
VL53L0X lox;             //датчик расстояния
SGP30 mySensor;          // датчик летучих соединений
Adafruit_APDS9960 apds;

// калибровочные значения с АЦП
const float air_value = 561.0;
const float water_value = 293.0;
const float moisture_0 = 0.0;
const float moisture_100 = 100.0;

#define I2C_HUB_ADDR 0x70
#define EN_MASK 0x08
#define DEF_CHANNEL 0x00
#define MAX_CHANNEL 0x08

#define NBOARDS 17           // Определяем количество плат
const unsigned int NUM = 7;  // Определяем номер этой платы. 7 - лаба,15 - динамика

struct multidata {  // Определяем структуру данных для обмена
  /* Номер платы (необходим для быстрого доступа по индексу
    в массиве структур) */
  uint8_t num;
  /* В структуру можно добавлять элементы
    например, ip-адрес текущей платы:*/
  IPAddress boardIP;

  String nameTeam;
  bool dina_Base;
  bool dina_TS;
  bool dina_St;
  bool dina_R;  //возврат
  bool alertTeam;
  GPcolor color_Res;
  byte vol_s_colb;
  byte vol_en_colb;
};

int mil = 0;
int co2 = 0;
int loc = 0;
int osv = 0;
int tok = 0;
int energy = 0;
int vlaz = 0;
int temperature = 0;
int davl = 0;
int mil_after = 0;
bool pomp = true;
bool vent = false;
bool door = false;
bool window = false;
String color_m = "ничего";
String color_m2 = "ничего";


multidata data[NBOARDS]{ 0 };  // Массив структур для обмена
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


void parsePacket(AsyncUDPPacket packet) {  // Определяем callback функцию обработки пакета
  const multidata* tmp = (multidata*)packet.data();
  //Serial.println("TESTEEETTETETE");

  // Вычисляем размер данных
  const size_t len = packet.length() / sizeof(data[0]);

  // Если адрес данных не равен нулю и размер данных больше нуля...
  if (tmp != nullptr && len > 0) {

    // Проходим по элементам массива
    for (size_t i = 0; i < len; i++) {

      // Если это не ESP на которой выполняется этот скетч, то забиваем и ничего не трогаем
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
        data[i].color_Res = tmp[i].color_Res;
        data[i].vol_s_colb = tmp[i].vol_s_colb;
        data[i].vol_en_colb = tmp[i].vol_en_colb;
      } else {
        //Serial.println(tmp[i].nameTeam);
        //эти данные должна отправлять динамика
        data[i].dina_St = tmp[i].dina_St;
        data[i].dina_Base = tmp[15].dina_Base;
        data[i].dina_TS = tmp[15].dina_TS;
        if(tmp[0].color_Res.r==255 && tmp[0].color_Res.r==255 && tmp[0].color_Res.r==0 ){
          color_m2="gold";
        }
        if(tmp[0].color_Res.r==0 && tmp[0].color_Res.r==255 && tmp[0].color_Res.r==255 ){
          color_m2="blue";
        }
        if(tmp[0].color_Res.r==0 && tmp[0].color_Res.r==0 && tmp[0].color_Res.r==0 ){
          color_m2="dark";
        }
        if(tmp[0].color_Res.r==0 && tmp[0].color_Res.r==255 && tmp[0].color_Res.r==0 ){
          color_m2="green";
        }
        if(tmp[0].color_Res.r==255 && tmp[0].color_Res.r==255 && tmp[0].color_Res.r==255 ){
          color_m2="ничего";
        }
      }
    }
  }
}




void setup() {
  Serial.begin(115200);
  Serial.println("RAB");
  Wire.begin();
  lox.init();
  lox.setTimeout(500);
#if defined LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
  lox.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  lox.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  lox.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif
#if defined HIGH_SPEED
  // reduce timing budget to 20 ms (default is about 33 ms)
  lox.setMeasurementTimingBudget(20000);
#elif defined HIGH_ACCURACY
  // increase timing budget to 200 ms
  lox.setMeasurementTimingBudget(200000);
#endif

  pca9536.reset();
  pca9536.setMode(IO_OUTPUT);

  /* if (mySensor.begin() == false) {
    Serial.println("No SGP30 Detected. Check connections.");
    //while (1)
    //  ;
  }
  mySensor.initAirQuality();*/

  mcp3021.begin(adcDeviceId);

  bool bme_status = bme280.begin();  // (0x76) (0x77) (также попробуйте просканировать адрес: https://github.com/MAKblC/Codes/tree/master/I2C%20scanner)
  if (!bme_status)
    Serial.println("Could not find a valid BME280 sensor, check wiring!");

  lightMeter.begin();
  data[NUM].num = NUM;
  data[NUM].vol_s_colb = 0;
  // Добавляем название команды в массив структур
  data[NUM].nameTeam = "Kuveliki";  //!!!!!!!!!!!!
  // Инициируем WiFi
  WiFi.begin(SSID, PASSWORD);
  // Ждём подключения WiFi
  Serial.print("Подключаем к WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(100);
    portal.enableAuth("admin", "5410");
  }
  Serial.println();


  if (!apds.begin()) {
    Serial.println("failed to initialize device! Please check your wiring.");
  } else Serial.println("Device initialized!");

  apds.enableColor(true);

  // Записываем адрес текущей платы в элемент структуры
  data[NUM].boardIP = WiFi.localIP();
  Serial.println(WiFi.localIP());

  // Инициируем mDNS с именем "esp32slave" + номер платы
  if (!MDNS.begin(String(slave_host + NUM).c_str())) {
    Serial.println("не получилось инициировать mDNS");
  }

  // Узнаём IP адрес платы с UDP сервером
  IPAddress server = MDNS.queryHost(master_host);

  // Если удалось подключиться по UDP
  if (udp.connect(server, PORT)) {

    Serial.println("UDP подключён");

    // вызываем callback функцию при получении пакета
    udp.onPacket(parsePacket);
  }
  data[NUM].vol_s_colb = 0;
  data[NUM].vol_en_colb = 0;

  Serial.println(server);
  portal.attachBuild(build);  //указываем функцию для "отрисовки"
  portal.attach(action);      //функция для считывания данных с страницы
  portal.start();             //запускаем gyverportal
  Serial.println("Веб страница запущена");
}

void action() {
  if (portal.click()) {
    if (portal.click("button_1")) {
      pomp = !pomp;
      pomp_change(pomp);
    }
  }
  if (portal.click()) {
    if (portal.click("button_izm")) {
      int ckn = izm_colr();
      switch (ckn) {  //0-green,1-blue,2-gold,3-dark
        case 0:
          color_m = "green";
          data[NUM].color_Res=GPcolor(0,255,0);
          break;
        case 1:
        data[NUM].color_Res=GPcolor(0,255,255);
          color_m = "blue";
          break;
        case 2:
          color_m = "gold";
          data[NUM].color_Res=GPcolor(255,255,0);
          break;
        case 3:
        data[NUM].color_Res=GPcolor(0,0,0);
          color_m = "dark";
          break;
      }
      if(color_m2!="ничего" && color_m!="ничего" && color_m!=color_m2){
        data[NUM].alertTeam=true;
      }
    }
  }

  if (portal.click()) {
    if (portal.click("button_2")) {
      vent = !vent;
      wind_change(!vent);
    }
  }
  if (portal.click()) {
    if (portal.click("button_3")) door = !door;
  }
  if (portal.click()) {
    if (portal.click("button_4")) window = !window;
  }
}

void build() {  // конструктор страницы
  GP.BUILD_BEGIN(600);
  GP.JQ_SUPPORT();
  GP.THEME(GP_DARK);
  // создаём блок вручную
  GP.BLOCK_TAB_BEGIN("Данные");

  GP.JQ_UPDATE_BEGIN();
  M_BOX(GP.LABEL("На базе "); GP.LED_GREEN("indicator_1", data[15].dina_Base););
  M_BOX(GP.LABEL("На станции "); GP.LED_GREEN("indicator_4", data[15].dina_TS););
  M_BOX(GP.BUTTON("button_1", "Помпа", "button_1", GP_GREEN); GP.LED_GREEN("indicator_2", !pomp););
  M_BOX(GP.BUTTON("button_2", "Вентилятор", "button_2", GP_GREEN); GP.LED_GREEN("indicator_3", vent););
  M_BOX(GP.BUTTON("button_3", "Дверь", "button_3", GP_GREEN); GP.LED_GREEN("indicator_5", door););
  M_BOX(GP.BUTTON("button_4", "Окно", "button_4", GP_GREEN); GP.LED_GREEN("indicator_", window););
  M_BOX(GP.BUTTON("button_izm", "Измерить цвет", "button_izm", GP_GREEN););
  GP.LABEL("Обьём до анализа: " + String(mil));
  GP.LABEL("Обьём после анализа: " + String(mil_after));
  GP.LABEL("CO2: " + String(co2) + " ЛОС: " + String(loc));
  GP.LABEL("Освещённость: " + String(osv));
  GP.LABEL("Ток: " + String(tok));         //-
  GP.LABEL("Энергия: " + String(energy));  //-
  GP.LABEL("Цвет: " + color_m);
  GP.LABEL("Цвет_запрошенный: " + color_m2);
  GP.LABEL("Влажность: " + String(vlaz) + " Температура: " + String(temperature) + " Давление: " + String(davl));
  GP.JQ_UPDATE_END();
  GP.BUILD_END();  //Завершаем строительство страницы
}


void loop() {
  if(measure_vol_datch_rer()>=146){
    pomp = false;
      pomp_change(pomp);
  }
  //Serial.println(vol());
  portal.tick();  //функция для работы веб интерфейса
  // Отправляем данные этой платы побайтово
  //measure_vol_laser();
  //measure_vol_datch();
  udp.broadcastTo((uint8_t*)&data[NUM], sizeof(data[0]), PORT);
  delay(50);
  if (millis() - time_r > 1000) {
    measure_vol_laser();
    temperature = get_temp();
    davl = get_pres();
    vlaz = get_hum();
    time_r = millis();
  }
}

void measure_vol_laser() {
  int now_vol = vol();
  data[NUM].vol_s_colb = now_vol;
  mil = now_vol;
}

void measure_vol_datch() {
  float adc0 = mcp3021.readADC();
  float h = map(adc0, air_value, water_value, moisture_0, moisture_100);
  // Вывод измеренных значений в терминал
  float ml = 0;
  if (h < 50 && h > 0) {
    ml = 50 / 50 * h;
  } else {
    if (h <= 75) {
      ml = 50 + 25 / (75 - 50) * (h - 50);
    } else {
      if (h <= 87) {
        ml = 75 + 25 / (87 - 75) * (h - 75);
      } else {
        if (h <= 93) {
          ml = 100 + 25 / (93 - 87) * (h - 87);
        } else {
          if (h < 97) {
            ml = 125 + 25 / (97 - 93) * (h - 93);
          } else {
            ml = 150;
          }
        }
      }
    }
  }
  data[NUM].vol_en_colb = ml;
  mil_after = ml;
}
int measure_vol_datch_rer() {
  float adc0 = mcp3021.readADC();
  float h = map(adc0, air_value, water_value, moisture_0, moisture_100);
  // Вывод измеренных значений в терминал
  float ml = 0;
  if (h < 50 && h > 0) {
    ml = 50 / 50 * h;
  } else {
    if (h <= 75) {
      ml = 50 + 25 / (75 - 50) * (h - 50);
    } else {
      if (h <= 87) {
        ml = 75 + 25 / (87 - 75) * (h - 75);
      } else {
        if (h <= 93) {
          ml = 100 + 25 / (93 - 87) * (h - 87);
        } else {
          if (h < 97) {
            ml = 125 + 25 / (97 - 93) * (h - 93);
          } else {
            ml = 150;
          }
        }
      }
    }
  }
  return int(ml) ;
}
//кулер
void wind_change(bool tr) {
  if (tr) {
    pca9536.setState(IO0, IO_HIGH);
  } else {
    pca9536.setState(IO0, IO_LOW);
  }
}

int izm_colr() {
  uint16_t r, g, b, c;
  float g2, b2;
  uint16_t gcounter = 0;
  uint16_t bcounter = 0;
  uint16_t gocounter = 0;
  uint16_t blcounter = 0;
  uint16_t counter = 0;
  int colr = 0;  //0-green,1-blue,2-gold,3-dark

  while (!apds.colorDataReady()) {
    delay(5);
  }
  for (int i = 0; i <= 3; i++) {
    apds.getColorData(&r, &g, &b, &c);
    g2 = g;
    b2 = b;

    if (r <= 25 and g2 / b2 >= 1.1 and g2 / b2 <= 2 and g > 30 and b > 30) {
      gcounter++;
    } else {
      if (r <= 35 and g % b >= 0 and g % b <= 10 and g >= 40 and b >= 40) {
        bcounter++;
      } else {
        if (g <= 30 and b <= 30 and r <= 25 and c <= 60) {
          blcounter++;
        } else {
          gocounter++;
        }
      }
    }
    counter++;
    delay(300);
  }

  if (gcounter > bcounter and gcounter > gocounter and gcounter > blcounter) {
    //Serial.println("Green");
    colr = 0;
  }
  if (bcounter > gcounter and bcounter > gocounter and bcounter > blcounter) {
    Serial.println("Blue");
    colr = 1;
  }
  if (gocounter > bcounter and gocounter > gcounter and gocounter > blcounter) {
    Serial.println("Gold");
    colr = 2;
  }
  if (blcounter > bcounter and blcounter > gocounter and blcounter > gcounter) {
    Serial.println("Black");
    colr = 3;
  }
  return (colr);
}

//помпа
void pomp_change(bool tr) {
  if (!tr) {
    pca9536.setState(IO1, IO_HIGH);
  } else {
    pca9536.setState(IO1, IO_LOW);
  }
}


//датчик расстояния
float dist() {
  int dist = int(lox.readRangeSingleMillimeters());
  return dist;
}

int vol() {
  int sum = 0;
  for (int i = 0; i < 20; i++) {
    sum += dist();
  }
  sum = sum / 20;
  // Serial.println(sum);
  int vl = int((3.14 * (0.00075625) * (15 - float(sum) / 10) / 100) * 1000 * 1000);
  return (vl);
}

//датчик освещености
float light() {
  float lux = lightMeter.readLightLevel();  // считывание освещенности
  return lux;
}

//датчик температуры, влажности и давления
float get_temp() {
  float t = bme280.readTemperature();
  return t;
}

float get_hum() {
  float h = bme280.readHumidity();
  return h;
}
float get_pres() {
  float p = bme280.readPressure() / 100.0F;
  return p;
}

// датчик протечки воды
float BDE() {
  float adc0 = mcp3021.readADC();
  float h = map(adc0, air_value, water_value, moisture_0, moisture_100);
  return h;
}

// функция смены порта
bool setBusChannel(uint8_t i2c_channel) {
  if (i2c_channel >= MAX_CHANNEL) {
    return false;
  } else {
    Wire.beginTransmission(I2C_HUB_ADDR);
    Wire.write(i2c_channel | EN_MASK);
    Wire.endTransmission();
    return true;
  }
}

float CO2() {
  return mySensor.CO2;
}
float TVOC() {
  return mySensor.TVOC;
}