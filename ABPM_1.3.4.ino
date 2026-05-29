////////////////   v 1.3.4   //////////////
/*

- Убраны стуки в крайних положениях сканирования
- Изменен алгоритм проверки связи с BMI160 (добавлено принудительное включение режима SPI)
- При нажатии кнопки влево/вправо происходит сначала выход из сканирования
  и только при повторном нажатии непосредственно поворот
- Время опроса приемника вынесено в настройки RX_POLL_INTERVAL
- Уменьшено время игнорирования дребезга контактов
- Уменьшено время реакции на нажатие кнопок
- Если отключен режим Trim, то в ПЗУ вместо трим коэффициентов записываются нули

*/
///////////////////////////////////////////

/*
Программа для управления шаговым двигателем NEMA 23 с помощью драйвера TMC2160 или совместимого
добавлена поддержка радио пультов. 
Приемник SYN480R

MIT License

Copyright (c) 2024 Sergey Balandin

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

Потенциометр SPEED подключается к A7
Педаль Right  - A0
Педаль Left -  A1
Кнопка Scan - A2
Кнопка Brake - A3
Светодиод - A5

Arduino NANO     SYN480R
  GND             GND
  3V3             VCC
  D2              DAT

Arduino NANO     TMC2160
  D6              CLK
  D7              DIR
  D8              EN
  GND             GND

Arduino NANO     BMI160
  +5V             VIN
                  3V3
  GND             GND
  D13             SCL (SCK)
  D11             SDA (MOSI)
  D10             CS  
  D12             SAO (MISO)


Схема подключения и описание режимов https://github.com/BalandinSV/ABPM8-firmware
*/
#include "GyverStepper.h" // https://github.com/GyverLibs/GyverStepper
#include "RCSwitch.h"     // Библиотека для работы с пультом https://github.com/sui77/rc-switch/tree/master
#include <EEPROM.h>       // Библиотека для работы с ПЗУ
#include <BMI160Gen.h>    // https://github.com/hanyazou/BMI160-Arduino/tree/master
#include "MadgwickAHRS.h" // https://github.com/robotclass/RobotClass-MadgwickAHRS
#include "GyverFilters.h" // Библиотека фильтров https://github.com/GyverLibs/GyverFilters
#include <Wire.h>
#include <SPI.h>

//////////////////// СЕКЦИЯ НАСТРОЕК ////////////////////////

#define IMU_CS_PIN 10
#define R_BUTTON_PIN A0
#define L_BUTTON_PIN A1
#define SCAN_BUTTON_PIN A2
#define BRAKE_BUTTON_PIN A3
#define SPEED_POT_PIN A7
#define LED_PIN A5

#define STEP_PIN 6
#define DIR_PIN 7
#define ENABLE_PIN 8
#define MICRO_STEP 6400         //6400 импульсов на оборот. MRES = 32 Для TMC2160 (M0-OFF, M1-ON)

#define SCAN_ANGLE_1 35         // Максимальный угол сканирования половина сектора сканирования
#define SCAN_ANGLE_2 80         // Максимальный угол сканирования половина сектора сканирования
#define SPEED_MIN 50            // Минимальная угловая скорость шаг/сек
#define SPEED_MAX 1600          // Максимальная угловая скорость шаг/сек
#define ACCELERATION 0          // Ускорение. 0 - выключено
#define NEUTRAL_ZONE 0.45       // Зона нечувствительности в режиме стабилизации в градусах. Не меняем
//#define ANGLE_TO_STEP 4.44    // 1600/360=4.44 для 1600 шаг/об
//#define ANGLE_TO_STEP 8.89    // 3200/360=8.89 для 3200 шаг/об
#define ANGLE_TO_STEP 17.78     // 6400/360=17.78 для 6400 шаг/об
//#define ANGLE_TO_STEP 35.55   // 12800/360=35.55 для 12800 шаг/об
#define GEAR_RATIO 1            // Передаточное число редуктора
#define MOTOR_DIRECTION 1       // Направления вращения мотора 1 или -1

#define TO_RAD 0.01745329252f   // Коэффициент для перевода градусов в радианы
#define IMU_PERIOD 40           // Период опроса IMU мс
#define MOTOR_PERIOD 250        // Период управляющего воздействия мс
#define RX_POLL_INTERVAL 120    // Период опроса приемника
#define CALIB_STEP 100          // Количество итераций калибровки трима
#define GYRO_DEAD_ZONE 3        //Зона нечувствительности гироскопа в LBS, 1LBS ~ 0.45 град/мин
#define TRIM_CALIBRATION 

//////////////////////////// РЕЖИМ ОТЛАДКИ ///////////////////////////

//#define DEBUG // Раскомментировать для активации сообщений при отладке

//////////////////////////////////////////////////////////////////////

#ifdef DEBUG
    #define DEBUG_PRINT(x) Serial.print(x)
    #define DEBUG_PRINTF(x, y) Serial.print(x, y)
    #define DEBUG_PRINTLN(x) Serial.println(x)
    #define DEBUG_PRINTLNF(x, y) Serial.println(x, y)
    #define DEBUG_BEGIN(x) Serial.begin(x)
#else
    #define DEBUG_PRINT(x)
    #define DEBUG_PRINTF(x, y)
    #define DEBUG_PRINTLN(x)
    #define DEBUG_PRINTLNF(x, y)
    #define DEBUG_BEGIN(x)
#endif

bool transmitButtonA = false;
bool transmitButtonB = false;
bool transmitButtonAB = false;

bool rightButtonPinState = false;
bool leftButtonPinState = false;
bool scanButtonPinState = false;
bool brakeButtonPinState = false;
bool scanMode = false;
bool holdMode = true;           // Режим при включении
bool stabilizationMode = false;
bool dir = true;
bool enableIMU = false;
bool speedChange = false;

//long TransmitButton;
float accelOffsetX;               // Переменная для хранения калибровки гироскопа XAccelOffset
float accelOffsetY;               // Переменная для хранения калибровки гироскопа YAccelOffset
float accelOffsetZ;               // Переменная для хранения калибровки гироскопа ZAccelOffset
float gyroOffsetX;                // Переменная для хранения калибровки гироскопа XGyroOffset
float gyroOffsetY;                // Переменная для хранения калибровки гироскопа YGyroOffset
float gyroOffsetZ;                // Переменная для хранения калибровки гироскопа ZGyroOffset

int8_t trimAccelOffsetX = 0;      // Переменная для хранения смещения (trim) акселерометра по оси X
int8_t trimAccelOffsetY = 0;      // Переменная для хранения смещения (trim) акселерометра по оси Y
int8_t trimAccelOffsetZ = 0;      // Переменная для хранения смещения (trim) акселерометра по оси Z

int8_t trimGyroOffsetX = 0;       // Переменная для хранения смещения (trim) гироскопа по оси X
int8_t trimGyroOffsetY = 0;       // Переменная для хранения смещения (trim) гироскопа по оси Y
int8_t trimGyroOffsetZ = 0;       // Переменная для хранения смещения (trim) гироскопа по оси Z

uint8_t blinkMode = 0;            // Режим мигания светодиодом
int speedPotValue = 0;            // Значение с АЦП потенциометра SPEED
int speedPotValuePrevius = 0;     // Для определения изменения скорости в режиме SCAN


uint32_t btnBrakeTimer = 0;       // Время нажатия кнопки BAKE
uint32_t btnScanTimer = 0;        // Время нажатия кнопки SCAN
uint32_t imu_t = 0;               // Переменная для хранения времени опроса IMU
uint32_t motor_t = 0;             // Переменная для хранения времени корректировки мотором
int32_t scanAngle = 120;
uint32_t transmitButtonCode_A[3]; //Массив для хранения кодов кнопок А разных брелоков 3 х 4 байта 
uint32_t transmitButtonCode_B[3]; //Массив для хранения кодов кнопок B разных брелоков 3 х 4 байта
uint32_t transmitButtonCode_AB[3]; //Массив для хранения кодов кнопок А+B разных брелоков 3 х 4 байта

float basePoint = 0;              // Точка отсчета для удержания
float currentAngle = 0;           // Текущий курс
float correctionValue = 0;        // Величина отклонения курса

// Переменные для IMU

float tDelta;                     // Время между измерениями
int aix, aiy, aiz;                // Сырые данные с акселерометра
int gix, giy, giz;                // Сырые данные с гироскопа
float imu[3];                     // Масив для записи текущих измерений
float quat[4];                    // Кватерионы

GStepper<STEPPER2WIRE> stepper(MICRO_STEP, STEP_PIN, DIR_PIN, ENABLE_PIN); // Инициализируем драйвер
RCSwitch mySwitch = RCSwitch();   // Инициализируем приемник

GMedian3<int> filterAxelX;         // Фильтр акселерометр ось X
GMedian3<int> filterAxelY;         // Фильтр акселерометр ось Y
GMedian3<int> filterAxelZ;         // Фильтр акселерометр ось Z
GMedian3<int> filterGyroX;         // Фильтр гироскоп ось X
GMedian3<int> filterGyroY;         // Фильтр гироскоп ось Y
GMedian3<int> filterGyroZ;         // Фильтр гироскоп ось Z

// Проверка связи с BMI160
bool checkBMI160 ()
{
    SPI.begin();
    SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE0)); // Настраиваем шину SPI
    
    digitalWrite(IMU_CS_PIN, LOW);
    SPI.transfer(0x80 | 0x7F);     // Для переключения в SPI режим необходимо прочитать любой регистр, читаем 0X7F
    SPI.transfer(0x00);            // Dummy read

    delayMicroseconds(10);

    SPI.transfer(0x7E);            // Soft Reset
    SPI.transfer(0xB6);
    digitalWrite(IMU_CS_PIN, HIGH);
    delay(1);
    
    digitalWrite(IMU_CS_PIN, LOW);
    SPI.transfer(0x80 | 0x00);     // Читаем Chip ID
    uint8_t chip_id = SPI.transfer(0x00);
    digitalWrite(IMU_CS_PIN, HIGH);
    delay(1);
    SPI.endTransaction();
    SPI.end();
    return (chip_id == 0xD1);
    
}

// Мигаем светодиодом N раз
void ledBlinkN (uint8_t _ledPin, uint32_t _blinkPeriod, uint8_t _blinkCount)
{
  bool _ledState = digitalRead(_ledPin);
  for (int i = 0; i < _blinkCount; i++)
  {
    _ledState = ! _ledState;
    digitalWrite(_ledPin, _ledState);
    delay(_blinkPeriod);
    _ledState = ! _ledState;
    digitalWrite(_ledPin, _ledState);
    delay(_blinkPeriod);
  }
}

// Проверяем дубли кодов передатчика
bool checkCode (uint32_t currentCode)
{
  for (uint8_t i = 0; i < 3; i++) 
     {
      if ((currentCode == transmitButtonCode_A[i]) || (currentCode == transmitButtonCode_B[i]) || (currentCode == transmitButtonCode_AB[i]))
         {
          DEBUG_PRINTLN("Duplicate code!");
          ledBlinkN(LED_PIN, 100, 5);
          return false;
         } 
     }
      return true;
}

// Мигаем светодиодом с разными режимами
void ledBlink(uint8_t _ledPin, uint32_t _blinkPeriod, uint8_t _blinkMode)
{
  static uint8_t blink_loop;
  static uint32_t tmr_Led;
  uint8_t modes[] = {
   0B00000000, //Blink mode 0 - Светодиод выключен
   0B11111111, //Blink mode 1 - Горит постоянно
   0B00001111, //Blink mode 2 - Мигание по 0.5 сек
   0B00000001, //Blink mode 3 - Короткая вспышка раз в секунду
   0B00000101, //Blink mode 4 - Две короткие вспышки раз в секунду
   0B00010101, //Blink mode 5 - Три короткие вспышки раз в секунду
   0B01010101  //Blink mode 6 - Частые короткие вспышки (4 раза в секунду)
  };

  uint8_t  modes_count = modes[_blinkMode];

      if (millis() - tmr_Led > _blinkPeriod){
        tmr_Led = millis();
        
        // Режим светодиода ищем по битовой маске       
       if (modes_count & 1 << (blink_loop & 0x07)) 
          digitalWrite(_ledPin, HIGH);                 
       else  digitalWrite(_ledPin, LOW);

       blink_loop++;  
        }

}

// Добавление новых брелоков

void addCodeRF(){

  uint32_t transmitButtonSeting[3] = {0,0,0}; //Массив для хранения кодов кнопок настраиваемого передатчика
  uint8_t  progButton = 0;
  uint32_t timerSetting = millis();
  uint32_t timerRF = millis();
  bool settingComplite = false;
  blinkMode = 3;
  DEBUG_PRINTLN("Program RF mode");
  DEBUG_PRINTLN("Press button #1 or hold BRAKE for erse all codes");
   while ((millis() - timerSetting < 30000) && !settingComplite) // Задаем время на программирование
    {

    ledBlink(LED_PIN, 150, blinkMode);

   // Проверяем долгое нажатие кнопки Brake, если да, то стираем коды всех передатчков
     if (!digitalRead(BRAKE_BUTTON_PIN) && !brakeButtonPinState && millis() - btnBrakeTimer > 100) {
       brakeButtonPinState = true;
       btnBrakeTimer = millis();
      }
     if (!digitalRead(BRAKE_BUTTON_PIN) && brakeButtonPinState && millis() - btnBrakeTimer > 5000)
      {
       brakeButtonPinState = false;
       btnBrakeTimer = millis();
       digitalWrite(LED_PIN, HIGH);
        // Записываем в EEPROM нули
        for (uint8_t i = 0; i<36; i++)
        {
          EEPROM.put(i, 0);
        }
        DEBUG_PRINTLN("EEPROM ERASED");
        settingComplite = true;
        delay(3000);  
      }
     if (digitalRead(BRAKE_BUTTON_PIN) && brakeButtonPinState && millis() - btnBrakeTimer > 100) {
       brakeButtonPinState = false;
       btnBrakeTimer = millis();
      }
   
   // Читаем сигнал с передатчика. Интервал задается в настройках
      if (millis() - timerRF > RX_POLL_INTERVAL)
      {
        timerRF = millis();
        if (mySwitch.available())
        { 
          switch (progButton)
          {
            case 0:
            {
              if (checkCode(mySwitch.getReceivedValue()))
                {
                  transmitButtonSeting[progButton] = (mySwitch.getReceivedValue());
                  DEBUG_PRINT("Saved code #1 ");
                  DEBUG_PRINTLN(transmitButtonSeting[progButton]);
                  DEBUG_PRINTLN("Press button #2");
                  progButton = 1;
                  blinkMode = 4;
                  break;
                } 
                } 
                break;
              case 1:
                 {
                  if ((checkCode(mySwitch.getReceivedValue())) && (mySwitch.getReceivedValue() != transmitButtonSeting[0]))
                  {
                    transmitButtonSeting[progButton] = (mySwitch.getReceivedValue());
                    DEBUG_PRINT("Saved code #2 ");
                    DEBUG_PRINTLN(transmitButtonSeting[progButton]);
                    DEBUG_PRINTLN("Press button #3");
                    progButton = 2;
                    blinkMode = 5;
                    break;
                  } 
                 } 
                 break;
              case 2:
                 {
                    if ((checkCode(mySwitch.getReceivedValue())) && (mySwitch.getReceivedValue() != transmitButtonSeting[0]) && (mySwitch.getReceivedValue() != transmitButtonSeting[1]))
                    {
                       transmitButtonSeting[progButton] = (mySwitch.getReceivedValue());
                       DEBUG_PRINT("Saved code #3 ");
                       DEBUG_PRINTLN(transmitButtonSeting[progButton]);
                       // Записываем коды кнопок в ПЗУ //
                             for (uint8_t i = 2; i >0; i--)
                                {
                                  transmitButtonCode_A[i] = transmitButtonCode_A[i-1];
                                  transmitButtonCode_B[i] = transmitButtonCode_B[i-1];
                                  transmitButtonCode_AB[i] = transmitButtonCode_AB[i-1];
                                }
                          transmitButtonCode_A[0] = transmitButtonSeting[0];
                          transmitButtonCode_B[0] = transmitButtonSeting[1];
                          transmitButtonCode_AB[0] = transmitButtonSeting[2];

                          EEPROM.put(0, transmitButtonCode_A);
                          EEPROM.put(12, transmitButtonCode_B);
                          EEPROM.put(24, transmitButtonCode_AB);
                          settingComplite = true;
                       //////////////////////////////////////
                       break;
                    } 
                 } 
                 break;
             }  
          } 
            mySwitch.resetAvailable();
       }
    }
    digitalWrite(LED_PIN, LOW);
}

void GetTransmitButton (){
 static uint32_t TimerRF;
  if (millis() - TimerRF > 150){
   TimerRF = millis();
   if (mySwitch.available())
   {
      for (uint8_t i = 0; i < 3; i++) 
       {
         if (mySwitch.getReceivedValue() == transmitButtonCode_A[i])
            {
              transmitButtonA = true;
              transmitButtonB = false;
              transmitButtonAB = false;
              break;
            }
         else if ((mySwitch.getReceivedValue() == transmitButtonCode_B[i]))
            {
              transmitButtonA = false;
              transmitButtonB = true;
              transmitButtonAB = false;
              break;
            }
         else if ((mySwitch.getReceivedValue() == transmitButtonCode_AB[i]))
            {
              transmitButtonA = false;
              transmitButtonB = false;
              transmitButtonAB = true;
              break;
            }
        }
      mySwitch.resetAvailable();
   }
   else{
    transmitButtonA = false;
    transmitButtonB = false;
    transmitButtonAB = false;
    }
  }

}

void trimCalibration() {

  DEBUG_PRINTLN("Trim calibration begin");

    for (int i = 0; i < 50; i++) // Производим 50 пустых измерений
      {
        BMI160.readMotionSensor(aix, aiy, aiz, gix, giy, giz);
        delay(IMU_PERIOD);
      }

    int32_t aix_mid_sum = 0;
    int32_t aiy_mid_sum = 0;
    int32_t aiz_mid_sum = 0;
    
    int32_t gix_mid_sum = 0;
    int32_t giy_mid_sum = 0;
    int32_t giz_mid_sum = 0;

    for (int i = 0; i < CALIB_STEP; i++)
      {
        BMI160.readMotionSensor(aix, aiy, aiz, gix, giy, giz);

        aix_mid_sum += aix;
        aiy_mid_sum += aiy;
        aiz_mid_sum += (aiz - 16384);

        gix_mid_sum += gix;
        giy_mid_sum += giy;
        giz_mid_sum += giz;

        delay(IMU_PERIOD);
        DEBUG_PRINT("#");
      }

    trimAccelOffsetX = aix_mid_sum / CALIB_STEP;
    trimAccelOffsetY = aiy_mid_sum / CALIB_STEP;
    trimAccelOffsetZ = aiz_mid_sum / CALIB_STEP;

    trimGyroOffsetX = gix_mid_sum / CALIB_STEP;
    trimGyroOffsetY = giy_mid_sum / CALIB_STEP;
    trimGyroOffsetZ = giz_mid_sum / CALIB_STEP;
    
  }

void calibrateIMU()
{
  DEBUG_PRINTLN("Calibration begin");
  
  ledBlinkN(LED_PIN, 500, 10);   // Мигаем светодиодом 10 раз, начало процесса калибровки
  digitalWrite(LED_PIN, HIGH);   //Зажигаем светодод - калибровка
  
  DEBUG_PRINTLN("About to calibrate. Make sure your board is stable and upright");
  delay(1000);

  BMI160.autoCalibrateGyroOffset();
  DEBUG_PRINTLN(" Done");

  DEBUG_PRINTLN("Starting ACCELERATION calibration and enabling offset compensation...");
  BMI160.autoCalibrateAccelerometerOffset(X_AXIS, 0);
  BMI160.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
  BMI160.autoCalibrateAccelerometerOffset(Z_AXIS, 1); // Ось Z расположена вертикально
    
  DEBUG_PRINTLN("Done");

#ifdef TRIM_CALIBRATION
  trimCalibration();
#endif
  
  EEPROM.put(36, BMI160.getAccelerometerOffset(X_AXIS));
  EEPROM.put(40, BMI160.getAccelerometerOffset(Y_AXIS));
  EEPROM.put(44, BMI160.getAccelerometerOffset(Z_AXIS));
  EEPROM.put(48, BMI160.getGyroOffset(X_AXIS));
  EEPROM.put(52, BMI160.getGyroOffset(Y_AXIS));
  EEPROM.put(56, BMI160.getGyroOffset(Z_AXIS));

#ifdef TRIM_CALIBRATION
  EEPROM.put(65, trimAccelOffsetX);
  EEPROM.put(66, trimAccelOffsetY);
  EEPROM.put(67, trimAccelOffsetZ);
  EEPROM.put(68, trimGyroOffsetX);
  EEPROM.put(69, trimGyroOffsetY);
  EEPROM.put(70, trimGyroOffsetZ);
#elif
  EEPROM.put(65, 0);
  EEPROM.put(66, 0);
  EEPROM.put(67, 0);
  EEPROM.put(68, 0);
  EEPROM.put(69, 0);
  EEPROM.put(70, 0);
#endif

#ifdef DEBUG   // Печатаем калибровочные коэффициенты
  // Заголовок
  Serial.println();
  Serial.println(F("NEW         \tX\tY\tZ"));

  // Accel Offset
  Serial.print(F("Accel Offset\t"));
  Serial.print(BMI160.getAccelerometerOffset(X_AXIS)); Serial.print(F("\t"));
  Serial.print(BMI160.getAccelerometerOffset(Y_AXIS)); Serial.print(F("\t"));
  Serial.println(BMI160.getAccelerometerOffset(Z_AXIS));

  // Gyro Offset  
  Serial.print(F("Gyro Offset\t"));
  Serial.print(BMI160.getGyroOffset(X_AXIS)); Serial.print(F("\t"));
  Serial.print(BMI160.getGyroOffset(Y_AXIS)); Serial.print(F("\t"));
  Serial.println(BMI160.getGyroOffset(Z_AXIS));

  // Trim Accel
  Serial.print(F("Trim Accel\t"));
  Serial.print(trimAccelOffsetX); Serial.print(F("\t"));
  Serial.print(trimAccelOffsetY); Serial.print(F("\t"));
  Serial.println(trimAccelOffsetZ);

  // Trim Gyro
  Serial.print(F("Trim Gyro\t"));
  Serial.print(trimGyroOffsetX); Serial.print(F("\t"));
  Serial.print(trimGyroOffsetY); Serial.print(F("\t"));
  Serial.println(trimGyroOffsetZ);
#endif

digitalWrite(LED_PIN, LOW);    //Гасим светодод
ledBlinkN(LED_PIN, 150, 5);    // Мигаем светодиодом 5 раз, окончание процесса калибровки
}

//////////// Стабилизация /////////////

void measurement(){
  /// Читаем данные с IMU ///
  if (imu_t + IMU_PERIOD < millis())
    { 
      tDelta = millis() - imu_t; // вычисляем дельту времени в миллисекундах
      imu_t = millis();
      BMI160.readMotionSensor(aix, aiy, aiz, gix, giy, giz);

      float ax_mpu = filterAxelX.filtered(aix - trimAccelOffsetX);
      float ay_mpu = filterAxelY.filtered(aiy - trimAccelOffsetY);
      float az_mpu = filterAxelZ.filtered(aiz - trimAccelOffsetZ);

      float gx_mpu = filterGyroX.filtered(gix - trimGyroOffsetX) * TO_RAD / 131.0;
      float gy_mpu = filterGyroY.filtered(giy - trimGyroOffsetY) * TO_RAD / 131.0;
      float gz_mpu = filterGyroZ.filtered(giz - trimGyroOffsetZ) * TO_RAD / 131.0;

      if ((abs(gix - trimGyroOffsetX)) <= GYRO_DEAD_ZONE) gx_mpu = 0;
      if ((abs(giy - trimGyroOffsetY)) <= GYRO_DEAD_ZONE) gy_mpu = 0;
      if ((abs(giz - trimGyroOffsetZ)) <= GYRO_DEAD_ZONE) gz_mpu = 0;

        // вызываем алгоритм фильтра Маджвика
        MadgwickAHRSupdateIMU(tDelta/1000.0, gx_mpu, gy_mpu, gz_mpu, ax_mpu, ay_mpu, az_mpu);
        quat[0] = q0; quat[1] = q1; quat[2] = q2; quat[3] = q3;
        // преобразуем кватернион в углы Эйлера
        quat2Euler(&quat[0], &imu[0]);
        currentAngle = imu[2] / TO_RAD;

    }
  }

      /// Управляем мотором ///
  void stabilization() {
    if (motor_t + MOTOR_PERIOD < millis())
    {
      motor_t = millis();
          
          if (abs(currentAngle - basePoint) > NEUTRAL_ZONE)
          {
            if (abs(currentAngle - basePoint) > 180)
            { 
              (currentAngle > basePoint) ? (correctionValue = (currentAngle - basePoint) - 360) : (correctionValue = (currentAngle - basePoint) + 360);
            }
            else correctionValue = currentAngle - basePoint;
            basePoint = currentAngle;
                stepper.setAcceleration(0);
                stepper.setMaxSpeed(abs(correctionValue  * ANGLE_TO_STEP * GEAR_RATIO) * 4.5); // Вычисляем скорость поворота мотора, для этого умножаем на величину (1000ms / период управляющего воздействия)
                stepper.setTarget(MOTOR_DIRECTION * correctionValue * ANGLE_TO_STEP * GEAR_RATIO, RELATIVE);
            
          }
          else correctionValue = 0;

      #ifdef DEBUG
        Serial.print(F("RAW Gyro Z "));
        Serial.print(giz);
        Serial.print(F("\t Angle IMU "));
        Serial.print(currentAngle);
        Serial.print(F("\t correctionValue "));
        Serial.println(correctionValue);
      #endif

    }
  }

void setup() {
  DEBUG_BEGIN(115200);
  pinMode(R_BUTTON_PIN, INPUT_PULLUP);
  pinMode(L_BUTTON_PIN, INPUT_PULLUP);
  pinMode(SCAN_BUTTON_PIN, INPUT_PULLUP);
  pinMode(BRAKE_BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  pinMode(IMU_CS_PIN, OUTPUT);

  digitalWrite(IMU_CS_PIN, LOW);    // При включении на CS должен быть низнкий логический уровень
  delay(50);                     
  digitalWrite(IMU_CS_PIN, HIGH);   // Переводим BMI160 в SPI режим высоким фронтом на CS
  delay(10);
 
  // Инициализация BMI160
  DEBUG_PRINT("IMU initialization...");
 
  if (checkBMI160())
    {
        enableIMU = true;
        DEBUG_PRINTLN("successful");
    }
  else
    {
        enableIMU = false;
        DEBUG_PRINTLN("failed");
        ledBlinkN(LED_PIN, 70, 10);
    }


enableIMU = true;

  if (enableIMU)
    {
      BMI160.begin(BMI160GenClass::SPI_MODE, IMU_CS_PIN);
      BMI160.setAccelerometerRate(25);
      BMI160.setAccelerometerRange(2);
      BMI160.setAccelDLPFMode(BMI160_DLPF_MODE_OSR4);
      DEBUG_PRINT("AccelDLPFMode = ");
      DEBUG_PRINTLN(BMI160.getAccelDLPFMode());
  
      // Настройка гироскопа
      BMI160.setGyroRate(25);
      BMI160.setGyroRange(250);
      BMI160.setGyroDLPFMode(BMI160_DLPF_MODE_OSR4);
      DEBUG_PRINT("GyroDLPFMode = ");
      DEBUG_PRINTLN(BMI160.getGyroDLPFMode());
      #ifdef DEBUG
        delay(2000);
      #endif

    }

 // Настройка шагового мотора
  stepper.autoPower(!holdMode);
  //stepper.autoPower(false);
  holdMode ? stepper.enable() : stepper.disable();
  stepper.setAcceleration(ACCELERATION * GEAR_RATIO);   // установка ускорения в шагах/сек/сек
  stepper.setMaxSpeed(SPEED_MAX * GEAR_RATIO);          // установка скорости в шагах/сек/сек
  stepper.disable();
  mySwitch.enableReceive(0);                            // Инициализация приемника на pin 2 (Interrupt 0)
  EEPROM.get(0, transmitButtonCode_A);                  // Читаем из ПЗУ массив кодов кнопок А
  EEPROM.get(12, transmitButtonCode_B);                 // Читаем из ПЗУ массив кодов кнопок В
  EEPROM.get(24, transmitButtonCode_AB);                // Читаем из ПЗУ массив кодов кнопок АВ

  // Вход в режим программирования пультов
  if (!digitalRead(BRAKE_BUTTON_PIN) && digitalRead(SCAN_BUTTON_PIN)){
  brakeButtonPinState = true;
  addCodeRF();
  EEPROM.get(0, transmitButtonCode_A);                  // Читаем из ПЗУ массив кодов кнопок А
  EEPROM.get(12, transmitButtonCode_B);                 // Читаем из ПЗУ массив кодов кнопок В
  EEPROM.get(24, transmitButtonCode_AB);                // Читаем из ПЗУ массив кодов кнопок АВ}
  }
  
  // Вход в режим калибровки гироскопа
  if (!digitalRead(BRAKE_BUTTON_PIN) && !digitalRead(SCAN_BUTTON_PIN) && enableIMU)
    {
      while ((!digitalRead(BRAKE_BUTTON_PIN) && !digitalRead(SCAN_BUTTON_PIN) && enableIMU))
      {
        ledBlink(LED_PIN, 150, 6);
      }
      
     calibrateIMU();
    }

  // Читаем значения оффсетов из ПЗУ
  EEPROM.get(36, accelOffsetX);
  EEPROM.get(40, accelOffsetY);
  EEPROM.get(44, accelOffsetZ);
  EEPROM.get(48, gyroOffsetX);
  EEPROM.get(52, gyroOffsetY);
  EEPROM.get(56, gyroOffsetZ);
  EEPROM.get(65, trimAccelOffsetX);
  EEPROM.get(66, trimAccelOffsetY);
  EEPROM.get(67, trimAccelOffsetZ);
  EEPROM.get(68, trimGyroOffsetX);
  EEPROM.get(69, trimGyroOffsetY);
  EEPROM.get(70, trimGyroOffsetZ);

  if (enableIMU)
    {
  // Передаем оффсеты в IMU
      BMI160.setAccelerometerOffset(X_AXIS, accelOffsetX);
      BMI160.setAccelerometerOffset(Y_AXIS, accelOffsetY);
      BMI160.setAccelerometerOffset(Z_AXIS, accelOffsetZ);
      BMI160.setGyroOffset(X_AXIS, gyroOffsetX);
      BMI160.setGyroOffset(Y_AXIS, gyroOffsetY);
      BMI160.setGyroOffset(Z_AXIS, gyroOffsetZ);

    #ifdef DEBUG
      // Заголовок
      Serial.println();
      Serial.println(F("EEPROM  \tX\tY\tZ"));

      // Accel Offset
      Serial.print(F("Accel Offset\t"));
      Serial.print(BMI160.getAccelerometerOffset(X_AXIS)); Serial.print(F("\t"));
      Serial.print(BMI160.getAccelerometerOffset(Y_AXIS)); Serial.print(F("\t"));
      Serial.println(BMI160.getAccelerometerOffset(Z_AXIS));

      // Gyro Offset  
      Serial.print(F("Gyro Offset\t"));
      Serial.print(BMI160.getGyroOffset(X_AXIS)); Serial.print(F("\t"));
      Serial.print(BMI160.getGyroOffset(Y_AXIS)); Serial.print(F("\t"));
      Serial.println(BMI160.getGyroOffset(Z_AXIS));

      // Trim Accel
      Serial.print(F("Trim Accel\t"));
      Serial.print(trimAccelOffsetX); Serial.print(F("\t"));
      Serial.print(trimAccelOffsetY); Serial.print(F("\t"));
      Serial.println(trimAccelOffsetZ);

      // Trim Gyro
      Serial.print(F("Trim Gyro\t"));
      Serial.print(trimGyroOffsetX); Serial.print(F("\t"));
      Serial.print(trimGyroOffsetY); Serial.print(F("\t"));
      Serial.println(trimGyroOffsetZ);
    #endif

    }

  enableIMU ? stabilizationMode = EEPROM.read(60) : stabilizationMode = false; // Читаем режим
  //holdMode = true;
  if (stabilizationMode && holdMode) blinkMode = 2;
  else blinkMode = holdMode;
  EEPROM.read(61) == 3 ? scanAngle = SCAN_ANGLE_1 : scanAngle = SCAN_ANGLE_2;

  DEBUG_PRINTLN();
  DEBUG_PRINTLN("Run");
}

void loop() {
  stepper.tick();                      // Тикаем мотором
  GetTransmitButton();                 // Проверка сигнала от пульта
   if (enableIMU)  measurement();      // Измеряем угол

  // Опрашиваем потенциометр SPEED

  static uint32_t tmr1;
  if (millis() - tmr1 > 100) {
    tmr1 = millis();
    speedPotValue = analogRead(SPEED_POT_PIN);
    if (!(stabilizationMode && holdMode && !scanMode)) stepper.setMaxSpeed(map(speedPotValue, 0, 1023, SPEED_MIN, SPEED_MAX) * GEAR_RATIO);
    if (scanMode)
    {
        if ( abs(speedPotValue - speedPotValuePrevius) > 5 )
        {
          speedChange = true;
          DEBUG_PRINT("Speed change ");
          DEBUG_PRINTLN(speedPotValue);
        }
        if (speedChange)
        {
            stepper.setMaxSpeed(map(speedPotValue, 0, 1023, SPEED_MIN, SPEED_MAX) * GEAR_RATIO);
        }
        else
        {   
            int EepromSpeed;
            EEPROM.get (62, EepromSpeed);
            stepper.setMaxSpeed(map(EepromSpeed, 0, 1023, SPEED_MIN, SPEED_MAX) * GEAR_RATIO);
        }
       
    }
  }

  // Кнопка BRAKE нажата

  if (!digitalRead(BRAKE_BUTTON_PIN) && !brakeButtonPinState && !scanMode) {
    brakeButtonPinState = true;
    btnBrakeTimer = millis();
    delay(20);
  }

  if (!digitalRead(BRAKE_BUTTON_PIN) && !scanMode && millis() - btnBrakeTimer > 2000) blinkMode = 6; // Если кнопка зажата больше 2 секунд мигаем светодиодом три раза

 // Кнопка BRAKE отпущена

  if (digitalRead(BRAKE_BUTTON_PIN) && brakeButtonPinState && !scanMode)
    {
      brakeButtonPinState = false;
      if (millis() - btnBrakeTimer > 2000 && enableIMU)
        { 
          stabilizationMode = EEPROM.read(60);
          stabilizationMode = !stabilizationMode;
          EEPROM.write(60, stabilizationMode);
          stabilizationMode = EEPROM.read(60);
          if (stabilizationMode && holdMode) blinkMode = 2;
          else blinkMode = holdMode;
          #ifdef DEBUG
            Serial.print(F("Stabilization Mode "));
            Serial.println(EEPROM.read(60));
          #endif
        }
      else
        {
          holdMode = !holdMode;
          stepper.autoPower(!holdMode);
          holdMode ? stepper.enable() : stepper.disable();
          scanMode = false;
          stepper.brake();
            if (stabilizationMode && holdMode) blinkMode = 2;
            else blinkMode = holdMode;
          #ifdef DEBUG
            Serial.println(holdMode ? "Hold enable" : "Hold disable");
          #endif
        basePoint = currentAngle;
        }
        
    }
  
  // Правая кнопка нажата
  if ((!digitalRead(R_BUTTON_PIN) || transmitButtonA) && !rightButtonPinState) {
    rightButtonPinState = true;
    stabilizationMode = false;
    if (scanMode && speedChange)
    {
      EEPROM.put(62, speedPotValue);
      speedChange = false;
      DEBUG_PRINTLN("New speed saved");
    }

    if (scanMode) {
      scanMode = false;
      stepper.brake();
      stepper.autoPower(!holdMode);
    }
    else {
      stepper.setAcceleration(ACCELERATION * GEAR_RATIO);
      stepper.setTarget(-1*round(360 * ANGLE_TO_STEP * GEAR_RATIO), RELATIVE); 
    }
    delay(20);
    } 

  // Правая кнопка отпущена
  
  if ((digitalRead(R_BUTTON_PIN) && !transmitButtonA && !transmitButtonB && !transmitButtonAB ) && rightButtonPinState) {
    rightButtonPinState = false;
        if (!scanMode)
        {
          stepper.brake();
          enableIMU ? stabilizationMode = EEPROM.read(60) : stabilizationMode = false;
          if (stabilizationMode && holdMode) blinkMode = 2;
          else blinkMode = holdMode;
          basePoint = currentAngle;
        }
    }
  
  // Левая кнопка нажата
    if ((!digitalRead(L_BUTTON_PIN) || transmitButtonB) && !leftButtonPinState) {
    leftButtonPinState = true;
    stabilizationMode = false;
    if (scanMode && speedChange)
    {
      EEPROM.put(62, speedPotValue);
      speedChange = false;
      DEBUG_PRINTLN("New speed saved");
    }

    if (scanMode) {
      scanMode = false;
      stepper.brake();
      stepper.autoPower(!holdMode);
    }
    else {
      stepper.setAcceleration(ACCELERATION * GEAR_RATIO);
      stepper.setTarget(round(360 * ANGLE_TO_STEP * GEAR_RATIO), RELATIVE); 
    }
    delay(20);

    }

  // Левая кнопка отпущена
  if ((digitalRead(L_BUTTON_PIN) && !transmitButtonA && !transmitButtonB && !transmitButtonAB) && leftButtonPinState) {
    leftButtonPinState = false;
        if (!scanMode) {
          stepper.brake();
          enableIMU ? stabilizationMode = EEPROM.read(60) : stabilizationMode = false;
          if (stabilizationMode && holdMode) blinkMode = 2;
          else blinkMode = holdMode;
          basePoint = currentAngle;
        }
  } 

  // Одновременно нажаты левая и правая кнопки (Кнопка сканирования)

   if (((!digitalRead(R_BUTTON_PIN) && !digitalRead(L_BUTTON_PIN)) || !digitalRead(SCAN_BUTTON_PIN) || transmitButtonAB) && !scanButtonPinState)
   {
      scanButtonPinState = true;
      btnScanTimer = millis();
      stabilizationMode = false;
      stepper.brake();
      stepper.setAcceleration(ACCELERATION * GEAR_RATIO);

    if (scanMode) 
      {
        stepper.reset();                        // Если мы уже в режиме сканирования, то сбрасываем начальную позицию
        DEBUG_PRINTLN("Zero point");  
      }
    else 
      { 
        scanMode = true;
        stepper.autoPower(false);
        speedPotValuePrevius = speedPotValue;   // Запоминаем текущее состояние потенциометра
        DEBUG_PRINTLN("Scan mode on");
      }

    blinkMode = EEPROM.read(61);
    dir = true; 
    delay(500);
  }

  if (( (!digitalRead(R_BUTTON_PIN) && !digitalRead(L_BUTTON_PIN)) || !digitalRead(SCAN_BUTTON_PIN) || transmitButtonAB) && (millis() - btnScanTimer > 2000)) blinkMode = 6;

  // Отпускаем кнопку сканирования)
   if ((digitalRead(R_BUTTON_PIN) && digitalRead(L_BUTTON_PIN)) && digitalRead(SCAN_BUTTON_PIN) && !transmitButtonAB && scanButtonPinState){
    scanButtonPinState = false;
    if (millis() - btnScanTimer > 2000)
        {
          EEPROM.read(61) == 3 ? EEPROM.write(61, 4) : EEPROM.write(61, 3);
          EEPROM.read(61) == 3 ? scanAngle = SCAN_ANGLE_1 : scanAngle = SCAN_ANGLE_2;
          blinkMode = EEPROM.read(61);
          DEBUG_PRINTLN(EEPROM.read(61) == 3 ? "SCAN_ANGLE_1" : "SCAN_ANGLE_2");
        }
     }
 

// Режим сканирования

if (scanMode)
  {
       if (!stepper.tick())
       { dir = !dir;
         delay(4);                          // Задержка при смене направления, чтобы не было уплывания средней точки
         digitalWrite(DIR_PIN, dir);
         delay(4);
         stepper.setTarget(dir ? round(scanAngle * ANGLE_TO_STEP * GEAR_RATIO) : -1*round(scanAngle * ANGLE_TO_STEP * GEAR_RATIO), ABSOLUTE);
         stepper.tick();
       } 
   } 

  // Режим стабилизации
  if (stabilizationMode && holdMode && enableIMU) stabilization();

   ledBlink(LED_PIN, 150, blinkMode);       // Мигаем светодиодом
}

/*
Распределение памяти
0 - 3    Кнопка А брелок #1
4 - 7    Кнопка А брелок #2
8 - 11   Кнопка А брелок #3
12 - 15  Кнопка B брелок #1
16 - 19  Кнопка B брелок #2
20 - 23  Кнопка А брелок #3
24 - 27  Кнопка A+B брелок #1
28 - 31  Кнопка А+B брелок #2
32 - 35  Кнопка А+B брелок #3
36 - 39  AccelerometrOffset X (float)
40 - 43  AccelerometrOffset Y (float)
44 - 47  AccelerometrOffset Y (float)
48 - 51  GyroOffset X (float)
52 - 55  GyroOffset Y (float)
56 - 59  GyroOffset Y (float)
60       stabilizationMode (bool)
61       Угол сканирования (2 или 3 он же режим мигания светодиода в scanMode)
62 - 64  Положение потенциометра (int)
65       trimAccelOffsetX
66       trimAccelOffsetY
67       trimAccelOffsetZ
68       trimGyroOffsetX
69       trimGyroOffsetY
70       trimGyroOffsetZ
*/