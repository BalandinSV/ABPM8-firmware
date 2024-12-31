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

//////////////////// СЕКЦИЯ НАСТРОЕК ////////////////////////

#define RightButton A0
#define LeftButton A1
#define ScanButton A2
#define BrakeButton A3
#define SpeedPotPin A7
#define LedPin A5

#define StepPin 6
#define DirPin 7
#define EnablePin 8
#define MicroStep 6400 //6400 импульсов на оборот. MRES = 32 Для TMC2160 (M0-OFF, M1-ON)

#define ScanAngle_1 35 // Максимальный угол сканирования половина сектора сканирования
#define ScanAngle_2 80 // Максимальный угол сканирования половина сектора сканирования
#define SpeedMin 50 // Минимальная угловая скорость шаг/сек
#define SpeedMax 1600 // Максимальная угловая скорость шаг/сек
#define Acceleration 0 // Ускорение. 0 - выключено
#define NeutralZone 0.50 // Зона нечуствительности в режиме стабилизации в градусах,
                         // чем меньше, тем плавнее, но больше потребление тока
//#define AngleToStep 4.44 // 1600/360=4.44 для 1600 шаг/об
//#define AngleToStep 8.89 // 3200/360=8.89 для 3200 шаг/об
#define AngleToStep 18.5 // 6400/360=17.78 для 6400 шаг/об
//#define AngleToStep 35.55 // 12800/360=35.55 для 12800 шаг/об
#define GearRatio 1 // Передаточное число редуктора

//#define LPF 0.9 // Коэффициент фильтра нижних частот
//#define KalmanNoise 20 // разброс измерения: шум измерений
//#define KalmanSpeed 0.8 // скорость изменения значений: 0.001-1

#define TO_RAD 0.01745329252f // Коэффициент для перевода градусов в радианы
#define imuPeriod 10 // Период опроса IMU мс
#define MotorPeriod 250 // Период управляющего воздействия мс

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

bool TransmitButton_A = false;
bool TransmitButton_B = false;
bool TransmitButton_AB = false;

bool RightButtonState = false;
bool LeftButtonState = false;
bool ScanButtonState = false;
bool BrakeButtonState = false;
bool ScanMode = false;
bool HoldMode = true;
bool StabilizationMode = false;
bool dir = true;
bool IMU_Enable = false;
bool Speed_Change = false;

long TransmitButton;
float AccelOffsetX;             // Переменная для хранения калибровки гироскопа XAccelOffset
float AccelOffsetY;             // Переменная для хранения калибровки гироскопа YAccelOffset
float AccelOffsetZ;             // Переменная для хранения калибровки гироскопа ZAccelOffset
float GyroOffsetX;              // Переменная для хранения калибровки гироскопа XGyroOffset
float GyroOffsetY;              // Переменная для хранения калибровки гироскопа YGyroOffset
float GyroOffsetZ;              // Переменная для хранения калибровки гироскопа ZGyroOffset

uint8_t blink_mode = 0;         // Режим мигания светодиодом
int SpeedPotValue = 0;          // Значение с АЦП потенциометра SPEED
int SpeedPotValuePrevius = 0;   // Для определения изменения скорости в режиме SCAN


uint32_t btnBrakeTimer = 0;     // Время нажатия кнопки BAKE
uint32_t btnScanTimer = 0;      // Время нажатия кнопки SCAN
uint32_t imu_t = 0;             // Переменная для хранения времени опроса IMU
uint32_t motor_t = 0;           // Переменная для хранения времени корректировки мотором
int32_t ScanAngle = 120;
uint32_t TransmitButtonCode_A[3]; //Массив для хранения кодов кнопок А разных брелоков 3 х 4 байта 
uint32_t TransmitButtonCode_B[3]; //Массив для хранения кодов кнопок B разных брелоков 3 х 4 байта
uint32_t TransmitButtonCode_AB[3]; //Массив для хранения кодов кнопок А+B разных брелоков 3 х 4 байта

float BasePoint = 0;             // Точка отсчета для удержания
float CurrentAngle = 0;          // Текущий курс
float Correct = 0;               // Величина отклонения курса

// Переменные для IMU

float tdelta;                     // Время между измерениями
int aix, aiy, aiz;                // Сырые данные с акселерометра
int gix, giy, giz;                // Сырые данные с гироскопа
float imu[3];                     // Масив для записи текущих измерений
float quat[4];                    // Кватерионы

GStepper<STEPPER2WIRE> stepper(MicroStep, StepPin, DirPin, EnablePin); // Инициализируем драйвер
RCSwitch mySwitch = RCSwitch();   // Инициализируем приемник

GMedian3<int> filterAxelX;         // Фильтр акселерометр ось X
GMedian3<int> filterAxelY;         // Фильтр акселерометр ось Y
GMedian3<int> filterAxelZ;         // Фильтр акселерометр ось Z
GMedian3<int> filterGyroX;         // Фильтр гироскоп ось X
GMedian3<int> filterGyroY;         // Фильтр гироскоп ось Y
GMedian3<int> filterGyroZ;         // Фильтр гироскоп ось Z


// Мигаем светодиодом N раз
void LedBlink_N (uint8_t led_Pin, uint32_t blinkPeriod, uint8_t blink_count)
{
  bool _ledState = digitalRead(led_Pin);
  for (int i = 0; i < blink_count; i++)
  {
    _ledState = ! _ledState;
    digitalWrite(LedPin, _ledState);
    delay(blinkPeriod);
    _ledState = ! _ledState;
    digitalWrite(LedPin, _ledState);
    delay(blinkPeriod);
  }
}

// Проверяем дубли кодов передатчика
bool CheckCode (uint32_t CurrentCode)
{
  for (uint8_t i = 0; i < 3; i++) 
     {
      if ((CurrentCode == TransmitButtonCode_A[i]) || (CurrentCode == TransmitButtonCode_B[i]) || (CurrentCode == TransmitButtonCode_AB[i]))
         {
          DEBUG_PRINTLN("Duplicate code!");
          LedBlink_N(LedPin, 100, 5);
          return false;
         } 
     }
      return true;
}

// Мигаем светодиодом с разными режимами
void LedBlink(uint8_t led_Pin, uint32_t blink_Period, uint8_t blink_Mode)
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

  uint8_t  modes_count = modes[blink_Mode];

      if (millis() - tmr_Led > blink_Period){
        tmr_Led = millis();
        
        // Режим светодиода ищем по битовой маске       
       if (modes_count & 1 << (blink_loop & 0x07)) 
          digitalWrite(LedPin, HIGH);                 
       else  digitalWrite(LedPin, LOW);

       blink_loop++;  
        }

}

// Добавление новых брелоков

void Settings(){

  uint32_t TransmitButtonSeting[3] = {0,0,0}; //Массив для хранения кодов кнопок настраиваемого передатчика
  uint8_t  ProgButton = 0;
  uint32_t TimerSetting = millis();
  uint32_t TimerRF = millis();
  bool SettingComplite = false;
  blink_mode = 3;
  DEBUG_PRINTLN("Program RF mode");
  DEBUG_PRINTLN("Press button #1 or hold BRAKE for erse all codes");
   while ((millis() - TimerSetting < 30000) && !SettingComplite) // Задаем время на программирование
    {

    LedBlink(LedPin, 150, blink_mode);

   // Проверяем долгое нажатие кнопки Brake, если да, то стираем коды всех передатчков
     if (!digitalRead(BrakeButton) && !BrakeButtonState && millis() - btnBrakeTimer > 100) {
       BrakeButtonState = true;
       btnBrakeTimer = millis();
      }
     if (!digitalRead(BrakeButton) && BrakeButtonState && millis() - btnBrakeTimer > 5000)
      {
       BrakeButtonState = false;
       btnBrakeTimer = millis();
       digitalWrite(LedPin, HIGH);
        // Записываем в EEPROM нули
        for (uint8_t i = 0; i<36; i++)
        {
          EEPROM.put(i, 0);
        }
        DEBUG_PRINTLN("EEPROM ERASED");
        SettingComplite = true;
        delay(3000);  
      }
     if (digitalRead(BrakeButton) && BrakeButtonState && millis() - btnBrakeTimer > 100) {
       BrakeButtonState = false;
       btnBrakeTimer = millis();
      }
   
   // Читаем сигнал с передатчика каждые 150 мс
      if (millis() - TimerRF > 150)
      {
        TimerRF = millis();
        if (mySwitch.available())
        { 
          //DEBUG_PRINT("Resive code ");
          //DEBUG_PRINTLN(getReceivedValue());
          switch (ProgButton)
          {
            case 0:
            {
              if (CheckCode(mySwitch.getReceivedValue()))
                {
                  TransmitButtonSeting[ProgButton] = (mySwitch.getReceivedValue());
                  DEBUG_PRINT("Saved code #1 ");
                  DEBUG_PRINTLN(TransmitButtonSeting[ProgButton]);
                  DEBUG_PRINTLN("Press button #2");
                  ProgButton = 1;
                  blink_mode = 4;
                  break;
                } 
                } 
                break;
              case 1:
                 {
                  if ((CheckCode(mySwitch.getReceivedValue())) && (mySwitch.getReceivedValue() != TransmitButtonSeting[0]))
                  {
                    TransmitButtonSeting[ProgButton] = (mySwitch.getReceivedValue());
                    DEBUG_PRINT("Saved code #2 ");
                    DEBUG_PRINTLN(TransmitButtonSeting[ProgButton]);
                    DEBUG_PRINTLN("Press button #3");
                    ProgButton = 2;
                    blink_mode = 5;
                    break;
                  } 
                 } 
                 break;
              case 2:
                 {
                    if ((CheckCode(mySwitch.getReceivedValue())) && (mySwitch.getReceivedValue() != TransmitButtonSeting[0]) && (mySwitch.getReceivedValue() != TransmitButtonSeting[1]))
                    {
                       TransmitButtonSeting[ProgButton] = (mySwitch.getReceivedValue());
                       DEBUG_PRINT("Saved code #3 ");
                       DEBUG_PRINTLN(TransmitButtonSeting[ProgButton]);
                       // Записываем коды кнопок в ПЗУ //
                             for (uint8_t i = 2; i >0; i--)
                                {
                                  TransmitButtonCode_A[i] = TransmitButtonCode_A[i-1];
                                  TransmitButtonCode_B[i] = TransmitButtonCode_B[i-1];
                                  TransmitButtonCode_AB[i] = TransmitButtonCode_AB[i-1];
                                }
                          TransmitButtonCode_A[0] = TransmitButtonSeting[0];
                          TransmitButtonCode_B[0] = TransmitButtonSeting[1];
                          TransmitButtonCode_AB[0] = TransmitButtonSeting[2];

                          EEPROM.put(0, TransmitButtonCode_A);
                          EEPROM.put(12, TransmitButtonCode_B);
                          EEPROM.put(24, TransmitButtonCode_AB);
                          SettingComplite = true;
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
    digitalWrite(LedPin, LOW);
}

void GetTransmitButton (){
 static uint32_t TimerRF;
  if (millis() - TimerRF > 150){
   TimerRF = millis();
   if (mySwitch.available())
   {
      for (uint8_t i = 0; i < 3; i++) 
       {
         if (mySwitch.getReceivedValue() == TransmitButtonCode_A[i])
            {
              TransmitButton_A = true;
              TransmitButton_B = false;
              TransmitButton_AB = false;
              break;
            }
         else if ((mySwitch.getReceivedValue() == TransmitButtonCode_B[i]))
            {
              TransmitButton_A = false;
              TransmitButton_B = true;
              TransmitButton_AB = false;
              break;
            }
         else if ((mySwitch.getReceivedValue() == TransmitButtonCode_AB[i]))
            {
              TransmitButton_A = false;
              TransmitButton_B = false;
              TransmitButton_AB = true;
              break;
            }
        }
      mySwitch.resetAvailable();
   }
   else{
    TransmitButton_A = false;
    TransmitButton_B = false;
    TransmitButton_AB = false;
    }
  }

}

void CalibrateIMU()
{
  DEBUG_PRINTLN("Calibration begin");
  // Мигаем светодиодом 10 раз, начало процесса калибровки
  LedBlink_N(LedPin, 500, 10);
  digitalWrite(LedPin, HIGH);   //Зажигаем светодод - калибровка
  
  DEBUG_PRINTLN("About to calibrate. Make sure your board is stable and upright");
  delay(1000);

  BMI160.autoCalibrateGyroOffset();
  DEBUG_PRINTLN(" Done");

  DEBUG_PRINTLN("Starting Acceleration calibration and enabling offset compensation...");
  BMI160.autoCalibrateAccelerometerOffset(X_AXIS, 0);
  BMI160.autoCalibrateAccelerometerOffset(Y_AXIS, 0);
  BMI160.autoCalibrateAccelerometerOffset(Z_AXIS, 1); // Ось Z расположена вертикально
    
  DEBUG_PRINTLN("Done");

  DEBUG_PRINTLN("Internal sensor offsets AFTER calibration...");

    //BMI160.setAccelOffsetEnabled(1);
    //BMI160.setGyroOffsetEnabled(1);
  
  DEBUG_PRINTLN("Calibration successful");
  
  EEPROM.put(36, BMI160.getAccelerometerOffset(X_AXIS));
  EEPROM.put(40, BMI160.getAccelerometerOffset(Y_AXIS));
  EEPROM.put(44, BMI160.getAccelerometerOffset(Z_AXIS));
  EEPROM.put(48, BMI160.getGyroOffset(X_AXIS));
  EEPROM.put(52, BMI160.getGyroOffset(Y_AXIS));
  EEPROM.put(56, BMI160.getGyroOffset(Z_AXIS));

#ifdef DEBUG
  Serial.print(F("XAccelOffset ")); Serial.println(BMI160.getAccelerometerOffset(X_AXIS));
  Serial.print(F("YAccelOffset ")); Serial.println(BMI160.getAccelerometerOffset(Y_AXIS));
  Serial.print(F("ZAccelOffset ")); Serial.println(BMI160.getAccelerometerOffset(Z_AXIS));
  Serial.print(F("XGyroOffset ")); Serial.println(BMI160.getGyroOffset(X_AXIS));
  Serial.print(F("YGyroOffset ")); Serial.println(BMI160.getGyroOffset(Y_AXIS));
  Serial.print(F("ZGyroOffset ")); Serial.println(BMI160.getGyroOffset(Z_AXIS)); 
#endif

digitalWrite(LedPin, LOW); //Гасим светодод
  // Мигаем светодиодом 5 раз, окончание процесса калибровки
  LedBlink_N(LedPin, 150, 5);
}

//////////// Стабилизация /////////////

void Measurement(){
  /// Читаем данные с IMU ///
  if (imu_t + imuPeriod < millis())
    { 
      tdelta = millis() - imu_t; // вычисляем дельту времени в миллисекундах
      imu_t = millis();
      BMI160.readMotionSensor(aix, aiy, aiz, gix, giy, giz);

      float gx_mpu = filterGyroX.filtered(gix) * TO_RAD / 131.0;
      float gy_mpu = filterGyroY.filtered(giy) * TO_RAD / 131.0;
      float gz_mpu = filterGyroZ.filtered(giz) * TO_RAD / 131.0;
      float ax_mpu = filterAxelX.filtered(aix);
      float ay_mpu = filterAxelY.filtered(aiy);
      float az_mpu = filterAxelZ.filtered(aiz);

        // вызываем алгоритм фильтра Маджвика
        MadgwickAHRSupdateIMU(tdelta/1000.0, gx_mpu, gy_mpu, gz_mpu, ax_mpu, ay_mpu, az_mpu);
        quat[0] = q0; quat[1] = q1; quat[2] = q2; quat[3] = q3;
        // преобразуем кватернион в углы Эйлера
        quat2Euler(&quat[0], &imu[0]);
        CurrentAngle = imu[2] / TO_RAD;

    }
  }

      /// Управляем мотором ///
  void Stabilization() {
    if (motor_t + MotorPeriod < millis())
    {
      motor_t = millis();
          
          if (abs(CurrentAngle - BasePoint) > NeutralZone)
          {
            if (abs(CurrentAngle - BasePoint) > 180)
            { 
              (CurrentAngle > BasePoint) ? (Correct = (CurrentAngle - BasePoint) - 360) : (Correct = (CurrentAngle - BasePoint) + 360);
            }
            else Correct = CurrentAngle - BasePoint;
            BasePoint = CurrentAngle;
                stepper.setAcceleration(0);
                stepper.setMaxSpeed(abs(Correct  * AngleToStep * GearRatio) * 4.5); // Вычисляем скорость поворота мотора, для этого умножаем на величину (1000ms / период управляющего воздействия)
                stepper.setTarget(-Correct * AngleToStep * GearRatio, RELATIVE);
            
          }
          else Correct = 0;
      // }
      #ifdef DEBUG
        Serial.print(F("Angle IMU "));
        Serial.print(CurrentAngle);
        Serial.print(F("\t Correction "));
        Serial.println(Correct);
      #endif

    }
  }

void setup() {
  DEBUG_BEGIN(115200);
  pinMode(RightButton, INPUT_PULLUP);
  pinMode(LeftButton, INPUT_PULLUP);
  pinMode(ScanButton, INPUT_PULLUP);
  pinMode(BrakeButton, INPUT_PULLUP);
  pinMode(LedPin, OUTPUT);
  DEBUG_PRINTLN("IMU initialization...");
 // Настройка гироскопа
  BMI160.begin(BMI160GenClass::SPI_MODE, /* SS pin# = */10);
  //BMI160.begin(BMI160GenClass::I2C_MODE);
  BMI160.testConnection() ? IMU_Enable = true : IMU_Enable = false;
  DEBUG_PRINTLN(IMU_Enable ? "IMI connection successful" : "IMU connection failed");
  if (IMU_Enable)
    {
      uint8_t dev_id = BMI160.getDeviceID();
      DEBUG_PRINTLN("DEVICE ID: ");
      DEBUG_PRINTLNF(dev_id, HEX);
  
      BMI160.setGyroRate(25);
      BMI160.setAccelerometerRate(25);
      BMI160.setAccelDLPFMode(1);
      DEBUG_PRINT("AccelDLPFMode = ");
      DEBUG_PRINTLN(BMI160.getAccelDLPFMode());

      BMI160.setAccelerometerRange(2);   // Set the accelerometer range to 2 g
      BMI160.setGyroRange(250);         // Set the gyroscope range to 250 degrees/second
      BMI160.setGyroDLPFMode(1);
      DEBUG_PRINT("GyroDLPFMode = ");
      DEBUG_PRINTLN(BMI160.getGyroDLPFMode());
    }

 // Настройка шагового мотора
  stepper.autoPower(true);
  stepper.setAcceleration(Acceleration * GearRatio);  // установка ускорения в шагах/сек/сек
  stepper.setMaxSpeed(SpeedMax * GearRatio);          // установка скорости в шагах/сек/сек
  stepper.disable();
  mySwitch.enableReceive(0);                          // Инициализация приемника на pin 2 (Interrupt 0)
  EEPROM.get(0, TransmitButtonCode_A);                // Читаем из ПЗУ массив кодов кнопок А
  EEPROM.get(12, TransmitButtonCode_B);               // Читаем из ПЗУ массив кодов кнопок В
  EEPROM.get(24, TransmitButtonCode_AB);              // Читаем из ПЗУ массив кодов кнопок АВ

  // Вход в режим программирования пультов
  if (!digitalRead(BrakeButton) && digitalRead(ScanButton)){
  BrakeButtonState = true;
  Settings();
  EEPROM.get(0, TransmitButtonCode_A); // Читаем из ПЗУ массив кодов кнопок А
  EEPROM.get(12, TransmitButtonCode_B); // Читаем из ПЗУ массив кодов кнопок В
  EEPROM.get(24, TransmitButtonCode_AB); // Читаем из ПЗУ массив кодов кнопок АВ}
  }
  
  // Вход в режим калибровки гироскопа
  if (!digitalRead(BrakeButton) && !digitalRead(ScanButton) && IMU_Enable)
    {
      while ((!digitalRead(BrakeButton) && !digitalRead(ScanButton) && IMU_Enable))
      {
        LedBlink(LedPin, 150, 6);
      }
      
     CalibrateIMU();
    }

  // Читаем значения оффсетов из ПЗУ
  EEPROM.get(36, AccelOffsetX);
  EEPROM.get(40, AccelOffsetY);
  EEPROM.get(44, AccelOffsetZ);
  EEPROM.get(48, GyroOffsetX);
  EEPROM.get(52, GyroOffsetY);
  EEPROM.get(56, GyroOffsetZ);

  if (IMU_Enable)
    {
      // Передаем оффсеты в IMU
      BMI160.setAccelerometerOffset(X_AXIS, AccelOffsetX);
      BMI160.setAccelerometerOffset(Y_AXIS, AccelOffsetY);
      BMI160.setAccelerometerOffset(Z_AXIS, AccelOffsetZ);
      BMI160.setGyroOffset(X_AXIS, GyroOffsetX);
      BMI160.setGyroOffset(Y_AXIS, GyroOffsetY);
      BMI160.setGyroOffset(Z_AXIS, GyroOffsetZ);

    #ifdef DEBUG
      Serial.println(F("Current offsets:"));
      Serial.print(F("XAccelOffset ")); Serial.println(BMI160.getAccelerometerOffset(X_AXIS));
      Serial.print(F("YAccelOffset ")); Serial.println(BMI160.getAccelerometerOffset(Y_AXIS));
      Serial.print(F("ZAccelOffset ")); Serial.println(BMI160.getAccelerometerOffset(Z_AXIS));
      Serial.print(F("XGyroOffset ")); Serial.println(BMI160.getGyroOffset(X_AXIS));
      Serial.print(F("YGyroOffset ")); Serial.println(BMI160.getGyroOffset(Y_AXIS));
      Serial.print(F("ZGyroOffset ")); Serial.println(BMI160.getGyroOffset(Z_AXIS)); 
    #endif
    }

  IMU_Enable ? StabilizationMode = EEPROM.read(60) : StabilizationMode = false; // Читаем режим
  HoldMode = true;
  if (StabilizationMode && HoldMode) blink_mode = 2;
  else blink_mode = HoldMode;
  EEPROM.read(61) == 3 ? ScanAngle = ScanAngle_1 : ScanAngle = ScanAngle_2;
  DEBUG_PRINTLN("Run");
}

void loop() {
  stepper.tick();                   // Тикаем мотором
  GetTransmitButton();              // Проверка сигнала от пульта
   if (IMU_Enable)  Measurement();  // Измеряем угол

  // Опрашиваем потенциометр SPEED

  static uint32_t tmr1;
  if (millis() - tmr1 > 100) {
    tmr1 = millis();
    SpeedPotValue = analogRead(SpeedPotPin);
    if (!(StabilizationMode && HoldMode && !ScanMode)) stepper.setMaxSpeed(map(SpeedPotValue, 0, 1023, SpeedMin, SpeedMax) * GearRatio);
    if (ScanMode)
    {
        if ( abs(SpeedPotValue - SpeedPotValuePrevius) > 5 )
        {
          Speed_Change = true;
          DEBUG_PRINTLN("Speed change");
        }
        if (Speed_Change)
        {
            stepper.setMaxSpeed(map(SpeedPotValue, 0, 1023, SpeedMin, SpeedMax) * GearRatio);
        }
        else
        {   
            int EepromSpeed;
            EEPROM.get (62, EepromSpeed);
            stepper.setMaxSpeed(map(EepromSpeed, 0, 1023, SpeedMin, SpeedMax) * GearRatio);
        }
       
    }
  }

  // Кнопка BRAKE нажата

  if (!digitalRead(BrakeButton) && !BrakeButtonState && !ScanMode) {
    BrakeButtonState = true;
    btnBrakeTimer = millis();
    delay(50);
  }

  if (!digitalRead(BrakeButton) && !ScanMode && millis() - btnBrakeTimer > 2000) blink_mode = 6; // Если кнопка зажата больше 2 секунд мигаем светодиодом три раза

 // Кнопка BRAKE отпущена

  if (digitalRead(BrakeButton) && BrakeButtonState && !ScanMode)
    {
      BrakeButtonState = false;
      if (millis() - btnBrakeTimer > 2000 && IMU_Enable)
        { 
          StabilizationMode = EEPROM.read(60);
          StabilizationMode = !StabilizationMode;
          EEPROM.write(60, StabilizationMode);
          StabilizationMode = EEPROM.read(60);
          if (StabilizationMode && HoldMode) blink_mode = 2;
          else blink_mode = HoldMode;
          #ifdef DEBUG
            Serial.print(F("Stabilization Mode "));
            Serial.println(EEPROM.read(60));
          #endif
        }
      else
        {
          HoldMode = !HoldMode;
          stepper.autoPower(!HoldMode);
          HoldMode ? stepper.enable() : stepper.disable();
          ScanMode = false;
          stepper.brake();
            if (StabilizationMode && HoldMode) blink_mode = 2;
            else blink_mode = HoldMode;
          #ifdef DEBUG
            Serial.println(HoldMode ? "Hold enable" : "Hold disable");
          #endif
        BasePoint = CurrentAngle;
        }
        
    }
  
  // Правая кнопка нажата
  if ((!digitalRead(RightButton) || TransmitButton_A) && !RightButtonState) {
    RightButtonState = true;
    StabilizationMode = false;
    if (ScanMode && Speed_Change)
    {
      EEPROM.put(62, SpeedPotValue);
      Speed_Change = false;
      DEBUG_PRINTLN("New speed saved");
    }
    ScanMode = false;
    stepper.setAcceleration(Acceleration * GearRatio);
    delay(50);
    stepper.setTarget(-1*round(360 * AngleToStep * GearRatio), RELATIVE);
    } 

  // Правая кнопка отпущена
  
  if ((digitalRead(RightButton) && !TransmitButton_A && !TransmitButton_B && !TransmitButton_AB ) && RightButtonState) {
    RightButtonState = false;
        if (!ScanMode)
        {
          stepper.brake();
          IMU_Enable ? StabilizationMode = EEPROM.read(60) : StabilizationMode = false;
          if (StabilizationMode && HoldMode) blink_mode = 2;
          else blink_mode = HoldMode;
          BasePoint = CurrentAngle;
        }
    }
  
  // Левая кнопка нажата
    if ((!digitalRead(LeftButton) || TransmitButton_B) && !LeftButtonState) {
    LeftButtonState = true;
    StabilizationMode = false;
    if (ScanMode && Speed_Change)
    {
      EEPROM.put(62, SpeedPotValue);
      Speed_Change = false;
      DEBUG_PRINTLN("New speed saved");
    }
    ScanMode = false;
     //stepper.setMaxSpeed(SpeedMax * GearRatio);
    stepper.setAcceleration(Acceleration * GearRatio);
    //stepper.reset();
    delay(50);
    stepper.setTarget(round(360 * AngleToStep * GearRatio), RELATIVE);
    }

  // Левая кнопка отпущена
  if ((digitalRead(LeftButton) && !TransmitButton_A && !TransmitButton_B && !TransmitButton_AB) && LeftButtonState) {
    LeftButtonState = false;
    //stepper.setAcceleration(0);
        if (!ScanMode) {
          //stepper.reset();
          stepper.brake();
          IMU_Enable ? StabilizationMode = EEPROM.read(60) : StabilizationMode = false;
          if (StabilizationMode && HoldMode) blink_mode = 2;
          else blink_mode = HoldMode;
          BasePoint = CurrentAngle;
        }
  } 

  // Одновременно нажаты левая и правая кнопки (Кнопка сканирования)

   if (((!digitalRead(RightButton) && !digitalRead(LeftButton)) || !digitalRead(ScanButton) || TransmitButton_AB) && !ScanButtonState){
   ScanButtonState = true;
   btnScanTimer = millis();
   StabilizationMode = false;
   stepper.brake();
   stepper.setAcceleration(Acceleration * GearRatio);

if (ScanMode) 
{
  stepper.reset(); // Если мы уже в режиме сканирования, то сбрасываем начальную позицию
  DEBUG_PRINTLN("Zero point");
}
else 
{ 
  ScanMode = true;
  SpeedPotValuePrevius = SpeedPotValue; // Запоминаем текущее состояние потенциометра
  DEBUG_PRINTLN("Scan mode on");
}

   blink_mode = EEPROM.read(61);
   dir = true; 
   delay(500);
  }

  if (( (!digitalRead(RightButton) && !digitalRead(LeftButton)) || !digitalRead(ScanButton) || TransmitButton_AB) && (millis() - btnScanTimer > 2000)) blink_mode = 6;

  // Отпускаем кнопку сканирования)
   if ((digitalRead(RightButton) && digitalRead(LeftButton)) && digitalRead(ScanButton) && !TransmitButton_AB && ScanButtonState){
    ScanButtonState = false;
    if (millis() - btnScanTimer > 2000)
        {
          EEPROM.read(61) == 3 ? EEPROM.write(61, 4) : EEPROM.write(61, 3);
          EEPROM.read(61) == 3 ? ScanAngle = ScanAngle_1 : ScanAngle = ScanAngle_2;
          blink_mode = EEPROM.read(61);
          DEBUG_PRINTLN(EEPROM.read(61) == 3 ? "ScanAngle_1" : "ScanAngle_2");
        }
     }
 

// Режим сканирования

if (ScanMode)
  {
       if (!stepper.tick())
       { dir = !dir;
         stepper.setTarget(dir ? round(ScanAngle * AngleToStep * GearRatio) : -1*round(ScanAngle * AngleToStep * GearRatio), ABSOLUTE);
       } 
   } 

  // Режим стабилизации
  if (StabilizationMode && HoldMode && IMU_Enable) Stabilization();

   //Stabilization();

   LedBlink(LedPin, 150, blink_mode); // Мигаем светодиодом
   //DEBUG_PRINTLN(Speed_Pot_Value);
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
60       StabilizationMode (bool)
61       Угол сканирования (2 или 3 он же режим мигания светодиода в ScanMode)
62 - 64  Положение потенциометра (int)
*/
