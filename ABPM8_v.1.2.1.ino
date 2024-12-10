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

Потенциометр SPEED подключается к A6
Потенциометр ANGLE подключается к A7
Кнопка/педаль Right  - A0
Кнопка/педаль Left -  A1
Кнопка Scan - A2
Кнопка Break - A3

Arduino UNO     SYN480R
  GND             GND
  3V3             VCC
  D2              DAT

Arduino UNO     TMC2160
  D6             CLK
  D7             DIR
  D8             EN

Схема подключения и описание режимов https://github.com/BalandinSV/ABPM8-firmware
*/
#include "GyverStepper.h" // https://github.com/GyverLibs/GyverStepper
#include "RCSwitch.h" // Библиотека для работы с пультом https://github.com/sui77/rc-switch/tree/master
#include <EEPROM.h> //Библиотека для работы с ПЗУ


//////////////////// СЕКЦИЯ НАСТРОЕК ////////////////////////

#define RightButton A0
#define LeftButton A1
#define ScanButton A2
#define BreakeButton A3
#define AnglePot A6
#define SpeedPot A7
#define LedPin 13

#define StepPin 6
#define DirPin 7
#define EnablePin 8
#define MicroStep 6400 //6400 импульсов на оборот. MRES = 32 Для TMC2160 (M0-OFF, M1-ON)

#define ScanAngleMin 10.00 // Минимальный угол сканирования
#define ScanAngleMax 180.00 // Максимальный угол сканирования половина сектора сканирования
#define SpeedMin 20 // Минимальная угловая скорость шаг/сек
#define SpeedMax 800 // Максимальная угловая скорость шаг/сек
#define Acceleration 0 // Ускорение. 0 - выключено
//#define AngleToStep 4.44 // 1600/360=4.44 для 1600 шаг/об
//#define AngleToStep 8.89 // 3200/360=8.89 для 3200 шаг/об
#define AngleToStep 17.78 // 6400/360=17.78 для 6400 шаг/об
//#define AngleToStep 35.55 // 12800/360=35.55 для 12800 шаг/об
#define GearRatio 1 // Передаточное число редуктора

/////////////////////// РЕЖИМ ОТЛАДКИ /////////////////////
//#define DEBUG // Раскомментировать для активации сообщений при отладке

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
bool BreakeButtonState = false;
bool ScanMode = false;
bool StabilisationMode = false;
bool StabilisationModeGlobal = false;
bool BreakeState = false;
bool dir = true;

long TransmitButton;

uint32_t btnTimer = 0;
int32_t ScanAngle = 120;
uint32_t TransmitButtonCode_A[3]; //Массив для хранения кодов кнопок А разных брелоков 3 х 4 байта 
uint32_t TransmitButtonCode_B[3]; //Массив для хранения кодов кнопок B разных брелоков 3 х 4 байта
uint32_t TransmitButtonCode_AB[3]; //Массив для хранения кодов кнопок А+B разных брелоков 3 х 4 байта

float BaseAngle = 0;
float CurrentAngle = 0;
float Correct = 0;

//GStepper< STEPPER2WIRE> stepper(steps, step, dir, en); // драйвер step-dir + пин enable

GStepper<STEPPER2WIRE> stepper(MicroStep, StepPin, DirPin, EnablePin); // Инициализируем драйвер
RCSwitch mySwitch = RCSwitch(); // Инициализируем приемник

bool CheckCode (uint32_t CurrentCode)
{
  for (uint8_t i = 0; i < 3; i++) 
     {
      if ((CurrentCode == TransmitButtonCode_A[i]) || (CurrentCode == TransmitButtonCode_B[i]) || (CurrentCode == TransmitButtonCode_AB[i]))
         {
          return false;
         } 
     }
      return true;
}

void Settings(){
  byte modes[] = {
   0B00000000, //Blink mode 0 - Светодиод выключен
   0B11111111, //Blink mode 1 - Горит постоянно
   0B00001111, //Blink mode 2 - Мигание по 0.5 сек
   0B00000001, //Blink mode 3 - Короткая вспышка раз в секунду
   0B00000101, //Blink mode 4 - Две короткие вспышки раз в секунду
   0B00010101, //Blink mode 5 - Три короткие вспышки раз в секунду
   0B01010101  //Blink mode 6 - Частые короткие вспышки (4 раза в секунду)
};
  uint32_t TransmitButtonSeting[3] = {0,0,0}; //Массив для хранения кодов кнопок настраиваемого передатчика
  uint8_t blink_mode = 3;
  uint8_t modes_count = modes[blink_mode];
  uint8_t  blink_loop = 0;
  uint8_t  ProgButton = 0;
  uint32_t tmrLed = millis();
  uint32_t TimerSetting = millis();
  uint32_t TimerRF = millis();
  bool SettingComplite = false;
   while ((millis() - TimerSetting < 30000) && !SettingComplite) // Задаем время на программирование
    {
      // Мигаем светодиодом
      modes_count = modes[blink_mode];
      if (millis() - tmrLed > 150){
        tmrLed = millis();
        // Режим светодиода ищем по битовой маске       
       if (modes_count & 1<<(blink_loop&0x07) ) digitalWrite(LedPin, HIGH); 
       else  digitalWrite(LedPin, LOW);
       blink_loop++;  
        }

   // Проверяем долгое нажатие кнопки Breake, если да, то стираем коды всех передатчков
     if (!digitalRead(BreakeButton) && !BreakeButtonState && millis() - btnTimer > 100) {
       BreakeButtonState = true;
       btnTimer = millis();
      }
     if (!digitalRead(BreakeButton) && BreakeButtonState && millis() - btnTimer > 5000)
      {
       BreakeButtonState = false;
       btnTimer = millis();
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
     if (digitalRead(BreakeButton) && BreakeButtonState && millis() - btnTimer > 100) {
       BreakeButtonState = false;
       btnTimer = millis();
      }
   
   // Читаем сигнал с передатчика
      if (millis() - TimerRF > 150)
      {
        TimerRF = millis();
        if (mySwitch.available())
        {
          switch (ProgButton)
          {
            case 0:
            {
              if (CheckCode(mySwitch.getReceivedValue()))
                {
                  TransmitButtonSeting[ProgButton] = (mySwitch.getReceivedValue());
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
                    ProgButton = 2;
                    blink_mode = 5;
                    break;
                  } 
                 } 
                 break;;
              case 2:
                 {
                    if ((CheckCode(mySwitch.getReceivedValue())) && (mySwitch.getReceivedValue() != TransmitButtonSeting[0]) && (mySwitch.getReceivedValue() != TransmitButtonSeting[1]))
                    {
                       TransmitButtonSeting[ProgButton] = (mySwitch.getReceivedValue());
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

void setup() {
  DEBUG_BEGIN(115200);
  pinMode(RightButton, INPUT_PULLUP);
  pinMode(LeftButton, INPUT_PULLUP);
  pinMode(ScanButton, INPUT_PULLUP);
  pinMode(BreakeButton, INPUT_PULLUP);
  pinMode(LedPin, OUTPUT);

 // Настройка шагового мотора
  stepper.autoPower(true);
  stepper.setAcceleration(Acceleration); // установка ускорения в шагах/сек/сек
  stepper.setMaxSpeed(SpeedMax * GearRatio); // установка скорости в шагах/сек/сек
  stepper.disable();
  mySwitch.enableReceive(0);  // Инициализация приемника на pin 2 (Interrupt 0)
  EEPROM.get(0, TransmitButtonCode_A); // Читаем из ПЗУ массив кодов кнопок А
  EEPROM.get(12, TransmitButtonCode_B); // Читаем из ПЗУ массив кодов кнопок В
  EEPROM.get(24, TransmitButtonCode_AB); // Читаем из ПЗУ массив кодов кнопок АВ

  // Вход в режим программирования пультов
  if (!digitalRead(BreakeButton)){
  BreakeButtonState = true;
  Settings();
  EEPROM.get(0, TransmitButtonCode_A); // Читаем из ПЗУ массив кодов кнопок А
  EEPROM.get(12, TransmitButtonCode_B); // Читаем из ПЗУ массив кодов кнопок В
  EEPROM.get(24, TransmitButtonCode_AB); // Читаем из ПЗУ массив кодов кнопок АВ}
  }
  DEBUG_PRINTLN("Run");
}

void loop() {
  stepper.tick(); // Тикаем мотором
  GetTransmitButton(); // Проверка сигнала от пульта

  // Опрашиваем потенциометр SPEED

  static uint32_t tmr1;
  if (millis() - tmr1 > 50) {
    tmr1 = millis();
    if (!StabilisationMode) stepper.setMaxSpeed(map(analogRead(SpeedPot), 0, 1023, SpeedMin, SpeedMax) * GearRatio);
  }

    // Опрашиваем потенциометр ANGLE

  static uint32_t tmr2;
  if (millis() - tmr2 > 50) {
    tmr2 = millis();
    if (!StabilisationMode) ScanAngle = (map(analogRead(AnglePot), 0, 1023, ScanAngleMin, ScanAngleMax));
  }

    // Опрашиваем кнопку BREAKE

  if (!digitalRead(BreakeButton) && !ScanMode && !BreakeButtonState && millis() - btnTimer > 100) {
    BreakeButtonState = true;
    btnTimer = millis();
    BreakeState = !BreakeState;
    stepper.autoPower(!BreakeState);
    BreakeState ? stepper.enable() : stepper.disable();
    stepper.brake();
    StabilisationMode = BreakeState;
    StabilisationModeGlobal = StabilisationMode;
    digitalWrite(LedPin, BreakeState);
  }
  if (digitalRead(BreakeButton) && BreakeButtonState && millis() - btnTimer > 100)
    {
      BreakeButtonState = false;
      btnTimer = millis();
    }
  
  // Правая кнопка нажата
  if ((!digitalRead(RightButton) || TransmitButton_A) && !RightButtonState) {
    RightButtonState = true;
    StabilisationMode = false;
    ScanMode = false;
    stepper.setMaxSpeed(SpeedMax * GearRatio);
    stepper.setAcceleration(Acceleration);
    stepper.reset();
    delay(50);
    stepper.setTarget(-1*round(360 * AngleToStep * GearRatio), RELATIVE);
    } 

  // Правая кнопка отпущена
  
  if ((digitalRead(RightButton) && !TransmitButton_A && !TransmitButton_B && !TransmitButton_AB ) && RightButtonState) {
    RightButtonState = false;
        if (!ScanMode)
        {
          stepper.reset();
          StabilisationMode = StabilisationModeGlobal;
        }
    }
  
  // Левая кнопка нажата
    if ((!digitalRead(LeftButton) || TransmitButton_B) && !LeftButtonState) {
    LeftButtonState = true;
    StabilisationMode = false;
    ScanMode = false;
    stepper.setMaxSpeed(SpeedMax * GearRatio);
    stepper.setAcceleration(Acceleration);
    stepper.reset();
    delay(50);
    stepper.setTarget(round(360 * AngleToStep * GearRatio), RELATIVE);
    }

  // Левая кнопка отпущена
  if ((digitalRead(LeftButton) && !TransmitButton_A && !TransmitButton_B && !TransmitButton_AB) && LeftButtonState) {
    LeftButtonState = false;
    //stepper.setAcceleration(0);
        if (!ScanMode) {
          stepper.reset();
          StabilisationMode = StabilisationModeGlobal;
        }
  } 

  // Одновременно нажаты левая и правая кнопки

   if ((!digitalRead(RightButton) && !digitalRead(LeftButton)) || !digitalRead(ScanButton) || TransmitButton_AB) {
   //if (((RightButtonState) && (LeftButtonState)) || !digitalRead(ScanButton) || TransmitButton_AB) {
   StabilisationMode = false;
   stepper.brake();
   stepper.setMaxSpeed(SpeedMax * GearRatio);
   stepper.setAcceleration(Acceleration);
   ScanMode = true;
   stepper.reset();
   dir = true; 
   delay(100);
  }
 

// Режим сканирования

if (ScanMode)
  {
       if (!stepper.tick())
       { dir = !dir;
         stepper.setTarget(dir ? round(ScanAngle * AngleToStep * GearRatio) : -1*round(ScanAngle * AngleToStep * GearRatio), ABSOLUTE);
       } 
   } 
}
