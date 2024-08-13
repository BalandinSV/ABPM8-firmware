/* Программа для управления шаговым двигателем NEMA 23 с помощью драйвера TB6600 или совместимого
добавлена поддержка радиопультов. Приемник SYN480R

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
Кнопка Right - A0
Кнопка Left -  A1
Кнопка Scan - A2
Кнопка Breake - A3
Выход приемника DATA подключается на - D2
Питание приемника VCC с вывода 3v3 Arduino
Схема подключения и описание режимов https://github.com/BalandinSV/ABPM8-firmware
*/
#include <GyverStepper2.h> // https://github.com/GyverLibs/GyverStepper
#include <RCSwitch.h> // Библиотека для работы с пультом https://github.com/sui77/rc-switch/tree/master
#include <EEPROM.h> //Библиотека для работы с ПЗУ
//GStepper< STEPPER2WIRE> stepper(steps, step, dir, en); // драйвер step-dir + пин enable
//3200 импульсов на оборот. Для TB6600 (S1-OFF, S2-OFF, S3-ON)
// PUL - D8 pin
// DIR - D7 pin
// ENABLE - D6 pin
GStepper2<STEPPER2WIRE> stepper(3200, 8, 7, 6); 
RCSwitch mySwitch = RCSwitch();

//////////////////// СЕКЦИЯ НАСТРОЕК ////////////////////////

#define RightButton A0
#define LeftButton A1
#define ScanButton A2
#define BreakeButton A3
#define AnglePot A6
#define SpeedPot A7
#define LedPin 13
#define ScanAngleMin 10.00 // Минимальный угол сканирования
#define ScanAngleMax 360.00 // Максимальный угол сканирования
#define SpeedMin 20 // Минимальная угловая скорость шаг/сек
#define SpeedMax 1000 // Максимальная угловая скорость шаг/сек
#define Acceleration 500 // Ускорение. 0 - выключено

/////////////////////////////////////////////////////////////

bool TransmitButton_A = false;
bool TransmitButton_B = false;
bool TransmitButton_AB = false;

bool RightButtonState = false;
bool LeftButtonState = false;
bool ScanButtonState = false;
bool BreakeButtonState = false;
bool ScanMode = false;
bool BreakeState = false;
bool dir = true;
int32_t ScanAngle = 120;
long TransmitButton;
bool ScanModeFirst = true;
uint32_t btnTimer = 0;
uint32_t TransmitButtonCode_A[3]; //Массив для хранения кодов кнопок А разных брелоков 3 х 4 байта 
uint32_t TransmitButtonCode_B[3]; //Массив для хранения кодов кнопок B разных брелоков 3 х 4 байта
uint32_t TransmitButtonCode_AB[3]; //Массив для хранения кодов кнопок А+B разных брелоков 3 х 4 байта

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
     if (!digitalRead(BreakeButton) && BreakeButtonState && millis() - btnTimer > 5000) {
       BreakeButtonState = false;
       btnTimer = millis();
       digitalWrite(LedPin, HIGH);
         // Записываем в EEPROM нули
             for (uint8_t i = 0; i<36; i++){
               EEPROM.put(i, 0);
             }
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
  //Serial.begin(115200);
  pinMode(RightButton, INPUT_PULLUP);
  pinMode(LeftButton, INPUT_PULLUP);
  pinMode(ScanButton, INPUT_PULLUP);
  pinMode(BreakeButton, INPUT_PULLUP);
  pinMode(LedPin, OUTPUT);

  stepper.autoPower(true);
  stepper.setAcceleration(Acceleration); // установка ускорения в шагах/сек/сек
  stepper.setMaxSpeed(SpeedMax); // установка скорости в шагах/сек/сек
  stepper.disable();
  mySwitch.enableReceive(0);  // Инициализация приемника на pin 2 (Interrupt 0)
  EEPROM.get(0, TransmitButtonCode_A); // Читаем из ПЗУ массив кодов кнопок А
  EEPROM.get(12, TransmitButtonCode_B); // Читаем из ПЗУ массив кодов кнопок В
  EEPROM.get(24, TransmitButtonCode_AB); // Читаем из ПЗУ массив кодов кнопок АВ

  if (!digitalRead(BreakeButton)){
  BreakeButtonState = true;
  Settings();
  EEPROM.get(0, TransmitButtonCode_A); // Читаем из ПЗУ массив кодов кнопок А
  EEPROM.get(12, TransmitButtonCode_B); // Читаем из ПЗУ массив кодов кнопок В
  EEPROM.get(24, TransmitButtonCode_AB); // Читаем из ПЗУ массив кодов кнопок АВ}
  }
}

void loop() {
  stepper.tick(); // Тикаем мотором
  GetTransmitButton(); // Проверка сигнала от пульта

  // Опрашиваем потенциометр SPEED

  static uint32_t tmr1;
  if (millis() - tmr1 > 50) {
    tmr1 = millis();
    stepper.setMaxSpeed(map(analogRead(SpeedPot), 0, 1023, SpeedMin, SpeedMax));
  }

    // Опрашиваем потенциометр ANGLE

  static uint32_t tmr2;
  if (millis() - tmr2 > 50) {
    tmr2 = millis();
    ScanAngle = (map(analogRead(AnglePot), 0, 1023, ScanAngleMin, ScanAngleMax));
  }

    // Опрашиваем кнопку BREAKE

  if (!digitalRead(BreakeButton) && !BreakeButtonState && millis() - btnTimer > 100) {
    BreakeButtonState = true;
    btnTimer = millis();
    BreakeState = !BreakeState;
    stepper.autoPower(!BreakeState);
    delay(100);
    if (BreakeState) stepper.enable();
       else stepper.disable();
    digitalWrite(LedPin, BreakeState);
  }
  if (digitalRead(BreakeButton) && BreakeButtonState && millis() - btnTimer > 100) {
    BreakeButtonState = false;
    btnTimer = millis();
  }
  
  // Правая кнопка нажата
  if ((!digitalRead(RightButton) || TransmitButton_A) && !RightButtonState && !LeftButtonState) {
    RightButtonState = true;
    ScanMode = false;
    ScanModeFirst = true;
    stepper.enable();
    stepper.setTargetDeg(-360.00, RELATIVE);
    } 

  // Правая кнопка отпущена
  
  if ((digitalRead(RightButton) && !TransmitButton_A && !TransmitButton_B && !TransmitButton_AB ) && RightButtonState) {
    RightButtonState = false;
       if (!ScanMode) stepper.brake();
  }
  
  // Левая кнопка нажата
    if ((!digitalRead(LeftButton) || TransmitButton_B) && !LeftButtonState && !RightButtonState) {
    LeftButtonState = true;
    ScanMode = false;
    ScanModeFirst = true;
    stepper.enable();
    stepper.setTargetDeg(360.00, RELATIVE);
  }

  // Левая кнопка отпущена
  if ((digitalRead(LeftButton) && !TransmitButton_A && !TransmitButton_B && !TransmitButton_AB) && LeftButtonState) {
    LeftButtonState = false;
       if (!ScanMode) stepper.brake();
  } 

  // Одновременно нажаты левая и правая кнопки

   if ((!digitalRead(RightButton) && !digitalRead(LeftButton)) || !digitalRead(ScanButton) || TransmitButton_AB) {
   ScanMode = true;
   ScanModeFirst = true;
   dir = true; 
   delay(100);
  }
 

// Режим сканирования

if (ScanMode){
           
       if (ScanModeFirst) {
          ScanModeFirst = false;
          stepper.enable();
          stepper.setTargetDeg(ScanAngle/2, RELATIVE);
          }
       else if (!stepper.tick())
       { dir = !dir;
         stepper.setTargetDeg(dir ? ScanAngle : -1*ScanAngle, RELATIVE);
       } 
       
   } 

}
