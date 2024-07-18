/* Программа для управления шаговым двигателем NEMA 23 с помощью драйвера TB6600
или совместимого
Потенциометр SPEED подключается к A6
Потенциометр ANGLE подключается к A7
Кнопка Right - A0
Кнопка Left -  A1
Кнопка Scan - A2
Кнопка Breake - A3
Схема подключения и описание режимов https://github.com/BalandinSV/Arduino-based-Panoptix-mount
*/
#include <GyverStepper2.h> // https://github.com/GyverLibs/GyverStepper
//GStepper< STEPPER2WIRE> stepper(steps, step, dir, en); // драйвер step-dir + пин enable
//3200 импульсов на оборот. Для TB6600 (S1-OFF, S2-OFF, S3-ON)
// PUL - D8 pin
// DIR - D7 pin
// ENABLE - D6 pin
GStepper2<STEPPER2WIRE> stepper(3200, 8, 7, 6); 

#define RightButton A0
#define LeftButton A1
#define ScanButton A2
#define BreakeButton A3
#define AnglePot A6
#define SpeedPot A7
#define LedPin 13
#define ScanAngleMin 10.00
#define ScanAngleMax 360.00
#define SpeedMin 20
#define SpeedMax 1000

bool RightButtonState = false;
bool LeftButtonState = false;
bool ScanButtonState = false;
bool ScanMode = false;
bool BreakeState = false;
int8_t dir = 1;
int32_t ScanAngle = 120;


bool ScanModeFirst = true;

void setup() {
  //Serial.begin(115200);
  pinMode(RightButton, INPUT_PULLUP);
  pinMode(LeftButton, INPUT_PULLUP);
  pinMode(ScanButton, INPUT_PULLUP);
  pinMode(BreakeButton, INPUT_PULLUP);
  pinMode(LedPin, OUTPUT);

  stepper.autoPower(true);
  stepper.setAcceleration(150); // установка ускорения в шагах/сек/сек
  stepper.setMaxSpeed(SpeedMax); // установка скорости в шагах/сек/сек
  stepper.disable();
}

void loop() {
  stepper.tick();

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

  static  uint32_t btnTimer = 0;
  static bool BreakeButtonState = false;
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

  if (!digitalRead(RightButton) && !RightButtonState && !LeftButtonState) {
    RightButtonState = true;
    ScanMode = false;
    ScanModeFirst = true;
    stepper.enable();
    stepper.setTargetDeg(-360.00, RELATIVE);
    } 

  // Правая кнопка отпущена
  
  if (digitalRead(RightButton) && RightButtonState) {
    //BreakeShaft();
    RightButtonState = false;
       if (!ScanMode) stepper.brake();
  }
  
  // Левая кнопка нажата
    if (!digitalRead(LeftButton) && !LeftButtonState && !RightButtonState) {
    LeftButtonState = true;
    ScanMode = false;
    ScanModeFirst = true;
    stepper.enable();
    stepper.setTargetDeg(360.00, RELATIVE);
  }

  // Левая кнопка отпущена
  if (digitalRead(LeftButton) && LeftButtonState) {
    LeftButtonState = false;
       if (!ScanMode) stepper.brake();
  } 

  // Одновременно нажаты левая и правая кнопки

  if ((!digitalRead(RightButton) && !digitalRead(LeftButton)) || !digitalRead(ScanButton)) {
   ScanMode = true;
   ScanModeFirst = true;
   dir = 1; 
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
       { dir = -1*dir;
         stepper.setTargetDeg(dir*ScanAngle, RELATIVE);
       } 
       
   } 

}
