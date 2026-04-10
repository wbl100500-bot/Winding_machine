/* Name: Winding machine
   Description: Arduino ATmega 328P + Stepper motor control CNC Shield v3 + 2004 LCD + Encoder KY-040

       Arduino pinout diagram:
          _______________
         |      USB      |
         |           AREF|
         |            GND|
         |             13| DIR A
         |RESET        12| STEP A
         |3V3         #11| STOP BT
         |5V          #10| BUZ OUT
         |GND          #9|
         |VIN           8| EN STEP
         |               |
         |              7| DIR Z
         |             #6|
  LCD RS |A0 14        #5| ENCODER CLK
  LCD EN |A1 15         4| STEP Z
  LCD D4 |A2 16   INT1 #3| ENCODER SW
  LCD D5 |A3 17   INT0  2| ENCODER DT
  LCD D6 |A4 18      TX 1|
  LCD D7 |A5 19      RX 0|
         |__A6_A7________|


  https://cxem.net/arduino/arduino235.php
  https://cxem.net/arduino/arduino245.php

*/

#include "config.h"  // все настройки железа здесь
#include "debug.h"

#define FPSTR(pstr) (const __FlashStringHelper *)(pstr)
#define LENGTH(a) (sizeof(a) / sizeof(*a))

#if DISPLAY_I2C == 1
#include <LiquidCrystal_I2C.h>
#else
#include <LiquidCrystal.h>
#endif
#include <EncButton.h>

namespace ManualEnc {
  static int8_t _dir    = 0;
  static bool   _turned = false;
  void setup() {
    pinMode(ENCODER_CLK, INPUT_PULLUP);
    pinMode(ENCODER_DT,  INPUT_PULLUP);
  }
  void tick() {
    static uint8_t last = HIGH;
    uint8_t clk = digitalRead(ENCODER_CLK);
    if (clk != last) {
      last = clk;
      if (clk == LOW) {
        _dir    = (digitalRead(ENCODER_DT) == HIGH) ? 1 : -1;
        _turned = true;
      }
    }
  }
  bool turn() { bool t = _turned; _turned = false; return t; }
  int8_t dir() { return _dir; }
}

template<uint8_t PIN>
class ManualBtn {
  uint8_t _cur   = HIGH;
  uint8_t _prev  = HIGH;
  bool    _click = false;
  bool    _press = false;
  bool    _rel   = false;
  uint32_t _downAt = 0;
public:
  void setup() { pinMode(PIN, INPUT_PULLUP); }
  void tick() {
    _press = false; _rel = false; _click = false;
    uint8_t raw = digitalRead(PIN);
    static uint32_t lastChange = 0;
    if (raw != _prev) { lastChange = millis(); _prev = raw; }
    if (millis() - lastChange > 20) {
      if (raw != _cur) {
        _cur = raw;
        if (_cur == LOW)  { _press = true; _downAt = millis(); }
        else              { _rel = true;
                            if (millis() - _downAt < 600) _click = true; }
      }
    }
  }
  bool click()    { return _click; }
  bool pressing() { return _cur == LOW; }
  bool press()    { return _press; }
  bool release()  { return _rel; }
};

ManualBtn<ENCODER_SW>  encBtn;
ManualBtn<BUTTON_STOP> pedal;
#define encoder encBtn
#include <AnalogKey.h>
#include <GyverPlanner.h>
#include <GyverStepper2.h>
#include "LiquidCrystalCyr.h"
#include "Menu.h"
#include "timer.h"
#include "buzzer.h"

#include "Screen.h"
#include "Winding.h"
#include "strings.h"

#ifndef STEPPER_A_STEPS
#define STEPPER_A_STEPS STEPPER_STEPS
#endif
#ifndef STEPPER_Z_STEPS
#define STEPPER_Z_STEPS STEPPER_STEPS
#endif
#ifndef STEPPER_A_MICROSTEPS
#define STEPPER_A_MICROSTEPS STEPPER_MICROSTEPS
#endif
#ifndef STEPPER_Z_MICROSTEPS
#define STEPPER_Z_MICROSTEPS STEPPER_MICROSTEPS
#endif
#ifndef STEPPER_Z_REVERSE
#define STEPPER_Z_REVERSE 0
#endif
#ifndef STEPPER_A_REVERSE
#define STEPPER_A_REVERSE 0
#endif
#ifndef TRANSFORMER_COUNT
#define TRANSFORMER_COUNT 3
#endif

#define STEPPER_Z_STEPS_COUNT (float(STEPPER_Z_STEPS) * STEPPER_Z_MICROSTEPS)
#define STEPPER_A_STEPS_COUNT (float(STEPPER_A_STEPS) * STEPPER_A_MICROSTEPS)
#define STEPPER_SPEED_LIMIT 54000
#ifdef GS_FAST_PROFILE
#define PLANNER_SPEED_LIMIT 111000
#else
#define PLANNER_SPEED_LIMIT 111000
#endif

#ifndef WINDING_SPEED_CAL_NUM
#define WINDING_SPEED_CAL_NUM 2L
#endif
#ifndef WINDING_SPEED_CAL_DEN
#define WINDING_SPEED_CAL_DEN 1L
#endif
#ifndef UNWIND_SPEED_CAL_NUM
#define UNWIND_SPEED_CAL_NUM 1L
#endif
#ifndef UNWIND_SPEED_CAL_DEN
#define UNWIND_SPEED_CAL_DEN 1L
#endif

#define SPEED_LIMIT (int32_t(PLANNER_SPEED_LIMIT) * 60L * WINDING_SPEED_CAL_DEN / (STEPPER_Z_STEPS_COUNT * WINDING_SPEED_CAL_NUM))
#define UNWIND_MAX_RPM 600
#define SPEED_INC 10
#define STEPPER_Z_MANUAL_SPEED 360
#define STEPPER_A_MANUAL_SPEED ((int)(360L * 1000 / THREAD_PITCH))

#define EEPROM_SETTINGS_VERSION 3
#define EEPROM_WINDINGS_VERSION 2
#define EEPROM_SETTINGS_ADDR 0x00
#define EEPROM_WINDINGS_ADDR 0x10
#define EEPROM_WINDINGS_CLASTER (sizeof(Winding) * WINDING_COUNT + 1)
#define EEPROM_UNWIND_ADDR    0x80
#define EEPROM_UNWIND_VERSION 1

#define WINDING_COUNT 3


Winding params[WINDING_COUNT];
UnwindParams unwindParams;
int16_t unwindStoredTurns = 0;

enum UnwindPauseAction : uint8_t {
  UnwindPauseStop = 0,
  UnwindPauseContinue = 1,
  UnwindPauseMod = 2
};

int8_t currentWinding = 0;

Settings settings;

enum menu_states {
  Autowinding1,CurrentTrans,PosControl,miSettings,miUnwinding,
  Winding1,Winding2,Winding3,StartAll,WindingBack,
  TurnsSet,LaySet,StepSet,SpeedSet,Direction,Start,Cancel,
  ShaftPos,ShaftStepMul,LayerPos,LayerStepMul,PosCancel,
  miSettingsStopPerLevel,AccelSet,miSettingsBack,
  UnwindSpeedSet,UnwindStepSet,UnwindTurnsSet,UnwindOldTurnsSet,UnwindDir,UnwindStart,UnwindBack
};  // Нумерованный список строк экрана

const char *boolSet[] = { STRING_OFF, STRING_ON };
const char *dirSet[] = { "<<<", ">>>" };
const uint8_t stepSet[] = { 1, 10, 100 };

MenuItem *menuItems[] = {
  new MenuItem(0, 0, MENU_01),
  new ByteMenuItem(0, 1, MENU_02, MENU_FORMAT_02, &settings.currentTransformer, 1, TRANSFORMER_COUNT),
  new MenuItem(0, 2, MENU_04),
  new MenuItem(0, 3, MENU_05),
  new MenuItem(0, 4, MENU_UNWIND),
  new ValMenuItem(1, 0, MENU_06, MENU_FORMAT_06),
  new ValMenuItem(1, 1, MENU_07, MENU_FORMAT_06),
  new ValMenuItem(1, 2, MENU_08, MENU_FORMAT_06),
  new MenuItem(1, 3, MENU_15),
  new MenuItem(1, 4, MENU_09),
  new IntMenuItem(2, 0, MENU_10, MENU_FORMAT_10, NULL, 1, 999),
  new IntMenuItem(2, 1, MENU_13, MENU_FORMAT_13, NULL, 1, 99),
  new FloatMenuItem(2, 2, MENU_11, MENU_FORMAT_11, NULL, 5, 9995, 5),
  new IntMenuItem(2, 3, MENU_12, MENU_FORMAT_10, NULL, SPEED_INC, SPEED_LIMIT, SPEED_INC),
  new BoolMenuItem(2, 4, MENU_14, NULL, dirSet),
  new MenuItem(2, 5, MENU_15),
  new MenuItem(2, 6, MENU_09),
  new IntMenuItem(10, 0, MENU_17, MENU_FORMAT_17, &settings.shaftPos, -999, 999),
  new SetMenuItem(10, 1, MENU_18, MENU_FORMAT_10, &settings.shaftStep, stepSet, 3),
  new IntMenuItem(10, 2, MENU_19, MENU_FORMAT_17, &settings.layerPos, -999, 999),
  new SetMenuItem(10, 3, MENU_18, MENU_FORMAT_10, &settings.layerStep, stepSet, 3),
  new MenuItem(10, 4, MENU_09),
  new BoolMenuItem(11, 0, MENU_22, &settings.stopPerLayer, boolSet),
  new IntMenuItem(11, 1, MENU_23, MENU_FORMAT_10, &settings.acceleration, 0, 600, 1),
  new MenuItem(11, 2, MENU_09),
  new IntMenuItem(3, 0, MENU_12, MENU_FORMAT_10, NULL, 30, UNWIND_MAX_RPM, 10),
  new FloatMenuItem(3, 1, MENU_11, MENU_FORMAT_11, NULL, 5, 9995, 5),
  new IntMenuItem(3, 2, MENU_UWTURNS, MENU_FORMAT_10, NULL, 1, 999),
  new IntMenuItem(3, 3, MENU_UWOLD, MENU_FORMAT_10, NULL, 0, 32767, 1),
  new BoolMenuItem(3, 4, MENU_14, NULL, dirSet),
  new MenuItem(3, 5, MENU_15),
  new MenuItem(3, 6, MENU_09),
};

byte up[8] = { 0b00100, 0b01110, 0b11111, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000 };    // Создаем свой символ ⯅ для LCD
byte down[8] = { 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b11111, 0b01110, 0b00100 };  // Создаем свой символ ⯆ для LCD

#if DISPLAY_I2C == 0
LiquidCrystalCyr lcd(DISPLAY_RS, DISPLAY_EN, DISPLAY_D4, DISPLAY_D5, DISPLAY_D6, DISPLAY_D7);  // Назначаем пины для управления LCD
#else
LiquidCrystalCyr lcd(DISPLAY_ADDRESS, DISPLAY_NCOL, DISPLAY_NROW);
#endif

MainMenu menu(menuItems, LENGTH(menuItems), lcd);
MainScreen screen(lcd);


GStepper2<STEPPER2WIRE> shaftStepper(STEPPER_Z_STEPS_COUNT, STEPPER_STEP_Z, STEPPER_DIR_Z, STEPPER_EN_Z);
GStepper2<STEPPER2WIRE> layerStepper(STEPPER_A_STEPS_COUNT, STEPPER_STEP_A, STEPPER_DIR_A, STEPPER_EN_A);
GPlanner<STEPPER2WIRE, 2> planner;

// encoder/pedal объявлены выше через ManualBtn

Buzzer buzzer(BUZZER);

enum { ButtonLEFT,
       ButtonUP,
       ButtonDOWN,
       ButtonRIGHT,
       ButtonSELECT
     };
int16_t key_signals[] = { KEYBOARD_LEFT, KEYBOARD_UP, KEYBOARD_DOWN, KEYBOARD_RIGHT, KEYBOARD_SELECT };
AnalogKey<KEYBOARD_PIN, LENGTH(key_signals), key_signals> keyboard;


void setup() {
  Serial.begin(9600);
  LoadSettings();
  ManualEnc::setup();
  encBtn.setup();
  pedal.setup();
  setupUnwindEncoder();

  layerStepper.disable();
  shaftStepper.disable();
  layerStepper.reverse(!STEPPER_A_REVERSE);
  shaftStepper.reverse(!STEPPER_Z_REVERSE);
  planner.addStepper(0, shaftStepper);
  planner.addStepper(1, layerStepper);

  lcd.createChar(0, up);                  // Записываем символ ⯅ в память LCD
  lcd.createChar(1, down);                // Записываем символ ⯆ в память LCD
  lcd.begin(DISPLAY_NCOL, DISPLAY_NROW);  // Инициализация LCD Дисплей
  menu.Draw();

}

void loop() {
  encoder.tick();
  ManualEnc::tick();
  KeyboardRead();

  if (ManualEnc::turn()) {
    menu.IncIndex(ManualEnc::dir());  // Если позиция энкодера изменена, то меняем menu.index и выводим экран
    menu.Draw();
  }

  if (encoder.click()) {
    switch (menu.index)  // Если было нажатие, то выполняем действие, соответствующее текущей позиции курсора
    {
      case Autowinding1:
        SaveSettings();
        LoadWindings();
        menu.index = Winding1;

        UpdateMenuItemText(0);
        UpdateMenuItemText(1);
        UpdateMenuItemText(2);
        break;

      case Winding1:
      case Winding2:
      case Winding3:
        currentWinding = menu.index - Winding1;
        menu.index = TurnsSet;
        ((IntMenuItem *)menu[TurnsSet])->value = &params[currentWinding].turns;
        ((IntMenuItem *)menu[StepSet])->value = &params[currentWinding].step;
        ((IntMenuItem *)menu[SpeedSet])->value = &params[currentWinding].speed;
        ((IntMenuItem *)menu[LaySet])->value = &params[currentWinding].layers;
        ((BoolMenuItem *)menu[Direction])->value = &params[currentWinding].dir;
        break;
      case WindingBack:
        menu.index = Autowinding1;
        break;
      case PosControl:
        menu.index = ShaftPos;
        break;
      case TurnsSet:
      case StepSet:
      case SpeedSet:
      case LaySet:
      case AccelSet:
        ValueEdit();
        break;
      case CurrentTrans:
      case miSettingsStopPerLevel:
      case Direction:
        menu.IncCurrent(1, true);
        break;
      case StartAll:
        AutoWindingAll(params, WINDING_COUNT);
        menu.Draw(true);
        break;
      case Start:
        SaveWindings();
        AutoWindingAll(params + currentWinding, 1);
        menu.index = Winding1 + currentWinding;
        UpdateMenuItemText(currentWinding);
        break;
      case Cancel:
        SaveWindings();
        menu.index = Winding1 + currentWinding;
        UpdateMenuItemText(currentWinding);
        break;

      case ShaftPos:
      case LayerPos:
        MoveTo((menu.index == LayerPos) ? layerStepper : shaftStepper, *((IntMenuItem *)menu[menu.index])->value);
        break;

      case ShaftStepMul:
      case LayerStepMul:
        menu.IncCurrent(1, true);
        ((IntMenuItem *)menu[menu.index - 1])->increment = *((SetMenuItem *)menu[menu.index])->value;
        break;
      case PosCancel:
        menu.index = PosControl;
        settings.shaftPos = 0;
        settings.layerPos = 0;
        break;

      case miSettings:
        menu.index = miSettingsStopPerLevel;
        break;

      case miSettingsBack:
        SaveSettings();
        menu.index = miSettings;
        break;
      case miUnwinding:
        LoadUnwindParams();
        menu.index = UnwindSpeedSet;
        ((IntMenuItem *)menu[UnwindSpeedSet])->value   = &unwindParams.speed;
        ((FloatMenuItem *)menu[UnwindStepSet])->value  = &unwindParams.step;
        ((IntMenuItem *)menu[UnwindTurnsSet])->value   = &unwindParams.turns;
        ((IntMenuItem *)menu[UnwindOldTurnsSet])->value = &unwindStoredTurns;
        ((BoolMenuItem *)menu[UnwindDir])->value       = &unwindParams.dir;
        break;
      case UnwindSpeedSet:
      case UnwindStepSet:
      case UnwindTurnsSet:
      case UnwindOldTurnsSet:
        ValueEdit(); break;
      case UnwindDir:
        menu.IncCurrent(1, true); break;
      case UnwindStart:
        SaveUnwindParams();
        SaveUnwindTurns(unwindStoredTurns);
        UnwindWinding(unwindParams);
        menu.index = miUnwinding;
        break;
      case UnwindBack:
        menu.index = miUnwinding;
        break;
    }
    menu.Draw();
  }
}

void UpdateMenuItemText(byte i) {
  ((ValMenuItem *)menu[Winding1 + i])->value = params[i].turns * params[i].layers;
}

void ValueEdit() {
  menu.DrawQuotes(1);
  do {
    encoder.tick();
    ManualEnc::tick();

    if (ManualEnc::turn())
      menu.IncCurrent(ManualEnc::dir() * (encoder.pressing() ? 10 : 1));

  } while (!encoder.click());
  menu.DrawQuotes(0);
}

void MoveTo(GStepper2<STEPPER2WIRE> &stepper, int &pos) {
  menu.DrawQuotes(1);
  stepper.enable();

  stepper.setAcceleration(STEPPER_Z_STEPS_COUNT * settings.acceleration / 60);
  stepper.setMaxSpeed(constrain(STEPPER_Z_STEPS_COUNT / 2, 1, STEPPER_SPEED_LIMIT));

  int o = pos;
  stepper.reset();

  do {
    stepper.tick();
    encoder.tick();
    ManualEnc::tick();

    if (ManualEnc::turn()) {
      menu.IncCurrent(ManualEnc::dir());
      stepper.setTargetDeg((double)(pos - o));
    }

  } while (!encoder.click() || stepper.getStatus() != 0);

  stepper.disable();
  menu.DrawQuotes(0);
}

void KeyboardRead() {
  if (KEYBOARD_PIN == 0 || KEYBOARD_PIN == A6)
    return;

  static int8_t oldKey = -1;
  int8_t key = keyboard.pressed();

  if (oldKey != key) {
    switch (key) {
      case ButtonLEFT:
        layerStepper.enable();
        layerStepper.setSpeedDeg(STEPPER_A_MANUAL_SPEED);
        break;
      case ButtonRIGHT:
        layerStepper.enable();
        layerStepper.setSpeedDeg(-STEPPER_A_MANUAL_SPEED);
        break;
      case ButtonUP:
        shaftStepper.enable();
        shaftStepper.setSpeedDeg(STEPPER_Z_MANUAL_SPEED);
        break;
      case ButtonDOWN:
        shaftStepper.enable();
        shaftStepper.setSpeedDeg(-STEPPER_Z_MANUAL_SPEED);
        break;
      case ButtonSELECT: break;
      default:
        layerStepper.brake();
        shaftStepper.brake();
        layerStepper.disable();
        shaftStepper.disable();
    }
    oldKey = key;
  }

  if (layerStepper.getStatus())
    layerStepper.tick();
  if (shaftStepper.getStatus())
    shaftStepper.tick();
}


volatile uint16_t speedMult88 = 256;  // 8.8 fixed-point, 1.0 = 256
static inline uint32_t applySpeedMult(uint32_t period, uint16_t mult88) {
  return (period * mult88 + 128u) >> 8;
}
static inline uint16_t getSpeedMult88() {
  noInterrupts();
  uint16_t mult = speedMult88;
  interrupts();
  return mult;
}
static inline void setSpeedMult88(uint16_t mult88) {
  noInterrupts();
  speedMult88 = mult88;
  interrupts();
}
static inline void setSpeedMultRatio(uint16_t baseSpeed, uint16_t currentSpeed) {
  if (currentSpeed == 0) {
    setSpeedMult88(256);
    return;
  }
  uint32_t scaled = ((uint32_t)baseSpeed << 8) / currentSpeed;
  if (scaled == 0) scaled = 1;
  if (scaled > 65535u) scaled = 65535u;
  setSpeedMult88((uint16_t)scaled);
}
#ifndef STEPPER_EN_Z
#define STEPPER_EN_Z STEPPER_EN
#endif
#ifndef STEPPER_EN_A
#define STEPPER_EN_A STEPPER_EN
#endif
volatile int32_t unwindPos = 0;
#define UNWIND_COUNTS_PER_REV ((int32_t)UNWIND_PPR * 2L)
#ifdef UNWIND_ENC_A
static bool unwindEncoderEnabled = false;
static volatile uint8_t* unwindEncAReg = nullptr;
static volatile uint8_t* unwindEncBReg = nullptr;
static uint8_t unwindEncAMask = 0;
static uint8_t unwindEncBMask = 0;
// Только канал A на прерывании (пин 18 = INT5 на Mega).
// Канал B читается внутри ISR только для определения направления.
// ISR_B убран — при двух ISR на A и B они давали противоположные знаки и обнуляли счёт.
// На CHANGE канала A получаем 2 импульса на период (x2), поэтому 1 оборот = UNWIND_PPR * 2.
void unwindISR_A() {
  bool a = unwindEncAReg && unwindEncAMask ? (*unwindEncAReg & unwindEncAMask) : digitalRead(UNWIND_ENC_A);
  bool b = unwindEncBReg && unwindEncBMask ? (*unwindEncBReg & unwindEncBMask) : digitalRead(UNWIND_ENC_B);
  if (a == b)
    unwindPos++;
  else
    unwindPos--;
}

void setupUnwindEncoder() {
  pinMode(UNWIND_ENC_A, INPUT_PULLUP);
  pinMode(UNWIND_ENC_B, INPUT_PULLUP);
  unwindEncAMask = digitalPinToBitMask(UNWIND_ENC_A);
  unwindEncBMask = digitalPinToBitMask(UNWIND_ENC_B);
  uint8_t portA = digitalPinToPort(UNWIND_ENC_A);
  uint8_t portB = digitalPinToPort(UNWIND_ENC_B);
  unwindEncAReg = (portA == NOT_A_PIN) ? nullptr : portInputRegister(portA);
  unwindEncBReg = (portB == NOT_A_PIN) ? nullptr : portInputRegister(portB);
  #ifdef UNWIND_ENC_Z
  pinMode(UNWIND_ENC_Z, INPUT_PULLUP);
  #endif
  detachInterrupt(digitalPinToInterrupt(UNWIND_ENC_A));
  unwindEncoderEnabled = false;
}

void setUnwindEncoderEnabled(bool enabled) {
  if (enabled == unwindEncoderEnabled)
    return;

  if (enabled)
    attachInterrupt(digitalPinToInterrupt(UNWIND_ENC_A), unwindISR_A, CHANGE);
  else
    detachInterrupt(digitalPinToInterrupt(UNWIND_ENC_A));

  unwindEncoderEnabled = enabled;
}
#else
void setupUnwindEncoder() {}
void setUnwindEncoderEnabled(bool) {}
#endif

ISR(TIMER1_COMPA_vect) {
  if (planner.tickManual())
    setPeriod(applySpeedMult(planner.getPeriod(), speedMult88));
  else
    stopTimer();
}

uint32_t getSpeed() {
  uint32_t p = applySpeedMult(planner.getPeriod(), getSpeedMult88());
  return (p == 0) ? 0 : (60000000ul / (STEPPER_Z_STEPS_COUNT * p));
}

int32_t RpmToShaftSpeed(int16_t rpm, int32_t calNum, int32_t calDen) {
  return (int32_t)STEPPER_Z_STEPS_COUNT * rpm * calNum / (60L * calDen);
}


void AutoWinding(const Winding &w, bool &direction)  // Подпрограмма автоматической намотки
{
  Winding current;  // Текущий виток и слой при автонамотке

  DebugWrite("Start");
  setUnwindEncoderEnabled(false);  // В режиме намотки энкодер размотки не используется.

  current.turns = 0;
  current.layers = 0;
  current.speed = w.speed;
  setSpeedMult88(256);
  current.dir = w.dir;
  current.step = w.step;

  screen.Draw();

  pedal.tick();
  bool run = pedal.pressing();  // педаль нажата - работаем

  shaftStepper.enable();  // Разрешение управления двигателями
  layerStepper.enable();

  planner.setAcceleration(STEPPER_Z_STEPS_COUNT * settings.acceleration / 60L);
  planner.setMaxSpeed(constrain(RpmToShaftSpeed(w.speed, WINDING_SPEED_CAL_NUM, WINDING_SPEED_CAL_DEN), 1L, (int32_t)PLANNER_SPEED_LIMIT));

  const int8_t shaftDir = w.dir ? 1 : -1;
  int32_t dShaft = STEPPER_Z_STEPS_COUNT * w.turns * shaftDir;
  int32_t dLayer = STEPPER_A_STEPS_COUNT * w.turns * w.step / int32_t(THREAD_PITCH) * (direction ? 1 : -1);
  int32_t p[] = { dShaft, dLayer };

  planner.reset();
  initTimer();

  int16_t shownTurns = -1;
  uint8_t shownPlannerStatus = 255;
  while (1) {
    if (run && !planner.getStatus()) {
      DebugWrite("READY");
      if (current.layers >= w.layers)
        break;

      if (settings.stopPerLayer && (current.layers > 0)) {
        screen.Message(STRING_2);  // "PRESS CONTINUE  "
        WaitButton();
        screen.Draw();
      }

      DebugWrite("setTarget", p[0], p[1]);
      planner.setTarget(p, RELATIVE);
      ++current.layers;
      p[1] = -p[1];
      direction = !direction;

      startTimer();
      setPeriod(applySpeedMult(planner.getPeriod(), getSpeedMult88()));

      screen.UpdateLayers(current.layers);
    }

    encoder.tick();
    ManualEnc::tick();
    pedal.tick();

    bool oldState = run;
    if (pedal.press() || pedal.release())
      run = pedal.pressing();
    else if (pedal.pressing() && encoder.click())
      run = !run;

    if (run != oldState) {
      if (run) {
        if (current.layers) {  // если цель не задали ещё, то не стартуем
          noInterrupts();
          planner.resume();
          interrupts();
          if (planner.getStatus()) {
            startTimer();
            setPeriod(applySpeedMult(planner.getPeriod(), getSpeedMult88()));
          }
        }
      } else {
        noInterrupts();
        planner.stop();
        interrupts();
      }
    }

    if (ManualEnc::turn()) {
      current.speed = constrain(current.speed + ManualEnc::dir() * SPEED_INC, SPEED_INC, SPEED_LIMIT);  // то меняем значение скорости
      //planner.setMaxSpeed(STEPPER_STEPS_COUNT * current.speed / 60L);
      setSpeedMultRatio(w.speed, current.speed);
      screen.UpdateSpeed(current.speed);
    }

    static uint32_t tmr;
    if (millis() - tmr >= 1000) {
      tmr = millis();

      int total_turns = (abs(shaftStepper.pos)) / STEPPER_Z_STEPS_COUNT;
      int16_t turnInLayer = total_turns % w.turns + 1;
      uint8_t plannerStatus = planner.getStatus();

      if (turnInLayer != shownTurns) {
        screen.UpdateTurns(turnInLayer);
        shownTurns = turnInLayer;
      }
      DebugWrite("pos", shaftStepper.pos, layerStepper.pos);
      if (plannerStatus != shownPlannerStatus) {
        screen.PlannerStatus(plannerStatus);
        shownPlannerStatus = plannerStatus;
      }
    }
  }

  layerStepper.disable();
  shaftStepper.disable();
}

int32_t LoadUnwindTurns();

void LoadUnwindParams() {
  int p = EEPROM_UNWIND_ADDR;
  byte v = 0; EEPROM_load(p, v);
  if (v == EEPROM_UNWIND_VERSION) Load(unwindParams, p);
  int32_t turns = LoadUnwindTurns();
  unwindStoredTurns = constrain(turns, 0L, 32767L);
}
void SaveUnwindParams() {
  int p = EEPROM_UNWIND_ADDR;
  byte v = EEPROM_UNWIND_VERSION;
  EEPROM_save(p, v); Save(unwindParams, p);
}
void SaveUnwindTurns(int32_t turns) {
  int p = EEPROM_UNWIND_ADDR + 1 + (int)sizeof(UnwindParams);
  EEPROM_save(p, turns);
}
int32_t LoadUnwindTurns() {
  int p = EEPROM_UNWIND_ADDR + 1 + (int)sizeof(UnwindParams);
  int32_t turns = 0;
  EEPROM_load(p, turns);
  return (turns < 0) ? 0 : turns;
}
void DrawUnwindScreen(int32_t turns, int16_t rpm, int16_t stepVal, bool running) {
  char buf[21];
  snprintf(buf, 21, "T:%05ld  %3dRPM", (long)turns, (int)rpm);
  lcd.printAt(0, 0, buf);
  snprintf(buf, 21, "S:%d.%03dmm          ", stepVal/1000, abs(stepVal%1000));
  lcd.printAt(0, 1, buf);
  if (lcd.nRows > 2) lcd.printAt(0, 2, "                    ");
  byte statusRow = (lcd.nRows > 3) ? 3 : (lcd.nRows - 1);
  lcd.printAt(0, statusRow, running ? "RUN   [BTN]=DIALOG  " : "PAUSE [BTN]=DIALOG  ");
}

uint8_t UnwindAskAction() {
  // Ждём отпускания кнопки — чтобы клик открытия не засчитался как выбор
  delay(30);
  while (digitalRead(ENCODER_SW) == LOW) { delay(5); }
  delay(30);

  uint8_t sel = UnwindPauseContinue;
  byte statusRow = (lcd.nRows > 3) ? 3 : (lcd.nRows - 1);
  lcd.printAt(0, statusRow, " STOP  >CONT   MOD  ");

  while (true) {
    ManualEnc::tick();
    encoder.tick();
    if (ManualEnc::turn()) {
      sel = (sel + 3 + ManualEnc::dir()) % 3;
      if (sel == UnwindPauseStop) lcd.printAt(0, statusRow, ">STOP   CONT   MOD  ");
      if (sel == UnwindPauseContinue) lcd.printAt(0, statusRow, " STOP  >CONT   MOD  ");
      if (sel == UnwindPauseMod) lcd.printAt(0, statusRow, " STOP   CONT  >MOD  ");
    }
    if (encoder.click()) return sel;
    delay(5);
  }
}

void UnwindWinding(const UnwindParams &w) {
  setUnwindEncoderEnabled(true);
  int32_t restoredTurns = LoadUnwindTurns();
  int32_t basePos = restoredTurns * UNWIND_COUNTS_PER_REV;
  noInterrupts(); unwindPos = basePos; interrupts();
  int16_t curSpeed = constrain(w.speed, 30, UNWIND_MAX_RPM);
  setSpeedMultRatio(w.speed, curSpeed);
  lcd.clear();
  DrawUnwindScreen(restoredTurns, curSpeed, w.step, false);
  pedal.tick();
  bool run = pedal.pressing();
  shaftStepper.enable(); layerStepper.enable();
  planner.setAcceleration(STEPPER_Z_STEPS_COUNT * settings.acceleration / 60L);
  planner.setMaxSpeed(constrain(
    RpmToShaftSpeed(w.speed, UNWIND_SPEED_CAL_NUM, UNWIND_SPEED_CAL_DEN), 1L,
    RpmToShaftSpeed(UNWIND_MAX_RPM, UNWIND_SPEED_CAL_NUM, UNWIND_SPEED_CAL_DEN)));
  shaftStepper.reverse(STEPPER_Z_REVERSE ^ w.dir);
  layerStepper.reverse(STEPPER_A_REVERSE);
  bool direction = false;
  int32_t dShaft = (int32_t)STEPPER_Z_STEPS_COUNT * w.turns;
  int32_t dLayer = (int32_t)STEPPER_A_STEPS_COUNT * w.turns * w.step / (int32_t)THREAD_PITCH;
  planner.reset(); initTimer();
  int32_t lastTurns = restoredTurns;
  int32_t savedTurns = restoredTurns;
  bool done = false;
  bool needRedraw = true;
  uint32_t lastSaveMs = millis();
  uint32_t lastDrawMs = 0;
  int16_t shownSpeed = -1;
  int32_t shownTurns = -1;
  bool shownRun = !run;

  while (!done) {
    if (run && !planner.getStatus()) {
      int32_t p[2] = { dShaft, dLayer*(direction?1:-1) };
      planner.setTarget(p, RELATIVE);
      direction = !direction;
      startTimer(); setPeriod(applySpeedMult(planner.getPeriod(), getSpeedMult88()));
    }

    encoder.tick(); ManualEnc::tick(); pedal.tick();

    bool oldRun = run;
    if (pedal.press() || pedal.release()) run = pedal.pressing();
    if (run != oldRun) {
      if (run) {
        noInterrupts(); planner.resume(); interrupts();
        if (planner.getStatus()) { startTimer(); setPeriod(applySpeedMult(planner.getPeriod(), getSpeedMult88())); }
      } else { noInterrupts(); planner.stop(); interrupts(); }
      SaveUnwindTurns(lastTurns);  // фиксируем прогресс при паузе
      savedTurns = lastTurns;
      lastSaveMs = millis();
      needRedraw = true;
    }

    if (ManualEnc::turn()) {
      curSpeed = constrain(curSpeed + ManualEnc::dir()*10, 30, UNWIND_MAX_RPM);
      setSpeedMultRatio(w.speed, curSpeed);
      needRedraw = true;
    }

    if (encoder.click()) {
      noInterrupts(); planner.stop(); interrupts();
      bool wasRun = run; run = false;
      int32_t pos; noInterrupts(); pos=unwindPos; interrupts();
      lastTurns = restoredTurns + abs(pos - basePos) / UNWIND_COUNTS_PER_REV;
      DrawUnwindScreen(lastTurns, curSpeed, w.step, false);

      uint8_t act = UnwindAskAction();
      if (act == UnwindPauseStop) {
        done = true;
      } else {
        run = (act == UnwindPauseContinue) ? wasRun : false;
        if (run) {
          noInterrupts(); planner.resume(); interrupts();
          if (planner.getStatus()) { startTimer(); setPeriod(applySpeedMult(planner.getPeriod(), getSpeedMult88())); }
        }
        DrawUnwindScreen(lastTurns, curSpeed, w.step, run);
        needRedraw = true;
      }
    }

    static uint32_t tmrU;
    if (millis()-tmrU >= 250) {
      tmrU = millis();
      int32_t pos; noInterrupts(); pos=unwindPos; interrupts();
      lastTurns = restoredTurns + abs(pos - basePos) / UNWIND_COUNTS_PER_REV;
      if (lastTurns != shownTurns || curSpeed != shownSpeed || run != shownRun) needRedraw = true;
    }

    // Периодически сохраняем прогресс в EEPROM, чтобы восстановиться после сброса.
    if ((millis() - lastSaveMs >= 3000) && (lastTurns != savedTurns)) {
      SaveUnwindTurns(lastTurns);
      savedTurns = lastTurns;
      lastSaveMs = millis();
    }

    // Меньше обращений к LCD -> стабильнее картинка на шумной линии.
    if (needRedraw || (millis() - lastDrawMs >= 1000)) {
      DrawUnwindScreen(lastTurns, curSpeed, w.step, run);
      shownTurns = lastTurns;
      shownSpeed = curSpeed;
      shownRun = run;
      lastDrawMs = millis();
      needRedraw = false;
    }
  }

  stopTimer(); noInterrupts(); planner.stop(); interrupts();
  shaftStepper.reverse(STEPPER_Z_REVERSE);
  layerStepper.disable(); shaftStepper.disable();
  int32_t pos; noInterrupts(); pos=unwindPos; interrupts();
  lastTurns = restoredTurns + abs(pos - basePos) / UNWIND_COUNTS_PER_REV;
  SaveUnwindTurns(lastTurns);
  setUnwindEncoderEnabled(false);
  lcd.clear();
  lcd.printAt(0, 0, "RAZMOTKA DONE");
  char buf[21]; snprintf(buf, 21, "VITKOV:%ld", (long)lastTurns);
  lcd.printAt(0, 1, buf);
  lcd.printAt(0, (lcd.nRows > 3) ? 3 : (lcd.nRows - 1), "PRESS BTN TO EXIT");
  WaitButton();
}


void AutoWindingAll(const Winding windings[], byte n) {
  bool direction = windings[0].dir;

  for (byte i = 0; i < n; ++i) {
    const Winding &w = windings[i];
    if (!w.turns || !w.layers || !w.step || !w.speed) continue;

    screen.Init(w);

    if (n > 1) {
      screen.Draw();
      screen.Message(STRING_3, i + 1);
      buzzer.Multibeep(2, 200, 200);
      WaitButton();
    }

    AutoWinding(w, direction);
  }

  screen.Message(STRING_1);  // "AUTOWINDING DONE"
  buzzer.Multibeep(3, 200, 200);
  WaitButton();
}


void WaitButton() {
  do {
    encoder.tick();
    KeyboardRead();
  } while (!encoder.click());
}

static uint8_t NormalizeStepMultiplier(uint8_t value) {
  if (value <= 1) return 1;
  if (value <= 10) return 10;
  return 100;
}



void LoadSettings() {
  settings = Settings();
  int p = EEPROM_SETTINGS_ADDR;
  byte v = 0;
  EEPROM_load(p, v);
  if (v != EEPROM_SETTINGS_VERSION) {
    settings.currentTransformer = constrain(settings.currentTransformer, 1, TRANSFORMER_COUNT);
    return;
  }

  Load(settings, p);
  settings.currentTransformer = constrain(settings.currentTransformer, 1, TRANSFORMER_COUNT);
  settings.shaftStep = NormalizeStepMultiplier(settings.shaftStep);
  settings.layerStep = NormalizeStepMultiplier(settings.layerStep);
}

void SaveSettings() {
  settings.currentTransformer = constrain(settings.currentTransformer, 1, TRANSFORMER_COUNT);
  settings.shaftStep = NormalizeStepMultiplier(settings.shaftStep);
  settings.layerStep = NormalizeStepMultiplier(settings.layerStep);
  int p = EEPROM_SETTINGS_ADDR;
  byte v = EEPROM_SETTINGS_VERSION;
  EEPROM_save(p, v);
  Save(settings, p);
}

void LoadWindings() {
  int p = EEPROM_WINDINGS_ADDR + (settings.currentTransformer - 1) * EEPROM_WINDINGS_CLASTER;

  byte v = 0;
  EEPROM_load(p, v);

  for (int j = 0; j < WINDING_COUNT; ++j)
    if (v == EEPROM_WINDINGS_VERSION)
      Load(params[j], p);
    else
      params[j] = Winding();
}

void SaveWindings() {
  int p = EEPROM_WINDINGS_ADDR + (settings.currentTransformer - 1) * EEPROM_WINDINGS_CLASTER;

  byte v = EEPROM_WINDINGS_VERSION;
  EEPROM_save(p, v);
  for (int j = 0; j < WINDING_COUNT; ++j)
    Save(params[j], p);
}
