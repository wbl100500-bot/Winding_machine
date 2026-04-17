#pragma once
// RAMPS 1.4 + RepRapDiscount Smart Controller 2004 + Arduino Mega 2560

// --- Энкодер меню (встроен в Smart Controller) ---
#define ENCODER_CLK         31           // BTN_EN1 (EXP2 pin 3)
#define ENCODER_DT          33           // BTN_EN2 (EXP2 pin 5)
#define ENCODER_SW          35           // BTN_ENC (EXP1 pin 9)

// --- Шаговые двигатели на RAMPS 1.4 ---
#define STEPPER_STEP_Z      54           // X STEP  (A0)
#define STEPPER_DIR_Z       55           // X DIR   (A1)
#define STEPPER_EN_Z        38           // X EN

#define STEPPER_STEP_A      46           // Z STEP
#define STEPPER_DIR_A       48           // Z DIR
#define STEPPER_EN_A        62           // Z EN  (A8)

#define STEPPER_EN          STEPPER_EN_Z // совместимость

// --- Педаль/кнопка СТОП ---
// Подключить между пином 2 и GND (INPUT_PULLUP — в покое HIGH, нажато LOW)
// Пока педаль не подключена — пин 2 подтянут к VCC, намотка запускается нормально
#define BUTTON_STOP         2

// --- Бузер ---
#define BUZZER              37           // BEEPER на EXP1

// --- Дисплей 2004 (параллельное подключение через EXP1) ---
#define DISPLAY_RS          16
#define DISPLAY_EN          17
#define DISPLAY_D4          23
#define DISPLAY_D5          25
#define DISPLAY_D6          27
#define DISPLAY_D7          29

#define KEYBOARD_PIN        0

#define THREAD_PITCH        1000

#define DISPLAY_NCOL        20
#define DISPLAY_NROW        4
#define DISPLAY_I2C         0
#define DISPLAY_ADDRESS     0x27

#define STEPPER_Z_STEPS       200
#define STEPPER_Z_MICROSTEPS  4
#define STEPPER_Z_REVERSE     1
#define STEPPER_A_STEPS       200
#define STEPPER_A_MICROSTEPS  8
#define STEPPER_A_REVERSE     0

#define ENCODER_TYPE        EB_HALFSTEP
#define ENCODER_INPUT       INPUT

#define KEYBOARD_LEFT       0
#define KEYBOARD_UP         33
#define KEYBOARD_DOWN       93
#define KEYBOARD_RIGHT      171
#define KEYBOARD_SELECT     350

// Для LCD с 8 пользовательскими символами кириллица в большом меню 20x4
// может «разваливаться» (одновременно на экране больше 8 уникальных глифов).
// По умолчанию используем английский интерфейс, чтобы меню отображалось стабильно.
#define LANGUAGE            EN
//#define DEBUG
#define TRANSFORMER_COUNT   3


// --- Энкодер разматываемой катушки (Mega пины 18/19/20) ---
#define UNWIND_ENC_A        18
#define UNWIND_ENC_B        19
#define UNWIND_ENC_Z        20
#define UNWIND_PPR          1024
