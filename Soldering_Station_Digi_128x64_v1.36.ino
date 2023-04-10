//Modular Soldering Station, simple PID With Power of Mathematic
//Software written by TA1AYH (Mehmet)
//Board designed by TA2PLT (Ercan)
//Mainboard Designed 2021 in TURKIYE, made in CHINA

#include "U8glib.h"
U8GLIB_SH1106_128X64_2X u8g(U8G_I2C_OPT_NO_ACK);
#include "Rotary.h"
#include <TimerOne.h>
#include "max6675.h"
#include <EEPROM.h>

String Version = "v1.36";
double use_ir = true;
//double use_ir = false;
double use_pwm = true;
//double use_pwm = false;

void Welcome(void) {
  u8g.drawRFrame(2, 2, 126, 62, 10);  // draws frame with rounded edges
  u8g.setFont(u8g_font_helvR14r);
  u8g.drawStr(18, 22, "MODULAR");
  u8g.drawStr(27, 42, "SOLDER");
  u8g.setFont(u8g_font_6x10);
  u8g.drawStr(65, 57, "By YOURCALL"); 
}

#define BUZZER_PIN A2
#define BUTTON_PIN 0
#define IR_TX_PIN 1
#define ENCODER_PIN_B 2
#define ENCODER_PIN_A 3
#define IR_RX_PIN A0
#define thermoDO 6   // CO
#define thermoCS 7   // CS
#define thermoCLK 8  //SCK
#define POWER_PIN 11

#define MIN_ELAPSED_TIME 30
int MAX_WAIT_TIME = 7;
const int WIDTH = 128;
const int HEIGHT = 64;
int x = 19;
int y[WIDTH];
MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

unsigned long ButtonTime = 0;
unsigned long TempReadTime = 0;

Rotary encoder = Rotary(ENCODER_PIN_A, ENCODER_PIN_B);

const int eepromIdleminAdress = 0;
const int eepromIdleAdress = 2;
const int eepromSetAdress = 4;
const int DDriveAdress = 6;
const int AddPowerAdress = 8;
const int LimitPWMAdress = 10;
const int IR_Adress = 12;
const int TMaxAdress = 14;
const int DisplayTypeAdress = 16;
const int MAX_WAIT_Adress = 18;
const int ProbAdress = 20;

float Temperature = 0;
float TargetTemp = 0;
float OldTargetTemp = 0;
float OldTemp = 0;
float Trent = 0;
bool Heater_Connected = 1;
bool OverTemp = 0;
bool Protection = 0;
bool DDrive = 0;
int LimitPWM = 100;
int AddPower = 0;
float LimitFFlow = 0;
int Prob = 0;
int restmin = 0;
int restsec = 0;
unsigned char microsecond_10 = 0;
unsigned char second = 0;
unsigned char minute = 0;
unsigned char _microsecond_10 = 0;
unsigned char _second = 0;
unsigned char _minute = 0;
int SET_LEVEL = 0;
int PageNumber = 1;
int Knob_Dir = 0;
int DisplayType = 0;
int PageCount = 11;  //Page Limit of Settings Menu
unsigned char Idle_minute = 5;
unsigned char Idle_second = 00;
boolean Flag_ReadTime = 0;
volatile int Blink = 0;
int SaveEp = 0;
int Alarming = 0;
int Power = 0;
uint8_t Idle_Power = 0;
float Gap = 0;
int Sira = 0;
float Flow[6];
float adj = 0.0;
float FFlow = 0.0;
int IR_ON = 0;
float IR_Val = 0;
int IR_V = 0;
int IR_Count = 0;
int Proximity = 0;
int Cleared = 1;
int TempChanged = 0;
double TMax = 450;
const int VMO = 500;
float tune = 0;



void setup() {
  clearY();
  //---------------------------------------------- Set PWM frequency for D3 & D11 ------------------------------
  //TCCR2B = TCCR2B & (B11111000 | B00000001);    // set timer 2 divisor to     1 for PWM frequency of 31372.55 Hz
  //TCCR2B = TCCR2B & (B11111000 | B00000010);    // set timer 2 divisor to     8 for PWM frequency of  3921.16 Hz
  //TCCR2B = TCCR2B & (B11111000 | B00000011);    // set timer 2 divisor to    32 for PWM frequency of   980.39 Hz
  TCCR2B = TCCR2B & (B11111000 | B00000100);  // set timer 2 divisor to    64 for PWM frequency of   490.20 Hz (The DEFAULT)
  //TCCR2B = TCCR2B & (B11111000 | B00000101);    // set timer 2 divisor to   128 for PWM frequency of   245.10 Hz
  //TCCR2B = TCCR2B & (B11111000 | B00000110);    // set timer 2 divisor to   256 for PWM frequency of   122.55 Hz
  //TCCR2B = TCCR2B & (B11111000 | B00000111);    // set timer 2 divisor to  1024 for PWM frequency of    30.64 Hz
  Timer1.initialize(10000);  //timing for 10ms=10000 cycle Saniyenin 10 da biri

  Timer1.attachInterrupt(TimingISR);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), rotaryEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), rotaryEncoder, CHANGE);

  pinMode(POWER_PIN, OUTPUT);
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(IR_TX_PIN, OUTPUT);
  pinMode(IR_RX_PIN, INPUT);
  pinMode(thermoCS, OUTPUT);
  pinMode(thermoDO, INPUT);
  pinMode(thermoCLK, OUTPUT);

  Beep(10);

  u8g.firstPage();
  do {
    Welcome();
  } while (u8g.nextPage());
  delay(3000);

  TargetTemp = readIntFromEEPROM(eepromSetAdress);
  if (isnan(TargetTemp)) {
    TargetTemp = 0;
    SavetoEEPROM(eepromSetAdress, TargetTemp);
  }
  if (use_pwm == false) {
    DDrive = 0;
  } else {
    DDrive = readIntFromEEPROM(DDriveAdress);
    if (isnan(DDrive) || (Idle_minute > 99 && DDrive == 1)) {
      DDrive = 0;
      SavetoEEPROM(DDriveAdress, DDrive);
    }
  }

  Idle_minute = readIntFromEEPROM(eepromIdleminAdress);
  if (isnan(Idle_minute) || Idle_minute > 99) {
    Idle_minute = 5;
    SavetoEEPROM(eepromIdleminAdress, Idle_minute);
  }

  Idle_Power = readIntFromEEPROM(eepromIdleAdress);
  if (isnan(Idle_Power) || Idle_Power > 90) {
    Idle_Power = 20;
    SavetoEEPROM(eepromIdleAdress, Idle_Power);
  }


  AddPower = readIntFromEEPROM(AddPowerAdress);
  if (isnan(AddPower) || AddPower < 0) {
    AddPower = 0;
    SavetoEEPROM(AddPowerAdress, AddPower);
  }

  LimitPWM = readIntFromEEPROM(LimitPWMAdress);
  if (isnan(LimitPWM) || LimitPWM < 0) {
    LimitPWM = 100;
    SavetoEEPROM(LimitPWMAdress, LimitPWM);
  }

  if (use_ir == false) {
    int IR_ON = 0;
  } else {
    IR_ON = readIntFromEEPROM(IR_Adress);
    if (isnan(IR_ON) || IR_ON < 0) {
      IR_ON = 0;
      SavetoEEPROM(IR_Adress, IR_ON);
    }
  }

  TMax = readIntFromEEPROM(TMaxAdress);
  if (isnan(TMax) || TMax < 100) {
    TMax = 450;
    SavetoEEPROM(TMaxAdress, TMax);
  }

  DisplayType = readIntFromEEPROM(DisplayTypeAdress);
  if (isnan(DisplayType)) {
    DisplayType = 0;
    SavetoEEPROM(DisplayTypeAdress, DisplayType);
  }

  Prob = readIntFromEEPROM(ProbAdress);
  if (Prob < 0 || Prob > 2) {
    Prob = 0;
    SavetoEEPROM(ProbAdress, Prob);
  }
  ButtonTime = millis();
  TempReadTime = millis();
}

void Calc() {
  FFlow = 0;
  if (Prob == 2) {  // for 30 V power Supply
    if (adj >= 0 && adj > 4 * Trent) FFlow = (sin(radians(adj)) * 220);
    FFlow = FFlow + (cos(radians(adj)) * (TargetTemp / 80));
    FFlow = FFlow + (pow(radians(TargetTemp), 2)) - (TargetTemp / (10 + tune / 2));
    FFlow = FFlow + (AddPower + tune) * (TargetTemp / 300);
  } else if (Prob == 1) {  //for 20 V power Supply
    if (adj >= -1) {
      FFlow = (sin(radians(adj)) * 320);
      FFlow = FFlow + (pow(radians(TargetTemp), 2)) - (TargetTemp / (25 + tune / 2));
    }
    FFlow = FFlow + (cos(radians(adj)) * (TargetTemp / 7));
    FFlow = FFlow + (AddPower + tune) * (TargetTemp / 400);
  } else if (Prob == 0) {  //for 18 V power Supply
    if (adj >= 0 && adj > 4 * Trent) FFlow = (sin(radians(adj)) * 220);
    FFlow = FFlow + (cos(radians(adj)) * (TargetTemp / 6.5));
    FFlow = FFlow + (pow(radians(TargetTemp), 2.1)) - (TargetTemp / (10 + tune / 2));
    FFlow = FFlow + (AddPower + tune) * (TargetTemp / 300);
    /*
    if (adj >= -1) FFlow = (sin(radians(adj)) * 300);
    if (adj >= -6) FFlow = FFlow + (cos(radians(adj)) * (TargetTemp / 7));
    if (TargetTemp >= 300) {
      FFlow = FFlow + (pow(radians(TargetTemp), 2));
    } else {
      FFlow = FFlow + (pow(radians(TargetTemp), 2)) - (TargetTemp / 20);
    }
    FFlow = FFlow + (AddPower + tune) * (TargetTemp / 250);
    */
  }

  if (FFlow > 255) FFlow = 255;
  if (FFlow < 0) FFlow = 0;

  Flow[Sira] = FFlow;

  if (Sira == 3) {
    Sira = 0;
    if (adj * Trent < adj) tune = tune + 0.7;
    else tune = tune - 0.7;
    //if ((Trent - adj ) < 0)tune = tune + 0.7; else tune = tune - 0.7;
    if (tune > 10) tune = 10;
    if (tune < -10) tune = -10;
    //if (adj > 3)tune = 0;
  } else {
    Sira = Sira + 1;
  }

  int FFlowx = 0;
  for (int ib = 0; ib <= 3; ib++) {
    FFlowx = FFlowx + Flow[ib];
  }
  FFlow = FFlowx / 4;

  if (FFlow > 255) FFlow = 255;
  if (FFlow < 0) FFlow = 0;
}

void loop() {  //************************************  LOOP  ***********************************

  if (Alarming == 2) {
    Alarm();
    delay(250);
    TempReadTime = millis();
    return;
  }

  if (digitalRead(BUTTON_PIN) == LOW) {
    delay(50);
    if (digitalRead(BUTTON_PIN) == LOW) {
      BUTTON_PIN_ACT();
      TempReadTime = millis();
      return;
    }
  }


  if (Knob_Dir == 1) {
    Knob_Dir = 0;
    if (PageNumber == 3) {
      if (DDrive == 0) {
        PageNumber = 4;
      } else {
        PageNumber = 5;
      }
    } else {
      if (PageNumber == PageCount) {
        PageNumber = 1;
      } else {
        PageNumber = PageNumber + 1;
        if (PageNumber > PageCount) PageNumber = 1;
        if (use_ir == false) {
          int IR_ON = 0;
          if (PageNumber == 6) PageNumber = 7;
        }
        if (use_pwm == false) {
          if (PageNumber == 3) PageNumber = 4;
        }
      }
    }
  } else if (Knob_Dir == -1) {
    Knob_Dir = 0;
    if (PageNumber == 5) {
      if (DDrive == 0) {
        PageNumber = 4;
      } else {
        PageNumber = 3;
      }
    } else {
      if (PageNumber == 1) {
        PageNumber = PageCount;
      } else {
        PageNumber = PageNumber - 1;
        if (PageNumber < 1) PageNumber = 1;
        if (use_ir == false) {
          int IR_ON = 0;
          if (PageNumber == 6) PageNumber = 5;
        }
        if (use_pwm == false) {
          if (PageNumber == 3) PageNumber = 2;
        }
      }
    }
  }

  if ((millis() - TempReadTime) > 500) {  //*****************************  TempRead,
    Temperature = thermocouple.readCelsius();
    if (OldTemp == 0) OldTemp = Temperature;
    Trent = Temperature - OldTemp;
    OldTemp = Temperature;
    TempReadTime = millis();

    if (isnan(Temperature)) {
      if (Heater_Connected == 1) {
        Heater_Connected = 0;
        Beep(200);
      }
      TargetTemp = 0;
      Power = 0;
    } else if (Heater_Connected == 0) {
      Beep(100);
      Heater_Connected = 1;
    }

    Gap = TargetTemp - Temperature;
    if (Gap < -88) {
      adj = -88;
      tune = -10;
    } else if (Gap > 88) {
      adj = 88;
      tune = 0;
    } else {
      adj = Gap;
    }

    if (Temperature > VMO) {
      analogWrite(POWER_PIN, 0);
      OverTemp = 1;
      FFlow = 0;
      Power = 0;
      TargetTemp = 0;
      if (DDrive == 1) DDrive = 0;
      Beep(10);
      SET_LEVEL = 0;
    } else {
      OverTemp = 0;
    }

    if (DDrive == 0) {
      if (TargetTemp == 0) {
        FFlow = 0;
        Power = 0;
        miniReset();
      } else {
        if (Gap < -88) {
          adj = -88;
          tune = -10;
          FFlow = 0;
        } else if (Gap > 88) {
          adj = 88;
          tune = 0;
          FFlow = 255;
        } else {
          adj = Gap;
          Calc();
        }
        Power = map(FFlow, 0, 255, 0, 100);
        if (Power > LimitPWM) {
          Power = LimitPWM;
          FFlow = (Power * 2.55);
        }
      }
    } else {
      if (Power == 0) {
        FFlow = 0;
        miniReset();
      } else {
        FFlow = (Power * 2.55);
        if (FFlow > 255) FFlow = 255;
        if (FFlow < 0) FFlow = 0;
      }
    }

    if (Power > Idle_Power) miniReset();

    if (Blink == 0) Blink = 1;
    else Blink = 0;

    if (DisplayType == 1) {
      Show_FX();
      if (SET_LEVEL != 0) {
        Goster();
      }
    } else {
      Goster();
    }
  }

  if (Temperature > TMax) {
    analogWrite(POWER_PIN, 0);
    Protection = 1;
    if (DDrive == 0) {
      if (TargetTemp > (TMax - 10)) TargetTemp = TMax - 5;
    }
    Beep(10);
  } else {
    Protection = 0;
  }

  if (OverTemp == 1) {
    u8g.firstPage();
    do {
      LimitTemp();
    } while (u8g.nextPage());
    return;
  }

  if ((millis() - ButtonTime) > (MAX_WAIT_TIME * 1000)) {
    if (SET_LEVEL > 0) {
      SET_LEVEL = 0;
      Alarming = 1;
      OldTargetTemp = TargetTemp;
      BUTTON_PIN_ACT();
      miniReset();
    } else {
      if (IR_ON > 0 && Proximity == 1) {
        Proximity = 0;
      }
    }
    if (SaveEp == 1 && DDrive == 0) {
      SavetoEEPROM(eepromSetAdress, int(TargetTemp));
      SaveEp = 0;
    }
  }

  if (IR_ON > 0) {
    if (SET_LEVEL == 0) {
      digitalWrite(IR_TX_PIN, HIGH);  // set the LED on
      delay(1);
    }
    IR_Val = analogRead(IR_RX_PIN);
    digitalWrite(IR_TX_PIN, LOW);  // set the LED on
    IR_V = int((IR_Val * 62) / 1023);

    if (Proximity == 0 && SET_LEVEL == 0) {
      if (IR_Val > IR_ON) {
        if (IR_Count >= 6) {
          if (TargetTemp > 0) {
            miniReset();
          } else {
            Alarming = 1;
            BUTTON_PIN_ACT();
          }
          IR_Count = 0;
          Beep(1);
          Proximity = 1;
          ButtonTime = millis();
        } else {
          IR_Count++;
        }
      } else {
        IR_Count = 0;
      }
    }
  }
  if (!Heater_Connected) return;
  if (Protection != 1) {
    analogWrite(POWER_PIN, FFlow);
  } else {
    analogWrite(POWER_PIN, 0);
  }
}

void rotaryEncoder() {
  uint8_t encoderStatus = encoder.process();
  if (encoderStatus) {
    Flag_ReadTime = 1;
    if (SET_LEVEL == 0) {  //***************************************** SET_LEVEL == 0
      if (encoderStatus == DIR_CW) {
        if ((millis() - ButtonTime) > MIN_ELAPSED_TIME) {
          if (TargetTemp == 0 && (Alarming == 1)) WakeUp();
          if (DDrive == 0) {
            TargetTemp = TargetTemp + 1;
            tune = 0;
          } else {
            Power = Power + 1;
            if (Power > 100) {
              Power = 100;
            } else if (Power > LimitPWM) {
              Power = LimitPWM;
            }
            FFlow = (Power * 2.55);
          }
        } else {
          if (TargetTemp == 0 && (Alarming == 1)) WakeUp();
          if (DDrive == 0) {
            TargetTemp = TargetTemp + 10;
            tune = 0;
          } else {
            Power = Power + 5;
            if (Power > 100) {
              Power = 100;
            } else if (Power > LimitPWM) {
              Power = LimitPWM;
            }
            FFlow = (Power * 2.55);
          }
        }

        if (TargetTemp > VMO) TargetTemp = VMO;
        SaveEp = 1;
        ButtonTime = millis();
        Beep(5);
        miniReset();
      } else if (encoderStatus == DIR_CCW) {

        if ((millis() - ButtonTime) > MIN_ELAPSED_TIME) {
          if (TargetTemp == 0 && (Alarming == 1)) WakeUp();
          if (DDrive == 0) {
            TargetTemp = TargetTemp - 1;
            tune = 0;
          } else {
            Power = Power - 1;
            if (Power < 0) {
              Power = 0;
              FFlow = 0;
            } else {
              FFlow = (Power * 2.55);
              if (FFlow > 255) FFlow = 255;
              if (FFlow < 0) FFlow = 0;
            }
          }
        } else {
          if (TargetTemp == 0 && (Alarming == 1)) WakeUp();
          if (DDrive == 0) {
            TargetTemp = TargetTemp - 10;
            tune = 0;
          } else {
            Power = Power - 5;
            if (Power < 0) {
              Power = 0;
              FFlow = 0;
            } else {
              FFlow = (Power * 2.55);
              if (FFlow > 255) FFlow = 255;
              if (FFlow < 0) FFlow = 0;
            }
          }
        }

        if (TargetTemp < 0) TargetTemp = 0;
        if (TargetTemp > VMO) TargetTemp = VMO;
        SaveEp = 1;
        ButtonTime = millis();
        Beep(5);
        miniReset();
      }
    } else if (SET_LEVEL == 1) {  //***************************************** SET_LEVEL == 1

      if (encoderStatus == DIR_CW) {
        Knob_Dir = 1;
      } else if (encoderStatus == DIR_CCW) {
        Knob_Dir = -1;
      }
      Beep(5);
      ButtonTime = millis();
    } else if (SET_LEVEL == 2) {  //***************************************** SET_LEVEL == 2
      if (DDrive == 1) {
        FFlow = 0;
        TargetTemp = 0;
      }
      if (PageNumber == 1) {  //******************************* PageNumber == 1
        if (encoderStatus == DIR_CW) {
          if ((millis() - ButtonTime) > MIN_ELAPSED_TIME) {
            Idle_minute = Idle_minute + 1;
          } else {
            Idle_minute = Idle_minute + 2;
          }

          if (Idle_minute > 99) {
            Idle_minute = 1;
          }
        } else if (encoderStatus == DIR_CCW) {
          if ((millis() - ButtonTime) > MIN_ELAPSED_TIME) {
            Idle_minute = Idle_minute - 1;
          } else {
            Idle_minute = Idle_minute - 2;
          }

          if (Idle_minute < 1 || Idle_minute > 99) {
            Idle_minute = 99;
            Beep(200);
          }
        }
        Beep(5);
        ButtonTime = millis();
      } else if (PageNumber == 2) {  //********************************************** PageNumber == 2
        if (encoderStatus == DIR_CW) {
          if ((millis() - ButtonTime) > MIN_ELAPSED_TIME) {
            Idle_Power = Idle_Power + 1;
          } else {
            Idle_Power = Idle_Power + 2;
          }
          if (Idle_Power > 90) {
            Idle_Power = 1;
            Beep(200);
          }
        } else if (encoderStatus == DIR_CCW) {
          if ((millis() - ButtonTime) > MIN_ELAPSED_TIME) {
            Idle_Power = Idle_Power - 1;
          } else {
            Idle_Power = Idle_Power - 5;
          }
          if (Idle_Power < 1 || Idle_Power > 90) {
            Idle_Power = 90;
            Beep(200);
          }
        }
        Beep(5);
        ButtonTime = millis();
      } else if (PageNumber == 3) {  //********************************************* PageNumber == 3
        if (encoderStatus == DIR_CW) {
          if (DDrive == 0) {
            DDrive = 1;
            stopwatchPause();
          } else {
            DDrive = 0;
            stopwatchStart();
          }
          FFlow = 0;
          TargetTemp = 0;
        } else if (encoderStatus == DIR_CCW) {
          if (DDrive == 0) {
            DDrive = 1;
            stopwatchPause();
            Beep(200);
          } else {
            DDrive = 0;
            stopwatchStart();
          }
          FFlow = 0;
          TargetTemp = 0;
        }
        Beep(5);
        ButtonTime = millis();
      } else if (PageNumber == 4) {  //********************************************* PageNumber == 4

        if (encoderStatus == DIR_CW) {
          if ((millis() - ButtonTime) > MIN_ELAPSED_TIME) {
            AddPower = AddPower + 1;
          } else {
            AddPower = AddPower + 2;
          }
          if (AddPower > 50) {
            AddPower = 50;
          }
        } else if (encoderStatus == DIR_CCW) {
          if ((millis() - ButtonTime) > MIN_ELAPSED_TIME) {
            AddPower = AddPower - 1;
          } else {
            AddPower = AddPower - 2;
          }
          if (AddPower > 50) {
            AddPower = 50;
          }
          if (AddPower < -50) {
            AddPower = -50;
          }
        }
        Beep(5);
        ButtonTime = millis();
      } else if (PageNumber == 5) {  //********************************************* PageNumber == 5

        if (encoderStatus == DIR_CW) {
          if ((millis() - ButtonTime) > MIN_ELAPSED_TIME) {
            LimitPWM = LimitPWM + 1;
          } else {
            LimitPWM = LimitPWM + 5;
          }
          if (LimitPWM > 100) {
            LimitPWM = 100;
          }
        } else if (encoderStatus == DIR_CCW) {
          if ((millis() - ButtonTime) > MIN_ELAPSED_TIME) {
            LimitPWM = LimitPWM - 1;
          } else {
            LimitPWM = LimitPWM - 5;
          }
          if (LimitPWM > 100) {
            LimitPWM = 100;
          }
          if (LimitPWM < 1) {
            LimitPWM = 1;
          }
        }
        if (DDrive == 1) {
          if (Power > LimitPWM) Power = LimitPWM;
          FFlow = (Power * 2.55);
        }
        Beep(5);
        ButtonTime = millis();
      } else if (PageNumber == 6) {  //******************************************* PageNumber == 6
        if (encoderStatus == DIR_CW) {
          IR_ON = IR_ON + 50;
          if (IR_ON > 1000) IR_ON = 0;
        } else if (encoderStatus == DIR_CCW) {
          IR_ON = IR_ON - 50;
          if (IR_ON <= 0) IR_ON = 0;
        }
        Beep(5);
        ButtonTime = millis();
      } else if (PageNumber == 7) {  //******************************************* PageNumber == 7

        if (encoderStatus == DIR_CW) {
          if ((millis() - ButtonTime) > MIN_ELAPSED_TIME) {
            TMax = TMax + 1;
          } else {
            TMax = TMax + 5;
          }
          if (TMax > VMO) {
            TMax = VMO;
          }
        } else if (encoderStatus == DIR_CCW) {
          if ((millis() - ButtonTime) > MIN_ELAPSED_TIME) {
            TMax = TMax - 1;
          } else {
            TMax = TMax - 5;
          }
          if (TMax < 100) {
            TMax = 100;
          }
          if (TMax > VMO) {
            TMax = VMO;
          }
        }
        Beep(5);
        ButtonTime = millis();
      } else if (PageNumber == 8) {  //*************************************** PageNumber == 8
        if (encoderStatus == DIR_CW) {
          if (DisplayType == 1)
            DisplayType = 0;
          else
            DisplayType = 1;
          Cleared = 0;
        } else if (encoderStatus == DIR_CCW) {
          if (DisplayType == 1)
            DisplayType = 0;
          else
            DisplayType = 1;
          Cleared = 0;
        }
        Beep(5);
        ButtonTime = millis();
      } else if (PageNumber == 9) {  //*************************************** PageNumber == 9
        if (encoderStatus == DIR_CW) {
          MAX_WAIT_TIME = MAX_WAIT_TIME + 1;
          if (MAX_WAIT_TIME >= 20) MAX_WAIT_TIME = 20;
        } else if (encoderStatus == DIR_CCW) {
          MAX_WAIT_TIME = MAX_WAIT_TIME - 1;
          if (MAX_WAIT_TIME <= 3) MAX_WAIT_TIME = 3;
        }
        Beep(5);
        ButtonTime = millis();
      } else if (PageNumber == 10) {  //*************************************** PageNumber == 10
        if (encoderStatus == DIR_CW) {
          Prob = Prob + 1;
          if (Prob > 2) Prob = 2;
        } else if (encoderStatus == DIR_CCW) {
          Prob = Prob - 1;
          if (Prob < 0) Prob = 0;
        }
        Beep(5);
        ButtonTime = millis();
      } else if (PageNumber == 11) {  //*************************************** PageNumber == 10
        Beep(5);
        ButtonTime = millis();
      }
    }
  }
}

void Goster() {
  u8g.firstPage();
  do {
    Show();
  } while (u8g.nextPage());
}

void Show() {
  u8g.setFont(u8g_font_6x10);
  if (SET_LEVEL == 0) {  // //***************************************** SET_LEVELS = 0
    u8g.drawStr(12, 9, "Hot End");
    u8g.setPrintPos(82, 9);
    if (DDrive == 0) {
      u8g.println(" Auto ");
    } else {
      u8g.println("Manual");
    }
    u8g.setFont(u8g_font_helvR14r);
    if (Heater_Connected == 0) {
      if (Blink == 1) u8g.drawStr(10, 25, " --- !");
    } else {
      if (Temperature < 10) {
        u8g.setPrintPos(5, 25);
      } else if (Temperature < 100) {
        u8g.setPrintPos(15, 25);
      } else if (Temperature < 800) {
        u8g.setPrintPos(5, 25);
      }
      u8g.print((int)Temperature);
      u8g.setFont(u8g_font_6x10);
      u8g.setPrintPos(40, 17);
      if (Blink == 0) u8g.print("o");
      u8g.drawStr(47, 22, "C");
    }
    u8g.setFont(u8g_font_helvR14r);
    if (DDrive == 0) {
      if (Heater_Connected == 0) {
        if (Blink == 1) u8g.drawStr(80, 25, " --- !");
      } else if (Protection == 1) {
        if (Blink == 1) u8g.drawStr(75, 25, "Prot !");
      } else if ((int)TargetTemp == 0) {
        if (Blink == 1) u8g.drawStr(82, 25, "idle!");
      } else {
        if ((int)TargetTemp < 10) {
          u8g.setPrintPos(92, 25);
        } else if ((int)TargetTemp < 100) {
          u8g.setPrintPos(87, 25);
        } else if ((int)TargetTemp < 800) {
          u8g.setPrintPos(78, 25);
        }
        u8g.print((int)TargetTemp);
        u8g.setFont(u8g_font_6x10);
        u8g.drawStr(110, 17, "o");
        u8g.drawStr(117, 22, "C");
      }

    } else {
      if (Heater_Connected == 0) {
        if (Blink == 1) u8g.drawStr(80, 25, " --- !");
      } else if (Protection == 1) {
        if (Blink == 1) u8g.drawStr(75, 25, "Prot !");
      } else {
        u8g.drawStr(74, 25, "PWM  ");
      }
    }

    u8g.setFont(u8g_font_helvR14r);
    if ((DDrive == 0 && TargetTemp == 0) || (DDrive == 1 && FFlow == 0)) {
      u8g.drawStr(15, 50, "0:00");
    } else {
      u8g.setPrintPos(15, 50);
      restmin = Idle_minute - minute;
      restsec = 60 - second;
      if (restmin < 0) restsec = 0;
      if (restsec == 60) {
        restsec = 0;
        if (restmin == Idle_minute) restmin = Idle_minute;
      } else {
        restmin = Idle_minute - minute - 1;
        restsec = 60 - second;
      }
      if (restmin < 0) restmin = 0;


      u8g.setPrintPos(15, 50);
      if (restmin < 10) {
        u8g.print((int)restmin);
      } else if (restmin < 100) {
        u8g.print((int)restmin);
      }
      u8g.print(":");
      if (restsec < 10) {
        u8g.print("0");
      }
      u8g.print((int)restsec);
    }

    u8g.setFont(u8g_font_helvR14r);
    if (Power < 10) {
      u8g.setPrintPos(95, 50);
    } else if (Power < 100) {
      u8g.setPrintPos(85, 50);
    } else if (Power == 100) {
      u8g.setPrintPos(75, 50);
    }
    u8g.print((int)Power);
    u8g.drawStr(110, 50, "%");

    u8g.setFont(u8g_font_6x10);
    u8g.setPrintPos(5, 61);
    if (IR_ON > 0 && Proximity == 0) {
      u8g.print("Timer+ IR");
    } else {
      if (Proximity == 0) {
        u8g.print("Timer");
      } else {
        u8g.print("Timer+");
      }
    }

    if (LimitPWM != 100) {
      if (AddPower != 0 && DDrive == 0) {
        u8g.drawStr(67, 61, "CAL.LimPWM");
      } else if (DDrive == 0) {
        u8g.drawStr(67, 61, "LimitedPWM");
      }
    } else {
      if (AddPower != 0 && DDrive == 0) {
        u8g.drawStr(82, 61, "CAL.PWM");
      } else if (DDrive == 0) {
        u8g.drawStr(108, 61, "PWM");
      }
    }
    u8g.drawRFrame(0, 0, 63, 30, 4);    //  HOTEND draws frame with rounded edges
    u8g.drawRFrame(65, 0, 63, 30, 4);   // SET TEMP draws frame with rounded edges
    u8g.drawRFrame(0, 32, 63, 32, 4);   // TIMER draws frame with rounded edges
    u8g.drawRFrame(65, 34, 63, 30, 4);  // PWM draws frame with rounded edges
    if (IR_ON > 0 && SET_LEVEL == 0) {
      u8g.drawLine(0, 51, IR_V, 51);  // TIMER draws frame with rounded edges
    }
  } else if (SET_LEVEL > 0) {  //***************************************** SET_LEVELS
    if (PageNumber != 11) u8g.drawRFrame(25, 0, 80, 14, 4);
    if (PageNumber == 1) {  //******************* PageNumber == 1
      u8g.drawStr(37, 10, "SETTING 1");
      u8g.drawStr(33, 30, "SLEEP TIMER");

      if (SET_LEVEL != 1) {
        u8g.setFont(u8g_font_helvR14r);
        u8g.setPrintPos(50, 50);
      } else {
        u8g.setFont(u8g_font_6x10);
        u8g.setPrintPos(47, 50);
      }

      u8g.print((int)Idle_minute);
      u8g.setFont(u8g_font_6x10);
      u8g.print(" min");
    } else if (PageNumber == 2) {  //******************* PageNumber == 2
      u8g.drawStr(37, 10, "SETTING 2");
      u8g.drawStr(1, 30, "SLEEP POWER THRESHOLD");

      if (SET_LEVEL != 1) {
        u8g.setFont(u8g_font_helvR14r);
        u8g.setPrintPos(49, 50);
      } else {
        u8g.setFont(u8g_font_6x10);
        u8g.setPrintPos(52, 50);
      }

      if (Idle_Power < 10) {
        u8g.print("0");
      }
      u8g.print((int)Idle_Power);
      u8g.print(" %");
    } else if (PageNumber == 3) {  //******************* PageNumber == 3
      u8g.drawStr(37, 10, "SETTING 3");
      u8g.drawStr(10, 30, "PWM MANUAL CONTROL");

      if (SET_LEVEL != 1) {
        u8g.setFont(u8g_font_helvR14r);
        u8g.setPrintPos(48, 50);
      } else {
        u8g.setFont(u8g_font_6x10);
        u8g.setPrintPos(55, 50);
      }
      if (DDrive == 0) {
        u8g.print("OFF");
      } else {
        u8g.print("ON ");
      }
    } else if (PageNumber == 4) {  //******************* PageNumber == 4
      u8g.drawStr(37, 10, "SETTING 4");
      //u8g.drawStr(1, 30, "Tune");
      u8g.drawStr(32, 30, "CALIBRATION");

      if (SET_LEVEL != 1) {
        u8g.setFont(u8g_font_6x10);
        /*
          u8g.setPrintPos(1, 50);
          if (tune < 0) {
          u8g.print((int)tune);
          } else {
          u8g.print("+");
          u8g.print((int)tune);
          }
        */
        u8g.setFont(u8g_font_helvR14r);
        u8g.drawStr(1, 50, "ADJUST");
      } else {
        u8g.setFont(u8g_font_6x10);
        /*
          u8g.setPrintPos(1, 50);
          if (tune < 0) {
          u8g.print((int)tune);
          } else {
          u8g.print("+");
          u8g.print((int)tune);
          }
        */
        u8g.drawStr(15, 50, "ADJUST POWER");
      }
      u8g.setPrintPos(99, 50);

      if (AddPower <= -10) {
        u8g.print("");
      } else if (AddPower < 0) {
        u8g.print(" ");
      } else if (AddPower < 10) {
        u8g.print("  ");
      } else if (AddPower < 100) {
        u8g.print("");
      }
      u8g.print((int)AddPower);

    } else if (PageNumber == 5) {  //******************* PageNumber == 5
      u8g.drawStr(37, 10, "SETTING 5");
      u8g.drawStr(25, 30, "PWM MAX LIMIT");
      if (SET_LEVEL != 1) {
        u8g.setFont(u8g_font_helvR14r);
        u8g.setPrintPos(45, 50);
        if (LimitPWM < 10) {
          u8g.print("  ");
        } else if (LimitPWM < 100) {
          u8g.print(" ");
        }
        u8g.print((int)LimitPWM);
        u8g.setFont(u8g_font_6x10);
        u8g.drawStr(75, 50, " %");
      } else {
        u8g.setPrintPos(45, 50);
        u8g.setFont(u8g_font_6x10);
        if (LimitPWM < 10) {
          u8g.print("  ");
        } else if (LimitPWM < 100) {
          u8g.print(" ");
        }
        u8g.print((int)LimitPWM);
        u8g.setFont(u8g_font_6x10);
        u8g.drawStr(69, 50, " %");
      }

    } else if (PageNumber == 6) {  //******************* PageNumber == 6
      if (IR_Val > IR_ON) {
        Beep(1);
        miniReset();
        ButtonTime = millis();
      }
      u8g.drawStr(37, 10, "SETTING 6");
      u8g.drawStr(1, 30, "RX");
      u8g.drawStr(30, 30, "IR WAKEUP");
      u8g.drawStr(95, 30, "SENS");
      if (SET_LEVEL == 1) {
        u8g.setFont(u8g_font_6x10);
        u8g.setPrintPos(1, 50);
        u8g.print((int)IR_Val);
        u8g.setPrintPos(50, 50);
        if (IR_ON == 0) {
          u8g.print("OFF");
        } else {
          u8g.print("ON ");
          u8g.setPrintPos(95, 50);
          u8g.print(IR_ON);
        }
      } else {
        u8g.setFont(u8g_font_helvR14r);
        u8g.setPrintPos(1, 50);
        u8g.print((int)IR_Val);
        u8g.setPrintPos(45, 50);
        if (IR_ON == 0) {
          u8g.print("OFF");
        } else {
          u8g.print("ON");
          u8g.setPrintPos(85, 50);
          u8g.print(IR_ON);
        }
      }
    } else if (PageNumber == 7) {  //******************* PageNumber == 7
      u8g.drawStr(37, 10, "SETTING 7");
      u8g.drawStr(1, 30, "PROTECTION LIMIT TEMP");
      if (SET_LEVEL != 1) {
        u8g.setFont(u8g_font_helvR14r);
        u8g.drawStr(13, 50, "TMAX:");
        u8g.setPrintPos(75, 50);
        u8g.print((int)TMax);
        u8g.setFont(u8g_font_6x10);
        u8g.drawStr(104, 42, "o");
        u8g.drawStr(110, 46, "C");
      } else {
        u8g.setFont(u8g_font_6x10);
        u8g.drawStr(32, 50, "TMAX:");
        u8g.setPrintPos(65, 50);
        u8g.print((int)TMax);
        u8g.drawStr(86, 47, "o");
        u8g.drawStr(92, 50, "C");
      }

    } else if (PageNumber == 8) {  //******************* PageNumber == 8
      u8g.drawStr(37, 10, "SETTING 8");
      u8g.drawStr(30, 30, "SCREEN TYPE");
      if (SET_LEVEL != 1) {
        u8g.setFont(u8g_font_helvR14r);
        u8g.setPrintPos(10, 50);
        if (DisplayType == 0) u8g.print("WINDOWED");
        else u8g.print("  GRAPHIC");
      } else {
        u8g.setFont(u8g_font_6x10);
        u8g.setPrintPos(40, 50);
        if (DisplayType == 0) u8g.print("WINDOWED");
        else u8g.print(" GRAPHIC");
      }
    } else if (PageNumber == 9) {  //******************* PageNumber == 8
      u8g.drawStr(37, 10, "SETTING 9");
      u8g.drawStr(15, 30, "SETTING WAIT TIME");
      if (SET_LEVEL != 1) {
        u8g.setFont(u8g_font_helvR14r);
        u8g.setPrintPos(37, 50);
        u8g.print(MAX_WAIT_TIME);
      } else {
        u8g.setFont(u8g_font_6x10);
        u8g.setPrintPos(50, 50);
        u8g.print(MAX_WAIT_TIME);
      }
      u8g.drawStr(60, 50, "Sec");
    } else if (PageNumber == 10) {  //******************* PageNumber == 10
      u8g.drawStr(37, 10, "SETTING 10");
      u8g.drawStr(1, 30, "INPUT VOLTAGE SETTING");
      if (SET_LEVEL != 1) {
        u8g.setFont(u8g_font_helvR14r);
        u8g.setPrintPos(25, 50);
        if (Prob == 0) {
          u8g.print("18V input");
        } else if (Prob == 1) {
          u8g.print("20V input");
        } else if (Prob == 2) {
          u8g.print("24V input");
        }
      } else {
        u8g.setFont(u8g_font_6x10);
        u8g.setPrintPos(40, 50);
        if (Prob == 0) {
          u8g.print("18V input");
        } else if (Prob == 1) {
          u8g.print("20V input");
        } else if (Prob == 2) {
          u8g.print("24V input");
        }
      }
    } else if (PageNumber == 11) {  //******************* PageNumber == 11
      u8g.drawRFrame(27, 10, 80, 45, 4);
      u8g.setFont(u8g_font_helvR14r);
      u8g.drawStr(45, 35, "EXIT");
      u8g.setFont(u8g_font_6x10);
      u8g.drawStr(55, 50, Version.c_str());
    }
  }
}

void Show_FX() {
  if (Temperature < 300 && TempChanged == 2) {
    TempChanged = 0;
    for (int i = 0; i <= x; i++) {
      y[i] = -1;  //60
    }
  } else if (Temperature >= 300 && TempChanged == 1) {
    TempChanged = 0;
    for (int i = 0; i <= x; i++) {
      y[i] = -1;  //14
    }
  }
  if (Cleared == 0) {
    x = 19;
    clearY();
    Cleared = 1;
  }
  if (Temperature < 300) {
    y[x] = map(Temperature, 0, 300, (HEIGHT - 4), 13);
    if (TempChanged == 0) TempChanged = 1;
  } else {
    y[x] = map(Temperature, 300, VMO, (HEIGHT - 4), 13);
    if (TempChanged == 0) TempChanged = 2;
  }

  if (SET_LEVEL == 0) {
    u8g.firstPage();
    do {
      drawChart();
      DrawCHR();
      drawY();
    } while (u8g.nextPage());
    delay(1);
  }

  x++;
  if (x >= WIDTH - 1) {
    Cleared = 0;
  } else if (y[x] >= HEIGHT - 13) {
    Cleared = 0;
  }
}

void drawChart() {
  u8g.drawLine(19, 14, 19, 63);   // Y ax
  u8g.drawLine(19, 60, 128, 60);  // X ax

  for (int i = 14; i <= WIDTH; i = i + 10) {
    if (i <= WIDTH) {
      if (i > 20) {
        u8g.drawPixel(i, 59);
        u8g.drawPixel(i, 60);
        if (Temperature < 300) {
          int lim = map(TMax, 0, 300, (HEIGHT - 4), 13);
          u8g.drawPixel(i, lim);  //memo
        } else {
          int lim = map(TMax, 300, VMO, (HEIGHT - 4), 13);
          u8g.drawPixel(i, lim);  //memo
        }
      }
      u8g.drawPixel(20, i);
      u8g.drawPixel(21, i);
    } else if (i <= WIDTH) {
      u8g.drawPixel(i, 59);
      u8g.drawPixel(i, 60);
    }
  }
}

void drawY() {
  for (int i = 20; i < WIDTH; i++) {
    if (y[i] != -1) {
      u8g.drawPixel(i - 1, y[i - 1]);
    } else {
      //break;
    }
  }
  if (IR_ON > 0 && SET_LEVEL == 0) {
    u8g.drawLine(127, 63, 127, (63 - IR_V));
  }
}

void clearY() {
  for (int i = 0; i < WIDTH; i++) {
    y[i] = -1;
  }
}

void DrawCHR() {
  u8g.setFont(u8g_font_6x10);
  if (IR_ON > 0) {
    if (Proximity == 0) {
      u8g.drawStr(0, 8, "IR");
    }
    u8g.drawStr(13, 8, "+");
  } else {
    u8g.setPrintPos(0, 8);
    if (tune < 0) {
      u8g.print((int)tune);
    } else {
      u8g.drawStr(0, 8, "+");
      u8g.setPrintPos(6, 8);
      u8g.print((int)tune);
    }
    /*
      u8g.setPrintPos(95, 53);
      if (Trent < 0) {
      u8g.print(Trent);
      } else {
      u8g.setPrintPos(101, 53);
      //u8g.print("+");
      u8g.print(Trent);  //(int)
      }
    */
  }

  if (Temperature < 300) {
    if (y[x] > 23) {
      u8g.drawStr(0, 17, "300");
    }
    if (y[x] < 56) {
      u8g.drawStr(0, 64, "O");
      u8g.drawStr(7, 60, "o");
      u8g.drawStr(13, 64, "C");
    }
  } else {
    if (y[x] > 23) {
      u8g.setPrintPos(0, 17);
      u8g.print(VMO);
    }
    if (y[x] < 54) {
      if (Heater_Connected == 1) u8g.drawStr(0, 64, "300");
    }
  }

  if (Power < 10) {
    u8g.setPrintPos(72, 10);
    u8g.print((int)Power);
  } else if (Power < 100) {
    u8g.setPrintPos(66, 10);
    u8g.print((int)Power);
  } else if (Power == 100) {
    u8g.setPrintPos(60, 10);
    u8g.print((int)Power);
  }

  if (Blink == 0) {
    u8g.drawStr(80, 10, "%");
  } else {
    u8g.drawStr(80, 10, " ");
  }

  u8g.drawRFrame(55, 0, 35, 13, 2);  // draws frame with rounded edges
  if (DDrive == 0) {
    if ((int)TargetTemp < 10) {
      u8g.setPrintPos(103, 10);
    } else if ((int)TargetTemp < 100) {
      u8g.setPrintPos(100, 10);
    } else if ((int)TargetTemp < 800) {
      u8g.setPrintPos(94, 10);
    }
    u8g.print((int)TargetTemp);
    u8g.drawStr(115, 7, "o");
    u8g.drawStr(121, 10, "C");
  } else {
    u8g.drawStr(102, 10, "PWM");
  }

  u8g.drawRFrame(92, 0, 36, 13, 2);  // draws frame with rounded edges


  if ((DDrive == 0 && TargetTemp == 0) || (DDrive == 1 && FFlow == 0)) {
    u8g.setPrintPos(28, 10);
    u8g.print("0:00");
  } else {
    restmin = Idle_minute - minute;
    restsec = 60 - second;
    if (restmin < 0) restsec = 0;
    if (restsec == 60) {
      restsec = 0;
      if (restmin == Idle_minute) restmin = Idle_minute;
    } else {
      restmin = Idle_minute - minute - 1;
      restsec = 60 - second;
    }
    if (restmin < 0) restmin = 0;

    if (restmin < 10) {
      u8g.setPrintPos(28, 10);
      u8g.print((int)restmin);
    } else if (restmin < 100) {
      u8g.setPrintPos(22, 10);
      u8g.print((int)restmin);
    }
    u8g.print(":");
    if (restsec < 10) {
      u8g.print("0");
      u8g.print((int)restsec);
    } else {
      u8g.print((int)restsec);
    }
  }

  u8g.drawRFrame(19, 0, 34, 13, 2);  // draws frame with rounded edges
  u8g.setPrintPos(0, y[x] + 2);
  u8g.print((int)Temperature);

  u8g.setFont(u8g_font_helvR14r);
  if (Heater_Connected == 0) {
    if (Blink == 1) u8g.drawStr(32, 45, "HEATER !");
  } else if (Protection == 1) {
    if (Blink == 1) u8g.drawStr(25, 45, "Protection!");
  } else if (DDrive == 1 && Power == 0) {
    u8g.drawStr(55, 45, "idle!");
  } else if (DDrive == 0 && TargetTemp == 0) {
    if (Blink == 1) u8g.drawStr(55, 45, "idle !");
  }
}

void LimitTemp() {
  u8g.setFont(u8g_font_helvR14r);
  u8g.drawStr(11, 30, "Limit Temp!");
  u8g.drawStr(6, 50, "TEMP: ");
  u8g.setPrintPos(66, 50);
  u8g.print((int)Temperature);
  u8g.setFont(u8g_font_6x10);
  u8g.drawStr(105, 43, "o");
  u8g.drawStr(112, 47, "C");
}

void WakeUp() {
  Alarming = 0;
  TargetTemp = OldTargetTemp;
  stopwatchStart();
  Beep(5);
  loop();
}
void BUTTON_PIN_ACT() {
  miniReset();
  if (SET_LEVEL == 0) {  //*****************************************SET_LEVELS = 0
    if (Alarming == 1) {
      Alarming = 0;
      TargetTemp = OldTargetTemp;
      stopwatchStart();
      Beep(5);
      return;
    }

    SET_LEVEL = 1;
    Beep(100);
    stopwatchPause();

  } else if (SET_LEVEL == 1) {  //*****************************************SET_LEVELS = 1
    if (PageNumber == 11) {     //** ****************** EXIT
      stopwatchStart();
      SET_LEVEL = 0;
      PageNumber = 1;
      Beep(50);
    } else {
      SET_LEVEL = 2;
      Beep(100);
      stopwatchPause();
    }
  } else if (SET_LEVEL == 2) {  //*****************************************SET_LEVELS = 2

    if (PageNumber == 1) {  //******************* PageNumber == 1
      SavetoEEPROM(eepromIdleminAdress, Idle_minute);

    } else if (PageNumber == 2) {  //******************* PageNumber == 2
      SavetoEEPROM(eepromIdleAdress, Idle_Power);

    } else if (PageNumber == 3) {  //******************* PageNumber == 3
      SavetoEEPROM(DDriveAdress, DDrive);

    } else if (PageNumber == 4) {  //******************* PageNumber == 4
      SavetoEEPROM(AddPowerAdress, AddPower);

    } else if (PageNumber == 5) {  //******************* PageNumber == 5
      SavetoEEPROM(LimitPWMAdress, LimitPWM);

    } else if (PageNumber == 6) {  //******************* PageNumber == 6   IR_ON
      SavetoEEPROM(IR_Adress, IR_ON);

    } else if (PageNumber == 7) {  //******************* PageNumber == 7
      SavetoEEPROM(TMaxAdress, TMax);

    } else if (PageNumber == 8) {  //******************* PageNumber == 8
      SavetoEEPROM(DisplayTypeAdress, DisplayType);
    } else if (PageNumber == 9) {  //******************* PageNumber == 8
      SavetoEEPROM(MAX_WAIT_Adress, MAX_WAIT_TIME);
    } else if (PageNumber == 10) {  //******************* PageNumber == 8
      SavetoEEPROM(ProbAdress, Prob);
    }
    SET_LEVEL = 1;
    Beep(100);
    stopwatchPause();
  }
  delay(200);
  ButtonTime = millis();
}

void SavetoEEPROM(int address, int number) {
  byte byte1 = number >> 8;
  byte byte2 = number & 0xFF;
  EEPROM.write(address, byte1);
  EEPROM.write(address + 1, byte2);
}

int readIntFromEEPROM(int address) {
  return (EEPROM.read(address) << 8) + EEPROM.read(address + 1);
}

void Alarm() {
  Alarming = 1;
  Beep(100);
  OldTargetTemp = TargetTemp;
  TargetTemp = 0;
  FFlow = 0;
  stopwatchReset();
}

void Beep(int sure) {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(sure);
  digitalWrite(BUZZER_PIN, LOW);
}

void stopwatchStart()  //timer1 on
{
  Timer1.start();
}

void stopwatchPause()  //timer1 off if [CS12 CS11 CS10] is [0 0 0].
{
  Timer1.stop();
}

void miniReset() {
  Flag_ReadTime = 0;
  _microsecond_10 = 0;
  _second = 0;
  _minute = 0;
  microsecond_10 = 0;
  second = 0;
  minute = 0;
}

void stopwatchReset() {
  Flag_ReadTime = 0;
  _microsecond_10 = 0;
  _second = 0;
  _minute = 0;
  microsecond_10 = 0;
  second = 0;
  minute = 0;
  stopwatchPause();
}

void TimingISR() {
  if (SET_LEVEL > 0) return;
  microsecond_10++;

  if (microsecond_10 == 100) {
    second++;
    if (second == 60) {
      second = 0;
      minute++;
      if (minute > 120) {
        minute = 0;
      }
    }
    if (minute == Idle_minute) {
      if (Alarming == 0) {
        Alarming = 2;
      }
      return;
    }
    microsecond_10 = 0;
  }
  if (Flag_ReadTime == 0) {
    _microsecond_10 = microsecond_10;
    _second = second;
    _minute = minute;
  }
}