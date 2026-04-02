/*
 * E-Auto-Wallbox für Typ2-Ladung
 * Basierend auf "Open EVSE Firmware"
 * 
 * Features:
 *  - Steuerung Pilot-Signal mit +12V/-12V oder PWM
 *  - Rückmessung Pilotspannung min/max
 *  - Zustandsmaschine inklusive Fehlerzustand
 *  - Fehler wird nach 10s wieder verlassen in Richtung "Standby"
 * 
 * Änderungshistorie:
 *  * 2020-03-31 Uwe:
 *       - Zustandsübergänge funktionieren, Trockentest an Schalterbox-EV-Simulator bestanden
 *  * 2020-04-01 Uwe:
 *       - LED-Streifen mit WS2812 angebunden, dafür grüne LED an D13 entfernt, weil D13 auch die rote On-Board-LED ist.
 *       - Strom von 12 auf 14 A erhoht, weil mit 12A PWM nur 11A (laut Sonoff) fließen. Ggf. ist die Kabelcodierung (1k5 am PP-Kontakt)
 *         jedoch das limitierende Element.
 *  * 2021-12-21 Uwe:
 *       - 5%-PWM for triggering Powerline-Communication
 *  * 2022-03-07 Uwe:
 *       - increased time for starting PWM from 200ms to 6s (just for testing)
 *  * 2022-03-10 Uwe:
 *       - time for starting PWM from 6s to 1s (just for testing)
 *       - bugfix: initialize the timer for PWM generation even on static pin output, and start it from 0 if PWM is started.
 *                 This avoids starting the PWM with an unintended looong -12V phase.
 *       - increased time for starting PWM from 200ms to 6s (just for testing)
 *  * 2022-08-10 Uwe:
 *       - PWM duty cycle adjustable by Poti on A2 (just for testing)
 *       - corrected the CP voltage scaling (was showing 12V even if electrically only 11V were present)
 *       - decreased time for starting the 5% PWM to 1 (this means "immediately", around 100ms)
 *       - added debug pin for timing measurement. Result:
 *           100 reads take ~12ms. But in between the reads, we need ~100ms for the delay(100) in the main loop.
 *       - to improve the latency, changed the main loop delay from 100ms to (20-12)ms. This leads to cycle time of 20ms.
 *  * 2022-08-12 Uwe:
 *       - Poti selects between 5% (left side) and adjustable (right side), and PWM is updated live
 *       
 * Todos:
 *    - Temperaturmessung, Abschaltung oder Ladestromreduzierung bei hoher Temperatur
 *    - Telemetriedaten in JSON-Format ausgeben, so dass z.B ein ESP8266 sie auf MQTT weiterleiten kann
 *    - Error-Codes mit den roten LEDs anzeigen (z.B. 1 LED = Diodencheck, 2 LEDs = Kurzschluss, 3 LEDs = Übertemperatur, ...)
 * 
 */

/*******************************************************************************/
/* LED-Streifen mit WS2812 */
#include <FastLED.h>
// How many leds are in the strip?
#define NUM_LEDS 6
// Data pin that led data will be written out over
#define DATA_PIN 11
// This is an array of leds.  One item for each led in your strip.
CRGB leds[NUM_LEDS];


void setLedStrip(uint32_t x) {
 uint8_t i;
 for(i = 0; i < NUM_LEDS; i++) {
      leds[i] = x;
 }
 // Show the leds
 FastLED.show();
}

/***********************************************************************************/
/* global variables and definitions */
 
#define VOLT_PIN 1 // ControlPilot analog voltage reading pin A1
#define POTI_PIN 2 /* Poti at A2 */
#define RED_LED_PIN 5 // Digital pin
#define CHARGING_PIN 8 // digital Charging LED and Relay Trigger pin
#define PILOT_PIN 10 // n.b. PILOT_PIN *MUST* be digital 10 because SetPWM() assumes it
//#define GREEN_LED_PIN 13 // Digital pin
#define DEBUG_PIN 6 /* for debugging */


int8_t I_Lade_Soll_A = 0;
int8_t I_Lade_SollAlt_A = -2;

//int8_t globalAmps = 0;
uint16_t nAdcPoti;


#define PWM_FIVE_PERCENT (256/20); /* 5% of full range. */
#define MAIN_LOOP_CYCLE_TIME_MS 20 /* 20 milliseconds main loop cycle time */

/**********************************************************************************/
/* Pilot-Signal-Erzeugung und -Messung */

typedef enum {
  PILOT_STATE_P12,PILOT_STATE_PWM,PILOT_STATE_N12} 
PILOT_STATE;
class J1772Pilot {
  uint8_t m_bit;
  uint8_t m_port;
  PILOT_STATE m_State;
public:
  J1772Pilot() {
  }
  void Init();
  void SetState(PILOT_STATE pstate); // P12/N12
  PILOT_STATE GetState() { 
    return m_State; 
  }
  int SetPWM(int amps); // 12V 1KHz PWM
};

J1772Pilot m_Pilot;
uint8_t pilotVoltageRange;




int16_t uPilotHigh_mV;
int16_t uPilotLow_mV;
uint8_t isPwmOn=0;

void readPilotVoltages(void) {
 int16_t reading;
 uPilotLow_mV = 32000;
 uPilotHigh_mV = -32000;
 digitalWrite(DEBUG_PIN,HIGH);
 // 1x = 114us 20x = 2.3ms 100x = 11.3ms
 for (int i=0;i < 100;i++) {
    reading = analogRead(VOLT_PIN);  // measures pilot voltage
    reading -= 561; /* entspricht 0V am ControlPilot */
    reading *= 34; /* auf Millivolt skalieren */
    if (reading > uPilotHigh_mV) {
        uPilotHigh_mV = reading;
      }
      else if (reading < uPilotLow_mV) {
        uPilotLow_mV = reading;
      }
    }
 digitalWrite(DEBUG_PIN,LOW);   
}
#define PILOT_RANGE_A 1
#define PILOT_RANGE_B 2
#define PILOT_RANGE_C 3
#define PILOT_RANGE_ERROR 4
#define PILOT_RANGE_ERROR_DIODE_CHECK 5

uint8_t convertPilotVoltageToRange(void) {
  uint8_t rc;
  if (isPwmOn && ((uPilotLow_mV>-10000) || (uPilotLow_mV<-13500))) {
    /* Bei PWM sollten wir -12V sehen, sonst ist etwas faul */
    rc = PILOT_RANGE_ERROR_DIODE_CHECK;
  } else if ((uPilotHigh_mV>=10000) and (uPilotHigh_mV<=13500)) {
    rc= PILOT_RANGE_A; /* 12V, not connected */
  } else if ((uPilotHigh_mV>=7000) and (uPilotHigh_mV<=10500)) {
    rc= PILOT_RANGE_B; /* 9V, vehicle detected */
  } else if ((uPilotHigh_mV>=4000) and (uPilotHigh_mV<=7500)) {
    rc= PILOT_RANGE_C; /* 6V, ready, charging */
  } else {
    rc= PILOT_RANGE_ERROR; /* Defekt */
  }
  return rc;
}

void printPilotRange(uint8_t r) {
  switch (r) {
    case PILOT_RANGE_A: Serial.print("Range A: not connected"); break;
    case PILOT_RANGE_B: Serial.print("Range B: vehicle detected"); break;
    case PILOT_RANGE_C: Serial.print("Range C: ready/charging"); break;
    case PILOT_RANGE_ERROR: Serial.print("Range error"); break;
    case PILOT_RANGE_ERROR_DIODE_CHECK: Serial.print("Range diode check failed"); break;
    default: Serial.print("Range undefined");
  }
}


void J1772Pilot::Init()
{
  pinMode(PILOT_PIN,OUTPUT);
  m_bit = digitalPinToBitMask(PILOT_PIN);
  m_port = digitalPinToPort(PILOT_PIN);

  SetState(PILOT_STATE_P12); // turns the pilot on 12V steady state
}


// no PWM pilot signal - steady state
// PILOT_STATE_P12 = steady +12V (EVSE_STATE_A - VEHICLE NOT CONNECTED)
// PILOT_STATE_N12 = steady -12V (EVSE_STATE_F - FAULT) 
void J1772Pilot::SetState(PILOT_STATE state)
{
  volatile uint8_t *out = portOutputRegister(m_port);

  uint8_t oldSREG = SREG;
  cli();
  /* prepare the timer, so it runs already in the background. When switching to PWM later, we get immediate the correct
   *  timing.
   */
  TCCR1A = _BV(COM1A0) | _BV(COM1B1) | _BV(WGM11) | _BV(WGM10);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11) | _BV(CS10);
  OCR1A = 249;
  OCR1B = 124; /* 50%, does not matter, it will be overwritten anyway when turning the PWM on. */
    
  //TCCR1A = 0; //disable pwm by turning off COM1A1,COM1A0,COM1B1,COM1B0
  TCCR1A = 3; //disable pwm by turning off COM1A1,COM1A0,COM1B1,COM1B0, but keep the WGM11 and WGM10 on, so that the timer still 
              // runs in the background.
  if (state == PILOT_STATE_P12) {
    *out |= m_bit;  // set pin high
  }
  else {
    *out &= ~m_bit;  // set pin low
  }
  SREG = oldSREG;
  isPwmOn = 0;
  m_State = state;
}


// set EVSE current capacity in Amperes
// duty cycle 
// outputting a 1KHz square wave to digital pin 10 via Timer 1
//
int J1772Pilot::SetPWM(int amps)
{
  uint8_t ocr1b = 0;
  isPwmOn=1;
  
  if ((amps >= 6) && (amps <= 51)) {
    ocr1b = 25 * amps / 6 - 1;  // J1772 states "Available current = (duty cycle %) X 0.6"
  } else if ((amps > 51) && (amps <= 80)) {
     ocr1b = amps + 159;  // J1772 states "Available current = (duty cycle % - 64) X 2.5"
  } else if (amps == -1) {
   /* requested 5% PWM to initiate digital communication */
    ocr1b = PWM_FIVE_PERCENT; /* 256/20 */
  } else {
    return 1; // error
  }

  if (ocr1b) {
    // Timer1 initialization:
    // 16MHz / 64 / (OCR1A+1) / 2 on digital 9
    // 16MHz / 64 / (OCR1A+1) on digital 10
    // 1KHz variable duty cycle on digital 10, 500Hz fixed 50% on digital 9
    // pin 10 duty cycle = (OCR1B+1)/(OCR1A+1)
    uint8_t oldSREG = SREG;
    cli();

    TCCR1A = _BV(COM1A0) | _BV(COM1B1) | _BV(WGM11) | _BV(WGM10);
    TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11) | _BV(CS10);
    OCR1A = 249;

    // 10% = 24 , 96% = 239
    OCR1B = ocr1b;
    TCNT1 =0; /* start the counter from the beginning, so we get a full first PWM cycle */

    SREG = oldSREG;

    m_State = PILOT_STATE_PWM;
    return 0;
  }
  else { // !duty
    // invalid amps
    return 1;
  }

}



void printPilotVoltages(void) {
  Serial.print(F("Pilot Voltages high="));
  Serial.print(  uPilotHigh_mV);
  Serial.print("  low=");
  Serial.println(  uPilotLow_mV);
}


void selftest(void) {
  uint8_t r;
  //digitalWrite(GREEN_LED_PIN,HIGH);
  //delay(1000);
  //digitalWrite(GREEN_LED_PIN,LOW);
  //digitalWrite(CHARGING_PIN,HIGH);
  //delay(300);
  //digitalWrite(CHARGING_PIN,LOW);
  //digitalWrite(RED_LED_PIN,HIGH);
  //delay(300);
  //digitalWrite(RED_LED_PIN,LOW);
  //Serial.println("Setting pilot to 12V");
  //m_Pilot.SetState(PILOT_STATE_P12);
  //delay(100);
  //readPilotVoltages();
  //printPilotVoltages();
  //delay(300);
  //Serial.println("Setting pilot to -12V");
  //m_Pilot.SetState(PILOT_STATE_N12);
  //delay(100);
  //readPilotVoltages();
  //printPilotVoltages();
  delay(1000);
  Serial.println("Setting pilot to PWM");
  m_Pilot.SetPWM(I_Lade_Soll_A);
  delay(100);
  readPilotVoltages();
  //printPilotVoltages();
  r=convertPilotVoltageToRange();
  //printPilotRange(r);

}


/*********************************************************************************************************/
/* Wallbox State Machine */

#define T_TRANSITION_DEBOUNCE (250 / MAIN_LOOP_CYCLE_TIME_MS) /* ca. 250 ms für normale Zustandsübergänge */
#define T_TRANSITION_DEBOUNCE_ERR_to_A (10000/MAIN_LOOP_CYCLE_TIME_MS) /* ca. 10 Sekunden zum Wechsel ERROR zu Standby */
//#define T_TRANSITION_DEBOUNCE_A_B (6000/MAIN_LOOP_CYCLE_TIME_MS) /* ca. 6 Sekunden vom Einstecken bis zum Aktivieren der PWM */
//#define T_TRANSITION_DEBOUNCE_A_B (1000/MAIN_LOOP_CYCLE_TIME_MS) /* ca. 1 Sekunde vom Einstecken bis zum Aktivieren der PWM */
#define T_TRANSITION_DEBOUNCE_A_B (200/MAIN_LOOP_CYCLE_TIME_MS) /* ca. 200 ms vom Einstecken bis zum Aktivieren der PWM */

#define WB_STATE_UNDEFINED 0 /* nicht initialisiert */
#define WB_STATE_A 1 /* Standby */
#define WB_STATE_B 2 /* vehicle detected */
#define WB_STATE_C 3 /* ready/charging */
#define WB_STATE_ERR 4 /* error */


uint8_t wallbox_state = WB_STATE_UNDEFINED;
uint16_t tTransitionDebounce_A_B;
uint16_t tTransitionDebounce_B_C;
uint16_t tTransitionDebounce_C_B;
uint16_t tTransitionDebounce_BC_A;
uint16_t tTransitionDebounce_A_ERR;
uint16_t tTransitionDebounce_BC_ERR;
uint16_t tTransitionDebounce_ERR_A;
uint8_t printModulo;

uint8_t checkTransition_A_B(void) {
  if (pilotVoltageRange==PILOT_RANGE_B) tTransitionDebounce_A_B++; else tTransitionDebounce_A_B=0;
  return (tTransitionDebounce_A_B >= T_TRANSITION_DEBOUNCE_A_B);
}

uint8_t checkTransition_B_C(void) {
  if (pilotVoltageRange==PILOT_RANGE_C) tTransitionDebounce_B_C++; else tTransitionDebounce_B_C=0;
  return (tTransitionDebounce_B_C >= T_TRANSITION_DEBOUNCE);
}

uint8_t checkTransition_BC_ERR(void) {
  if ((pilotVoltageRange==PILOT_RANGE_ERROR) || (pilotVoltageRange==PILOT_RANGE_ERROR_DIODE_CHECK)) tTransitionDebounce_BC_ERR++; else tTransitionDebounce_BC_ERR=0;
  return (tTransitionDebounce_BC_ERR >= T_TRANSITION_DEBOUNCE);
}

uint8_t checkTransition_C_B(void) {
  if (pilotVoltageRange==PILOT_RANGE_B) tTransitionDebounce_C_B++; else tTransitionDebounce_C_B=0;
  return (tTransitionDebounce_C_B >= T_TRANSITION_DEBOUNCE);
}

uint8_t checkTransition_BC_A(void) {
  if (pilotVoltageRange==PILOT_RANGE_A) tTransitionDebounce_BC_A++; else tTransitionDebounce_BC_A=0;
  return (tTransitionDebounce_BC_A >= T_TRANSITION_DEBOUNCE);
}

uint8_t checkTransition_A_ERR(void) {
  if  ((pilotVoltageRange==PILOT_RANGE_ERROR) 
    || (pilotVoltageRange==PILOT_RANGE_ERROR_DIODE_CHECK)
    || (pilotVoltageRange==PILOT_RANGE_C)) tTransitionDebounce_A_ERR++; else tTransitionDebounce_A_ERR=0;
  return (tTransitionDebounce_A_ERR >= T_TRANSITION_DEBOUNCE);
}

uint8_t checkTransition_ERR_A(void) {
  tTransitionDebounce_ERR_A++; /* Feste Zeit ohne Bedingung, dann einfach wieder in Normalzustand wechseln */
  return (tTransitionDebounce_ERR_A >= T_TRANSITION_DEBOUNCE_ERR_to_A);
}

void resetAllTimers(void) {
  tTransitionDebounce_A_B=0;
  tTransitionDebounce_B_C=0;
  tTransitionDebounce_C_B=0;
  tTransitionDebounce_BC_A=0;
  tTransitionDebounce_A_ERR=0;
  tTransitionDebounce_BC_ERR=0;
  tTransitionDebounce_ERR_A=0;
}


void enterState_A(void) {
  /* Standby */
  Serial.println("Entering State STANDBY");
  #ifdef GREEN_LED_PIN
    digitalWrite(GREEN_LED_PIN,HIGH); /* Grün */
  #endif  
  digitalWrite(CHARGING_PIN,LOW);  
  digitalWrite(RED_LED_PIN,LOW); 
  setLedStrip(0x002000); /* GRÜN, aber nicht zu hell */
  m_Pilot.SetState(PILOT_STATE_P12);  /* +12V */
  resetAllTimers();
  wallbox_state = WB_STATE_A;
}

void enterState_B(void) {
  /* vehicle detected */
  Serial.println("Entering State VEHICLE DETECTED");
  #ifdef GREEN_LED_PIN
    digitalWrite(GREEN_LED_PIN,HIGH); /* Grün */
  #endif  
  digitalWrite(CHARGING_PIN,LOW);  
  digitalWrite(RED_LED_PIN,HIGH); /* plus ROT  ergibt GELB */ 
  setLedStrip(0x404000); /* GELB */
  m_Pilot.SetPWM(I_Lade_Soll_A);  /* PWM */  
  resetAllTimers();
  wallbox_state = WB_STATE_B;
}

void enterState_C(void) {
  /* ready/charging */
  Serial.println("Entering State READY/CHARGING");
  #ifdef GREEN_LED_PIN
    digitalWrite(GREEN_LED_PIN,LOW);
  #endif
  digitalWrite(CHARGING_PIN,HIGH);   /* Relais-EIN und BLAU */ 
  digitalWrite(RED_LED_PIN,LOW); 
  setLedStrip(0x000040); /* BLAU */
  m_Pilot.SetPWM(I_Lade_Soll_A);  /* PWM */  
  resetAllTimers();
  wallbox_state = WB_STATE_C;
}

void enterState_ERR(void) {
  /* error */
  Serial.println("Entering State ERROR");
  #ifdef GREEN_LED_PIN
    digitalWrite(GREEN_LED_PIN,LOW);
  #endif  
  digitalWrite(CHARGING_PIN,LOW);
  digitalWrite(RED_LED_PIN,HIGH); 
  setLedStrip(0x400000); /* ROT */
  /* Wir könnten hier mit -12V dem Fahrzeug einen Fehler signalisieren. Aber um es nicht
   *  zu sehr zu verwirren, schalten wir mit konstant +12V die Ladung ab, damit sollte ein stabiler und sicherer
   *  Zustand erreicht sein.
   */
  m_Pilot.SetState(PILOT_STATE_P12);  /* +12V */
  resetAllTimers();
  wallbox_state = WB_STATE_ERR;
}


void runWbStateMachine(void) {
  readPilotVoltages();
  printModulo++;
  pilotVoltageRange = convertPilotVoltageToRange();
  if ((printModulo % 32)==0) {
     printPilotVoltages();
     printPilotRange(pilotVoltageRange);
     Serial.print("I_Ladee_Soll_A");
     Serial.println(I_Lade_Soll_A);
  }
  switch (wallbox_state) {
   case WB_STATE_A: /* standby */
     if (checkTransition_A_B()) enterState_B();
     if (checkTransition_A_ERR()) enterState_ERR();
     /* Wir erlauben hier explizit NICHT den direkten Übergang zu "C ready/charging", sonst könnte man mit bloßem Anlegen eines
      *  Widerstands die Spannung einschalten.
      */
     break;
   case WB_STATE_B: /* vehicle detected */
     if (checkTransition_B_C()) enterState_C();
     if (checkTransition_BC_ERR()) enterState_ERR();
     if (checkTransition_BC_A()) enterState_A();
     break;
   case WB_STATE_C: /* ready / charging */
     if (checkTransition_C_B()) enterState_B();
     if (checkTransition_BC_A()) enterState_A();
     if (checkTransition_BC_ERR()) enterState_ERR();
     break;
   case WB_STATE_ERR: /* Fehler */
     if (checkTransition_ERR_A()) enterState_A();   
     break;
   default:
     enterState_A(); /* Beim Init und falls der Zustand seltsame Werte hat */
  }
  delay(MAIN_LOOP_CYCLE_TIME_MS-12); /* 12 ms are needed for the ADC-multi-read-loop */
  
}


/*********************************************************************************************************/


void setup() {
  Serial.begin(115200);
  Serial.println(F("Starte..."));
  pinMode(CHARGING_PIN,OUTPUT);
  pinMode(DEBUG_PIN, OUTPUT);
  #ifdef GREEN_LED_PIN
    pinMode (GREEN_LED_PIN, OUTPUT);
    digitalWrite(GREEN_LED_PIN,LOW);
  #endif  
  pinMode (RED_LED_PIN, OUTPUT);
  digitalWrite(RED_LED_PIN,LOW);
  digitalWrite(CHARGING_PIN,LOW);
  FastLED.addLeds<WS2811, DATA_PIN, RGB>(leds, NUM_LEDS);
  m_Pilot.Init(); // init the pilot
  /*
  setLedStrip(0xFF0000);
  delay(1000);
  setLedStrip(0x00FF00);
  delay(1000);
  setLedStrip(0x0000FF);
  delay(1000);
  */
  setLedStrip(0x000000);
  
}

void readPoti(void) {
  uint16_t deltaI;

  #ifdef USE_POTI_FOR_CHARGE_CURRENT_SELECTION
  nAdcPoti = analogRead(POTI_PIN);
  //globalTest = nAdcPoti / 4 / 5;
  if (nAdcPoti<512) {
    /* left side -> digital communication, 5% PWM */
    I_Lade_Soll_A = -1;
  } else {  
    deltaI = nAdcPoti-512; /* right side of poti, scaled 0 to 511 */
    deltaI/=51; /* right side of poti, scaled 0 to 10 */
    I_Lade_Soll_A = 5 + deltaI; /* minimum value 5A, maximum 15A */
  }
  #else
    /* no poti for charge current selection, so we use fix value */
    I_Lade_Soll_A = 16;
  #endif  
  if (I_Lade_Soll_A != I_Lade_SollAlt_A) {
    I_Lade_SollAlt_A = I_Lade_Soll_A;
    if ((wallbox_state == WB_STATE_B) || (wallbox_state == WB_STATE_C)) {
      m_Pilot.SetPWM(I_Lade_Soll_A);  /* live update of PWM */
    }

  }
}

void loop() {
  //selftest();
  runWbStateMachine();
  readPoti();
}
