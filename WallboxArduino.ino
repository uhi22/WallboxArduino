/*
 * Electric Vehicle Wallbox for Type 2 Charging
 * Based on "Open EVSE Firmware"
 * 
 * Features:
 *  - Control pilot signal with +12V/-12V or PWM
 *  - Pilot voltage min/max measurement
 *  - State machine including error state
 *  - Error state is exited after 10s back to "Standby"
 *  - Current control via potentiometer or via photo resistor (sunlight-dependent)
 * 
 *       
 * Todos:
 *    - Temperature measurement, shutdown or charge current reduction at high temperature
 *    - Output telemetry data in JSON format so that e.g. an ESP8266 can forward it via MQTT
 *    - Display error codes with the red LEDs (e.g. 1 LED = diode check, 2 LEDs = short circuit, 3 LEDs = over-temperature, ...)
 * 
 */

/*******************************************************************************/
/* WS2812 LED strip */
#include <FastLED.h>
// How many leds are in the strip?
#define NUM_LEDS 6
// Data pin that led data will be written out over
#define DATA_PIN 11
// This is an array of leds.  One item for each led in your strip.
CRGB leds[NUM_LEDS];

uint16_t nLoop;

void setLedStrip(uint32_t x) {
 uint8_t i;
 for(i = 0; i < NUM_LEDS; i++) {
      leds[i] = x;
 }
 // Show the leds
 FastLED.show();
}

/***********************************************************************************/
/* Global variables and definitions */
 
#define USE_CURRENT_CONTROL_VIA_PHOTO_RESISTOR
 
#define VOLT_PIN 1          // ControlPilot analog voltage reading pin A1
#define POTI_PIN 2          /* Potentiometer at A2 */
#define RED_LED_PIN 5       // Digital pin
#define CHARGING_PIN 8      // Digital charging LED and relay trigger pin
#define PILOT_PIN 10        // n.b. PILOT_PIN *MUST* be digital 10 because SetPWM() assumes it
//#define GREEN_LED_PIN 13  // Digital pin
#define DEBUG_PIN 6         /* For debugging */

#ifdef USE_CURRENT_CONTROL_VIA_PHOTO_RESISTOR
#define PHOTO_PIN 0                /* Photo resistor on A0 */
#define PHOTO_THRESHOLD_LOW  150   /* ADC counts below this = too dark, no charging */
#define PHOTO_THRESHOLD_HIGH 800   /* ADC counts above this = full sun -> max current */
#define SOLAR_I_MIN_A 6            /* Minimum charge current when just enough light */
#define SOLAR_I_MAX_A 16           /* Maximum charge current at full sun */
#endif

int8_t I_Lade_Soll_A = 0;
int8_t I_Lade_SollAlt_A = -2;

uint16_t nAdcPoti;

#define PWM_FIVE_PERCENT (256/20); /* 5% of full range. */
#define MAIN_LOOP_CYCLE_TIME_MS 20 /* 20 milliseconds main loop cycle time */

/**********************************************************************************/
/* Pilot signal generation and measurement */

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
    reading -= 561; /* corresponds to 0V on the ControlPilot */
    reading *= 34;  /* scale to millivolts */
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
    /* During PWM we should see -12V; if not, something is wrong */
    rc = PILOT_RANGE_ERROR_DIODE_CHECK;
  } else if ((uPilotHigh_mV>=10000) and (uPilotHigh_mV<=13500)) {
    rc= PILOT_RANGE_A; /* 12V, not connected */
  } else if ((uPilotHigh_mV>=7000) and (uPilotHigh_mV<=10500)) {
    rc= PILOT_RANGE_B; /* 9V, vehicle detected */
  } else if ((uPilotHigh_mV>=4000) and (uPilotHigh_mV<=7500)) {
    rc= PILOT_RANGE_C; /* 6V, ready, charging */
  } else {
    rc= PILOT_RANGE_ERROR; /* Fault */
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


// No PWM pilot signal - steady state
// PILOT_STATE_P12 = steady +12V (EVSE_STATE_A - VEHICLE NOT CONNECTED)
// PILOT_STATE_N12 = steady -12V (EVSE_STATE_F - FAULT) 
void J1772Pilot::SetState(PILOT_STATE state)
{
  volatile uint8_t *out = portOutputRegister(m_port);

  uint8_t oldSREG = SREG;
  cli();
  /* Prepare the timer so it runs already in the background. When switching to PWM later, we get immediate the correct
   * timing.
   */
  TCCR1A = _BV(COM1A0) | _BV(COM1B1) | _BV(WGM11) | _BV(WGM10);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11) | _BV(CS10);
  OCR1A = 249;
  OCR1B = 124; /* 50%, does not matter, it will be overwritten anyway when turning the PWM on. */
    
  TCCR1A = 3; // Disable PWM by turning off COM1A1,COM1A0,COM1B1,COM1B0, but keep WGM11 and WGM10 on
              // so that the timer still runs in the background.
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


// Set EVSE current capacity in Amperes
// Duty cycle
// Outputting a 1KHz square wave to digital pin 10 via Timer 1
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
   /* Requested 5% PWM to initiate digital communication */
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
    TCNT1 = 0; /* Start the counter from the beginning so we get a full first PWM cycle */

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
  delay(1000);
  Serial.println("Setting pilot to PWM");
  m_Pilot.SetPWM(I_Lade_Soll_A);
  delay(100);
  readPilotVoltages();
  r=convertPilotVoltageToRange();
}


/*********************************************************************************************************/
/* Wallbox State Machine */

#define T_TRANSITION_DEBOUNCE (250 / MAIN_LOOP_CYCLE_TIME_MS)             /* approx. 250 ms for normal state transitions */
#define T_TRANSITION_DEBOUNCE_ERR_to_A (10000/MAIN_LOOP_CYCLE_TIME_MS)   /* approx. 10 seconds to transition from ERROR to Standby */
#define T_TRANSITION_DEBOUNCE_A_B (200/MAIN_LOOP_CYCLE_TIME_MS)          /* approx. 200 ms from plug-in to activating PWM */

#define WB_STATE_UNDEFINED 0 /* not initialised */
#define WB_STATE_A 1         /* Standby */
#define WB_STATE_B 2         /* vehicle detected */
#define WB_STATE_C 3         /* ready/charging */
#define WB_STATE_ERR 4       /* error */


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
  tTransitionDebounce_ERR_A++; /* Fixed time with no condition, then simply transition back to normal state */
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
    digitalWrite(GREEN_LED_PIN,HIGH); /* Green */
  #endif  
  digitalWrite(CHARGING_PIN,LOW);  
  digitalWrite(RED_LED_PIN,LOW); 
  setLedStrip(0x002000); /* GREEN, but not too bright */
  m_Pilot.SetState(PILOT_STATE_P12);  /* +12V */
  resetAllTimers();
  wallbox_state = WB_STATE_A;
}

void enterState_B(void) {
  /* Vehicle detected */
  if (I_Lade_Soll_A == 0) {
      Serial.println("Would enter state B, but target current is zero.");
      return;
  }
  Serial.println("Entering State VEHICLE DETECTED");
  #ifdef GREEN_LED_PIN
    digitalWrite(GREEN_LED_PIN,HIGH); /* Green */
  #endif  
  digitalWrite(CHARGING_PIN,LOW);  
  digitalWrite(RED_LED_PIN,HIGH); /* Red + Green = Yellow */
  setLedStrip(0x404000); /* YELLOW */
  m_Pilot.SetPWM(I_Lade_Soll_A);  /* PWM */  
  resetAllTimers();
  wallbox_state = WB_STATE_B;
}

void enterState_C(void) {
  /* Ready/charging */
  if (I_Lade_Soll_A == 0) {
      Serial.println("Would enter state C, but target current is zero.");
      return;
  }
  Serial.println("Entering State READY/CHARGING");
  #ifdef GREEN_LED_PIN
    digitalWrite(GREEN_LED_PIN,LOW);
  #endif
  digitalWrite(CHARGING_PIN,HIGH);  /* Relay ON and BLUE */
  digitalWrite(RED_LED_PIN,LOW); 
  setLedStrip(0x000040); /* BLUE */
  m_Pilot.SetPWM(I_Lade_Soll_A);  /* PWM */  
  resetAllTimers();
  wallbox_state = WB_STATE_C;
}

void enterState_ERR(void) {
  /* Error */
  Serial.println("Entering State ERROR");
  #ifdef GREEN_LED_PIN
    digitalWrite(GREEN_LED_PIN,LOW);
  #endif  
  digitalWrite(CHARGING_PIN,LOW);
  digitalWrite(RED_LED_PIN,HIGH); 
  setLedStrip(0x400000); /* RED */
  /* We could signal a fault to the vehicle here using -12V, but to avoid confusing it
   * we disable charging with a steady +12V, which should result in a stable and safe state.
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
     Serial.print("I_Lade_Soll_A=");  /* fixed typo: was "I_Ladee_Soll_A" */
     Serial.println(I_Lade_Soll_A);
  }
  switch (wallbox_state) {
   case WB_STATE_A: /* standby */
     if (checkTransition_A_B()) enterState_B();
     if (checkTransition_A_ERR()) enterState_ERR();
     /* We explicitly do NOT allow a direct transition to "C ready/charging" here,
      * otherwise simply applying a resistor could switch on the voltage.
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
   case WB_STATE_ERR: /* error */
     if (checkTransition_ERR_A()) enterState_A();   
     break;
   default:
     enterState_A(); /* On init and if the state has unexpected values */
  }
  delay(MAIN_LOOP_CYCLE_TIME_MS-12); /* 12 ms are needed for the ADC multi-read loop */
  
}


/*********************************************************************************************************/


void setup() {
  Serial.begin(115200);
  Serial.println(F("Starting..."));
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
  setLedStrip(0x000000);
}

void readSolar(void) {
  uint16_t nAdcPhoto;
  int8_t newCurrent;

  nAdcPhoto = analogRead(PHOTO_PIN);

  if (nAdcPhoto < PHOTO_THRESHOLD_LOW) {
    /* Too dark — no solar power available, disable charging */
    newCurrent = 0;
  } else if (nAdcPhoto >= PHOTO_THRESHOLD_HIGH) {
    /* Full sun — charge at maximum current */
    newCurrent = SOLAR_I_MAX_A;
  } else {
    /* Linear interpolation between min and max current */
    uint16_t range    = PHOTO_THRESHOLD_HIGH - PHOTO_THRESHOLD_LOW;
    uint16_t aboveLow = nAdcPhoto - PHOTO_THRESHOLD_LOW;
    newCurrent = SOLAR_I_MIN_A
               + (int8_t)((uint16_t)(SOLAR_I_MAX_A - SOLAR_I_MIN_A) * aboveLow / range);
  }

  /* Only act on changes */
  if (newCurrent != I_Lade_Soll_A) {
    I_Lade_Soll_A = newCurrent;
    Serial.print(F("Solar ADC="));
    Serial.print(nAdcPhoto);
    Serial.print(F("  I_Lade_Soll_A="));
    Serial.println(I_Lade_Soll_A);

    if (newCurrent == 0) {
      /* Sun gone — if we were charging, gracefully back to standby */
      if ((wallbox_state == WB_STATE_B) || (wallbox_state == WB_STATE_C)) {
        enterState_A();
      }
    } else {
      /* Sun present — update PWM live if already in B or C */
      if ((wallbox_state == WB_STATE_B) || (wallbox_state == WB_STATE_C)) {
        m_Pilot.SetPWM(I_Lade_Soll_A);
      }
    }
  }
}

void readPoti(void) {
  /* Note: this function is only active when USE_CURRENT_CONTROL_VIA_PHOTO_RESISTOR is not defined.
   * It is kept here for reference and future use.
   */
  uint16_t deltaI;

  #ifdef USE_POTI_FOR_CHARGE_CURRENT_SELECTION
  nAdcPoti = analogRead(POTI_PIN);
  if (nAdcPoti<512) {
    /* Left side -> digital communication, 5% PWM */
    I_Lade_Soll_A = -1;
  } else {  
    deltaI = nAdcPoti-512; /* Right side of potentiometer, scaled 0 to 511 */
    deltaI/=51;            /* Right side of potentiometer, scaled 0 to 10 */
    I_Lade_Soll_A = 5 + deltaI; /* Minimum value 5A, maximum 15A */
  }
  #else
    /* No potentiometer for charge current selection, use fixed value */
    I_Lade_Soll_A = 16;
  #endif  
  if (I_Lade_Soll_A != I_Lade_SollAlt_A) {
    I_Lade_SollAlt_A = I_Lade_Soll_A;
    if ((wallbox_state == WB_STATE_B) || (wallbox_state == WB_STATE_C)) {
      m_Pilot.SetPWM(I_Lade_Soll_A);  /* Live update of PWM */
    }
  }
}

void loop() {
  runWbStateMachine();
  #ifdef USE_CURRENT_CONTROL_VIA_PHOTO_RESISTOR
    static uint16_t nLoop = 0;
    nLoop++;
    if (nLoop > 100) { /* 100 x 20ms = 2s interval for printing the photo resistor ADC for calibration purposes */
        Serial.println(analogRead(PHOTO_PIN));
        nLoop = 0;
    }
    readSolar();
  #else
    readPoti();
  #endif
}
