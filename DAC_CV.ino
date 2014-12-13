////////////////////////////////////////////////////////////////////////////////////////
// VCO_MIDI
// MIDI control of AS10K through MCV Module
// By Bleeps and Chips  (J.Cano & Y. Torroja)
// www.bleepsandchips.com
// Nov 2014
////////////////////////////////////////////////////////////////////////////////////////
// CC BY-SA 3.0 - https://creativecommons.org/licenses/by-sa/3.0/
// Feel free to use and modify this code in any way you want
// Would be great if you maintain our authorship somehow :)
////////////////////////////////////////////////////////////////////////////////////////
#include <Arduino.h>

// Using DAC MCP2822

#include <EEPROM.h>
#include <avr/pgmspace.h>
#include <SPI.h>

////////////////////////////////////////////////////////////////
// MIDI notes frequencies (rounded to nearest integer)
int MIDI_frequencies[128] PROGMEM = {
  8,    9,    9,   10,   10,   11,   12,   12,   13,   14,   15,   15,   16,   17,   18,   19, 
  21,   22,   23,   24,   26,   28,   29,   31,   33,   35,   37,   39,   41,   44,   46,   49,   
  52,   55,   58,   62,   65,   69,   73,   78,   82,   87,   92,   98,  104,  110,  117,  123,  
  131,  139,  147,  156,  165,  175,  185,  196,  208,  220,  233,  247,  262,  277,  294,  311,  
  330,  349,  370,  392,  415,  440,  466,  494,  523,  554,  587,  622,  659,  698,  740,  784,  
  831,  880,  932,  988, 1047, 1109, 1175, 1245, 1319, 1397, 1480, 1568, 1661, 1760, 1865, 1976, 
  2093, 2217, 2349, 2489, 2637, 2794, 2960, 3136, 3322, 3520, 3729, 3951, 4186, 4435, 4699, 4978, 
  5274, 5588, 5920, 5920, 6645, 7040, 7459, 7902, 8372, 8870, 9397, 9956, 10548, 11175, 11840, 12544};

////////////////////////////////////////////////////////////////
// Global variables
int cvSteps;
int dac_values[128];
int measuredCycles;
float measuredFreq;
volatile boolean firstEdge;
volatile boolean finished;
volatile unsigned int ovfCounter;
volatile long int inTime, lastInTime;
volatile unsigned long totalTime;

////////////////////////////////////////////////////////////////
// Overflows to wait for a signal change (100 ms)
#define TIMEOUT_OVF 400

////////////////////////////////////////////////////////////////
// Number of repetitions to measure wave period/frequency
#define MEASURE_CYCLES 8

////////////////////////////////////////////
// Save/load calibration data
void loadCalibration() {
  Serial.print(F("Reading calibration from EEPROM ("));
  Serial.print(sizeof(dac_values));
  Serial.print(F(" bytes)..."));
  for(int i = 0; i < 127; i++) {
    dac_values[i]  =  EEPROM.read((i << 1));
    dac_values[i] |= (EEPROM.read((i << 1) + 1) << 8);
  }
  Serial.println("OK");
}

void saveCalibration() {
  Serial.print(F("Writing calibration to EEPROM ("));
  Serial.print(sizeof(dac_values));
  Serial.print(F(" bytes)..."));
  for(int i = 0; i < 127; i++) {
    EEPROM.write((i << 1),   (byte)(dac_values[i] & 0xFF));
    EEPROM.write((i << 1) + 1, (byte)((dac_values[i] >> 8) & 0xFF));
  }
  Serial.println("OK");
}

/////////////////////////////////////////////
// Initialization
void CV_DAC_Setup(int steps) {
  
  #if defined(TCCR5B) && defined(TCCR5A) && defined(TIMSK5)  // For atmega2560/1280
    TCCR5A  = (1 << COM5B1) | (0 << COM5B0) | (1 << WGM51) | (1 << WGM50);
    TCCR5B  = (1 << WGM53) | (1 << WGM52) | (0 << CS52) | (0 << CS51)| (1 << CS50);
    OCR5A   = steps - 1;
    OCR5B   = 0;
  #else
    TCCR1A  = (1 << COM1B1) | (0 << COM1B0) | (1 << WGM11) | (1 << WGM10);
    TCCR1B  = (1 << WGM13) | (1 << WGM12) | (0 << CS12) | (0 << CS11)| (1 << CS10);
    OCR1A   = steps - 1;
    OCR1B   = 0;
  #endif

  cvSteps = steps;

  pinMode( CS0_PIN, OUTPUT );
  SPI.begin(); 
  SPI.setClockDivider(SPI_CLOCK_DIV2);
}

/////////////////////////////////////////////
// Set DAC to val
// assuming single channel, gain=2
void CV_DAC_Set(unsigned int val)
{
  byte lowByte = val & 0xff;
  byte highByte = ((val >> 8) & 0xff) | 0x10;

  digitalWrite(CS0_PIN, LOW);
  SPI.transfer(highByte);
  SPI.transfer(lowByte);
  digitalWrite(CS0_PIN, HIGH);
}

/////////////////////////////////////////////
// Set DAC to note
void CV_DAC_SetNote(int n) {
  Serial.print(n);
  Serial.print(" -> ");
  Serial.println(dac_values[n]);
  CV_DAC_Set(dac_values[n]);
}

/////////////////////////////////////////////
// Set Gate (0 or not 0)
void CV_Set_Gate(int val) {
  if (val != 0) {
    digitalWrite(GATE_PIN, HIGH);
  } 
  else {
    digitalWrite(GATE_PIN, LOW);
  }
}

////////////////////////////////////////////
// Interrupt routines to measure frequency
#if defined(TCCR5B) && defined(TCCR5A) && defined(TIMSK5)
  ISR( TIMER5_OVF_vect ) {
#else
  ISR( TIMER1_OVF_vect ) {
#endif
  ovfCounter++;
}

#if defined(TCCR5B) && defined(TCCR5A) && defined(TIMSK5)
  ISR( TIMER5_CAPT_vect ) {
#else
  ISR( TIMER1_CAPT_vect ) {
#endif
  if ( firstEdge ) {
    lastInTime = ICR1;
    ovfCounter = 0;
    firstEdge  = false;
    totalTime  = 0;
    return;
  } 
  inTime       = ICR1;
  totalTime   += (unsigned long)ovfCounter * cvSteps + (inTime - lastInTime);
  measuredCycles++;
  if ( measuredCycles == MEASURE_CYCLES ) {
    measuredFreq = F_CPU / ( (float)totalTime / MEASURE_CYCLES );
    finished = true;
    // Disable measuring associated interrupts 
    #if defined(TCCR5B) && defined(TCCR5A) && defined(TIMSK5)
      TIMSK5 &= ~((1 << ICIE5) | (1 << TOIE5));
    #else
      TIMSK1 &= ~((1 << ICIE1) | (1 << TOIE1));
    #endif
  }
  lastInTime   = inTime;
  ovfCounter   = 0;  
}


////////////////////////////////////////////
// Calibration support procedures
// STEP BY STEP CALIBRATION
// Starts measuring wave frequency
void startFreqMeasure() {
  cli();
  #if defined(TCCR5B) && defined(TCCR5A) && defined(TIMSK5)
    // Interrupts by overflow and input capture
    TIMSK5 = (1 << ICIE5) | (1 << TOIE5);
    // Clear prevoius interrupts flags
    TIFR5  = (1 << ICF5) | (1 << TOV5);
  #else
    // Interrupts by overflow and input capture
    TIMSK1 = (1 << ICIE1) | (1 << TOIE1);
    // Clear prevoius interrupts flags
    TIFR1  = (1 << ICF1) | (1 << TOV1);
  #endif
  firstEdge      = true;
  finished       = false;
  ovfCounter     = 0;
  totalTime      = 0;
  measuredCycles = 0;
  sei();
}

////////////////////////////////////////////
// Checks if it is measuring 
boolean isMeasuring() {
  if (ovfCounter > TIMEOUT_OVF) return false; // finished due to timeout  
  return !finished;
}

////////////////////////////////////////////
// Gets the result in Hz
float getFreqMeasure() {
  if ( !finished ) {
    return 0.0;
  } 
  else {
    return measuredFreq;
  }  
}

////////////////////////////////////////////
// Performs calibration for MIDI range 24-107
void do_DAC_Calibration(int initial_val) {
  int dac_val = initial_val;

  // Initialize values
  for(int i = 0; i < 127; i++) dac_values[i] = 0;

  int   numSaturations = 0;
  float lastMeasuredFreq = 0.0;

  // Calibrating notes from C1 to B7
  for(int i = 24; i < 119; i++) {

    int freq = pgm_read_word(MIDI_frequencies + i);   // Frecuancia de la nota MIDI a calibrar

    boolean next_note = false;
    float lastError = 1.0;
    float lastFreq  = 0.0;
    int   lastDac   = dac_val;

    while (!next_note) {

      // Serial.print(F(" Trying DAC value: "));
      // Serial.print( dac_val ); 

      // Set CV DAC value
      CV_DAC_Set(dac_val);   

      // Start measuring frequency
      startFreqMeasure();
      while(isMeasuring());

      if (getFreqMeasure() != 0) {
        Serial.print(dac_val);
        Serial.print(F(" --> measured Hz: "));
        Serial.println( getFreqMeasure() );
        float f = getFreqMeasure();
        float e = abs(f - freq) / freq;
        if (e < lastError) {
          lastError = e;
          lastFreq  = f;
          lastDac   = dac_val;
        }
        if (getFreqMeasure() >= freq) {
          Serial.print(F("MIDI note "));
          Serial.print( i );
          Serial.print(F(" - target freq "));
          Serial.print( freq );
          Serial.print(F(" calibrated at ")); 
          float f = lastFreq; // getFreqMeasure();
          Serial.print( f );
          Serial.print(F(" - error (%): ")); 
          Serial.println( (freq - f)/freq * 100);    
          dac_values[i] = lastDac; // dac_val;
          next_note = true;
          CV_Set_Gate(1);
          delay(1);
          CV_Set_Gate(0);

        } 
        else {
          if ( dac_val > cvSteps) {
            Serial.println(F("Calibration finished by DAC limit!!"));
            i = 127;
            break;
          } 
          else {
            numSaturations = 0;
          }  
        }
        lastMeasuredFreq = getFreqMeasure();  
      } 
      else {
        Serial.print(dac_val);
        Serial.println(F(" --> no measure!"));
      }
      // Next step
      dac_val++;
    }
  }

  // Stop CV
  CV_DAC_Set(0);

}




