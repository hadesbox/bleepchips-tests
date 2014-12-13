////////////////////////////////////////////////////////////////////////////////////////
// VCO_MIDI
// MIDI control of AS10K through MCV Module
// By Bleeps and Chips (J.Cano & Y. Torroja)
// www.bleepsandchips.com
// Nov 2014
////////////////////////////////////////////////////////////////////////////////////////
// CC BY-SA 3.0 - https://creativecommons.org/licenses/by-sa/3.0/
// Feel free to use and modify this code in any way you want
// Would be great if you maintain our authorship somehow :)
////////////////////////////////////////////////////////////////////////////////////////
#include <Arduino.h>

// Stop VCO on note off: Comment following line if you do not want VCO to stop
// when MIDI note_off is received (thats the normal mode of analog synthesisers, VCO
// is always running)
#define STOP_VCO_ON_NOTE_OFF 1

// To simplify this development, this pins are fixed. Please, do not chage!
// GATE pin 
#define GATE_PIN 9   //-> To ENV Gate

#if defined(TCCR5B) && defined(TCCR5A) && defined(TIMSK5)  // For atmega2560/1280
  // CS0 pin 
  #define CS0_PIN  53  //-> To 4 MCV (CSO)
  // Measure pin
  #define FREQ_PIN 48  //
#else
  // CS0 pin 
  #define CS0_PIN  10  //-> To 4 MCV (CSO)
  // Measure pin
  #define FREQ_PIN 8   //
#endif

// Calibration pin
#define CAL_PIN 12   // High level on this pin starts calibration

//-----------------------------------------------------------------------------------------------------
// DAC modifications
//
// On Arduino UNO, Duecimila, Duemilanove...
// SCK -> PIN 13
// SDI -> PIN 11
// CS0 -> PIN 10
//
// On Arduino MEGA...
// SCK -> PIN 52
// SDI -> PIN 51
// CS0 -> PIN 53
//
//-----------------------------------------------------------------------------------------------------

#define MIDI_NOTE_ON  0b10010000
#define MIDI_NOTE_OFF 0b10000000

void setup() {  
  // Initialize serial port to get midi messages
  Serial.begin(57600);
  
  // setup CV using DAC
  CV_DAC_Setup(4096);

  // restore calibration
  loadCalibration();
  
  // setup calibration pin
  pinMode(CAL_PIN, INPUT);
  pinMode(GATE_PIN, OUTPUT);
}

void loop() {  
  
  int i=0;
  for(i=0;i<128;i++){
    CV_DAC_SetNote(i);
    CV_Set_Gate(1);
    delay(100);
    CV_Set_Gate(0);
  }

}



