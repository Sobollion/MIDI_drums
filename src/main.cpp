/********************
Arduino MIDI Drum by Sobollion 06.02.17
*/
#include "EEPROMAnything.h"
#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <menu.h>//menu macros and objects
#include <menuLCDs.h>//F. Malpartida LCD's
#include <menuFields.h>
#include "ClickEncoderStream.h"
#include <TimerOne.h>
#include <EEPROM.h>
#include <avr/wdt.h>


//Piezo defines
#define NUM_PIEZOS 2

#define state_pin 6
bool state;


struct THR {   // СЃС‚СЂСѓРєС‚СѓСЂР° РґР°РЅРЅС‹С…
  int SNARE_THRESHOLD;
  int LTOM_THRESHOLD;
  int RTOM_THRESHOLD;
  int LCYM_THRESHOLD;
  int RCYM_THRESHOLD;
  int KICK_THRESHOLD;
} THR;

#define START_SLOT 0     //first analog slot of piezos

//MIDI note defines for each trigger
struct NOTE {
int KICK_NOTE;
int SNARE_NOTE;
int LTOM_NOTE;
int RTOM_NOTE;
int LCYM_NOTE;
int RCYM_NOTE;
} NOTE;

//MIDI defines
#define NOTE_ON_CMD 0x90
#define NOTE_OFF_CMD 0x80
#define MAX_MIDI_VELOCITY 127
int MIDI_CHANNEL;

//MIDI baud rate
#define SERIAL_RATE 31250

//Program defines
//ALL TIME MEASURED IN MILLISECONDS
#define SIGNAL_BUFFER_SIZE 100
#define PEAK_BUFFER_SIZE 30
#define MAX_TIME_BETWEEN_PEAKS 20
#define MIN_TIME_BETWEEN_NOTES 50

//map that holds the mux slots of the piezos
//unsigned short slotMap[NUM_PIEZOS]; //= {0,1,2,3,6,7};
unsigned short slotMap[NUM_PIEZOS];

//map that holds the respective note to each piezo
unsigned short noteMap[NUM_PIEZOS];

//map that holds the respective threshold to each piezo
unsigned short thresholdMap[NUM_PIEZOS];

//Ring buffers to store analog signal and peaks
short currentSignalIndex[NUM_PIEZOS];
short currentPeakIndex[NUM_PIEZOS];
unsigned short signalBuffer[NUM_PIEZOS][SIGNAL_BUFFER_SIZE];
unsigned short peakBuffer[NUM_PIEZOS][PEAK_BUFFER_SIZE];

boolean noteReady[NUM_PIEZOS];
unsigned short noteReadyVelocity[NUM_PIEZOS];
boolean isLastPeakZeroed[NUM_PIEZOS];

unsigned long lastPeakTime[NUM_PIEZOS];
unsigned long lastNoteTime[NUM_PIEZOS];

/////////////////////////////////////////
//LCD I2C
//sda analog 4
//scl analog 5
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
menuLCD menu_lcd(lcd,16,2);

/////////////////////////////////////////
//https://omerk.github.io/lcdchargen/
//ввод кастом чаров

byte HHopen[8] = {
  0b00100,
  0b11111,
  0b00100,
  0b11111,
  0b00100,
  0b00100,
  0b00100,
  0b00100
};
byte HHclosed[8] = {
  0b00000,
  0b00100,
  0b11111,
  0b11111,
  0b00100,
  0b00100,
  0b00100,
  0b00100
};
byte Crash[8] = {
  0b00010,
  0b00100,
  0b01110,
  0b10001,
  0b00001,
  0b00001,
  0b00001,
  0b00001
};
byte Ride[8] = {
  0b00001,
  0b00010,
  0b00100,
  0b01010,
  0b10001,
  0b00001,
  0b00001,
  0b00001
};
byte Kick[8] = {
  0b00000,
  0b01110,
  0b11111,
  0b10001,
  0b10101,
  0b10101,
  0b01110,
  0b00110
};
byte Snare[8] = {
  0b00000,
  0b11111,
  0b10001,
  0b11111,
  0b00100,
  0b00100,
  0b01110,
  0b10101
};
byte Tom[8] = {
  0b00000,
  0b01110,
  0b10001,
  0b11111,
  0b11111,
  0b01110,
  0b00000,
  0b00000
};


void getEEPROM ()
  {                  // чтение данных
    if (EEPROM.read(0) != 120)
    {  // данных нет, записываем по умолчанию
      THR.SNARE_THRESHOLD = 30;
      THR.LTOM_THRESHOLD = 30;
      THR.RTOM_THRESHOLD = 30;
      THR.LCYM_THRESHOLD = 100;
      THR.RCYM_THRESHOLD = 100;
      THR.KICK_THRESHOLD = 50;
      NOTE.SNARE_NOTE = 70;
      NOTE.LTOM_NOTE = 71;
      NOTE.RTOM_NOTE = 72;
      NOTE.LCYM_NOTE = 73;
      NOTE.RCYM_NOTE = 74;
      NOTE.KICK_NOTE = 75;
      EEPROM_writeAnything(1, THR);
      EEPROM_writeAnything(13, NOTE);
      EEPROM.update(0, 120); // отметили наличие данных
    }
    EEPROM.get(1, THR);
    EEPROM.get(13, NOTE);
  }

void saveEEPROMTHR () { // запись данных
EEPROM_writeAnything(1, THR);
}

void saveEEPROMNOTE () { // запись данных
EEPROM_writeAnything(13, NOTE);
}

////////////////////////////////////////////ENCODER
// ENCODER (aka rotary switch) PINS
// rotary
#define CK_ENC  2 // Quand encoder on an ISR capable input
#define DT_ENC  4
#define SW_ENC  5

ClickEncoder qEnc(DT_ENC,CK_ENC,SW_ENC , 2, LOW);
ClickEncoderStream enc(qEnc,1);
void timerIsr() {qEnc.service();}

///////////////////////////////////////////////////////////////////////////
//functions to wire as menu actions

//aux function

/////////////////////////////////////////////////////////////////////////
// MENU DEFINITION
// here we define the menu structure and wire actions functions to it
// empty options are just for scroll testing

 MENU(subMenu,"MIDI-Notes",
  FIELD(NOTE.KICK_NOTE,"Kick"," ",0x0,0x7F,0x1,0),
  FIELD(NOTE.RTOM_NOTE,"Rtom"," ",0x0,0x7F,0x1,0),
  FIELD(NOTE.RCYM_NOTE,"Rcym"," ",0x0,0x7F,0x1,0),
  FIELD(NOTE.LCYM_NOTE,"Lcym"," ",0x0,0x7F,0x1,0),
  FIELD(NOTE.SNARE_NOTE,"Snare"," ",0x0,0x7F,0x1,0),
  FIELD(NOTE.LTOM_NOTE,"Ltom"," ",0x0,0x7F,0x1,0),
  OP("SAVE TO EEPROM",saveEEPROMNOTE)
);

 MENU(subMenu2,"Thresholds",
  FIELD(THR.KICK_THRESHOLD,"Kick"," ",0,100,1,0),
  FIELD(THR.RTOM_THRESHOLD,"Rtom"," ",0,100,1,0),
  FIELD(THR.RCYM_THRESHOLD,"Rcym"," ",0,100,1,0),
  FIELD(THR.LCYM_THRESHOLD,"Lcym"," ",0,100,1,0),
  FIELD(THR.SNARE_THRESHOLD,"Snare"," ",0,100,1,0),
  FIELD(THR.LTOM_THRESHOLD,"Ltom"," ",0,100,1,0),
  OP("SAVE TO EEPROM",saveEEPROMTHR)
);

MENU(mainMenu,"MIDI_DRUM",
  SUBMENU(subMenu),
  SUBMENU(subMenu2),
  FIELD(MIDI_CHANNEL,"MIDI-channel"," ",0x0,0xF,0x1,0)
 );




/////////////////////////////////////////////////////////////////////////
void setup() {
  lcd.begin(16,2);

///////////////////////////////////////
// CUSTOM CHARS
  lcd.createChar(0, Kick);
  lcd.createChar(1, Snare);
  lcd.createChar(2, Ride);
  lcd.createChar(3, Crash);
  lcd.createChar(4, HHopen);
  lcd.createChar(5, HHclosed);
  lcd.createChar(6, Tom);



/////////////////////////////////////
// preview

lcd.setCursor(0, 0);
lcd.print("  MIDI DRUMS");
lcd.setCursor(4, 1);
lcd.print(char(0));
lcd.setCursor(5, 1);
lcd.write((uint8_t)1);
lcd.setCursor(6, 1);
lcd.write((uint8_t)2);
lcd.setCursor(7, 1);
lcd.write((uint8_t)3);
lcd.setCursor(8, 1);
lcd.write((uint8_t)4);
lcd.setCursor(9, 1);
lcd.write((uint8_t)5);
lcd.setCursor(10, 1);
lcd.write((uint8_t)6);

////////////ЧИТАЕМ ИЗ ЕЕПРОМА
getEEPROM();

//lcd.print("ver. 1.0");
delay(2000);
lcd.clear();
lcd.setCursor(2, 0);
lcd.print("Waiting for");
lcd.setCursor(4, 1);
lcd.print("command");
  delay(1000);
  lcd.clear();
  qEnc.setAccelerationEnabled(true);
  qEnc.setDoubleClickEnabled(true);
  Timer1.initialize(1000);
  Timer1.attachInterrupt(timerIsr);

  Serial.begin(SERIAL_RATE);

  //initialize globals
  for(short i=0; i<NUM_PIEZOS; ++i)
  {
    /////////////////ЗДЕСЬ НУЖНО ИСКЛЮЧИТЬ ИСПОЛЬЗОВАНИЕ 4 и 5 аналоговых портов
    currentSignalIndex[i] = 0;
    currentPeakIndex[i] = 0;
    memset(signalBuffer[i],0,sizeof(signalBuffer[i]));
    memset(peakBuffer[i],0,sizeof(peakBuffer[i]));
    noteReady[i] = false;
    noteReadyVelocity[i] = 0;
    isLastPeakZeroed[i] = true;
    lastPeakTime[i] = 0;
    lastNoteTime[i] = 0;
    slotMap[i] = START_SLOT + i;
  }

  thresholdMap[0] = THR.KICK_THRESHOLD;
  thresholdMap[1] = THR.RTOM_THRESHOLD;
  thresholdMap[2] = THR.RCYM_THRESHOLD;
  thresholdMap[3] = THR.LCYM_THRESHOLD;
  thresholdMap[4] = THR.SNARE_THRESHOLD;
  thresholdMap[5] = THR.LTOM_THRESHOLD;

  noteMap[0] = NOTE.KICK_NOTE;
  noteMap[1] = NOTE.RTOM_NOTE;
  noteMap[2] = NOTE.RCYM_NOTE;
  noteMap[3] = NOTE.LCYM_NOTE;
  noteMap[4] = NOTE.SNARE_NOTE;
  noteMap[5] = NOTE.LTOM_NOTE;
  }

void midiNoteOn(byte note, byte midiVelocity)
  {
  Serial.write(NOTE_ON_CMD + MIDI_CHANNEL);
  Serial.write(note);
  Serial.write(midiVelocity);
  }

void midiNoteOff(byte note, byte midiVelocity)
  {
  Serial.write(NOTE_OFF_CMD + MIDI_CHANNEL);
  Serial.write(note);
  Serial.write(midiVelocity);
  }

void noteFire(unsigned short note, unsigned short velocity)
  {
  if(velocity > MAX_MIDI_VELOCITY)
    velocity = MAX_MIDI_VELOCITY;

  midiNoteOn(note, velocity);
  midiNoteOff(note, velocity);
  }

void recordNewPeak(short slot, short newPeak)
  {
  isLastPeakZeroed[slot] = (newPeak == 0);

  unsigned long currentTime = millis();
  lastPeakTime[slot] = currentTime;

  //new peak recorded (newPeak)
  peakBuffer[slot][currentPeakIndex[slot]] = newPeak;

  //1 of 3 cases can happen:
  // 1) note ready - if new peak >= previous peak
  // 2) note fire - if new peak < previous peak and previous peak was a note ready
  // 3) no note - if new peak < previous peak and previous peak was NOT note ready

  //get previous peak
  short prevPeakIndex = currentPeakIndex[slot]-1;
  if(prevPeakIndex < 0) prevPeakIndex = PEAK_BUFFER_SIZE-1;
  unsigned short prevPeak = peakBuffer[slot][prevPeakIndex];

  if(newPeak > prevPeak && (currentTime - lastNoteTime[slot])>MIN_TIME_BETWEEN_NOTES)
  {
    noteReady[slot] = true;
    if(newPeak > noteReadyVelocity[slot])
      noteReadyVelocity[slot] = newPeak;
  }
  else if(newPeak < prevPeak && noteReady[slot])
  {
    noteFire(noteMap[slot], noteReadyVelocity[slot]);
    noteReady[slot] = false;
    noteReadyVelocity[slot] = 0;
    lastNoteTime[slot] = currentTime;
  }

  currentPeakIndex[slot]++;
  if(currentPeakIndex[slot] == PEAK_BUFFER_SIZE) currentPeakIndex[slot] = 0;
  }



void MIDI_DRUM()
  {
    unsigned long currentTime = millis();

    for(short i=0; i<NUM_PIEZOS; ++i)
    {
      //get a new signal from analog read
      unsigned short newSignal = analogRead(slotMap[i]);
      signalBuffer[i][currentSignalIndex[i]] = newSignal;

      //if new signal is 0
      if(newSignal < thresholdMap[i])
      {
        if(!isLastPeakZeroed[i] && (currentTime - lastPeakTime[i]) > MAX_TIME_BETWEEN_PEAKS)
        {
          recordNewPeak(i,0);
        }
        else
        {
          //get previous signal
          short prevSignalIndex = currentSignalIndex[i]-1;
          if(prevSignalIndex < 0) prevSignalIndex = SIGNAL_BUFFER_SIZE-1;
          unsigned short prevSignal = signalBuffer[i][prevSignalIndex];

          unsigned short newPeak = 0;

          //find the wave peak if previous signal was not 0 by going
          //through previous signal values until another 0 is reached
          while(prevSignal >= thresholdMap[i])
          {
            if(signalBuffer[i][prevSignalIndex] > newPeak)
            {
              newPeak = signalBuffer[i][prevSignalIndex];
            }

            //decrement previous signal index, and get previous signal
            prevSignalIndex--;
            if(prevSignalIndex < 0) prevSignalIndex = SIGNAL_BUFFER_SIZE-1;
            prevSignal = signalBuffer[i][prevSignalIndex];
          }

          if(newPeak > 0)
          {
            recordNewPeak(i, newPeak);
          }
        }

      }

      currentSignalIndex[i]++;
      if(currentSignalIndex[i] == SIGNAL_BUFFER_SIZE) currentSignalIndex[i] = 0;
    }
  }

///////////////////////////////////////////////////////////////////////////////
// loop

void loop() {

    if (state == 0)
    {
      MIDI_DRUM();
    }
    else
    {
      mainMenu.poll(menu_lcd,enc);
    }
    state = digitalRead(state_pin);
      }
