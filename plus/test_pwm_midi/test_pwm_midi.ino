/*
 * MIDI 8 Drum Gate Outputs, with PWM output on some pins, reflecting velocity - Channel 10
 * 
 * Requires the Arduino MIDI Library:
 * https://github.com/FortySevenEffects/arduino_midi_library
 * 
*/

#include <MIDI.h>
#include <PWMFreak.h>

// Pin definitions
#define MIDI_LED A0

#define MIDI_IN 0

#define NBR_GATE_OUTS 8
byte gate_pins[NBR_GATE_OUTS] = {
  2, 3, 4, 5, 6, 7, 8, 9
};    // array of output pin numbers (Arduino #)

byte pwm_pins[NBR_GATE_OUTS] = {
  0, 1, 0, 1, 1, 0, 0, 1
};    // list of pins that can do PWM. 0 is "no PWM", 1 is "PWM available"
/*
 * The following pins can do PWM:
 * 
 * D3, D5, D6, D9/D10*
 * 
 * (*: D10 is not in use here)
 */

// state of each gate output
bool gate_states[NBR_GATE_OUTS];


// MIDI Channel we want to react to
#define MIDI_CHANNEL 10
//#define MIDI_CHANNEL MIDI_CHANNEL_OMNI

// lowest note we want to respond to
#define LOWEST_MIDI_NOTE 36
// highest note will be 
#define HIGHEST_MIDI_NOTE LOWEST_MIDI_NOTE + NBR_GATE_OUTS - 1


MIDI_CREATE_DEFAULT_INSTANCE();

void blink_MIDI_LED(void)
{
  // pin PC0 is connected to MIDI activity LED
  PORTC ^= (1 << 0);
}


void handleNoteOn(byte channel, byte pitch, byte velocity)
{
  // gate is the output number to be activated
  byte gate = 0;
  
  blink_MIDI_LED();
  // if the received NoteOn message is in the range of the note we can respond to
  if ((pitch >= LOWEST_MIDI_NOTE) && (pitch <= HIGHEST_MIDI_NOTE))
  {
    // set the gate number according to the MIDI note number
    gate = pitch - LOWEST_MIDI_NOTE;
    // Gate ON, or PWM ?
    if (pwm_pins[gate] == 0)
    {
      // not a PWM pin so we output digital stuff:
      digitalWrite(gate_pins[gate], HIGH);
    }
    else
    {
      // pin can do PWM so lets write velocity
      analogWrite(gate_pins[gate], velocity << 1);
    }
    // record the gate state
    gate_states[gate] = HIGH;
  }
}

void handleNoteOff(byte channel, byte pitch, byte velocity)
{
  // Do something when the note is released.
  // Note that NoteOn messages with 0 velocity are interpreted as NoteOffs.

  // gate is the output number to be deactivated
  byte gate = 0;
  
  // if the received NoteOff message is in the range of the note we can respond to
  if ((pitch >= LOWEST_MIDI_NOTE) && (pitch <= HIGHEST_MIDI_NOTE))
  {
    // set the gate number according to the MIDI note number
    gate = pitch - LOWEST_MIDI_NOTE;
    // Gate OFF, or PWM at 0 ?
    if (pwm_pins[gate] == 0)
    {
      // not a PWM pin so we output digital stuff:
      digitalWrite(gate_pins[gate], LOW);
    }
    else
    {
      // pin can do PWM so lets write velocity (or 0)
      analogWrite(gate_pins[gate], 0);
    }
    // record the gate state
    gate_states[gate] = LOW;
  }
  blink_MIDI_LED();
}

// -----------------------------------------------------------------------------

void setup()
{
  // set Inputs
  pinMode(MIDI_IN, INPUT);

  // set Outputs
  pinMode (MIDI_LED, OUTPUT);

  digitalWrite(MIDI_LED, LOW);
  delay(200);
  blink_MIDI_LED();
  delay(200);

  // GATE outputs
  for (byte i = 0; i < NBR_GATE_OUTS; i++)
  {
    pinMode(gate_pins[i], OUTPUT);

    digitalWrite(gate_pins[i], HIGH);
    delay(50);
    digitalWrite(gate_pins[i], LOW);
    delay(50);
  }

  // change PWM for use divisor value 1
  setPwmFrequency(3, 1);  // Pins 3 and 11 are paired on timer2
  setPwmFrequency(5, 1);  // Pins 5 and 6  are paired on timer0
  setPwmFrequency(9, 1);  // Pins 9 and 10 are paired on timer1

  // Connect the handleNoteOn function to the library,
  // so it is called upon reception of a NoteOn.
  MIDI.setHandleNoteOn(handleNoteOn);

  // Do the same for NoteOffs
  MIDI.setHandleNoteOff(handleNoteOff);

  // Initiate MIDI communications, listen to channel 10 only, for drums
  MIDI.begin(MIDI_CHANNEL);
}

void loop()
{
    // Call MIDI.read the fastest you can for real-time performance.
    MIDI.read();
}
