/*
 * MIDI 8 D+, with PWM output on some pins
 * 
 * Requires the Arduino MIDI Library:
 * https://github.com/FortySevenEffects/arduino_midi_library
 * 
 * 
 * Code from CVPAL (note_stack, midi_handler)
 * Copyright 2013 Emilie Gillet (emilie.o.gillet@gmail.com)
 * https://github.com/pichenettes/cvpal

 * 
 * Will output GATE signal on output 1 (pin D2) and filtered PWM signal on output 2 (pin D3)
 * MIDI LED blinks on activity
 * MIDI messages for all channels are processed
 * 
 * Test with the following commands from Linux:
 * amidi -p hw:2 -S '90 0C 7F'
 * amidi -p hw:2 -S '90 24 7F'
 * amidi -p hw:2 -S '90 30 7F'
 * amidi -p hw:2 -S '90 3C 7F'
 * amidi -p hw:2 -S '90 48 7F'
 * (respectively sends notes ON)
*/



#include <PWMFreak.h>

// Pin definitions
#define MIDI_LED A0
#define MIDI_IN 0

/*
 * ======================
 * MIDI 8D Output Mapping
 * ======================
 * 
    Output  Pin     Function
    -------------------------
    DIG 1   D2      START/STOP
    DIG 2   D4      PPQN Clock
    DIG 3   D7      SIXTEENTH Clock
    DIG 4   D8      TRIPLET Clock
    
    PWM 1   D9/D10  PITCH
    PWM 2   D3      VELOCITY
    PWM 3   D5      CC
    PWM 4   D6      GATE
*/

// GATE pins
#define STARTSTOP_PIN   2   // D2 (output 1)
#define PPQN_PIN        4   // D4 (output 2)
#define SIXTEENTH_PIN   7   // D7 (output 3)
#define TRIPLET_PIN     8   // D8 (output 4)
#define GATE_PIN        6   // D6 (output 8)

// PWM pins
#define PITCH_MSB_PIN   9   // D9/D10 (output 5)
#define PITCH_LSB_PIN   10  // D9/D10 (output 5)
#define VELOCITY_PIN    3   // D3     (output 6)
#define CC_PIN          5   // D5     (output 7)



//#define NBR_PWM_OUTS 5
//const byte pwm_pins[NBR_PWM_OUTS] = {
//  3, 5, 6, 9, 10
//};    // array of pwm output pin numbers (Arduino #)

/*
 * The following pins can do PWM:
 * 
 * D3, D5, D6, D9/D10*
 * 
 * D10 is for 16bit resolution and is combined with D9
 */

//#define NBR_DIG_OUTS 5
//const byte dig_pins[NBR_DIG_OUTS] = {
//  2, 4, 7, 8, 6
//};    // array of non-pwm output pin numbers (Arduino #)




#include <MIDI.h>

// MIDI Channel we want to react to
#define MIDI_CHANNEL MIDI_CHANNEL_OMNI

MIDI_CREATE_DEFAULT_INSTANCE();





#include "note_stack.h"

const uint8_t kNumVoices = 1;
NoteStack<6> mono_allocator[kNumVoices];

const uint8_t kRetriggerDuration = 2;

int16_t pitch_bend;
uint8_t control_change;
bool needs_refresh;
bool legato;
uint8_t force_retrigger;


struct State {
//  uint16_t cv[3];
//  bool gate[5];
  uint16_t pitch;
  uint8_t velocity;
  bool gate;
};

State state;

// index for State.gate[] array of digital outputs
//#define STARTSTOP 0 // D2 (output 1)
//#define PPQN 1      // D4 (output 2)
//#define SIXTEENTH 2 // D7 (output 3)
//#define TRIPLET 3   // D8 (output 4)
//#define GATE 4      // D6 (output 8)

// index for State.cv[] array of analog outputs
//#define PITCH 0     // D9/D10 (output 5)
//#define VELOCITY 1  // D3     (output 6)
//#define CC 2        // D5     (output 7)



bool play_flag = false;
uint8_t clock_step = 0;


// -----------------------------------------------------------------------------



volatile uint8_t control_clock_tick;

ISR(TIMER0_COMPA_vect) {
  // 1kHz clock for timing trigger pulses.
  ++control_clock_tick;

  //toggle_MIDI_LED();
}

void toggle_MIDI_LED(void)
{
  // pin PC0 is connected to MIDI activity LED
  PORTC ^= (1 << 0);
}

void handleNoteOff(byte channel, byte pitch, byte velocity);



void setup()
{
  // set Inputs
  pinMode(MIDI_IN, INPUT);

  // set Outputs
  pinMode (MIDI_LED,      OUTPUT);

  // GATE pins
  pinMode (MIDI_LED,      OUTPUT);
  pinMode (STARTSTOP_PIN, OUTPUT);
  pinMode (PPQN_PIN,      OUTPUT);
  pinMode (SIXTEENTH_PIN, OUTPUT);
  pinMode (TRIPLET_PIN,   OUTPUT);
  pinMode (GATE_PIN,      OUTPUT);

  // PWM pins
  pinMode (PITCH_MSB_PIN, OUTPUT);
  pinMode (PITCH_LSB_PIN, OUTPUT);
  pinMode (VELOCITY_PIN,  OUTPUT);
  pinMode (CC_PIN,        OUTPUT);


  // change PWM for use divisor value 1
  setPwmFrequency(3, 1);  // Pins 3 and 11 are paired on timer2
  //setPwmFrequency(5, 1);  // Pins 5 and 6  are paired on timer0
  setPwmFrequency(9, 1);  // Pins 9 and 10 are paired on timer1


  MIDI.setHandleNoteOn(handleNoteOn);
  MIDI.setHandleNoteOff(handleNoteOff);

  MIDI.setHandlePitchBend(handlePitchBend);
  MIDI.setHandleControlChange(handleControlChange);

  MIDI.setHandleClock(handleClock);
  MIDI.setHandleStart(handleStart);
  MIDI.setHandleContinue(handleContinue);
  MIDI.setHandleStop(handleStop);

  //MIDI.setHandleSystemReset(handleSystemReset);

  mono_allocator[0].Init();   // note_stack initialisation


  MIDI.begin(MIDI_CHANNEL);


  TCCR0A |= (1 << WGM01);                       // Set Timer0 to CTC (Clear Timer on Compare Match) mode
  TCCR0B |= (1 << CS01) | (1 << CS00);          // Set prescaler to 64 (prescaler bits: CS02=0, CS01=1, CS00=1)
  OCR0A = 124; // (16MHz / (64 * 1000Hz)) - 1   // Set compare match register to generate a 1kHz frequency
  TIMSK0 |= (1 << OCIE0A);                      // Enable timer compare interrupt
  sei();                                        // Enable interrupts


  // initialize MIDI LED state (off) and blink to say "hello"
  digitalWrite(MIDI_LED, LOW);
  delay(200);
  toggle_MIDI_LED();
  delay(200);
}

void loop()
{
  MIDI.read();

  if (needs_refresh)
  {
    render_mono_pitch();
    render_gates();
    render_cc();
    render_clocks();

    needs_refresh = false;
  }

  if (control_clock_tick)
  {
    --control_clock_tick;
    tick();
  }
}

void handleNoteOn(byte channel, byte pitch, byte velocity)
{
  // Pitch: 
  // Velocity; 0 to 127

  toggle_MIDI_LED();

  if (velocity == 0)
  {
    // probably already handled by MIDI Library... TBC
    handleNoteOff(channel, pitch, velocity);
  }
  else
  {
    mono_allocator[0].NoteOn(pitch, velocity);
//    if (state.gate[GATE] && !legato[0]) {
    if (state.gate && !legato) {
      force_retrigger = kRetriggerDuration;
    }
  }
  needs_refresh = true;

  toggle_MIDI_LED();
}

void handleNoteOff(byte channel, byte pitch, byte velocity)
{
  // Pitch: 
  // Velocity; 0 to 127
  // Note that NoteOn messages with 0 velocity are interpreted as NoteOffs.
  
  toggle_MIDI_LED();

  uint8_t previous_top_note = mono_allocator[0].most_recent_note().note;
  mono_allocator[0].NoteOff(pitch);
  if (mono_allocator[0].size() && mono_allocator[0].most_recent_note().note != previous_top_note) {
    force_retrigger = !legato ? kRetriggerDuration : 0;
  }
  needs_refresh = true;

  toggle_MIDI_LED();
}

void handlePitchBend(byte channel, int bend)
{
  // bend: -8192 to 8191
  toggle_MIDI_LED();
  pitch_bend = bend;
  needs_refresh = true;
  toggle_MIDI_LED();
}

void handleControlChange(byte channel, byte number, byte value)
// number:  0 to 127
// value:   0 to 127
{
  toggle_MIDI_LED();
  if (number == 1)
  {
    control_change = value << 1;  // 127 -> 255
  } else if (number == 68)
  {
    legato = value >= 64;
  }
  needs_refresh = true;
  toggle_MIDI_LED();
}

void handleClock(void)
{
  toggle_MIDI_LED();

  clock_step++;

  needs_refresh = true;
  toggle_MIDI_LED();
}

void handleStart(void)
{
  toggle_MIDI_LED();
  play_flag = true;
  clock_step = 0;

  // manually switch ON STARTSTOP_PIN
  digitalWrite(STARTSTOP_PIN, HIGH);

  needs_refresh = true;
  toggle_MIDI_LED();
}

void handleContinue(void)
{
  toggle_MIDI_LED();
  play_flag = true;

  needs_refresh = true;
  toggle_MIDI_LED();
}

void handleStop(void)
{
  toggle_MIDI_LED();
  play_flag = false;
  clock_step = 0;
  
  // manually switch OFF STARTSTOP_PIN
  digitalWrite(STARTSTOP_PIN, LOW);

  needs_refresh = true;
  toggle_MIDI_LED();
}


inline uint16_t NoteToCv(uint8_t pitch, int16_t bend)
{
  // generate a CV signal based on MIDI note number "pitch" which is between 12 (C0 = 0V) and 72 (C5 = 5V)
  // adapt that MIDI note number to n/65535, where note 12 corresponds to 0/65535 and 72 to 65535/65535
  // if using 16 bits resolution, you can only use the pwm_pins 3 and 4 (D9 and D10)
  if (pitch < 12) { pitch = 12; }
  if (pitch > 72) { pitch = 72; }
  
  uint16_t pwm = map(pitch, 12, 72, 0, 65535);
  return pwm;

  //int16_t pitch = base << 7;
  //pitch += bend >> 5;
  //return calibration_table_[output_channel].Calibrate(pitch);
  //return pitch;
}

void render_mono_pitch(void)
{
  if (mono_allocator[0].size()) {
    int16_t note = mono_allocator[0].most_recent_note().note;
//    state.cv[PITCH] = NoteToCv(note, pitch_bend[0]);
//    state.cv[VELOCITY] = mono_allocator[0].most_recent_note().velocity << 1;  // shift to map to 0..256
    state.pitch = NoteToCv(note, pitch_bend);
    state.velocity = mono_allocator[0].most_recent_note().velocity << 1;  // shift to map to 0..256
//    state.gate[GATE] = !force_retrigger[0];
    state.gate = !force_retrigger;
  } else {
//    state.gate[GATE] = false;
    state.gate = false;
  }

//  analogWrite(PITCH_MSB_PIN, (state.cv[PITCH] & 0xff00) >> 8);
//  analogWrite(PITCH_LSB_PIN, state.cv[PITCH] & 0xff);
  analogWrite(PITCH_MSB_PIN, (state.pitch & 0xff00) >> 8);
  analogWrite(PITCH_LSB_PIN, state.pitch & 0xff);

//  analogWrite(VELOCITY_PIN, state.cv[VELOCITY]);
  analogWrite(VELOCITY_PIN, state.velocity);
}

void render_gates(void)
{
//  digitalWrite(GATE_PIN, state.gate[GATE]);
  digitalWrite(GATE_PIN, state.gate);
}

void render_cc(void)
{
  analogWrite(CC_PIN, control_change);
}

void render_clocks(void)
{
  if ( play_flag )
  {
    clock_step = clock_step % 96; // this will reset the clock_step to 0 after 96 ppqn are received,
    //assuming your MIDI clock data is being sent at the standard 24ppqn that will reset clock_step every bar.

    // Each pulse is 1 ppqn long
 
/*    
     // every 96 clock steps
    if (clock_step == 0) {
      digitalWrite(WHOLE_PIN, HIGH);      // whole note ON
    } else {
      digitalWrite(WHOLE_PIN, LOW);       // whole note OFF
    }
*/
/*
    // every 48 on 96 clock steps
    if (clock_step % 48 == 0) {
      digitalWrite(HALF_PIN, HIGH);       // half note ON
    } else {
      digitalWrite(HALF_PIN, LOW);        // half note OFF
    }
*/
/*
    // every 24 clock steps
    if (clock_step % 24 == 0) {
      digitalWrite(QUARTER_PIN, HIGH);    // quarter note ON
    } else {
      digitalWrite(QUARTER_PIN, LOW);     // quarter note OFF
    }
*/
/*
    // every 12 clock steps
    if (clock_step % 12 == 0) {
      digitalWrite(EIGHTH_PIN, HIGH);     // eighth note ON
    } else {
      digitalWrite(EIGHTH_PIN, LOW);      // eighth note OFF
    }
*/
    // every 6 clock steps
    if (clock_step % 6 == 0) {
      digitalWrite(SIXTEENTH_PIN, HIGH);  // sixteenth note ON
    } else {
      digitalWrite(SIXTEENTH_PIN, LOW);   // sixteenth note OFF
    }

    // every 32 clock steps
    if (clock_step % 32 == 0) {
      digitalWrite(TRIPLET_PIN, HIGH);    // triplet note ON
    } else {
      digitalWrite(TRIPLET_PIN, LOW);     // triplet note OFF
    }

    // every 4 clock steps
    if (clock_step % 4 == 0) {
      digitalWrite(PPQN_PIN, HIGH);       // PPQN note ON
    } else {
      digitalWrite(PPQN_PIN, LOW);        // PPQN note OFF
    }
  }
}

void tick()
{
  if (force_retrigger)
  {
    --force_retrigger;
    needs_refresh = true;
  }
}
