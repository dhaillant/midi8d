/*
 * One Gate per MIDI Channel
 * 
 * Each gate (1–8) remains ON as long as a note is held on its corresponding MIDI channel.
 * 
 * Example: if a NoteOn message is received on MIDI channel 2, Gate 2 switches ON (+5V).
 * To keep track of the number of active notes per channel, each channel is managed by note_stack.
 * note_stack is a set of functions written by Émilie Gilet for the CVpal module by Mutable Instruments.
 * See note_stack.h for more details.
 * 
 * Version for original MIDI8d and MIDI8d+ (with PWM outputs)
 * Change HW_VERSION for correct pin order.
 * 
 * Requires the Arduino MIDI Library:
 * https://github.com/FortySevenEffects/arduino_midi_library
 * 
*/

/* 
 * Hardware version
 * 
 * Comment/Uncomment the correct lines: 
 */
//#define HW_MIDI8d_Original
#define HW_MIDI8d_Plus


#ifdef HW_MIDI8d_Plus
  // Pin definitions
  #define MIDI_LED A0
  
  #define MIDI_IN 0
  
  #define NBR_GATE_OUTS 8
  
  byte gate_pins[NBR_GATE_OUTS] = {
    2, 4, 7, 8, 9, 3, 5, 6
  };    // array of output pin numbers (Arduino #)
#else
  // Pin definitions
  #define MIDI_LED A0
  
  #define MIDI_IN 0
  
  #define NBR_GATE_OUTS 8
  
  byte gate_pins[NBR_GATE_OUTS] = {
    2, 3, 4, 5, 6, 7, 8, 9
  };    // array of output pin numbers (Arduino #)
#endif



#include <MIDI.h>

// MIDI Channel we want to react to
#define MIDI_CHANNEL MIDI_CHANNEL_OMNI

MIDI_CREATE_DEFAULT_INSTANCE();





bool MIDI_LED_needs_refresh = false;

// MIDI_LED_BLINK_TIME is in ms (timer is 1kHz)
// (do not exceed uint8_t max value):
#define MIDI_LED_BLINK_TIME 20
uint8_t MIDI_blink_counter;


#include "note_stack.h"

const uint8_t kNumVoices = NBR_GATE_OUTS; // one voice per MIDI channel, limited by the # of outputs
NoteStack<6> mono_allocator[kNumVoices];  // 6 notes per voice

const uint8_t kRetriggerDuration = 2;

bool needs_refresh;
bool legato;


struct State {
  bool gate[kNumVoices];
};

State state;




// -----------------------------------------------------------------------------

volatile uint8_t control_clock_tick;

ISR(TIMER0_COMPA_vect) {
  // 1kHz clock for timing trigger pulses.
  ++control_clock_tick;
}

void tick()
{
  // if MIDI_blink_counter is not 0
  if (MIDI_blink_counter)
  {
    --MIDI_blink_counter;
  }
  else
  {
    // MIDI_blink_counter is 0 so we can request a refresh of the outputs
    MIDI_LED_needs_refresh = true;
  }
}

// start MIDI LED timer
void blink_MIDI_LED(void)
{
  // set the counter to a predefined value. It will be decreased by tick() at 1kHz. At 0, LED will be turned off
  MIDI_blink_counter = MIDI_LED_BLINK_TIME;
  MIDI_LED_needs_refresh = true;
}



void setup()
{
  // set Inputs
  pinMode(MIDI_IN, INPUT);

  // set Outputs
  pinMode (MIDI_LED, OUTPUT);

  // initialize MIDI LED state (off) and blink
  for (uint8_t i = 0; i < 4; i++)
  {
    digitalWrite(MIDI_LED, LOW);
    delay(50);
    digitalWrite(MIDI_LED, HIGH);
    delay(100);
  }

  // GATE outputs
  for (byte i = 0; i < NBR_GATE_OUTS; i++)
  {
    pinMode(gate_pins[i],       OUTPUT);

    digitalWrite(gate_pins[i],  HIGH);
    delay(50);
    digitalWrite(gate_pins[i],  LOW);
    delay(50);
  }

  // note_stack initialisation
  for (uint8_t i = 0; i < NBR_GATE_OUTS; i++)
  {
    mono_allocator[i].Init();
  }

  // Set a 1kHz timer for non-blocking Trigger and blinking durations/delays
  TCCR0A |= (1 << WGM01);                       // Set Timer0 to CTC (Clear Timer on Compare Match) mode
  TCCR0B |= (1 << CS01) | (1 << CS00);          // Set prescaler to 64 (prescaler bits: CS02=0, CS01=1, CS00=1)
  OCR0A = 124; // (16MHz / (64 * 1000Hz)) - 1   // Set compare match register to generate a 1kHz frequency
  TIMSK0 |= (1 << OCIE0A);                      // Enable timer compare interrupt
  sei();                                        // Enable interrupts


  // connect MIDI messages to handlers
  MIDI.setHandleNoteOn(handleNoteOn);
  MIDI.setHandleNoteOff(handleNoteOff);

  // Initiate MIDI communications, listen to ALL channels
  MIDI.begin(MIDI_CHANNEL);
}

void loop()
{
  // read incomming MIDI messages
  MIDI.read();

  // update outputs if needed
  if (needs_refresh)
  {
    render_gates();

    needs_refresh = false;
  }
  if (MIDI_LED_needs_refresh)
  {
    render_MIDI_LED();
    MIDI_LED_needs_refresh = false;
  }


  if (control_clock_tick)
  {
    --control_clock_tick;
    tick();
  }
}

void handleNoteOn(byte channel, byte pitch, byte velocity)
{
  if ((channel > 0) && (channel <= 8))
  {
    if (velocity == 0)
    {
      // probably already handled by MIDI Library... TBC
      handleNoteOff(channel, pitch, velocity);
    }
    else
    {
      mono_allocator[channel - 1].NoteOn(pitch, velocity);

      // switch the output to ON for the gate corresponding to the channel on which we received the incoming NoteOn
      state.gate[channel - 1] = true;
    }
    needs_refresh = true;
  }

  blink_MIDI_LED();
}

void handleNoteOff(byte channel, byte pitch, byte velocity)
{
  // NoteOn messages with 0 velocity are interpreted as NoteOffs.

  if ((channel > 0) && (channel <= 8))
  {
    uint8_t previous_top_note = mono_allocator[0].most_recent_note().note;
    mono_allocator[channel - 1].NoteOff(pitch);

    // check if there are stored notes: if yes, keep the output ON; if no, switch the output OFF
    state.gate[channel - 1] = (mono_allocator[channel - 1].size() > 0 ? true : false);

    needs_refresh = true;
  }

  blink_MIDI_LED();
}

void render_gates(void)
{
  for (uint8_t i = 0; i < kNumVoices; i++)
  {
    digitalWrite(gate_pins[i], state.gate[i]);
  }
}

void render_MIDI_LED()
{
  digitalWrite(MIDI_LED,MIDI_blink_counter > 0 ? LOW : HIGH);
}
