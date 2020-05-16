#include <MIDI.h>


#define MIDI_LED A0
#define MIDI_IN 0

void blink_MIDI_LED(void)
{
  // pin PC0 is connected to MIDI activity LED
  PORTC ^= (1 << 0);
}

uint8_t gate_pin[8];
uint8_t gate_state[8];


MIDI_CREATE_DEFAULT_INSTANCE();

// -----------------------------------------------------------------------------

// This function will be automatically called when a NoteOn is received.
// It must be a void-returning function with the correct parameters,
// see documentation here:
// https://github.com/FortySevenEffects/arduino_midi_library/wiki/Using-Callbacks

void handleNoteOn(byte channel, byte pitch, byte velocity)
{
    // Do whatever you want when a note is pressed.

    // Try to keep your callbacks short (no delays ect)
    // otherwise it would slow down the loop() and have a bad impact
    // on real-time performance.

    blink_MIDI_LED();

    // Gate ON
    
    digitalWrite(gate_pin[pitch-36], HIGH);

    delay(1);
    blink_MIDI_LED();
}

void handleNoteOff(byte channel, byte pitch, byte velocity)
{
    // Do something when the note is released.
    // Note that NoteOn messages with 0 velocity are interpreted as NoteOffs.

    blink_MIDI_LED();

    // Gate OFF
  digitalWrite(gate_pin[pitch-36], LOW);

    delay(1);
    blink_MIDI_LED();
}

// -----------------------------------------------------------------------------

void setup()
{
  pinMode(MIDI_IN, INPUT);

  pinMode (MIDI_LED, OUTPUT);
  digitalWrite(MIDI_LED, LOW);
  delay(200);
  digitalWrite(MIDI_LED, HIGH);
  delay(200);
  

  // GATE outputs
  uint8_t j = 0;
  for (uint8_t i = 2; i <= 9; i++)
  {
    gate_pin[j] = i;

    pinMode(i, OUTPUT);

    digitalWrite(i, HIGH);
    delay(200);
    digitalWrite(i, LOW);
    delay(200);
    j++;
  }
  

    // Connect the handleNoteOn function to the library,
    // so it is called upon reception of a NoteOn.
    MIDI.setHandleNoteOn(handleNoteOn);  // Put only the name of the function

    // Do the same for NoteOffs
    MIDI.setHandleNoteOff(handleNoteOff);

    // Initiate MIDI communications, listen to channel 10 only, for drums
    MIDI.begin(10);
    //MIDI.begin(MIDI_CHANNEL_OMNI);
}

void loop()
{
    // Call MIDI.read the fastest you can for real-time performance.
    MIDI.read();
/*
  digitalWrite(MIDI_LED, HIGH);
  delay(500);
  digitalWrite(MIDI_LED, LOW);
  delay(500);
*/
    // There is no need to check if there are messages incoming
    // if they are bound to a Callback function.
    // The attached method will be called automatically
    // when the corresponding message has been received.
}
