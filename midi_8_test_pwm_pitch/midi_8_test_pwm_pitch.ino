/*
 * MIDI 8 D+, with PWM output on some pins
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

//#define CV_OUT 6

#define NBR_PWM_OUTS 5
byte pwm_pins[NBR_PWM_OUTS] = {
  3, 5, 6, 9, 10
};    // array of pwm output pin numbers (Arduino #)

/*
 * The following pins can do PWM:
 * 
 * D3, D5, D6, D9/D10*
 * 
 * (*: D10 is not in use here)
 */

#define NBR_DIG_OUTS 4
byte dig_pins[NBR_DIG_OUTS] = {
  2, 4, 7, 8
};    // array of non-pwm output pin numbers (Arduino #)


// MIDI Channel we want to react to
//#define MIDI_CHANNEL 10
#define MIDI_CHANNEL MIDI_CHANNEL_OMNI

MIDI_CREATE_DEFAULT_INSTANCE();

void toggle_MIDI_LED(void)
{
  // pin PC0 is connected to MIDI activity LED
  PORTC ^= (1 << 0);
}


void handleNoteOn(byte channel, byte pitch, byte velocity)
{
  // Do something when the note is pressed.

  toggle_MIDI_LED();
  
  byte pwm = map(pitch, 12, 72, 0, 255);
  analogWrite(pwm_pins[0], pwm);

  toggle_MIDI_LED();

  //Serial.println(pitch);
}

void handleNoteOff(byte channel, byte pitch, byte velocity)
{
  // Do something when the note is released.
  // Note that NoteOn messages with 0 velocity are interpreted as NoteOffs.
  toggle_MIDI_LED();

  analogWrite(pwm_pins[0], 0);
  toggle_MIDI_LED();
}

// -----------------------------------------------------------------------------

void setup()
{
  // set Inputs
  pinMode(MIDI_IN, INPUT);

  // set Outputs
  pinMode (MIDI_LED, OUTPUT);

  // initialize MIDI LED state (off)
  digitalWrite(MIDI_LED, LOW);
  delay(200);
  toggle_MIDI_LED();
  delay(200);

  // PWM outputs
  //pinMode(CV_OUT, OUTPUT);

  // change PWM for use divisor value 1
  setPwmFrequency(3, 1);  // Pins 3 and 11 are paired on timer2
  setPwmFrequency(5, 1);  // Pins 5 and 6  are paired on timer0
  setPwmFrequency(9, 1);  // Pins 9 and 10 are paired on timer1

  for (byte i = 0; i < NBR_PWM_OUTS; i++)
  {
    pinMode(pwm_pins[i], OUTPUT);

    analogWrite(pwm_pins[i], 0);
  }


  // Connect the handleNoteOn function to the library,
  // so it is called upon reception of a NoteOn.
  MIDI.setHandleNoteOn(handleNoteOn);

  // Do the same for NoteOffs
  MIDI.setHandleNoteOff(handleNoteOff);

  // Initiate MIDI communications, listen to channel 10 only, for drums
  MIDI.begin(MIDI_CHANNEL);

  // debug through serial communication
  Serial.begin(115200);
  Serial.println("Hello test pitch");

  // tests
  // list PWM values for 5 octaves (8 bits):
  uint16_t pwm = 0;
  for (byte i = 12; i < 73; i++)
  {
    pwm = map(i, 12, 72, 0, 255);
    Serial.println(pwm);
  }
  // list PWM values for 5 octaves (16 bits):
  for (byte i = 12; i < 73; i++)
  {
    pwm = map(i, 12, 72, 0, 65535);
    Serial.println(pwm);
  }
}

void loop()
{
    // Call MIDI.read the fastest you can for real-time performance.
    MIDI.read();
}
