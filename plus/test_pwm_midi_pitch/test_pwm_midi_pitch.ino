/*
 * MIDI 8 D+, with PWM output on some pins
 * 
 * Requires the Arduino MIDI Library:
 * https://github.com/FortySevenEffects/arduino_midi_library
 * 
 * HW 0.1-buffered
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

#include <MIDI.h>
#include <PWMFreak.h>

// Pin definitions
#define MIDI_LED A0
#define MIDI_IN 0


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

  // generate a GATE signal
  // change dig_pins index to choose other digital output
  digitalWrite(dig_pins[0], HIGH);
  
  // generate a CV signal based on MIDI note number "pitch" which is between 12 (C0 = 0V) and 72 (C5 = 5V)
  // adapt that MIDI note number to n/255, where note 12 corresponds to 0/255 and 72 to 255/255
  // change pwm_pins index to choose other "analog" output
  if (pitch < 12) { pitch = 12; }
  if (pitch > 72) { pitch = 72; }
  
  byte pwm = map(pitch, 12, 72, 0, 255);
  analogWrite(pwm_pins[0], pwm);

  // Note: in order to use the module as a true MIDI2CV, we need to add buffer memory (FIFO) of incoming messages


  toggle_MIDI_LED();
}

void handleNoteOff(byte channel, byte pitch, byte velocity)
{
  // Do something when the note is released.
  // Note that NoteOn messages with 0 velocity are interpreted as NoteOffs.
  
  toggle_MIDI_LED();

  // switch off GATE signal
  // change dig_pins index to choose other digital output
  digitalWrite(dig_pins[0], LOW);

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

  for (byte i = 0; i < NBR_DIG_OUTS; i++)
  {
    pinMode(dig_pins[i], OUTPUT);

    digitalWrite(dig_pins[i], LOW);
  }


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

  // *** Do NOT activate Serial as it interacts with MIDI communication ***
  // debug through serial communication
  //Serial.begin(115200);
  //Serial.println("Hello test pitch");
  // *** Do NOT activate Serial as it interacts with MIDI communication ***

  // tests
/*  // list PWM values for 5 octaves (8 bits):
  uint16_t pwm = 0;
  for (byte i = 12; i < 73; i++)
  {
    pwm = map(i, 12, 72, 0, 255);
    //Serial.println(pwm);
  }
  // list PWM values for 5 octaves (16 bits):
  for (byte i = 12; i < 73; i++)
  {
    pwm = map(i, 12, 72, 0, 65535);
    //Serial.println(pwm);
  }*/
}

void loop()
{
    // Call MIDI.read the fastest you can for real-time performance.
    MIDI.read();
}
