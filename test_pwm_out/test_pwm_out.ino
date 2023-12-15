/*
 * MIDI 8 D+, PWM output test
 * 
 * 
*/

#include <PWMFreak.h>


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



// -----------------------------------------------------------------------------

void setup()
{
  for (byte i = 0; i < NBR_PWM_OUTS; i++)
  {
    pinMode(pwm_pins[i], OUTPUT);

    analogWrite(pwm_pins[i], 255);
  }

  // change PWM for use divisor value 1
  setPwmFrequency(3, 1);  // Pins 3 and 11 are paired on timer2
  setPwmFrequency(5, 1);  // Pins 5 and 6  are paired on timer0
  setPwmFrequency(9, 1);  // Pins 9 and 10 are paired on timer1

  // comment for audio test
  delay(1000);
}

void loop()
{
  for (byte j = 0; j < 256; j++)
  {
    for (byte i = 0; i < NBR_PWM_OUTS; i++)
    {
      analogWrite(pwm_pins[i], j);
    }
    delay(300);
  }
}
