# midi8d
MIDI to 8 digital outputs
---

MIDI8d is a very simple, yet super versatile MIDI-to-8-digital-outputs module, that you can program and adapt to suit your needs.

It's a 4 HP wide module in Eurorack format.

The core is an Arduino Nano (ATmega328p).

MIDI8d can't produce CV. The outputs are digital only: 0 or 5 volts.

It is perfect for generating gate or trig signals, for driving drums for example, or clock events.
Two example sketches demonstrate how to use the module.

See https://github.com/dhaillant/midi8d/wiki for detailed firmware information

---
New :

The MIDI8d+ (note the PLUS) has the ability to produce **4 analog outputs**.

These analog signals are generated by PWM and are filtered (low pass filter to attenuate the PWM noise) and buffered (the analog value doesn't change upon load).

This is a very convenient, easy and inexpensive solution for adding CV controls to your modular system.

However, PWM isn't exactly perfect for sensitive and precise CV values (example, VCO tuning), but PWM is ok for anything else: CV for VCF frequency, CV for VCA, CV for delays, FM modulations etc. You can even produce audio signals.

The first 3 analog outputs have 8 bits resolution. The 4th one is 16 bits.

---


MIDI8d Hardware documentation can be found at https://www.davidhaillant.com/midi8d-8-digital-outputs/

MIDI8d+ Hardware documentation can be found at https://www.davidhaillant.com/midi-8dp-midi-to-4-analog-and-4-digital-outputs/

