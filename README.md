arduino-sous-vide-device
========================

Code for running a Sous Vide controller on an Arduino with a 16x2 LCD screen, either a 10K Thermistor or a DS18B20 Temperature Sensor, and some buttons (at time of writing, three, which will be used as T++, T--, and "Start/Stop").

It also includes an output pin to control a relay (a PowerSwitch Tail IIU via a transistor to allow more current draw in my case), an optional TMP36 Ambient Temperature Sensor, 

It's based on assorted libraries and example code (PID Library, OneWire protocol, Dallas Semiconductor TLC library, etc). I will attempt to collect references here, but if I miss them, please feel free to let me know.

Plans include adding a timer function, an RTC and data logger for tracking performance, autotuning, a fourth button (for "menu") and a more sophisticated UI to allow for editing of parameters etc, and docuementation for the circuits and components needed to set this up.