# ESP32-based Guitar Tuner
Documentation of a devices that performs as fully standalone tuner dedicated for eletric guitars.
This repository is dedicated to my engineering thesis which is ESP32 microcontroller-based guitar tuner.
The goal of this project is to create a uC-based chromatic guitar tuner with acceptable precision and simple
features. I dedicate this repository to people like me who couldn't find a bit more advanced implementations
of uC-based guitar tuner.

The tuner is fully based on ESP32 LoLin WEMOS Lite developement board with code based on ArduinoFFT library (Arduino IDE) for
calculations of the incoming signal spectrum for every ~0.11s signal frame. Each frame is being transformed to frequency
domain and the fundamental frequency (f0) is picked up in real-time using a simple peak detection algorythm.
To enhance frequency resolution parabolic interpolation with Blackman-Harris window is utilized which provide satisfactory results.

The detected f0 is converted to the closest 12-TET note and then difference in cents is compared with proper 12-TET frequency.
Then both the note name and difference in cents is displayed on proper 7seg display and a LED array. The display is provided with
my own design and code (look at guitarTunerDisplay.cpp)

The device is designed to work with an analog signal processing path utilizing low pass filters for antialiasing purposes and
with designed display described in the previous paragraph.
The documentation for electrical connections is provided in the repository (schematic.pdf).


