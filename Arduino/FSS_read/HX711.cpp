#include <Arduino.h>
#include "HX711.h"

#if ARDUINO_VERSION <= 106
    void yield(void) {};
#endif

// Receiving the initial variables
HX711::HX711(int n_channel, int dout[], byte pd_sck, byte gain) {
  begin(n_channel, dout, pd_sck, gain);
}

HX711::HX711() {
}

HX711::~HX711() {
}

// Declaring the initially received variables and setting the pin modes
void HX711::begin(int n_channel, int dout[], byte pd_sck, byte gain) {
  PD_SCK = pd_sck;
  N_CHANNEL = n_channel;

  pinMode(PD_SCK, OUTPUT);
  for (int i = 0; i < N_CHANNEL; i++) {
    pinMode(dout[i],INPUT);
  }

  set_gain(dout, gain);
}

bool HX711::is_ready(int dout_pin) {
  return digitalRead(dout_pin) == LOW;
}

// Setting the gain value to be used in the read function
// Set as 1 as default
void HX711::set_gain(int dout[], byte gain) {
  switch (gain) {
    case 128:   // channel A, gain factor 128
      GAIN = 1;
      break;
    case 64:    // channel A, gain factor 64
      GAIN = 3;
      break;
    case 32:    // channel B, gain factor 32
      GAIN = 2;
      break;
  }

  digitalWrite(PD_SCK, LOW);
  // Not sure of this but it reads the values of the sensors without actually using the result given out
  for (int i = 0; i < N_CHANNEL; i++) {
    read(dout[i]);
  }
}

long HX711::read(int dout_pin) {
  // Waiting for the chip to be ready
  while (!is_ready(dout_pin)) {
    // Will do nothing on Arduino but prevent resets of ESP8266 (Watchdog Issue)
    yield();
  }

  unsigned long value = 0;
  uint8_t data[3] = { 0 };
  uint8_t filler = 0x00;

  // Pulsing the clock pin 24 times to read the data
  data[2] = shiftIn(dout_pin, PD_SCK, MSBFIRST);
  data[1] = shiftIn(dout_pin, PD_SCK, MSBFIRST);
  data[0] = shiftIn(dout_pin, PD_SCK, MSBFIRST);

  // Setting the channel and the gain factor for the next reading using the clock pin
  for (unsigned int i = 0; i < GAIN; i++) {
    digitalWrite(PD_SCK, HIGH);
    digitalWrite(PD_SCK, LOW);
  }

  // Replicating the most significant bit to pad out a 32-bit signed integer
  if (data[2] & 0x80) {
    filler = 0xFF;
  } else {
    filler = 0x00;
  }

  // Constructing a 32-bit signed integer
  value = ( static_cast<unsigned long>(filler) << 24
      | static_cast<unsigned long>(data[2]) << 16
      | static_cast<unsigned long>(data[1]) << 8
      | static_cast<unsigned long>(data[0]) );

  return static_cast<long>(value);
}

// Reading the average value of the sensor every few iterations
// Used only for the offset
long HX711::read_average(int dout_pin, byte times) {
  long sum = 0;
  for (byte i = 0; i < times; i++) {
    sum += read(dout_pin);
    yield();
  }
  return sum / times;
}

// Reading the value of the determined sensor and subtracting the value by the offset acquired
long HX711::get_value(int dout_pin, byte times, long offset) {
  long value;
  value = read(dout_pin) - offset;
  return value;
}

// Determines the offset for the specific sensor
long HX711::tare(int dout_pin, byte times) {
  long OFFSET;
  OFFSET = read_average(dout_pin, times);
  return OFFSET;
}
