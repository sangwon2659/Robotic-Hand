#include <Arduino.h>
#include "HX711.h"

#if ARDUINO_VERSION <= 106
    // "yield" is not implemented as noop in older Arduino Core releases, so let's define it.
    // See also: https://stackoverflow.com/questions/34497758/what-is-the-secret-of-the-arduino-yieldfunction/34498165#34498165
    void yield(void) {};
#endif

HX711::HX711(int n_channel, int dout[], byte pd_sck, byte gain) {
  begin(n_channel, dout, pd_sck, gain);
}

HX711::HX711() {
}

HX711::~HX711() {
}

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
  for (int i = 0; i < N_CHANNEL; i++) {
    read(dout[i]);
  }
}

long HX711::read(int dout_pin) {
  // wait for the chip to become ready
  while (!is_ready(dout_pin)) {
    // Will do nothing on Arduino but prevent resets of ESP8266 (Watchdog Issue)
    yield();
  }

  unsigned long value = 0;
  uint8_t data[3] = { 0 };
  uint8_t filler = 0x00;

  // pulse the clock pin 24 times to read the data
  data[2] = shiftIn(dout_pin, PD_SCK, MSBFIRST);
  data[1] = shiftIn(dout_pin, PD_SCK, MSBFIRST);
  data[0] = shiftIn(dout_pin, PD_SCK, MSBFIRST);

  // set the channel and the gain factor for the next reading using the clock pin
  for (unsigned int i = 0; i < GAIN; i++) {
    digitalWrite(PD_SCK, HIGH);
    digitalWrite(PD_SCK, LOW);
  }

  // Replicate the most significant bit to pad out a 32-bit signed integer
  if (data[2] & 0x80) {
    filler = 0xFF;
  } else {
    filler = 0x00;
  }

  // Construct a 32-bit signed integer
  value = ( static_cast<unsigned long>(filler) << 24
      | static_cast<unsigned long>(data[2]) << 16
      | static_cast<unsigned long>(data[1]) << 8
      | static_cast<unsigned long>(data[0]) );

  return static_cast<long>(value);
}

long HX711::read_average(int dout_pin, byte times) {
  long sum = 0;
  for (byte i = 0; i < times; i++) {
    sum += read(dout_pin);
    yield();
  }
  return sum / times;
}

long HX711::get_value(int dout_pin, byte times, long offset) {
  long value;
  value = read_average(dout_pin, times) - offset;
  return value;
}

long HX711::tare(int dout_pin, byte times) {
  long OFFSET;
  OFFSET = read_average(dout_pin, times);
  return OFFSET;
}
