#include <Arduino.h>
#include "HX711.h"

#if ARDUINO_VERSION <= 106
void yield(void) {};
#endif

// Declaration of 객체 and initiating the begin function
HX711::HX711(byte dout, byte pd_sck, byte gain) {
  begin(dout, pd_sck, gain);
}

HX711::HX711() {
}

HX711::~HX711() {
}

// Setting pinmodes for each 객체
void HX711::begin(byte dout, byte pd_sck, byte gain) {
  PD_SCK = pd_sck;
  DOUT = dout;

  pinMode(PD_SCK, OUTPUT);
  pinMode(DOUT, INPUT);

  set_gain(gain);
}

// Original algorithm of identifying whether the sensor is read to send data
// When the sensor outputs low after sending a packet of data means it is ready to send another
bool HX711::is_ready() {
  delayMicroseconds(100);
  return digitalRead(DOUT) == LOW;
}

// Gain is about how many additional clocks(MOSI-like) to be sent after receiving 3 bytes of data
void HX711::set_gain(byte gain) {
  switch (gain) {
  case 128:   
    GAIN = 1;
    break;
  case 64:    
    GAIN = 3;
    break;
  case 32:    
    GAIN = 2;
    break;
  }

  digitalWrite(PD_SCK, LOW);
  read();
}


long HX711::read() {
  unsigned long value = 0;
  // An array of length 3 with 8 bits each
  uint8_t data[3] = { 0 };
  uint8_t filler = 0x00;

  // Giving 8 pulses to the MISO and acquiring 8 bits of data for each pulse
  // Saving the data to the data array
  data[2] = shiftIn(DOUT, PD_SCK, MSBFIRST);
  data[1] = shiftIn(DOUT, PD_SCK, MSBFIRST);
  data[0] = shiftIn(DOUT, PD_SCK, MSBFIRST);

  // Sending additional pulse to the sensor according to the gain value
  for (unsigned int i = 0; i < GAIN; i++) {
    digitalWrite(PD_SCK, HIGH);
    digitalWrite(PD_SCK, LOW);
  }

  // Converting the 3 bytes data to 4 bytes data with the sign information
  // Using data[2] & 1000 0000 for identifying the sign
  if (data[2] & 0x80) {
    filler = 0xFF;
  }
  else {
    filler = 0x00;
  }

  // Putting in the identified sign to the first 8 bits and filling in the rest with the data array
  value = (static_cast<unsigned long>(filler) << 24
    | static_cast<unsigned long>(data[2]) << 16
    | static_cast<unsigned long>(data[1]) << 8
    | static_cast<unsigned long>(data[0]));

  return static_cast<long>(value);
}

long HX711::read_average(byte times) {
  long sum = 0;
  for (byte i = 0; i < times; i++) {
    sum += read();
    yield();
  }
  return sum / times;
}

// Deleted the - offset due to some occurence yet identified
double HX711::get_value(byte times) {
  return read_average(times);
}

float HX711::get_units(byte times) {
  return get_value(times) / SCALE;
}

void HX711::tare(byte times) {
  double sum = read_average(times);
  set_offset(sum);
}

void HX711::set_scale(float scale) {
  SCALE = scale;
}

float HX711::get_scale() {
  return SCALE;
}

void HX711::set_offset(long offset) {
  OFFSET = offset;
}

long HX711::get_offset() {
  return OFFSET;
}

void HX711::power_down() {
  digitalWrite(PD_SCK, LOW);
  digitalWrite(PD_SCK, HIGH);
}

void HX711::power_up() {
  digitalWrite(PD_SCK, LOW);
}
