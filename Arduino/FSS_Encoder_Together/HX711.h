#ifndef HX711_h
#define HX711_h

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

class HX711
{
  public:
    byte PD_SCK;
    byte DOUT;
    byte GAIN;
    long OFFSET = 0;
    float SCALE = 1;

    HX711(byte dout, byte pd_sck, byte gain = 128);

    HX711();

    virtual ~HX711();
    void begin(byte dout, byte pd_sck, byte gain = 128);

    bool is_ready();

    void set_gain(byte gain = 128);

    long read();

    long read_average(byte times = 1);

    double get_value(byte times = 1);

    float get_units(byte times = 1);

    void tare(byte times = 10);

    void set_scale(float scale = 1.f);

    float get_scale();

    void set_offset(long offset = 0);

    long get_offset();

    void power_down();

    void power_up();
};

#endif /* HX711_h */
