// ifndef -> if not defined
// This is done to prevent re-declaration, which results in error
#ifndef HX711_h
#define HX711_h

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

class HX711
{
  // Variables to be  used throughout the cpp file
  private:
    byte PD_SCK;  
    byte GAIN;    
    int N_CHANNEL;
    float SCALE = 1;  

  public:
    HX711(int n_channel, int dout[], byte pd_sck, byte gain = 128);

    HX711();

    virtual ~HX711();

    // Allows to set the pins and gain later than in the constructor
    void begin(int n_channel, int dout[], byte pd_sck, byte gain = 128);

    bool is_ready(int dout_pin);
    
    // Setting the gain value to be used in the read function
    // Set as 1 as default
    void set_gain(int dout[], byte gain = 128);

    long read(int dout_pin);
    // Reading the average value of the sensor every few iterations
    // Used only for the offset
    long read_average(int dout_pin, byte times = 10);

    // Reading the value of the determined sensor and subtracting the value by the offset acquired
    long get_value(int dout_pin, byte times, long offset);

    // Determines the offset for the specific sensor
    long tare(int dout_pin, byte times = 50);

};

#endif /* HX711_h */
