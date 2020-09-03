// Calling the header file
#include "HX711.h" 
// SCK Number
#define CLK  2
// 4 FSS Sensors
#define n_channel 4
int DOUT[n_channel] = {3,4,5,6};
// Number of iterations for averaging
int data_stack = 1;
// Number of iterations for computing the offset
int offset_stack = 50;
long offset[n_channel];
long value[n_channel];
// Giving initial variables to the cpp file
// Name set as FSS
HX711 FSS(n_channel, DOUT, CLK);

void setup() {
  Serial.begin(115200);
  // Putting in values for each slot in the offset array 
  for(int i=0; i<n_channel; i++){
    // FSS.tare gives out the offset for DOUT[x]
    offset[i] = FSS.tare(DOUT[i], offset_stack);
  }
}

void loop() {
  // Putting in values for each slot in the value array
  for(int i=0; i<n_channel; i++){
    // FSS.get_value gives out the value of the sensor for DOUT[x]
    value[i] = FSS.get_value(DOUT[i], data_stack, offset[i]);
  }
  // Printing or Plotting the values of the sensor
  for(int i=0; i<(n_channel-1);i++){
    Serial.print(value[i]);
    Serial.print("\t");
}  
  Serial.println(value[n_channel-1]);
}
