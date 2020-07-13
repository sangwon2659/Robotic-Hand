// SPI Library
#include <SPI.h>

// Having two ADCs with pins 9 and 10
// Having 7 channels for each ADC
const int CS[2] = {9,10};
const int n_channel = 3;
unsigned int val[n_channel]={0,0,0};
int Hz = 1000;
// Declaring time variables for the sampling frequency
unsigned long t1;
unsigned long t2;

void setup()
{
  Serial.begin(115200);
  // Setting the pinMode for each CS and having them as high
  for(int i = 0; i < sizeof(CS); i++)
  {
    pinMode(CS[i], OUTPUT);
    digitalWrite(CS[i],HIGH); 
  }
  // Starting the SPI communication
  // Mode_0 as default
  SPI.begin();
  // ClockDivider is the frequency of the clock
  // Does not matter to much
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  t1 = micros();
}

void loop()
{ 
  t2 = micros();
  // Only to operate with the given frequency
  if(t2 - t1 > Hz)
  {
    for (int k = 0; k < n_channel; k++)
    {
      // Putting all the values of the channels into the val array
      val[k] = read_ADC(1,k+2);
    }
    // Sending 2 bytes of data for each channel so n_channel*2 in total
    Serial.write((byte*)val,n_channel*2);
    // Sending one more byte of data
    // So has to receive n_channel*2 + 1 bytes of data from the receiver
    Serial.write('\n');
  }
}

int read_ADC(int n_CS, int channel)
{
  // Declaring the result variable
  // uint16_t -> unsigned short that is equal to 2 bytes
  int result;
  // Declaring byte format according to the datasheet
  byte firstTransmissionByte = B00000111;
  byte firstTransmissionByte_ = B00000110;
  byte channelConfig = channel << 6;

  // Configuring the transmission bytes according to the channel number
  // First two bytes send data about the desired channel
  // Last byte does not matter in its data information; therefore 0x00
  // Digital write low to let the sensor know the beginning of communication
  digitalWrite(CS[n_CS],LOW);
  if(channel>3)
    SPI.transfer(firstTransmissionByte);
  else
    SPI.transfer(firstTransmissionByte_);

  byte MSB = SPI.transfer(channelConfig);
  byte LSB = SPI.transfer(0x00);

  // Digital write high to end the communication
  digitalWrite(CS[n_CS],HIGH);

  // Last 4 bits of the second byte contains the sensor data information
  // So mask the MSB
  // The overall result is the last 4 bits of the second byte and the third byte
  // So move the MSB to the left by 8 bits and combine it with the LSB
  MSB = MSB & B00001111;
  result = MSB<<8 | LSB;

  return result;
}
