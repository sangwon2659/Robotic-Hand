// SPI Library
#include <SPI.h>

// Having one encoder and two ADCs with pins 8, 9 and 10 respectively
// Having 8 channels for each ADC
// CS[0] => Encoder & Rest Tactile
const int CS[3] = {8,9,10};
const int n_channel = 5;
// All data to be put into val array
unsigned int val[n_channel+1]={0,0,0,0,0,0};
int Hz = 1000;

// Variables for read_Encoder
// Declaring stack variable for moving average operation
const int size_of_stack = 3;
long stack[size_of_stack];
// For masking the bits for encoder results
word mask_results = 0b0011111111111111;

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
  SPI.begin();
  SPISettings spiSettings(20000000, MSBFIRST, SPI_MODE1);
  SPI.beginTransaction(spiSettings);
  t1 = micros();
  for(int i = 0; i < size_of_stack; ++i)
  {
    stack[i] = 0;
  }
}

void loop()
{
  t2 = micros();
  // Only to operate with the given frequency
  if(t2 - t1 > Hz)
  {
    for (int k = 0; k < n_channel; k++)
    {
      // Putting all the values of the channels upto the second last of the val array
      val[k] = read_ADC(2,k+2);
    }

    // Encoder
    // Variables for the moving average operation
    long sum = 0;
    long avg;

    // Reading the value from the read_Encoder function and masking the results
    val[n_channel] = read_Encoder(CS[0]) & mask_results;

    // Summing all the values(except for the one in the current loop) stored in the stack
    for(int i = 0; i < size_of_stack; ++i)
    {
      sum += (long)stack[i];
    }
    // Adding the value of the one in the current loop
    sum += val[n_channel];
    
    // Averaging the result
    avg = (long)sum/size_of_stack;

    // Moving the latest value(the one before the one in the current loop) of the encoder to the second latest slot
    for(int i = 1; i < size_of_stack; ++i)
    {
      stack[i] = stack[i-1];
    }

    // Putting in the one in the current loop as the latest value in the stack
    stack[0] = val[n_channel];
    val[n_channel] = avg*(36000.0/16384.0);
   
    // Sending 2 bytes of data for each channel so n_channel*2 in total
    Serial.write((byte*)val,(n_channel+1)*2);
    // Sending one more byte of data for discretion purposes
    // So has to receive n_channel*2 + 1 bytes of data from the receiver
    Serial.write('\n');

    //For Testing
    //Serial.print(val[3]);
    //Serial.print("\t");
    //Serial.print(val[4]);
    //Serial.print("\t");
    //Serial.println(val[5]);
    //delay(25);
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

long read_Encoder(int cs) {
  // Declaring incoming byte from SPI
  byte inByte = 0x00;  
  byte inByte2 = 0x00;
  long value = 0;

  // Beginning Transmission
  digitalWrite(cs, LOW);
  delayMicroseconds(10);

  // SPI.transfer used for sending and receiving data
  // No need to give any data to the slave so 0x00
  // Suppose to give all 1s but substituted doing this by connecting the MOSI with 3.3V
  inByte = SPI.transfer(0x00);
  inByte2 = SPI.transfer(0x00);
 
  // Combining the byte with the previous one:
  // | compares each bit and gives 1 if 1 is in either data
  // << 8 moves the bits to the left by 8 spaces
  value = inByte << 8 | inByte2;
 
  // Giving high voltage for deacctivation
  digitalWrite(cs, HIGH);
  delayMicroseconds(10);
  return value;
}
