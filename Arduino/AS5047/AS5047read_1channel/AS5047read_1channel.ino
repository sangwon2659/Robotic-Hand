// SPI Library
#include <SPI.h>

// Declare int array CS for chip select
const int cs = 2;
const int size_of_stack = 7;
long stack[size_of_stack];
long value;

// 0b -> Binary & 0x -> Hexa
// This grabs the returned data without the read/write or parity bits
word mask_results = 0b0011111111111111;

void setup() {
  // Arduino data transmission
  Serial.begin(115200);

  // Beginning SPI communication
  SPI.begin();
  
  // Transmission with Most Significant Bit first and with mode_1
  // ClockDivider can be considered as the transmission or receiving rate
  SPI.setBitOrder(MSBFIRST);  
  SPI.setDataMode(SPI_MODE1);  
  SPI.setClockDivider(10);

  // Different spiSettings set when multi-channel but only one in this case so 주석
  // SPISettings spiSettings(1000000, MSBFIRST, SPI_MODE1);
  // SPI.beginTransaction(spiSettings);

  // Declaring pinmode and giving high voltage for deactivation
  pinMode(cs,OUTPUT);
  digitalWrite(cs,HIGH);

  // Defining the stack for averaging purposes
  for(int i = 0; i < size_of_stack; ++i)
  {
    stack[i] = 0;
  }
}

long readRegister(int cs) {
  // Incoming byte from SPI
  byte inByte = 0x00;   
  byte inByte2 = 0x00;
  long result = 0;

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
  result = inByte << 8 | inByte2;
  
  // Giving high voltage for deacctivation
  digitalWrite(cs, HIGH);
  delayMicroseconds(10);
  return result;
}

void loop() {
    long sum = 0;
    long avg;

    // Only taking the useful data
    value = readRegister(cs) & mask_results;
    for(int i = 0; i < size_of_stack-1; ++i)
    {
      sum += (long)stack[i];
    }
    sum += value;
    // Averaging the result
    avg = (long)sum/size_of_stack;

    // Converting the data to degrees
    Serial.println(avg*360.0/16363.0);

    // Making a new set of stack
    for(int i = 1; i < size_of_stack; ++i)
    {
      stack[i] = stack[i-1];
    }
    stack[0] = value;
    delayMicroseconds(10);
}
