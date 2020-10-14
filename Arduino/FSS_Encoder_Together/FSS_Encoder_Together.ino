#include "HX711.h"
#include <SPI.h>
#define calibration_factor 256.00

// The sensor designed to send pulses to MISO-like at 80Hz
// When this signal occurs at the same time as the MOSI-like for data request,
// The data seen as a noise
// In order to avoid this, interrupt pins have been used
// DOUT from sensor was connected to both the interrupt pin and the MISO-like pin

int numPin = 5;
// Declaration for FSS sensors
// intPin for interrupt pins in Arduino mega
int intPin[4] = {2,3,18,19};
// MOSI-like pins not clk pins to be exact
int clkPin[4] = {4,6,8,10};
// MISO-like pins
int doutPin[4] = {5,7,9,11};
int32_t value[5] = {0};
// For offset setting purposes
int offsetBuffer[4] = {0};
double sumBuffer[4] = {0};
unsigned long time;

// 객체 선언
HX711 scale_0(doutPin[0], clkPin[0]);
HX711 scale_1(doutPin[1], clkPin[1]);
HX711 scale_2(doutPin[2], clkPin[2]);
HX711 scale_3(doutPin[3], clkPin[3]);

// Declaration for the encoder
const int cs = 12;
long value_;

word mask_results = 0b0011111111111111;

// Functions to be initiated with the interrupts
void read_value_0(){
  // Putting in data into the buffer for offset setting
  if(offsetBuffer[0] < 20){
    sumBuffer[0] += scale_0.get_value();
    offsetBuffer[0] += 1;
  }
  else if(offsetBuffer[0] == 20){
    sumBuffer[0] = sumBuffer[0]/20;
    offsetBuffer[0] += 1;
  }
  // digitalRead == LOW for stable read of FALLING
  // Why sumBuffer has to be *2 has yet been identified
  else if(offsetBuffer[0] == 21 && digitalRead(doutPin[0]) == LOW){
    value[0] = scale_0.get_value() - 2*(sumBuffer[0]);
  }
}

void read_value_1(){
  if(offsetBuffer[1] < 20){
    sumBuffer[1] += scale_1.get_value();
    offsetBuffer[1] += 1;
  }
  else if(offsetBuffer[1] == 20){
    sumBuffer[1] = sumBuffer[1]/20;
    offsetBuffer[1] += 1;
  }
  else if(offsetBuffer[1] == 21 && digitalRead(doutPin[1]) == LOW){
    value[1] = scale_1.get_value() - 2*(sumBuffer[1]);
  }
}

void read_value_2(){
  if(offsetBuffer[2] < 20){
    sumBuffer[2] += scale_2.get_value();
    offsetBuffer[2] += 1;
  }
  else if(offsetBuffer[2] == 20){
    sumBuffer[2] = sumBuffer[2]/20;
    offsetBuffer[2] += 1;
  }
  else if(offsetBuffer[2] == 21 && digitalRead(doutPin[2]) == LOW){
    value[2] = scale_2.get_value() - 2*(sumBuffer[2]);
  }
}

void read_value_3(){
  if(offsetBuffer[3] < 20){
    sumBuffer[3] += scale_3.get_value();
    offsetBuffer[3] += 1;
  }
  else if(offsetBuffer[3] == 20){
    sumBuffer[3] = sumBuffer[3]/20;
    offsetBuffer[3] += 1;
  }
  else if(offsetBuffer[3] == 21 && digitalRead(doutPin[3]) == LOW){
    value[3] = scale_3.get_value() - 2*(sumBuffer[3]);
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

void setup() {
  Serial.begin(115200);

  // Setup for the FSS sensors
  // Interrupt pins set mode as input over here where the MISO-like pins declared in .cpp
  for(int i = 0; i < numPin; i++){
    pinMode(intPin[i], INPUT);
  }

  // Setting scale and offset
  // Offset not used in .cpp but offset set instead in the interrupt functions
  scale_0.set_scale(calibration_factor);
  scale_0.tare();
  Serial.println(scale_0.OFFSET);
  scale_1.set_scale(calibration_factor);
  scale_1.tare();
  Serial.println(scale_1.OFFSET);
  scale_2.set_scale(calibration_factor);
  scale_2.tare();
  Serial.println(scale_2.OFFSET);
  scale_3.set_scale(calibration_factor);
  scale_3.tare();
  Serial.println(scale_3.OFFSET);

  // Declaring the intpins as interrupt pins and declaring the initiating functions
  attachInterrupt(digitalPinToInterrupt(intPin[0]), read_value_0, FALLING);
  attachInterrupt(digitalPinToInterrupt(intPin[1]), read_value_1, FALLING);
  attachInterrupt(digitalPinToInterrupt(intPin[2]), read_value_2, FALLING);
  attachInterrupt(digitalPinToInterrupt(intPin[3]), read_value_3, FALLING);

  // Setup for the encoder
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
}

void loop() {

  time = millis();

  if(time-millis() > 20){
    long sum = 0;
    long avg;

    // Only taking the useful data
    value_ = readRegister(cs) & mask_results;
    value_ = (long)value_*(36000.0/16384);

    value[numPin-1] = value_;

    for(int i=0;i<numPin-1;i++){
      Serial.print(value[i]);
      Serial.print("\t");
    }
    Serial.println(value[numPin-1]);
    

    //Serial.write((byte*)value,numPin*4);
    // Serial.write("\t") for Serial.print on monitor screen
    // Has to be "\n" for it to be read as single lines on the ROS segment
    //Serial.write("\n");
  }
  }

//   Printing the value array after obtaining the values using the interrupt functions
//   for(int i = 0; i < (numPin-1); i++){
//     Serial.print(value[i]);
//     Serial.print("\t");
//   }
//   Serial.println(value[numPin-1]);


