#include <SPI.h>
#include "HX711.h" //HX711로드셀 엠프 관련함수 호출

#define calibration_factor 256.00 // 로드셀 스케일 값 선언
#define CLK  2  //엠프 클락 핀 넘버 
#define CLK_2 3
#define CLK_3 4
#define CLK_4 5
#define DOUT  6 //엠프 데이터 아웃 핀 넘버 선언
#define DOUT_2 7
#define DOUT_3 8
#define DOUT_4 9

HX711 scale(DOUT, CLK); //엠프 핀 선언 
HX711 scale2(DOUT_2, CLK_2);
HX711 scale3(DOUT_3, CLK_3);
HX711 scale4(DOUT_4, CLK_4);

// Encoder Constants
const int cs = 10;
const int size_of_stack = 3;
long stack[size_of_stack];
word mask_results = 0b0011111111111111;
unsigned int result = 0;

int32_t value[5] = {0};

void setup() {
  Serial.begin(115200);  // 시리얼 통신 개방
  Serial.println("HX711 scale TEST");  
  scale.set_scale(calibration_factor);  //스케일 지정
  scale2.set_scale(calibration_factor);
  scale3.set_scale(calibration_factor);
  scale4.set_scale(calibration_factor);
  scale.tare(); 
  scale2.tare();
  scale3.tare();
  scale4.tare(); //스케일 설정
  Serial.println("Readings:");
  pinMode(cs, OUTPUT);
  digitalWrite(cs, HIGH);
  SPI.begin();
  SPISettings spiSettings(20000000, MSBFIRST, SPI_MODE1);
  SPI.beginTransaction(spiSettings);
  for(int i=0; i<size_of_stack; i++){
    stack[i] = 0;
  }
}

void loop() {
  long sum = 0;
  long avg = 0;
  
  value[0] = scale.get_value(1);
  value[1] = scale2.get_value(1);
  value[2] = scale3.get_value(1);
  value[3] = scale4.get_value(1);
  
  result = read_Encoder(cs) & mask_results;
  for(int i=0; i < size_of_stack; i++){
    sum += (long)stack[i];
  }
  sum += result;
  avg = (long)sum/size_of_stack;

  for(int i = 1; i < size_of_stack; i++){
    stack[i] = stack[i-1];
  }
  stack[0] = result;
  value[4] = avg*(36000.0/16384.0);

  for(int i=0; i<4; i++){
    Serial.print(value[i]);
    Serial.print("\t");
  }
  Serial.println(value[4]);
 
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
