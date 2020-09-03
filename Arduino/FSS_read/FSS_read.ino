#include "HX711.h" //HX711로드셀 엠프 관련함수 호출
#define CLK  2  //엠프 클락 핀 넘버
#define n_channel 4
int DOUT[n_channel] = {3,4,5,6}; //엠프 데이터 아웃 핀 넘버 선언
int data_stack = 1;
int offset_stack = 50;
long offset[n_channel];
long value[n_channel];
HX711 FSS(n_channel, DOUT, CLK); //엠프 핀 선언 

void setup() {
  Serial.begin(115200);  // 시리얼 통신 개방 
  for(int i=0; i<n_channel; i++){
    offset[i] = FSS.tare(DOUT[i], offset_stack);
  }
}

void loop() {
  for(int i=0; i<n_channel; i++){
    value[i] = FSS.get_value(DOUT[i], data_stack, offset[i]);
  }
  for(int i=0; i<(n_channel-1);i++){
    Serial.print(value[i]);
    Serial.print("\t");
}  
  Serial.println(value[n_channel-1]);
}
