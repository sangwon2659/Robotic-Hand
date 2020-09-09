//Use this instead of FSS_read (problem with SLK presumably)
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

int32_t value;

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
}

void loop() {
//  Serial.print("Reading: ");
  value = scale.get_value(1);
  Serial.print(value);
  Serial.print("\t");
  value = scale2.get_value(1);
  Serial.print(value);
  Serial.print("\t");  
  value = scale3.get_value(1);
  Serial.print(value);
  Serial.print("\t");  
  value = scale4.get_value(1);
  Serial.println(value);  //무제 출력 
//  Serial.print(" g"); //단위
//  Serial.println(); 
}
