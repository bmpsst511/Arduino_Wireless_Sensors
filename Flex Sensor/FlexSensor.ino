const int FLEX_PIN = A0; // 將類比腳位AO設為讀取腳位Pin connected to voltage divider output 
// 使用220歐姆電阻，並使用電表量測正確值 220k resistor, and enter them below: 
const float R_DIV = 246.0; 
// 3.3v的供電給感測器，並使用電表量測正確值 Measured voltage of Ardunio 3.3V line 
const float VCC = 3.2; 
//FlexSensor在180度彎曲下的電阻值，並使用電表量測正確值 accurately calculate bend degree. 
const float STRAIGHT_RESISTANCE = 9000.0; 
// FlexSensor在無彎曲下的電阻值，並使用電表量測正確值 resistance when straight 
const float BEND_RESISTANCE = 22000.0; 
//宣告濾波以符點數
float Filter_Value;
void setup()  
{   
Serial.begin(9600);   
pinMode(FLEX_PIN, INPUT); 
} 

void loop()  
{   
  // Read the ADC, and calculate voltage and resistance from it
  Filter_Value = Filter();       // 獲得濾波器輸出值   
  float flexV = Filter_Value * VCC / 1023.0;   
  float flexR = R_DIV * (VCC / flexV - 1.0);   
  //Serial.println("Resistance: " + String(flexR) + " ohms");   
  // Use the calculated resistance to estimate the sensor's   
  // bend angle:   
  float angle = map(flexR, STRAIGHT_RESISTANCE, BEND_RESISTANCE, 0, 90.0);   
  Serial.println("Bend: " + String(angle) + " degrees");   
  //Serial.println();   
  delay(50); 
  }
/*
A、名稱：加權遞推平均濾波法
B、方法：
    是針對遞推平均濾波法的改進．即不同時刻的數據加以不同的權重
    通常是，越靠近現時刻的數據，權取得越大。
    給予新採樣值得權係數越大，則靈敏度越高，但信號平滑度越低。
C、優點：
    適用於有效大純滯後時間常數的對象，和採樣周期較短的系統。
D、缺點：
    對於純滯後時間長數較小、採樣周數較長、變化緩慢的信號；
    不能迅速反應系統當前所受干擾的嚴重程度，濾波效果差。
*/
// 加权递推平均滤波法
#define FILTER_N 12
int coe[FILTER_N] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};    // 加權係數表
int sum_coe = 1 + 2 + 3 + 4 + 5 + 6 + 7 + 8 + 9 + 10 + 11 + 12; // 加權係數和
int filter_buf[FILTER_N + 1];
float Filter() {
  int i;
  float filter_sum = 0;
  filter_buf[FILTER_N] = analogRead(FLEX_PIN);
  for(i = 0; i < FILTER_N; i++) {
    filter_buf[i] = filter_buf[i + 1]; // 所有數據左移，低位仍掉
    filter_sum += filter_buf[i] * coe[i];
  }
  filter_sum /= sum_coe;
  return filter_sum;
}
