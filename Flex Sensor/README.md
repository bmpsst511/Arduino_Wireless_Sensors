---
title: 'GitHub Arduino Sensor Flex Sensor Readme'
disqus: hackmd
---

## 彎曲感測器（Flex Sensor）
---

[**HACKMD Link**](https://hackmd.io/@J-T-LEE/Flex-Sensor)    
[**GITHUB Link**](https://github.com/bmpsst511/Arduino_Wireless_Sensors/tree/master/Flex%20Sensor) 

彎曲感測器 Flex Sensor：主要透過其表面貼附的可導電材質(如可導電墨水或石墨稀等複合材料)外力使之彎曲時可改變其電阻值。市面上常看到的有長度2.2~inch~與4.5~inch~，不同的長度會有不同的阻值，因此在使用前須先量測其在<font color=red>**不彎曲狀態**</font>下的阻值與<font color=red>**接近90度彎曲**</font>狀態下的阻值，並導入置程式算法內得出較可靠得可讀式角度數據。

---
實作
---

### Fritzing 元件導入檔鏈結
[NodeMCU](https://github.com/roman-minyaylov/nodemcu-v3-fritzing)

本次實驗使用的為4.5inch的彎曲感測器    
哪裡買:    
[彎曲感測器2.2英吋](暫定賣場連結)    
[彎曲感測器4.5英吋](暫定賣場連結)

![彎曲感測器接線圖](https://i.imgur.com/BtmAUhp.png)



上圖為**NodeMCU V3**與**Flex Sensor**在軟體**Fritzing**內所繪的接線圖

### 前置作業
---
#### 測量電阻Measuring resistances

4.5英吋在不彎曲時的阻值約 <font color=red>9000ohm</font>

4.5英吋在彎曲接近90度時的阻值約 <font color=red>22000ohm</font>

#### 濾波應用

由於當讀取彎曲感測器值的時候，在Arduino序列繪圖家的顯示值十分不穩定，因此加了一個濾波算法去平滑得到的曲線如下

**加權遞推平均濾波法**：
是針對遞推平均濾波法的改進．即不同時刻的數據加以不同的權重，通常是越靠近現時刻的數據，權取得越大。給予新採樣值的權係數越大，則靈敏度越高，但信號平滑度越低。

**優點**：
    適用於有效大純滯後時間常數的對象，和採樣周期較短的系統。
**缺點**：
    對於純滯後時間長數較小、採樣周數較長、變化緩慢的信號；
    不能迅速反應系統當前所受干擾的嚴重程度，濾波效果差。

---
### 程式碼
---

**Code Part**↓
```clike=
const int FLEX_PIN = A0; // 將類比腳位AO設為讀取腳位Pin connected to voltage divider output 
// 使用220歐姆電阻，並使用電表量測正確值 220 resistor, and enter them below: 
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
```
---

無線傳輸 Wireless Transmission
---
NodeMCU為客戶端，使用UDP架構來傳輸慣性感測器資料。    
NodeMCU set as a client and carries out the wireless data transmission with UDP protocol.

```clike=
/** WIFI LIBRARY PART**/
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WiFiUDP.h>
/** WIFI LIBRARY PART**/

/** WIFI分享器設定 **/
const char* ssid ="你看不到我";
const char* password = "你以為我會打出來嗎...!?";
const char ip[]="192.168.XX.XXX"; //分享器給你Server的IP位址
/** WIFI分享器設定 **/

/** 指定Port 且設置為客戶端**/
WiFiServer server(27);
WiFiUDP Client;
/** 指定Port 且設置為客戶端**/

const int FLEX_PIN = A0; // 將類比腳位AO設為讀取腳位Pin connected to voltage divider output 
// 使用220歐姆電阻，並使用電表量測正確值 220 resistor, and enter them below: 
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

  /** 上電後執行WIFI連線與顯示相關資訊**/
  WiFi.mode(WIFI_STA); 
  Serial.println("Orientation Sensor Test"); Serial.println("");
  WiFi.begin(ssid,password);
  Serial.println("Connecting");
 
  while(WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
 
  Serial.print("Connected to "); 
  Serial.println(ssid);
  Serial.print("IP Address: "); 
  Serial.println(WiFi.localIP());
 
  // Start the UDP client
  Client.begin(27);
  /** 上電後執行WIFI連線與顯示相關資訊**/
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
  
    /**開始發送資料給Server端 **/
    // Send the distance to the client, along with a break to separate our messages
    Client.beginPacket(ip,27); //前面指定的Port
    Client.println(angle);
    Client.endPacket();
    delay(50);
  /**開始發送資料給Server端**/ 
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
```

---
Reference

[Arduino十大濾波器算法程序大全](https://www.geek-workshop.com/thread-7694-1-1.html)

[Flex Sensor 與 Arduino ](https://www.instructables.com/id/How-to-use-a-Flex-Sensor-Arduino-Tutorial/)


###### tags: `GITHUB`
