---
title: 'GitHub Arduino Sensor Distance Sensor Readme'
disqus: hackmd
---

## 距離感測器（Distance Sensor）
---

[**HACKMD Link**](https://hackmd.io/@J-T-LEE/Distance-Sensor)    
[**GITHUB Link**](https://github.com/bmpsst511/Arduino_Wireless_Sensors/tree/master/IR%20Distance%20Sensor) 

紅外線距離感測器 Distance Sensor：紅外測距感測器由一對紅外信號發射器與接收二極體(IRED)組成，發射管發射特定頻率的紅外線訊號，接收管用來獲取特定頻率的紅外線訊號，當發射訊號的檢測方向遇到障礙物時，紅外線訊號反射回來被接收管接收，輸出的電壓對應檢測距離，經過處理之後，數位感測器通訊接口返回到開發板，即可利用紅外線的返回訊號來得出距離數據。


---
實作
---

### Fritzing 元件導入檔鏈結
[NodeMCU](https://github.com/roman-minyaylov/nodemcu-v3-fritzing)
[Sharp IR Sensor](https://fritzing.org/projects/gp2y0a21yk0f)

本次實驗使用的為夏普紅外線距離感測器家族成員之一，此型號可提供4～30cm的探測距離屬於紅外測距中的基礎類產品，同樣也擁有夏普在紅外距離探測領域一貫的品質。此感測器可以用於機器人的測距、避障以及高級的路徑規劃，是機器視覺及其應用領域的不錯選擇。
[距離感測器](http://riobotics-test.weebly.com/sharp-gp2d120.html)    

![距離感測器接線圖](https://i.imgur.com/nBla2yw.png)



上圖為**NodeMCU V3**與**IR Distance Sensor**在軟體**Fritzing**內所繪的接線圖

---
### 程式碼
---

**Code Part**↓
```clike=
// Arduino Code to measure distance with a Sharp GP2D12 sensor
// www.swanrobotics.com

int IR_SENSOR = 0; // 類比讀取腳設為A0Sensor is connected to the analog A0
int intSensorResult = 0; //Sensor result
float fltSensorCalc = 0; //Calculated value

void setup()
{
Serial.begin(9600); // Setup communication with computer to present results serial monitor
}

void loop()
{
// read the value from the ir sensor

intSensorResult = analogRead(IR_SENSOR); //Get sensor value
fltSensorCalc = (6787.0 / (intSensorResult - 3.0)) - 4.0; //Calculate distance in cm

Serial.print(fltSensorCalc); //Send distance to computer
Serial.println(" cm"); //Add cm to result
delay(200); //Wait
}
```




###### tags: `GITHUB`
