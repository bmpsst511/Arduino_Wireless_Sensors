---
title: 'GitHub Arduino Sensor IMU-BNO055 Readme'
disqus: hackmd
---
## 九軸慣性感測器（Inertial measurement unit IMU）
---
**HACKMD Link** : https://hackmd.io/@J-T-LEE/IMU-BNO055

九軸慣性感測器主要構成有：三軸加速度計、三軸MEMS陀螺儀、三軸磁力計。

**三軸MEMS陀螺儀**：同時測定6個方向的位置，移動軌跡，加速。單軸的只能測量一個方向的量，也就是一個系統需要三個陀螺儀，而3軸的一個就能替代三個單軸的。 3軸的體積小、重量輕、結構簡單、可靠性好，是MEMS陀螺的發展趨勢。

**三軸壓電式加速度感測器**： 加速度計基本原理就是高中學過的力學知識，力和加速度的方向一致。
可以想像一個小球裝在一個感應的小盒子裡。水平放置的時候，重力使得小球與下表面接觸，下表面會產生一個向上的力，與重力相抵消，則產生一個向上的加速度，大小為G。我們假設IMU的測量範圍是正負16G，則測得向上加速度為16個G的時候，真實物體向上的加速度只有15個G。反之，向下測得有16個G的時候，真實加速度有17個G。

**三軸磁力計**： 磁力計是把磁場、電流、應力應變、溫度、光等外界因素引起敏感元件磁性能轉換成電信號，以這種方式來檢測相應物理量的器件。磁力計廣泛用於現代工業和電子產品中以感應磁場強度來測量電流、位置、方向等物理參數。在現有技術中，有許多不同類型的傳感器用於測量磁場和其他參數。

**轉換概念**：在n系中，加速度計輸出為，經過bCn（用四元數表示的轉換矩陣）轉換之後到b系中的值為；在b系中，加速度計的測量值為，現在和均表示在b系中的豎直向下的向量，由此，我們來做向量積（叉積），得到誤差，利用這個誤差來修正bCn矩陣，於是四元數就在這樣一個過程中被修正了。但是，由於加速度計無法感知z軸上的旋轉運動，所以還需要用地磁計來進一步補償。
加速度計在靜止時測量的是重力加速度，是有大小和方向的；同理，地磁計同樣測量的是地球磁場的大小和方向，只不過這個方向不再是豎直向下，而是與x軸（或者y軸）呈一個角度，與z軸呈一個角度。記作，假設x軸對準北邊，所以by=0，即。倘若知道bx和bz的精確值，那麼就可以採用和加速度計一樣的修正方法來修正。只不過在加速度計中，在n系中的參考向量是，變成了地磁計的。如果我們知道bx和bz的精確值，那麼就可以擺脫掉加速度計的補償，直接用地磁計和陀螺儀進行姿態解算，但是你看過誰只用陀螺儀和地磁計進行姿態解算嗎？沒有，因為沒人會去測量當地的地磁場相對於東北天坐標的夾角，也就是bx和bz（插曲：關於這個bx和bz的理解：可以對比重力加速度的理解，就像vx vy vz似的，因為在每一處的歸一化以後的重力加速度都是0 0 1然後旋轉到機體坐標系，而地球每一處的地磁大小都不一樣的，不能像重力加速度那樣直接旋轉得到了，只能用磁力計測量到的數據去強制擬合。）。那麼現在怎麼辦？前面已經講了，姿態解算就是求解旋轉矩陣，這個矩陣的作用就是將b系和n正確的轉化直到重合。現在我們假設nCb旋轉矩陣是經過加速度計校正後的矩陣，當某個確定的向量（b系中）經過這個矩陣旋轉之後（到n系），這兩個坐標系在XOY平面上重合（參考DCM [IMU:Theory](http://www.ent.mrt.ac.lk/~rohan/teaching/EN4562/LectureNotes/Lec%203%20IMU%20Theory.pdf)的Drift cancellation部分），只是在z軸旋轉上會存在一個偏航角的誤差。

 實作
---

### Fritzing 元件導入檔鏈結
[Adafruit](https://github.com/adafruit/Fritzing-Library) and
[NodeMCU](https://github.com/roman-minyaylov/nodemcu-v3-fritzing)

感測器有別於一般常使用的**MPU6050,9150,9255**等，採用的是**Adafruit BNO055**
開發版則使用**NodeMCU V3**達到資料無線傳輸的目的。

![九軸接線圖](https://i.imgur.com/teqKlCr.png =400x250)

上圖為**NodeMCU V3**與**Adafruit BNO055**在軟體**Fritzing**內所繪的接線圖

### 程式碼

#### Arduino Adafruit Library安裝
[Adafruit_Sensor](https://github.com/adafruit/Adafruit_Sensor)

[Adafruit_BNO055](https://github.com/adafruit/Adafruit_BNO055)

```
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

String PoseX, PoseY, PoseZ, On;
/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055(55);

/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx"); 
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}
 
void setup() {
  Serial.begin(115200);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  pinMode(TouchSensor, INPUT);

   /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

    /* Use external crystal for better accuracy */
  bno.setExtCrystalUse(true);
   
  /* Display some basic information on this sensor */
  displaySensorDetails();
  
}
 
void loop() {

/* Get a new sensor event */
  sensors_event_t event;
  bno.getEvent(&event);
  imu::Vector<3> linAcc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  /* Board layout:
         +----------+
         |         *| RST   PITCH  ROLL  HEADING
     ADR |*        *| SCL
     INT |*        *| SDA     ^            /->
     PS1 |*        *| GND     |            |
     PS0 |*        *| 3VO     Y    Z-->    \-X
         |         *| VIN
         +----------+
  */


  /* The processing sketch expects data as roll, pitch, heading */
  //Serial.print(F("Orientation: "));
  Serial.print((float)event.orientation.x);Serial.print(";");
 // Serial.print(F(" "));
  Serial.print((float)event.orientation.y);Serial.print(";");
 // Serial.print(F(" "));
  Serial.print((float)event.orientation.z);Serial.print("\t");
 // Serial.println(F(""));
  PoseX = (float)event.orientation.x;
    PoseY = (float)event.orientation.y;
      PoseZ = (float)event.orientation.z;
  /* Also send calibration data for each sensor. */
  uint8_t sys, gyro, accel, mag = 3;
  bno.getCalibration(&sys, &gyro, &accel, &mag);
 /* Serial.print(F("Calibration: "));
  Serial.print(sys, DEC);
  Serial.print(F(" "));
  Serial.print(gyro, DEC);
  Serial.print(F(" "));
  Serial.print(accel, DEC);
  Serial.print(F(" "));
  Serial.println(mag, DEC);*/
  Serial.print("\r\n");
  delay(BNO055_SAMPLERATE_DELAY_MS);
  }

```



###### tags: `Arduino Sensor` `GITHUB`
