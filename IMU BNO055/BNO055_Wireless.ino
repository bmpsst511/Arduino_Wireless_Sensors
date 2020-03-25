/** WIFI LIBRARY PART**/
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WiFiUDP.h>
/** WIFI LIBRARY PART**/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>

/** WIFI分享器設定 **/
const char* ssid ="你看不到我";
const char* password = "你以為我會打出來嗎...!?";
const char ip[]="192.168.XX.XXX"; //分享器給你Server的IP位址
/** WIFI分享器設定 **/

String PoseX, PoseY, PoseZ;

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055(55);

/** 指定Port 且設置為客戶端**/
WiFiServer server(27);
WiFiUDP Client;
/** 指定Port 且設置為客戶端**/


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

/*Get a new sensor event */
  sensors_event_t event;
  bno.getEvent(&event);
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
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

  Serial.print((int)euler.x());Serial.print(";");
  Serial.print((int)euler.y());Serial.print(";");
  Serial.print((int)euler.z());Serial.print("\t");
  PoseX = (int)euler.x();
  PoseY = (int)euler.y();
  PoseZ = (int)euler.z();

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
  
  /**開始發送資料給Server端 **/
    // Send the distance to the client, along with a break to separate our messages
    Client.beginPacket(ip,27); //前面指定的Port
    Client.println(PoseX+";"+PoseY+";"+PoseZ);
    Client.endPacket();
    delay(BNO055_SAMPLERATE_DELAY_MS);
  /**開始發送資料給Server端 **/

  }
