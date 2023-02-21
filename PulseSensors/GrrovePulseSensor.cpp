#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WiFiUDP.h>
#include <Wire.h>

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

const char *ssid =  "Drink";     // replace with your wifi ssid and wpa2 key
const char *pass =  "03487150";

const char ip[]="192.168.0.101"; //分享器給你Server的IP位址

String PoseX, PoseY, PoseZ;
int iTestNum = 0;

/** 指定Port 且設置為客戶端**/
WiFiServer server(27);
WiFiUDP Client;
//WiFiClient client;
/** 指定Port 且設置為客戶端**/
 
void setup() 
{
  Serial.begin(9600); /* begin serial for debug */

  //delay(10);

  /* Serial.println("Connecting to ");
  Serial.println(ssid); 

  WiFi.begin(ssid, pass); 
  int iCount=0;
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(500);
    Serial.print(".");

    if(iCount>100) {
      Serial.print("retry");
      WiFi.begin(ssid, pass); 
      iCount = 0;
    } 
    iCount++;
  }
  Serial.println("");
  Serial.println("WiFi connected"); 
  Serial.print("IP Address: "); 
  Serial.println(WiFi.localIP());

  // Start the UDP client
  Client.begin(27); */
  Wire.begin(D1, D2); /* join i2c bus with SDA=D1 and SCL=D2 of NodeMCU */
}
 
void loop() {
  // put your main code here, to run repeatedly:

  /**開始發送資料給Server端 **/
  // Send the distance to the client, along with a break to separate our messages

  //Client.beginPacket(ip,27); //前面指定的Port
  // heart beat
  Wire.requestFrom(0xA0 >> 1, 1);    // request 1 bytes from slave device
  while(Wire.available()) {          // slave may send less than requested
    unsigned char c = Wire.read();   // receive heart rate value (a byte)
    Serial.println(c, DEC);         // print heart rate value
    //Client.println(c, DEC);
    Serial.println(c);         // print heart rate value
    Serial.print(".");
  }
  //Client.endPacket();

  //delay(BNO055_SAMPLERATE_DELAY_MS);
  delay(500);
}
