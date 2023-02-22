#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WiFiUDP.h>
#include <OSCMessage.h>
#include <Wire.h>

const char *ssid =  "xxx";     // replace with your wifi ssid and wpa2 key
const char *pass =  "xxx";

const IPAddress outIp(172,20,10,5); //分享器給你Server的IP位址
const unsigned int outPort = 8000;
const unsigned int localPort = 6969;

/** 指定Port 且設置為客戶端**/
WiFiServer server(outPort);
WiFiUDP Udp;
//WiFiClient client;
/** 指定Port 且設置為客戶端**/
 
void setup() 
{
  Serial.begin(9600); /* begin serial for debug */

  delay(10);

  Serial.println("Connecting to ");
  Serial.println(ssid); 
  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected"); 
  Serial.print("IP Address: "); 
  Serial.println(WiFi.localIP());

  // Start the UDP client
  Udp.begin(localPort);
  Wire.begin(D1, D2); /* join i2c bus with SDA=D1 and SCL=D2 of NodeMCU */
}
 
void loop() {
  // put your main code here, to run repeatedly:

  /**開始發送資料給Server端 **/
  // Send the distance to the client, along with a break to separate our messages

  //Client.beginPacket(ip,27); //前面指定的Port
  // heart beat
  OSCMessage msg("/osc/inputs");
  Wire.requestFrom(0xA0 >> 1, 1);    // request 1 bytes from slave device
  while(Wire.available()) {          // slave may send less than requested
    unsigned char c = Wire.read();   // receive heart rate value (a byte)
    Serial.println(c, DEC);         // print heart rate value
    //Serial.println(c);         // print heart rate value
    //Serial.print(".");
    //Start to send value to max/msp
    //Udp.println(c, DEC);
    msg.add(c);
    Udp.beginPacket(outIp, outPort);
    msg.send(Udp);
    Udp.endPacket();
  }

  //delay(BNO055_SAMPLERATE_DELAY_MS);
  delay(500);
}
