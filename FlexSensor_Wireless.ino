#include <Arduino.h>

#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WiFiUDP.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

const int FLEX_PIN = A0; // Pin connected to voltage divider output 
// Measure the voltage at 5V and the actual resistance of your 
// 47k resistor, and enter them below: 
const float VCC = 3.2; 
// Measured voltage of Ardunio 5V line 
const float R_DIV = 2200.0; 
// Measured resistance of 3.3k resistor 
// Upload the code, then try to adjust these values to more 
// accurately calculate bend degree. 
const float STRAIGHT_RESISTANCE = 25000.0; 
// resistance when straight 
const float BEND_RESISTANCE = 125000.0; 
// resistance at 90 deg 
float Filter_Value;
const char ip[]="172.20.10.3";
const char* ssid =/*"dlink";*//*"iPhone"*/"LeeiPhone";
const char* password = /*"468255000";*//*"19940625"*/"hnwl0618";
WiFiServer server(27);
WiFiUDP Client;

void setup() 
{
  Serial.begin(9600);
  pinMode(FLEX_PIN, INPUT); 
  WiFi.mode(WIFI_STA); 
  
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
  
}

// 加??推平均?波法
#define FILTER_N 12
int coe[FILTER_N] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};    // 加?系?表
int sum_coe = 1 + 2 + 3 + 4 + 5 + 6 + 7 + 8 + 9 + 10 + 11 + 12; // 加?系?和
int filter_buf[FILTER_N + 1];
float Filter() {
  int i;
  float filter_sum = 0;
  filter_buf[FILTER_N] = analogRead(FLEX_PIN);
  for(i = 0; i < FILTER_N; i++) {
    filter_buf[i] = filter_buf[i + 1]; // 所有?据左移，低位仍掉
    filter_sum += filter_buf[i] * coe[i];
  }
  filter_sum /= sum_coe;
  return filter_sum;
}

void loop() 
{
  // Read the ADC, and calculate voltage and resistance from it
  Filter_Value = Filter();       // 獲得濾波器輸出值   
  //int flexADC = analogRead(FLEX_PIN);   
  float flexV = Filter_Value * VCC / 1023.0;   
  float flexR = R_DIV * (VCC / flexV - 1.0);   
  Serial.println("Resistance: " + String(flexR) + " ohms");   
  // Use the calculated resistance to estimate the sensor's   
  // bend angle:   
  float angle = map(flexR, STRAIGHT_RESISTANCE, BEND_RESISTANCE, 0, 90.0);   
  //Serial.println("Bend: " + String(angle) + " degrees");   
  //Serial.println();   
  // Send the distance to the client, along with a break to separate our messages
  Client.beginPacket(ip,27);
  Client.println(flexR);
  Client.endPacket();
  delay(50);
}
