#include <Arduino.h>

//Outside LED Control
//int ledPin = D0;

/****--------------------Slide Switch-------------------------------****/
// constants won't change. They're used here to
// set pin numbers:
const int slidebuttonPin = D2;             // the number of the pushbutton pin
//const int ledPin =    D2;            // the number of the LED pin
// variables will change:
int slidebuttonState = 0;                 // variable for reading the pushbutton status
/****--------------------Slide Switch-------------------------------****/

/****--------------------Push Switch-------------------------------****/
const int pushButton = D1;
const int ledPin =    D0; 
int pushbuttonState = 0;
/****--------------------Push Switch-------------------------------****/

/****--------------------Slider sensor-------------------------------****/
int sensorPin = A0;
int value = 0;
/****--------------------Slider sensor-------------------------------****/

void setup() {
  //Outside LED Control
  //pinMode(ledPin, OUTPUT);
  //Outside LED Control

/****--------------------Slide Switch-------------------------------****/
// constants won't change. They're used here to
  // initialize the LED pin as an output:
    //pinMode(ledPin, OUTPUT);
    // initialize the pushbutton pin as an input:
    pinMode(slidebuttonPin, INPUT);
/****--------------------Slide Switch-------------------------------****/

/****--------------------Push Switch-------------------------------****/
  // initialize serial communication at 9600 bits per second:
/*   Serial.begin(9600);
  // make the pushbutton's pin an input:
  pinMode(pushButton, INPUT);
  pinMode(ledPin, OUTPUT); */
/****--------------------Push Switch-------------------------------****/

/****--------------------Slider sensor-------------------------------****/
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
/****--------------------Slider sensor-------------------------------****/

}

void loop() {
  //Outside LED Control
/*   digitalWrite(ledPin, HIGH); // 設定PIN13腳位為高電位= 0V ，LED 處於發亮狀態!!
  delay(100); // 等待100 毫秒 (也就是發亮 0.1 秒)
  digitalWrite(ledPin, LOW); // 設定PIN13腳位為低電位= 0V ，LED 處於熄滅狀態!!
  delay(100); // 等待100 毫秒 (也就是熄滅 0.1 秒) */
  //Outside LED Control

/****--------------------Slide Switch-------------------------------****/
    // read the state of the pushbutton value:
    slidebuttonState = digitalRead(slidebuttonPin);
    // check if the pushbutton is pressed.
    // if it is, the buttonState is HIGH:
    if (slidebuttonState == HIGH) {
        // turn LED on:
        digitalWrite(ledPin, HIGH);
    } else {
        // turn LED off:
        digitalWrite(ledPin, LOW);
    }
/****--------------------Slide Switch-------------------------------****/

/****--------------------Push Switch-------------------------------****/
// read the input pin:
/*    pushbuttonState = digitalRead(pushButton);
  // print out the state of the button:
  Serial.println(pushbuttonState);
      if (pushbuttonState == LOW) {
        // turn LED on:
        digitalWrite(ledPin, HIGH);
    } else {
        // turn LED off:
        digitalWrite(ledPin, LOW);
    } */
/****--------------------Push Switch-------------------------------****/

/****--------------------Slider sensor-------------------------------****/
  // initialize serial communication at 9600 bits per second:
  value = analogRead(sensorPin);
  Serial.println(value, DEC);
/****--------------------Slider sensor-------------------------------****/

}
