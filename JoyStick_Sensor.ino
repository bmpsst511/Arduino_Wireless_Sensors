void setup()
{
  Serial.begin(9600); //Set serial baud rate to 9600 bps
}
void loop()
{
int val;
int val2;
val=analogRead(0);//Read slider value from analog 0
val2=analogRead(2);//Read slider value from analog 0
//Serial.print(val,DEC);//Print the value to serial port
//Serial.println(val2,DEC);//Print the value to serial port
Serial.println(val+val2,DEC);//Print the value to serial port
delay(100);
}
