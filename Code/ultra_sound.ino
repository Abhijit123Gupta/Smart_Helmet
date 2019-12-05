const int pingPin = 7; // Trigger Pin of Ultrasonic Sensor
const int echoPin = 6; // Echo Pin of Ultrasonic Sensor
int i;
int inter=13;
boolean alc;
boolean flex;
//interrupt pin
void setup() {
  // put your setup code here, to run once:
pinMode(A0,INPUT);
pinMode(pingPin, OUTPUT);
pinMode(echoPin, INPUT);
Serial.begin(9600);
pinMode(inter,OUTPUT);
pinMode(2,INPUT);
pinMode(10,OUTPUT);
i=0;

//Alcohol and Flex Sensor
alc=digitalRead(11);
flex = analogRead(A0);

if(alc==1 &&  flex>400 )//Change Threshold after testing
Serial.println("ALL GOOD");
else
{
Serial.println("NOT GOOD.");
digitalWrite(10,HIGH);
delay(1000);
digitalWrite(10,LOW);
}
}
void loop() {
  
  // put your main code here, to run repeatedly:
long duration, inches, cm;
   
   digitalWrite(pingPin, LOW);
   delayMicroseconds(2);
   digitalWrite(pingPin, HIGH);
   delayMicroseconds(10);
   digitalWrite(pingPin, LOW);
   
   duration = pulseIn(echoPin, HIGH);
   inches = microsecondsToInches(duration);
   cm = microsecondsToCentimeters(duration);
   //Serial.print(inches);
   //Serial.print("in, ");
   Serial.print(cm);
   Serial.print("cm");
   Serial.println();
   //delay(100);
   if(cm<=110){
    Serial.println("sound sensor enabled");
   
    sound();
    
   }
   
}

long microsecondsToInches(long microseconds) {
   return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds) {
   return microseconds / 29 / 2;
}

void sound() {
  // put your main code here, to run repeatedly:

int i,temp,s;
temp=0;
for(i=0;i<100;i++)
{

s=digitalRead(2);
temp=temp+s;
//Serial.println(s);
}

if(temp>45)
 {digitalWrite(inter,HIGH);
  Serial.println("interrupt");
  delay(2000);}
//Serial.println(" high");
//delay(2000);}
else //Serial.println("low");
digitalWrite(inter,LOW);

}
