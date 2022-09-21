//PING PARALLAX DISTANCE SENSOR

const int pingPin = 11; 
unsigned int duration, inches; 
  

void setup() { 
  // put your setup code here, to run once: 
  Serial.begin(9600); 
} 
  

void loop() { 
  // put your main code here, to run repeatedly: 
  pinMode(pingPin, OUTPUT); 
  digitalWrite(pingPin, LOW); 
  delayMicroseconds(2); 
  digitalWrite(pingPin, HIGH); 
  delayMicroseconds(5); 
  digitalWrite(pingPin, LOW); 
  pinMode(pingPin, INPUT); 
  duration = pulseIn(pingPin, HIGH); 
  inches = duration /74/2; 
  Serial.println(inches); 
  delay(200); 
  

} 
