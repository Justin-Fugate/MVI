//ARDUINO 8-BIT LED BINARY DISTANCE METER

//this assigns a pin to the ping sensor, a const is a variable that is read-only 
const int pingPin = 13;  
  

//this assigns variables for the ping sensor readings, an unsigned int is an int that cannot be negative, it only stores positive values 
unsigned int duration, cm; 
  

//converted 8 bit float value (0.0 - 255.0) 
float decimalValue;  
  

//converted 8 bit int value (0 - 255) 
int binaryOutNum;  
  

//8 bit binary number output into 8 places 
String binaryOutStr; 
  

//an array to store the binary values for the pin output from the binaryOutStr string 
byte myBins[9]; 
  

//how many pins being used as binary output 
const byte numPins = 8;  
  

// used for HIGH or LOW (pin on or off) 
int state;  
  

//this is an array to store the pin numbers, a byte takes up less space than an int 
byte binPins[] = {10,9,8,7,6,5,4,3}; 
   

void setup()  
{  
  // put your setup code here, to run once:  
  

  //sets the serial port to use 
  Serial.begin(9600);  
  

  //sets up all the pins as OUTPUT 
  for(int i = 0; i < 8; i++) 
  { 
    pinMode(binPins[i], OUTPUT); 
  } 
  
}  


void loop()  
{  
  // put your main code here, to run repeatedly:  
  

  //run the getDistance function, which sends and receives from the ping sensor to get distance 
  getDistance();  
   

  //run the convertTo8Bit function, convert the distance in inches to an 8 bit value 
  convertTo8Bit(); 
  

  //run the convertToBinary function, convert the remapped 8 bit integer from convertTo8Bit function into binary format 
  convertToBinary(); 
  

  //run the pinsBinaryOut function, assigns the binaryOutStr value to a byte array, and then iterates over the pins to assign HIGH or LOW values 
  pinsBinaryOut(); 
   
} 
  

//sends out a signal from the ping sensor, checks how long it takes to bounce back, and calculates distance 
void getDistance() 
{ 
  pinMode(pingPin, OUTPUT);  
  digitalWrite(pingPin, LOW);  
  delayMicroseconds(2);  
  digitalWrite(pingPin, HIGH);  
  delayMicroseconds(5);  
  digitalWrite(pingPin, LOW);  
  pinMode(pingPin, INPUT);  
  duration = pulseIn(pingPin, HIGH); 
  cm = duration /29/2; //in centimeters because it gives more values to remap the range than inches 
} 
  

//maps the ping sensor distance reading from cm to an 8 bit range (0-255) 
void convertTo8Bit() 
{ 
  decimalValue = cm * 0.69; //the cm value is an int, but the decimal in the multiplied number forces the result to be a float 
  binaryOutNum = (int) decimalValue; //converts the float value above back into an int and drops the decimal value 
} 
  

//find the length of the binary number, subtract it from 8, take that value and iterate that many times and add a 0 to the front until all 8 places are filled 
void convertToBinary() 
{ 
  int zeros = 8 - String(binaryOutNum, BIN).length(); 
  String myStr; 
  for (int i=0; i<zeros; i++)  
  { 
    myStr = myStr + "0"; 
  } 
  binaryOutStr = myStr + String(binaryOutNum,BIN); 
  Serial.println(binaryOutStr); 
  delay(50); 
} 
  

//take the binaryOutStr value, assign it to the myBins byte array, check to see if each value is a 1 or 0, and turn the pin to HIGH or LOW accordingly, iterate across all pins 
void pinsBinaryOut() 
{ 
  for (int i = 0; i < numPins; i++) 
  { 
    binaryOutStr.getBytes(myBins, 9); 
    if(myBins[i] == '1') state = HIGH; 
    if(myBins[i] == '0') state = LOW; 
    digitalWrite(binPins[i], state);  
  } 
} 
