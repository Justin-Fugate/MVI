//LIDAR RESPONSIVE ANALOG READ

#include <Wire.h> 
#include <LIDARLite.h> 
#include <ResponsiveAnalogRead.h> 
  


float decimalValue; 
  

int eightBitValue; 
  

int dist; 
  

LIDARLite lidarLite; 
ResponsiveAnalogRead analog(dist, true, 0.003); 
  

int cal_cnt = 0; 
  

void setup() 
{ 
  Serial.begin(19200); // Initialize serial connection to display distance readings 
  

  lidarLite.begin(0, true); // Set configuration to default and I2C to 400 kHz 
  lidarLite.configure(0); // Change this number to try out alternate configurations 
} 
  

void loop() 
{ 
   

   

  // At the beginning of every 100 readings, 
  // take a measurement with receiver bias correction 
  if ( cal_cnt == 0 ) { 
    dist = lidarLite.distance();      // With bias correction 
  } else { 
    dist = lidarLite.distance(false); // Without bias correction 
  } 
  

  // Increment reading counter 
  cal_cnt++; 
  cal_cnt = cal_cnt % 100; 
  

  analog.update(dist); 
   

  //convertTo8Bit(); 
  

  // Display distance 
  //Serial.print(lidarLite.distance()); 
  //Serial.print("\t"); 
  Serial.println(analog.getValue()); 
  

  delay(125); 
} 
  

/*void convertTo8Bit() 
{ 
  decimalValue = dist * 0.69; //the dist value is an int, but the decimal in the multiplied number forces the result to be a float 
  eightBitValue = (int) decimalValue; //converts the float value above back into an int and drops the decimal value 
}*/ 
 

 

//DECIMAL to 10-BIT

//send control value to pins 
void DecTo10bit(int a)   
{ 
  if(a >= 512){ 
    digitalWrite(pin9, HIGH); 
    a = a % 512; 
  } 
  if(a >= 256){ 
    digitalWrite(pin8, HIGH); 
    a = a % 256; 
  } 
  if(a >= 128){ 
    digitalWrite(pin7, HIGH); 
    a = a % 128; 
  } 
  if(a >= 64){ 
    digitalWrite(pin6, HIGH); 
    a = a % 64; 
  } 
  if(a >= 32){ 
    digitalWrite(pin5, HIGH); 
    a = a % 32; 
  } 
  if(a >= 16){ 
    digitalWrite(pin4, HIGH); 
    a = a % 16; 
  } 
  if(a >= 8){ 
    digitalWrite(pin3, HIGH); 
    a = a % 8; 
  } 
  if(a >= 4){ 
    digitalWrite(pin2, HIGH); 
    a = a % 4; 
  } 
  if(a >= 2){ 
    digitalWrite(pin1, HIGH); 
    a = a % 2; 
  } 
  if(a >= 1){ 
    digitalWrite(pin0, HIGH); 
    a = a % 1; 
  } 
} 


   
