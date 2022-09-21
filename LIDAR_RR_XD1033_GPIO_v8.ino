#include <Wire.h> 
#include <LIDARLite.h> 
#include <ResponsiveAnalogRead.h> 
#include <Adafruit_NeoPixel.h>
#include "WS2812_Definitions.h"

#define WAITVAL 10

#define PIN 4
#define LED_COUNT 5

#define LOOPMAX 500
  
float decimalValue; 
int eightBitValue; 
int dist, z, critZ, aFull, wait; 
uint8_t edgeMax, edge3, edgeMin, pot4, pot3, pot2, pot1;

bool rangeFlag, zone1Flag, zone2Flag, zone3Flag;
uint8_t lastZoneFlagged, currentZone;
  
LIDARLite lidarLite; 
ResponsiveAnalogRead analog(dist, true, 0.003); 

Adafruit_NeoPixel leds = Adafruit_NeoPixel(LED_COUNT, PIN, NEO_GRB + NEO_KHZ800);
  
int cal_cnt = 0; 
int loopCounter = 0;
int PWMCounter = 0;
  
void setup() 
{ 
  Serial.begin(9600); // Initialize serial connection to plot output

  StartLED();
  
  lidarLite.begin(0, true); // Set configuration to default and I2C to 400 kHz 
  lidarLite.configure(0); // Change this number to try out alternate configurations 
  pinMode(7, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(8, INPUT);

  lastZoneFlagged = 0;
  currentZone = 0;

  wait = WAITVAL;

  
} //end setup
  
void loop() 
{ 

  if(loopCounter < LOOPMAX)
  {
    loopCounter++;
  } else
  {
    loopCounter = 0;
  }
  
  analog.update( LidarBias() ); 
  z = analog.getValue();

  //limiter
  if(z > 464) 
  {
    z = 464;
  }
  z = map( z,0,464,1,100);

  //read from calibration pots to set trigger-zone edges
  GetEdges();
  
  delay(10); 

  //determine if zones have been breached and trigger BSoutput
  //ZoneCheck();
  ZoneCheckOG();

  //checkDIP for value
  //collect color from pot 1 and write to LEDs
  if(   ( loopCounter == 0 )          ||
        ( loopCounter == LOOPMAX/4 )  ||
        ( loopCounter == LOOPMAX/2 )  ||
        ( loopCounter == (LOOPMAX/4) + (LOOPMAX/2) )  
    )
  {
      SetColorFromPot1();
      SerialState();
  }

  void DistanceIsBrightness();
  
  //SerialState();
  PlotZAndEdges();
  //PlotPots();
  //PlotEdges();

} //end loop

int LidarBias()
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

  return dist;
}

void SerialState()
{
  if( digitalRead(8) == HIGH )
  {
    PlotZAndEdges();
  } 
  else
  {
    StreamDistance();
  }
}

void ZoneCheck()
{
  if(z > edgeMax)
  {
    currentZone = 0;
  }
  else if(edgeMax > z > edge3)
  {
    currentZone = 2;
  }
  else if(edge3 > z)
  {
    currentZone = 1;
  }
  
  if(lastZoneFlagged != currentZone)
  {
    if(currentZone == 0)
    {
      Pulse(7);
      delay(wait);
      lastZoneFlagged = 0;
    }
    else if(currentZone == 1)
    {
      Pulse(9);
      delay(wait);
      lastZoneFlagged = 1;
    }
    else
    {
      Pulse(12);
      delay(wait);
      lastZoneFlagged = 2;
    }  
  }

}
  
void ZoneCheckOG() 
{
  if(z > edgeMax) //subject is out of range
  {
    //if the subject was previously in range and moves out, run reset pin
    if(lastZoneFlagged != 0)
    {
      Pulse(7); //reset pin is D7, BS_GPIO_5
      delay(wait);
      lastZoneFlagged = 0;
 
    } 
    //else do nothing
  } 
  else //subject is in range
  {
    //if the subject was out of range and moves in, OR if moving between zones, run trigger 1 or 2
    if(lastZoneFlagged == 0)
    {
      if(lastZoneFlagged != 2)
      {
      Pulse(12); //reset pin is D7, BS_GPIO_5
      delay(wait);
      lastZoneFlagged = 2;
      }
    }
  }
}

void Pulse(int a)
{
    digitalWrite(a, HIGH);
    delay(100);
    digitalWrite(a, LOW);
}

void GetEdges()
{
  float d_all;
  //turn on power to the pots
  digitalWrite(13, HIGH);
  //ensure the pots are powered up
  delay(2);
  
  //sample and store each pot, remap 1-100
  //minimum val of 1 to avoid div errors
  pot4 = 
    map(
      analogRead(0),
      0,930,
      4,100
    );
  pot3 =
    map(
      analogRead(1),
      0,930,
      1,100
    );
  pot2 =
    map(
      analogRead(2),
      0,930,
      1,97
    );
  if( pot4 <= (pot2+2) )
  {
    pot4 = pot2+2;
  }

  //assign bounds
  edgeMax = pot4;
  edgeMin = pot2;
  d_all = (float)(edgeMax - edgeMin);

  //scale inner edges
  edge3 = edgeMax - (uint8_t)( d_all * Percent(pot3) ); 
  
  //turn off power to the pots
  digitalWrite(13, LOW); 

  }

float Percent(uint8_t n)
{
  return (float)n / 100;
}

void PlotLidar()
{
  // Display distance 
  //raw
  Serial.print(lidarLite.distance()); 
  Serial.print("\t"); 
  //filtered
  Serial.println(z);
}

void PlotPots()
{
  Serial.print(pot4);
  Serial.print("\t");
  Serial.print(pot3);
  Serial.print("\t");
  Serial.print(pot2);
  Serial.print("\t");
  Serial.println(pot1);
}

void PlotZAndEdges()
{
  Serial.print(lastZoneFlagged*10);
  Serial.print("\t");
  Serial.print(z);
  Serial.print("\t");
  Serial.print(edgeMax);
  Serial.print("\t");
  Serial.print(edge3);
  Serial.print("\t");
  Serial.println(edgeMin);


}

void StreamDistance()
{
  Serial.println(z);
}

void StartLED()
{
  leds.begin();  // Call this to start up the LED strip.
  clearLEDs();   // This function, defined below, turns all LEDs off...
  leds.show();   // ...but the LEDs don't actually update until you call this.
}

void clearLEDs()
{
  for (int i=0; i<LED_COUNT; i++)
  {
    leds.setPixelColor(i, 0);
  }
}

void SetColorFromPot1()
{
  //analogRead(3), pot1
  //based on value, return color to use in LED function
  for(int i=0; i<LED_COUNT; i++)
  {
    led.setPixelColor(i, map( analogRead(3),0,930,0,192) )
  }
}

void DistanceIsBrightness()
{
  //use z to set brightness of led, between 1 and 100
  //z is the number of cycles between 'show' commands
  //using PWM...
  if(PWMCounter < z)
  {
    PWMCounter++;
  } else
  {
    leds.show();
    PWMCounter = 0;
    delay(1)
    clearLEDs();
  }
  
  
}
 
