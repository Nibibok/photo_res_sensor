#include <math.h>
#include <Arduino.h>
#include <float.h>

constexpr uint32_t ledPin = 13;   // select the pin for the LED
float ADCvoltage = 5.0;           // max voltage a pin reads is 5.
constexpr float period = 1e6 / 1; // readout period in microseconds (1 s rn)
uint32_t lastT = 0;
constexpr float Rconst = 10000; // resistance (ohm) of the constant resistor (pm 5%)
constexpr int bitres = 10;
//constexpr float power = 1.0; // power of the inverse resitance law
constexpr uint32_t bitThreshold = 20;

constexpr int REDUNDANCY = 4; // number of redundant readings to average out noise
constexpr int AVG_SIZE = 1;

uint32_t maxADC = (pow(2, 10) - 1) * AVG_SIZE;
//constexpr float Llimit = 4.9 *  // Lowest voltage that we will consider 

constexpr int outPin[3][REDUNDANCY] = {{2, 3, 4, 5}, { 6, 7, 8, 9}, {10, 11, 12, 13}}; // output pins for the sensors

#define READOUT 1 // set to 1 to enable serial readout, 0 to disable

void setup()
{
  // declare the ledPin as an OUTPUT:
  // pinMode(ledPin, OUTPUT);
  Serial.begin(57600);
  for (int i = 0; i < REDUNDANCY; i++)
  {
    pinMode(outPin[0][i], INPUT);
    pinMode(outPin[1][i], INPUT);
    pinMode(outPin[2][i], INPUT);
  }
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
}


float nearestNeighbor(float* pVals, char str){
  // Find the binomial pairs with the lowes difference and take their average
  int p1=-1, p2=-1; //Initialize with easy to spots
  float diff = -1; 
  int invalids = 0;
  float valMax = pVals[0]; 
  for (int i=0; i < REDUNDANCY; i++){
    // count the numper of invalid measurements (set below 0)
    if (pVals[i] < 0){invalids += 1;}
    if (pVals[i] > valMax){valMax = pVals[i];}
    for (int j=i+1; j < REDUNDANCY; j++){
      float diffNew = fabs(pVals[i] - pVals[j]);
      // Update to the smallest difference
      if ((diffNew < diff || diff==-1) && (pVals[i])!= -1){diff = diffNew; p1 = i; p2 = j;}
    }
  }
  // At least two valid indices
  if (p1 !=-1 || p2 !=-1){  
    float nearestAverage = (pVals[p1] + pVals[p2])/2;
    return nearestAverage;
    }
  // Everything is invalid -> return 0?
  else if (invalids == 4){
    return 0; 
  }
  // Redundancy is exhausted -> return only valid
  else if (invalids ==3){
    return valMax;
  } 
  // Fkd up big
  else {
    return -1; 
    }
}


float resitanceMeasure(uint32_t reading){
  // ADC
  float V = (float)reading / maxADC * ADCvoltage;
  float resistance;
  if (reading < bitThreshold){
    // invalidate below threshold
    Serial.print("   INVALID!   ");
    resistance -1;
  }
  else {
    // basic resistance formula
    resistance = Rconst * (ADCvoltage - V) / V;
  }
  return resistance;
}
 

void loop()
{
  uint32_t countT = micros();
  if (countT - lastT >= period)
  {
    // read the value from the sensor:
    // Readings is a voltage bit thing
    uint32_t xReadings[REDUNDANCY];
    uint32_t yReadings[REDUNDANCY];
    uint32_t zReadings[REDUNDANCY];
    float xResistance[REDUNDANCY];
    float yResistance[REDUNDANCY];
    float zResistance[REDUNDANCY];
    float xIntensity[REDUNDANCY];
    float yIntensity[REDUNDANCY];
    float zIntensity[REDUNDANCY];
    for (int i = 0; i < REDUNDANCY; i++)
    {
      pinMode(outPin[0][i], OUTPUT);
      pinMode(outPin[1][i], OUTPUT);
      pinMode(outPin[2][i], OUTPUT);
      digitalWrite(outPin[0][i], HIGH);
      digitalWrite(outPin[1][i], HIGH);
      digitalWrite(outPin[2][i], HIGH);
      xReadings[i] = 0;
      yReadings[i] = 0;
      zReadings[i] = 0;
      for (int j = 0; j < AVG_SIZE; j++)
      {
        if (i % 2 == 0)
        {
          // Only values above a usefull threshold are taken in the average
          xReadings[i] = analogRead(A0);
          yReadings[i] = analogRead(A1);
          zReadings[i] = analogRead(A2);
        }
        else
        {
          xReadings[i] = analogRead(A3);
          yReadings[i] = analogRead(A4);
          zReadings[i] = analogRead(A5);
        }
      }

      pinMode(outPin[0][i], INPUT);
      pinMode(outPin[1][i], INPUT);
      pinMode(outPin[2][i], INPUT);
#if READOUT == 1
      Serial.print("Redundancy ");
      Serial.print(i);
      Serial.print("\tVx: ");
      Serial.print(xReadings[i]/AVG_SIZE);
      Serial.print("\tVy: ");
      Serial.print(yReadings[i]/AVG_SIZE);
      Serial.print("\tVz: ");
      Serial.print(zReadings[i]/AVG_SIZE);
#endif
      xResistance[i] = resitanceMeasure(xReadings[i]);
      yResistance[i] = resitanceMeasure(yReadings[i]);
      zResistance[i] = resitanceMeasure(zReadings[i]);
      xIntensity[i] = 1e3 / xResistance[i];
      yIntensity[i] = 1e3 / yResistance[i];
      zIntensity[i] = 1e3 / zResistance[i];
#if READOUT == 1
      Serial.print("\tRx: ");
      Serial.print(xResistance[i]);
      Serial.print("\tRy: ");
      Serial.print(yResistance[i]);
      Serial.print("\tRz: ");
      Serial.print(zResistance[i]);
      Serial.print("\tIx: ");
      Serial.print(xIntensity[i]);
      Serial.print("\tIy: ");
      Serial.print(yIntensity[i]);
      Serial.print("\tIz: ");
      Serial.println(zIntensity[i]);
#endif
    }
    float xICorrected = nearestNeighbor(xIntensity, "X");
    float yICorrected = nearestNeighbor(yIntensity, "Y");
    float zICorrected = nearestNeighbor(zIntensity, "Z");
#if READOUT ==1
    Serial.print("x_corrected: ");
    Serial.print(xICorrected);
    Serial.print("\ty_corrected: ");
    Serial.print(yICorrected);
    Serial.print("\tz_corrected: ");
    Serial.println(zICorrected);
#endif

    //Sunvector
    float norm = sqrt(pow(xICorrected, 2) + pow(yICorrected, 2) + pow(zICorrected, 2));
    float sunX = xICorrected/norm;
    float sunY = yICorrected/norm;
    float sunZ = zICorrected/norm;
    Serial.print("Sx: ");
    Serial.print(sunX);
    Serial.print("   ");
    Serial.print("Sy: ");
    Serial.print(sunY);
    Serial.print("   ");
    Serial.print("Sz: ");
    Serial.println(sunZ);


    lastT = micros();
  }
}
