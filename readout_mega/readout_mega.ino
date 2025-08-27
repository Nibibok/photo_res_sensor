#include <math.h>
#include <Arduino.h>
#include <float.h>

constexpr uint32_t ledPin = 13;   // select the pin for the LED
constexpr float period = 1e6 / 1; // readout period in microseconds (1 s rn)
uint32_t lastT = 0;
constexpr float Rconst = 10000; // resistance (ohm) of the constant resistor (pm 5%)
constexpr int bitres = 10;
constexpr int AVG_SIZE = 1; //average over
constexpr uint32_t maxADC = (pow(2, bitres) - 1) * AVG_SIZE; //mega 2560 and uno have 10 bit
constexpr float ADCvoltage = 5.0;           // max voltage a pin reads is 5.
constexpr uint32_t bitThreshold = 10;
constexpr int REDUNDANCY = 4; // number of redundant readings
constexpr int outPin[3][REDUNDANCY] = {{2, 3, 4, 5}, { 6, 7, 8, 9}, {10, 11, 12, 13}}; // output pins for the sensors
constexpr uint8_t pins[3][REDUNDANCY] = {{A0, A1, A2, A3}, {A4, A5, A6, A7}, {A8, A9, A10, A11}}; //readout pins
constexpr float law = 1; // power of the inverse law of photoresistor.


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
  pinMode(A6, INPUT);
  pinMode(A7, INPUT);
  pinMode(A8, INPUT);
  pinMode(A9, INPUT);
  pinMode(A10, INPUT);
  pinMode(A11, INPUT);
}



float nearestNeighbor(float* pVals, const int coord){
  // Find the binomial pairs with the lowes difference and take their average
  int p1=-1, p2=-1; //Initialize with easy to spots
  float diff = 1e10; 
  int invalids = 0;
  float valMax = pVals[0]; 
  for (int i=0; i < REDUNDANCY; i++){
    // Count the numper of invalid measurements (set below 0)
    if (pVals[i] < 0){
      invalids += 1;
      }
    // Save the maximum
    if (pVals[i] > valMax){
      valMax = pVals[i];
      }
    for (int j=i+1; j < REDUNDANCY; j++){
      float diffNew = fabs(pVals[i] - pVals[j]);
      // Update to the smallest difference if not -1
      if ( pVals[i] > 0 && pVals[j] > 0 && diffNew < diff) {
        diff = diffNew; 
        p1 = i; 
        p2 = j;
        }
    }
  }
  // Everything below threshold assume legitimate 0
  if (invalids == 4){
    Serial.print(" Legit 0 ");
    return 0; 
  }
  // Redundancy is exhausted -> return only valid
  else if (invalids ==3){
    Serial.print(" EDGING ");
    return valMax;
  }
  // At least two valid indices
  else if (p1 !=-1 && p2 !=-1){  
    float nearestAverage = (pVals[p1] + pVals[p2])/2;
    Serial.print(" GOOD SET ");
    Serial.print("  Invalids  ");
    Serial.print(invalids);
    return nearestAverage;
    }
  // This should only happen ilogically
  else {
    Serial.print(" BAD ??? ");
    return -1; 
    }
}


float resitanceMeasure(uint32_t reading){
  // Do ADC and calculae the resustance
  float V = (float)reading / maxADC * ADCvoltage;
  // Below threshold is marked with -1
  float resistance = -1;
  if (reading > bitThreshold){
    // invalidate below threshold
    resistance = Rconst * (ADCvoltage - V) / V;
  }
  return resistance;
}

void sunVec(float* r, float* result){
  //calculate norm even with 0 readings
  float sqSum = 0; 
  for (int i=0; i<3; i++){
    if (r[i] > 0){
      sqSum += pow(r[i], -2/law);    
    }
  }
  for (int i=0; i<3; i++)
    if (r[i] > 0){
      result[i] = pow(r[i], -1/law)/sqrt(sqSum);    
    }
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
    for (int i = 0; i < REDUNDANCY; i++)
    {
      pinMode(outPin[0][i], OUTPUT);
      pinMode(outPin[1][i], OUTPUT);
      pinMode(outPin[2][i], OUTPUT);
      digitalWrite(outPin[0][i], HIGH);
      digitalWrite(outPin[1][i], HIGH);
      digitalWrite(outPin[2][i], HIGH);
      xReadings[i] = analogRead(pins[0][i]);
      yReadings[i] = analogRead(pins[1][i]);
      zReadings[i] = analogRead(pins[2][i]);
      delay(10); // Assume the arduino wants to set a pin to INPUT very fast
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
#if READOUT == 1
      Serial.print("\tRx: ");
      Serial.print(xResistance[i]);
      Serial.print("\tRy: ");
      Serial.print(yResistance[i]);
      Serial.print("\tRz: ");
      Serial.println(zResistance[i]);
#endif
    }
    if (nmrMode == 1) {

    }
    else {
      float xRCorrected = nearestNeighbor(xResistance, 0);
      float yRCorrected = nearestNeighbor(yResistance, 1);
      float zRCorrected = nearestNeighbor(zResistance, 2);
    }
    float R[3] = {xRCorrected, yRCorrected, zRCorrected};
    Serial.println("");
#if READOUT ==1
    Serial.print("\tx_corrected: ");
    Serial.print(xRCorrected);
    Serial.print("\ty_corrected: ");
    Serial.print(yRCorrected);
    Serial.print("\tz_corrected: ");
    Serial.println(zRCorrected);
#endif

    //get the Sunvector
    float psunVector[3] = {0,0,0};
    sunVec(R, psunVector);
    Serial.print("Sx: ");
    Serial.print(psunVector[0]);
    Serial.print("   ");
    Serial.print("Sy: ");
    Serial.print(psunVector[1]);
    Serial.print("   ");
    Serial.print("Sz: ");
    Serial.println(psunVector[2]);
    lastT = micros();
  }
}
