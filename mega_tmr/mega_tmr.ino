#include <math.h>
#include <Arduino.h>
#include <float.h>

const double pi = M_PI;
constexpr uint32_t ledPin = 13;   // select the pin for the LED
constexpr float period = 1e6 / 1; // readout period in microseconds (1 s rn)
uint32_t lastT = 0;
constexpr float Rconst = 10000; // resistance (ohm) of the constant resistor (pm 5%)
constexpr int bitres = 10;
constexpr int AVG_SIZE = 10; //average over this amount of corrected values
constexpr uint32_t maxADC = (pow(2, bitres) - 1) * AVG_SIZE; //mega 2560 and uno have 10 bit
constexpr float ADCvoltage = 5.0;           // max voltage a pin reads is 5.
constexpr uint32_t bitThreshold = 20;
constexpr int REDUNDANCY = 4; // number of redundant readings
constexpr int outPin[3][REDUNDANCY] = {{2, 3, 4, 5}, { 6, 7, 8, 9}, {10, 11, 12, 13}}; // output pins for the sensors
constexpr uint8_t pins[3][REDUNDANCY] = {{A0, A1, A2, A3}, {A4, A5, A6, A7}, {A8, A9, A10, A11}}; //readout pins
constexpr float law = 1; // power of the inverse law of photoresistor.
uint8_t unSelect = 3; // which LDR is skipped

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



float tmr(int* pVals, const int cord){
  // Find the binomial pairs with the lowes difference and take their average
  int p1=-1, p2=-1; //Initialize with easy to spots
  float diff = pow(2,10) + 2;// more than bit res 
  int noted = 0; // count below threshold
  for (int i=0; i < REDUNDANCY; i++){
    // count values below threshold
    if (i != unSelect) {
      if (pVals[i] < bitThreshold){
        noted += 1;
        }
      // Loop over second unique reading
      for (int j=i+1; j < REDUNDANCY; j++){
        if (j != unSelect) {
          float diffNew = fabs(pVals[i] - pVals[j]);
          // update if smallest difference is found
          if ( diffNew < diff) {
            diff = diffNew; 
            p1 = i; 
            p2 = j;
          }
        }
      }
    }
  }
  // Everything below threshold assume legitimate 0
  if (noted == 3){
    Serial.print(" Legit 0 ");
    return 0; 
  }
  // Takes the two best
  else if (p1 !=-1 && p2 !=-1){  
    float nearestAverage = (float)(pVals[p1] + pVals[p2])/2;
    Serial.print(" NORMAL SET ");
    Serial.print("  zeros  ");
    Serial.print(noted);
    return nearestAverage;
    }
  // This should only happen ilogically
  else {
    Serial.print(" BAD ??? ");
    return -1; 
    }
}


float resitanceMeasure(uint32_t reading){
  // Do ADC and calculate the resustance
  float V = (float)reading / maxADC * ADCvoltage;
  float  resistance;
  if (V==0){
    // handle exceptions like a not lazy person
    resistance=-1;
  }
  else {
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
    else {
      result[i] = -1;
    }
}

void sunAng(float* v, float* result){
  // retrieve sunangles on lit sides
  // angles ambiguity is not applicable since 1 octant of r^3 is used
  // [0] for theta [1] for phi
  if (v[0] > 0 && v[1] > 0 && v[2] > 0){
    // angle that v makes wrt z
    float r = sqrt(pow(v[0], 2) + pow(v[1], 2));
    result[0] = atan(v[2]/r)*180/pi;
    result[1] = atan(v[1]/v[0])*180/pi;
  }
  else if (v[0] > 0 && v[1] > 0 && v[2] < 0){
    // z is not lit case
    result[0] = -1;
    result[1] = atan(v[1]/v[0])*180/pi; 
  }
  else if (v[0] > 0 && v[1] < 0 && v[2] > 0){
    // y is not lit so phi < 0 or > 180
    result[0] = atan(v[2]/v[0])*180/pi;
    result[1] = -1; 
  }
  else if (v[0] < 0 && v[1] > 0 && v[2] > 0){
    // x is not lit so phi < -90 or > 90
    result[0] = atan(v[2]/v[1])*180/pi;
    result[1] = -1; 
  }
  else {
    result[0] = -1;
    result[1] = -1; 
  }
}

void loop()
{
  // this loop is timed
  uint32_t countT = micros();
  float xReading_sum  = 0; 
  float yReading_sum  = 0; 
  float zReading_sum  = 0; 
  if (countT - lastT >= period){
    for (int j = 0; j < AVG_SIZE; j++) {
      // Readings is a voltage bit from the sensor
      int xReadings[REDUNDANCY];
      int yReadings[REDUNDANCY];
      int zReadings[REDUNDANCY];
      for (int i = 0; i < REDUNDANCY; i++) {
        pinMode(outPin[0][i], OUTPUT);
        pinMode(outPin[1][i], OUTPUT);
        pinMode(outPin[2][i], OUTPUT);
        digitalWrite(outPin[0][i], HIGH);
        digitalWrite(outPin[1][i], HIGH);
        digitalWrite(outPin[2][i], HIGH);
        xReadings[i] = analogRead(pins[0][i]);
        yReadings[i] = analogRead(pins[1][i]);
        zReadings[i] = analogRead(pins[2][i]);
        // Delay to prevent too fast reading/switching (if that exists)
        delay(10); 
        pinMode(outPin[0][i], INPUT);
        pinMode(outPin[1][i], INPUT);
        pinMode(outPin[2][i], INPUT);

#if READOUT == 1
      // Serial.print("\n Series ");
      // Serial.println(j);
      Serial.print("Redundancy ");
      Serial.print(i);
      Serial.print("\tVx: ");
      Serial.print(xReadings[i]);
      Serial.print("\tVy: ");
      Serial.print(yReadings[i]);
      Serial.print("\tVz: ");
      Serial.println(zReadings[i]);
#endif
      }
      //Apply redundancy and do a correction
      xReading_sum += tmr(xReadings, 0);
      yReading_sum += tmr(yReadings, 1);
      zReading_sum += tmr(zReadings, 2);
      // cycle through skipping an LDR per side
      unSelect += 1;
      unSelect %= 4;
    }
    float xResistance = resitanceMeasure(xReading_sum);
    float yResistance = resitanceMeasure(yReading_sum);
    float zResistance = resitanceMeasure(zReading_sum);
    float R[3] = {xResistance, yResistance, zResistance};
#if READOUT == 1
    Serial.println("");
    Serial.print("Rx: ");
    Serial.print(xResistance);
    Serial.print("\tRy: ");
    Serial.print(yResistance);
    Serial.print("\tRz: ");
    Serial.println(zResistance);
#endif
    Serial.println("");
    // get the Sunvector
    float sunVector[3] = {0,0,0};
    sunVec(R, sunVector);
    Serial.print("Sx: ");
    Serial.print(sunVector[0]);
    Serial.print("\tSy: ");
    Serial.print(sunVector[1]);
    Serial.print("\tSz: ");
    Serial.print(sunVector[2]);
    // get the sun angles
    float sunAngles[2] = {0,0};
    sunAng(sunVector, sunAngles);
    Serial.print("\ttheta: ");
    Serial.print(sunAngles[0]);
    Serial.print("\tphi: ");
    Serial.println(sunAngles[1]);
    unSelect += 1;
    unSelect %= 4;
    lastT = micros();
  }
}
