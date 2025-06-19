#include <math.h>
#include <Arduino.h>

constexpr uint32_t ledPin = 13;   // select the pin for the LED
float ADCvoltage = 5.0;           // max voltage a pin reads is 5.
constexpr float period = 1e6 / 1; // readout period in microseconds (1 s rn)
uint32_t lastT = 0;
constexpr float Rconst = 10000; // resistance (ohm) of the constant resistor
constexpr int bitres = 10;
int xVec[3] = {1, 0, 0}; // vector component of a sensor side
int yVec[3] = {0, 1, 0};
int zVec[3] = {0, 0, 1};

constexpr int REDUNDANCY = 3; // number of redundant readings to average out noise
constexpr int AVG_SIZE = 100;

uint32_t maxADC = (pow(2, 10) - 1) * AVG_SIZE;

constexpr int outPin[REDUNDANCY][3] = {{0, 4, 8}, {1, 5, 9}, {2, 6, 10}}; // output pins for the sensors

void setup()
{
  // declare the ledPin as an OUTPUT:
  // pinMode(ledPin, OUTPUT);
  Serial.begin(57600);
  for (int i = 0; i < REDUNDANCY; i++)
  {
    pinMode(outPin[i][0], INPUT);
    pinMode(outPin[i][1], INPUT);
    pinMode(outPin[i][2], INPUT);
  }
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
}

void loop()
{
  uint32_t countT = micros();
  if (countT - lastT >= period)
  {
    // read the value from the sensor:
    uint32_t xReadings[REDUNDANCY];
    uint32_t yReadings[REDUNDANCY];
    uint32_t zReadings[REDUNDANCY];
    uint32_t xResistance[REDUNDANCY];
    uint32_t yResistance[REDUNDANCY];
    uint32_t zResistance[REDUNDANCY];
    float xIntensity[REDUNDANCY];
    float yIntensity[REDUNDANCY];
    float zIntensity[REDUNDANCY];
    for (int i = 0; i < REDUNDANCY; i++)
    {
      pinMode(outPin[i][0], OUTPUT);
      pinMode(outPin[i][1], OUTPUT);
      pinMode(outPin[i][2], OUTPUT);
      digitalWrite(outPin[i][0], HIGH);
      digitalWrite(outPin[i][1], HIGH);
      digitalWrite(outPin[i][2], HIGH);
      xReadings[i] = 0;
      yReadings[i] = 0;
      zReadings[i] = 0;
      for (int j = 0; j < AVG_SIZE; j++)
      {
        if (i % 2 == 0)
        {
          xReadings[i] += analogRead(A0);
          yReadings[i] += analogRead(A1);
          zReadings[i] += analogRead(A2);
        }
        else
        {
          xReadings[i] += analogRead(A3);
          yReadings[i] += analogRead(A4);
          zReadings[i] += analogRead(A5);
        }
      }
      
      pinMode(outPin[i][0], INPUT);
      pinMode(outPin[i][1], INPUT);
      pinMode(outPin[i][2], INPUT);
      Serial.print("Redundancy ");
      Serial.print(i);
      Serial.print("\tx: ");
      Serial.print(xReadings[i]);
      Serial.print("\ty: ");
      Serial.print(yReadings[i]);
      Serial.print("\tz: ");
      Serial.print(zReadings[i]);
      
      float Vx = (float)xReadings[i]/maxADC * ADCvoltage;
      float Vy = (float)yReadings[i]/maxADC * ADCvoltage;
      float Vz = (float)zReadings[i]/maxADC * ADCvoltage;
      xResistance[i] = Rconst*(ADCvoltage - Vx)/Vx;
      yResistance[i] = Rconst*(ADCvoltage - Vy)/Vy;
      zResistance[i] = Rconst*(ADCvoltage - Vz)/Vz;
      xIntensity[i] = 1e6 / xResistance[i];
      yIntensity[i] = 1e6 / yResistance[i];
      zIntensity[i] = 1e6 / zResistance[i];
      Serial.prnt("\tRx: ");
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
    }
    lastT = micros();
  }
}
