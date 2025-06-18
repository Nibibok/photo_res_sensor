#include <math.h>
#include <Arduino.h>

constexpr uint32_t ledPin = 13;      // select the pin for the LED
int maxADC = pow(2,10) - 1;
float ADCvoltage = 5.0; // max voltage a pin reads is 5.
constexpr float period = pow(10,6)/1;// readout period in microseconds (1 s rn)
uint32_t lastT = 0; 
constexpr float Rconst = 10000;// resistance (ohm) of the constant resistor 
constexpr int bitres = 10;
int[3] xVec = {1,0,0}; // vector component of a sensor side
int[3] yVec = {0,1,0};
int[3] zVec = {0,0,1};

void setup() {
  // declare the ledPin as an OUTPUT:
  //pinMode(ledPin, OUTPUT);
  Serial.begin(9600);
}


void loop() {
  uint32_t countT = micros();
  if (countT - lastT >= period){
    // read the value from the sensor:
    int  sensor0 = analogRead(A0);
    int  sensor1 = analogRead(A1);
    // Led on
    digitalWrite(ledPin, HIGH);
    // print sensorvalues
    Serial.print("sensor0");
	  Serial.print(" , ");
    Serial.print("sensor1");
    Serial.print(" , ");
    Serial.print("V0");
	  Serial.print(" , ");
    Serial.print("V1");
    Serial.print(" , ");
    Serial.print("R0");
	  Serial.print(" , ");
    Serial.println("R1");
    Serial.print(sensor0);
	  Serial.print(" , ");
    Serial.print(sensor1);
    Serial.print(" , ");
    // calculate V over sensor voltages
    float V0 = (float)sensor0/maxADC * ADCvoltage;
    float V1 = (float)sensor1/maxADC * ADCvoltage;
    // print over sensor voltages
    Serial.print(V0);
	  Serial.print(" , ");
    Serial.print(V1);
    Serial.print(" , ");
    // Calculate photoresistane
    float R0 = Rconst*(ADCvoltage - V0)/V0;
    float R1 = Rconst*(ADCvoltage - V1)/V1;
    Serial.print(R0);
	  Serial.print(" , ");
    Serial.println(R1);
    // Somewhere convert to relative light strenght
    digitalWrite(ledPin, LOW);
    lastT = micros();
  }
}
