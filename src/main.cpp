#include <Arduino.h>
#include <QTRSensors.h>

// followline PID algorithm for Bricolabs Raptor robot
// Arduino Pro Micro, TB6612 driver, Pololu QTR-8 sensor

// TB6612 driver pinout

// Botón start
const int botonStart = 7;
boolean botonPresionado = false;

// Motor A
const int STBY = 15; // standby
const int PWMA = 5; // speed and direction control motor A (left)
const int AIN1 = 2;
const int AIN2 = 3;

// Motor B
const int PWMB = 10; // speed and direction control motor B (right)
const int BIN1 = 16;
const int BIN2 = 14;

// Pololu QTR-8A analog array readout
QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

// parameters and variables for non linear PID
const int vmin=80;
const int vmax=150;
const float kp=.015;
const float ki=0.0003;
const float kd=0.2;
const float kv=0.07;
int p,d,u,vbase;
long i=0;
int p_old=0;

void drive(int L, int R) { // speed for wheels Left and Right, positive is forward
  L=constrain(L,-255,255); // avoid PWM overflow
  R=constrain(R,-255,255);
  
  digitalWrite(AIN1, L<0); // switch < and >= if left wheel doesnt spin as expected
  digitalWrite(AIN2, L>=0);
  analogWrite(PWMA, abs(L));
  
  digitalWrite(BIN1, R<0); // switch < and >= if left wheel doesnt spin as expected
  digitalWrite(BIN2, R>=0);
  analogWrite(PWMB, abs(R));
}

void setup() {
  pinMode(botonStart, INPUT);
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]) {A6, A7, A8, A9, A0, A1, A2, A3}, SensorCount);

  while (!botonPresionado) {
    if (digitalRead(botonStart) == HIGH)
      botonPresionado = true;
    }
  delay(1000);  // esperamos 1 segundo
}

void loop() {

  digitalWrite(STBY, HIGH); 
  qtr.read(sensorValues); // read raw sensor values

  
  // print the sensor values as numbers from 0 to 1023, where 0 means maximum
  // reflectance and 1023 means minimum reflectance
  /*
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  Serial.println();
  */
  
  p = -7*sensorValues[0]-5*sensorValues[1]-3*sensorValues[2]-sensorValues[3]+sensorValues[4]+3*sensorValues[5]+5*sensorValues[6]+7*sensorValues[7];
  i=i+p;
  d=p-p_old;
  p_old=p;
  if ((p*i)<0) i=0;  // integral windup

  u=kp*p+ki*i+kd*d;
  vbase=vmin+(vmax-vmin)*exp(-kv*abs(kp*p));
  
  /*// Test vel
  Serial.print(vbase+u);
  Serial.print(" ; ");
  Serial.println(vbase-u);
  */
  drive(vbase+u,vbase-u);
}