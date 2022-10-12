#include <Arduino.h>
#include <BlueMotor.h>
#include <Romi32U4.h>

long oldValue = 0;
long newValue;
long count = 0;
unsigned time = 0;

static int ENCA_PIN = 0;
static int ENCB_PIN = 1;

static int ENCA_VAL;
static int ENCB_VAL;

BlueMotor::BlueMotor()
{
}

void BlueMotor::setup()
{
    pinMode(PWMOutPin, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(ENCA_PIN, INPUT);
    pinMode(ENCB_PIN, INPUT);
    ENCA_VAL = digitalRead(ENCA_PIN);
    ENCB_VAL = digitalRead(ENCB_PIN);
    TCCR1A = 0xA8; //0b10101000; //gcl: added OCR1C for adding a third PWM on pin 11
    TCCR1B = 0x11; //0b00010001;
    ICR1 = 400;
    OCR1C = 0;

    attachInterrupt(digitalPinToInterrupt(ENCA_PIN), isrA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCB_PIN), isrB, CHANGE);
    reset();
}

long BlueMotor::getPosition()
{
    long tempCount = 0;
    noInterrupts();
    tempCount = count;
    interrupts();
    return tempCount;
}

void BlueMotor::reset()
{
    noInterrupts();
    count = 0;
    interrupts();
}


void BlueMotor::isrA()
{
  ENCA_VAL = digitalRead(ENCA_PIN);
  if (ENCB_VAL == ENCA_VAL) {
    count++;
  }
  else {
    count--;
  }
}

void BlueMotor::isrB()
{
  ENCB_VAL = digitalRead(ENCB_PIN);
  if (ENCA_VAL == ENCB_VAL) {
    count--;
  }
  else {
    count++;
  }
}

void BlueMotor::setEffort(int effort)
{
    if (effort < 0)
    {
        setEffort(-effort, true);
    }
    else
    {
        setEffort(effort, false);
    }
}

void BlueMotor::setEffort(int effort, bool clockwise)
{
    if (effort < 70 && effort != 0){
      effort = 70;
    }
    if (clockwise)
    {
        digitalWrite(AIN1, HIGH);
        digitalWrite(AIN2, LOW);
    }
    else
    {
        digitalWrite(AIN1, LOW);
        digitalWrite(AIN2, HIGH);
    }
    OCR1C = constrain(effort, 0, 400);
}

void BlueMotor::moveTo(long target)  //Move to this encoder position within the specified
{                                    //tolerance in the header file using proportional control
                                     //then stop

    long delta = target - this->getPosition();
    while (abs(delta) > this->tolerance){
        delta = target - this->getPosition();
        this->setEffort(delta * this->P);
    }
    
    this->setEffort(0);
}

bool BlueMotor::moveToNB(long target) {
  long delta = target - this->getPosition();
  if(abs(delta) > this->tolerance){
    this->setEffort(constrain(delta * this->P, -400, 400));
    return false;
  } else {
    this->setEffort(0);
    return true;
  }
}

float BlueMotor::get_rpm(int dt, long dp){
    return ((float(dp)/this->CPR)/(dt/1000.0/60.0));
}
