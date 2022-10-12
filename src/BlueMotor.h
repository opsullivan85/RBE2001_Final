#pragma once

class BlueMotor
{
public:
    BlueMotor();
    void setEffort(int effort);
    void moveTo(long position);
    bool moveToNB(long position);
    float get_rpm(int dt, long dp);
    long getPosition();
    void reset();
    void setup();
    // const int CPR = 540;
    const int CPR = 553;

private:
    void setEffort(int effort, bool clockwise);
    static void isrA();
    static void isrB();
    const int tolerance = 3;
    const int PWMOutPin = 11;
    const int AIN2 = 4;
    const int AIN1 = 13;
    // const int ENCA = 0;
    // const int ENCB = 1;
    const float P = 10;
};