#include <servo32u4.h>

#define open_position 1900
#define closed_position 1035
#define safe_position open_position

uint16_t Servo32U4Base::setMinMaxMicroseconds(uint16_t min, uint16_t max)
{
    // swap if in the wrong place
    if(min > max) {uint16_t temp = min; min = max; max = temp;}

    usMin = min;
    usMax = max;

    return usMax - usMin; //return the range, in case the user wants to do a sanity check
}

void Servo32U4Pin5::attach(void) 
{
    pinMode(5, OUTPUT); // set pin as OUTPUT

    cli();

    // clear then set the OCR3A bits (pin 5)
    TCCR3A = 0x82; //WGM
    TCCR3B = 0x1A; //WGM + CS = 8
    ICR3 = 39999; //20ms
    OCR3A = 3000;

    sei();

    isAttached = true;
}

void Servo32U4Pin5::detach(void)
{
    cli();

    // clear the OCR3A bits
    TCCR3A &= 0x7f; //cancel OCR3A
    sei();

    isAttached = false;
}

void Servo32U4Pin5::writeMicroseconds(uint16_t microseconds)
{
    if (!isAttached)
    {
        attach();
    }

    microseconds = constrain(microseconds, usMin, usMax);

    //prescaler is 8, so 1 timer count = 0.5 us
    OCR3A = (microseconds << 1) - 1; // multiplies by 2
}

void Servo32U4Pin6::attach(void) 
{
    pinMode(6, OUTPUT); // set pin as OUTPUT

    cli();

    // Be careful here, since Timer4 is used to manage speed controller. See Chassis::init()
    TCCR4C |= 0x05;

    sei();

    isAttached = true;
}

void Servo32U4Pin6::detach(void) 
{
    cli();

    // Be careful here, since Timer4 is used to manage speed controller. See Chassis::init()
    TCCR4C = 0x00;

    sei();

    isAttached = false;
}

// Resolution is 64 us; not great, but shouldn't be too constraining
void Servo32U4Pin6::writeMicroseconds(uint16_t microseconds)
{
    if (!isAttached)
    {
        attach();
    }

    microseconds = constrain(microseconds, usMin, usMax);

    //prescaler is 8, so 1 timer count = 64 us
    OCR4D = (microseconds >> 6) - 1; // divides by 64
}

void Servo32U4Pin13::attach(void) 
{
    pinMode(13, OUTPUT); // set pin as OUTPUT

    cli();

    // Be careful here, since Timer4 is used to manage speed controller. See Chassis::init()
    TCCR4A = 0x82;

    sei();

    isAttached = true;
}

void Servo32U4Pin13::detach(void) 
{
    cli();

    // Be careful here, since Timer4 is used to manage speed controller. See Chassis::init()
    TCCR4A = 0x00;

    sei();

    isAttached = false;
}

// Resolution is 64 us; not great, but shouldn't be too constraining
void Servo32U4Pin13::writeMicroseconds(uint16_t microseconds)
{
    if (!isAttached)
    {
        attach();
    }

    microseconds = constrain(microseconds, usMin, usMax);

    //prescaler is 8, so 1 timer count = 64 us
    OCR4A = (microseconds >> 6) - 1; // divides by 64
}

void Servo32U4Pin12::attach(void) 
{
    pinMode(12, OUTPUT); // set pin as OUTPUT

    cli();

    // Be careful here, since Timer4 is used to manage speed controller. See Chassis::init()
    TCCR4C |= 0x05;

    sei();

    isAttached = true;
}

void Servo32U4Pin12::detach(void) 
{
    cli();

    // Be careful here, since Timer4 is used to manage speed controller. See Chassis::init()
    TCCR4C = 0x00;

    sei();

    isAttached = false;
}

// Resolution is 64 us; not great, but shouldn't be too constraining
void Servo32U4Pin12::writeMicroseconds(uint16_t microseconds)
{
    if (!isAttached)
    {
        attach();
    }

    microseconds = constrain(microseconds, usMin, usMax);

    //prescaler is 8, so 1 timer count = 64 us
    OCR4D = 250 - (microseconds >> 6) - 1; // divides by 64
}

/// @brief converts microseconds to resistor values
/// @param pos 
/// @return resistor value
int Servo32U4Base::microsec_to_pos(int pos)
{
    int resistor_value = .183*pos + 12;
    return resistor_value;
}



/// @brief Goes to position and returns true if reached, if stuck returns false and moves to safe position
/// @param pos position to go to
/// @return success status of move
bool Servo32U4Base::grabber_goto(int pos)
{
    int curr_pos = analogRead(Servo_sense);
    int prev_pos = curr_pos + 5;    //sets prev position a little different to avoid exiting while loop
    int tar_pos = microsec_to_pos(pos);  //converts position in microseconds to resistor value
    this->writeMicroseconds(pos);
    while(true){
        curr_pos = analogRead(Servo_sense);
        if(abs(curr_pos - prev_pos)<2 && abs(curr_pos - tar_pos)>10){  //if prev val and curr val are too close and not within a range of the tar val open gripper
            return false;
        }
        else if(abs(curr_pos - tar_pos)<10) {   //if the grabber reaches a range of the tar val return true
            return true;
        }
        prev_pos = curr_pos;
        delay(20);
    }
}

/// @brief Opens the grabber
/// @return success status of move
bool Servo32U4Base::grabber_open()  //method to open grabber
{
    return grabber_goto(1900);
}

/// @brief Closes the grabber
/// @return success status of move
bool Servo32U4Base::grabber_close() //method to close grabber
{
    return grabber_goto(1035);
}