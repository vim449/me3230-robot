#pragma once

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
#define TIMERS_AVAILABLE
#endif

#include <Arduino.h>

class L298N {
public:
  // CONSTRUCTORS
  // User-defined pin selection.
  L298N(unsigned char ENpin, unsigned char Cpin, unsigned char Dpin);

  // PUBLIC METHODS
  void init();                // Initialize pins and timer if applicable.
  void setSpeed(int speed); // Set speed for M1.
  // void setM2Speed(int speed); // Set speed for M2.
  // void setSpeeds(int m1Speed, int m2Speed); // Set speed for both Motors.
  void flip(boolean flip);  // Flip the direction of the speed for M1.
  // void flipM2(boolean flip);  // Flip the direction of the speed for M2.
  void setBrake(int brake); // brake motor 1
  // void setM2Brake(int brake); // brake motor 2
  // void setBrakes(int m1Brake, int m2Brake); // set brakes on both motors

private:
  // motor pins
  unsigned char _PWM;
  unsigned char _Cpin;
  unsigned char _Dpin;
  // timer pins
  static const unsigned char TIMER1_PIN1 = 11;
  static const unsigned char TIMER1_PIN2 = 12;
  static const unsigned char TIMER3_PIN1 = 5;
  static const unsigned char TIMER3_PIN2 = 2;
  static const unsigned char TIMER4_PIN1 = 6;
  static const unsigned char TIMER4_PIN2 = 7;
  static const unsigned char TIMER5_PIN1 = 45;
  static const unsigned char TIMER5_PIN2 = 46;

  boolean _flip;
  boolean _isOutput1;
  int _timer;
};
