#include "L298N.h"

L298N::L298N(unsigned char ENpin, unsigned char Cpin, unsigned char Dpin)
    : _PWM(ENpin), _Cpin(Cpin), _Dpin(Dpin), _flip(false) {}
// Public Methods //////////////////////////////////////////////////////////////
void L298N::init() {
  // Define pinMode for the pins and set the frequency for timer2.

  pinMode(_PWM, OUTPUT);
  pinMode(_Cpin, OUTPUT);
  pinMode(_Dpin, OUTPUT);
  _timer = 0;

#ifdef TIMERS_AVAILABLE
  // prescaler: clockI/O / 1
  // outputs enabled
  // phase-correct PWM
  // top of 400
  //
  // PWM frequency calculation
  // 16MHz / 1 (prescaler) / 2 (phase-correct) / 400 (top) = 20kHz
  volatile uint8_t *TCRA[5] = {&TCCR1A, nullptr, &TCCR3A, &TCCR4A, &TCCR5A};
  volatile uint8_t *TCRB[5] = {&TCCR1B, nullptr, &TCCR3B, &TCCR4B, &TCCR5B};
  volatile uint16_t *ICR[5] = {&ICR1, nullptr, &ICR3, &ICR4, &ICR5};
  if (_PWM == TIMER1_PIN1 || _PWM == TIMER1_PIN2) {
    _timer = 1;
    _isOutput1 = _PWM == TIMER1_PIN1;
  }
  if (_PWM == TIMER3_PIN1 || _PWM == TIMER3_PIN2) {
    _timer = 3;
    _isOutput1 = _PWM == TIMER3_PIN1;
  }
  if (_PWM == TIMER4_PIN1 || _PWM == TIMER4_PIN2) {
    _timer = 4;
    _isOutput1 = _PWM == TIMER4_PIN1;
  }
  if (_PWM == TIMER5_PIN1 || _PWM == TIMER5_PIN2) {
    _timer = 5;
    _isOutput1 = _PWM == TIMER5_PIN1;
  }
  *TCRA[_timer - 1] = 0b10100000;
  *TCRB[_timer - 1] = 0b00010001;
  *ICR[_timer - 1] = 400;
#endif
}

// Set speed for motor 1, speed is a number between -400 and 400
void L298N::setSpeed(int speed) {
  unsigned char forward = 1;

  if (speed < 0) {
    speed = -speed; // Make speed a positive quantity
    forward = 0;    // Preserve the direction
  }
  speed = speed > 400 ? 400 : speed; // max duty cycle
  if (forward ^ _flip) { // if flipped or negative speed but not both
    digitalWrite(_Cpin, HIGH);
    digitalWrite(_Dpin, LOW);
  } else {
    digitalWrite(_Cpin, LOW);
    digitalWrite(_Dpin, HIGH);
  }

#ifdef TIMERS_AVAILABLE
  // OCR5 is intentionally flipped
  volatile uint16_t *OCRA[] = {
      &OCR1A, nullptr, &OCR3A, &OCR4A, &OCR5B,
  };
  volatile uint16_t *OCRB[] = {
      &OCR1B, nullptr, &OCR3B, &OCR4B, &OCR5A,
  };
  if (_timer) {
    if (_isOutput1)
      *OCRA[_timer] = speed;
    else
      *OCRB[_timer] = speed;
  } else {
    analogWrite(_PWM, speed * 51 / 80); // map 400 to 255
  }
#else
  analogWrite(_PWM, speed * 51 / 80); // map 400 to 255
#endif
}

// Flip direction for motor 1
void L298N::flip(boolean flip) { _flip = flip; }

// Set brake for motor 1, brake is a number between 0 and 400, 0 corresponds to
// full coast, 400 corresponds to full brake
void L298N::setBrake(int brake) {
  digitalWrite(_Cpin, LOW);
  digitalWrite(_Dpin, LOW);
  brake = brake > 0 ? brake : -brake;
  brake = brake > 400 ? 400 : brake;
#ifdef TIMERS_AVAILABLE
  // OCR5 is intentionally flipped
  volatile uint16_t *OCRA[] = {
      &OCR1A, nullptr, &OCR3A, &OCR4A, &OCR5B,
  };
  volatile uint16_t *OCRB[] = {
      &OCR1B, nullptr, &OCR3B, &OCR4B, &OCR5A,
  };
  if (_timer) {
    if (_isOutput1)
      *OCRA[_timer - 1] = brake;
    else
      *OCRB[_timer - 1] = brake;
  } else {
    analogWrite(_PWM, brake * 51 / 80); // map 400 to 255
  }
#else
  analogWrite(_PWM, brake * 51 / 80); // map 400 to 255
#endif
}