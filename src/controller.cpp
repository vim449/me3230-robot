// TODO, generalize SISO control to decentralized MIMO control with scalar gains
#include <Arduino.h>
// L298N::L298N(unsigned char ENpin, unsigned char Cpin, unsigned char Dpin)
//: _PWM(ENpin), _Cpin(Cpin), _Dpin(Dpin), _flip(false) {}

class SISOPIDControl {
public:
  SISOPIDControl(double Kp, double Ki, double Kd, double min, double max)
      : _K1(Kp + Kd + Kp), _K2(-Kp - 2 * Kd), _K3(Kd), _e(0), _e1(0), _e2(0),
        _u(0), _min(min), _max(max) {}

  void update(double actualState, double targetState) {
    _e2 = _e1;
    _e1 = _e;
    _e = actualState - targetState;
    _u += (_K1 * _e + _K2 * _e1 + _K3 * _e2);
    _u = constrain(_u, _min, _max);
  }

  void reset() {
    _u = 0;
    _e1 = 0, _e2 = 0;
  }

private:
  double _K1, _K2, _K3;
  double _e, _e1, _e2;
  double _u;
  double _min, _max;
};

class SISOPDControl {};