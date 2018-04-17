#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(std::vector<double>& params) {
  Kp = params[0];
  Ki = params[1];
  Kd = params[2];
  p_error = 0;
  d_error = 0;
  i_error = 0;
}

void PID::UpdateError(double cte) {
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;
}

double PID::ComputeControl() {
  return - Kp * p_error - Kd * d_error - Ki * i_error;
}

