#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  Kp = Kp;
  Ki = Ki;
  Kd = Kd;
  previous_cte = 0;
  total_cte = 0;
}

void PID::UpdateError(double cte) {
  p_error = Kp * cte;
  d_error = Kd * (cte - previous_cte);
  previous_cte = cte;
  total_cte += cte;
  d_error = Ki * total_cte;
}

double PID::TotalError() {
  double total = -p_error - d_error - p_error;
  return total;
}

