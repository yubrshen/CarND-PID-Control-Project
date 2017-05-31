#include "Twiddler.h"

double sum(std::vector<double> v) {
  double s = 0;
  for (auto& n : v)
    s += n;
  return s;
}

Twiddler::Twiddler(double p_coeff, double d_coeef, double i_coeff) {
  params.push_back(p_coeff);
  params.push_back(d_coeef);
  params.push_back(i_coeff);

  best_params = params; // assume to be deepcopy in C++

  delta_params.push_back(1.0);
  delta_params.push_back(1.0);
  delta_params.push_back(1.0);
  }

Twiddler::~Twiddler() {}

bool Twiddler::process(double cte) {
  if (settled) return false;    // adjustment_needed = false bypass the twiddle process
  if (counter_event < STABILIZATION)
    ; // just keep running
  else {
    error += cte * cte;        // start to collect the error
  }
  bool adjustment_needed = (counter_event == (COLLECTION + STABILIZATION));
  if (adjustment_needed) {
    settled = (sum(delta_params) < adjustment_allowance); /* need to implementation sum of vector elements */
    if (!settled) {
      evaluate_and_adjust();
      reset();
    } else {
      params = best_params;   /*  assume to be deepcopy*/
    }
  }
  counter_event += 1;
  return adjustment_needed;
  }

void Twiddler::reset() {
    counter_event = -1; // so that the next meaningful count starts with 0
    error = 0;
  }

void Twiddler::evaluate_and_adjust() {
  if ((best_error == -1) || (error < best_error)) { // improved
    if (best_error == -1) {
      best_error = error;     // this staement cannot be factored out,
      // as best_error cannot be changed before test against its current value
      start_adjust_next_parameter(-1);
    } else {                  // (error < best_error)
      best_error = error;
      best_params = params; // commit the exploration, assume deepcopy
      delta_params[next_param] *= 1.1;
      start_adjust_next_parameter(next_param);
    }
  } else {                // worsened
    params = best_params;  // abandon the previous experiment, assume deepcopy
    if (previous_adjustment == 1) { // the previous adjustment is increase, then there is decrease to try
      params[next_param] += -delta_params[next_param];
      previous_adjustment = -1;
    } else { // previous_adjustment == -1, already exhausted the adjustment for the parameter
      delta_params[next_param] *= 0.9;
      start_adjust_next_parameter(next_param);
    }
  }
}


void Twiddler::start_adjust_next_parameter(int current) {
    next_param = (current +1) % params.size();
    params[next_param] += delta_params[next_param];
    previous_adjustment = 1;
  }
