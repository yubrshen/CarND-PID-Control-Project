#include "Twiddler.h"
#include <iostream>
#include <math.h>

double sum(std::vector<double> v) {
  double s = 0;
  for (auto& n : v)
    s += n;
  return s;
}

void print_something(std::vector<double> x) {
  for (auto const& c : x)
    std::cout << c << ' ';
}

Twiddler::Twiddler(double p_coeff, double d_coeef, double i_coeff) {
  params.push_back(p_coeff);
  params.push_back(d_coeef);
  params.push_back(i_coeff);

  best_params = params; // assume to be deepcopy in C++

  // so far the working adjustment ranges: [0.1, 0.1, 0.1]
  delta_params.push_back(1.5);
  delta_params.push_back(1.5);
  delta_params.push_back(1.5);
  }

Twiddler::~Twiddler() {}

bool Twiddler::process(double cte) {
  if (settled) return false;    // adjustment_needed = false bypass the twiddle process
  bool adjustment_needed = false;
  double worst_error_permitted = 3.85;
  if (worst_error_permitted < fabs(cte)) {
    error = 999999.0; // the error is inadmissible
    adjustment_needed = true; // early terminate the process
  } else {
    if (counter_event < STABILIZATION) {
      ;
    }
    else {
      int collected = counter_event - STABILIZATION;
      if (-1 == collected) {      // start to collect the error, and normalize per session
        error += cte * cte;
      } else {
        error = ((error * collected) + cte * cte)/float(collected + 1);
        //std::cout << "error: " << error << std::endl;
      }
      adjustment_needed = ((-1 < best_error) && ((best_error + 0.1) < error)); // hopelessly large, early terminate
    }
    adjustment_needed = (adjustment_needed || (counter_event == (COLLECTION + STABILIZATION)));
  }
  if (adjustment_needed) {
    double s = sum(delta_params);
    settled = (s < adjustment_allowance); /* need to implementation sum of vector elements */
    if (!settled) {
      evaluate_and_adjust();
      reset();
    } else {
      params = best_params;   /*  assume to be deepcopy*/
      std::cout << "settled down on twiddle: the best params: ";
      print_something(params);
      std::cout << std::endl;
    }

    std::cout << "sum(delta_params): " << s << " adjustment_allowance: " << adjustment_allowance << " best_error: " << best_error << std::endl;
  }
  counter_event += 1;
  return adjustment_needed;
}

void Twiddler::reset() {
  counter_event = -1; // so that the next meaningful count starts with 0
  // std::cout << "reset error to 0" << std::endl;
  error = 0;
}

void Twiddler::evaluate_and_adjust() {
  // std::cout << "in evaluate_and_adjust: error: " << error << std::endl;
  if ((best_error == -1.0) || (error < best_error)) { // improved
    if (best_error == -1.0) {
      std::cout << "initial set to best_error: " << best_error << " to " << error << std::endl;
      best_error = error;     // this staement cannot be factored out,
      // as best_error cannot be changed before test against its current value
      start_adjust_next_parameter(-1);
    } else {                  // (error < best_error)
      std::cout << "improved overall error: " << best_error << " original error: " << error << " proposed params: ";
      print_something(params);
      best_error = error;
      best_params = params; // commit the exploration, assume deepcopy
      std::cout << " committed params: ";
      print_something(best_params);
      std::cout << std::endl;
      delta_params[next_param] *= 1.1;
      start_adjust_next_parameter(next_param);
    }
  } else {                // worsened
    std::cout << "abandon changed: ";
    print_something(params);
    std::cout << " keep the best_params: ";
    print_something(best_params);
    std::cout << std::endl;
    params = best_params;  // abandon the previous experiment, assume deepcopy
    // std::cout << "after withdraw, params: ";
    // print_something(params);
    // std::cout << std::endl;
    if (previous_adjustment == 1) { // the previous adjustment is increase, then there is decrease to try
      // params[next_param] += -delta_params[next_param];
      if (params[next_param] == 0.0) {
        params[next_param] += -delta_params[next_param];
      } else {
        params[next_param] *= (1 - delta_params[next_param]);
      }
      std::cout << "Testing: ";
      print_something(params);
      std::cout << std::endl;

      //if (next_param == 2) std::cout << "decrease next_param: " << next_param << std::endl;
      previous_adjustment = -1;
    } else { // previous_adjustment == -1, already exhausted the adjustment for the parameter
      delta_params[next_param] *= 0.9;
      start_adjust_next_parameter(next_param);
    }
  }
}

void Twiddler::start_adjust_next_parameter(int current) {
    next_param = (current +1) % params.size();
    // params[next_param] += delta_params[next_param];
    if (params[next_param] == 0.0) {
      params[next_param] += delta_params[next_param];
    } else {
      params[next_param] *= (1 + delta_params[next_param]);
    }
    std::cout << "Testing: ";
    print_something(params);
    std::cout << std::endl;
    //if (next_param == 2) std::cout << "increase next_param: " << next_param << std::endl;
    previous_adjustment = 1;
  }
