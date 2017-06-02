#ifndef TWIDDLER_H
#define TWIDDLER_H

#include <vector>

class Twiddler {
 public:
  int counter_event = 0;
  bool settled = false;
  double error = 0;
  double best_error = -1.0;

  std::vector<double> params; // = [p, d, i];
  std::vector<double> best_params; // = deepcopy(self.params);
  std::vector<double> delta_params; // = [0.1, 0.1, 0.1];

  double adjustment_allowance = 0.007;
  int next_param = -1;
  int previous_adjustment = 1;
  int STABILIZATION = 500;       /* increase the time of observation before adjusting, to be smoothier*/
  int COLLECTION = 50;

  /*
   * Constructor
   */

  Twiddler(double p_coeff, double d_coeef, double i_coeff);

  /*
   * Destructor.
   */
  virtual ~Twiddler();

  bool process(double cte);

 private:
  void reset();

  void evaluate_and_adjust();

  void start_adjust_next_parameter(int current);
};


#endif /* TWIDDLER_H */
