#ifndef PID_H
#define PID_H

static constexpr int    NUM_COEF = 3;
static constexpr double THROTTLE = 0.6;

class PID {
 public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();

  /**
   * Get the average error so far
   */
  double AverageError();

  /**
   * Print internal coefficients
   */
  void PrintCoefficients();

 private:
  /**
   * PID Errors
   */
  long   count;
  double p_error;
  double i_error;
  double d_error;

  /**
   * PID Coefficients
   */ 
  double K[NUM_COEF];
};

#endif  // PID_H
