#include "PID.h"
#include <iostream>
#include <cmath>

PID::PID() : count(0), p_error(0), i_error(0), d_error(0), K{} {}

PID::~PID() {}

/**
 * Initialize PID coefficients (and errors, if needed)
 */
void PID::Init(double Kp_, double Ki_, double Kd_) {
  K[0] = Kp_;
  K[1] = Kd_;
  K[2] = Ki_;
}

/**
 * Update PID errors based on cte.
 */
void PID::UpdateError(double cte) {
  ++count;
  d_error = cte - p_error;
  if (K[2] > 0) i_error += cte;
  p_error = cte;
}
/**
 * Get the average error so far
 */
double PID::AverageError() {
  return i_error / count;
}


/**
 * Calculate and return the total error
 */
double PID::TotalError() {
  return -K[0] * p_error - K[1] * d_error - K[2] * i_error;
}

/**
 * Print internal coefficients
 */
void PID::PrintCoefficients() {
  std::cout << "Kp: " << K[0] << " Kd: " << K[1] << " Ki: " << K[2] << std::endl;
}

