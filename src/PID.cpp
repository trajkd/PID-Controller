#include "PID.h"
#include <iostream>
#include <stdlib.h>
PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * Initialize PID coefficients (and errors, if needed)
   */
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;

  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
  
  double best_err = 9999.9;
}

void PID::UpdateError(double cte) {
  /**
   * Update PID errors based on cte.
   */
  d_error = cte - p_error;
  i_error += cte;
  p_error = cte;
}

void PID::Twiddle(double cte) {
  double p[3] = {0., 0., 0.};
  double dp[3] = {1., 1., 1.};
  Kp = p[0];
  Ki = p[1];
  Kd = p[2];
  UpdateError(cte);
  best_err = TotalError();
  double tol = 0.2;
  int it = 0;
  while ((dp[0] + dp[1] + dp[2]) > tol) {
    // std::cout << (dp[0] + dp[1] + dp[2]) << " Iteration " << it << ", best error = " << best_err << std::endl;
    // std::cin.ignore();
    for (int i = 0; i < sizeof(p)/sizeof(p[0]); i++) {
      p[i] += dp[i];
      UpdateError(cte);
      best_err = TotalError();
      if (cte < best_err) {
        best_err = cte;
        dp[i] *= 1.1;
      }
      else {
        p[i] -= 2*dp[i];
        UpdateError(cte);
        best_err = TotalError();
        if (cte < best_err) {
          best_err = cte;
          dp[i] *= 1.1;
        }
        else {
          p[i] += dp[i];
          dp[i] *= 0.9;
        }
      }
    }
    //std::cout << p[0] << " " << p[1] << " " << p[2] << std::endl;
    //std::cin.ignore();
    it += 1;
  }
  Kp = p[0];
  Ki = p[1];
  Kd = p[2];
}

double PID::TotalError() {
  /**
   * Calculate and return the total error
   */
  return -Kp * p_error - Kd * d_error - Ki * i_error;
}