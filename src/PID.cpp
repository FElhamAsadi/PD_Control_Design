#include "PID.h"

/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_)
{
  //Initialize PID coefficients (and errors, if needed)
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;   

}

void PID::UpdateError(double cte) {
  // Update PID errors based on cte
  d_error = cte - p_error;  //difference from old cte to the new cte
  p_error = cte;             //proportional error
  i_error += cte;            //sum of the ctes 
}

double PID::TotalError() {
  //Calculate and return the total error
  return -Kp * p_error - Ki * i_error - Kd * d_error;
}


