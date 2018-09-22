#ifndef PID_H
#define PID_H


class PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */ 
  double Kp;
  double Ki;
  double Kd;
  
  //double dKp;
  //double dKi;
  //double dKd;

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();
  
  /*
   * Compute optimum P,I,D gains(adaptive).
   */
  //void Twiddle(double tol, double dKp, double dKi, double dKd, double cte);

};

#endif /* PID_H */
