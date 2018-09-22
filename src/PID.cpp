#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd)
{
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
}

void PID::UpdateError(double cte)
{
  // d_error = cte - previous_cte
  d_error = cte - p_error; //Since p_error from last cycle is previous cte
  p_error = cte;
  i_error += cte;
}

double PID::TotalError()
{
  return (Kp*p_error + Ki*i_error + Kd*d_error);
  
}

/*
void PID::Twiddle(double tol, double dKp, double dKi, double dKd, double cte)
{
  double best_error, error;
  UpdateError(cte);
  best_error = TotalError();
  
  
  std::cout << "PID gains initial:  Kp = " << this->Kp << "   Ki = " << this->Ki << "   Kd = " << this->Kd << std::endl;
  
  while( (dKp + dKi + dKd) > tol)
  {
    // Modify Kp
    this->Kp += dKp;
    UpdateError(cte);
    error = TotalError();
    
    if(error < best_error)
    {
      best_error = error;
      dKp *= 1.1;
    }
    else
    {
      this->Kp -= 2*dKp;
      UpdateError(cte);
      error = TotalError();
      if(error < best_error)
      {
        error = best_error;
        dKp *= 1.1;
      }
      else
      {
        this->Kp += dKp;
        dKp *= 0.9;
      }
    }
    
    // Modify Ki
    this->Ki += dKi;
    UpdateError(cte);
    error = TotalError();
    
    if(error < best_error)
    {
      best_error = error;
      dKi *= 1.1;
    }
    else
    {
      this->Ki -= 2*dKi;
      UpdateError(cte);
      error = TotalError();
      if(error < best_error)
      {
        error = best_error;
        dKi *= 1.1;
      }
      else
      {
        this->Ki += dKi;
        dKi *= 0.9;
      }
    }
    
    // Modify Kd
    this->Kd += dKd;
    UpdateError(cte);
    error = TotalError();
    
    if(error < best_error)
    {
      best_error = error;
      dKd *= 1.1;
    }
    else
    {
      this->Kd -= 2*dKd;
      UpdateError(cte);
      error = TotalError();
      if(error < best_error)
      {
        error = best_error;
        dKd *= 1.1;
      }
      else
      {
        this->Kd += dKd;
        dKd *= 0.9;
      }
    }
    
  }
   std::cout << "PID gains final:  Kp = " << this->Kp << "   Ki = " << this->Ki << "   Kd = " << this->Kd << std::endl;
}
 */

