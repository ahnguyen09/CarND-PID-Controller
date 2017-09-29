#ifndef PID_H
#define PID_H

class PID {
private:
  /*
  * Errors
  */
  double p_error_;
  double i_error_;
  double d_error_;

  /*
  * Coefficients
  */ 
  double Kp_;
  double Ki_;
  double Kd_;

  /*
  * Twiddle variables
  */
  double tolerance_; 
  bool twiddle_initialized_;
  double Kpid_[3];
  double delta_Kpid_[3];

  /*
  * Twiddle controller variables
  */
  int step_;
  int next_step_;
  int param_idx_;
  int iter_;
  int wait_iter_;
  int max_iter_; 
  int num_iter_check_;
  const int NUM_PARAMS = 3;
  double err_;
  double accu_err_;
  double best_err_;
  double beta_;
  
  /*
  * used to store previous cte (crosstrack error)
  */
  double prev_cte_;

public:


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
  * Initialize twiddle.
  */
  void InitTwiddle(double alpha, double beta, double tolerance, int max_iter, int num_iter_check);

  /*
  * update twiddle.
  */
  //twiddle step (case) controller
  //initial step 0: wait in step 0 Until twiddle is initialized then go to step 1; param_idx = 0;
  //Step 1: should check tolerance and max iter, if any is reached go back to step 0. 
  //        Else: P[param_idx] += dP[param_idx] //add corresponding delta based on param_idx; go to step 2
  //step 2: wait for num_iter_check before looking at avg squared error; go to step 3:
  //step 3: if err< best_err: 
  //          best_err = err
  //          dP[i] *= (1+ 0.1(beta))
  //          param_idx += 1
  //          next_step = 1
  //        else:
  //          P[i] -= 2 *dP[i]
  //          next_step = 4
  //step 4: wait for num_iter_check before looking at avg squared error; go to step 5:
  //step 5: if err< best_err:
  //          best_err = err
  //			dP[i] *= (1+ 0.1(beta))
  //        else:
  //          P[i] += dP[i]
  //			dP[i] *= (1- 0.1(beta))
  //        param_idx += 1
  //        next_step = 1
  //if param_idx >= 3 then reset it to 0
  //set step = next_step
  void UpdateTwiddle(double err);

  /*
  * Update PID Coefficients.
  */
  void UpdatePID(double Kpid[]);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  double GetP();

  double GetI();
  
  double GetD();

  bool IsTwiddleInitialized();
};

#endif /* PID_H */
