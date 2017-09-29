#include <iostream>
#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    p_error_ = 0;
    i_error_ = 0;
    d_error_ = 0;

    Kp_ = Kp;
    Ki_ = Ki;
    Kd_ = Kd;

    prev_cte_ = 0;

    twiddle_initialized_ = false;

    Kpid_[0] = Kp_;
    Kpid_[1] = Ki_;
    Kpid_[2] = Kd_;
}

void PID::InitTwiddle(double alpha, double beta, double tolerance, int max_iter, int num_iter_check) {
    delta_Kpid_[0] = Kpid_[0]*alpha;
    delta_Kpid_[1] = Kpid_[1]*alpha;
    delta_Kpid_[2] = Kpid_[2]*alpha;

    beta_ = beta;
    
    tolerance_ = tolerance;
    max_iter_ = max_iter;
    iter_ = 0;
    wait_iter_ = 0;
    param_idx_ = 0;
    num_iter_check_ = num_iter_check;
    //start with high error
    err_ = 9.9E9;
    accu_err_ = 0;
    best_err_ = 9.9E9;

    step_ = 0;
    next_step_ = 0;

    twiddle_initialized_ = true;              
}
       
void PID::UpdateTwiddle(double err) {
    //this function is called in main loop, so looping turns into if statements
    //this function will only calculate if twiddle is initialized
    double err_sum = 0;
    //cout << "step " << step_ << endl;
    //cout << "err " << err_ << endl;
    //cout << "iter " << iter_ << endl;
    //cout << "kp " << Kp_ << " ki " << Ki_ << " kd " << Kd_ << endl;

    switch(step_) {
        case 0:
            if (twiddle_initialized_) {
                next_step_ = 1;
            } else {
                next_step_ = 0;
            }
            break;
        case 1:
        //Step 1: should check tolerance (sum(dP) > tol) and max iter, if any is reached go back to step 0. 
        //Else: P[param_idx] += dP[param_idx] //add corresponding delta based on param_idx; go to step 2
            err_sum = delta_Kpid_[0] + delta_Kpid_[1] + delta_Kpid_[2];
            if ((iter_ < max_iter_) && (err_sum > tolerance_)) {
                Kpid_[param_idx_] += delta_Kpid_[param_idx_];
                UpdatePID(Kpid_);
                next_step_ = 2;
            } else {
                //reset init
                twiddle_initialized_ = false;
                next_step_ = 0;
            }
            break;
        case 2:
        //step 2: wait for num_iter_check before looking at avg squared error; go to step 3:
            if (wait_iter_ < num_iter_check_) {
                //wait and accumulate err
                accu_err_ += err*err;
                wait_iter_++;
            } else {
                err_ = accu_err_/num_iter_check_;
                //reset accumulated error because err_ before next step
                accu_err_ = 0;
                wait_iter_ = 0;
                next_step_ = 3;
            }
            break;
        case 3:
        //step 3: 
            if (err_ < best_err_) {
                best_err_ = err_;
                delta_Kpid_[param_idx_] *= (1.0 + beta_);
                param_idx_++;
                next_step_ = 1;
            } else {
                Kpid_[param_idx_] -= 2*delta_Kpid_[param_idx_];
                UpdatePID(Kpid_);
                next_step_ = 4;
            }
            break;
        case 4:
        //step 4: wait for num_iter_check before looking at avg squared error; go to step 5:
            if (wait_iter_ < num_iter_check_) {
                //wait and accumulate err
                accu_err_ += err*err;
                wait_iter_++;
            } else {
                err_ = accu_err_/num_iter_check_;
                //reset accumulated error because err_ before next step
                accu_err_ = 0;
                wait_iter_ = 0;
                next_step_ = 5;
            }
            break;
        case 5:
        //step 5: 
            if (err_ < best_err_) {
                best_err_ = err_;
                delta_Kpid_[param_idx_] *= (1.0 + beta_);
            } else {
                Kpid_[param_idx_] += delta_Kpid_[param_idx_];
                UpdatePID(Kpid_);
                delta_Kpid_[param_idx_] *= (1.0 - beta_);
            }
            param_idx_++;
            next_step_ = 1;
            break;
    }
    //if param_idx_ >= 3 then reset it to 0
    if (param_idx_ >= NUM_PARAMS) {
        param_idx_ = 0;
        iter_++;
    }
    step_ = next_step_;  
}

void PID::UpdatePID(double Kpid[]) {
    Kp_ = Kpid[0];
    Ki_ = Kpid[1];
    Kd_ = Kpid[2];
}

void PID::UpdateError(double cte) {
    p_error_ = cte;
    i_error_ += cte;
    d_error_ = cte - prev_cte_;
    prev_cte_ = cte;
}

double PID::TotalError() {
    return Kp_*p_error_ + Ki_*i_error_ + Kd_*d_error_;
}

double PID::GetP() {
    return Kp_;
}

double PID::GetI() {
    return Ki_;
}

double PID::GetD() {
    return Kd_;
}

bool PID::IsTwiddleInitialized() {
    return twiddle_initialized_;
}

