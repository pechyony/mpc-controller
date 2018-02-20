#ifndef MPC_H
#define MPC_H

#include <vector>
#include <cppad/cppad.hpp>
#include "Eigen-3.3/Eigen/Core"

using namespace std;
using CppAD::AD;

// auxiliary class to compute objective function and constraints, used by IPOPT solver
class FG_eval {
  public:
    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
    FG_eval(unsigned int N_, float dt_, float C_cte_, float C_epsi_, float C_v_, 
            float C_delta_, float C_a_, float C_delta_diff_, float C_a_diff_, float C_slowdown_);
    void set_coeffs(Eigen::VectorXd coeffs_) { coeffs = coeffs_; }
    void set_indices(size_t x_start, size_t y_start, size_t psi_start, size_t v_start, size_t cte_start, size_t epsi_start,
                     size_t delta_start, size_t a_start);
    void set_curr_activations(float delta, float a);                 
    void operator()(ADvector& fg, const ADvector& vars);

   private: 
     // Fitted polynomial coefficients
     Eigen::VectorXd coeffs;

     unsigned int N;
     float dt;
     float C_cte;
     float C_epsi;
     float C_v;
     float C_delta;
     float C_a;
     float C_delta_diff;
     float C_a_diff;
     float C_slowdown;
     float curr_delta;
     float curr_a;

     size_t x_start;
     size_t y_start;
     size_t psi_start;
     size_t v_start;
     size_t cte_start;
     size_t epsi_start;
     size_t delta_start;
     size_t a_start;
};

// MPC controller
class MPC {
  public:
    typedef CPPAD_TESTVECTOR(double) Dvector;

    MPC(unsigned int N_, float dt_, float C_cte_, float C_epsi_, float C_v_, 
        float C_delta_, float C_a_, float C_delta_diff_, float C_a_diff_, float C_corner_);

    virtual ~MPC();

    // Solve the model given an initial state and polynomial coefficients.
    // Return the first actuations.
    vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
  
  private:
    unsigned int N;
    FG_eval fg_eval;

    size_t x_start;
    size_t y_start;
    size_t psi_start;
    size_t v_start;
    size_t cte_start;
    size_t epsi_start;
    size_t delta_start;
    size_t a_start;

    size_t n_vars;
    size_t n_constraints;

    Dvector vars;
    Dvector vars_lowerbound;
    Dvector vars_upperbound;
    Dvector constraints_lowerbound;
    Dvector constraints_upperbound;
};

#endif /* MPC_H */
