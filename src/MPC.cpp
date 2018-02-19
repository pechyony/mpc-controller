#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include <stdlib.h>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// TODO: Set the timestep length and duration
double delay = 0.1;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

// desired speed
const double desired_speed = 100; 

//
// FG_eval class implementation.
//

FG_eval::FG_eval(unsigned int N_, float dt_, float C_cte_, float C_epsi_, float C_v_, 
                 float C_delta_, float C_a_, float C_delta_diff_, float C_a_diff_, float C_corner_) :
                 N(N_), dt(dt_), C_cte(C_cte_), C_epsi(C_epsi_), C_v(C_v_), C_delta(C_delta_), C_a(C_a_),
                 C_delta_diff(C_delta_diff_), C_a_diff(C_a_diff_), C_corner (C_corner_), curr_delta(0), curr_a(0)
{
}

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends.
void FG_eval::set_indices(size_t x_start_, size_t y_start_, size_t psi_start_, size_t v_start_, size_t cte_start_, 
                          size_t epsi_start_, size_t delta_start_, size_t a_start_)
{
    x_start = x_start_;
    y_start = y_start_;
    psi_start = psi_start_;
    v_start = v_start_;
    cte_start = cte_start_;
    epsi_start = epsi_start_;
    delta_start = delta_start_;
    a_start = a_start_;
}

void FG_eval::set_curr_activations(float delta, float a)
{
    curr_delta = delta;
    curr_a = a;
}

void FG_eval::operator()(ADvector& fg, const ADvector& vars) {
    
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // The cost is stored is the first element of `fg`.
    // Any additions to the cost should be added to `fg[0]`.
    fg[0] = 0;

    // The part of the cost based on the reference state.
    for (int t = 0; t < 2 * N - 1; t++) {
      fg[0] += C_cte * CppAD::pow(vars[cte_start + t], 2);
      fg[0] += C_epsi * CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += C_v * CppAD::pow(vars[v_start + t] - desired_speed, 2);
    }

    // Minimize the use of actuators.
    for (int t = 0; t < N - 1; t++) {
      fg[0] += C_delta * CppAD::pow(vars[delta_start + t], 2);
      fg[0] += C_a * CppAD::pow(vars[a_start + t], 2);
    }

    // Penalize acceleration when the car is in dangerous area (cte or epsi are large)
    for (int t = 0; t < N - 1; t++) {
      fg[0] += C_corner * CppAD::pow(vars[epsi_start + t],2) * CppAD::pow(vars[a_start + t],2);
      fg[0] += C_corner * CppAD::pow(vars[cte_start + t],2) * CppAD::pow(vars[a_start + t],2);
    }

    // Minimize the value gap between sequential actuations.
    for (int t = 0; t < N - 2; t++) {
      fg[0] += C_delta_diff * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += C_a_diff * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }

    //
    // Setup Constraints
    //

    // Initial constraints
    //
    // We add 1 to each of the starting indices due to cost being located at index 0 of `fg`.
    // This bumps up the position of all the other values.
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // The rest of the constraints
    for (int t = 1; t < 2 * N - 1; t++) {

      // The state at time t+1 
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];

      // The state at time t
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];

      // Get actuation at time t.
      AD<double> delta0;
      AD<double> a0;
      if (t == 1) {
        // these are the last applied delta and acceleration (found at the previous solution of optimization problem )
        delta0 = curr_delta;
        a0 = curr_a;
      }
      else {
        std::div_t quot_rem = div(t,2);
        delta0 = vars[delta_start + quot_rem.quot - 1];
        a0 = vars[a_start + quot_rem.quot - 1];
      }

      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * x0 * x0;
      AD<double> psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0);

      // Here's `x` to get you started.
      // The idea here is to constraint this value to be 0.
      //
      // Recall the equations for the model:
      // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      // v_[t+1] = v[t] + a[t] * dt
      // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
      
      double delta_t;
      if (t % 2 == 0)
          delta_t = dt;
      else
          delta_t = delay;

      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * delta_t);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * delta_t);
      fg[1 + psi_start + t] = psi1 - (psi0 - v0 * delta0 / Lf * delta_t);
      fg[1 + v_start + t] = v1 - (v0 + a0 * delta_t);
      fg[1 + cte_start + t] =
          cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * delta_t));
      fg[1 + epsi_start + t] =
          epsi1 - ((psi0 - psides0) - v0 * delta0 / Lf * delta_t);
    }
}

//
// MPC class implementation.
//
MPC::MPC(unsigned int N_, float dt_, float C_cte_, float C_epsi_, float C_v_, 
         float C_delta_, float C_a_, float C_delta_diff_, float C_a_diff_, float C_corner_) : 
         N(N_), fg_eval(N_, dt_, C_cte_, C_epsi_, C_v_, C_delta_, C_a_, C_delta_diff_, C_a_diff_, C_corner_) 
{
    // The solver takes all the state variables and actuator
    // variables in a singular vector. Thus, we should to establish
    // when one variable starts and another ends.

    x_start = 0;
    y_start = x_start + 2 * N - 1;
    psi_start = y_start + 2 * N - 1;
    v_start = psi_start + 2 * N - 1;
    cte_start = v_start + 2 * N - 1;
    epsi_start = cte_start + 2 * N - 1;
    delta_start = epsi_start + 2 * N - 1;
    a_start = delta_start + N - 1;

    // Let fg_eval know the boundaries of the variables
    fg_eval.set_indices(x_start,y_start,psi_start,v_start,cte_start,epsi_start,delta_start,a_start);

    // number of variables in optimization problem
    n_vars = N * 6 + (N-1) * 2 + (N-1) * 6;

    // number of constraints in optimization problem
    n_constraints = N * 6 + (N-1) * 6;

    // define vector of variables and vectors of upper/lower boundaries of constraints
    vars = Dvector(n_vars);
    vars_lowerbound = Dvector(n_vars);
    vars_upperbound = Dvector(n_vars);

    // Set all non-actuators upper and lowerlimits
    // to the max negative and positive values.

    // Set maximal and minimal values of acceleration
    for (int i = 0; i < delta_start; i++) {
        vars_lowerbound[i] = -1.0e19;
        vars_upperbound[i] = 1.0e19;
    }

    // The upper and lower limits of delta are set to -25 and 25
    // degrees (values in radians).
    for (int i = delta_start; i < a_start; i++) {
        vars_lowerbound[i] = -0.436332;
        vars_upperbound[i] = 0.436332;
    }

    // Acceleration/decceleration upper and lower limits.
    for (int i = a_start; i < n_vars; i++) {
        vars_lowerbound[i] = -1.0; 
        vars_upperbound[i] = 1.0; 
    }

    // Lower and upper limits for constraints
    // All of these should be 0 except the initial
    // state indices.

    constraints_lowerbound = Dvector(n_constraints);
    constraints_upperbound = Dvector(n_constraints);
  
    for (int i = 0; i < n_constraints; i++) {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }
}

MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  
  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.

  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;

  // define constraints that correspond to initial state. All other constraints were defined 
  // in the constructor of the class

  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;

  // object that computes objective and constraints
  fg_eval.set_coeffs(coeffs);

  // options for IPOPT solver
  std::string options;
  options += "Integer print_level  0\n";

  // Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";

  // the solver has a maximum time limit of 0.5 seconds.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // remember current activations, they will be used to solve optimization problem at the next iteration
  fg_eval.set_curr_activations(solution.x[delta_start], solution.x[a_start]);

  // return the first actuator values
  vector<double> out;

  out.push_back(solution.x[delta_start]);
  out.push_back(solution.x[a_start]);

  // return predicted car positions
  for (int t = 1; t < 2 * N - 1; t++) {
    out.push_back(solution.x[x_start + t]);
    out.push_back(solution.x[y_start + t]);
  }

  return out;
}
