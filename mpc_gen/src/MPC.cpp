#include "MPC.h"
#include <math.h>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <ackermann_msgs/AckermannDriveStamped.h>


using CppAD::AD;

// We set the number of timesteps to 40
// and the timestep evaluation frequency or evaluation
// period to 0.1.
size_t N = 40;
double dt = 0.1;

const double Lf = 1.0;

// Both the reference cross track and orientation errors are 0.
// The reference velocity is set to 10 mph.
double ref_cte  = 0;
double ref_epsi = 0;
double ref_v    = 10;

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we establish it
// when one variable starts and another ends.
size_t x_start      = 0;
size_t y_start      = x_start     + N;
size_t psi_start    = y_start     + N;
size_t v_start      = psi_start   + N;
size_t cte_start    = v_start     + N;
size_t epsi_start   = cte_start   + N;
size_t delta_start  = epsi_start  + N;
size_t a_start      = delta_start + N - 1;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  std::vector<Point2D> obstacles;
  FG_eval(Eigen::VectorXd coeffs, std::vector<Point2D> obstacles) { this->coeffs = coeffs; this->obstacles = obstacles; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // fg a vector of constraints, x is a vector of constraints.
    fg[0] = 0;

    // The part of the cost based on the reference state.
    for (int i = 0; i < N; i++) {
      fg[0] += 500*CppAD::pow(vars[cte_start  + i] - ref_cte,   2);
      fg[0] += 800*CppAD::pow(vars[epsi_start + i] - ref_epsi,  2);
      fg[0] += 1.0*CppAD::pow(vars[v_start    + i] - ref_v,     2);
    }

    // Cost to minimize the use of actuators.
    for (int i = 0; i < N - 1; i++) {
      fg[0] += 1*CppAD::pow(vars[delta_start  + i], 2);
      fg[0] += 1*CppAD::pow(vars[a_start      + i], 2);
    }

    // Cost to minimize the value gap between sequential actuations.
    for (int i = 0; i < N - 2; i++) {
      fg[0] += 800.0 * CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
      fg[0] +=  10*     CppAD::pow(vars[a_start     + i + 1] - vars[a_start     + i], 2);
    }
    for (int i = 0; i < N - 1; i++) {
      AD<double> x = vars[x_start + i];
      AD<double> y = vars[y_start + i];
      AD<double> psi = vars[psi_start + i];
      for (const auto& obstacle : obstacles) {
        // Calculate the squared distance to each obstacle
        AD<double> dist_sq = CppAD::pow(x - obstacle.x, 2) + CppAD::pow(y - obstacle.y, 2);
        // Add penalty if the distance is below a threshold
        fg[0] += 100000000000 * CppAD::exp(-dist_sq / 10);
      }
  }


    // Initial constraints

    fg[1 + x_start]     = vars[x_start]   ;
    fg[1 + y_start]     = vars[y_start]   ;
    fg[1 + psi_start]   = vars[psi_start] ;
    fg[1 + v_start]     = vars[v_start]   ;
    fg[1 + cte_start]   = vars[cte_start] ;
    fg[1 + epsi_start]  = vars[epsi_start];

    // Other constraints
    for (int i = 0; i < N - 1; i++) {
      // The state at time t+1 .
      AD<double> x1     = vars[x_start    + i + 1];
      AD<double> y1     = vars[y_start    + i + 1];
      AD<double> psi1   = vars[psi_start  + i + 1];
      AD<double> v1     = vars[v_start    + i + 1];
      AD<double> cte1   = vars[cte_start  + i + 1];
      AD<double> epsi1  = vars[epsi_start + i + 1];

      // The state at time t
      AD<double> x0     = vars[x_start    + i];
      AD<double> y0     = vars[y_start    + i];
      AD<double> psi0   = vars[psi_start  + i];
      AD<double> v0     = vars[v_start    + i];
      AD<double> cte0   = vars[cte_start  + i];
      AD<double> epsi0  = vars[epsi_start + i];

      // Only considering the actuation at time t
      AD<double> delta0 = vars[delta_start  + i];
      AD<double> a0     = vars[a_start      + i];

      // 3rd order derivative
      AD<double> f0       = coeffs[0] + coeffs[1]  * x0 + coeffs[2] * CppAD::pow(x0,2) + coeffs[3] * CppAD::pow(x0,3);
      AD<double> psides0  = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 *coeffs[3]* CppAD::pow(x0, 2));

      fg[2 + x_start      + i] = x1     - (x0               + v0 * CppAD::cos(psi0)   * dt) ;
      fg[2 + y_start      + i] = y1     - (y0               + v0 * CppAD::sin(psi0)   * dt) ;
      fg[2 + psi_start    + i] = psi1   - (psi0             + v0 * delta0 / Lf        * dt) ;
      fg[2 + v_start      + i] = v1     - (v0               + a0                      * dt) ;
      fg[2 + cte_start    + i] = cte1   - ((f0 - y0)        + (v0 * CppAD::sin(epsi0) * dt));
      fg[2 + epsi_start   + i] = epsi1  - ((psi0 - psides0) + v0 * delta0 / Lf        * dt) ;
    }
  }
};

MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs, std::vector<Point2D> obstacles) {
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;
  double x    = state[0];
  double y    = state[1];
  double psi  = state[2];
  double v    = state[3];
  double cte  = state[4];
  double epsi = state[5];
  //  Set the number of model variables (includes both states and inputs).

  size_t n_vars =  N * 6 + (N - 1) * 2;;
  //  Set the number of constraints
  size_t n_constraints = N * 6;

  // Initial value of the independent variables.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  // Set the initial variable values
  vars[x_start]     = x;
  vars[y_start]     = y;
  vars[psi_start]   = psi;
  vars[v_start]     = v;
  vars[cte_start]   = cte;
  vars[epsi_start]  = epsi;

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  //  Set lower and upper limits for variables.

  for (int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  for (int i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.11;
    vars_upperbound[i] = 0.11;
  }

  for (int i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = 0.5;
    vars_upperbound[i] = 1.5;
  }

  // Lower and upper limits for the constraints
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  constraints_lowerbound[x_start]     = x;
  constraints_lowerbound[y_start]     = y;
  constraints_lowerbound[psi_start]   = psi;
  constraints_lowerbound[v_start]     = v;
  constraints_lowerbound[cte_start]   = cte;
  constraints_lowerbound[epsi_start]  = epsi;

  constraints_upperbound[x_start]     = x;
  constraints_upperbound[y_start]     = y;
  constraints_upperbound[psi_start]   = psi;
  constraints_upperbound[v_start]     = v;
  constraints_upperbound[cte_start]   = cte;
  constraints_upperbound[epsi_start]  = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs,obstacles);

  // options for IPOPT solver
  std::string options;

  options += "Integer print_level  0\n";
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  options += "Numeric max_cpu_time          0.05\n";

  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  return {solution.x[x_start      + 1],
          solution.x[y_start      + 1],
          solution.x[psi_start    + 1],
          solution.x[v_start      + 1],
          solution.x[cte_start    + 1],
          solution.x[epsi_start   + 1],
          solution.x[delta_start] + solution.x[delta_start  + 1],
          solution.x[a_start]     + solution.x[a_start      + 1]
        };
}
