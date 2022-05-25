#include "mpc_osqp.h"

#include <chrono>
#include <ctime>
#include <limits>
#include <utility>
#include <vector>

int main()
{
  {const int states = 4;
  const int controls = 2;
  const int horizon = 3;
  const int max_iter = 100;
  const double eps = 0.001;
  const double max = std::numeric_limits<double>::max();

  Eigen::MatrixXd A(states, states);
  A << 5., 6., 7., 8., 7., 8., 7., 8., 9., 10., 7., 8., 11., 4., 7., 8.;

  Eigen::MatrixXd B(states, controls);
  B << 2., 3, 2., 7, 2, 9, 3, 8;

  Eigen::MatrixXd Q(states, states);
  Q << 10., 0, 0, 0, 0, 10., 0, 0, 0, 0, 10.0, 0, 0, 0, 0, 10.0;

  Eigen::MatrixXd R(controls, controls);
  R << 0.1, 0, 0, 0.1;

  Eigen::MatrixXd lower_control_bound(controls, 1);
  lower_control_bound << 9.6 - 10.5916, 9.6 - 10.5916;

  Eigen::MatrixXd upper_control_bound(controls, 1);
  upper_control_bound << 13 - 10.5916, 13 - 10.5916;

  Eigen::MatrixXd lower_state_bound(states, 1);
  lower_state_bound << -M_PI / 6, -M_PI / 6, -1 * max, -1 * max;

  Eigen::MatrixXd upper_state_bound(states, 1);
  upper_state_bound << M_PI / 6, M_PI / 6, max, max;

  Eigen::MatrixXd initial_state(states, 1);
  initial_state << 0, 0, 0, 0;
  std::vector<double> control_cmd1(controls, 0);
  std::vector<double> control_cmd2(controls, 0);

  Eigen::MatrixXd reference_state(states, 1);
  reference_state << 0, 0, 10, 0;
  // std::vector<Eigen::MatrixXd> reference(horizon, reference_state);

  // OSQP
  MpcOsqp mpc_osqp_solver(A, B, Q, R, initial_state, lower_control_bound,
                          upper_control_bound, lower_state_bound,
                          upper_state_bound, reference_state, max_iter, horizon,
                          eps);
  auto start_time_osqp = std::chrono::system_clock::now();
  mpc_osqp_solver.Solve(&control_cmd1);
  auto end_time_osqp = std::chrono::system_clock::now();
  std::chrono::duration<double> diff_OSQP = end_time_osqp - start_time_osqp;
  std::cout << "OSQP used time: " << diff_OSQP.count() * 1000 << " ms."<<std::endl;
  std::cout<<control_cmd1[0]<<std::endl<<control_cmd1[1];}
  {const int states = 6;
  const int controls = 2;
  const int horizon = 10;
  const int max_iter = 100;
  const double eps = 0.000001;
  const double max = std::numeric_limits<double>::max();

  Eigen::MatrixXd A(states, states);
  A << 0,1,0,0,0,0,0,-2.4919,149.514,0,0,0,0,0,0,1,0,0,0,0,0,-2.4919,0,0,0,0,0,0,0,1,0,0,0,0,0,0;

  Eigen::MatrixXd B(states, controls);
  B << 0,0,74.757,0,0,0,52.557,0,0,0,0,-1;

  Eigen::MatrixXd Q(states, states);
  Q << 0.05,0,0,0,0,0,  0,0,0,0,0,0,  0,0,1,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0,0,0;

  Eigen::MatrixXd R(controls, controls);
  R << 1,0,0,1;

  Eigen::MatrixXd lower_control_bound(controls, 1);
  lower_control_bound << -30,-10;

  Eigen::MatrixXd upper_control_bound(controls, 1);
  upper_control_bound << 30, 10;

  Eigen::MatrixXd lower_state_bound(states, 1);
  lower_state_bound << -1000,-1000,-3.14,-3.14,-1000,-1000;

  Eigen::MatrixXd upper_state_bound(states, 1);
  upper_state_bound << 1000,1000,3.14,1000,1000,1000;

  Eigen::MatrixXd initial_state(states, 1);
  initial_state << 0, 0, 0, 0,0,0;
  std::vector<double> control_cmd1(controls, 0);
  std::vector<double> control_cmd2(controls, 0);

  Eigen::MatrixXd reference_state(states, 1);
  reference_state << 10,0,0,0,10,0;
  std::vector<Eigen::MatrixXd> reference(horizon, reference_state);

  // OSQP
  MpcOsqp mpc_osqp_solver(A, B, Q, R, initial_state, lower_control_bound,
                          upper_control_bound, lower_state_bound,
                          upper_state_bound, reference_state, max_iter, horizon,
                          eps);
  auto start_time_osqp = std::chrono::system_clock::now();
  mpc_osqp_solver.Solve(&control_cmd1);
  auto end_time_osqp = std::chrono::system_clock::now();
  std::chrono::duration<double> diff_OSQP = end_time_osqp - start_time_osqp;
  std::cout << "OSQP used time: " << diff_OSQP.count() * 1000 << " ms."<<std::endl;
  std::cout<<control_cmd1[0]<<std::endl<<control_cmd1[1];
  }
}