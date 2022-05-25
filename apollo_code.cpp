#include "Eigen/Core"
#include "Eigen/SVD"
#include <iostream>
#include "Eigen/LU"
using Matrix = Eigen::MatrixXd;

# define M_PI		3.14159265358979323846	/* pi */
// number of states, includes
// lateral error, lateral error rate, heading error, heading error rate,
// station error, velocity error,
const int basic_state_size_ = 6;

const int controls_ = 2;

const int horizon_ = 10;

// the following parameters are vehicle physics related.
  // control time interval
  double ts_ = 0.01;
  // corner stiffness; front
  double cf_ = 155494.663;
  // corner stiffness; rear
  double cr_ = 155494.663;
  // mass of the vehicle
  const double mass_fl = 520;
  const double mass_fr = 520;
  const double mass_rl = 520;
  const double mass_rr = 520;
  const double mass_front = mass_fl + mass_fr;
  const double mass_rear = mass_rl + mass_rr;
  double mass_ = mass_front + mass_rear;
   // distance between front and rear wheel center
  double wheelbase_ = 2.8448;
  // distance from front wheel center to COM
  double lf_ = wheelbase_ * (1.0 - mass_front / mass_);
  // distance from rear wheel center to COM
  double lr_ = wheelbase_ * (1.0 - mass_rear / mass_);
  // rotational inertia
  double iz_ = lf_ * lf_ * mass_front + lr_ * lr_ * mass_rear;
  // the ratio between the turn of the steering wheel and the turn of the wheels
  double steer_ratio_ = 16;
  // the maximum turn of steer
  double max_steer_angle = 8.20304748437;
  double steer_single_direction_max_degree_ = max_steer_angle * 180 / M_PI;;
  // the maximum turn of vehicle wheel
  double wheel_single_direction_max_degree_ = steer_single_direction_max_degree_ / steer_ratio_ / 180 * M_PI;;

  // vehicle velocity
  double v = 60;

int main()
{
    // Matrix init operations.
    // Matrix_a_中的系数分为两类，一类于速度无关，另外一类与速度相关，放在matrix_a_coeff中
    Eigen::MatrixXd matrix_a_ = Matrix::Zero(basic_state_size_, basic_state_size_);
    Eigen::MatrixXd matrix_ad_ = Matrix::Zero(basic_state_size_, basic_state_size_);
    matrix_a_(0, 1) = 1.0;
    matrix_a_(1, 2) = (cf_ + cr_) / mass_;
    matrix_a_(2, 3) = 1.0;
    matrix_a_(3, 2) = (lf_ * cf_ - lr_ * cr_) / iz_;
    matrix_a_(4, 5) = 1.0;
    matrix_a_(5, 5) = 0.0;

    Eigen::MatrixXd matrix_a_coeff_ = Matrix::Zero(basic_state_size_, basic_state_size_);
    matrix_a_coeff_(1, 1) = -(cf_ + cr_) / mass_;
    matrix_a_coeff_(1, 3) = (lr_ * cr_ - lf_ * cf_) / mass_;
    matrix_a_coeff_(2, 3) = 1.0;
    matrix_a_coeff_(3, 1) = (lr_ * cr_ - lf_ * cf_) / iz_;
    matrix_a_coeff_(3, 3) = -1.0 * (lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_;
    matrix_a_(1, 1) = matrix_a_coeff_(1, 1) / v;
    matrix_a_(1, 3) = matrix_a_coeff_(1, 3) / v;
    matrix_a_(3, 1) = matrix_a_coeff_(3, 1) / v;
    matrix_a_(3, 3) = matrix_a_coeff_(3, 3) / v;
    Matrix matrix_i = Matrix::Identity(matrix_a_.cols(), matrix_a_.cols());
    matrix_ad_ = (matrix_i - ts_ * 0.5 * matrix_a_).inverse() *
               (matrix_i + ts_ * 0.5 * matrix_a_);
    std::cout<<"matrix_a:"<<matrix_a_<<std::endl;

    Eigen::MatrixXd matrix_b_ = Matrix::Zero(basic_state_size_, controls_);
    Eigen::MatrixXd matrix_bd_ = Matrix::Zero(basic_state_size_, controls_);
    matrix_b_(1, 0) = cf_ / mass_;
    matrix_b_(3, 0) = lf_ * cf_ / iz_;
    matrix_b_(4, 1) = 0.0;
    matrix_b_(5, 1) = -1.0;
    matrix_bd_ = matrix_b_ * ts_;
    std::cout<<"matrix_b:"<<matrix_b_<<std::endl;
    Eigen::MatrixXd matrix_state_ = Matrix::Zero(basic_state_size_, 1);
    Eigen::MatrixXd matrix_k_ = Matrix::Zero(1, basic_state_size_);

    Eigen::MatrixXd matrix_r_ = Matrix::Identity(controls_, controls_);
    Eigen::MatrixXd matrix_r_updated_;

    Eigen::MatrixXd matrix_q_ = Matrix::Zero(basic_state_size_, basic_state_size_);
    Eigen::MatrixXd matrix_q_updated_;
    
    std::cout<<"test success ended!"<<std::endl;

    return 0;
}

    
    