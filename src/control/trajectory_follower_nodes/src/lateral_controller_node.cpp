// Copyright 2018-2019 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <deque>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "trajectory_follower_nodes/lateral_controller_node.hpp"

#include "tf2_ros/create_timer_ros.h"

#define DEG2RAD 3.1415926535 / 180.0
#define RAD2DEG 180.0 / 3.1415926535

#define UPDATE_MPC_PARAM(PARAM_STRUCT, NAME) \
  update_param(parameters, "mpc_" #NAME, PARAM_STRUCT.NAME)

namespace autoware
{
namespace motion
{
namespace control
{
namespace trajectory_follower_nodes
{
namespace
{
using namespace std::chrono_literals;

template<typename T>
void update_param(
  const std::vector<rclcpp::Parameter> & parameters, const std::string & name, T & value)
{
  auto it = std::find_if(
    parameters.cbegin(), parameters.cend(),
    [&name](const rclcpp::Parameter & parameter) {return parameter.get_name() == name;});
  if (it != parameters.cend()) {
    value = static_cast<T>(it->template get_value<T>());
  }
}
}  // namespace

MPCFollower::MPCFollower(const rclcpp::NodeOptions & node_options)
: Node("mpc_follower", node_options),
  tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
{
  using std::placeholders::_1;

  ctrl_period_ = declare_parameter("ctrl_period", 0.03);
  enable_path_smoothing_ = declare_parameter("enable_path_smoothing", true);
  enable_yaw_recalculation_ = declare_parameter("enable_yaw_recalculation", false);
  path_filter_moving_ave_num_ =
    static_cast<int>(declare_parameter("path_filter_moving_ave_num", 35));
  curvature_smoothing_num_ = static_cast<int>(declare_parameter("curvature_smoothing_num", 35));
  traj_resample_dist_ = declare_parameter("traj_resample_dist", 0.1);  // [m]
  admissible_position_error_ = declare_parameter("admissible_position_error", 5.0);
  admissible_yaw_error_rad_ = declare_parameter("admissible_yaw_error_rad", M_PI_2);
  use_steer_prediction_ = declare_parameter("use_steer_prediction", false);
  mpc_param_.steer_tau = declare_parameter("vehicle_model_steer_tau", 0.1);

  /* stop state parameters */
  stop_state_entry_ego_speed_ = declare_parameter("stop_state_entry_ego_speed", 0.2);        // [m]
  stop_state_entry_target_speed_ = declare_parameter("stop_state_entry_target_speed", 0.1);  // [m]
  stop_state_keep_stopping_dist_ = declare_parameter("stop_state_keep_stopping_dist", 0.5);  // [m]

  /* mpc parameters */
  double steer_lim_deg, steer_rate_lim_degs;
  steer_lim_deg = declare_parameter("steer_lim_deg", 35.0);
  steer_rate_lim_degs = declare_parameter("steer_rate_lim_degs", 150.0);
  steer_lim_ = steer_lim_deg * DEG2RAD;
  steer_rate_lim_ = steer_rate_lim_degs * DEG2RAD;
  wheelbase_ = declare_parameter("wheel_base", 2.7);

  /* vehicle model setup */
  vehicle_model_type_ = declare_parameter("vehicle_model_type", "kinematics");
  if (vehicle_model_type_ == "kinematics") {
    vehicle_model_ptr_ =
      std::make_shared<trajectory_follower::KinematicsBicycleModel>(
      wheelbase_, steer_lim_,
      mpc_param_.steer_tau);
  } else if (vehicle_model_type_ == "kinematics_no_delay") {
    vehicle_model_ptr_ = std::make_shared<trajectory_follower::KinematicsBicycleModelNoDelay>(
      wheelbase_, steer_lim_);
  } else if (vehicle_model_type_ == "dynamics") {
    double mass_fl = declare_parameter("mass_fl", 600.0);
    double mass_fr = declare_parameter("mass_fr", 600.0);
    double mass_rl = declare_parameter("mass_rl", 600.0);
    double mass_rr = declare_parameter("mass_rr", 600.0);
    double cf = declare_parameter("cf", 155494.663);
    double cr = declare_parameter("cr", 155494.663);

    // vehicle_model_ptr_ is only assigned in ctor, so parameter value have to be passed at init time  // NOLINT
    vehicle_model_ptr_ = std::make_shared<trajectory_follower::DynamicsBicycleModel>(
      wheelbase_, mass_fl, mass_fr, mass_rl, mass_rr, cf, cr);
  } else {
    RCLCPP_ERROR(get_logger(), "vehicle_model_type is undefined");
  }

  /* QP solver setup */
  const std::string qp_solver_type = declare_parameter("qp_solver_type", "unconstraint_fast");
  if (qp_solver_type == "unconstraint_fast") {
    qpsolver_ptr_ = std::make_shared<trajectory_follower::QPSolverEigenLeastSquareLLT>();
  } else if (qp_solver_type == "osqp") {
    qpsolver_ptr_ = std::make_shared<trajectory_follower::QPSolverOSQP>(get_logger());
  } else {
    RCLCPP_ERROR(get_logger(), "qp_solver_type is undefined");
  }

  /* delay compensation */
  {
    const double delay_tmp = declare_parameter("input_delay", 0.0);
    const double delay_step = std::round(delay_tmp / ctrl_period_);
    mpc_param_.input_delay = delay_step * ctrl_period_;
    input_buffer_ = std::deque<double>(static_cast<size_t>(delay_step), 0.0);
  }

  /* initialize lowpass filter */
  {
    const double steering_lpf_cutoff_hz = declare_parameter("steering_lpf_cutoff_hz", 3.0);
    const double error_deriv_lpf_cutoff_hz = declare_parameter("error_deriv_lpf_cutoff_hz", 5.0);
    lpf_steering_cmd_.initialize(ctrl_period_, steering_lpf_cutoff_hz);
    lpf_lateral_error_.initialize(ctrl_period_, error_deriv_lpf_cutoff_hz);
    lpf_yaw_error_.initialize(ctrl_period_, error_deriv_lpf_cutoff_hz);
  }

  /* set up ros system */
  initTimer(ctrl_period_);

  pub_ctrl_cmd_ =
    create_publisher<autoware_auto_msgs::msg::AckermannLateralCommand>("~/output/lateral_control_cmd", 1);
  pub_predicted_traj_ =
    create_publisher<autoware_auto_msgs::msg::Trajectory>("~/output/predicted_trajectory", 1);
  sub_ref_path_ = create_subscription<autoware_auto_msgs::msg::Trajectory>(
    "~/input/reference_trajectory", rclcpp::QoS{1},
    std::bind(&MPCFollower::onTrajectory, this, _1));
  sub_current_vel_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    "~/input/current_velocity", rclcpp::QoS{1}, std::bind(&MPCFollower::onVelocity, this, _1));
  sub_steering_ = create_subscription<autoware_auto_msgs::msg::VehicleKinematicState>(
    "~/input/current_kinematic_state", rclcpp::QoS{1}, std::bind(&MPCFollower::onSteering, this, _1));

  // TODO(Frederik.Beaujean) ctor is too long, should factor out parameter declarations
  declareMPCparameters();

  /* get parameter updates */
  set_param_res_ =
    this->add_on_set_parameters_callback(std::bind(&MPCFollower::paramCallback, this, _1));
}

MPCFollower::~MPCFollower()
{
  autoware_auto_msgs::msg::AckermannLateralCommand stop_cmd = getStopControlCommand();
  publishCtrlCmd(stop_cmd);
}

void MPCFollower::onTimer()
{
  updateCurrentPose();

  if (!checkData()) {
    publishCtrlCmd(getStopControlCommand());
    return;
  }

  autoware_auto_msgs::msg::AckermannLateralCommand ctrl_cmd;

  if (!is_ctrl_cmd_prev_initialized_) {
    ctrl_cmd_prev_ = getInitialControlCommand();
    is_ctrl_cmd_prev_initialized_ = true;
  }

  const bool is_mpc_solved = calculateMPC(&ctrl_cmd);

  if (isStoppedState()) {
    // Reset input buffer
    for (auto & value : input_buffer_) {
      value = ctrl_cmd_prev_.steering_tire_angle;
    }
    // Use previous command value as previous raw steer command
    raw_steer_cmd_prev_ = ctrl_cmd_prev_.steering_tire_angle;

    publishCtrlCmd(ctrl_cmd_prev_);
    return;
  }

  if (!is_mpc_solved) {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      get_logger(), *get_clock(), 5000 /*ms*/,
      "MPC is not solved. publish 0 velocity.");
    ctrl_cmd = getStopControlCommand();
  }

  ctrl_cmd_prev_ = ctrl_cmd;
  publishCtrlCmd(ctrl_cmd);
}

bool MPCFollower::checkData()
{
  if (!vehicle_model_ptr_ || !qpsolver_ptr_) {
    RCLCPP_DEBUG(
      get_logger(), "vehicle_model = %d, qp_solver = %d", vehicle_model_ptr_ != nullptr,
      qpsolver_ptr_ != nullptr);
    return false;
  }

  if (!current_pose_ptr_ || !current_velocity_ptr_ || !current_steer_ptr_) {
    RCLCPP_DEBUG(
      get_logger(), "waiting data. pose = %d, velocity = %d,  steer = %d",
      current_pose_ptr_ != nullptr, current_velocity_ptr_ != nullptr,
      current_steer_ptr_ != nullptr);
    return false;
  }

  if (ref_traj_.size() == 0) {
    RCLCPP_DEBUG(get_logger(), "trajectory size is zero.");
    return false;
  }

  return true;
}

// TODO(Frederik.Beaujean) This method is too long, could be refactored into many smaller units
bool MPCFollower::calculateMPC(autoware_auto_msgs::msg::AckermannLateralCommand * ctrl_cmd)
{
  if (!ctrl_cmd) {
    return false;
  }

  /* recalculate velocity from ego-velocity with dynamics */
  trajectory_follower::MPCTrajectory reference_trajectory =
    applyVelocityDynamicsFilter(ref_traj_, current_velocity_ptr_->twist.linear.x);

  MPCData mpc_data;
  if (!getData(reference_trajectory, &mpc_data)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "fail to get Data.");
    return false;
  }

  /* define initial state for error dynamics */
  Eigen::VectorXd x0 = getInitialState(mpc_data);

  /* delay compensation */

  if (!updateStateForDelayCompensation(reference_trajectory, mpc_data.nearest_time, &x0)) {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      get_logger(), *get_clock(), 1000 /*ms*/,
      "updateStateForDelayCompensation failed. stop computation.");
    return false;
  }

  /* resample ref_traj with mpc sampling time */
  trajectory_follower::MPCTrajectory mpc_resampled_ref_traj;
  const double mpc_start_time = mpc_data.nearest_time + mpc_param_.input_delay;
  if (!resampleMPCTrajectoryByTime(mpc_start_time, reference_trajectory, &mpc_resampled_ref_traj)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(),
      1000 /*ms*/, "trajectory resampling failed.");
    return false;
  }

  /* generate mpc matrix : predict equation Xec = Aex * x0 + Bex * Uex + Wex */
  MPCMatrix mpc_matrix = generateMPCMatrix(mpc_resampled_ref_traj);

  /* solve quadratic optimization */
  Eigen::VectorXd Uex;
  if (!executeOptimization(mpc_matrix, x0, &Uex)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "optimization failed.");
    return false;
  }

  /* apply saturation and filter */
  const double u_saturated = std::max(std::min(Uex(0), steer_lim_), -steer_lim_);
  const double u_filtered = lpf_steering_cmd_.filter(u_saturated);

  /* set control command */
  {
    const auto & dt = mpc_param_.prediction_dt;
    ctrl_cmd->steering_tire_angle = static_cast<float>(u_filtered);
    ctrl_cmd->steering_tire_rotation_rate = static_cast<float>((Uex(1) - Uex(0)) / dt);
  }

  storeSteerCmd(u_filtered);

  /* save input to buffer for delay compensation*/
  input_buffer_.push_back(ctrl_cmd->steering_tire_angle);
  input_buffer_.pop_front();
  raw_steer_cmd_pprev_ = raw_steer_cmd_prev_;
  raw_steer_cmd_prev_ = Uex(0);

  return true;
}

bool MPCFollower::getData(const trajectory_follower::MPCTrajectory & traj, MPCData * data)
{
  static constexpr auto duration = 5000 /*ms*/;
  size_t nearest_idx;
  if (!trajectory_follower::MPCUtils::calcNearestPoseInterp(
      traj, current_pose_ptr_->pose, &(data->nearest_pose), &(nearest_idx),
      &(data->nearest_time), get_logger(), *get_clock()))
  {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      get_logger(), *get_clock(), duration,
      "calculateMPC: error in calculating nearest pose. stop mpc.");
    return false;
  }

  /* get data */
  data->nearest_idx = static_cast<int>(nearest_idx);
  data->steer = static_cast<double>(current_steer_ptr_->state.front_wheel_angle_rad);
  data->lateral_err = trajectory_follower::MPCUtils::calcLateralError(
    current_pose_ptr_->pose,
    data->nearest_pose);
  data->yaw_err = trajectory_follower::MPCUtils::normalizeRadian(
    tf2::getYaw(current_pose_ptr_->pose.orientation) - tf2::getYaw(data->nearest_pose.orientation));

  /* get predicted steer */
  if (!steer_prediction_prev_) {
    steer_prediction_prev_ = std::make_shared<double>(
      current_steer_ptr_->state.front_wheel_angle_rad);
  }
  data->predicted_steer = calcSteerPrediction();
  *steer_prediction_prev_ = data->predicted_steer;

  /* check error limit */
  const double dist_err = trajectory_follower::MPCUtils::calcDist2d(
    current_pose_ptr_->pose,
    data->nearest_pose);
  if (dist_err > admissible_position_error_) {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      get_logger(), *get_clock(), duration, "position error is over limit. error = %fm, limit: %fm",
      dist_err, admissible_position_error_);
    return false;
  }

  /* check yaw error limit */
  if (std::fabs(data->yaw_err) > admissible_yaw_error_rad_) {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      get_logger(), *get_clock(), duration, "yaw error is over limit. error = %fdeg, limit %fdeg",
      RAD2DEG * data->yaw_err, RAD2DEG * admissible_yaw_error_rad_);
    return false;
  }

  /* check trajectory time length */
  auto end_time = data->nearest_time + mpc_param_.input_delay + getPredictionTime();
  if (end_time > traj.relative_time.back()) {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      get_logger(), *get_clock(), 1000 /*ms*/, "path is too short for prediction.");
    return false;
  }
  return true;
}

double MPCFollower::calcSteerPrediction()
{
  auto t_start = time_prev_;
  auto t_end = this->now();
  time_prev_ = t_end;

  const double duration = (t_end - t_start).seconds();
  const double time_constant = mpc_param_.steer_tau;

  const double initial_response = std::exp(-duration / time_constant) * (*steer_prediction_prev_);

  if (ctrl_cmd_vec_.size() <= 2) {return initial_response;}

  return initial_response + getSteerCmdSum(t_start, t_end, time_constant);
}

double MPCFollower::getSteerCmdSum(
  const rclcpp::Time & t_start, const rclcpp::Time & t_end, const double time_constant)
{
  if (ctrl_cmd_vec_.size() <= 2) {return 0.0;}

  // Find first index of control command container
  size_t idx = 1;
  while (t_start > rclcpp::Time(ctrl_cmd_vec_.at(idx).stamp)) {
    if ((idx + 1) >= ctrl_cmd_vec_.size()) {return 0.0;}
    ++idx;
  }

  // Compute steer command input response
  double steer_sum = 0.0;
  auto t = t_start;
  while (t_end > rclcpp::Time(ctrl_cmd_vec_.at(idx).stamp)) {
    const double duration = (rclcpp::Time(ctrl_cmd_vec_.at(idx).stamp) - t).seconds();
    t = rclcpp::Time(ctrl_cmd_vec_.at(idx).stamp);
    steer_sum +=
      (1 - std::exp(-duration / time_constant)) *
      static_cast<double>(ctrl_cmd_vec_.at(idx - 1).steering_tire_angle);
    ++idx;
    if (idx >= ctrl_cmd_vec_.size()) {break;}
  }

  const double duration = (t_end - t).seconds();
  steer_sum +=
    (1 - std::exp(-duration / time_constant)) *
    static_cast<double>(ctrl_cmd_vec_.at(idx - 1).steering_tire_angle);

  return steer_sum;
}

void MPCFollower::storeSteerCmd(const double steer)
{
  const auto time_delayed = this->now() + rclcpp::Duration::from_seconds(mpc_param_.input_delay);
  autoware_auto_msgs::msg::AckermannLateralCommand cmd;
  cmd.stamp = time_delayed;
  cmd.steering_tire_angle = static_cast<float>(steer);

  // store published ctrl cmd
  ctrl_cmd_vec_.emplace_back(cmd);

  if (ctrl_cmd_vec_.size() <= 2) {
    return;
  }

  // remove unused ctrl cmd
  constexpr double store_time = 0.3;
  if (
    (time_delayed - ctrl_cmd_vec_.at(1).stamp).seconds() >
    mpc_param_.input_delay + store_time)
  {
    ctrl_cmd_vec_.erase(ctrl_cmd_vec_.begin());
  }
}

bool MPCFollower::resampleMPCTrajectoryByTime(
  double ts, const trajectory_follower::MPCTrajectory & input,
  trajectory_follower::MPCTrajectory * output) const
{
  std::vector<double> mpc_time_v;
  for (int i = 0; i < mpc_param_.prediction_horizon; ++i) {
    mpc_time_v.push_back(ts + i * mpc_param_.prediction_dt);
  }
  if (!trajectory_follower::MPCUtils::linearInterpMPCTrajectory(
      input.relative_time, input,
      mpc_time_v, output))
  {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      get_logger(), const_cast<rclcpp::Clock &>(*get_clock()),
      1000 /*ms*/,
      "calculateMPC: mpc resample error. stop mpc calculation. check code!");
    return false;
  }
  return true;
}

Eigen::VectorXd MPCFollower::getInitialState(const MPCData & data)
{
  const int DIM_X = vehicle_model_ptr_->getDimX();
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(DIM_X);

  const auto & lat_err = data.lateral_err;
  const auto & steer = use_steer_prediction_ ? data.predicted_steer : data.steer;
  const auto & yaw_err = data.yaw_err;

  if (vehicle_model_type_ == "kinematics") {
    x0 << lat_err, yaw_err, steer;
  } else if (vehicle_model_type_ == "kinematics_no_delay") {
    x0 << lat_err, yaw_err;
  } else if (vehicle_model_type_ == "dynamics") {
    double dlat = (lat_err - lateral_error_prev_) / ctrl_period_;
    double dyaw = (yaw_err - yaw_error_prev_) / ctrl_period_;
    lateral_error_prev_ = lat_err;
    yaw_error_prev_ = yaw_err;
    dlat = lpf_lateral_error_.filter(dlat);
    dyaw = lpf_yaw_error_.filter(dyaw);
    x0 << lat_err, dlat, yaw_err, dyaw;
    RCLCPP_DEBUG(get_logger(), "(before lpf) dot_lat_err = %f, dot_yaw_err = %f", dlat, dyaw);
    RCLCPP_DEBUG(get_logger(), "(after lpf) dot_lat_err = %f, dot_yaw_err = %f", dlat, dyaw);
  } else {
    RCLCPP_ERROR(get_logger(), "vehicle_model_type is undefined");
  }
  return x0;
}

bool MPCFollower::updateStateForDelayCompensation(
  const trajectory_follower::MPCTrajectory & traj, const double & start_time, Eigen::VectorXd * x)
{
  const int DIM_X = vehicle_model_ptr_->getDimX();
  const int DIM_U = vehicle_model_ptr_->getDimU();
  const int DIM_Y = vehicle_model_ptr_->getDimY();

  Eigen::MatrixXd Ad(DIM_X, DIM_X);
  Eigen::MatrixXd Bd(DIM_X, DIM_U);
  Eigen::MatrixXd Wd(DIM_X, 1);
  Eigen::MatrixXd Cd(DIM_Y, DIM_X);

  Eigen::MatrixXd x_curr = *x;
  double mpc_curr_time = start_time;
  for (unsigned int i = 0; i < input_buffer_.size(); ++i) {
    double k = 0.0;
    double v = 0.0;
    if (
      !trajectory_follower::LinearInterpolate::interpolate(
        traj.relative_time, traj.k,
        mpc_curr_time, k) ||
      !trajectory_follower::LinearInterpolate::interpolate(
        traj.relative_time, traj.vx,
        mpc_curr_time, v))
    {
      RCLCPP_ERROR(
        get_logger(),
        "mpc resample error at delay compensation, stop mpc calculation. check code!");
      return false;
    }

    /* get discrete state matrix A, B, C, W */
    vehicle_model_ptr_->setVelocity(v);
    vehicle_model_ptr_->setCurvature(k);
    vehicle_model_ptr_->calculateDiscreteMatrix(Ad, Bd, Cd, Wd, ctrl_period_);
    Eigen::MatrixXd ud = Eigen::MatrixXd::Zero(DIM_U, 1);
    ud(0, 0) = input_buffer_.at(i);  // for steering input delay
    x_curr = Ad * x_curr + Bd * ud + Wd;
    mpc_curr_time += ctrl_period_;
  }
  *x = x_curr;
  return true;
}

trajectory_follower::MPCTrajectory MPCFollower::applyVelocityDynamicsFilter(
  const trajectory_follower::MPCTrajectory & input, const double v0)
{
  int nearest_idx = trajectory_follower::MPCUtils::calcNearestIndex(input, current_pose_ptr_->pose);
  if (nearest_idx < 0) {return input;}

  const double alim = mpc_param_.acceleration_limit;
  const double tau = mpc_param_.velocity_time_constant;

  trajectory_follower::MPCTrajectory output = input;
  trajectory_follower::MPCUtils::dynamicSmoothingVelocity(
    static_cast<size_t>(nearest_idx), v0,
    alim, tau, &output);
  const double t_ext = 100.0;  // extra time to prevent mpc calculation failure due to short time
  const double t_end = output.relative_time.back() + getPredictionTime() + t_ext;
  const double v_end = 0.0;
  output.vx.back() = v_end;  // set for end point
  output.push_back(
    output.x.back(), output.y.back(), output.z.back(), output.yaw.back(), v_end, output.k.back(),
    output.smooth_k.back(), t_end);
  return output;
}

/*
 * predict equation: Xec = Aex * x0 + Bex * Uex + Wex
 * cost function: J = Xex' * Qex * Xex + (Uex - Uref)' * R1ex * (Uex - Urefex) + Uex' * R2ex * Uex
 * Qex = diag([Q,Q,...]), R1ex = diag([R,R,...])
 */
MPCFollower::MPCMatrix MPCFollower::generateMPCMatrix(
  const trajectory_follower::MPCTrajectory & reference_trajectory)
{
  using Eigen::MatrixXd;

  const int N = mpc_param_.prediction_horizon;
  const double DT = mpc_param_.prediction_dt;
  const int DIM_X = vehicle_model_ptr_->getDimX();
  const int DIM_U = vehicle_model_ptr_->getDimU();
  const int DIM_Y = vehicle_model_ptr_->getDimY();

  MPCMatrix m;
  m.Aex = MatrixXd::Zero(DIM_X * N, DIM_X);
  m.Bex = MatrixXd::Zero(DIM_X * N, DIM_U * N);
  m.Wex = MatrixXd::Zero(DIM_X * N, 1);
  m.Cex = MatrixXd::Zero(DIM_Y * N, DIM_X * N);
  m.Qex = MatrixXd::Zero(DIM_Y * N, DIM_Y * N);
  m.R1ex = MatrixXd::Zero(DIM_U * N, DIM_U * N);
  m.R2ex = MatrixXd::Zero(DIM_U * N, DIM_U * N);
  m.Urefex = MatrixXd::Zero(DIM_U * N, 1);

  /* weight matrix depends on the vehicle model */
  MatrixXd Q = MatrixXd::Zero(DIM_Y, DIM_Y);
  MatrixXd R = MatrixXd::Zero(DIM_U, DIM_U);
  MatrixXd Q_adaptive = MatrixXd::Zero(DIM_Y, DIM_Y);
  MatrixXd R_adaptive = MatrixXd::Zero(DIM_U, DIM_U);

  MatrixXd Ad(DIM_X, DIM_X);
  MatrixXd Bd(DIM_X, DIM_U);
  MatrixXd Wd(DIM_X, 1);
  MatrixXd Cd(DIM_Y, DIM_X);
  MatrixXd Uref(DIM_U, 1);

  constexpr double ep = 1.0e-3;  // large enough to ignore velocity noise

  /* predict dynamics for N times */
  for (int i = 0; i < N; ++i) {
    const double ref_vx = reference_trajectory.vx[static_cast<size_t>(i)];
    const double ref_vx_squared = ref_vx * ref_vx;

    // curvature will be 0 when vehicle stops
    const double ref_k = reference_trajectory.k[static_cast<size_t>(i)] * sign_vx_;
    const double ref_smooth_k = reference_trajectory.smooth_k[static_cast<size_t>(i)] * sign_vx_;

    /* get discrete state matrix A, B, C, W */
    vehicle_model_ptr_->setVelocity(ref_vx);
    vehicle_model_ptr_->setCurvature(ref_k);
    vehicle_model_ptr_->calculateDiscreteMatrix(Ad, Bd, Cd, Wd, DT);

    Q = Eigen::MatrixXd::Zero(DIM_Y, DIM_Y);
    R = Eigen::MatrixXd::Zero(DIM_U, DIM_U);
    Q(0, 0) = getWeightLatError(ref_k);
    Q(1, 1) = getWeightHeadingError(ref_k);
    R(0, 0) = getWeightSteerInput(ref_k);

    Q_adaptive = Q;
    R_adaptive = R;
    if (i == N - 1) {
      Q_adaptive(0, 0) = mpc_param_.weight_terminal_lat_error;
      Q_adaptive(1, 1) = mpc_param_.weight_terminal_heading_error;
    }
    Q_adaptive(1, 1) += ref_vx_squared * getWeightHeadingErrorSqVel(ref_k);
    R_adaptive(0, 0) += ref_vx_squared * getWeightSteerInputSqVel(ref_k);

    /* update mpc matrix */
    int idx_x_i = i * DIM_X;
    int idx_x_i_prev = (i - 1) * DIM_X;
    int idx_u_i = i * DIM_U;
    int idx_y_i = i * DIM_Y;
    if (i == 0) {
      m.Aex.block(0, 0, DIM_X, DIM_X) = Ad;
      m.Bex.block(0, 0, DIM_X, DIM_U) = Bd;
      m.Wex.block(0, 0, DIM_X, 1) = Wd;
    } else {
      m.Aex.block(idx_x_i, 0, DIM_X, DIM_X) = Ad * m.Aex.block(idx_x_i_prev, 0, DIM_X, DIM_X);
      for (int j = 0; j < i; ++j) {
        int idx_u_j = j * DIM_U;
        m.Bex.block(idx_x_i, idx_u_j, DIM_X, DIM_U) =
          Ad * m.Bex.block(idx_x_i_prev, idx_u_j, DIM_X, DIM_U);
      }
      m.Wex.block(idx_x_i, 0, DIM_X, 1) = Ad * m.Wex.block(idx_x_i_prev, 0, DIM_X, 1) + Wd;
    }
    m.Bex.block(idx_x_i, idx_u_i, DIM_X, DIM_U) = Bd;
    m.Cex.block(idx_y_i, idx_x_i, DIM_Y, DIM_X) = Cd;
    m.Qex.block(idx_y_i, idx_y_i, DIM_Y, DIM_Y) = Q_adaptive;
    m.R1ex.block(idx_u_i, idx_u_i, DIM_U, DIM_U) = R_adaptive;

    /* get reference input (feed-forward) */
    vehicle_model_ptr_->setCurvature(ref_smooth_k);
    vehicle_model_ptr_->calculateReferenceInput(Uref);
    if (std::fabs(Uref(0, 0)) < DEG2RAD * mpc_param_.zero_ff_steer_deg) {
      Uref(0, 0) = 0.0;  // ignore curvature noise
    }
    m.Urefex.block(i * DIM_U, 0, DIM_U, 1) = Uref;
  }

  /* add lateral jerk : weight for (v * {u(i) - u(i-1)} )^2 */
  for (int i = 0; i < N - 1; ++i) {
    const double ref_vx = reference_trajectory.vx[static_cast<size_t>(i)];
    sign_vx_ = ref_vx > ep ? 1 : (ref_vx < -ep ? -1 : sign_vx_);
    const double ref_k = reference_trajectory.k[static_cast<size_t>(i)] * sign_vx_;
    const double j = ref_vx * ref_vx * getWeightLatJerk(ref_k) / (DT * DT);  // lateral jerk weight
    const Eigen::Matrix2d J = (Eigen::Matrix2d() << j, -j, -j, j).finished();
    m.R2ex.block(i, i, 2, 2) += J;
  }

  addSteerWeightR(&m.R1ex);

  return m;
}

/*
 * solve quadratic optimization.
 * cost function: J = Xex' * Qex * Xex + (Uex - Uref)' * R1ex * (Uex - Urefex) + Uex' * R2ex * Uex
 *                , Qex = diag([Q,Q,...]), R1ex = diag([R,R,...])
 * constraint matrix : lb < U < ub, lbA < A*U < ubA
 * current considered constraint
 *  - steering limit
 *  - steering rate limit
 *
 * (1)lb < u < ub && (2)lbA < Au < ubA --> (3)[lb, lbA] < [I, A]u < [ub, ubA]
 * (1)lb < u < ub ...
 * [-u_lim] < [ u0 ] < [u_lim]
 * [-u_lim] < [ u1 ] < [u_lim]
 *              ~~~
 * [-u_lim] < [ uN ] < [u_lim] (*N... DIM_U)
 * (2)lbA < Au < ubA ...
 * [prev_u0 - au_lim*ctp] < [   u0  ] < [prev_u0 + au_lim*ctp] (*ctp ... ctrl_period)
 * [    -au_lim * dt    ] < [u1 - u0] < [     au_lim * dt    ]
 * [    -au_lim * dt    ] < [u2 - u1] < [     au_lim * dt    ]
 *                            ~~~
 * [    -au_lim * dt    ] < [uN-uN-1] < [     au_lim * dt    ] (*N... DIM_U)
 */
bool MPCFollower::executeOptimization(
  const MPCMatrix & m, const Eigen::VectorXd & x0, Eigen::VectorXd * Uex)
{
  using Eigen::MatrixXd;
  using Eigen::VectorXd;

  if (!isValid(m)) {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      get_logger(), *get_clock(), 1000 /*ms*/, "model matrix is invalid. stop MPC.");
    return false;
  }

  const int DIM_U_N = mpc_param_.prediction_horizon * vehicle_model_ptr_->getDimU();

  // cost function: 1/2 * Uex' * H * Uex + f' * Uex,  H = B' * C' * Q * C * B + R
  const MatrixXd CB = m.Cex * m.Bex;
  const MatrixXd QCB = m.Qex * CB;
  // MatrixXd H = CB.transpose() * QCB + m.R1ex + m.R2ex; // This calculation is heavy. looking for a good way.  //NOLINT
  MatrixXd H = MatrixXd::Zero(DIM_U_N, DIM_U_N);
  H.triangularView<Eigen::Upper>() = CB.transpose() * QCB;
  H.triangularView<Eigen::Upper>() += m.R1ex + m.R2ex;
  H.triangularView<Eigen::Lower>() = H.transpose();
  MatrixXd f = (m.Cex * (m.Aex * x0 + m.Wex)).transpose() * QCB - m.Urefex.transpose() * m.R1ex;
  addSteerWeightF(&f);

  MatrixXd A = MatrixXd::Identity(DIM_U_N, DIM_U_N);
  for (int i = 1; i < DIM_U_N; i++) {
    A(i, i - 1) = -1.0;
  }

  VectorXd lb = VectorXd::Constant(DIM_U_N, -steer_lim_);  // min steering angle
  VectorXd ub = VectorXd::Constant(DIM_U_N, steer_lim_);   // max steering angle
  VectorXd lbA = VectorXd::Constant(DIM_U_N, -steer_rate_lim_ * mpc_param_.prediction_dt);
  VectorXd ubA = VectorXd::Constant(DIM_U_N, steer_rate_lim_ * mpc_param_.prediction_dt);
  lbA(0, 0) = raw_steer_cmd_prev_ - steer_rate_lim_ * ctrl_period_;
  ubA(0, 0) = raw_steer_cmd_prev_ + steer_rate_lim_ * ctrl_period_;

  auto t_start = std::chrono::system_clock::now();
  bool solve_result = qpsolver_ptr_->solve(H, f.transpose(), A, lb, ub, lbA, ubA, *Uex);
  auto t_end = std::chrono::system_clock::now();
  if (!solve_result) {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "qp solver error");
    return false;
  }

  {
    auto t = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();
    RCLCPP_DEBUG(
      get_logger(), "qp solver calculation time = %f [ms]", t);
  }

  if (Uex->array().isNaN().any()) {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      get_logger(), *get_clock(), 1000 /*ms*/, "model Uex includes NaN, stop MPC.");
    return false;
  }
  return true;
}

void MPCFollower::addSteerWeightR(Eigen::MatrixXd * R_ptr) const
{
  const int N = mpc_param_.prediction_horizon;
  const double DT = mpc_param_.prediction_dt;

  auto & R = *R_ptr;

  /* add steering rate : weight for (u(i) - u(i-1) / dt )^2 */
  {
    const double steer_rate_r = mpc_param_.weight_steer_rate / (DT * DT);
    const Eigen::Matrix2d D = steer_rate_r * (Eigen::Matrix2d() << 1.0, -1.0, -1.0, 1.0).finished();
    for (int i = 0; i < N - 1; ++i) {
      R.block(i, i, 2, 2) += D;
    }
    if (N > 1) {
      // steer rate i = 0
      R(0, 0) += mpc_param_.weight_steer_rate / (ctrl_period_ * ctrl_period_);
    }
  }

  /* add steering acceleration : weight for { (u(i+1) - 2*u(i) + u(i-1)) / dt^2 }^2 */
  {
    const double w = mpc_param_.weight_steer_acc;
    const double steer_acc_r = w / std::pow(DT, 4);
    const double steer_acc_r_cp1 = w / (std::pow(DT, 3) * ctrl_period_);
    const double steer_acc_r_cp2 = w / (std::pow(DT, 2) * std::pow(ctrl_period_, 2));
    const double steer_acc_r_cp4 = w / std::pow(ctrl_period_, 4);
    const Eigen::Matrix3d D =
      steer_acc_r *
      (Eigen::Matrix3d() << 1.0, -2.0, 1.0, -2.0, 4.0, -2.0, 1.0, -2.0, 1.0).finished();
    for (int i = 1; i < N - 1; ++i) {
      R.block(i - 1, i - 1, 3, 3) += D;
    }
    if (N > 1) {
      // steer acc i = 1
      R(0, 0) += steer_acc_r * 1.0 + steer_acc_r_cp2 * 1.0 + steer_acc_r_cp1 * 2.0;
      R(1, 0) += steer_acc_r * -1.0 + steer_acc_r_cp1 * -1.0;
      R(0, 1) += steer_acc_r * -1.0 + steer_acc_r_cp1 * -1.0;
      R(1, 1) += steer_acc_r * 1.0;
      // steer acc i = 0
      R(0, 0) += steer_acc_r_cp4 * 1.0;
    }
  }
}

void MPCFollower::addSteerWeightF(Eigen::MatrixXd * f_ptr) const
{
  if (f_ptr->rows() < 2) {
    return;
  }

  const double DT = mpc_param_.prediction_dt;
  auto & f = *f_ptr;

  // steer rate for i = 0
  f(0, 0) += -2.0 * mpc_param_.weight_steer_rate / (std::pow(DT, 2)) * 0.5;

  // const double steer_acc_r = mpc_param_.weight_steer_acc / std::pow(DT, 4);
  const double steer_acc_r_cp1 = mpc_param_.weight_steer_acc / (std::pow(DT, 3) * ctrl_period_);
  const double steer_acc_r_cp2 =
    mpc_param_.weight_steer_acc / (std::pow(DT, 2) * std::pow(ctrl_period_, 2));
  const double steer_acc_r_cp4 = mpc_param_.weight_steer_acc / std::pow(ctrl_period_, 4);

  // steer acc  i = 0
  f(0, 0) += ((-2.0 * raw_steer_cmd_prev_ + raw_steer_cmd_pprev_) * steer_acc_r_cp4) * 0.5;

  // steer acc for i = 1
  f(0, 0) += (-2.0 * raw_steer_cmd_prev_ * (steer_acc_r_cp1 + steer_acc_r_cp2)) * 0.5;
  f(0, 1) += (2.0 * raw_steer_cmd_prev_ * steer_acc_r_cp1) * 0.5;
}

double MPCFollower::getPredictionTime() const
{
  return (mpc_param_.prediction_horizon - 1) * mpc_param_.prediction_dt + mpc_param_.input_delay +
         ctrl_period_;
}

bool MPCFollower::isValid(const MPCMatrix & m) const
{
  if (
    m.Aex.array().isNaN().any() || m.Bex.array().isNaN().any() || m.Cex.array().isNaN().any() ||
    m.Wex.array().isNaN().any() || m.Qex.array().isNaN().any() || m.R1ex.array().isNaN().any() ||
    m.R2ex.array().isNaN().any() || m.Urefex.array().isNaN().any())
  {
    return false;
  }

  if (
    m.Aex.array().isInf().any() || m.Bex.array().isInf().any() || m.Cex.array().isInf().any() ||
    m.Wex.array().isInf().any() || m.Qex.array().isInf().any() || m.R1ex.array().isInf().any() ||
    m.R2ex.array().isInf().any() || m.Urefex.array().isInf().any())
  {
    return false;
  }

  return true;
}
void MPCFollower::onTrajectory(const autoware_auto_msgs::msg::Trajectory::SharedPtr msg)
{
  current_trajectory_ptr_ = msg;

  if (msg->points.size() < 3) {
    RCLCPP_DEBUG(get_logger(), "received path size is < 3, not enough.");
    return;
  }

  if (!isValidTrajectory(*msg)) {
    RCLCPP_ERROR(get_logger(), "Trajectory is invalid!! stop computing.");
    return;
  }

  trajectory_follower::MPCTrajectory mpc_traj_raw;        // received raw trajectory
  trajectory_follower::MPCTrajectory mpc_traj_resampled;  // resampled trajectory
  trajectory_follower::MPCTrajectory mpc_traj_smoothed;   // smooth filtered trajectory

  /* resampling */
  trajectory_follower::MPCUtils::convertToMPCTrajectory(*current_trajectory_ptr_, &mpc_traj_raw);
  if (!trajectory_follower::MPCUtils::resampleMPCTrajectoryByDistance(
      mpc_traj_raw, traj_resample_dist_, &mpc_traj_resampled))
  {
    RCLCPP_WARN(get_logger(), "spline error!!!!!!");
    return;
  }

  /* path smoothing */
  mpc_traj_smoothed = mpc_traj_resampled;
  int mpc_traj_resampled_size = static_cast<int>(mpc_traj_resampled.size());
  if (enable_path_smoothing_ && mpc_traj_resampled_size > 2 * path_filter_moving_ave_num_) {
    if (
      !trajectory_follower::MoveAverageFilter::filt_vector(
        path_filter_moving_ave_num_,
        mpc_traj_smoothed.x) ||
      !trajectory_follower::MoveAverageFilter::filt_vector(
        path_filter_moving_ave_num_,
        mpc_traj_smoothed.y) ||
      !trajectory_follower::MoveAverageFilter::filt_vector(
        path_filter_moving_ave_num_,
        mpc_traj_smoothed.yaw) ||
      !trajectory_follower::MoveAverageFilter::filt_vector(
        path_filter_moving_ave_num_,
        mpc_traj_smoothed.vx))
    {
      RCLCPP_DEBUG(get_logger(), "path callback: filtering error. stop filtering.");
      mpc_traj_smoothed = mpc_traj_resampled;
    }
  }

  /* calculate yaw angle */
  if (enable_yaw_recalculation_) {
    trajectory_follower::MPCUtils::calcTrajectoryYawFromXY(&mpc_traj_smoothed);
    trajectory_follower::MPCUtils::convertEulerAngleToMonotonic(&mpc_traj_smoothed.yaw);
  }

  /* calculate curvature */
  trajectory_follower::MPCUtils::calcTrajectoryCurvature(
    static_cast<size_t>(
      curvature_smoothing_num_), &mpc_traj_smoothed);

  /* add end point with vel=0 on traj for mpc prediction */
  {
    auto & t = mpc_traj_smoothed;
    const double t_ext = 100.0;  // extra time to prevent mpc calculation failure due to short time
    const double t_end = t.relative_time.back() + getPredictionTime() + t_ext;
    const double v_end = 0.0;
    t.vx.back() = v_end;  // set for end point
    t.push_back(
      t.x.back(), t.y.back(), t.z.back(), t.yaw.back(), v_end, t.k.back(), t.smooth_k.back(),
      t_end);
  }

  if (!mpc_traj_smoothed.size()) {
    RCLCPP_DEBUG(get_logger(), "path callback: trajectory size is undesired.");
    return;
  }

  ref_traj_ = mpc_traj_smoothed;
}

void MPCFollower::updateCurrentPose()
{
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      get_logger(), *get_clock(), 5000 /*ms*/,
      "cannot get map to base_link transform. %s", ex.what());
    return;
  }

  geometry_msgs::msg::PoseStamped ps;
  ps.header = transform.header;
  ps.pose.position.x = transform.transform.translation.x;
  ps.pose.position.y = transform.transform.translation.y;
  ps.pose.position.z = transform.transform.translation.z;
  ps.pose.orientation = transform.transform.rotation;
  current_pose_ptr_ = std::make_shared<geometry_msgs::msg::PoseStamped>(ps);
}

void MPCFollower::onSteering(const autoware_auto_msgs::msg::VehicleKinematicState::SharedPtr msg)
{
  current_steer_ptr_ = msg;
}

void MPCFollower::onVelocity(geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  current_velocity_ptr_ = msg;
}

autoware_auto_msgs::msg::AckermannLateralCommand MPCFollower::getStopControlCommand() const
{
  autoware_auto_msgs::msg::AckermannLateralCommand cmd;
  cmd.steering_tire_angle = static_cast<float>(steer_cmd_prev_);
  cmd.steering_tire_rotation_rate = 0.0f;
  return cmd;
}

autoware_auto_msgs::msg::AckermannLateralCommand MPCFollower::getInitialControlCommand() const
{
  autoware_auto_msgs::msg::AckermannLateralCommand cmd;
  cmd.steering_tire_angle = current_steer_ptr_->state.front_wheel_angle_rad;
  cmd.steering_tire_rotation_rate = 0.0f;
  return cmd;
}

bool MPCFollower::isStoppedState() const
{
  const int nearest = trajectory_follower::MPCUtils::calcNearestIndex(
    *current_trajectory_ptr_,
    current_pose_ptr_->pose);
  // If the nearest index is not found, return false
  if (nearest < 0) {return false;}
  const double dist = calcStopDistance(nearest);
  if (dist < stop_state_keep_stopping_dist_) {
    RCLCPP_DEBUG(
      get_logger(),
      "stop_dist = %f < %f : stop_state_keep_stopping_dist_. keep stopping.", dist,
      stop_state_keep_stopping_dist_);
    return true;
  }
  RCLCPP_DEBUG(get_logger(), "stop_dist = %f release stopping.", dist);

  const double current_vel = current_velocity_ptr_->twist.linear.x;
  const double target_vel =
    current_trajectory_ptr_->points.at(static_cast<size_t>(nearest)).longitudinal_velocity_mps;
  if (
    std::fabs(current_vel) < stop_state_entry_ego_speed_ &&
    std::fabs(target_vel) < stop_state_entry_target_speed_)
  {
    return true;
  } else {
    return false;
  }
}

double MPCFollower::calcStopDistance(const int origin) const
{
  constexpr float zero_velocity = std::numeric_limits<float>::epsilon();
  const float origin_velocity =
    current_trajectory_ptr_->points.at(static_cast<size_t>(origin)).longitudinal_velocity_mps;
  double stop_dist = 0.0;

  // search forward
  if (std::fabs(origin_velocity) > zero_velocity) {
    for (int i = origin + 1; i < static_cast<int>(current_trajectory_ptr_->points.size()) - 1;
      ++i)
    {
      const auto & p0 = current_trajectory_ptr_->points.at(static_cast<size_t>(i));
      const auto & p1 = current_trajectory_ptr_->points.at(static_cast<size_t>(i - 1));
      stop_dist += trajectory_follower::MPCUtils::calcDist2d(p0, p1);
      if (std::fabs(p0.longitudinal_velocity_mps) < zero_velocity) {
        break;
      }
    }
    return stop_dist;
  }

  // search backward
  for (int i = origin - 1; 0 < i; --i) {
    const auto & p0 = current_trajectory_ptr_->points.at(static_cast<size_t>(i));
    const auto & p1 = current_trajectory_ptr_->points.at(static_cast<size_t>(i + 1));
    if (std::fabs(p0.longitudinal_velocity_mps) > zero_velocity) {
      break;
    }
    stop_dist -= trajectory_follower::MPCUtils::calcDist2d(p0, p1);
  }
  return stop_dist;
}

void MPCFollower::publishCtrlCmd(autoware_auto_msgs::msg::AckermannLateralCommand ctrl_cmd)
{
  ctrl_cmd.stamp = this->now();
  pub_ctrl_cmd_->publish(ctrl_cmd);
  steer_cmd_prev_ = ctrl_cmd.steering_tire_angle;
}

void MPCFollower::initTimer(double period_s)
{
  auto timer_callback = std::bind(&MPCFollower::onTimer, this);
  const auto period_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period_s));
  timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    this->get_clock(), period_ns, std::move(timer_callback),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(timer_, nullptr);
}

void MPCFollower::declareMPCparameters()
{
  mpc_param_.prediction_horizon = static_cast<int>(declare_parameter("mpc_prediction_horizon", 50));
  mpc_param_.prediction_dt = declare_parameter("mpc_prediction_dt", 0.1);
  mpc_param_.weight_lat_error = declare_parameter("mpc_weight_lat_error", 0.1);
  mpc_param_.weight_heading_error = declare_parameter("mpc_weight_heading_error", 0.0);
  mpc_param_.weight_heading_error_squared_vel = declare_parameter(
    "mpc_weight_heading_error_squared_vel", 0.3);
  mpc_param_.weight_steering_input = declare_parameter("mpc_weight_steering_input", 1.0);
  mpc_param_.weight_steering_input_squared_vel = declare_parameter(
    "mpc_weight_steering_input_squared_vel", 0.25);
  mpc_param_.weight_lat_jerk = declare_parameter("mpc_weight_lat_jerk", 0.0);
  mpc_param_.weight_steer_rate = declare_parameter("mpc_weight_steer_rate", 0.0);
  mpc_param_.weight_steer_acc = declare_parameter("mpc_weight_steer_acc", 0.000001);
  mpc_param_.low_curvature_weight_lat_error = declare_parameter(
    "mpc_low_curvature_weight_lat_error", 0.1);
  mpc_param_.low_curvature_weight_heading_error = declare_parameter(
    "mpc_low_curvature_weight_heading_error", 0.0);
  mpc_param_.low_curvature_weight_heading_error_squared_vel = declare_parameter(
    "mpc_low_curvature_weight_heading_error_squared_vel", 0.3);
  mpc_param_.low_curvature_weight_steering_input = declare_parameter(
    "mpc_low_curvature_weight_steering_input", 1.0);
  mpc_param_.low_curvature_weight_steering_input_squared_vel = declare_parameter(
    "mpc_low_curvature_weight_steering_input_squared_vel", 0.25);
  mpc_param_.low_curvature_weight_lat_jerk = declare_parameter(
    "mpc_low_curvature_weight_lat_jerk",
    0.0);
  mpc_param_.low_curvature_weight_steer_rate = declare_parameter(
    "mpc_low_curvature_weight_steer_rate", 0.0);
  mpc_param_.low_curvature_weight_steer_acc = declare_parameter(
    "mpc_low_curvature_weight_steer_acc", 0.000001);
  mpc_param_.low_curvature_thresh_curvature = declare_parameter(
    "mpc_low_curvature_thresh_curvature", 0.0);
  mpc_param_.weight_terminal_lat_error = declare_parameter("mpc_weight_terminal_lat_error", 1.0);
  mpc_param_.weight_terminal_heading_error = declare_parameter(
    "mpc_weight_terminal_heading_error",
    0.1);
  mpc_param_.zero_ff_steer_deg = declare_parameter("mpc_zero_ff_steer_deg", 0.5);
  mpc_param_.acceleration_limit = declare_parameter("mpc_acceleration_limit", 2.0);
  mpc_param_.velocity_time_constant = declare_parameter("mpc_velocity_time_constant", 0.3);
}

rcl_interfaces::msg::SetParametersResult MPCFollower::paramCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  // strong exception safety wrt MPCParam
  MPCParam param = mpc_param_;
  try {
    UPDATE_MPC_PARAM(param, prediction_horizon);
    UPDATE_MPC_PARAM(param, prediction_dt);
    UPDATE_MPC_PARAM(param, weight_lat_error);
    UPDATE_MPC_PARAM(param, weight_heading_error);
    UPDATE_MPC_PARAM(param, weight_heading_error_squared_vel);
    UPDATE_MPC_PARAM(param, weight_steering_input);
    UPDATE_MPC_PARAM(param, weight_steering_input_squared_vel);
    UPDATE_MPC_PARAM(param, weight_lat_jerk);
    UPDATE_MPC_PARAM(param, weight_steer_rate);
    UPDATE_MPC_PARAM(param, weight_steer_acc);
    UPDATE_MPC_PARAM(param, low_curvature_weight_lat_error);
    UPDATE_MPC_PARAM(param, low_curvature_weight_heading_error);
    UPDATE_MPC_PARAM(param, low_curvature_weight_heading_error_squared_vel);
    UPDATE_MPC_PARAM(param, low_curvature_weight_steering_input);
    UPDATE_MPC_PARAM(param, low_curvature_weight_steering_input_squared_vel);
    UPDATE_MPC_PARAM(param, low_curvature_weight_lat_jerk);
    UPDATE_MPC_PARAM(param, low_curvature_weight_steer_rate);
    UPDATE_MPC_PARAM(param, low_curvature_weight_steer_acc);
    UPDATE_MPC_PARAM(param, low_curvature_thresh_curvature);
    UPDATE_MPC_PARAM(param, weight_terminal_lat_error);
    UPDATE_MPC_PARAM(param, weight_terminal_heading_error);
    UPDATE_MPC_PARAM(param, zero_ff_steer_deg);
    UPDATE_MPC_PARAM(param, acceleration_limit);
    UPDATE_MPC_PARAM(param, velocity_time_constant);

    // initialize input buffer
    update_param(parameters, "input_delay", param.input_delay);
    const int delay_step = static_cast<int>(std::round(param.input_delay / ctrl_period_));
    const double delay = delay_step * ctrl_period_;
    if (param.input_delay != delay) {
      param.input_delay = delay;
      input_buffer_ = std::deque<double>(static_cast<size_t>(delay_step), 0.0);
    }

    // transaction succeeds, now assign values
    mpc_param_ = param;
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    result.successful = false;
    result.reason = e.what();
  }

  return result;
}

bool MPCFollower::isValidTrajectory(const autoware_auto_msgs::msg::Trajectory & traj) const
{
  for (const auto & p : traj.points) {
    if (
      !isfinite(p.x) || !isfinite(p.y) || !isfinite(p.heading.imag) || !isfinite(p.heading.real) ||
      !isfinite(p.longitudinal_velocity_mps) ||
      !isfinite(p.lateral_velocity_mps) || !isfinite(p.lateral_velocity_mps) ||
      !isfinite(p.heading_rate_rps) || !isfinite(p.front_wheel_angle_rad) ||
      !isfinite(p.rear_wheel_angle_rad))
    {
      return false;
    }
  }
  return true;
}

}  // namespace trajectory_follower_nodes
}  // namespace control
}  // namespace motion
}  // namespace autoware

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::motion::control::trajectory_follower_nodes::MPCFollower)
