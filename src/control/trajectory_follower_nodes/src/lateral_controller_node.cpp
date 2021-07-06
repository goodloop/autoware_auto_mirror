// Copyright 2021 The Autoware Foundation
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
  update_param(parameters, "m_mpc" #NAME, PARAM_STRUCT.NAME)

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
  m_tf_buffer(this->get_clock()), m_tf_listener(m_tf_buffer)
{
  using std::placeholders::_1;

  m_mpc.m_ctrl_period = declare_parameter("ctrl_period", 0.03);
  m_enable_path_smoothing = declare_parameter("enable_path_smoothing", true);
  m_enable_yaw_recalculation = declare_parameter("enable_yaw_recalculation", false);
  m_path_filter_moving_ave_num =
    static_cast<int>(declare_parameter("path_filter_moving_ave_num", 35));
  m_curvature_smoothing_num = static_cast<int>(declare_parameter("curvature_smoothing_num", 35));
  m_traj_resample_dist = declare_parameter("traj_resample_dist", 0.1);  // [m]
  m_mpc.m_admissible_position_error = declare_parameter("admissible_position_error", 5.0);
  m_mpc.m_admissible_yaw_error_rad = declare_parameter("admissible_yaw_error_rad", M_PI_2);
  m_mpc.m_use_steer_prediction = declare_parameter("use_steer_prediction", false);
  m_mpc.m_param.steer_tau = declare_parameter("vehicle_model_steer_tau", 0.1);

  /* stop state parameters */
  m_stop_state_entry_ego_speed = declare_parameter("stop_state_entry_ego_speed", 0.2);        // [m]
  m_stop_state_entry_target_speed = declare_parameter("stop_state_entry_target_speed", 0.1);  // [m]
  m_stop_state_keep_stopping_dist = declare_parameter("stop_state_keep_stopping_dist", 0.5);  // [m]

  /* mpc parameters */
  const double steer_lim_deg = declare_parameter("steer_lim_deg", 35.0);
  const double steer_rate_lim_degs = declare_parameter("steer_rate_lim_degs", 150.0);
  m_mpc.m_steer_lim = steer_lim_deg * DEG2RAD;
  m_mpc.m_steer_rate_lim = steer_rate_lim_degs * DEG2RAD;
  const double wheelbase = declare_parameter("wheel_base", 2.7);

  /* vehicle model setup */
  const std::string vehicle_model_type = declare_parameter("vehicle_model_type", "kinematics");
  std::shared_ptr<trajectory_follower::VehicleModelInterface> vehicle_model_ptr;
  if (vehicle_model_type == "kinematics") {
    vehicle_model_ptr =
      std::make_shared<trajectory_follower::KinematicsBicycleModel>(
      wheelbase, m_mpc.m_steer_lim,
      m_mpc.m_param.steer_tau);
  } else if (vehicle_model_type == "kinematics_no_delay") {
    vehicle_model_ptr = std::make_shared<trajectory_follower::KinematicsBicycleModelNoDelay>(
      wheelbase, m_mpc.m_steer_lim);
  } else if (vehicle_model_type == "dynamics") {
    const double mass_fl = declare_parameter("mass_fl", 600.0);
    const double mass_fr = declare_parameter("mass_fr", 600.0);
    const double mass_rl = declare_parameter("mass_rl", 600.0);
    const double mass_rr = declare_parameter("mass_rr", 600.0);
    const double cf = declare_parameter("cf", 155494.663);
    const double cr = declare_parameter("cr", 155494.663);

    // vehicle_model_ptr is only assigned in ctor, so parameter value have to be passed at init time  // NOLINT
    vehicle_model_ptr = std::make_shared<trajectory_follower::DynamicsBicycleModel>(
      wheelbase, mass_fl, mass_fr, mass_rl, mass_rr, cf, cr);
  } else {
    RCLCPP_ERROR(get_logger(), "vehicle_model_type is undefined");
  }

  /* QP solver setup */
  const std::string qp_solver_type = declare_parameter("qp_solver_type", "unconstraint_fast");
  std::shared_ptr<trajectory_follower::QPSolverInterface> qpsolver_ptr;
  if (qp_solver_type == "unconstraint_fast") {
    qpsolver_ptr = std::make_shared<trajectory_follower::QPSolverEigenLeastSquareLLT>();
  } else if (qp_solver_type == "osqp") {
    qpsolver_ptr = std::make_shared<trajectory_follower::QPSolverOSQP>(get_logger());
  } else {
    RCLCPP_ERROR(get_logger(), "qp_solver_type is undefined");
  }

  /* delay compensation */
  {
    const double delay_tmp = declare_parameter("input_delay", 0.0);
    const double delay_step = std::round(delay_tmp / m_mpc.m_ctrl_period);
    m_mpc.m_param.input_delay = delay_step * m_mpc.m_ctrl_period;
    m_mpc.m_input_buffer = std::deque<double>(static_cast<size_t>(delay_step), 0.0);
  }

  /* initialize lowpass filter */
  {
    const double steering_lpf_cutoff_hz = declare_parameter("steering_lpf_cutoff_hz", 3.0);
    const double error_deriv_lpf_cutoff_hz = declare_parameter("error_deriv_lpf_cutoff_hz", 5.0);
    m_mpc.m_lpf_steering_cmd.initialize(m_mpc.m_ctrl_period, steering_lpf_cutoff_hz);
    m_mpc.m_lpf_lateral_error.initialize(m_mpc.m_ctrl_period, error_deriv_lpf_cutoff_hz);
    m_mpc.m_lpf_yaw_error.initialize(m_mpc.m_ctrl_period, error_deriv_lpf_cutoff_hz);
  }

  /* set up ros system */
  initTimer(m_mpc.m_ctrl_period);

  m_pub_ctrl_cmd =
    create_publisher<autoware_auto_msgs::msg::AckermannLateralCommand>(
    "output/lateral_control_cmd",
    1);
  m_pub_predicted_traj =
    create_publisher<autoware_auto_msgs::msg::Trajectory>("output/predicted_trajectory", 1);
  m_sub_ref_path = create_subscription<autoware_auto_msgs::msg::Trajectory>(
    "input/reference_trajectory", rclcpp::QoS{1},
    std::bind(&MPCFollower::onTrajectory, this, _1));
  m_sub_current_vel = create_subscription<geometry_msgs::msg::TwistStamped>(
    "input/current_velocity", rclcpp::QoS{1}, std::bind(&MPCFollower::onVelocity, this, _1));
  m_sub_steering = create_subscription<autoware_auto_msgs::msg::VehicleKinematicState>(
    "input/current_kinematic_state", rclcpp::QoS{1}, std::bind(&MPCFollower::onSteering, this, _1));

  // TODO(Frederik.Beaujean) ctor is too long, should factor out parameter declarations
  declareMPCparameters();

  /* get parameter updates */
  m_set_param_res =
    this->add_on_set_parameters_callback(std::bind(&MPCFollower::paramCallback, this, _1));

  m_mpc.setQPSolver(qpsolver_ptr);
  m_mpc.setVehicleModel(vehicle_model_ptr, vehicle_model_type);

  m_mpc.setLogger(get_logger());
  m_mpc.setClock(get_clock());
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

  if (!m_is_ctrl_cmd_prev_initialized) {
    m_ctrl_cmd_prev = getInitialControlCommand();
    m_is_ctrl_cmd_prev_initialized = true;
  }

  const bool is_mpc_solved = m_mpc.calculateMPC(
    *m_current_steer_ptr,
    m_current_velocity_ptr->twist.linear.x,
    m_current_pose_ptr->pose, ctrl_cmd);

  if (isStoppedState()) {
    // Reset input buffer
    for (auto & value : m_mpc.m_input_buffer) {
      value = m_ctrl_cmd_prev.steering_tire_angle;
    }
    // Use previous command value as previous raw steer command
    m_mpc.m_raw_steer_cmd_prev = m_ctrl_cmd_prev.steering_tire_angle;

    publishCtrlCmd(m_ctrl_cmd_prev);
    return;
  }

  if (!is_mpc_solved) {
    RCLCPP_WARN_SKIPFIRST_THROTTLE(
      get_logger(), *get_clock(), 5000 /*ms*/,
      "MPC is not solved. publish 0 velocity.");
    ctrl_cmd = getStopControlCommand();
  }

  m_ctrl_cmd_prev = ctrl_cmd;
  publishCtrlCmd(ctrl_cmd);
}

bool MPCFollower::checkData()
{
  if (!m_mpc.hasVehicleModel()) {
    RCLCPP_DEBUG(
      get_logger(), "MPC does not have a vehicle model");
    return false;
  }
  if (!m_mpc.hasQPSolver()) {
    RCLCPP_DEBUG(
      get_logger(), "MPC does not have a QP solver");
    return false;
  }

  if (!m_current_pose_ptr || !m_current_velocity_ptr || !m_current_steer_ptr) {
    RCLCPP_DEBUG(
      get_logger(), "waiting data. pose = %d, velocity = %d,  steer = %d",
      m_current_pose_ptr != nullptr, m_current_velocity_ptr != nullptr,
      m_current_steer_ptr != nullptr);
    return false;
  }

  if (m_mpc.m_ref_traj.size() == 0) {
    RCLCPP_DEBUG(get_logger(), "trajectory size is zero.");
    return false;
  }

  return true;
}

void MPCFollower::onTrajectory(const autoware_auto_msgs::msg::Trajectory::SharedPtr msg)
{
  m_current_trajectory_ptr = msg;

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
  trajectory_follower::MPCUtils::convertToMPCTrajectory(*m_current_trajectory_ptr, &mpc_traj_raw);
  if (!trajectory_follower::MPCUtils::resampleMPCTrajectoryByDistance(
      mpc_traj_raw, m_traj_resample_dist, &mpc_traj_resampled))
  {
    RCLCPP_WARN(get_logger(), "spline error!!!!!!");
    return;
  }

  /* path smoothing */
  mpc_traj_smoothed = mpc_traj_resampled;
  int mpc_traj_resampled_size = static_cast<int>(mpc_traj_resampled.size());
  if (m_enable_path_smoothing && mpc_traj_resampled_size > 2 * m_path_filter_moving_ave_num) {
    if (
      !trajectory_follower::MoveAverageFilter::filt_vector(
        m_path_filter_moving_ave_num,
        mpc_traj_smoothed.x) ||
      !trajectory_follower::MoveAverageFilter::filt_vector(
        m_path_filter_moving_ave_num,
        mpc_traj_smoothed.y) ||
      !trajectory_follower::MoveAverageFilter::filt_vector(
        m_path_filter_moving_ave_num,
        mpc_traj_smoothed.yaw) ||
      !trajectory_follower::MoveAverageFilter::filt_vector(
        m_path_filter_moving_ave_num,
        mpc_traj_smoothed.vx))
    {
      RCLCPP_DEBUG(get_logger(), "path callback: filtering error. stop filtering.");
      mpc_traj_smoothed = mpc_traj_resampled;
    }
  }

  /* calculate yaw angle */
  if (m_enable_yaw_recalculation) {
    trajectory_follower::MPCUtils::calcTrajectoryYawFromXY(&mpc_traj_smoothed);
    trajectory_follower::MPCUtils::convertEulerAngleToMonotonic(&mpc_traj_smoothed.yaw);
  }

  /* calculate curvature */
  trajectory_follower::MPCUtils::calcTrajectoryCurvature(
    static_cast<size_t>(
      m_curvature_smoothing_num), &mpc_traj_smoothed);

  /* add end point with vel=0 on traj for mpc prediction */
  {
    auto & t = mpc_traj_smoothed;
    const double t_ext = 100.0;  // extra time to prevent mpc calculation failure due to short time
    const double t_end = t.relative_time.back() + m_mpc.getPredictionTime() + t_ext;
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

  m_mpc.m_ref_traj = mpc_traj_smoothed;
}

void MPCFollower::updateCurrentPose()
{
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = m_tf_buffer.lookupTransform("map", "base_link", tf2::TimePointZero);
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
  m_current_pose_ptr = std::make_shared<geometry_msgs::msg::PoseStamped>(ps);
}

void MPCFollower::onSteering(const autoware_auto_msgs::msg::VehicleKinematicState::SharedPtr msg)
{
  m_current_steer_ptr = msg;
}

void MPCFollower::onVelocity(geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  m_current_velocity_ptr = msg;
}

autoware_auto_msgs::msg::AckermannLateralCommand MPCFollower::getStopControlCommand() const
{
  autoware_auto_msgs::msg::AckermannLateralCommand cmd;
  cmd.steering_tire_angle = static_cast<float>(m_steer_cmd_prev);
  cmd.steering_tire_rotation_rate = 0.0f;
  return cmd;
}

autoware_auto_msgs::msg::AckermannLateralCommand MPCFollower::getInitialControlCommand() const
{
  autoware_auto_msgs::msg::AckermannLateralCommand cmd;
  cmd.steering_tire_angle = m_current_steer_ptr->state.front_wheel_angle_rad;
  cmd.steering_tire_rotation_rate = 0.0f;
  return cmd;
}

bool MPCFollower::isStoppedState() const
{
  const int nearest = trajectory_follower::MPCUtils::calcNearestIndex(
    *m_current_trajectory_ptr,
    m_current_pose_ptr->pose);
  // If the nearest index is not found, return false
  if (nearest < 0) {return false;}
  const double dist = m_mpc.calcStopDistance(*m_current_trajectory_ptr, nearest);
  if (dist < m_stop_state_keep_stopping_dist) {
    RCLCPP_DEBUG(
      get_logger(),
      "stop_dist = %f < %f : m_stop_state_keep_stopping_dist. keep stopping.", dist,
      m_stop_state_keep_stopping_dist);
    return true;
  }
  RCLCPP_DEBUG(get_logger(), "stop_dist = %f release stopping.", dist);

  const double current_vel = m_current_velocity_ptr->twist.linear.x;
  const double target_vel =
    m_current_trajectory_ptr->points.at(static_cast<size_t>(nearest)).longitudinal_velocity_mps;
  if (
    std::fabs(current_vel) < m_stop_state_entry_ego_speed &&
    std::fabs(target_vel) < m_stop_state_entry_target_speed)
  {
    return true;
  } else {
    return false;
  }
}

void MPCFollower::publishCtrlCmd(autoware_auto_msgs::msg::AckermannLateralCommand ctrl_cmd)
{
  ctrl_cmd.stamp = this->now();
  m_pub_ctrl_cmd->publish(ctrl_cmd);
  m_steer_cmd_prev = ctrl_cmd.steering_tire_angle;
}

void MPCFollower::initTimer(double period_s)
{
  auto timer_callback = std::bind(&MPCFollower::onTimer, this);
  const auto period_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period_s));
  m_timer = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    this->get_clock(), period_ns, std::move(timer_callback),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(m_timer, nullptr);
}

void MPCFollower::declareMPCparameters()
{
  m_mpc.m_param.prediction_horizon =
    static_cast<int>(declare_parameter("mpc_prediction_horizon", 50));
  m_mpc.m_param.prediction_dt = declare_parameter("mpc_prediction_dt", 0.1);
  m_mpc.m_param.weight_lat_error = declare_parameter("mpc_weight_lat_error", 0.1);
  m_mpc.m_param.weight_heading_error = declare_parameter("mpc_weight_heading_error", 0.0);
  m_mpc.m_param.weight_heading_error_squared_vel = declare_parameter(
    "mpc_weight_heading_error_squared_vel", 0.3);
  m_mpc.m_param.weight_steering_input = declare_parameter("mpc_weight_steering_input", 1.0);
  m_mpc.m_param.weight_steering_input_squared_vel = declare_parameter(
    "mpc_weight_steering_input_squared_vel", 0.25);
  m_mpc.m_param.weight_lat_jerk = declare_parameter("mpc_weight_lat_jerk", 0.0);
  m_mpc.m_param.weight_steer_rate = declare_parameter("mpc_weight_steer_rate", 0.0);
  m_mpc.m_param.weight_steer_acc = declare_parameter("mpc_weight_steer_acc", 0.000001);
  m_mpc.m_param.low_curvature_weight_lat_error = declare_parameter(
    "mpc_low_curvature_weight_lat_error", 0.1);
  m_mpc.m_param.low_curvature_weight_heading_error = declare_parameter(
    "mpc_low_curvature_weight_heading_error", 0.0);
  m_mpc.m_param.low_curvature_weight_heading_error_squared_vel = declare_parameter(
    "mpc_low_curvature_weight_heading_error_squared_vel", 0.3);
  m_mpc.m_param.low_curvature_weight_steering_input = declare_parameter(
    "mpc_low_curvature_weight_steering_input", 1.0);
  m_mpc.m_param.low_curvature_weight_steering_input_squared_vel = declare_parameter(
    "mpc_low_curvature_weight_steering_input_squared_vel", 0.25);
  m_mpc.m_param.low_curvature_weight_lat_jerk = declare_parameter(
    "mpc_low_curvature_weight_lat_jerk",
    0.0);
  m_mpc.m_param.low_curvature_weight_steer_rate = declare_parameter(
    "mpc_low_curvature_weight_steer_rate", 0.0);
  m_mpc.m_param.low_curvature_weight_steer_acc = declare_parameter(
    "mpc_low_curvature_weight_steer_acc", 0.000001);
  m_mpc.m_param.low_curvature_thresh_curvature = declare_parameter(
    "mpc_low_curvature_thresh_curvature", 0.0);
  m_mpc.m_param.weight_terminal_lat_error = declare_parameter("mpc_weight_terminal_lat_error", 1.0);
  m_mpc.m_param.weight_terminal_heading_error = declare_parameter(
    "mpc_weight_terminal_heading_error",
    0.1);
  m_mpc.m_param.zero_ff_steer_deg = declare_parameter("mpc_zero_ff_steer_deg", 0.5);
  m_mpc.m_param.acceleration_limit = declare_parameter("mpc_acceleration_limit", 2.0);
  m_mpc.m_param.velocity_time_constant = declare_parameter("mpc_velocity_time_constant", 0.3);
}

rcl_interfaces::msg::SetParametersResult MPCFollower::paramCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  // strong exception safety wrt MPCParam
  trajectory_follower::MPCParam param = m_mpc.m_param;
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
    const int delay_step = static_cast<int>(std::round(param.input_delay / m_mpc.m_ctrl_period));
    const double delay = delay_step * m_mpc.m_ctrl_period;
    if (param.input_delay != delay) {
      param.input_delay = delay;
      m_mpc.m_input_buffer = std::deque<double>(static_cast<size_t>(delay_step), 0.0);
    }

    // transaction succeeds, now assign values
    m_mpc.m_param = param;
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
