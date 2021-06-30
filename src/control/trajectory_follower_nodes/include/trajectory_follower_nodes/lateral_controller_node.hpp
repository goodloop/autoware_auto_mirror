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

/**
 * @file moc_follower.h
 * @brief mpc follower class
 * @author Takamasa Horibe
 * @date 2019.05.01
 */

#ifndef TRAJECTORY_FOLLOWER_NODES__LATERAL_CONTROLLER_NODE_HPP_
#define TRAJECTORY_FOLLOWER_NODES__LATERAL_CONTROLLER_NODE_HPP_

#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "trajectory_follower_nodes/visibility_control.hpp"

#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "trajectory_follower/interpolate.hpp"
#include "trajectory_follower/lowpass_filter.hpp"
#include "trajectory_follower/mpc_trajectory.hpp"
#include "trajectory_follower/mpc_utils.hpp"
#include "trajectory_follower/qp_solver/qp_solver_osqp.hpp"
#include "trajectory_follower/qp_solver/qp_solver_unconstr_fast.hpp"
#include "trajectory_follower/vehicle_model/vehicle_model_bicycle_dynamics.hpp"
#include "trajectory_follower/vehicle_model/vehicle_model_bicycle_kinematics.hpp"
#include "trajectory_follower/vehicle_model/vehicle_model_bicycle_kinematics_no_delay.hpp"

#include "autoware_auto_msgs/msg/ackermann_lateral_command.hpp"
#include "autoware_auto_msgs/msg/trajectory.hpp"
#include "autoware_auto_msgs/msg/vehicle_kinematic_state.hpp"
#include "osqp_interface/osqp_interface.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace autoware
{
namespace motion
{
namespace control
{
namespace trajectory_follower_nodes
{
namespace trajectory_follower = ::motion::control::trajectory_follower;
/**
 * @class MPC-based waypoints follower class
 * @brief calculate control command to follow reference waypoints
 */

struct MPCData
{
  int nearest_idx;
  double nearest_time;
  geometry_msgs::msg::Pose nearest_pose;
  double steer;
  double predicted_steer;
  double lateral_err;
  double yaw_err;
};
class MPCFollower : public rclcpp::Node
{
public:
  /**
   * @brief constructor
   */
  explicit TRAJECTORY_FOLLOWER_PUBLIC MPCFollower(const rclcpp::NodeOptions & node_options);

  /**
   * @brief destructor
   */
  virtual ~MPCFollower();

private:
  //!< @brief topic publisher for control command
  rclcpp::Publisher<autoware_auto_msgs::msg::AckermannLateralCommand>::SharedPtr
    pub_ctrl_cmd_;
  //!< @brief topic publisher for predicted trajectory
  rclcpp::Publisher<autoware_auto_msgs::msg::Trajectory>::SharedPtr
    pub_predicted_traj_;
  //!< @brief topic subscription for reference waypoints
  rclcpp::Subscription<autoware_auto_msgs::msg::Trajectory>::SharedPtr
    sub_ref_path_;
  //!< @brief subscription for current steering
  rclcpp::Subscription<autoware_auto_msgs::msg::VehicleKinematicState>::SharedPtr
    sub_steering_;
  //!< @brief subscription for current velocity
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
    sub_current_vel_;
  //!< @brief timer to update after a given interval
  rclcpp::TimerBase::SharedPtr timer_;
  //!< initialize timer to work in real, simulation, and replay
  void initTimer(double period_s);

  //!< @brief reference trajectory to be followed
  trajectory_follower::MPCTrajectory ref_traj_;
  //!< @brief lowpass filter for steering command
  trajectory_follower::Butterworth2dFilter lpf_steering_cmd_;
  //!< @brief lowpass filter for lateral error
  trajectory_follower::Butterworth2dFilter lpf_lateral_error_;
  //!< @brief lowpass filter for heading error
  trajectory_follower::Butterworth2dFilter lpf_yaw_error_;
  //!< @brief vehicle model type for MPC
  std::string vehicle_model_type_;
  //!< @brief vehicle model for MPC
  std::shared_ptr<trajectory_follower::VehicleModelInterface> vehicle_model_ptr_;
  //!< @brief qp solver for MPC
  std::shared_ptr<trajectory_follower::QPSolverInterface> qpsolver_ptr_;
  //!< @brief mpc_output buffer for delay time compensation
  std::deque<double> input_buffer_;

  /* parameters for control*/
  //!< @brief control frequency [s]
  double ctrl_period_;
  //!< @brief cutoff frequency for steering command [Hz]
  double steering_lpf_cutoff_hz_;
  //!< @brief use stop cmd when lateral error exceeds this [m]
  double admissible_position_error_;
  //!< @brief use stop cmd when yaw error exceeds this [rad]
  double admissible_yaw_error_rad_;
  //!< @brief steering command limit [rad]
  double steer_lim_;
  //!< @brief steering rate limit [rad/s]
  double steer_rate_lim_;
  //!< @brief vehicle wheelbase length [m]
  double wheelbase_;

  /* parameters for path smoothing */
  //!< @brief flag for path smoothing
  bool enable_path_smoothing_;
  //!< @brief flag for recalculation of yaw angle after resampling
  bool enable_yaw_recalculation_;
  //!< @brief flag to use predicted steer, not measured steer.
  bool use_steer_prediction_;
  //!< @brief param of moving average filter for path smoothing
  int path_filter_moving_ave_num_;
  //!< @brief point-to-point index distance for curvature calculation
  int curvature_smoothing_num_;
  //!< @brief path resampling interval [m]
  double traj_resample_dist_;

  /* parameters for stop state */
  double stop_state_entry_ego_speed_;
  double stop_state_entry_target_speed_;
  double stop_state_keep_stopping_dist_;

  struct MPCParam
  {
    //!< @brief prediction horizon step
    int prediction_horizon;
    //!< @brief prediction horizon sampling time
    double prediction_dt;

    //!< @brief threshold that feed-forward angle becomes zero
    double zero_ff_steer_deg;
    //!< @brief delay time for steering input to be compensated
    double input_delay;
    //!< @brief for trajectory velocity calculation
    double acceleration_limit;
    //!< @brief for trajectory velocity calculation
    double velocity_time_constant;
    //!< @brief time constant for steer model
    double steer_tau;

    // for weight matrix Q
    //!< @brief lateral error weight
    double weight_lat_error;
    //!< @brief heading error weight
    double weight_heading_error;
    //!< @brief heading error * velocity weight
    double weight_heading_error_squared_vel;
    //!< @brief terminal lateral error weight
    double weight_terminal_lat_error;
    //!< @brief terminal heading error weight
    double weight_terminal_heading_error;
    //!< @brief lateral error weight in matrix Q in low curvature point
    double low_curvature_weight_lat_error;
    //!< @brief heading error weight in matrix Q in low curvature point
    double low_curvature_weight_heading_error;
    //!< @brief heading error * velocity weight in matrix Q in low curvature point
    double low_curvature_weight_heading_error_squared_vel;

    // for weight matrix R
    //!< @brief steering error weight
    double weight_steering_input;
    //!< @brief steering error * velocity weight
    double weight_steering_input_squared_vel;
    //!< @brief lateral jerk weight
    double weight_lat_jerk;
    //!< @brief steering rate weight
    double weight_steer_rate;
    //!< @brief steering angle acceleration weight
    double weight_steer_acc;
    //!< @brief steering error weight in matrix R in low curvature point
    double low_curvature_weight_steering_input;
    //!< @brief steering error * velocity weight in matrix R in low curvature point
    double low_curvature_weight_steering_input_squared_vel;
    //!< @brief lateral jerk weight in matrix R in low curvature point
    double low_curvature_weight_lat_jerk;
    //!< @brief steering rate weight in matrix R in low curvature point
    double low_curvature_weight_steer_rate;
    //!< @brief steering angle acceleration weight in matrix R in low curvature
    double low_curvature_weight_steer_acc;

    //!< @brief threshold of curvature to use "low curvature" parameter
    double low_curvature_thresh_curvature;
  };
  // for mpc design parameter
  MPCParam mpc_param_;

  struct MPCMatrix
  {
    Eigen::MatrixXd Aex;
    Eigen::MatrixXd Bex;
    Eigen::MatrixXd Wex;
    Eigen::MatrixXd Cex;
    Eigen::MatrixXd Qex;
    Eigen::MatrixXd R1ex;
    Eigen::MatrixXd R2ex;
    Eigen::MatrixXd Urefex;
    Eigen::MatrixXd Yrefex;
  };

  //!< @brief measured pose
  geometry_msgs::msg::PoseStamped::SharedPtr current_pose_ptr_;
  //!< @brief measured velocity
  geometry_msgs::msg::TwistStamped::SharedPtr current_velocity_ptr_;
  //!< @brief measured steering
  autoware_auto_msgs::msg::VehicleKinematicState::SharedPtr current_steer_ptr_;
  autoware_auto_msgs::msg::Trajectory::SharedPtr
  //!< @brief reference trajectory
    current_trajectory_ptr_;

  //!< @brief mpc raw output in previous period
  double raw_steer_cmd_prev_ = 0.0;
  //!< @brief mpc raw output in two times previous period
  double raw_steer_cmd_pprev_ = 0.0;
  //!< @brief mpc filtered output in previous period
  double steer_cmd_prev_ = 0.0;
  //!< @brief previous lateral error for derivative
  double lateral_error_prev_ = 0.0;
  //!< @brief previous lateral error for derivative
  double yaw_error_prev_ = 0.0;

  std::shared_ptr<double> steer_prediction_prev_;
  rclcpp::Time time_prev_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  //!< @brief sign of previous target speed to calculate curvature when the target speed is 0.
  double sign_vx_ = 0.0;
  std::vector<autoware_auto_msgs::msg::AckermannLateralCommand>
  //!< buffer of send command
  ctrl_cmd_vec_;

  //!< @brief flag of ctrl_cmd_prev_ initialization
  bool is_ctrl_cmd_prev_initialized_ = false;
  //!< @brief previous control command
  autoware_auto_msgs::msg::AckermannLateralCommand ctrl_cmd_prev_;

  tf2_ros::Buffer tf_buffer_;
  //!< @brief tf listener
  tf2_ros::TransformListener tf_listener_;

  /**
   * @brief compute and publish control command for path follow with a constant control period
   */
  void onTimer();

  /**
   * @brief set current_trajectory_ with received message
   */
  void onTrajectory(const autoware_auto_msgs::msg::Trajectory::SharedPtr);

  /**
   * @brief update current_pose from tf
   */
  void updateCurrentPose();

  /**
   * @brief check if the received data is valid.
   */
  bool checkData();

  /**
   * @brief get variables for mpc calculation
   */
  bool getData(const trajectory_follower::MPCTrajectory & traj, MPCData * data);

  double calcSteerPrediction();
  double getSteerCmdSum(
    const rclcpp::Time & t_start, const rclcpp::Time & t_end,
    const double time_constant);
  void storeSteerCmd(const double steer);

  /**
   * @brief set current_steer with received message
   */
  void onSteering(const autoware_auto_msgs::msg::VehicleKinematicState::SharedPtr msg);

  /**
   * @brief set current_velocity with received message
   */
  void onVelocity(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

  /**
   * @brief publish control command as autoware_msgs/ControlCommand type
   * @param [in] cmd published control command
   */
  void publishCtrlCmd(autoware_auto_msgs::msg::AckermannLateralCommand cmd);

  /**
   * @brief calculate control command by MPC algorithm
   * @param [out] cmd calculated control command with mpc algorithm
   */
  bool calculateMPC(autoware_auto_msgs::msg::AckermannLateralCommand * cmd);

  /**
   * @brief set initial condition for mpc
   * @param [in] mpc data
   */
  Eigen::VectorXd getInitialState(const MPCData & data);

  /**
   * @brief update status for delay compensation
   * @param [in] start_time start time for update
   * @param [out] x updated state at delayed_time
   */
  bool updateStateForDelayCompensation(
    const trajectory_follower::MPCTrajectory & traj, const double & start_time,
    Eigen::VectorXd * x);

  /**
   * @brief generate MPC matrix with trajectory and vehicle model
   * @param [in] reference_trajectory used for linearization around reference trajectory
   */
  MPCMatrix generateMPCMatrix(const trajectory_follower::MPCTrajectory & reference_trajectory);

  /**
   * @brief generate MPC matrix with trajectory and vehicle model
   * @param [out] Uex optimized input vector
   */
  bool executeOptimization(
    const MPCMatrix & mpc_matrix, const Eigen::VectorXd & x0, Eigen::VectorXd * Uex);

  /**
   * @brief get stop command
   */
  autoware_auto_msgs::msg::AckermannLateralCommand getStopControlCommand() const;

  /**
   * @brief get initial command
   */
  autoware_auto_msgs::msg::AckermannLateralCommand getInitialControlCommand() const;

  /**
   * @brief check ego car is in stopped state
   */
  bool isStoppedState() const;

  /**
   * @brief calculate distance to stop point
   */
  double calcStopDistance(const int origin) const;

  /**
   * @brief resample trajectory with mpc resampling time
   */
  bool resampleMPCTrajectoryByTime(
    double start_time, const trajectory_follower::MPCTrajectory & input,
    trajectory_follower::MPCTrajectory * output) const;

  /**
   * @brief apply velocity dynamics filter with v0 from closest index
   */
  trajectory_follower::MPCTrajectory applyVelocityDynamicsFilter(
    const trajectory_follower::MPCTrajectory & trajectory, const double v0);

  /**
   * @brief get total prediction time of mpc
   */
  double getPredictionTime() const;

  /**
   * @brief add weights related to lateral_jerk, steering_rate, steering_acc into R
   */
  void addSteerWeightR(Eigen::MatrixXd * R) const;

  /**
   * @brief add weights related to lateral_jerk, steering_rate, steering_acc into f
   */
  void addSteerWeightF(Eigen::MatrixXd * f) const;

  /**
   * @brief check if the matrix has invalid value
   */
  bool isValid(const MPCMatrix & m) const;

  /**
   * @brief check if the trajectory has valid value
   */
  bool isValidTrajectory(const autoware_auto_msgs::msg::Trajectory & traj) const;

  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  /**
   * @brief Declare MPC parameters as ROS parameters to allow tuning on the fly
   */
  void declareMPCparameters();

  /**
   * @brief Called when parameters are changed outside of node
   */
  rcl_interfaces::msg::SetParametersResult paramCallback(
    const std::vector<rclcpp::Parameter> & parameters);


  bool isLowCurvature(const double curvature)
  {
    return std::fabs(curvature) < mpc_param_.low_curvature_thresh_curvature;
  }

  double getWeightLatError(const double curvature)
  {
    return isLowCurvature(curvature) ? mpc_param_.low_curvature_weight_lat_error :
           mpc_param_.weight_lat_error;
  }

  double getWeightHeadingError(const double curvature)
  {
    return isLowCurvature(curvature) ? mpc_param_.low_curvature_weight_heading_error :
           mpc_param_.weight_heading_error;
  }

  double getWeightHeadingErrorSqVel(const double curvature)
  {
    return isLowCurvature(curvature) ? mpc_param_.low_curvature_weight_heading_error_squared_vel :
           mpc_param_.weight_heading_error_squared_vel;
  }

  double getWeightSteerInput(const double curvature)
  {
    return isLowCurvature(curvature) ? mpc_param_.low_curvature_weight_steering_input :
           mpc_param_.weight_steering_input;
  }

  double getWeightSteerInputSqVel(const double curvature)
  {
    return isLowCurvature(curvature) ? mpc_param_.low_curvature_weight_steering_input_squared_vel :
           mpc_param_.weight_steering_input_squared_vel;
  }

  double getWeightLatJerk(const double curvature)
  {
    return isLowCurvature(curvature) ? mpc_param_.low_curvature_weight_lat_jerk :
           mpc_param_.weight_lat_jerk;
  }

  double getWeightSteerRate(const double curvature)
  {
    return isLowCurvature(curvature) ? mpc_param_.low_curvature_weight_steer_rate :
           mpc_param_.weight_steer_rate;
  }

  double getWeightSteerAcc(const double curvature)
  {
    return isLowCurvature(curvature) ? mpc_param_.low_curvature_weight_steer_acc :
           mpc_param_.weight_steer_acc;
  }
};
}  // namespace trajectory_follower_nodes
}  // namespace control
}  // namespace motion
}  // namespace autoware

#endif  // TRAJECTORY_FOLLOWER_NODES__LATERAL_CONTROLLER_NODE_HPP_
