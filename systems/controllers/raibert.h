#pragma once

#include <drake/common/trajectories/piecewise_polynomial.h>
#include "systems/framework/output_vector.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"

namespace dairlib {
namespace systems {

// TODO(yminchen): we can replace cp with raibert style control. (feedforward
// term is v*T/2)
// TODO(yminchen): we can make global target position an input port of the
// the system if it's needed in the future.

/// RaibertFootPlacement calculates and outputs the desired foot position to
/// achieve the target velocity.
///
/// Controller description:
///  (output) Let the output X (2 dimensional) be the
///  desired foot position to achieve the desired COM
///  velocity according to the Raibert style controller.
///  Since we apply the same control law to both sagital and lateral plane, we
///  will only explain for the case of sagital plane. I.e., consider X is
///  1D here.
///  (input) Let an input be v_des, the desired COM velocity.
///  Then according to the Raibert hopping controller, the desired foot
///  position is given according to:
///  X = T_st/2 * (v) + k * (v - v_des),
///  where v/v_des is the current/desired center of mass velocity, and
///  k_fb is the gain of the feedback term.
///  (Raibert controller uses k_ff = T/2 where T is the stride duration.)
///
///  Additionally, we only apply the above control law when the robot is facing
///  the target position. Otherwise, delta_r = [0; 0].
///
/// Input:
///  - State of the robot
///
/// Output:
///  - A 2D vector, delta_r.
///
/// Requirement: quaternion floating-based Cassie only
class RaibertFootTraj : public drake::systems::LeafSystem<double> {
 public:
  RaibertFootTraj(const RigidBodyTree<double>& tree, double mid_foot_height,
                  double stance_duration_per_leg, int left_foot_idx,
                  Eigen::Vector3d pt_on_left_foot, int right_foot_idx,
                  Eigen::Vector3d pt_on_right_foot, int pelvis_idx,
                  double center_line_offset);

  const drake::systems::InputPort<double>& get_input_port_state() const {
    return this->get_input_port(state_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_fsm() const {
    return this->get_input_port(fsm_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_des_xy_vel() const {
    return this->get_input_port(vel_xy_port_);
  }
  const drake::systems::InputPort<double>& get_input_port_des_yaw() const {
    return this->get_input_port(des_yaw_port_);
  }

 private:
  drake::systems::EventStatus DiscreteVariableUpdate(
      const drake::systems::Context<double>& context,
      drake::systems::DiscreteValues<double>* discrete_state) const;
  void CalcFootPlacement(const drake::systems::Context<double>& context,
                         drake::trajectories::Trajectory<double>* traj) const;

  drake::trajectories::PiecewisePolynomial<double> createSplineForSwingFoot(
      const double start_time_of_this_interval,
      const double end_time_of_this_interval,
      const Eigen::Vector3d& init_swing_foot_pos, const Eigen::Vector2d& CP,
      const double stance_foot_height) const;

  const RigidBodyTree<double>& tree_;
  double t_st_;
  double mid_foot_height_;
  int pelvis_idx_;
  int left_foot_idx_;
  int right_foot_idx_;
  Eigen::Vector3d pt_on_left_foot_;
  Eigen::Vector3d pt_on_right_foot_;
  double center_line_offset_;
  //  double target_pos_offset_;

  int prev_td_swing_foot_idx_;
  int prev_td_time_idx_;
  int prev_fsm_state_idx_;

  Eigen::MatrixXd K_;  // feedback vector
  int state_port_;
  int vel_xy_port_;
  int des_yaw_port_;
  int fsm_port_;

  //  double kp_pos_sagital_;
  //  double kd_pos_sagital_;
  //  double vel_max_sagital_;
  //  double vel_min_sagital_;
  //  double k_fp_ff_sagital_;
  double k_fp_fb_sagital_;

  //  double kp_pos_lateral_;
  //  double kd_pos_lateral_;
  //  double vel_max_lateral_;
  //  double vel_min_lateral_;
  //  double k_fp_ff_lateral_;
  double k_fp_fb_lateral_;

  bool is_quaternion_;

  const int left_stance_ = 0;
  const int right_stance_ = 1;
};

}  // namespace systems
}  // namespace dairlib
