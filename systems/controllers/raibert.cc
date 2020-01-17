#include "systems/controllers/raibert.h"

#include <math.h>
#include <string>

#include "attic/multibody/rigidbody_utils.h"
#include "systems/controllers/control_utils.h"
#include "drake/math/quaternion.h"

using std::cout;
using std::endl;
using std::string;

using dairlib::systems::OutputVector;
using Eigen::MatrixXd;
using Eigen::Quaterniond;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;

using drake::systems::BasicVector;
using drake::systems::Context;
using drake::systems::DiscreteUpdateEvent;
using drake::systems::DiscreteValues;
using drake::systems::EventStatus;
using drake::systems::LeafSystem;
using drake::trajectories::PiecewisePolynomial;

namespace dairlib::systems {

RaibertFootTraj::RaibertFootTraj(const RigidBodyTree<double>& tree,
                                 double mid_foot_height, double stance_time,
                                 int left_foot_idx,
                                 Eigen::Vector3d pt_on_left_foot,
                                 int right_foot_idx,
                                 Eigen::Vector3d pt_on_right_foot,
                                 int pelvis_idx, double center_line_offset)
    : tree_(tree),
      t_st_(stance_time),
      mid_foot_height_(mid_foot_height),
      pelvis_idx_(pelvis_idx),
      left_foot_idx_(left_foot_idx),
      right_foot_idx_(right_foot_idx),
      pt_on_left_foot_(pt_on_left_foot),
      pt_on_right_foot_(pt_on_right_foot),
      center_line_offset_(center_line_offset) {
  // Input/Output Setup
  state_port_ = this
                    ->DeclareVectorInputPort(OutputVector<double>(
                        tree.get_num_positions(), tree.get_num_velocities(),
                        tree.get_num_actuators()))
                    .get_index();
  vel_xy_port_ =
      this->DeclareVectorInputPort(BasicVector<double>(2)).get_index();
  fsm_port_ = this->DeclareVectorInputPort(BasicVector<double>(1)).get_index();
  des_yaw_port_ =
      this->DeclareVectorInputPort(BasicVector<double>(1)).get_index();

  //  this->DeclareVectorOutputPort(BasicVector<double>(2),
  //                                &RaibertFootTraj::CalcFootPlacement);
  PiecewisePolynomial<double> pp(VectorXd::Zero(0));
  drake::trajectories::Trajectory<double>& traj_instance = pp;
  this->DeclareAbstractOutputPort("cp_traj", traj_instance,
                                  &RaibertFootTraj::CalcFootPlacement);

  // Foot placement control (Sagital) parameters
  //  vel_max_sagital_ = 1;
  //  vel_min_sagital_ = -1;  // TODO(yminchen): need to test this

  // Foot placement control (Lateral) parameters
  //  vel_max_lateral_ = 0.5;
  //  vel_min_lateral_ = -0.5;

  // State variables inside this controller block
  DeclarePerStepDiscreteUpdateEvent(&RaibertFootTraj::DiscreteVariableUpdate);
  // The swing foot position in the beginning of the swing phase
  prev_td_swing_foot_idx_ = this->DeclareDiscreteState(3);
  // The time of the last touch down
  prev_td_time_idx_ = this->DeclareDiscreteState(1);
  // The last state of FSM
  prev_fsm_state_idx_ = this->DeclareDiscreteState(-0.1 * VectorXd::Ones(1));

  //  k_fp_ff_lateral_ = 0.08;
  left_foot_idx_ = multibody::GetBodyIndexFromName(tree, "toe_left");
  right_foot_idx_ = multibody::GetBodyIndexFromName(tree, "toe_right");

  k_fp_fb_sagital_ = 0.04;
  k_fp_fb_lateral_ = 0.02;
  K_ = MatrixXd::Zero(2, 2);  // Only for x,y vel
  K_(0, 0) = k_fp_fb_sagital_;
  K_(1, 1) = k_fp_fb_lateral_;

  is_quaternion_ = multibody::IsFloatingBase(tree);
}

EventStatus RaibertFootTraj::DiscreteVariableUpdate(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {
  // Read in finite state machine
  const BasicVector<double>* fsm_output =
      (BasicVector<double>*)this->EvalVectorInput(context, fsm_port_);
  VectorXd fsm_state = fsm_output->get_value();

  auto prev_fsm_state = discrete_state->get_mutable_vector(prev_fsm_state_idx_)
                            .get_mutable_value();

  if (fsm_state(0) != prev_fsm_state(0)) {  // if at touchdown
    prev_fsm_state(0) = fsm_state(0);

    auto swing_foot_pos_td =
        discrete_state->get_mutable_vector(prev_td_swing_foot_idx_)
            .get_mutable_value();
    auto prev_td_time = discrete_state->get_mutable_vector(prev_td_time_idx_)
                            .get_mutable_value();

    // Read in current state
    const OutputVector<double>* robot_output =
        (OutputVector<double>*)this->EvalVectorInput(context, state_port_);

    // Get time
    double timestamp = robot_output->get_timestamp();
    auto current_time = static_cast<double>(timestamp);
    prev_td_time(0) = current_time;

    // Kinematics cache and indices
    KinematicsCache<double> cache = tree_.CreateKinematicsCache();
    VectorXd q = robot_output->GetPositions();
    // Modify the quaternion in the begining when the state is not received from
    // the robot yet (cannot have 0-norm quaternion when using doKinematics)
    if (is_quaternion_) {
      multibody::SetZeroQuaternionToIdentity(&q);
    }
    cache.initialize(q);
    tree_.doKinematics(cache);
    int swing_foot_idx =
        (fsm_state(0) == right_stance_) ? left_foot_idx_ : right_foot_idx_;
    Vector3d pt_on_swing_foot =
        (fsm_state(0) == right_stance_) ? pt_on_left_foot_ : pt_on_right_foot_;

    // Swing foot position (Forward Kinematics) at touchdown
    swing_foot_pos_td =
        tree_.transformPoints(cache, pt_on_swing_foot, swing_foot_idx, 0);
  }

  return EventStatus::Succeeded();
}

PiecewisePolynomial<double> RaibertFootTraj::createSplineForSwingFoot(
    const double start_time_of_this_interval,
    const double end_time_of_this_interval, const Vector3d& init_swing_foot_pos,
    const Vector2d& CP, const double stance_foot_height) const {
  // Two segment of cubic polynomial with velocity constraints
  std::vector<double> T_waypoint = {
      start_time_of_this_interval,
      (start_time_of_this_interval + end_time_of_this_interval) / 2,
      end_time_of_this_interval};

  std::vector<MatrixXd> Y(T_waypoint.size(), MatrixXd::Zero(3, 1));
  // x
  Y[0](0, 0) = init_swing_foot_pos(0);
  Y[1](0, 0) = (init_swing_foot_pos(0) + CP(0)) / 2;
  Y[2](0, 0) = CP(0);
  // y
  Y[0](1, 0) = init_swing_foot_pos(1);
  Y[1](1, 0) = (init_swing_foot_pos(1) + CP(1)) / 2;
  Y[2](1, 0) = CP(1);
  // z
  /// We added stance_foot_height because we want the desired trajectory to be
  /// relative to the stance foot in case the floating base state estimation
  /// drifts.
  Y[0](2, 0) = init_swing_foot_pos(2);
  Y[1](2, 0) = mid_foot_height_ + stance_foot_height;
  Y[2](2, 0) = init_swing_foot_pos(2) + stance_foot_height;
  //  Y[2](2, 0) = desired_final_foot_height_ + stance_foot_height(0);

  std::vector<MatrixXd> Y_dot(T_waypoint.size(), MatrixXd::Zero(3, 1));
  // x
  Y_dot[0](0, 0) = 0;
  Y_dot[1](0, 0) = (CP(0) - init_swing_foot_pos(0)) / t_st_;
  Y_dot[2](0, 0) = 0;
  // y
  Y_dot[0](1, 0) = 0;
  Y_dot[1](1, 0) = (CP(1) - init_swing_foot_pos(1)) / t_st_;
  Y_dot[2](1, 0) = 0;
  // z
  Y_dot[0](2, 0) = 0;
  Y_dot[1](2, 0) = 0;
  //  Y_dot[2](2, 0) = desired_final_vertical_foot_velocity_;
  Y_dot[2](2, 0) = 0;
  PiecewisePolynomial<double> swing_foot_spline =
      PiecewisePolynomial<double>::Cubic(T_waypoint, Y, Y_dot);

  return swing_foot_spline;
}

void RaibertFootTraj::CalcFootPlacement(
    const Context<double>& context,
    drake::trajectories::Trajectory<double>* traj) const {
  // Read in current state
  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  const BasicVector<double>* v_des =
      (BasicVector<double>*)this->EvalVectorInput(context, vel_xy_port_);
  const BasicVector<double>* yaw_des =
      (BasicVector<double>*)this->EvalVectorInput(context, des_yaw_port_);
  const BasicVector<double>* fsm =
      (BasicVector<double>*)this->EvalVectorInput(context, fsm_port_);
  const auto prev_td_time =
      context.get_discrete_state(prev_td_time_idx_).get_value();
  const auto swing_foot_pos_td =
      context.get_discrete_state(prev_td_swing_foot_idx_).get_value();

  VectorXd q = robot_output->GetPositions();
  VectorXd v = robot_output->GetVelocities();

  // Kinematics cache and indices
  KinematicsCache<double> cache = tree_.CreateKinematicsCache();
  multibody::SetZeroQuaternionToIdentity(&q);
  cache.initialize(q);
  tree_.doKinematics(cache);

  // Get center of mass position and velocity
  Vector3d CoM = tree_.centerOfMass(cache);
  MatrixXd J = tree_.centerOfMassJacobian(cache);
  Vector3d com_vel = J * v;

  // Get proximated heading angle of pelvis
  Vector3d pelvis_heading_vec =
      tree_.CalcBodyPoseInWorldFrame(cache, tree_.get_body(pelvis_idx_))
          .linear()
          .col(0);
  double approx_pelvis_yaw =
      atan2(pelvis_heading_vec(1), pelvis_heading_vec(0));

  // Get desired heading direction
  double desired_yaw = yaw_des->get_value()(0);

  // Calculate the current-desired yaw angle difference
  double heading_error = desired_yaw - approx_pelvis_yaw;
  bool pause_walking_position_control =
      heading_error > M_PI / 2 || heading_error < -M_PI / 2;

  Vector3d foot_pos_des_global(0, 0, 0);
  //  Vector3d delta_CP_lateral_3D_global(0, 0, 0);

  // Apply walking speed control only when the robot is facing the target
  // position.
  if (!pause_walking_position_control) {
    // Extract quaternion from floating base position
    Quaterniond Quat(q(3), q(4), q(5), q(6));
    Quaterniond Quat_conj = Quat.conjugate();
    Vector4d quat(q.segment(3, 4));
    Vector4d quad_conj(Quat_conj.w(), Quat_conj.x(), Quat_conj.y(),
                       Quat_conj.z());

    // Calculate local target position and com velocity
    Vector3d local_com_vel = drake::math::quatRotateVec(quad_conj, com_vel);

    Vector2d raibert_foot_pos =
        local_com_vel.head(2) * t_st_ / 2 +
        K_ * (local_com_vel.head(2) - v_des->get_value());

//    std::cout << "des local_pos:" << raibert_foot_pos.transpose() << std::endl;
    std::cout << "v_des:" << v_des->get_value().transpose() << std::endl;
    // Calc local and global desired foot pos
    //    Vector2d raibert_foot_pos =
    //        com_vel.head(2) * t_st_ / 2 +
    //        K_ * (com_vel.head(2) - v_des->get_value());

    int stance_foot_idx;
    Vector3d pt_on_stance_foot;
    if (fsm->get_value()(0) == right_stance_) {
      stance_foot_idx = right_foot_idx_;
      pt_on_stance_foot = pt_on_right_foot_;
    } else {
      stance_foot_idx = left_foot_idx_;
      pt_on_stance_foot = pt_on_left_foot_;
    }
    Vector3d stance_foot_pos =
        tree_.transformPoints(cache, pt_on_stance_foot, stance_foot_idx, 0);

    Vector3d foot_pos_des_local;
    foot_pos_des_local << raibert_foot_pos, 0;
    foot_pos_des_global = drake::math::quatRotateVec(quat, foot_pos_des_local);

    foot_pos_des_global.head(2) = ImposeHalfplaneGuard(
        foot_pos_des_global.head(2), (left_stance_ == fsm->get_value()(0)),
        approx_pelvis_yaw, CoM.head(2), stance_foot_pos.head(2),
        center_line_offset_);
    std::cout << "raibert foot pos:" << foot_pos_des_global.transpose() <<
    std::endl;

//    Vector3d foot_pos_des_local;
//    foot_pos_des_local << raibert_foot_pos, 0;
//    foot_pos_des_global = drake::math::quatRotateVec(quat, foot_pos_des_local);
    //    foot_pos_des_local = foot_pos_des_local + center_line_offset_;

    // Assign foot placement
    //    traj->get_mutable_value() =
    //        (foot_pos_des_global + delta_CP_lateral_3D_global).head(2);

    auto casted_traj = (PiecewisePolynomial<double>*)dynamic_cast<
        PiecewisePolynomial<double>*>(traj);
    *casted_traj = createSplineForSwingFoot(
        prev_td_time(0), prev_td_time(0) + t_st_, swing_foot_pos_td,
        foot_pos_des_global.head(2), stance_foot_pos(2));
  }
}

}  // namespace dairlib::systems
