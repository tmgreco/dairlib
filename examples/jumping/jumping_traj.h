#pragma once

#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "systems/framework/output_vector.h"
#include "systems/controllers/control_utils.h"

namespace dairlib {
namespace examples{
namespace jumping {
namespace osc {

class JumpingTraj : public drake::systems::LeafSystem<double> {
 public:
  JumpingTraj(const RigidBodyTree<double>& tree,
				int pelvis_idx,
				int left_foot_idx,
				int right_foot_idx,
				drake::trajectories::Trajectory<double>* crouch_traj,
				double height = 0.8);

  const drake::systems::InputPort<double>& get_input_port_state() const {
	return this->get_input_port(state_port_);
  }

 private:
  void CalcDesiredTraj(const drake::systems::Context<double>& context,
				 drake::trajectories::Trajectory<double>* traj) const;

  const RigidBodyTree<double>& tree_;
  int hip_idx_;
  int left_foot_idx_;
  int right_foot_idx_;
  double height_;

  int state_port_;

  // Eigen::Vector3d front_contact_disp_ = Eigen::Vector3d(-0.0457, 0.112, 0);
  // Eigen::Vector3d rear_contact_disp_ = Eigen::Vector3d(0.088, 0, 0);

  //testing
  std::unique_ptr<double> first_msg_time_;
};

}  // namespace osc
}  // namespace jumping
}  // namespace examples
}  // namespace dairlib


