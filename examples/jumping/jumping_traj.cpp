#pragma once

#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "systems/framework/output_vector.h"
#include "systems/controllers/control_utils.h"

namespace dairlib {
namespace jumping {
namespace osc {


JumpingTraj(const RigidBodyTree<double>& tree,
                   int pelvis_idx,
                   int left_foot_idx,
                   int right_foot_idx,
                   double height = 0.9){







}



void Jumping::CalcDesiredTraj(const drake::systems::Context<double>& context,
                 drake::trajectories::Trajectory<double>* traj) const;

}  // namespace osc
}  // namespace cassie
}  // namespace dairlib


