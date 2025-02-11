#pragma once

#include <memory>

#include "drake/solvers/constraint.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "systems/trajectory_optimization/dircon_kinematic_data.h"

namespace dairlib {

template <typename T>
class DirconPositionData : public DirconKinematicData<T> {
 public:
  DirconPositionData(const drake::multibody::MultibodyPlant<T>& plant,
    const drake::multibody::Body<T>& body,
    Eigen::Vector3d pt, bool isXZ = false,
    Eigen::Vector3d surface_normal = Eigen::Vector3d(0,0,1));
  ~DirconPositionData();

  // The workhorse function, updates and caches everything needed by the
  // outside world
  void updateConstraint(const drake::systems::Context<T>& context);

  void addFixedNormalFrictionConstraints(double mu);

 private:
    const drake::multibody::Body<T>& body_;
    Eigen::Vector3d pt_;
    bool isXZ_;
    Eigen::Matrix<double, 2, 3> TXZ_;
    Eigen::Matrix3d T_ground_incline_;
    Eigen::Matrix<double, 2, 3> TXZ_and_ground_incline_;
};
}  // namespace dairlib
