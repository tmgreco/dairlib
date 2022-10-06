#pragma  once

#include <drake/multibody/plant/multibody_plant.h>
#include "drake/common/drake_copyable.h"
#include "drake/common/symbolic.h"
#include "drake/solvers/constraint.h"
#include "drake/systems/trajectory_optimization/multiple_shooting.h"
#include "solvers/nonlinear_constraint.h"
#include "multibody/kinematic/kinematic_evaluator_set.h"

template <typename T>
class CentroidalDynamicsConstraint : public dairlib::solvers::NonlinearConstraint<T> {

 public:
  /// Requires two context pointers to be pasesd as arguments, one for each
  /// knot point. The constraint will create its own pointer for the collocation
  /// point context.
  CentroidalDynamicsConstraint(const drake::multibody::MultibodyPlant<T>& plant,
                              drake::systems::Context<T>* context,
                              int n_contact, int knot_index);

 public:
  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<T>>& x,
                          drake::VectorX<T>* y) const override;

 private:
  drake::VectorX<T> CalcTimeDerivativesWithForce(
      drake::systems::Context<T>* context,
      const drake::VectorX<T>& xCent,
      const drake::VectorX<T>& contact_locations,
      const drake::VectorX<T>& contact_forces) const;

  const drake::multibody::MultibodyPlant<T>& plant_;
  drake::systems::Context<T>* context_;
  int n_x_;
  int n_q_;
  int n_u_;
  int n_contact_;
  const drake::VectorX<T> zero_control_;
};



