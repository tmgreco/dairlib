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
                              drake::systems::Context<T>* context_0,
                              drake::systems::Context<T>* context_1, int knot_index);

 public:
  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<T>>& x,
                          drake::VectorX<T>* y) const override;

 private:
  drake::VectorX<T> CalcTimeDerivativesWithForce(
      drake::systems::Context<T>* context,
      const drake::VectorX<T>& forces) const;

  const drake::multibody::MultibodyPlant<T>& plant_;
  const dairlib::multibody::KinematicEvaluatorSet<T>& evaluators_;
  drake::systems::Context<T>* context_0_;
  drake::systems::Context<T>* context_1_;
  std::unique_ptr<drake::systems::Context<T>> context_col_;
  const std::vector<int> quat_start_indices_;
  int n_x_;
  int n_u_;
  int n_l_;
  DynamicsCache<T>* cache_;
};
template<typename T>
CentroidalDynamicsConstraint<T>::CentroidalDynamicsConstraint(const drake::multibody::MultibodyPlant<T> &plant,
                                                              drake::systems::Context<T> *context_0,
                                                              drake::systems::Context<T> *context_1,
                                                              int knot_index): dairlib::solvers::NonlinearConstraint<T>(
                                                              5,
                                                              Eigen::VectorXd::Zero(plant.num_positions() + plant.num_velocities()),
                                                              Eigen::VectorXd::Zero(plant.num_positions() + plant.num_velocities()),
                                                              "collocation["+
                                                                  std::to_string(knot_index) + "]"),
                                                              plant_(plant),
                                                              context_0_(context_0),
                                                              context_1_(context_1),
                                                              context_col_(plant.CreateDefaultContext()),
                                                              quat_start_indices_(multibody::QuaternionStartIndices(plant)),
                                                              n_x_(plant.num_positions() + plant.num_velocities()),
                                                              n_u_(plant.num_actuators()),
                                                              n_l_(evaluators.count_full()),
                                                              cache_(cache) {}



