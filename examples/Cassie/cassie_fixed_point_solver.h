#include "examples/Cassie/cassie_utils.h"
#include "multibody/kinematic/kinematic_evaluator_set.h"
#include "solvers/nonlinear_constraint.h"

namespace dairlib {
/// Utility method to solve for a fixed point for Cassie
/// This is a very narrow method, but could be useful across different
/// Cassie examples
/// @param plant 
/// @param height The pelvis height to solve for
/// @param mu Coefficient of friction
/// @param min_normal_force Minimum normal force at each contact point
/// @param linear_friction_cone use linear approximation of friction cone
/// @param toe_spread y-position of the toes (constrained)
/// @param q Pointer to the resulting position
/// @param u Pointer to the resulting actuation input
/// @param lambda Pointer to the constraint force, though not that useful
///    without exposing the underlying constraint set/plant, it is included
///    here for debugging purposes
/// @param visualize_model_urdf Draw the resulting pose via DrakeVisualizer
///    Requires a model file location to draw. Default is "" (no draw)
void CassieFixedPointSolver(
    const drake::multibody::MultibodyPlant<double>& plant, double height,
    double mu, double min_normal_force, bool linear_friction_cone,
    double toe_spread, Eigen::VectorXd* q_result, Eigen::VectorXd* u_result,
    Eigen::VectorXd* lambda_result, std::string visualize_model_urdf = "",
    double ground_incline = 0, Eigen::VectorXd* all_sol = nullptr);

/// Utility method to solve for loop constraints for Cassie for a neutral
/// position
/// @param plant 
/// @param q Pointer to the resulting position
/// @param u Pointer to the resulting actuation input
/// @param lambda Pointer to the constraint force, though not that useful
///    without exposing the underlying constraint set/plant, it is included
///    here for debugging purposes
/// @param visualize_model_urdf Draw the resulting pose via DrakeVisualizer
///    Requires a model file location to draw. Default is "" (no draw)
void CassieFixedBaseFixedPointSolver(
    const drake::multibody::MultibodyPlant<double>& plant,
    Eigen::VectorXd* q_result, Eigen::VectorXd* u_result,
    Eigen::VectorXd* lambda_result, std::string visualize_model_urdf = "");

/// Solve for a dynamically feasible state given Cassie's pelvis xy velocity.
/// This class is similar to CassieFixedPointSolver expect for the target pelvis
/// velocity.
void CassieInitStateSolver(
    const drake::multibody::MultibodyPlant<double>& plant,
    const Eigen::VectorXd& pelvis_xy_vel, double height, double mu,
    double min_normal_force, bool linear_friction_cone, double toe_spread,
    double ground_incline, const Eigen::VectorXd& q_desired,
    const Eigen::VectorXd& u_desired, const Eigen::VectorXd& lambda_desired,
    Eigen::VectorXd* q_result, Eigen::VectorXd* v_result,
    Eigen::VectorXd* u_result, Eigen::VectorXd* lambda_result);

/// Constraint for CassieInitStateSolver
class VdotConstraint : public dairlib::solvers::NonlinearConstraint<double> {
 public:
  VdotConstraint(const drake::multibody::MultibodyPlant<double>& plant,
                 const multibody::KinematicEvaluatorSet<double>& evaluators);
  ~VdotConstraint() override = default;

  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<double>>& vars,
                          drake::VectorX<double>* y) const override;

 private:
  const drake::multibody::MultibodyPlant<double>& plant_;
  const drake::multibody::BodyFrame<double>& world_;
  std::unique_ptr<drake::systems::Context<double>> context_;
  const multibody::KinematicEvaluatorSet<double>& evaluators_;
  int n_q_;
  int n_v_;
};

/// Constraint for CassieInitStateSolver
class BodyPointVelConstraint
    : public dairlib::solvers::NonlinearConstraint<double> {
 public:
  BodyPointVelConstraint(
      const drake::multibody::MultibodyPlant<double>& plant,
      const multibody::KinematicEvaluatorSet<double>& evaluators);
  ~BodyPointVelConstraint() override = default;

  void EvaluateConstraint(const Eigen::Ref<const drake::VectorX<double>>& vars,
                          drake::VectorX<double>* y) const override;

 private:
  const drake::multibody::MultibodyPlant<double>& plant_;
  const drake::multibody::BodyFrame<double>& world_;
  std::unique_ptr<drake::systems::Context<double>> context_;
  const multibody::KinematicEvaluatorSet<double>& evaluators_;
  int n_q_;
  int n_v_;
};

}  // namespace dairlib
