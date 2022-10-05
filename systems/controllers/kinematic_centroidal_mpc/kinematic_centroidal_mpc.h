
#pragma once
#include <drake/solvers/mathematical_program.h>
#include "multibody/kinematic/world_point_evaluator.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/context.h"

class KinematicCentroidalMPC {
 public:
  KinematicCentroidalMPC(const drake::multibody::MultibodyPlant<double>& plant,
                         drake::systems::Context<double>* context,
                         int n_knot_points,
                         double dt,
                         const std::vector<std::shared_ptr<dairlib::multibody::WorldPointEvaluator<double>>>& contact_points);

  void AddCentroidalTrackingReference(std::unique_ptr<drake::trajectories::Trajectory<double>> centroidal_ref_traj,
                                      const Eigen::Matrix<double, 13, 13>& Q_centroidal);

  void AddKinematicTrackingReference(std::unique_ptr<drake::trajectories::Trajectory<double>> kinematic_ref_traj,
                                      const Eigen::MatrixXd& Q_kinematic);

  void AddContactPosTrackingReference(std::unique_ptr<drake::trajectories::Trajectory<double>> contact_ref_traj,
                                     const Eigen::MatrixXd& Q_contact);

  void AddForceTrackingReference(std::unique_ptr<drake::trajectories::Trajectory<double>> force_ref_traj,
                                      const Eigen::MatrixXd& Q_force);

 private:
  void AddCentroidalDynamics();

  void AddKinematicDynamics();

  void AddContactConstraints();

//  void AddFrictionConeConstraints();

  void AddTorqueLimits();

  void AddStateLimits();

  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;

  int n_knot_points_;
  double dt_;

  std::vector<std::shared_ptr<dairlib::multibody::WorldPointEvaluator<double>>> contact_points_;

  const int n_centroidal_pos_ = 7;
  const int n_centroidal_vel_ = 6;
  int n_revolute_joints_;
  int n_contact_points_;

  std::unique_ptr<drake::trajectories::Trajectory<double>> centroidal_ref_traj_;
  Eigen::Matrix<double, 13, 13> Q_centroidal_;
  std::unique_ptr<drake::trajectories::Trajectory<double>> kinematic_ref_traj_;
  Eigen::MatrixXd Q_kinematic_;
  std::unique_ptr<drake::trajectories::Trajectory<double>> contact_ref_traj_;
  Eigen::MatrixXd Q_contact_;
  std::unique_ptr<drake::trajectories::Trajectory<double>> force_ref_traj_;
  Eigen::MatrixXd Q_force_;


  std::vector<std::unordered_map<std::shared_ptr<dairlib::multibody::WorldPointEvaluator<double>>, bool>> contact_sequence_;
  // MathematicalProgram
  std::unique_ptr<drake::solvers::MathematicalProgram> prog_;

  //DecisionVariables
  // kinematic state
  std::vector<drake::solvers::VectorXDecisionVariable>  q_;
  // Centroidal state
  std::vector<drake::solvers::VectorXDecisionVariable>  r_;
  // Contact position index by time, then by contact
  std::vector<std::vector<drake::solvers::VectorXDecisionVariable>>  contact_pos_;
  // Contact force index by time, then by contact
  std::vector<std::vector<drake::solvers::VectorXDecisionVariable>>  contact_force_;
};

