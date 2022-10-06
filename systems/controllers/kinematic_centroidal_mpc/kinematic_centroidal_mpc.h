
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


  void AddStateReference(std::unique_ptr<drake::trajectories::Trajectory<double>> ref_traj,
                                      const Eigen::MatrixXd& Q);

  void AddContactPosTrackingReference(std::unique_ptr<drake::trajectories::Trajectory<double>> contact_ref_traj,
                                     const Eigen::MatrixXd& Q_contact);

  void AddForceTrackingReference(std::unique_ptr<drake::trajectories::Trajectory<double>> force_ref_traj,
                                      const Eigen::MatrixXd& Q_force);

   drake::solvers::VectorXDecisionVariable state_vars(
      int knotpoint_index) const;

   drake::solvers::VectorXDecisionVariable centroidal_pos_vars(
      int knotpoint_index) const;

   drake::solvers::VectorXDecisionVariable centroidal_vel_vars(
      int knotpoint_index) const;

   drake::solvers::VectorXDecisionVariable kinematic_pos_vars(
      int knotpoint_index) const;

   drake::solvers::VectorXDecisionVariable kinematic_vel_vars(
      int knotpoint_index) const;

 private:
  /*!
   * @brief Adds dynamics for centroidal state
   */
  void AddCentroidalDynamics();

  /*!
   * @brief Enforces zero force for feet in flight
   */
  void AddFlightContactForceConstraints();

  /*!
   * @brief Enforce dynmaics for kinematics
   */
  void AddKinematicDynamics();

  /*!
   * @brief Ensures that contact point for feet line up with kinematics, and for stance feet are not moving
   */
  void AddContactConstraints();

//  void AddFrictionConeConstraints();

//  void AddTorqueLimits();
//
//  void AddStateLimits();

  const drake::multibody::MultibodyPlant<double>& plant_;
  drake::systems::Context<double>* context_;

  int n_knot_points_;
  double dt_;

  std::vector<std::shared_ptr<dairlib::multibody::WorldPointEvaluator<double>>> contact_points_;

  const int n_centroidal_pos_ = 7;
  const int n_centroidal_vel_ = 6;
  int n_q_;
  int n_v_;
  int n_kinematic_q_;
  int n_kinematic_v_;
  int n_contact_points_;

  std::unique_ptr<drake::trajectories::Trajectory<double>> ref_traj_;
  Eigen::MatrixXd Q_;
  std::unique_ptr<drake::trajectories::Trajectory<double>> contact_ref_traj_;
  Eigen::MatrixXd Q_contact_;
  std::unique_ptr<drake::trajectories::Trajectory<double>> force_ref_traj_;
  Eigen::MatrixXd Q_force_;


  std::vector<std::unordered_map<std::shared_ptr<dairlib::multibody::WorldPointEvaluator<double>>, bool>> contact_sequence_;
  // MathematicalProgram
  std::unique_ptr<drake::solvers::MathematicalProgram> prog_;

  //DecisionVariables
  // Full robot state
  std::vector<drake::solvers::VectorXDecisionVariable>  x_vars_;
  // Contact position index by time, then by contact
  std::vector<std::vector<drake::solvers::VectorXDecisionVariable>>  contact_pos_;
  // Contact force index by time, then by contact
  std::vector<std::vector<drake::solvers::VectorXDecisionVariable>>  contact_force_;

  std::vector<std::unique_ptr<drake::systems::Context<double>>> contexts_;

};

