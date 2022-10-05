#include "kinematic_centroidal_mpc.h"
#include "drake/common/trajectories/piecewise_polynomial.h"

KinematicCentroidalMPC::KinematicCentroidalMPC(const drake::multibody::MultibodyPlant<double> &plant,
                                               drake::systems::Context<double> *context,
                                               int n_knot_points,
                                               double dt,
                                               const std::vector<std::shared_ptr<dairlib::multibody::WorldPointEvaluator<double>>>& contact_points):
                                               plant_(plant),
                                               context_(context),
                                               n_knot_points_(n_knot_points),
                                               dt_(dt),
                                               n_revolute_joints_(plant.num_positions()-n_centroidal_pos_),
                                               n_contact_points_(contact_points.size()),
                                               contact_points_(contact_points){

  for(int knot = 0; knot < n_knot_points; knot ++){
    q_.push_back(prog_->NewContinuousVariables(n_revolute_joints_ * 2, "q_" + std::to_string(knot)));
    r_.push_back(prog_->NewContinuousVariables(n_centroidal_pos_ * n_centroidal_vel_, "r_" + std::to_string(knot)));
    for(int contact = 0; contact < n_contact_points_; contact ++){
      contact_pos_[contact].push_back(prog_->NewContinuousVariables(3, "contact_pos_" + std::to_string(knot) + "_" + std::to_string(contact)));
      contact_force_[contact].push_back(prog_->NewContinuousVariables(3, "contact_force_" + std::to_string(knot) + "_" + std::to_string(contact)));
    }
  }

  // Maybe do something dynamics cache

}
void KinematicCentroidalMPC::AddCentroidalTrackingReference(std::unique_ptr<drake::trajectories::Trajectory<double>> centroidal_ref_traj,
                                                            const Eigen::Matrix<double, 13, 13> &Q_centroidal) {
  centroidal_ref_traj_ = std::move(centroidal_ref_traj);
  Q_centroidal_ = Q_centroidal;
}

void KinematicCentroidalMPC::AddKinematicTrackingReference(std::unique_ptr<drake::trajectories::Trajectory<double>> kinematic_ref_traj,
                                                           const Eigen::MatrixXd &Q_kinematic) {
  // Ensure matrix is square and has correct number of rows and columns
  DRAKE_DEMAND(Q_kinematic.rows() == 2 * n_revolute_joints_);
  DRAKE_DEMAND(Q_kinematic.cols() == 2 * n_revolute_joints_);

  kinematic_ref_traj_ = std::move(kinematic_ref_traj);
  Q_kinematic_ = Q_kinematic;
}

void KinematicCentroidalMPC::AddContactPosTrackingReference(std::unique_ptr<drake::trajectories::Trajectory<double>> contact_ref_traj,
                                                            const Eigen::MatrixXd &Q_contact) {
  DRAKE_DEMAND(Q_contact_.rows() == 3 * n_contact_points_);
  DRAKE_DEMAND(Q_contact_.cols() == 3 * n_contact_points_);

  contact_ref_traj_ = std::move(contact_ref_traj);
  Q_contact_ = Q_contact;

}

void KinematicCentroidalMPC::AddForceTrackingReference(std::unique_ptr<drake::trajectories::Trajectory<double>> force_ref_traj,
                                                       const Eigen::MatrixXd &Q_force) {
  DRAKE_DEMAND(Q_force.rows() == 3 * n_contact_points_);
  DRAKE_DEMAND(Q_force.cols() == 3 * n_contact_points_);

  force_ref_traj_ = std::move(force_ref_traj);
  Q_force_ = Q_force;
}

void KinematicCentroidalMPC::AddCentroidalDynamics() {
  for(int knot_point = 0; knot_point < n_knot_points_; knot_point ++){

  }
}
