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
                                               contact_points_(contact_points),
                                               n_q_(plant.num_positions()),
                                               n_v_(plant.num_velocities()),
                                               n_contact_points_(contact_points.size()){

  n_kinematic_q_ = n_q_ - n_centroidal_pos_;
  n_kinematic_v_ = n_v_ - n_centroidal_vel_;

  for(int knot = 0; knot < n_knot_points; knot ++){
    x_vars_.push_back(prog_->NewContinuousVariables(n_q_ + n_v_, "x_vars_" + std::to_string(knot)));
    for(int contact = 0; contact < n_contact_points_; contact ++){
      contact_pos_[contact].push_back(prog_->NewContinuousVariables(3, "contact_pos_" + std::to_string(knot) + "_" + std::to_string(contact)));
      contact_force_[contact].push_back(prog_->NewContinuousVariables(3, "contact_force_" + std::to_string(knot) + "_" + std::to_string(contact)));
    }
  }
}

void KinematicCentroidalMPC::AddStateReference(std::unique_ptr<drake::trajectories::Trajectory<double>> ref_traj,
                                               const Eigen::MatrixXd &Q) {
  // Ensure matrix is square and has correct number of rows and columns
  DRAKE_DEMAND(Q.rows() == n_q_ + n_v_);
  DRAKE_DEMAND(Q.cols() == n_q_ + n_v_);

  ref_traj_ = std::move(ref_traj);
  Q_ = Q;

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
