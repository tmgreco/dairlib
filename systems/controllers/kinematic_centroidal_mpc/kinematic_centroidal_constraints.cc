//
// Created by shane on 10/5/22.
//

#include <drake/math/quaternion.h>
#include "kinematic_centroidal_constraints.h"
#include "multibody/multibody_utils.h"

template<typename T>
CentroidalDynamicsConstraint<T>::CentroidalDynamicsConstraint(const drake::multibody::MultibodyPlant<T> &plant,
                                                              drake::systems::Context<T> *context,
                                                              int n_contact,
                                                              double dt,
                                                              int knot_index): dairlib::solvers::NonlinearConstraint<T>(
    13, 1 + 2 * plant.num_positions() + 2 * plant.num_velocities() + 2 * 3 * n_contact,
    Eigen::VectorXd::Zero(13),
    Eigen::VectorXd::Zero(13),
    "centroidal_collocation[" +
        std::to_string(knot_index) + "]"),
                                                                               plant_(plant),
                                                                               context_(context),
                                                                               n_x_(plant.num_positions()
                                                                                        + plant.num_velocities()),
                                                                               n_q_(plant.num_positions()),
                                                                               n_u_(plant.num_actuators()),
                                                                               n_contact_(n_contact),
                                                                               dt_(dt),
                                                                               zero_control_(Eigen::VectorXd::Zero(n_u_)) {}


/// The format of the input to the eval() function is in the order
///   - x0, state at time k
///   - x1, state at time k+1
///   - cj0, contact locations time k
///   - Fj0, contact forces at time k
template <typename T>
void CentroidalDynamicsConstraint<T>::EvaluateConstraint(
    const Eigen::Ref<const drake::VectorX<T>>& x, drake::VectorX<T>* y) const {
  // Extract decision variables
  const auto& x0 = x.segment(0, n_x_);
  const auto& x1 = x.segment(n_x_, n_x_);
  const auto& cj0 = x.segment(n_x_ + n_x_, 3 * n_contact_);
  const auto& Fj0 = x.segment(n_x_ + n_x_ + 3 * n_contact_, 3 * n_contact_);

  // Extract centroidal state = [qw, qx, qy, qz, rx, ry ,rz, wx, wy, wz, drz, dry, drz]
  drake::VectorX<T> x0Cent(13);
  x0Cent <<x0.segment(0, 7) , x0.segment(n_q_, 6);
  drake::VectorX<T> x1Cent(13);
  x1Cent <<x1.segment(0, 7) , x1.segment(n_q_, 6);

  // Evaluate dynamics at k and k+1
  dairlib::multibody::SetContext<T>(plant_, x0, zero_control_, context_);

  const auto& xdot0Cent = CalcTimeDerivativesWithForce(context_, x0Cent,cj0, Fj0);

  // Cubic interpolation to get xcol and xdotcol.
  const auto& x1Predict = x0Cent + xdot0Cent * dt_;

  *y = x1Cent - x1Predict;
}
template<typename T>
drake::VectorX<T> CentroidalDynamicsConstraint<T>::CalcTimeDerivativesWithForce(drake::systems::Context<T> *context,
                                                                                const drake::VectorX<T>& xCent,
                                                                                const drake::VectorX<T>& contact_locations,
                                                                                const drake::VectorX<T>& contact_forces) const {
  const drake::Vector4<T>& quat = xCent.segment(0, 4);
  const auto& r = xCent.segment(4, 3);
  const drake::Vector3<T>& omega = xCent.segment(7, 3);
  const auto& d_r = xCent.segment(10, 3);

  const auto& body_frame = plant_.get_body(*(plant_.GetFloatingBaseBodies().begin())).body_frame();
  const drake::multibody::SpatialInertia< T >& spatial_inertia = plant_.CalcSpatialInertia(*context, body_frame, plant_.GetBodyIndices(drake::multibody::default_model_instance()));
  const auto& rotational_inertia = spatial_inertia.CalcRotationalInertia().CopyToFullMatrix3();
  const auto& mass = spatial_inertia.get_mass();

  drake::Vector3<T> sum_moments;
  drake::Vector3<T> sum_forces;
  for(int contact = 0; contact < n_contact_; contact ++){
    const drake::Vector3<T>& location = contact_locations.segment(contact * 3, 3);
    const drake::Vector3<T>& force = contact_forces.segment(contact * 3, 3);

    sum_moments = sum_forces + (location - r).cross(force);
    sum_forces = sum_forces + force;
  }

  const auto d_quat = drake::math::CalculateQuaternionDtFromAngularVelocityExpressedInB(Eigen::Quaternion<T>(quat), omega);
  // Check to make sure the rotation is correct
  const auto d_omega = rotational_inertia.inverse()* (drake::math::RotationMatrix(Eigen::Quaternion<T>(quat)).transpose() * sum_moments - omega.cross(rotational_inertia * omega));
  const auto dd_r = sum_forces/mass + drake::Vector3<T>(0, 0, 9.81);

  drake::Vector<T, 13> rv;
  rv << d_quat, d_r, d_omega, dd_r;
  return rv;
}
DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS (class CentroidalDynamicsConstraint);
