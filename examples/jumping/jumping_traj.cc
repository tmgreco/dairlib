#pragma once

#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "systems/framework/output_vector.h"
#include "systems/controllers/control_utils.h"



using std::cout;
using std::endl;
using std::string;

using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;

using drake::systems::Context;
using drake::systems::DiscreteValues;
using drake::systems::DiscreteUpdateEvent;
using drake::systems::EventStatus;
using drake::systems::BasicVector;

using drake::trajectories::Trajectory;
using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::ExponentialPlusPiecewisePolynomial;



namespace dairlib {
namespace jumping {
namespace osc {


JumpingTraj::JumpingTraj(	const RigidBodyTree<double>& tree,
							drake::trajectories::Trajectory<double>& crouchTraj,
							double height = 0.79) :
					crouch_traj_(crouchTraj){


}



Trajectory<double> generateNeutralTraj(const drake::systems::Context<double>& context,
										VectorXd& q, VectorXd& v){

	// Kinematics cache and indices
	KinematicsCache<double> cache = tree_.CreateKinematicsCache();
	// Modify the quaternion in the begining when the state is not received from
	// the robot yet
	// Always remember to check 0-norm quaternion when using doKinematics
	multibody::SetZeroQuaternionToIdentity(&q);
	cache.initialize(q);
	tree_.doKinematics(cache);

	Vector2d l_foot = tree_.transformPoints(cache, 0, left_foot_idx, 0); 
	Vector2d r_foot = tree_.transformPoints(cache, 0, right_foot_idx, 0); 

	Vector3d feet_center = (l_foot + r_foot) / 2;

	Vector3d desired_com(feet_center(0), feet_center(1), feet_center(2) + height_);
	return PiecewisePolynomial<double>(desired_com);
}

Trajectory<double> generateCrouchTraj(const drake::systems::Context<double>& context,
										VectorXd& q, VectorXd& v){
	// Kinematics cache and indices
	KinematicsCache<double> cache = tree_.CreateKinematicsCache();
	// Modify the quaternion in the begining when the state is not received from
	// the robot yet
	// Always remember to check 0-norm quaternion when using doKinematics
	multibody::SetZeroQuaternionToIdentity(&q);
	cache.initialize(q);
	tree_.doKinematics(cache);

	//Follow the saved trajectory generated from trajectory optimization
	// Maybe parameterized by q,v from the neutral traj??





	return PiecewisePolynomial<double>();
}

/*
	Move the feet relative to the COM
	The trajectory of the COM cannot be altered, so must solve for 
	foot positions as a function of COM. 

*/
Trajectory<double> generateFlightTraj(const drake::systems::Context<double>& context,
										VectorXd& q, VectorXd& v){

	// Kinematics cache and indices
	KinematicsCache<double> cache = tree_.CreateKinematicsCache();
	// Modify the quaternion in the begining when the state is not received from
	// the robot yet
	// Always remember to check 0-norm quaternion when using doKinematics
	multibody::SetZeroQuaternionToIdentity(&q);
	cache.initialize(q);
	tree_.doKinematics(cache);

	// find a function that calculates the center of mass for a rigidbodytree

	Vector2d l_foot = tree_.transformPoints(cache, 0, left_foot_idx, 0); 
	Vector2d r_foot = tree_.transformPoints(cache, 0, right_foot_idx, 0); 
	Vector2d center_of_mass = tree_.transformPoints(cache, 0, torso_idx, 0);

	// l_foot 
	// TODO: calculate the feet pos as a function of COM

	return PiecewisePolynomial<double>();
}

Trajectory<double> generateLandingTraj(const drake::systems::Context<double>& context,
										VectorXd& q, VectorXd& v){

	// TODO: calculate center of mass traj to stabilize the robot

	return PiecewisePolynomial<double>();
}




void JumpingTraj::CalcDesiredTraj(const drake::systems::Context<double>& context,
									 drake::trajectories::Trajectory<double>* traj) const {


	// Read in current state
	const OutputVector<double>* robot_output = (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
	VectorXd q = robot_output->GetPositions();
	VectorXd v = robot_output->GetVelocities();

	// Read in finite state machine
	const BasicVector<double>* fsm_output = (BasicVector<double>*)this->EvalVectorInput(context, fsm_port_);
	VectorXd fsm_state = fsm_output->get_value();

	const auto prev_td_time = context.get_discrete_state(prev_td_time_idx_).get_value();
	double timestamp = robot_output->get_timestamp();
	double current_time = static_cast<double>(timestamp);

	ExponentialPlusPiecewisePolynomial<double>* casted_traj = (ExponentialPlusPiecewisePolynomial<double>*)
											dynamic_cast<ExponentialPlusPiecewisePolynomial<double>*> (traj);
	

	// casted_traj = 

	switch(fsm_state){
		case(0): //NEUTRAL
			casted_traj = generateNeutralTraj(context, q, v);
			break;
		case(1): //CROUCH
			casted_traj = generateCrouchTraj(context, q, v);
			break;
		case(2): //FLIGHT
			casted_traj = generateFlightTraj(context, q, v);
			break;
		case(3): //LAND
			casted_traj = generateLandingTraj(context, q, v);
			break;
	}












}

}  // namespace osc
}  // namespace cassie
}  // namespace dairlib


