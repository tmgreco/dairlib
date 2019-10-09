#include "examples/jumping/com_traj.h"
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
using dairlib::systems::OutputVector;
using drake::trajectories::Trajectory;
using drake::trajectories::PiecewisePolynomial;
using drake::trajectories::ExponentialPlusPiecewisePolynomial;



namespace dairlib {
namespace examples {
namespace jumping {
namespace osc {


CoMTraj::CoMTraj(const RigidBodyTree<double>& tree,
				int hip_idx,
				int left_foot_idx,
				int right_foot_idx,
				PiecewisePolynomial<double> crouch_traj,
				double height):
				tree_(tree),
				hip_idx_(hip_idx),
				left_foot_idx_(left_foot_idx),
				right_foot_idx_(right_foot_idx),
				crouch_traj_(crouch_traj),
				height_(height){
	
	this->set_name("com_traj");
	  // Input/Output Setup
	state_port_ = this->DeclareVectorInputPort(OutputVector<double>(
                  tree.get_num_positions(),
                  tree.get_num_velocities(),
                  tree.get_num_actuators())).get_index();
	fsm_port_ = this->DeclareVectorInputPort(
                BasicVector<double>(1)).get_index();
	
	PiecewisePolynomial<double> empty_pp_traj(VectorXd(0));
	Trajectory<double>& traj_inst = empty_pp_traj;
	this->DeclareAbstractOutputPort("com_traj", traj_inst,
		&CoMTraj::CalcTraj);
	time_idx_ = this->DeclareDiscreteState(1);
	fsm_idx_ = this->DeclareDiscreteState(1);

	DeclarePerStepDiscreteUpdateEvent(&CoMTraj::DiscreteVariableUpdate);

}

EventStatus CoMTraj::DiscreteVariableUpdate(
    const Context<double>& context,
    DiscreteValues<double>* discrete_state) const {

	auto prev_fsm_state = discrete_state->get_mutable_vector(fsm_idx_).get_mutable_value();
	auto prev_time = discrete_state->get_mutable_vector(time_idx_).get_mutable_value();

	const BasicVector<double>* fsm_output = (BasicVector<double>*)this->EvalVectorInput(context, fsm_port_);
	VectorXd fsm_state = fsm_output->get_value();

	const OutputVector<double>* robot_output = (OutputVector<double>*)
		this->EvalVectorInput(context, state_port_);
	double timestamp = robot_output->get_timestamp();
	double current_time = static_cast<double>(timestamp);

	if(prev_fsm_state(0) != fsm_state(0)){ //When to reset the clock
		prev_fsm_state(0) = fsm_state(0);
		prev_time(0) = current_time;
	}
	return EventStatus::Succeeded();
}

PiecewisePolynomial<double> CoMTraj::generateNeutralTraj(const drake::systems::Context<double>& context,
										VectorXd& q, VectorXd& v) const{

	// Kinematics cache and indices
	KinematicsCache<double> cache = tree_.CreateKinematicsCache();
	// Modify the quaternion in the begining when the state is not received from
	// the robot yet
	// Always remember to check 0-norm quaternion when using doKinematics
	dairlib::multibody::SetZeroQuaternionToIdentity(&q);
	cache.initialize(q);
	tree_.doKinematics(cache);

	Vector3d pt_on_foot = Eigen::VectorXd::Zero(3);

	Vector3d l_foot = tree_.transformPoints(cache, pt_on_foot, left_foot_idx_, 0); 
	Vector3d r_foot = tree_.transformPoints(cache, pt_on_foot, right_foot_idx_, 0); 

	Vector3d feet_center = (l_foot + r_foot) / 2;

	Vector3d desired_com(feet_center(0), feet_center(1), feet_center(2) + height_);
	return PiecewisePolynomial<double>(desired_com);
}

PiecewisePolynomial<double> CoMTraj::generateCrouchTraj(const drake::systems::Context<double>& context,
										VectorXd& q, VectorXd& v) const{
	// Kinematics cache and indices
	// KinematicsCache<double> cache = tree_.CreateKinematicsCache();
	const OutputVector<double>* robot_output = (OutputVector<double>*)
		this->EvalVectorInput(context, state_port_);
	double timestamp = robot_output->get_timestamp();
	double current_time = static_cast<double>(timestamp);

	double prev_time = static_cast<double>(context.get_discrete_state().get_vector(time_idx_).get_value()(0));

	int t = (int)((current_time - prev_time)/(1.22/100.0));
	// Vector3d desired_com(crouch_traj_(t, 1), 0, crouch_traj_(t, 3));

	// return PiecewisePolynomial<double>(desired_com);
	return crouch_traj_;
}

/*
	Move the feet relative to the COM
	The trajectory of the COM cannot be altered, so must solve for 
	foot positions as a function of COM. 

*/
PiecewisePolynomial<double> CoMTraj::generateFlightTraj(const drake::systems::Context<double>& context,
										VectorXd& q, VectorXd& v) const{
	// Do nothing
	return PiecewisePolynomial<double>(VectorXd(0));
}

PiecewisePolynomial<double> CoMTraj::generateLandingTraj(const drake::systems::Context<double>& context,
										VectorXd& q, VectorXd& v) const{

	// Kinematics cache and indices
	KinematicsCache<double> cache = tree_.CreateKinematicsCache();
	// Modify the quaternion in the begining when the state is not received from
	// the robot yet
	// Always remember to check 0-norm quaternion when using doKinematics
	dairlib::multibody::SetZeroQuaternionToIdentity(&q);
	cache.initialize(q);
	tree_.doKinematics(cache);

	Vector3d pt_on_foot = Eigen::VectorXd::Zero(3);

	Vector3d l_foot = tree_.transformPoints(cache, pt_on_foot, left_foot_idx_, 0); 
	Vector3d r_foot = tree_.transformPoints(cache, pt_on_foot, right_foot_idx_, 0); 
	// Vector3d center_of_mass = tree_.centerOfMass(cache);

	Vector3d feet_center = (l_foot + r_foot) / 2;

	// desired pos is in between the two feet and at the current COM height
	Vector3d desired_com(feet_center(0), feet_center(1), feet_center(2) + height_);
	return PiecewisePolynomial<double>(desired_com);
}




void CoMTraj::CalcTraj(const drake::systems::Context<double>& context,
									 drake::trajectories::Trajectory<double>* traj) const {

	// Read in current state
	const OutputVector<double>* robot_output = (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
	VectorXd q = robot_output->GetPositions();
	VectorXd v = robot_output->GetVelocities();

	// Read in finite state machine
	const BasicVector<double>* fsm_output = (BasicVector<double>*)this->EvalVectorInput(context, fsm_port_);
	VectorXd fsm_state = fsm_output->get_value();

	PiecewisePolynomial<double>* casted_traj = (PiecewisePolynomial<double>*)
											dynamic_cast<PiecewisePolynomial<double>*> (traj);
	
	switch((int)fsm_state(0)){
		case(0): //NEUTRAL
			*casted_traj = generateNeutralTraj(context, q, v);
			// std::cout << "Generated com for netural traj: " << casted_traj->getPolynomialMatrix(0) << std::endl;
			break;
		case(1): //CROUCH
			*casted_traj = generateCrouchTraj(context, q, v);
			break;
		case(2): //FLIGHT
			// *casted_traj = generateFlightTraj(context, q, v);
			// does nothing in flight
			break;
		case(3): //LAND
			*casted_traj = generateLandingTraj(context, q, v);
			break;
	}


}

}  // namespace osc
}  // namespace jumping
}  // namespace examples
}  // namespace dairlib


