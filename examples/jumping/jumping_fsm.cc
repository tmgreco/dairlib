#pragma once

#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "systems/framework/output_vector.h"
#include "systems/controllers/control_utils.h"

namespace dairlib {
namespace jumping {
namespace osc {

JumpingFiniteStateMachine::JumpingFiniteStateMachine(const RigidBodyTree<doubule> & tree,
													 double wait_time)
													:wait_time_(wait_time)
														{
	curr_state_ = init_state_;
	initial_timestamp_ = 0.0;
	state_port_ = this->DeclareVectorInputPort(OutputVector<double>(
													tree.get_num_positions(),
                                   					tree.get_num_velocities(),
                                       				tree.get_num_actuators()
                                       				)).get_index();
	this->DeclareVectorOutputPort(BasicVector<double>(1),
                                	&JumpingFiniteStateMachine::CalcFiniteState);
}


void JumpingFiniteStateMachine::CalcFiniteState(const Context<double>& context,
												BasicVector<double>* fsm_state) const{

	const OutputVector<double>* robot_output = (OutputVector<double>*)this->EvalVectorInput(context, state_port_);

	bool trigger = false;
	std::vector<double> ground_reaction_forces = context.getReactionForces();
	double timestamp = robot_output->get_timestamp();
	double current_sim_time = static_cast<double>(timestamp);

	switch(curr_state_){
		case(NEUTRAL):
			if(current_sim_time > initial_timestamp_ + wait_time_){
				curr_state_ = CROUCH;
				break;
			}
		case(CROUCH):
			if(trigger){
				curr_state_ = FLIGHT;
				break;
			}
		case(FLIGHT):
			if(trigger){
				curr_state_ = LAND;
				break;
			}
		case(LAND):
			if(trigger){
				curr_state_ = NEUTRAL;
				break;
			}
		default:
			std::cerr << "Invalid state, defaulting to NEUTRAL." << std::endl;
			curr_state_ = NEUTRAL;
	}

	VectorXd current_state(1);
	current_state << curr_state_;
	fsm_state->get_mutable_value() = curr_state_;
}

}  // namespace osc
}  // namespace cassie
}  // namespace dairlib


