#pragma once

#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "systems/framework/output_vector.h"
#include "systems/controllers/control_utils.h"

namespace dairlib {
namespace examples{
namespace jumping {
namespace osc {

enum FSM_State {NEUTRAL, CROUCH, FLIGHT, LAND};

class JumpingFiniteStateMachine : public drake::systems::LeafSystem<double> {
public:
	JumpingFiniteStateMachine(	const RigidBodyTree<double>& tree,
								const double wait_time);

	const drake::systems::InputPort<double>& get_input_port_state() const {
		return this->get_input_port(state_port_);
	}

private:

	void CalcFiniteState(	const drake::systems::Context<double>& context,
							drake::systems::BasicVector<double>* fsm_state) const;

	int state_port_;
	double timestamp_;

	const FSM_State init_state_ = NEUTRAL;
	FSM_State curr_state_;
};

}  // namespace osc
}  // namespace jumping
}  // namespace examples
}  // namespace dairlib


