#include <chrono>
#include <thread>

#include <gflags/gflags.h>

#include "drake/lcm/drake_lcm.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/solve.h"
#include "drake/solvers/solution_result.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/systems/rendering/multibody_position_to_geometry_pose.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"

#include "common/find_resource.h"
#include "multibody/multibody_utils.h"
#include "multibody/visualization_utils.h"
#include "systems/primitives/subvector_pass_through.h"
#include "solvers/optimization_utils.h"
#include "systems/trajectory_optimization/dircon_position_data.h"
#include "systems/trajectory_optimization/dircon_kinematic_data_set.h"
#include "systems/trajectory_optimization/hybrid_dircon.h"
#include "systems/trajectory_optimization/dircon_opt_constraints.h"

#include "examples/jumping/traj_logger.h"


DEFINE_double(realtime_factor, .5,
			  "Playback speed.  See documentation for "
			  "Simulator::set_target_realtime_rate() for details.");
DEFINE_double(gravity, 9.81,
				"Gravity acceleration constant");
DEFINE_double(mu, 0.7, "The static coefficient of friction");
// DEFINE_double(mu_kinetic, 0.7, "The dynamic coefficient of friction");
DEFINE_double(v_tol, 0.01, "The maximum slipping speed allowed during stiction (m/s)");


using drake::multibody::MultibodyPlant;
using drake::multibody::Body;
using drake::multibody::Parser;
using drake::geometry::SceneGraph;
using drake::systems::DiagramBuilder;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix3Xd;
using std::vector;
using drake::trajectories::PiecewisePolynomial;
using drake::solvers::SolutionResult;
using std::cout;
using std::endl;


namespace dairlib{

using systems::trajectory_optimization::HybridDircon;
using systems::trajectory_optimization::DirconDynamicConstraint;
using systems::trajectory_optimization::DirconKinematicConstraint;
using systems::trajectory_optimization::DirconOptions;
using systems::trajectory_optimization::DirconKinConstraintType;
using systems::SubvectorPassThrough;

namespace examples{
namespace jumping{

const std::string channel_x = FLAGS_channel;
const std::string channel_u = "CASSIE_INPUT";

int doMain(int argc, char* argv[]){
	gflags::ParseCommandLineFlags(&argc, &argv, true);
	DiagramBuilder<double> builder;
	
	// Initialize the plant
	MultibodyPlant<double> plant;
	SceneGraph<double>& scene_graph = *(builder.AddSystem<SceneGraph>());
	Parser parser(&plant, &scene_graph);
	std::string full_name = FindResourceOrThrow("examples/jumping/five_link_biped.urdf");
	parser.AddModelFromFile(full_name);
	plant.mutable_gravity_field().set_gravity_vector(-FLAGS_gravity * Eigen::Vector3d::UnitZ());
	plant.WeldFrames(
			plant.world_frame(), 
			plant.GetFrameByName("base"),
			drake::math::RigidTransform<double>()
			);
	plant.Finalize();

	auto osc = builder.AddSystem<systems::controllers::OperationalSpaceControl>(plant, plant, false, false);

	int hip_index = GetBodyIndexFromName(plant, "torso");
	int l_foot_index = GetBodyIndexFromName(plant, "l_foot");
	int r_foot_index = GetBodyIndexFromName(plant, "r_foot");

	int n_v = plant.get_num_velocities();
	MatrixXd Q_accel = 10 * MatrixXd::Identity(n_v, n_v);
	osc->SetAccelerationCostForAllJoints(Q_accel);

	double lambda_contact_relax = 20000;
	osc->SetWeightOfSoftContactConstraint(lambda_contact_relax);

	double mu = FLAGS_mu;
	osc->setContactFriction(mu);

	osc->addContactPoint("l_foot");
	osc->addContactPoint("r_foot");

	MatrixXd W_com = MatrixXd::Identity(3, 3);
	W_com(0, 0) = 2000;
	W_com(1, 1) = 2000;
	W_com(2, 2) = 200;

  	double xy_scale = 10;
	double g_over_l = 9.81/FLAGS_height;
	MatrixXd K_p_com = (xy_scale*sqrt(g_over_l)  - g_over_l) *
	  MatrixXd::Identity(3, 3);
	MatrixXd K_d_com = xy_scale * MatrixXd::Identity(3, 3);

	K_p_com(2, 2) = 10;
	K_d_com(2, 2) = 10;

	ComTrackingData center_of_mass_traj("com_traj", 3,
										K_p_com, K_d_com, W_com * FLAGS_cost_weight_multiplier,
										&tree_with_springs, &tree_without_springs);
	osc->AddTrackingData(&center_of_mass_traj);

	osc->Build();

	  // Create state receiver.
	auto state_receiver = builder.AddSystem<systems::RobotOutputReceiver>(tree_with_springs);
	// Create command sender.
	auto command_pub = builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(channel_u, lcm, {TriggerType::kForced}));
	auto command_sender = builder.AddSystem<systems::RobotCommandSender>(tree_with_springs);


	auto traj_generator = builder.AddSystem<jumping::osc::JumpingTraj>(plant, hip_index, l_foot_index, r_foot_index);
	build.Connect(state_receiver->get_output_port(0), traj_generator->get_input_port_state());
	builder.Connect(command_sender->get_output_port(0), command_pub->get_input_port());
	builder.Connect(state_receiver->get_output_port(0),
	          	osc->get_robot_output_input_port());
	builder.Connect(osc->get_output_port(0),
	          	command_sender->get_input_port(0));
	builder.Connect(traj_generator->get_output_port(0),
	          	osc->get_tracking_data_input_port("com_traj"));

	auto diagram = builder.Build();
	auto contect = diagram->CreateDefaultContext();

	const auto& diagram = *diagram;
	drake::systems::Simulator<double> simulator(std::move(owned_diagram),
  												std::move(context));
	auto& diagram_context = simulator.get_mutable_context();

	auto& state_receiver_context =
	  diagram.GetMutableSubsystemContext(*state_receiver, &diagram_context);

	// Wait for the first message.
	drake::log()->info("Waiting for first state message on " + channel_x);
	drake::lcm::Subscriber<dairlib::lcmt_robot_output> state_sub(lcm,
	  															channel_x);
	LcmHandleSubscriptionsUntil(lcm, [&]() {
		return state_sub.count() > 0;
	});

	// Initialize the context based on the first message.
	const double t0 = state_sub.message().utime * 1e-6;
	diagram_context.SetTime(t0);
	auto& input_value = state_receiver->get_input_port(0).FixValue(
	                    &state_receiver_context, state_sub.message());

	drake::log()->info("controller started");
	while (true) {
		// Wait for an lcmt_robot_output message.
		state_sub.clear();
		LcmHandleSubscriptionsUntil(lcm, [&]() {
			return state_sub.count() > 0;
		});
		// Write the lcmt_robot_output message into the context and advance.
		input_value.GetMutableData()->set_value(state_sub.message());
		const double time = state_sub.message().utime * 1e-6;

		// Check if we are very far ahead or behind
		// (likely due to a restart of the driving clock)
		if (time > simulator.get_context().get_time() + 1.0 || time < simulator.get_context().get_time() - 1.0) {
			
			std::cout << "Controller time is " << simulator.get_context().get_time()
			  << ", but stepping to " << time << std::endl;
			std::cout << "Difference is too large, resetting controller time." <<
			  std::endl;
			simulator.get_mutable_context().SetTime(time);
		}

		simulator.AdvanceTo(time);
		// Force-publish via the diagram
		diagram.Publish(diagram_context);
	}

	return 0;

}


}  // namespace jumping
}  // namespace examples
}  // namespace dairlib

int main(int argc, char* argv[]) { 
	return dairlib::examples::jumping::doMain(argc, argv); 
}
