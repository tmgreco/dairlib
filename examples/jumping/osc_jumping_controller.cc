#include <chrono>
#include <thread>

#include <gflags/gflags.h>

#include "drake/lcm/drake_lcm.h"

#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/analysis/simulator.h"

#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/systems/rendering/multibody_position_to_geometry_pose.h"
#include "drake/geometry/geometry_visualization.h"


#include "dairlib/lcmt_robot_output.hpp"
#include "dairlib/lcmt_robot_input.hpp"
#include "dairlib/lcmt_pd_config.hpp"
#include "systems/robot_lcm_systems.h"
#include "attic/multibody/rigidbody_utils.h"
#include "examples/jumping/jumping_traj.h"
#include "examples/jumping/jumping_fsm.h"
#include "systems/controllers/osc/operational_space_control.h"
#include "systems/controllers/osc/osc_tracking_data.h"



#include "common/find_resource.h"
#include "multibody/multibody_utils.h"
#include "multibody/visualization_utils.h"
#include "systems/primitives/subvector_pass_through.h"
#include "drake/multibody/parsers/urdf_parser.h"


#include "examples/jumping/traj_logger.h"


DEFINE_double(realtime_factor, .5,
			  "Playback speed.  See documentation for "
			  "Simulator::set_target_realtime_rate() for details.");
DEFINE_double(gravity, 9.81,
				"Gravity acceleration constant");
DEFINE_double(mu, 0.7, "The static coefficient of friction");
// DEFINE_double(mu_kinetic, 0.7, "The dynamic coefficient of friction");
DEFINE_double(v_tol, 0.01, "The maximum slipping speed allowed during stiction (m/s)");
DEFINE_string(state_simulation_channel, "RABBIT_STATE_SIMULATION", 
              "Channel to publish/receive state from simulation");
DEFINE_double(wait_time, 5.0, "The length of time to wait in the neutral state before jumping (s)");
DEFINE_double(publish_rate, 200, "Publishing frequency (Hz)");
DEFINE_double(height, 0.8, "Standing height of the five link biped");

// using drake::multibody::MultibodyPlant;
using drake::multibody::Body;
// using drake::multibody::Parser;

using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::lcm::LcmPublisherSystem;
using drake::geometry::SceneGraph;
using drake::systems::DiagramBuilder;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix3Xd;
using std::vector;
using drake::trajectories::PiecewisePolynomial;
using std::cout;
using std::endl;


namespace dairlib{

using multibody::GetBodyIndexFromName;
using systems::controllers::ComTrackingData;
using systems::controllers::TransTaskSpaceTrackingData;
using systems::controllers::RotTaskSpaceTrackingData;
using systems::controllers::JointSpaceTrackingData;
using examples::jumping::osc::JumpingFiniteStateMachine;
using examples::jumping::osc::JumpingTraj;
using systems::SubvectorPassThrough;

namespace examples{
namespace jumping{
namespace osc{

const std::string channel_x = FLAGS_state_simulation_channel;
const std::string channel_u = "RABBIT_INPUT";

int doMain(int argc, char* argv[]){
	gflags::ParseCommandLineFlags(&argc, &argv, true);
	DiagramBuilder<double> builder;
	
	std::string filename = FindResourceOrThrow("examples/jumping/five_link_biped.urdf");
	// Initialize the plant
	RigidBodyTree<double> tree_with_springs;
	RigidBodyTree<double> tree_without_springs;
	drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
		FindResourceOrThrow(filename), drake::multibody::joints::kQuaternion, &tree_with_springs);
	const double terrain_size = 100;
	const double terrain_depth = 0.20;
	drake::multibody::AddFlatTerrainToWorld(&tree_with_springs,
	                                      terrain_size, terrain_depth);

	// MultibodyPlant<double> plant;
	// SceneGraph<double>& scene_graph = *(builder.AddSystem<SceneGraph>());
	// Parser parser(&plant, &scene_graph);
	// parser.AddModelFromFile(full_name);
	// plant.mutable_gravity_field().set_gravity_vector(-FLAGS_gravity * Eigen::Vector3d::UnitZ());
	// plant.WeldFrames(
	// 		plant.world_frame(), 
	// 		plant.GetFrameByName("base"),
	// 		drake::math::RigidTransform<double>()
	// 		);
	// plant.Finalize();

	int hip_index = GetBodyIndexFromName(tree_with_springs, "torso");
	int l_foot_index = GetBodyIndexFromName(tree_with_springs, "left_foot");
	int r_foot_index = GetBodyIndexFromName(tree_with_springs, "right_foot");

	// Create Operational space control
		// Create state receiver.
	// Create command sender.
	auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>();

	auto state_receiver = builder.AddSystem<systems::RobotOutputReceiver>(tree_with_springs);
	auto command_pub = builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
										channel_u, lcm, 1.0 / FLAGS_publish_rate));
	auto command_sender = builder.AddSystem<systems::RobotCommandSender>(tree_with_springs);
	auto fsm = builder.AddSystem<JumpingFiniteStateMachine>(tree_with_springs, FLAGS_wait_time);
	auto traj_generator = builder.AddSystem<JumpingTraj>(
										tree_with_springs, hip_index, l_foot_index, r_foot_index);
	auto osc = builder.AddSystem<systems::controllers::OperationalSpaceControl>(
	           tree_with_springs, tree_with_springs, false, false);

	// ******Begin osc configuration*******

	// Acceleration Cost
	int n_v = tree_with_springs.get_num_velocities();
	MatrixXd Q_accel = 10 * MatrixXd::Identity(n_v, n_v);
	osc->SetAccelerationCostForAllJoints(Q_accel);

	// Contact Constraint Slack Variables
	double lambda_contact_relax = 20000;
	osc->SetWeightOfSoftContactConstraint(lambda_contact_relax);


	// All foot contact specification for osc
	double mu = FLAGS_mu;
	osc->SetContactFriction(mu);
	Vector3d foot_contact_disp(0, 0, 0);
	osc->AddStateAndContactPoint(NEUTRAL, "left_foot", foot_contact_disp);
	osc->AddStateAndContactPoint(NEUTRAL, "right_foot", foot_contact_disp);
	osc->AddStateAndContactPoint(CROUCH, "left_foot", foot_contact_disp);
	osc->AddStateAndContactPoint(CROUCH, "right_foot", foot_contact_disp);

	// Gains for COM tracking
	MatrixXd W_com = MatrixXd::Identity(3, 3);
	W_com(0, 0) = 2000;
	W_com(1, 1) = 2000;
	W_com(2, 2) = 200;

  	double xy_scale = 10;
	double g_over_l = 9.81/FLAGS_height;
	MatrixXd K_p_com = (xy_scale*sqrt(g_over_l)  - g_over_l) * MatrixXd::Identity(3, 3);
	MatrixXd K_d_com = xy_scale * MatrixXd::Identity(3, 3);

	K_p_com(2, 2) = 10;
	K_d_com(2, 2) = 10;

	ComTrackingData center_of_mass_traj("com_traj", 3,
										K_p_com, K_d_com, W_com * FLAGS_cost_weight_multiplier,
										&tree_with_springs, &tree_with_springs);
	osc->A(&center_of_mass_traj);

	osc->Build();


	// ******End of osc configuration*******



	builder.Connect(state_receiver->get_output_port(0), 
				traj_generator->get_input_port_state());
	builder.Connect(command_sender->get_output_port(0), 
				command_pub->get_input_port());
	builder.Connect(state_receiver->get_output_port(0),
	          	osc->get_robot_output_input_port());
	builder.Connect(osc->get_output_port(0),
	          	command_sender->get_input_port(0));
	builder.Connect(traj_generator->get_output_port(0),
	          	osc->get_tracking_data_input_port("com_traj"));
	builder.Connect(state_receiver->get_output_port(0),
					fsm->get_input_port_state());
	builder.Connect(fsm->get_output_port(0),
					traj_generator->get_input_port_fsm());

	

	// Create the diagram and context
	auto diagram = builder.Build();
	auto context = diagram->CreateDefaultContext();

	/// Use the simulator to drive at a fixed rate
	/// If set_publish_every_time_step is true, this publishes twice
	/// Set realtime rate. Otherwise, runs as fast as possible
	auto stepper = std::make_unique<drake::systems::Simulator<double>>(*diagram,
	             std::move(context));
	stepper->set_publish_every_time_step(false);
	stepper->set_publish_at_initialization(false);
	stepper->set_target_realtime_rate(1.0);
	stepper->Initialize();

	drake::log()->info("controller started");
	stepper->AdvanceTo(std::numeric_limits<double>::infinity());

	return 0;

}


}  // namespace osc
}  // namespace jumping
}  // namespace examples
}  // namespace dairlib

int main(int argc, char* argv[]) { 
	return dairlib::examples::jumping::doMain(argc, argv); 
}
