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
#include "examples/jumping/com_traj.h"
#include "examples/jumping/flight_foot_traj.h"
#include "examples/jumping/jumping_fsm.h"
#include "systems/controllers/osc/operational_space_control.h"
// #include "systems/controllers/osc/osc_tracking_data.h"



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
DEFINE_double(height, 0.7, "Standing height of the five link biped");

DEFINE_double(torso_orientation_cost, 0.1,"Weight to scale the torso orientation cost");

// using drake::multibody::MultibodyPlant;
using drake::multibody::Body;
// using drake::multibody::Parser;

using drake::systems::lcm::LcmSubscriberSystem;
using drake::systems::lcm::LcmPublisherSystem;
using drake::geometry::SceneGraph;
using drake::systems::DiagramBuilder;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
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
using systems::SubvectorPassThrough;

namespace examples{
namespace jumping{
namespace osc{

const std::string channel_x = FLAGS_state_simulation_channel;
const std::string channel_u = "RABBIT_INPUT";

int doMain(int argc, char* argv[]){
	gflags::ParseCommandLineFlags(&argc, &argv, true);
	DiagramBuilder<double> builder;
	
	std::string filename = "examples/jumping/five_link_biped.urdf";
	// Initialize the plant
	RigidBodyTree<double> tree_with_springs;
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
	// int netural_height = FLAGS_height;
	PiecewisePolynomial<double> jumping_traj_from_optimization = 
		loadStateTrajToPP("saved_trajs/");
	MatrixXd com_traj_from_optimization = readCSV("com_pos_matrix.csv");


	// Create Operational space control
		// Create state receiver.
	// Create command sender.
	auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>();
	// Create state receiver.
	auto state_sub = builder.AddSystem(LcmSubscriberSystem::Make<dairlib::lcmt_robot_output>(
						channel_x, lcm));
	auto state_receiver = builder.AddSystem<systems::RobotOutputReceiver>(tree_with_springs);
	auto traj_generator = builder.AddSystem<CoMTraj>(tree_with_springs, 
														hip_index, l_foot_index, r_foot_index,
														com_traj_from_optimization,
														FLAGS_height);
	auto l_foot_traj_generator = builder.AddSystem<FlightFootTraj>(tree_with_springs, 
														hip_index, l_foot_index, r_foot_index,
														true, FLAGS_height);
	auto r_foot_traj_generator = builder.AddSystem<FlightFootTraj>(tree_with_springs, 
														hip_index, l_foot_index, r_foot_index,
														false, FLAGS_height);
	auto fsm = builder.AddSystem<dairlib::examples::JumpingFiniteStateMachine>(tree_with_springs, FLAGS_wait_time);
	auto command_pub = builder.AddSystem(LcmPublisherSystem::Make<dairlib::lcmt_robot_input>(
										channel_u, lcm, 1.0 / FLAGS_publish_rate));
	auto command_sender = builder.AddSystem<systems::RobotCommandSender>(tree_with_springs);
	auto osc = builder.AddSystem<systems::controllers::OperationalSpaceControl>(
	           tree_with_springs, tree_with_springs, true, false);

	// JumpingTraj output port indices
	// int com_output_port = 0;
	// int lfoot_output_port = 1;
	// int rfoot_output_port = 2;
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
	osc->AddStateAndContactPoint(LAND, "left_foot", foot_contact_disp);
	osc->AddStateAndContactPoint(LAND, "right_foot", foot_contact_disp);

	// ***** COM tracking term ******
	// Gains for COM tracking
	MatrixXd W_com = MatrixXd::Identity(3, 3);
	W_com(0, 0) = 2000;
	W_com(1, 1) = 1;
	W_com(2, 2) = 2000;

  	double xy_scale = 10;
	double g_over_l = 9.81/FLAGS_height;
	MatrixXd K_p_com = (xy_scale*sqrt(g_over_l)  - g_over_l) * MatrixXd::Identity(3, 3);
	MatrixXd K_d_com = xy_scale * MatrixXd::Identity(3, 3);
	K_p_com(2, 2) = 50;
	K_d_com(2, 2) = 14;

	ComTrackingData com_tracking_data("com_traj", 3,
		K_p_com, K_d_com, W_com,
		&tree_with_springs, &tree_with_springs);
	com_tracking_data.AddStateToTrack(NEUTRAL);
	com_tracking_data.AddStateToTrack(CROUCH);
	com_tracking_data.AddStateToTrack(LAND);
	osc->AddTrackingData(&com_tracking_data);

	// ***** Torso balance term ******
	double w_pelvis_balance = 200;
	double w_heading = 200;
	double k_p_pelvis_balance = 10;
	double k_d_pelvis_balance = 10;
	double k_p_heading = 10;
	double k_d_heading = 10;
	Matrix3d W_pelvis = MatrixXd::Identity(3, 3);
	W_pelvis(0, 0) = w_pelvis_balance;
	W_pelvis(1, 1) = w_pelvis_balance;
	W_pelvis(2, 2) = w_heading;
	Matrix3d K_p_pelvis = MatrixXd::Identity(3, 3);
	K_p_pelvis(0, 0) = k_p_pelvis_balance * 2;
	K_p_pelvis(1, 1) = k_p_pelvis_balance * 2;
	K_p_pelvis(2, 2) = k_p_heading;
	Matrix3d K_d_pelvis = MatrixXd::Identity(3, 3);
	K_d_pelvis(0, 0) = k_d_pelvis_balance;
	K_d_pelvis(1, 1) = k_d_pelvis_balance;
	K_d_pelvis(2, 2) = k_d_heading;
	RotTaskSpaceTrackingData pelvis_rot_traj("pelvis_rot_traj", 3,
		K_p_pelvis, K_d_pelvis, W_pelvis * FLAGS_torso_orientation_cost,
		&tree_with_springs, &tree_with_springs);
	pelvis_rot_traj.AddFrameToTrack("torso");
	VectorXd pelvis_desired_quat(4);
	pelvis_desired_quat << 1, 0, 0, 0;
	osc->AddConstTrackingData(&pelvis_rot_traj, pelvis_desired_quat);

	// ****** Feet tracking term ******	
	MatrixXd W_swing_foot = 200 * MatrixXd::Identity(3, 3);
	MatrixXd K_p_sw_ft = 100 * MatrixXd::Identity(3, 3);
	MatrixXd K_d_sw_ft = 10 * MatrixXd::Identity(3, 3);
	TransTaskSpaceTrackingData flight_phase_left_foot_traj("l_foot_traj", 3,
		K_p_sw_ft, K_d_sw_ft, W_swing_foot,
		&tree_with_springs, &tree_with_springs);
	TransTaskSpaceTrackingData flight_phase_right_foot_traj("r_foot_traj", 3,
		K_p_sw_ft, K_d_sw_ft, W_swing_foot,
		&tree_with_springs, &tree_with_springs);
	flight_phase_left_foot_traj.AddStateAndPointToTrack(FLIGHT, "left_foot");
	flight_phase_right_foot_traj.AddStateAndPointToTrack(FLIGHT, "right_foot");
	osc->AddTrackingData(&flight_phase_left_foot_traj);
	osc->AddTrackingData(&flight_phase_right_foot_traj);

	osc->Build();


	// ******End of osc configuration*******

	builder.Connect(state_sub->get_output_port(),
				state_receiver->get_input_port(0));
	builder.Connect(state_receiver->get_output_port(0), 
				traj_generator->get_state_input_port());
	builder.Connect(state_receiver->get_output_port(0), 
				l_foot_traj_generator->get_state_input_port());
	builder.Connect(state_receiver->get_output_port(0), 
				r_foot_traj_generator->get_state_input_port());
	builder.Connect(state_receiver->get_output_port(0),
          		osc->get_robot_output_input_port());
	builder.Connect(state_receiver->get_output_port(0),
				fsm->get_state_input_port());
	builder.Connect(fsm->get_output_port(0),
				traj_generator->get_fsm_input_port());
	builder.Connect(fsm->get_output_port(0),
				l_foot_traj_generator->get_fsm_input_port());
	builder.Connect(fsm->get_output_port(0),
				r_foot_traj_generator->get_fsm_input_port());
	builder.Connect(traj_generator->get_output_port(0),
          		osc->get_tracking_data_input_port("com_traj"));
	
	builder.Connect(l_foot_traj_generator->get_output_port(0),
          		osc->get_tracking_data_input_port("l_foot_traj"));	
	builder.Connect(r_foot_traj_generator->get_output_port(0),
          		osc->get_tracking_data_input_port("r_foot_traj"));
	builder.Connect(fsm->get_output_port(0),
				osc->get_fsm_input_port());
	builder.Connect(osc->get_output_port(0),
          		command_sender->get_input_port(0));
	builder.Connect(command_sender->get_output_port(0), 
				command_pub->get_input_port());
	

	// Create the diagram and context
	auto diagram = builder.Build();
	auto context = diagram->CreateDefaultContext();

	std::cout << "Built diagram" << std::endl;
	/// Use the simulator to drive at a fixed rate
	/// If set_publish_every_time_step is true, this publishes twice
	/// Set realtime rate. Otherwise, runs as fast as possible
	auto stepper = std::make_unique<drake::systems::Simulator<double>>(*diagram,
	             std::move(context));
	stepper->set_publish_every_time_step(false);
	stepper->set_publish_at_initialization(false);
	stepper->set_target_realtime_rate(1.0);
	stepper->Initialize();
	std::cout << "Running simulation" << std::endl;

	drake::log()->info("controller started");
	stepper->AdvanceTo(std::numeric_limits<double>::infinity());

	return 0;

}


}  // namespace osc
}  // namespace jumping
}  // namespace examples
}  // namespace dairlib

int main(int argc, char* argv[]) { 
	return dairlib::examples::jumping::osc::doMain(argc, argv); 
}
