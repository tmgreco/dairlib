#include <chrono>

#include <gflags/gflags.h>

#include "drake/lcm/drake_lcm.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/solve.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/systems/rendering/multibody_position_to_geometry_pose.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/solvers/solve.h"
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


DEFINE_double(realtime_factor, .5,
			  "Playback speed.  See documentation for "
			  "Simulator::set_target_realtime_rate() for details.");
DEFINE_double(gravity, 9.81,
				"Gravity acceleration constant");
DEFINE_double(mu_static, 0.7, "The static coefficient of friction");
DEFINE_double(mu_kinetic, 0.7, "The dynamic coefficient of friction");
DEFINE_double(v_tol, 0.01, "The maximum slipping speed allowed during stiction (m/s)");

DEFINE_double(max_duration, 20, "Maximum trajectory duration (s)");

// Simulation parameters.
DEFINE_double(timestep, 1e-5, "The simulator time step (s)");

using drake::multibody::MultibodyPlant;
using drake::multibody::Body;
using drake::multibody::Parser;
using drake::geometry::SceneGraph;
using drake::systems::DiagramBuilder;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix3Xd;
using drake::trajectories::PiecewisePolynomial;
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

// Simple example which simulates the (passive) Kneed walker.  Run drake-visualizer
// to see the animated result.


// MultibodyPlant<double> build_model(drake::systems::DiagramBuilder<double>& builder) {
// 	MultibodyPlant<double> plant;
// 	SceneGraph<double>& scene_graph = *(builder.AddSystem<SceneGraph>());
// 	Parser parser(&plant, &scene_graph);
// 	std::string full_name = FindResourceOrThrow("examples/jumping/five_link_biped.urdf");
// 	parser.AddModelFromFile(full_name);

// 	plant.mutable_gravity_field().set_gravity_vector(-9.81 * Eigen::Vector3d::UnitZ());

// 	plant.WeldFrames(
// 			plant.world_frame(), 
// 			plant.GetFrameByName("base"),
// 	  		drake::math::RigidTransform<double>()
// 	  		);

// 	plant.Finalize();

// 	return plant;
// }

// HybridDircon<double> run_traj_opt(MultibodyPlant<double>* plant){
drake::trajectories::PiecewisePolynomial<double> run_traj_opt(MultibodyPlant<double>* plant){

	Eigen::VectorXd x_0 = Eigen::VectorXd::Zero(	plant->num_positions()
												+ plant->num_velocities()
												);

	Eigen::VectorXd init_l_vec(2);
	init_l_vec << 0, 20*FLAGS_gravity;
	int num_forces = 4;
	int num_states = 10;

	// Initial states, forces, and constraints
	std::vector<MatrixXd> init_x; // states
	std::vector<MatrixXd> init_u; // forces
	std::vector<PiecewisePolynomial<double>> init_l_traj; // contact forces at knot points
	std::vector<PiecewisePolynomial<double>> init_lc_traj; // contact forces at collocation points
	std::vector<PiecewisePolynomial<double>> init_vc_traj; // velocity constraint at collocation points


	double time_constant = 0.2;

	std::vector<double> init_time;
	for (int i = 0; i < 2*num_states - 1; ++i){
		init_time.push_back(i*time_constant);
		init_x.push_back(x_0);
		init_u.push_back(VectorXd::Random(num_forces));
	}

	auto init_x_traj = PiecewisePolynomial<double>::ZeroOrderHold(init_time, init_x);
	auto init_u_traj = PiecewisePolynomial<double>::ZeroOrderHold(init_time, init_u);

	for (int i = 0; i < 2; ++i){
		std::vector<MatrixXd> init_l_j;
		std::vector<MatrixXd> init_lc_j;
		std::vector<MatrixXd> init_vc_j;
		std::vector<double> init_time_j;
		for (int i = 0; i < num_states; ++i) {
			init_time_j.push_back(i*time_constant);
			init_l_j.push_back(init_l_vec);
			init_lc_j.push_back(init_l_vec);
			init_vc_j.push_back(VectorXd::Zero(2));
		}

		auto init_l_traj_j = PiecewisePolynomial<double>::ZeroOrderHold(init_time_j, init_l_j);
		auto init_lc_traj_j = PiecewisePolynomial<double>::ZeroOrderHold(init_time_j, init_l_j);
		auto init_vc_traj_j = PiecewisePolynomial<double>::ZeroOrderHold(init_time_j, init_l_j);
	
		init_l_traj.push_back(init_l_traj_j);
		init_lc_traj.push_back(init_lc_traj_j);
		init_vc_traj.push_back(init_vc_traj_j);
	}
	// End of initalization

	// Start of constraint specification
	const Body<double>& left_lower_leg = plant->GetBodyByName("left_lower_leg");
	const Body<double>& right_lower_leg = plant->GetBodyByName("right_lower_leg");

	Vector3d pt;
	pt << 0, 0, -0.5;
	bool isXZ = true;

	auto leftFootConstraint = DirconPositionData<double>(*plant, left_lower_leg,
	                                                   pt, isXZ);
	auto rightFootConstraint = DirconPositionData<double>(*plant, right_lower_leg,
                                                        pt, isXZ);


	// FIGURE OUT WHAT THIS IS
	Vector3d normal;
	normal << 0, 0, 1;
	double mu = 1;

	leftFootConstraint.addFixedNormalFrictionConstraints(normal, mu);
	rightFootConstraint.addFixedNormalFrictionConstraints(normal, mu);

	// Making a vector for a single constraint seems unncessary
	std::vector<DirconKinematicData<double>*> leftConstraints;
	leftConstraints.push_back(&leftFootConstraint);
	auto leftDataSet = DirconKinematicDataSet<double>(*plant, &leftConstraints);

	std::vector<DirconKinematicData<double>*> rightConstraints;
	rightConstraints.push_back(&rightFootConstraint);
	auto rightDataSet = DirconKinematicDataSet<double>(*plant, &rightConstraints);
	
	auto leftOptions = DirconOptions(leftDataSet.countConstraints());
	leftOptions.setConstraintRelative(0, true);

	auto rightOptions = DirconOptions(rightDataSet.countConstraints());
	rightOptions.setConstraintRelative(0, true);
	// End of constraint specification

	// FIGURE OUT WHAT THIS IS
	std::vector<int> timesteps;
	timesteps.push_back(10);
	timesteps.push_back(10);
	std::vector<double> min_dt;
	min_dt.push_back(.01);
	min_dt.push_back(.01);
	std::vector<double> max_dt;
	max_dt.push_back(.3);
	max_dt.push_back(.3);

	std::vector<DirconKinematicDataSet<double>*> dataset_list;
	dataset_list.push_back(&leftDataSet);
	dataset_list.push_back(&rightDataSet);

	std::vector<DirconOptions> options_list;
	options_list.push_back(leftOptions);
	options_list.push_back(rightOptions);

	// Trajectory Optimization Setup
	auto trajopt = std::make_shared<HybridDircon<double>>(*plant, 
													timesteps,
													min_dt,
													max_dt,
													dataset_list,
													options_list);

	trajopt->AddDurationBounds(FLAGS_max_duration, FLAGS_max_duration);
	trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(),
							"Print file", "five_link_biped_snopt.out");
	trajopt->SetSolverOption(drake::solvers::SnoptSolver::id(),
							"Major iterations limit", 100);
	for (uint j = 0; j < timesteps.size(); j++) {
		trajopt->drake::systems::trajectory_optimization::MultipleShooting::SetInitialTrajectory(init_u_traj, init_x_traj);
		trajopt->SetInitialForceTrajectory(j, 
								init_l_traj[j],
								init_lc_traj[j],
		                        init_vc_traj[j]);
	}


	// Linear constraints
	auto x0 = trajopt->initial_state();
	auto xf = trajopt->final_state();

	int num_q = plant->num_positions();
	
	// trajopt->AddLinearConstraint(x);

	const double R = 10;
	auto u = trajopt->input();
	trajopt->AddRunningCost(u.transpose()*R*u);
	// trajopt->AddLinearConstraint(x0(positions_map["planar_z"]) == initial_x);
	

	// Actual solving
	auto start = std::chrono::high_resolution_clock::now();
	const auto result = Solve(*trajopt, trajopt->initial_guess());
	auto finish = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsed = finish - start;
	std::cout << "Solve time:" << elapsed.count() <<std::endl;
	std::cout << "Cost:" << result.get_optimal_cost() <<std::endl;

	return trajopt->ReconstructStateTrajectory(result);
}

void simulate_traj(MultibodyPlant<double>* plant){

	//output initial states
	auto positions_map = multibody::makeNameToPositionsMap(*plant);
	auto velocities_map = multibody::makeNameToVelocitiesMap(*plant);
	for (auto const& element : positions_map)
		cout << element.first << " = " << element.second << endl;
	for (auto const& element : velocities_map)
		cout << element.first << " = " << element.second << endl;

	// drake::systems::Simulator<double> simulator(*diagram);
	// drake::systems::Context<double>& context =
	// 	diagram->GetMutableSubsystemContext(*plant, &simulator.get_mutable_context());



	return;
}

int doMain(int argc, char* argv[]){
	gflags::ParseCommandLineFlags(&argc, &argv, true);
	DiagramBuilder<double> builder;
	
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

	auto optimal_traj = run_traj_opt(&plant);
	// run_traj_opt(&plant);

	// const drake::trajectories::PiecewisePolynomial<double> pp_xtraj = optimal_traj->ReconstructStateTrajectory(result);
	multibody::connectTrajectoryVisualizer(	&plant, 
											&builder, 
											&scene_graph,
											optimal_traj
											);

	auto diagram = builder.Build();


	drake::systems::Simulator<double> simulator(*diagram);
	drake::systems::Context<double>& context =
		diagram->GetMutableSubsystemContext(plant, &simulator.get_mutable_context());
	int num_u = plant.num_actuators();
	auto zero_input = Eigen::MatrixXd::Zero(num_u,1);
	context.FixInputPort(0, zero_input);




	simulator.set_target_realtime_rate(FLAGS_realtime_factor);
	simulator.Initialize();
	simulator.StepTo(10);

	// simulate_traj();
	return 0;
}



}  // namespace jumping
}  // namespace examples
}  // namespace dairlib

int main(int argc, char* argv[]) { 
	return dairlib::examples::jumping::doMain(argc, argv); 
}
