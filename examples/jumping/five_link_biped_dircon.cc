
#include <gflags/gflags.h>
#include "drake/lcm/drake_lcm.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/systems/rendering/multibody_position_to_geometry_pose.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/solvers/solve.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/parsing/parser.h"

#include "multibody/multibody_utils.h"
#include "multibody/visualization_utils.h"
#include "common/find_resource.h"
#include "systems/primitives/subvector_pass_through.h"
#include "solvers/optimization_utils.h"
#include "systems/trajectory_optimization/dircon_position_data.h"
#include "systems/trajectory_optimization/dircon_kinematic_data_set.h"
#include "systems/trajectory_optimization/hybrid_dircon.h"
#include "systems/trajectory_optimization/dircon_opt_constraints.h"



DEFINE_double(realtime_factor, .5,
			  "Playback speed.  See documentation for "
			  "Simulator::set_target_realtime_rate() for details.");

// Simulation parameters.
DEFINE_double(timestep, 1e-5, "The simulator time step (s)");

using drake::multibody::MultibodyPlant;
using drake::multibody::Body;
using drake::multibody::Parser;
using drake::geometry::SceneGraph;
using drake::systems::DiagramBuilder;
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

void run_traj_opt(MultibodyPlant<double>* plant){
	Eigen::VectorXd x0 = Eigen::VectorXd::Zero(	plant.num_positions()
												+ plant.num_velocities()
												);

	Eigen::VectorXd init_l_vec(2);
	init_l_vec << 0, 20*9.81;
	int num_forces = 4;
	int num_states = 10;

	std::vector<MatrixXd> init_x;
	std::vector<MatrixXd> init_u;
	std::vector<PiecewisePolynomial<double>> init_l_traj;
	std::vector<PiecewisePolynomial<double>> init_lc_traj;
	std::vector<PiecewisePolynomial<double>> init_vc_traj;

	std::vector<double> init_time;
	for (int i = 0; i < 2*num_states - 1; ++i){
		init_time.push_back(i*.2);
		init_x.push_back(x0);
		init_u.push_back(VectorXd::Random(num_forces));
	}

	auto init_x_traj = PiecewisePolynomial<double>::ZeroOrderHold(init_time, init_x);
	auto init_u_traj = PiecewisePolynomial<double>::ZeroOrderHold(init_time, init_u);

	for (int i = 0; i < 2; ++i){
		std::vector<MatrixXd> init_l_j;
		std::vector<MatrixXd> init_lc_j;
		std::vector<MatrixXd> init_vc_j;
		std::vector<double> init_time_j;
		for (int )
	}

	return;
}

void simulate_traj(MultibodyPlant<double>* plant){

	//output initial states
	auto positions_map = multibody::makeNameToPositionsMap(*plant);
	auto velocities_map = multibody::makeNameToVelocitiesMap(*plant);
	for (auto const& element : positions_map)
		cout << element.first << " = " << element.second << endl;
	for (auto const& element : velocities_map)
		cout << element.first << " = " << element.second << endl;



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

	plant.mutable_gravity_field().set_gravity_vector(-9.81 * Eigen::Vector3d::UnitZ());

	plant.WeldFrames(
			plant.world_frame(), 
			plant.GetFrameByName("base"),
			drake::math::RigidTransform<double>()
			);

	plant.Finalize();

	auto optimal_traj = run_traj_opt(&plant);

	const drake::trajectories::PiecewisePolynomial<double> pp_xtraj = trajopt->ReconstructStateTrajectory(result);
	multibody::connectTrajectoryVisualizer(	&plant, 
											&builder, 
											&scene_graph,
											pp_xtraj
											);
	auto diagram = builder.Build();
	simulate_traj();
	return 0;
}



}  // namespace jumping
}  // namespace examples
}  // namespace dairlib

int main(int argc, char* argv[]) { 
	return dairlib::examples::jumping::doMain(argc, argv); 
}
