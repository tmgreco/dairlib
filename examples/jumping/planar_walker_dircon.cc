
#include <gflags/gflags.h>
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/joints/floating_base_types.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

namespace dairlib{
namespace examples{
namespace jumping{


// Simple example which simulates the (passive) Kneed walker.  Run drake-visualizer
// to see the animated result.
DEFINE_double(realtime_factor, .5,
			  "Playback speed.  See documentation for "
			  "Simulator::set_target_realtime_rate() for details.");

// Simulation parameters.
DEFINE_double(timestep, 1e-5, "The simulator time step (s)");

MultibodyPlant<double> build_model() {
	MultibodyPlant<double> plant;
	SceneGraph<double>& scene_graph = *(builder.AddSystem<SceneGraph>());
	Parser parser(&plant, &scene_graph);
	std::string full_name = FindResourceOrThrow("examples/PlanarWalker/PlanarWalker.urdf");
	parser.AddModelFromFile(full_name);

	plant.mutable_gravity_field().set_gravity_vector(-9.81 * Eigen::Vector3d::UnitZ());

	plant.WeldFrames(
			plant.world_frame(), 
			plant.GetFrameByName("base"),
	  		drake::math::RigidTransform<double>()
	  		);

	plant.Finalize();

	return plant;
}

int run_traj_opt(MultibodyPlant<double>& plant){

}

int simulate_traj(){

	//output initial states
  	auto positions_map = multibody::makeNameToPositionsMap(plant);
	auto velocities_map = multibody::makeNameToVelocitiesMap(plant);
	for (auto const& element : positions_map)
		cout << element.first << " = " << element.second << endl;
	for (auto const& element : velocities_map)
		cout << element.first << " = " << element.second << endl;


}


int main(int argc, char* argv[]) {
	gflags::ParseCommandLineFlags(&argc, &argv, true);
	drake::systems::DiagramBuilder<double> builder;
	plant = build_model(builder);

	run_traj_opt(plant)


	multibody::connectTrajectoryVisualizer(	&plant, 
											&builder, 
											&scene_graph,
											pp_xtraj
											);
	auto diagram = builder.Build();
	simulate_traj();
}



}  // namespace jumping
}  // namespace examples
}  // namespace dairlib