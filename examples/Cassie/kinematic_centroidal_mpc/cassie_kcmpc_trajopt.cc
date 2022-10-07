
#include <drake/systems/framework/diagram_builder.h>
#include <drake/geometry/scene_graph.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <iostream>
#include <drake/systems/primitives/trajectory_source.h>
#include <drake/systems/rendering/multibody_position_to_geometry_pose.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/geometry/drake_visualizer.h>
#include "common/find_resource.h"
#include "systems/controllers/kinematic_centroidal_mpc/kinematic_centroidal_mpc.h"
#include "examples/Cassie/cassie_utils.h"
#include "systems/primitives/subvector_pass_through.h"
#include "multibody/com_pose_system.h"
#include "multibody/visualization_utils.h"

using drake::geometry::SceneGraph;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::geometry::DrakeVisualizer;

void DoMain(int n_knot_points, double duration, double com_height, double tol){
  // Create fix-spring Cassie MBP
  drake::systems::DiagramBuilder<double> builder;
  SceneGraph<double>& scene_graph = *builder.AddSystem<SceneGraph>();
  scene_graph.set_name("scene_graph");

  MultibodyPlant<double> plant(0.0);
  MultibodyPlant<double> plant_vis(0.0);

  Parser parser(&plant);
  Parser parser_vis(&plant_vis, &scene_graph);

  std::string full_name =
      dairlib::FindResourceOrThrow("examples/Cassie/urdf/cassie_fixed_springs.urdf");
  parser.AddModelFromFile(full_name);
  parser_vis.AddModelFromFile(full_name);
  plant.Finalize();
  plant_vis.Finalize();
  std::map<std::string, int> positions_map = dairlib::multibody::MakeNameToPositionsMap(plant);

  auto left_toe_pair = dairlib::LeftToeFront(plant);
  auto left_heel_pair = dairlib::LeftToeRear(plant);
  auto right_toe_pair = dairlib::RightToeFront(plant);
  auto right_heel_pair = dairlib::RightToeRear(plant);

  std::vector<int> toe_active_inds{0, 1, 2};
  std::vector<int> heel_active_inds{0, 1, 2};

  auto left_toe_eval = std::make_shared<dairlib::multibody::WorldPointEvaluator<double>>(
      plant, left_toe_pair.first, left_toe_pair.second,
      Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), toe_active_inds);

  auto left_heel_eval = std::make_shared<dairlib::multibody::WorldPointEvaluator<double>>(
      plant, left_heel_pair.first, left_heel_pair.second,
      Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), heel_active_inds);

  auto right_toe_eval = std::make_shared<dairlib::multibody::WorldPointEvaluator<double>>(
      plant, right_toe_pair.first, right_toe_pair.second,
      Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), toe_active_inds);

  auto right_heel_eval = std::make_shared<dairlib::multibody::WorldPointEvaluator<double>>(
      plant, right_heel_pair.first, right_heel_pair.second,
      Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), heel_active_inds);

  std::cout<<"Creating MPC"<<std::endl;
  KinematicCentroidalMPC mpc (plant, n_knot_points, duration/(n_knot_points-1),
                              {left_toe_eval, left_heel_eval, right_toe_eval, right_heel_eval});

  std::cout<<"Setting initial guess"<<std::endl;
  mpc.SetZeroInitialGuess();

  double cost_force = 1;
  double cost_state = 1;
  std::cout<<"Adding force cost"<<std::endl;
  mpc.AddConstantForceTrackingReference(Eigen::VectorXd::Zero(12), cost_force * Eigen::MatrixXd::Identity(12,12));

  std::cout<<"Adding state cost"<<std::endl;
  Eigen::VectorXd reference_state = Eigen::VectorXd::Zero(plant.num_positions() + plant.num_velocities());
  reference_state[positions_map.at("base_qw")] = 1;
  reference_state[positions_map.at("base_z")] = com_height;
  mpc.AddConstantStateReference(reference_state,
                                cost_state * Eigen::MatrixXd::Identity(plant.num_positions() + plant.num_velocities(),plant.num_positions() + plant.num_velocities()));

  std::cout<<"Adding solver options"<<std::endl;
  {
    drake::solvers::SolverOptions options;
    auto id = drake::solvers::IpoptSolver::id();
    options.SetOption(id, "tol", tol);
    options.SetOption(id, "dual_inf_tol", tol);
    options.SetOption(id, "constr_viol_tol", tol);
    options.SetOption(id, "compl_inf_tol", tol);
    options.SetOption(id, "max_iter", 500);
    options.SetOption(id, "nlp_lower_bound_inf", -1e6);
    options.SetOption(id, "nlp_upper_bound_inf", 1e6);
    options.SetOption(id, "print_timing_statistics", "yes");
    options.SetOption(id, "print_level", 5);

    // Set to ignore overall tolerance/dual infeasibility, but terminate when
    // primal feasible and objective fails to increase over 5 iterations.
    options.SetOption(id, "acceptable_compl_inf_tol", tol);
    options.SetOption(id, "acceptable_constr_viol_tol", tol);
    options.SetOption(id, "acceptable_obj_change_tol", 1e-3);
    options.SetOption(id, "acceptable_tol", 1e2);
    options.SetOption(id, "acceptable_iter", 5);
    mpc.Build(options);
  }

  std::cout<<"Adding visualization callback"<<std::endl;
  double alpha = .2;
  mpc.CreateVisualizationCallback(
      "examples/Cassie/urdf/cassie_fixed_springs.urdf", alpha);

  std::cout << "Solving DIRCON\n\n";
  const auto pp_xtraj = mpc.Solve();

  auto traj_source =
      builder.AddSystem<drake::systems::TrajectorySource>(pp_xtraj);
  auto passthrough = builder.AddSystem<dairlib::systems::SubvectorPassThrough>(
      plant.num_positions() + plant.num_velocities(), 0, plant.num_positions());
  builder.Connect(traj_source->get_output_port(),
                  passthrough->get_input_port());
  auto to_pose =
      builder.AddSystem<drake::systems::rendering::MultibodyPositionToGeometryPose<double>>(plant_vis);
  builder.Connect(passthrough->get_output_port(), to_pose->get_input_port());

  builder.Connect(
      to_pose->get_output_port(),
      scene_graph.get_source_pose_port(plant_vis.get_source_id().value()));

  DrakeVisualizer<double>::AddToBuilder(&builder, scene_graph);
  auto diagram = builder.Build();

  while (true) {
    drake::systems::Simulator<double> simulator(*diagram);
    simulator.set_target_realtime_rate(.1);
    simulator.Initialize();
    simulator.AdvanceTo(pp_xtraj.end_time());
  }

}

int main(int argc, char* argv[]) {
  DoMain(12, 0.5, 0.9,1e-4);
}
