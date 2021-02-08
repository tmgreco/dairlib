#include <memory>
#include <chrono>
#include <unistd.h>
#include <gflags/gflags.h>
#include <string.h>

#include "drake/solvers/snopt_solver.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/solvers/solve.h"

#include "common/find_resource.h"
#include "systems/trajectory_optimization/dircon/dircon.h"
#include "multibody/kinematic/world_point_evaluator.h"
#include "multibody/multibody_utils.h"
#include "multibody/visualization_utils.h"
#include "multibody/kinematic/kinematic_constraints.h"

#include "examples/Spirit/animate_spirit.h"
#include "examples/Spirit/spirit_utils.h"
#include "lcm/dircon_saved_trajectory.h"

using drake::AutoDiffXd;
using drake::multibody::MultibodyPlant;
using drake::geometry::SceneGraph;
using drake::multibody::Parser;
using drake::trajectories::PiecewisePolynomial;

using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::Matrix3d;
using Eigen::MatrixXd;

DEFINE_string(data_directory, "/home/shane/Drake_ws/dairlib/examples/Spirit/saved_trajectories/",
              "directory to save/read data");

DEFINE_string(file_name, "jump_10m_hq_work4","file to read data");


int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  auto plant = std::make_unique<MultibodyPlant<double>>(0.0);
  auto plant_vis = std::make_unique<MultibodyPlant<double>>(0.0);
  auto scene_graph = std::make_unique<SceneGraph<double>>();
  Parser parser(plant.get());
  Parser parser_vis(plant_vis.get(), scene_graph.get());
  std::string full_name =
      dairlib::FindResourceOrThrow("examples/Spirit/spirit_drake.urdf");

  parser.AddModelFromFile(full_name);
  parser_vis.AddModelFromFile(full_name);
 
  plant->Finalize();
  plant_vis->Finalize();

  dairlib::DirconTrajectory old_traj(FLAGS_data_directory+FLAGS_file_name);
  auto x_trajs = old_traj.ReconstructStateDiscontinuousTrajectory();
  auto x_traj = old_traj.ReconstructStateTrajectory();
  auto u_traj = old_traj.ReconstructInputTrajectory();

  std::cout<<"Mechanical Work = " << dairlib::calcWork(*plant, x_trajs, u_traj) << std::endl;

  std::cout<<"Electrical Work 1 = " << dairlib::calcElectricalWork1(*plant, x_trajs, u_traj) << std::endl;

  std::cout<<"Electrical Work 2 = " << dairlib::calcElectricalWork2(*plant, x_trajs, u_traj) << std::endl;

  std::cout<<"Electrical Work 3 = " << dairlib::calcElectricalWork3(*plant, x_trajs, u_traj) << std::endl;

  std::cout<<"Integral of Actuation = " << dairlib::calcTorqueInt(*plant, u_traj) << std::endl;

  std::cout<<"Integral of Velocity = " << dairlib::calcVelocityInt(*plant, x_trajs) << std::endl;

  dairlib::runAnimate( std::move(plant), plant_vis.get(), std::move(scene_graph), x_traj,0.25);
}

