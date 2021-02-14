#include <memory>
#include <chrono>
#include <unistd.h>
#include <gflags/gflags.h>
#include <string.h>
#include <drake/multibody/inverse_kinematics/inverse_kinematics.h>

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

  std::vector<MatrixXd> x_points;
  std::vector<double> time_vec = {0,1,2,3};
  VectorXd x_const;
  dairlib::ikSpiritStand(*plant, x_const, {true, true, true, true}, 0.25, 0.15, 0, 0);
  x_points.push_back(x_const);
  dairlib::ikSpiritStand(*plant, x_const, {true, true, true, true}, 0.25, 0.2, 0, -0.35);
  x_points.push_back(x_const);
  dairlib::ikSpiritStand(*plant, x_const, {false, true, false, true}, 0.3, 0.25, 0, -0.35/2);
  x_points.push_back(x_const);
  dairlib::ikSpiritStand(*plant, x_const, {false, false, false, false}, 0.4, 0.25, 0, 0);
  x_points.push_back(x_const);

  PiecewisePolynomial<double> pp_xtraj = PiecewisePolynomial<double>::FirstOrderHold(time_vec,x_points);

  dairlib::runAnimate( std::move(plant), plant_vis.get(), std::move(scene_graph), pp_xtraj,1.0);
}

