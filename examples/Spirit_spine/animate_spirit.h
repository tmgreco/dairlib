#pragma once
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
#include "drake/geometry/drake_visualizer.h"
#include "drake/solvers/solve.h"

#include "multibody/visualization_utils.h"

#include "common/find_resource.h"
#include "systems/trajectory_optimization/dircon/dircon.h"
#include "multibody/kinematic/world_point_evaluator.h"
#include "multibody/multibody_utils.h"
#include "multibody/visualization_utils.h"
#include "multibody/kinematic/kinematic_constraints.h"
#include "systems/trajectory_optimization/dircon/dircon.h"
#include "lcm/dircon_saved_trajectory.h"
#include "examples/Spirit_spine/surface_conf.h"


#include "examples/Spirit_spine/spirit_utils.h"
namespace dairlib {

  /// See runAnimate(
  ///    std::unique_ptr<MultibodyPlant<T>> plant_ptr,
  ///    MultibodyPlant<double>* plant_double_ptr,
  ///    std::unique_ptr<SceneGraph<double>> scene_graph_ptr,
  ///    PiecewisePolynomial<double> pp_xtraj )
  ///
  /// Takes the plants and scenegraph and a trajectory and 
  /// creates a visualization of that trajectory (example
  /// built in the main file).
template <typename T>
void runAnimate(
    std::unique_ptr<drake::multibody::MultibodyPlant<T>> plant_ptr,
    drake::multibody::MultibodyPlant<double>* plant_double_ptr,
    std::unique_ptr<drake::geometry::SceneGraph<double>> scene_graph_ptr,
    drake::trajectories::PiecewisePolynomial<double> pp_xtraj,
    double real_time_factor = 1
    ) ;

template <typename T>
void addGaussionNoiseToStateTraj(drake::multibody::MultibodyPlant<T>& plant,
                                drake::trajectories::PiecewisePolynomial<T>& state_traj);

void animateTraj(std::string& urdf_path);

void addOffset(drake::trajectories::PiecewisePolynomial<double>& state_traj,double x, double y);

void copyAndStitchMultiplePeriods(drake::trajectories::PiecewisePolynomial<double>& state_traj);

template <typename T>
void rotateTraj(drake::multibody::MultibodyPlant<T>& plant,
                drake::trajectories::PiecewisePolynomial<T>& state_traj,
                double yaw);
}  // dairlib namespace
