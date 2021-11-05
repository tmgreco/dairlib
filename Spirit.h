#ifndef _spirit
#define _spirit

#include <memory>
#include <chrono>
#include <unistd.h>
#include <gflags/gflags.h>
#include <string.h>
#include <cmath>
#include <experimental/filesystem>

#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/ipopt_solver.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/multibody/parsing/parser.h"
#include <drake/multibody/inverse_kinematics/inverse_kinematics.h>
#include <drake/solvers/choose_best_solver.h>
#include "drake/geometry/drake_visualizer.h"
#include "drake/solvers/solve.h"

#include "common/find_resource.h"
#include "systems/trajectory_optimization/dircon/dircon.h"
#include "multibody/kinematic/world_point_evaluator.h"
#include "solvers/nonlinear_constraint.h"
#include "multibody/kinematic/distance_evaluator.h"
#include "multibody/multibody_utils.h"
#include "multibody/visualization_utils.h"
#include "multibody/kinematic/kinematic_constraints.h"

#include "examples/Spirit/animate_spirit.h"
#include "common/file_utils.h"
#include "lcm/dircon_saved_trajectory.h"
#include "solvers/optimization_utils.h"
#include "solvers/nonlinear_cost.h"
#include "examples/Spirit/spirit_jump.h"
#include "examples/Spirit/behavior_configuration.h"
using drake::multibody::MultibodyPlant;
using drake::trajectories::PiecewisePolynomial;
using drake::geometry::SceneGraph;

namespace dairlib {
    using systems::trajectory_optimization::DirconModeSequence;
    using systems::trajectory_optimization::DirconMode;
    using systems::trajectory_optimization::Dircon;
    using std::vector;

template <class T>
class Spirit {

public:


    Spirit();
    // Spirit(std::string urdf_path):plant (make_unique<MultibodyPlant<T>>()),
    //             plant_vis (make_unique<MultibodyPlant<T>>()),
    //             scene_graph (make_unique<SceneGraph<T>>());
    
    void jump();


private:
    double apex_goal;
    double duration;
    bool ipopt;
    double standHeight;
    double foreAftDisplacement;
    double inputCost;
    double velocityCost;
    double eps;
    double tol;
    double mu;

    std::string data_directory;
    std::string distance_name;
    
    bool runAllOptimization;
    bool skipInitialOptimization;
    bool minWork;


    std::unique_ptr<MultibodyPlant<T>> plant;
    std::unique_ptr<MultibodyPlant<T>> plant_vis;
    std::unique_ptr<SceneGraph<T>> scene_graph;
    dairlib::SpiritJump<JumpConfiguration,T> jump_behavior;

};
}


#endif
