/*
 * File: spirit_jump.h
 * ----------------
 * This interface for spirit jumping behaviors.
 * Date: 2021-10-30
 */

#ifndef _spirit_jump
#define _spirit_jump

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

#include "examples/Spirit/spirit_utils.h"



using drake::multibody::MultibodyPlant;
using drake::trajectories::PiecewisePolynomial;
using drake::geometry::SceneGraph;
using drake::multibody::Parser;
namespace dairlib {
    using systems::trajectory_optimization::DirconModeSequence;
    using systems::trajectory_optimization::DirconMode;
    using systems::trajectory_optimization::Dircon;
    using std::vector;

template <class Y>
class SpiritJump {

public:

    SpiritJump();
    SpiritJump( double apex_goal, 
                double duration, 
                bool ipopt);
   
    /// badSpiritJump, generates a bad initial guess for the spirit jump traj opt
    void badSpiritJump(MultibodyPlant<Y>& plant);
    /// addCost, adds the cost to the trajopt jump problem. See runSpiritJump for a description of the inputs

    void addCost(
                MultibodyPlant<Y>& plant, 
                dairlib::systems::trajectory_optimization::Dircon<Y>& trajopt,
                const double cost_actuation,
                const double cost_velocity,
                const double cost_work);

    // addConstraints, adds constraints to the trajopt jump problem. See runSpiritJump for a description of the inputs
    
    void addConstraints(
                        MultibodyPlant<Y>& plant, 
                        dairlib::systems::trajectory_optimization::Dircon<Y>& trajopt,
                        const double apex_height,
                        const double initial_height,
                        const double fore_aft_displacement,
                        const bool lock_rotation,
                        const bool lock_legs_apex,
                        const bool force_symmetry,
                        const bool use_nominal_stand,
                        const double max_duration,
                        const double eps);

    /// getModeSequence, initializes the trajopt mode seqence for jump, see runSpiritJump for a def of inputs

    std::tuple<  std::vector<std::unique_ptr<dairlib::systems::trajectory_optimization::DirconMode<Y>>>,
                std::vector<std::unique_ptr<multibody::WorldPointEvaluator<Y>>> ,
                std::vector<std::unique_ptr<multibody::KinematicEvaluatorSet<Y>>>>
    getModeSequence(
        MultibodyPlant<Y>& plant, 
        const double mu,
        std::vector<int> num_knot_points,
        DirconModeSequence<Y>& sequence);

    void loadOldTrajectory(std::string traj_dir);

    /// runSpiritJump, runs a trajectory optimization problem for spirit jumping on flat ground
    void run(MultibodyPlant<Y>& plant,
        const bool animate,
        std::vector<int> num_knot_points,
        const double apex_height,
        const double initial_height,
        const double fore_aft_displacement,
        const bool lock_rotation,
        const bool lock_legs_apex,
        const bool force_symmetry,
        const bool use_nominal_stand,
        const double max_duration,
        const double cost_actuation,
        const double cost_velocity,
        const double cost_work,
        const double mu,
        const double eps,
        const double tol,
        const std::string& file_name_out,
        const std::string& file_name_in= ""
        );


private:
    double apex_goal;
    double duration;
    bool ipopt;

    
    PiecewisePolynomial<Y> x_traj; /// initial and solution state trajectory
    PiecewisePolynomial<Y> u_traj; /// initial and solution control trajectory
    vector<PiecewisePolynomial<Y>> l_traj; /// initial and solution contact force trajectory
    vector<PiecewisePolynomial<Y>> lc_traj; /// initial and solution contact force slack variable trajectory
    vector<PiecewisePolynomial<Y>> vc_traj; /// initial and solution contact velocity slack variable trajectory


};
}

#endif

