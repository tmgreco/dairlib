#ifndef _spirit_behavior
#define _spirit_behavior

#include <memory>
#include <chrono>
#include <unistd.h>
#include <gflags/gflags.h>
#include <string.h>

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

using drake::multibody::MultibodyPlant;
using drake::trajectories::PiecewisePolynomial;
using drake::geometry::SceneGraph;
namespace dairlib {
template <typename C, class Y> // C: confuguration struct
class Behavior{
    public:
    virtual void addCost(
                MultibodyPlant<Y>& plant, 
                dairlib::systems::trajectory_optimization::Dircon<Y>& trajopt)=0;

    // addConstraints, adds constraints to the trajopt jump problem. See runSpiritJump for a description of the inputs
    virtual void config(C input_configuration) = 0;
    virtual void addConstraints(
                        MultibodyPlant<Y>& plant, 
                        dairlib::systems::trajectory_optimization::Dircon<Y>& trajopt,
                        ) =0;

    /// getModeSequence, initializes the trajopt mode seqence for jump, see runSpiritJump for a def of inputs

    virtual std::tuple<  std::vector<std::unique_ptr<dairlib::systems::trajectory_optimization::DirconMode<Y>>>,
                std::vector<std::unique_ptr<multibody::WorldPointEvaluator<Y>>> ,
                std::vector<std::unique_ptr<multibody::KinematicEvaluatorSet<Y>>>>
    getModeSequence(
        MultibodyPlant<Y>& plant, 
        DirconModeSequence<Y>& sequence) = 0;

    virtual void loadOldTrajectory(std::string traj_dir)=0;

    /// runSpiritJump, runs a trajectory optimization problem for spirit jumping on flat ground
    virtual void run(MultibodyPlant<Y>& plant) = 0;


private:
    C configuration;

    PiecewisePolynomial<Y> x_traj; /// initial and solution state trajectory
    PiecewisePolynomial<Y> u_traj; /// initial and solution control trajectory
    vector<PiecewisePolynomial<Y>> l_traj; /// initial and solution contact force trajectory
    vector<PiecewisePolynomial<Y>> lc_traj; /// initial and solution contact force slack variable trajectory
    vector<PiecewisePolynomial<Y>> vc_traj; /// initial and solution contact velocity slack variable trajectory
    };
} 

#endif