/*
 * File: spirit_jump.h
 * ----------------
 * This interface for spirit jumping behaviors.
 * Date: 2021-10-30
 */

#ifndef _spirit_jump
#define _spirit_jump

#include <cmath>
#include <experimental/filesystem>
#include <filesystem>


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
#include "examples/Spirit/behavior.h"

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
class SpiritJump : public Behavior<Y> {
public:

    SpiritJump();

    void config(std::string yaml_path, std::string saved_directory, int index);
    /// generateInitialGuess, generates a bad initial guess for the spirit jump traj opt
    void generateInitialGuess(MultibodyPlant<Y>& plant);

    // addConstraints, adds constraints to the trajopt jump problem. See runSpiritJump for a description of the inputs
    
    void addConstraints(MultibodyPlant<Y>& plant, 
                        dairlib::systems::trajectory_optimization::Dircon<Y>& trajopt
                        );

    /// runSpiritJump, runs a trajectory optimization problem for spirit jumping on flat ground
    void run(MultibodyPlant<Y>& plant,
            PiecewisePolynomial<Y>* pp_xtraj);


private:
    double duration;
    bool animate;
    double apex_height;
    double initial_height;
    double fore_aft_displacement;
    bool lock_rotation;
    bool lock_legs_apex;
    bool force_symmetry;
    bool use_nominal_stand;
    double max_duration;
    double eps;

};
}

#endif

