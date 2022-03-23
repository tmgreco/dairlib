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
namespace dairlib {
    using systems::trajectory_optimization::Dircon;


template <class Y>  
class SpiritJump : public Behavior<Y> {
public:

    SpiritJump();

    /// Assigns values to member variables according to input yaml file
    /// \param yaml_path path of the yaml file
    /// \param saved_directory directory for saving the output trajectories (namely for setting file_name_out)
    /// \param index indicates that we are using the index^th configuration
    /// \param plant: robot model
    void config(std::string yaml_path, std::string saved_directory, int index,MultibodyPlant<Y>* plant);
    
    /// Generate a bad initial guess for the spirit jump traj opt
    void generateInitialGuess(MultibodyPlant<Y>& plant);

    /// Adds constraints to the trajopt jump problem
    /// \param plant robot model
    /// \param trajopt trajectory optimization problem to be solved
    void addConstraints(MultibodyPlant<Y>& plant, 
                        dairlib::systems::trajectory_optimization::Dircon<Y>& trajopt
                        );

    /// Run a trajectory optimization problem for spirit jump
    /// \param plant robot model
    /// \param pp_xtraj state trajectory pointer used for animation
    /// \param surface_vector vector of surfaces in the scene (for animation)
    void run(MultibodyPlant<Y>& plant,
            PiecewisePolynomial<Y>* pp_xtraj,
            std::vector<SurfaceConf>* surface_vector);

    void setUpModeSequence();

    std::tuple<  std::vector<std::unique_ptr<dairlib::systems::trajectory_optimization::DirconMode<Y>>>,
    std::vector<std::unique_ptr<multibody::WorldPointEvaluator<Y>>> ,
    std::vector<std::unique_ptr<multibody::KinematicEvaluatorSet<Y>>>>
    getModeSequence(
                        MultibodyPlant<Y>& plant,
                        DirconModeSequence<Y>& sequence){
        dairlib::ModeSequenceHelper msh;
        this->getModeSequenceHelper(msh);
        
        auto [modeVector, toeEvals, toeEvalSets] = createSpiritModeSequence(plant, msh);

        for (auto& mode : modeVector){
            for (int i = 0; i < mode->evaluators().num_evaluators(); i++ ){
            mode->MakeConstraintRelative(i,0);
            mode->MakeConstraintRelative(i,1);
            }
            mode->SetDynamicsScale(
                {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17}, 200);
            if (mode->evaluators().num_evaluators() > 0)
            {
            mode->SetKinVelocityScale(
                {0, 1, 2, 3}, {0, 1, 2}, 1);
            mode->SetKinPositionScale(
                {0, 1, 2, 3}, {0, 1, 2}, 200);
            }
            sequence.AddMode(mode.get());
        }
        return {std::move(modeVector), std::move(toeEvals), std::move(toeEvalSets)};
        }

private:
    double duration; //!< maximum duration of the jumping behavior
    double apex_height;
    double initial_height; //!< initial stand height
    double fore_aft_displacement; //!< how long the robot jump forward
    bool lock_rotation;
    bool lock_legs_apex; //!< reach a specific legs configuration at apex
    bool force_symmetry;
    bool use_nominal_stand;
    double max_duration; //!< maximum duration of the jumping behavior
    double eps; //!< tolerance for the constraints

};
}

#endif

