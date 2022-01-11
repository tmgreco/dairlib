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

    void config(std::string yaml_path, std::string saved_directory, int index,MultibodyPlant<Y>* plant);
    /// generateInitialGuess, generates a bad initial guess for the spirit jump traj opt
    void generateInitialGuess(MultibodyPlant<Y>& plant);

    // addConstraints, adds constraints to the trajopt jump problem. See runSpiritJump for a description of the inputs
    
    void addConstraints(MultibodyPlant<Y>& plant, 
                        dairlib::systems::trajectory_optimization::Dircon<Y>& trajopt
                        );

    /// runSpiritJump, runs a trajectory optimization problem for spirit jumping on flat ground
    void run(MultibodyPlant<Y>& plant,
            PiecewisePolynomial<Y>* pp_xtraj,
            std::vector<SurfaceConf>* surface_vector);

    std::tuple<  std::vector<std::unique_ptr<dairlib::systems::trajectory_optimization::DirconMode<Y>>>,
    std::vector<std::unique_ptr<multibody::WorldPointEvaluator<Y>>> ,
    std::vector<std::unique_ptr<multibody::KinematicEvaluatorSet<Y>>>>
    getModeSequence(
                        MultibodyPlant<Y>& plant,
                        DirconModeSequence<Y>& sequence,
                        std::vector<std::string>& mode_vector,
                        std::vector<double>& minT_vector,
                        std::vector<double>& maxT_vector){
        dairlib::ModeSequenceHelper msh;
        this->getModeSequenceHelper(msh, mode_vector,minT_vector,maxT_vector);
        
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

