#ifndef _spirit_parkour
#define _spirit_parkour

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
#include "examples/Spirit/spirit_optimal_stand.h"


using drake::multibody::MultibodyPlant;
using drake::trajectories::PiecewisePolynomial;
namespace dairlib {
    using systems::trajectory_optimization::Dircon;


template <class Y>  
class SpiritParkourJump : public Behavior<Y> {
public:

    SpiritParkourJump();
    void config(std::string yaml_path, std::string saved_directory, int index,MultibodyPlant<Y>* plant);
    
    void offsetConstraint(
        MultibodyPlant<Y>& plant,
        dairlib::systems::trajectory_optimization::Dircon<Y>& trajopt,
        const Eigen::Ref<const drake::solvers::VectorXDecisionVariable>& xb,
        Eigen::Vector3d normal,
        Eigen::Vector3d offset,
        double eps2 = 0.01
        );
    /// generateInitialGuess, generates a bad initial guess for the spirit jump traj opt
    void generateInitialGuess(MultibodyPlant<Y>& plant);

    void concatVectorTraj(
        vector<PiecewisePolynomial<Y>>& traj, 
        vector<PiecewisePolynomial<Y>> otherTraj, 
        bool mergeInnerMode);
    // addConstraints, adds constraints to the trajopt jump problem. See runSpiritJump for a description of the inputs
    void addConstraints(MultibodyPlant<Y>& plant, 
                        dairlib::systems::trajectory_optimization::Dircon<Y>& trajopt
                        );

    /// runSpiritJump, runs a trajectory optimization problem for spirit jumping on flat ground
    void run(MultibodyPlant<Y>& plant,
            PiecewisePolynomial<Y>* pp_xtraj,
            std::vector<SurfaceConf>* surface_vector);

    void setUpModeSequence();

    /// addCost, adds the cost to the trajopt jump problem. See runSpiritBoxJump for a description of the inputs
    void addCost(
            MultibodyPlant<Y>& plant,
            dairlib::systems::trajectory_optimization::Dircon<Y>& trajopt);

    void addCostLegs(MultibodyPlant<Y>& plant,
                 dairlib::systems::trajectory_optimization::Dircon<Y>& trajopt,
                 const double cost_actuation,
                 const double cost_velocity,
                 const std::vector<double>& joints,
                 const int mode_index);

    std::tuple<  std::vector<std::unique_ptr<dairlib::systems::trajectory_optimization::DirconMode<Y>>>,
    std::vector<std::unique_ptr<multibody::WorldPointEvaluator<Y>>> ,
    std::vector<std::unique_ptr<multibody::KinematicEvaluatorSet<Y>>>>
    getModeSequence(MultibodyPlant<Y>& plant,
                    DirconModeSequence<Y>& sequence){
    for (int i=0;i<nJumps*5+1;i++){
        if (i%5==3 || i%5==4) this->num_knot_points.push_back(nKnotpoints_flight);
        else this->num_knot_points.push_back(nKnotpoints_stances);
    }
    
    dairlib::ModeSequenceHelper msh;
    this->getModeSequenceHelper(msh);

    auto [modeVector, toeEvals, toeEvalSets] = createSpiritModeSequence(plant, msh);

    for (auto& mode : modeVector){
        for (int i = 0; i < mode->evaluators().num_evaluators(); i++ ){
        mode->MakeConstraintRelative(i,0);
        mode->MakeConstraintRelative(i,1);
        }
        mode->SetDynamicsScale(
            {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19}, 150.0);
        if (mode->evaluators().num_evaluators() == 4)
        {
        mode->SetKinVelocityScale(
            {0, 1, 2, 3}, {0, 1, 2}, 1.0);
        mode->SetKinPositionScale(
            {0, 1, 2, 3}, {0, 1, 2}, 150.0);
        }
        else if (mode->evaluators().num_evaluators() == 2){
        mode->SetKinVelocityScale(
            {0, 1}, {0, 1, 2}, 1.0);
        mode->SetKinPositionScale(
            {0, 1}, {0, 1, 2}, 150.0);
        }
        sequence.AddMode(mode.get());
    }
    return {std::move(modeVector), std::move(toeEvals), std::move(toeEvalSets)};
    }

private:
    // vector<PiecewisePolynomial<Y>> x_traj;


    std::unique_ptr<MultibodyPlant<Y>> plant;
    std::vector<std::tuple<Eigen::Vector3d,Eigen::Vector3d,double>> transitionSurfaces;
    
    int nKnotpoints_flight;
    int nKnotpoints_stances;

    double cost_velocity_legs_flight;
    double cost_actuation_legs_flight;
    double apex_height;
    double max_apex_pitch_magnitude;
    double max_pitch_magnitude;
    double initial_height;
    double fore_aft_displacement;
    bool lock_rotation;
    bool lock_legs_apex;
    bool force_symmetry;
    bool use_nominal_stand;
    double max_duration;
    double eps;  
    double work_constraint_scale;
    int nJumps;
    double stand_height;
public:
    dairlib::OptimalSpiritStand initialStand;
    dairlib::OptimalSpiritStand finalStand;
};
}

#endif

