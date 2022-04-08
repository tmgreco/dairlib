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

#include "examples/Spirit_spine/animate_spirit.h"
#include "common/file_utils.h"
#include "lcm/dircon_saved_trajectory.h"
#include "solvers/optimization_utils.h"
#include "solvers/nonlinear_cost.h"

#include "examples/Spirit_spine/spirit_utils.h"
#include "examples/Spirit_spine/behavior.h"
#include "examples/Spirit_spine/spirit_optimal_stand.h"


using drake::multibody::MultibodyPlant;
using drake::trajectories::PiecewisePolynomial;
namespace dairlib {
    using systems::trajectory_optimization::Dircon;


template <class Y>  
class SpiritParkourJump : public Behavior<Y> {
public:

    SpiritParkourJump();
    /// Assigns values to member variables according to input yaml file
    /// \param yaml_path path of the yaml file
    /// \param saved_directory directory for saving the output trajectories (namely for setting file_name_out)
    /// \param index indicates that we are using the index^th configuration
    /// \param plant: robot model
    void config(std::string yaml_path, std::string saved_directory, int index,MultibodyPlant<Y>* plant);
    
    /// Add constraints for states at the transitional surface
    /// \param plant robot model
    /// \param trajopt trajectory optimization problem to be solved
    /// \param xb state to be constrained
    /// \param normal normal vector of the transitional surface
    /// \param offset offset vector of the transitional surface
    /// \param eps2 particular tolerance for this constraint
    void offsetConstraint(
        MultibodyPlant<Y>& plant,
        dairlib::systems::trajectory_optimization::Dircon<Y>& trajopt,
        const Eigen::Ref<const drake::solvers::VectorXDecisionVariable>& xb,
        Eigen::Vector3d normal,
        Eigen::Vector3d offset,
        double eps2 = 0.01
        );
    /// Generate a bad initial guess by stitching several bounding behaviors together
    void generateInitialGuess(MultibodyPlant<Y>& plant);

    void concatVectorTraj(
        vector<PiecewisePolynomial<Y>>& traj, 
        vector<PiecewisePolynomial<Y>> otherTraj, 
        bool mergeInnerMode);
    /// Adds constraints to the trajopt parkour wall jump problem
    /// \param plant robot model
    /// \param trajopt trajectory optimization problem to be solved
    void addConstraints(MultibodyPlant<Y>& plant, 
                        dairlib::systems::trajectory_optimization::Dircon<Y>& trajopt
                        );

    /// Run a trajectory optimization problem for spirit parkour jump
    /// \param plant robot model
    /// \param pp_xtraj state trajectory pointer used for animation
    /// \param surface_vector vector of surfaces in the scene (for animation)
    void run(MultibodyPlant<Y>& plant,
            PiecewisePolynomial<Y>* pp_xtraj,
            std::vector<SurfaceConf>* surface_vector);

    void setUpModeSequence();

    /// Adds cost to the trajopt problem
    /// \param plant robot model
    /// \param trajopt trajectory optimization problem to be solved
    void addCost(
            MultibodyPlant<Y>& plant,
            dairlib::systems::trajectory_optimization::Dircon<Y>& trajopt);

    /// add cost for legs
    /// \param joints add cost for which joint of the legs
    /// \param mode_index add most for which mode 
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

    std::unique_ptr<MultibodyPlant<Y>> plant; //!< the robot
    std::vector<std::tuple<Eigen::Vector3d,Eigen::Vector3d,double>> transitionSurfaces; //!< vector that stores transition surfaces (surface to jump)
    
    int nKnotpoints_flight; //!< number of know points for each flight mode
    int nKnotpoints_stances; //!< number of know points for each stance mode


    double apex_height;
    double max_apex_pitch_magnitude; //!< max pitch magnitude at apex
    double max_pitch_magnitude; //!< ax pitch magnitude at stances
    double initial_height; //!< initial stance height
    double fore_aft_displacement; //!< how long the robot jump forward
    bool lock_rotation;
    bool lock_legs_apex; //!< reach a specific legs configuration at apex
    bool force_symmetry;
    bool use_nominal_stand;
    double max_duration; //!< maximum duration of the bounding behavior
    double eps;   //!< tolerance for the constraints
    double work_constraint_scale;
    int nJumps; //!< number of jumps
    double stand_height;
public:
    dairlib::OptimalSpiritStand initialStand;
    dairlib::OptimalSpiritStand finalStand;
};
}

#endif

