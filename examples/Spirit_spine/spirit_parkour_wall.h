#ifndef _spirit_parkour_wall
#define _spirit_parkour_wall

#include <cmath>
#include <experimental/filesystem>
#include <filesystem>
#include <math.h>

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
class SpiritParkourWallPronk : public Behavior<Y> {
public:

    SpiritParkourWallPronk();
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

    /// Run a trajectory optimization problem for spirit parkour wall jump
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
        if (i%5==2 || i%5==3) this->num_knot_points.push_back(nKnotpoints_flight);
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
            {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20}, 150.0);
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


    double max_apex_pitch_magnitude; //!< max pitch magnitude at apex
    double max_pitch_magnitude; //!< ax pitch magnitude at stances
    double fore_aft_displacement; //!< how long the robot jump forward
    bool pose_ref;
    bool use_nominal_stand;
    double max_duration; //!< maximum duration of the bounding behavior
    double eps;  //!< tolerance for the constraints
    double work_constraint_scale;
    double roll_ref;
    double pitch_ref;
    double yaw_ref;
    double final_x;
    int nJumps; //!< number of jumps
    double xtol; //!< tolerace of transformed x axis displacement where robot land on the transition surfaces
    double ytol; //!< tolerace of initial and final y position for adjusting lateral distance to the wall
    std::vector<double> apex_heights;
    bool warm_up;
    bool apex_flag=true;
    void saveContactForceData(dairlib::systems::trajectory_optimization::Dircon<Y>& trajopt,
                            const drake::solvers::MathematicalProgramResult& result,
                            double forward_dist, double lateral_dist, std::string file_path, bool is_success){
        drake::trajectories::PiecewisePolynomial<Y> state_traj=trajopt.ReconstructStateTrajectory(result);
        std::ofstream myfile; // 
        myfile.open(file_path);
        myfile << "Forward dist,"<<forward_dist <<",Lateral dist,"<<lateral_dist<< ",Electrical work," << this->electrical_work <<",Electrical power,"<<this->electrical_power <<",success?,"<<is_success<< "\n";
        myfile<< "Time, Front L ,,, Front R ,,, Back L ,,, Back R ,,,, x, y, z, vx, vy, vz,";
        if (this->spine_type=="twisting") {
            myfile<<"joint 12, joint 12 dot, joint 12 torque ,,,,qw,qx,qy,qz,x,y,z,q12,q9,q11,q8,q10,q2,q6,q0,q4,q3,q7,q1,q5,";
            myfile<<"wx,wy,wz,vx,vy,vz,q12d,q9d,q11d,q8d,q10d,q2d,q6d,q0d,q4d,q3d,q7d,q1d,q5d,,,";
            myfile<<"f12,f8,f0,f1,f9,f2,f3,f10,f4,f5,f11,f6,f7\n";
        }
        else {myfile<<",,,qw,qx,qy,qz,x,y,z,q8,q9,q10,q11,q0,q2,q4,q6,q1,q3,q5,q8,";
            myfile<<"wx,wy,wz,vx,vy,vz,q8d,q9d,q10d,q11d,q0d,q2d,q4d,q6d,q1d,q3d,q5d,q8d,,,";
            myfile<<"f8,f0,f1,f9,f2,f3,f10,f4,f5,f11,f6,f7\n";
        }
        Y traj_end_time=this->u_traj.end_time();
        for (Y current_time=0;current_time<traj_end_time;current_time+=0.01){
            int mode=-1;
            for (int i=0;i<this->mode_vector.size();i++){
            Y start_time=this->l_traj[i].start_time();
            Y end_time=this->l_traj[i].end_time();
            if (abs(start_time)!=std::numeric_limits<Y>::infinity() && abs(end_time)!=std::numeric_limits<Y>::infinity() 
                && current_time>=start_time && current_time<end_time){
                mode=i;
            }
            }
            myfile << current_time << ",";
            if (mode==-1){
                for (int i = 0; i < 12; i++) myfile << 0 << ",";
            }
            else{
                // leg order: front left, rare left, front right, rare left
                Eigen::Matrix<bool,1,4> contact_bool;
                if (this->mode_vector[mode]=="stance") contact_bool<< true,  true,  true,  true;
                else if (this->mode_vector[mode]=="flight") contact_bool<< false, false, false, false;
                else if (this->mode_vector[mode]=="rear_stance") contact_bool<<false, true, false, true;
                else if (this->mode_vector[mode]=="front_stance") contact_bool<< true, false, true ,false;
                else if (this->mode_vector[mode]=="diag1") contact_bool<< true, false, false, true ;
                else if (this->mode_vector[mode]=="diag2") contact_bool<<  false,true,true, false ;
                int temp_index=0;
                for (int j=0;j<4;j++){
                    if (contact_bool(1,j)){
                        for (int k=0;k<3;k++) myfile << this->l_traj[mode].value(current_time)(temp_index+k,0) << ",";
                        temp_index+=3;
                    }
                    else{
                        for (int k=0;k<3;k++) myfile << 0 << ",";
                    }
                }
            }
            myfile<<",";
            for (int i=4;i<7;i++) myfile << state_traj.value(current_time)(i,0) << ",";
            if (this->spine_type=="twisting"){
                for (int i=3;i<6;i++) myfile << state_traj.value(current_time)(20+i,0) << ",";
                myfile << state_traj.value(current_time)(7,0) << ","<< state_traj.value(current_time)(26,0) << ","<<
                        this->u_traj.value(current_time)(0,0)<<",";
                myfile<<",,,";
                for (int i=0;i<39;i++) myfile << state_traj.value(current_time)(i,0) << ",";
                myfile<<",,";
                for (int i=0;i<13;i++) myfile << this->u_traj.value(current_time)(i,0) << ",";
                }
            else{
                for (int i=3;i<6;i++) myfile << state_traj.value(current_time)(19+i,0) << ",";
                myfile<<",,,";
                for (int i=0;i<37;i++) myfile << state_traj.value(current_time)(i,0) << ",";
                myfile<<",,";
                for (int i=0;i<12;i++) myfile << this->u_traj.value(current_time)(i,0) << ",";
            }
            myfile <<"\n";
        }
        myfile.close(); // <- note this correction!!
    }

public:
    dairlib::OptimalSpiritStand initialStand;
    dairlib::OptimalSpiritStand finalStand;
};
}

#endif

