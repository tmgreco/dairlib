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

#include "systems/trajectory_optimization/dircon/dircon.h"
#include "examples/Spirit/spirit_utils.h"
#include "examples/Spirit/surface_conf.h"
using drake::multibody::MultibodyPlant;
using drake::trajectories::PiecewisePolynomial;
using drake::geometry::SceneGraph;
using drake::solvers::VariableRefList;

namespace dairlib {
    
    using systems::trajectory_optimization::DirconModeSequence;
    using std::vector;
    using std::map;
    template <class Y> 
    class Behavior{
        public:
        
        void addCost(
                    MultibodyPlant<Y>& plant, 
                    dairlib::systems::trajectory_optimization::Dircon<Y>& trajopt){
                        auto u   = trajopt.input();
                        // Setup the traditional cost function
                        trajopt.AddRunningCost( u.transpose()*cost_actuation*u);
                        trajopt.AddVelocityCost(cost_velocity);
                        AddWorkCost(plant, trajopt, cost_work);
                    }

        // addConstraints, adds constraints to the trajopt jump problem. See runSpiritJump for a description of the inputs
        virtual void config(std::string yaml_path, std::string saved_directory, int index,MultibodyPlant<Y>* plant) = 0; //Maybe it should load configuration directly from the path
        virtual void addConstraints(
                            MultibodyPlant<Y>& plant, 
                            dairlib::systems::trajectory_optimization::Dircon<Y>& trajopt
                            ) =0;

        /// runSpiritJump, runs a trajectory optimization problem for spirit jumping on flat ground
        virtual void run(MultibodyPlant<Y>& plant,
                        PiecewisePolynomial<Y>* pp_xtraj,
                        std::vector<SurfaceConf>* surface_vector) = 0;

        void loadOldTrajectory(std::string traj_dir){
            dairlib::DirconTrajectory old_traj(traj_dir);
            this->x_traj = old_traj.ReconstructStateTrajectory();
            this->u_traj = old_traj.ReconstructInputTrajectory();
            this->l_traj = old_traj.ReconstructLambdaTrajectory();
            this->lc_traj = old_traj.ReconstructLambdaCTrajectory();
            this->vc_traj = old_traj.ReconstructGammaCTrajectory();
        }

    protected:
        std::vector<int> num_knot_points;
        double mu;
        double tol;
        bool ipopt;
        double cost_actuation;
        double cost_velocity;
        double cost_work;
        double cost_velocity_legs_flight;
        double cost_actuation_legs_flight;
        double cost_time;

        std::string file_name_out;
        std::string file_name_in= "";

        PiecewisePolynomial<Y> x_traj; /// initial and solution state trajectory
        PiecewisePolynomial<Y> u_traj; /// initial and solution control trajectory
        vector<PiecewisePolynomial<Y>> l_traj; /// initial and solution contact force trajectory
        vector<PiecewisePolynomial<Y>> lc_traj; /// initial and solution contact force slack variable trajectory
        vector<PiecewisePolynomial<Y>> vc_traj; /// initial and solution contact velocity slack variable trajectory

        void addPoseConstraints(dairlib::systems::trajectory_optimization::Dircon<Y>& trajopt,
                                Eigen::VectorBlock<const drake::solvers::VectorXDecisionVariable>  &var,
                                map<std::string, int> &name_map,
                                double qx,
                                double qy,
                                double qz,
                                double qw,
                                double tolerance){
            trajopt.AddBoundingBoxConstraint(qw - tolerance, qw + tolerance, var(name_map.at("base_qw")));
            trajopt.AddBoundingBoxConstraint(qx - tolerance, qx + tolerance, var(name_map.at("base_qx")));
            trajopt.AddBoundingBoxConstraint(qy - tolerance, qy + tolerance, var(name_map.at("base_qy")));
            trajopt.AddBoundingBoxConstraint(qz - tolerance, qz + tolerance, var(name_map.at("base_qz")));
        }


        void getModeSequenceHelper(
            dairlib::ModeSequenceHelper& msh,
            std::vector<std::string>& mode_vector,
            std::vector<double>& minT_vector,
            std::vector<double>& maxT_vector){
                int counter=0;
                for (std::string mode_name : mode_vector){
                    if (mode_name=="stance"){
                    msh.addMode( // Stance
                        (Eigen::Matrix<bool,1,4>() << true,  true,  true,  true).finished(), // contact bools
                        num_knot_points[counter],  // number of knot points in the collocation
                        Eigen::Vector3d::UnitZ(), // normal
                        Eigen::Vector3d::Zero(),  // world offset
                        mu, //friction
                        minT_vector[counter],
                        maxT_vector[counter]
                        );
                    }
                    else if(mode_name=="flight"){
                    msh.addMode( // Flight
                        (Eigen::Matrix<bool,1,4>() << false, false, false, false).finished(), // contact bools
                        num_knot_points[counter],  // number of knot points in the collocation
                        Eigen::Vector3d::UnitZ(), // normal
                        Eigen::Vector3d::Zero(),  // world offset
                        mu, //friction
                        minT_vector[counter],
                        maxT_vector[counter]
                        );
                    }
                    else if (mode_name=="rear_stance"){
                    msh.addMode( // Rear stance
                        (Eigen::Matrix<bool,1,4>() << false, true, false, true).finished(), // contact bools
                        num_knot_points[counter],  // number of knot points in the collocation
                        Eigen::Vector3d::UnitZ(), // normal
                        Eigen::Vector3d::Zero(),  // world offset
                        mu, //friction
                        minT_vector[counter],
                        maxT_vector[counter]
                        );
                    }
                    else if (mode_name=="front_stance"){
                    msh.addMode( // Rear stance
                        (Eigen::Matrix<bool,1,4>() <<  true, false, true ,false).finished(), // contact bools
                        num_knot_points[counter],  // number of knot points in the collocation
                        Eigen::Vector3d::UnitZ(), // normal
                        Eigen::Vector3d::Zero(),  // world offset
                        mu, //friction
                        minT_vector[counter],
                        maxT_vector[counter]
                        );
                    }
                    else{
                        std::cout<<"Wrong mode!"<<std::endl;
                    }
                    counter++;
                }
            }                   

        /// getModeSequence, initializes the trajopt mode seqence for jump, see runSpiritJump for a def of inputs
        virtual std::tuple<  std::vector<std::unique_ptr<dairlib::systems::trajectory_optimization::DirconMode<Y>>>,
                    std::vector<std::unique_ptr<multibody::WorldPointEvaluator<Y>>> ,
                    std::vector<std::unique_ptr<multibody::KinematicEvaluatorSet<Y>>>>
        getModeSequence(
                        MultibodyPlant<Y>& plant,
                        DirconModeSequence<Y>& sequence,
                        std::vector<std::string>& mode_vector,
                        std::vector<double>& minT_vector,
                        std::vector<double>& maxT_vector)=0;

    void setSolver(dairlib::systems::trajectory_optimization::Dircon<Y>& trajopt){
        if (ipopt) {
            // Ipopt settings adapted from CaSaDi and FROST
            auto id = drake::solvers::IpoptSolver::id();
            trajopt.SetSolverOption(id, "tol", tol);
            trajopt.SetSolverOption(id, "dual_inf_tol", tol);
            trajopt.SetSolverOption(id, "constr_viol_tol", tol);
            trajopt.SetSolverOption(id, "compl_inf_tol", tol);
            trajopt.SetSolverOption(id, "max_iter", 1000000);
            trajopt.SetSolverOption(id, "nlp_lower_bound_inf", -1e6);
            trajopt.SetSolverOption(id, "nlp_upper_bound_inf", 1e6);
            trajopt.SetSolverOption(id, "print_timing_statistics", "yes");
            trajopt.SetSolverOption(id, "print_level", 5);

            // Set to ignore overall tolerance/dual infeasibility, but terminate when
            // primal feasible and objective fails to increase over 5 iterations.
            trajopt.SetSolverOption(id, "acceptable_compl_inf_tol", tol);
            trajopt.SetSolverOption(id, "acceptable_constr_viol_tol", tol);
            trajopt.SetSolverOption(id, "acceptable_obj_change_tol", tol);
            trajopt.SetSolverOption(id, "acceptable_tol", tol);
            trajopt.SetSolverOption(id, "acceptable_iter", 5);
        } else {
            // Set up Trajectory Optimization options
            trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(),
                                    "Print file", "../snopt.out");
            trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(),
                                    "Major iterations limit", 100000);
            trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(), "Iterations limit", 100000);
            trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(),
                                    "Major optimality tolerance",
                                    tol);  // target optimality
            trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(), "Major feasibility tolerance", tol);
            trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(), "Verify level",
                                    0);  // 0
            }
        }

    void initTrajectory(dairlib::systems::trajectory_optimization::Dircon<Y>& trajopt,
                            DirconModeSequence<Y>& sequence){
            if (file_name_in.empty()){
                trajopt.drake::systems::trajectory_optimization::MultipleShooting::
                    SetInitialTrajectory(this->u_traj, this->x_traj);
                for (int j = 0; j < sequence.num_modes(); j++) {
                trajopt.SetInitialForceTrajectory(j, this->l_traj[j], this->lc_traj[j],
                                                    this->vc_traj[j]);
                }
            }else{
                std::cout<<"Loading decision var from file, will fail if num dec vars changed" <<std::endl;
                dairlib::DirconTrajectory loaded_traj(file_name_in);
                trajopt.SetInitialGuessForAllVariables(loaded_traj.GetDecisionVariables());
            }
        }

    void setupVisualization(dairlib::systems::trajectory_optimization::Dircon<Y>& trajopt,
                            DirconModeSequence<Y>& sequence){
        int num_ghosts = 3;// Number of ghosts in visualization. NOTE: there are limitations on number of ghosts based on modes and knotpoints
        std::vector<unsigned int> visualizer_poses; // Ghosts for visualizing during optimization
        for(int i = 0; i < sequence.num_modes(); i++){
            visualizer_poses.push_back(num_ghosts); 
        }
        trajopt.CreateVisualizationCallback(
            dairlib::FindResourceOrThrow("examples/Spirit/spirit_drake.urdf"),
            visualizer_poses, 0.2); // setup which URDF, how many poses, and alpha transparency 
    }

    void saveTrajectory(MultibodyPlant<Y>& plant,
                        dairlib::systems::trajectory_optimization::Dircon<Y>& trajopt,
                        const drake::solvers::MathematicalProgramResult& result){
        if(!file_name_out.empty()){
            dairlib::DirconTrajectory saved_traj(
                plant, trajopt, result, "Jumping trajectory",
                "Decision variables and state/input trajectories "
                "for jumping");
            saved_traj.WriteToFile(file_name_out);
            dairlib::DirconTrajectory old_traj(file_name_out);
            this->x_traj = old_traj.ReconstructStateTrajectory();
            this->u_traj = old_traj.ReconstructInputTrajectory();
            this->l_traj = old_traj.ReconstructLambdaTrajectory();
            this->lc_traj = old_traj.ReconstructLambdaCTrajectory();
            this->vc_traj = old_traj.ReconstructGammaCTrajectory();

        } else{
            std::cout << "warning no file name provided, will not be able to return full solution" << std::endl;
            this->x_traj  = trajopt.ReconstructStateTrajectory(result);
            this->u_traj  = trajopt.ReconstructInputTrajectory(result);
            this->l_traj  = trajopt.ReconstructLambdaTrajectory(result);
        }
        auto x_trajs = trajopt.ReconstructDiscontinuousStateTrajectory(result);
        std::cout<<"Work = " << dairlib::calcElectricalWork(plant, x_trajs, this->u_traj) << std::endl;
        //  double cost_work_acceleration = solvers::EvalCostGivenSolution(
        //      result, cost_joint_work_bindings);
        //  std::cout<<"Cost Work = " << cost_work_acceleration << std::endl;
    }
};
}
//     class QuadrupedalMode {
//   private:
//     bool contact_set_[4];
//     int num_knots_;
//     Eigen::Vector3d normal_;
//     Eigen::Vector3d offset_;
//     double mu_;
//     double minT_;
//     double maxT_;

//   public:
//     QuadrupedalMode(  bool contact_set[4],
//                       Eigen::Vector3d normal,
//                       Eigen::Vector3d offset,
//                       double mu = std::numeric_limits<double>::infinity() ,
//                       int num_knots = 7,
//                       double minT = 0,
//                       double maxT = std::numeric_limits<double>::infinity()   ) {
//       for (size_t i = 0; i < 4; i++)
//       {
//         contact_set_[i] = contact_set[i];
//       }
//       num_knots_ = num_knots;
//       normal_ = normal;
//       offset_ = offset;
//       mu_ = mu;
//       minT_ = minT;
//       maxT_ = maxT;
//       }
      
// QuadrupedalMode(std::string contact_set_str,
//                     Eigen::Vector3d normal,
//                     Eigen::Vector3d offset,
//                     double mu = std::numeric_limits<double>::infinity(),
//                     int num_knots = 7,
//                     double minT = 0,
//                     double maxT = std::numeric_limits<double>::infinity()) {
//       assert(contact_set_str.length() == 4  && "String must be four characters long");
//       transform(contact_set_str.begin(), contact_set_str.end(), contact_set_str.begin(), ::tolower);
//       for( int i = 0; i<4; i++){
//         assert(contact_set_str[i]=='t'||contact_set_str[i]=='f'||contact_set_str[i]=='0'||contact_set_str[i]=='1' && "String must be made up of t's and f's or 1's and 0's");
//         contact_set_[i] = (contact_set_str[i]!='f'&&contact_set_str[i]!='0');
//         }      
//       num_knots_ = num_knots;
//       normal_ = normal;
//       normal.normalize();
//       offset_ = offset;
//       mu_ = mu;
//       minT_ = minT;
//       maxT_ = maxT;
//       }

//     static QuadrupedalMode flight(int num_knots,
//                                  double minT = 0,
//                                   double maxT = std::numeric_limits<double>::infinity()){
//       return QuadrupedalMode( {false,false,false,false},
//                               Eigen::Vector3d::UnitZ(), 
//                               Eigen::Vector3d::Zero(), 
//                               std::numeric_limits<double>::infinity(),
//                               num_knots,
//                               minT,
//                               maxT);
//     }
//     // Simple verbose printing method
//     void print(){
//       std::cout << "************** Quadrupedal Mode *****" << std::endl;
//       std::cout << std::boolalpha;
//       std::cout << "Active Toe Contacts: " << contact_set_[0] << ", " << contact_set_[1] << ", " << contact_set_[2] << ", " << contact_set_[3] << std::endl;
//       std::cout << "Number of Knots: " << num_knots_ << std::endl;
//       std::cout << "Contact Normal: " << "(" << normal_.transpose() << ")^T" << std::endl;
//       std::cout << "Contact Offset: " << "(" << offset_.transpose() << ")^T" << std::endl;
//       std::cout << "Friction coefficient, mu: " << mu_ << std::endl;
//       std::cout << "Time Range: " << "min: " << minT_ << " max: "<<  maxT_ << std::endl;
//       std::cout << "*************************************" << std::endl;
//     }
//     // Get and Set methods
//     Eigen::Vector3d normal(){return normal_;}
//     Eigen::Vector3d offset(){return offset_;}
//     double mu(){return mu_;}
//     double minT(){return minT_;}
//     double maxT(){return maxT_;}
//     int num_knots(){return num_knots_;}

//     void set_normal(Eigen::Vector3d new_normal){ normal_ = new_normal;}
//     void set_offset(Eigen::Vector3d new_offset){ offset_ = new_offset;}
//     void set_mu(double mu){ mu_ = mu;}
//     void set_minT(double minT){ minT_ = minT;}
//     void set_maxT(double maxT){ maxT_ = maxT;}
//     void set_num_knots(int num_knots){num_knots_ = num_knots;}

//     Eigen::Vector3d affinePosition(double height){ return (offset() + (normal() * height)); }
    
// };


// class QuadrupedalModeSequence {
//   public:
//     std::vector<dairlib::QuadrupedalMode> mode_sequence;
//     void addMode(dairlib::QuadrupedalMode qMode){ mode_sequence.push_back(qMode); }
//     void addFlight(int num_knots,
//                    double minT = 0,
//                    double maxT = std::numeric_limits<double>::infinity()){
//       mode_sequence.push_back(dairlib::QuadrupedalMode::flight(num_knots,
//                                                                minT,
//                                                                maxT));
//     }
//     void print(){ for(auto mode : mode_sequence){mode.print();} }
//     void set_mu(double mu){for(auto mode : mode_sequence){mode.set_mu(mu);}
// };
// };




#endif