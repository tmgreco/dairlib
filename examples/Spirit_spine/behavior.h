#ifndef _spirit_behavior
#define _spirit_behavior

#include <memory>
#include <chrono>
#include <unistd.h>
#include <gflags/gflags.h>
#include <string.h>
#include <iostream>
#include <fstream>

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
#include "examples/Spirit_spine/spirit_utils.h"
#include "examples/Spirit_spine/surface_conf.h"

using drake::multibody::MultibodyPlant;
using drake::trajectories::PiecewisePolynomial;
using drake::geometry::SceneGraph;
using drake::solvers::VariableRefList;

namespace dairlib {
    
    using systems::trajectory_optimization::DirconModeSequence;
    using std::vector;
    using std::map;

    /// Abstract base class for all spirit behaviors
    template <class Y> 
    class Behavior{
        public:
        /// Adds cost to the trajopt problem
        /// \param plant robot model
        /// \param trajopt trajectory optimization problem to be solved
        void addCost(MultibodyPlant<Y>& plant, 
                    dairlib::systems::trajectory_optimization::Dircon<Y>& trajopt){
                        auto u   = trajopt.input();
                        // Setup the traditional cost function
                        trajopt.AddRunningCost( u.transpose()*cost_actuation*u);
                        trajopt.AddVelocityCost(cost_velocity);
                        AddWorkCost(plant, trajopt, cost_work);
                    }

        /// Assigns values to member variables according to input yaml file
        /// \param yaml_path path of the yaml file
        /// \param saved_directory directory for saving the output trajectories (namely for setting file_name_out)
        /// \param index indicates that we are using the index^th configuration
        /// \param plant: robot model
        virtual void config(std::string yaml_path, std::string saved_directory, int index,MultibodyPlant<Y>* plant) = 0; //Maybe it should load configuration directly from the path
        
        /// Adds constraints to the trajopt problem
        /// \param plant robot model
        /// \param trajopt trajectory optimization problem to be solved
        virtual void addConstraints(
                            MultibodyPlant<Y>& plant, 
                            dairlib::systems::trajectory_optimization::Dircon<Y>& trajopt
                            ) =0;

        /// Run a trajectory optimization problem for spirit behavior
        /// \param plant robot model
        /// \param pp_xtraj state trajectory pointer used for animation
        /// \param surface_vector vector of surfaces in the scene (for animation)
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


        /// Set up mode sequence for trajectory optimization problem. Will be defined differently in different behaviors
        virtual void setUpModeSequence()=0;
        /// Typically enable animation at the last optimization
        void enable_animate(){get_animate_info=true;}
        bool if_animate(){return get_animate_info;}
        void setFileNameIn(std::string name) {file_name_in=name;}
        void setFileNameOut(std::string name) {file_name_out=name;}
        std::string getFileNameIn(){ return file_name_in;}
        std::string getFileNameOut(){ return file_name_out;}
        void setMeanAndVar(double m, double v){
            mean=m; 
            var=v;
        }
        double getCost(){return mechanical_work;}

        std::string urdf_path;
        std::string spine_type;
        std::string action;
        std::string data_directory;
    protected:
        double mechanical_work;
        double mechanical_power;
        double electrical_work;
        double electrical_power;
        double mean;
        double var;
        int index;
        std::vector<int> num_knot_points; //!< Sequence of #knot points for each mode
        double mu; //!< Coefficient of friction
        double tol; //!< Tolerance for solver
        bool ipopt; //!< Choice of optimization solver. True=ipopt, False=snopt
        double cost_actuation; //!< Cost coefficient of actuation
        double cost_velocity; //!< Cost coefficient of velocity
        double cost_work; //!< Cost coefficient of work
        double cost_power; //!< Cost coefficient of average power
        double cost_velocity_legs_flight; //!< Cost coefficient of velocity for legs during flight
        double cost_actuation_legs_flight; //!< Cost coefficient of actuation for legs during flight
        double cost_time; //!< Cost coefficient of time
        bool get_animate_info=false; //!< Whether or not return the trajectory to Spirit for animation


        std::string file_name_out; //!< Store optimized trajectories into this file if not empty
        std::string file_name_in= ""; //!< Load initial trajectories from this file if not empty

        PiecewisePolynomial<Y> x_traj; //!< initial and solution state trajectory
        PiecewisePolynomial<Y> u_traj; //!< initial and solution control trajectory
        vector<PiecewisePolynomial<Y>> l_traj; //!< initial and solution contact force trajectory
        vector<PiecewisePolynomial<Y>> lc_traj; //!< initial and solution contact force slack variable trajectory
        vector<PiecewisePolynomial<Y>> vc_traj; //!< initial and solution contact velocity slack variable trajectory
        

        std::vector<std::string> mode_vector; //!< vector of mode names for setting up mode sequence
        std::vector<double> minT_vector; //!< vector of minimum time of each mode for setting up mode sequence
        std::vector<double> maxT_vector; //!< vector of maximum time of each mode for setting up mode sequence
        std::vector<Eigen::Vector3d> normal_vector; //!< vector of normal vectors of toes' contact planes
        std::vector<Eigen::Vector3d> offset_vector; //!< vector of offset vectors of toes' contact planes

        /// constraint the orientation of a state within a quaternion with a tolerance
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

        /// Push elements to the five mode sequence helper vectors
        void addModeToSequenceVector(std::string mode_name,
                                    Eigen::Vector3d normal, 
                                    Eigen::Vector3d offset,
                                    double minT,
                                    double maxT){
            mode_vector.push_back(mode_name);
            normal_vector.push_back(normal);
            offset_vector.push_back(offset);                           
            minT_vector.push_back(minT);
            maxT_vector.push_back(maxT);
        }

        /// get mode sequence helper according to the five mode sequence helper vectors
        void getModeSequenceHelper(dairlib::ModeSequenceHelper& msh){
            int counter=0;
            for (std::string mode_name : mode_vector){
                Eigen::Matrix<bool,1,4> contact_bool;
                if (mode_name=="stance") contact_bool<< true,  true,  true,  true;
                else if (mode_name=="flight") contact_bool<< false, false, false, false;
                else if (mode_name=="rear_stance") contact_bool<<false, true, false, true;
                else if (mode_name=="front_stance") contact_bool<< true, false, true ,false;
                else if (mode_name=="diag1") contact_bool<< true, false, false, true ;
                else if (mode_name=="diag2") contact_bool<<  false,true,true, false ;
                else std::cout<<"Wrong mode name!"<<std::endl;
                msh.addMode( // Stance
                    contact_bool, // contact bools
                    num_knot_points[counter],  // number of knot points in the collocation
                    normal_vector[counter], // normal
                    offset_vector[counter],  // world offset
                    mu, //friction
                    minT_vector[counter], // minimum time
                    maxT_vector[counter] // maximum time
                    );
                
                counter++;
            }
        }                   

        /// getModeSequence, initializes the trajopt mode seqence for jump, see runSpiritJump for a def of inputs
        virtual std::tuple<  std::vector<std::unique_ptr<dairlib::systems::trajectory_optimization::DirconMode<Y>>>,
                    std::vector<std::unique_ptr<multibody::WorldPointEvaluator<Y>>> ,
                    std::vector<std::unique_ptr<multibody::KinematicEvaluatorSet<Y>>>>
        getModeSequence(
                        MultibodyPlant<Y>& plant,
                        DirconModeSequence<Y>& sequence)=0;
        /// Set solver
        /// \param trajopt trajectory optimization problem to be solved
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
        /// Initialize trajectory
        /// \param trajopt trajectory optimization problem to be solved
        void initTrajectory(MultibodyPlant<Y>& plant,
                                dairlib::systems::trajectory_optimization::Dircon<Y>& trajopt,
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
                    addGaussionNoiseToInputTraj(plant,trajopt,loaded_traj);
                    addGaussionNoiseToStateTraj(plant,trajopt,loaded_traj);
                }
            }

        /// \param trajopt trajectory optimization problem to be solved
        void setupVisualization(dairlib::systems::trajectory_optimization::Dircon<Y>& trajopt,
                                DirconModeSequence<Y>& sequence){
            int num_ghosts = 3;// Number of ghosts in visualization. NOTE: there are limitations on number of ghosts based on modes and knotpoints
            std::vector<unsigned int> visualizer_poses; // Ghosts for visualizing during optimization
            for(int i = 0; i < sequence.num_modes(); i++){
                visualizer_poses.push_back(num_ghosts); 
            }
            trajopt.CreateVisualizationCallback(
                dairlib::FindResourceOrThrow(urdf_path),
                visualizer_poses, 0.2); // setup which URDF, how many poses, and alpha transparency 
        }
        
        /// \param trajopt trajectory optimization problem to be solved
        void saveTrajectory(MultibodyPlant<Y>& plant,
                            dairlib::systems::trajectory_optimization::Dircon<Y>& trajopt,
                            const drake::solvers::MathematicalProgramResult& result){
            if(!file_name_out.empty()){
                dairlib::DirconTrajectory saved_traj(
                    plant, trajopt, result, "Optimized trajectory",
                    "Decision variables and state/input trajectories "
                    "for a bahavior");
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
            mechanical_work=dairlib::calcMechanicalWork(plant, x_trajs, this->u_traj);
            mechanical_power=dairlib::calcMechanicalWork(plant, x_trajs, this->u_traj)/this->u_traj.end_time();
            electrical_work=dairlib::calcElectricalWork(plant, x_trajs, this->u_traj, 0);
            electrical_power=electrical_work/this->u_traj.end_time();
            std::cout<<"Electrical Work = " << electrical_work << std::endl;
            std::cout<<"Mechanical Work = " <<  mechanical_work<< std::endl;
            std::cout<<"Electrical power = " << electrical_power << std::endl;
            std::cout<<"Mechanical Power = " <<  mechanical_power<< std::endl;
            //  double cost_work_acceleration = solvers::EvalCostGivenSolution(
            //      result, cost_joint_work_bindings);
            //  std::cout<<"Cost Work = " << cost_work_acceleration << std::endl;
        }

        void perturbResult(MultibodyPlant<Y>& plant,
                            dairlib::systems::trajectory_optimization::Dircon<Y>& trajopt,
                            drake::solvers::MathematicalProgramResult& result)
        {
            for (int mode = 0; mode < trajopt.num_modes(); mode++) {
                for (int j = 0; j < trajopt.mode_length(mode); j++) {
                    drake::VectorX<Y> xk = result.GetSolution(trajopt.state_vars(mode, j));
                    // std::cout<<"Mode: "<< mode << ", knot: "<< j <<", state: \n" << xk << std::endl;
                    result.AddNoiseToSolution(trajopt.state_vars(mode, j),mean,var);
                    xk = result.GetSolution(trajopt.state_vars(mode, j));
                    // std::cout<<"After adding noise\nMode: "<< mode << ", knot: "<< j <<", state: \n" << xk << std::endl;
                }
            }
        }

        void saveContactForceData(double param, std::string file_path, bool is_success){
            std::ofstream myfile; // 
            myfile.open(file_path);
            myfile << "param,"<<param << ",Electrical work," << electrical_work <<",Electrical power,"<<electrical_power <<",success?,"<<is_success<< "\n";
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
                    if (mode_vector[mode]=="stance") contact_bool<< true,  true,  true,  true;
                    else if (mode_vector[mode]=="flight") contact_bool<< false, false, false, false;
                    else if (mode_vector[mode]=="rear_stance") contact_bool<<false, true, false, true;
                    else if (mode_vector[mode]=="front_stance") contact_bool<< true, false, true ,false;
                    else if (mode_vector[mode]=="diag1") contact_bool<< true, false, false, true ;
                    else if (mode_vector[mode]=="diag2") contact_bool<<  false,true,true, false ;
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
                for (int i=4;i<7;i++) myfile << this->x_traj.value(current_time)(i,0) << ",";
                if (spine_type=="twisting"){
                    for (int i=3;i<6;i++) myfile << this->x_traj.value(current_time)(20+i,0) << ",";
                    myfile << this->x_traj.value(current_time)(7,0) << ","<< this->x_traj.value(current_time)(26,0) << ","<<
                            this->u_traj.value(current_time)(0,0)<<",";
                    myfile<<",,,";
                    for (int i=0;i<39;i++) myfile << this->x_traj.value(current_time)(i,0) << ",";
                    myfile<<",,";
                    for (int i=0;i<13;i++) myfile << this->u_traj.value(current_time)(i,0) << ",";
                    }
                else{
                    for (int i=3;i<6;i++) myfile << this->x_traj.value(current_time)(19+i,0) << ",";
                    myfile<<",,,";
                    for (int i=0;i<37;i++) myfile << this->x_traj.value(current_time)(i,0) << ",";
                    myfile<<",,";
                    for (int i=0;i<12;i++) myfile << this->u_traj.value(current_time)(i,0) << ",";
                }
                myfile <<"\n";
            }
            myfile.close(); // <- note this correction!!
        }

        void addGaussionNoiseToStateTraj(MultibodyPlant<Y>& plant,
                                            dairlib::systems::trajectory_optimization::Dircon<Y>& trajopt,
                                            dairlib::DirconTrajectory& initial_guess_traj){
            drake::trajectories::PiecewisePolynomial<Y> state_traj=initial_guess_traj.ReconstructStateTrajectory();

            std::vector<double> breaks=state_traj.get_breaks();
            int num_states=plant.num_positions()+plant.num_velocities();
            auto normal_dist = std::bind(std::normal_distribution<double>{mean, var/100},
                                std::mt19937(std::random_device{}())); // Normal distribution
            auto normal_dist_joints = std::bind(std::normal_distribution<double>{mean, var},
                                std::mt19937(std::random_device{}())); // Normal distribution
            
            for (int i = 0; i < static_cast<int>(breaks.size())-1; ++i) {
                drake::MatrixX<drake::Polynomial<Y>> org_matrix=state_traj.getPolynomialMatrix(i);
                // for (int j=4;j<7;j++){
                    // drake::Polynomial<Y> scalar_poly(normal_dist());
                    // org_matrix(j)*=scalar_poly;
                // }
                drake::Polynomial<Y> scalar_poly(normal_dist());
                org_matrix(4)*=scalar_poly;
                org_matrix(23)*=scalar_poly;
                
                state_traj.setPolynomialMatrixBlock(org_matrix,i);
            }

            trajopt.drake::systems::trajectory_optimization::MultipleShooting::
                        SetInitialTrajectory(initial_guess_traj.ReconstructInputTrajectory(), state_traj);
        }
        void addGaussionNoiseToVelocitiesTraj(MultibodyPlant<Y>& plant,
                                            dairlib::systems::trajectory_optimization::Dircon<Y>& trajopt,
                                            dairlib::DirconTrajectory& initial_guess_traj){
            PiecewisePolynomial<Y> state_traj = initial_guess_traj.ReconstructStateTrajectory();
            /// Create offset polynomial
            std::vector<double> breaks=state_traj.get_breaks();
            std::vector<Eigen::MatrixXd> samples(breaks.size());
            int num_states=plant.num_positions()+plant.num_velocities();
            auto normal_dist = std::bind(std::normal_distribution<double>{mean, var},
                                std::mt19937(std::random_device{}())); // Normal distribution
            auto normal_dist_joints = std::bind(std::normal_distribution<double>{mean, var},
                                std::mt19937(std::random_device{}())); // Normal distribution
            // std::cout<<"Generate "<<breaks.size()<<" random dist. "<<"mean "<<mean<<" var "<<var<<std::endl;
            for (int i = 0; i < static_cast<int>(breaks.size()); ++i) {
                samples[i].resize(num_states, 1);
                double scaling=normal_dist();
                // std::cout<<i<<": "<<normal_dist()<<std::endl;
                for (int j=0;j<num_states;j++) samples[i](j, 0) = 1;
                for (int j=20;j<26;j++) samples[i](j, 0) = scaling; // x offset
                for (int j=26;j<num_states;j++) samples[i](j, 0) = normal_dist_joints();
            }
            PiecewisePolynomial<Y> offset_pp = PiecewisePolynomial<Y>::FirstOrderHold(breaks, samples);

            state_traj*=offset_pp;
            trajopt.drake::systems::trajectory_optimization::MultipleShooting::
                        SetInitialTrajectory(initial_guess_traj.ReconstructInputTrajectory(), state_traj);
        }

        void addGaussionNoiseToInputTraj(MultibodyPlant<Y>& plant,
                                            dairlib::systems::trajectory_optimization::Dircon<Y>& trajopt,
                                            dairlib::DirconTrajectory& initial_guess_traj){
            PiecewisePolynomial<Y> input_traj = initial_guess_traj.ReconstructInputTrajectory();

            std::vector<double> breaks=input_traj.get_breaks();
            auto normal_dist = std::bind(std::normal_distribution<double>{mean, var},
                                std::mt19937(std::random_device{}())); // Normal distribution
            
            for (int i = 0; i < static_cast<int>(breaks.size())-1; ++i) {
                drake::MatrixX<drake::Polynomial<Y>> org_matrix=input_traj.getPolynomialMatrix(i);
                for (int j=0;j<plant.num_actuators();j++){
                    drake::Polynomial<Y> scalar_poly(normal_dist());
                    org_matrix(j)*=scalar_poly;
                }
                
                input_traj.setPolynomialMatrixBlock(org_matrix,i);
            }

            trajopt.drake::systems::trajectory_optimization::MultipleShooting::
                        SetInitialTrajectory(input_traj ,initial_guess_traj.ReconstructStateTrajectory());
        }
    };
}



#endif