#include <memory>
#include <chrono>
#include <tuple>
#include <unistd.h>
#include <gflags/gflags.h>
#include <string.h>
#include <assert.h>
#include <Eigen/StdVector>

#include "drake/solvers/snopt_solver.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/multibody/parsing/parser.h"
#include <drake/multibody/inverse_kinematics/inverse_kinematics.h>
#include "drake/geometry/geometry_visualization.h"
#include "drake/solvers/solve.h"

#include "common/find_resource.h"
#include "systems/trajectory_optimization/dircon/dircon.h"
#include "multibody/kinematic/world_point_evaluator.h"
#include "solvers/nonlinear_constraint.h"
#include "multibody/kinematic/distance_evaluator.h"
#include "multibody/multibody_utils.h"
#include "multibody/visualization_utils.h"
#include "multibody/kinematic/kinematic_constraints.h"

#include "common/file_utils.h"
#include "lcm/dircon_saved_trajectory.h"
#include "solvers/optimization_utils.h"

#include "examples/Spirit/spirit_utils.h"
#include "examples/Spirit/spirit_optimal_stand.h"


using drake::AutoDiffXd;
using drake::multibody::MultibodyPlant;
using drake::geometry::SceneGraph;
using drake::multibody::Parser;
using drake::trajectories::PiecewisePolynomial;

using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::Matrix3d;
using Eigen::MatrixXd;

namespace dairlib {

    using systems::trajectory_optimization::DirconModeSequence;
    using systems::trajectory_optimization::DirconMode;
    using systems::trajectory_optimization::Dircon;
    using systems::trajectory_optimization::KinematicConstraintType;

    using std::vector;
    using std::cout;
    using std::endl;

    OptimalSpiritStand::OptimalSpiritStand(
            drake::multibody::MultibodyPlant<double>* plantPtr, 
            double height, 
            Eigen::Vector3d normal,
            double tol,
            bool animate) {     // Constructor
        height_ = height;
        normal_ = normal/normal.norm();
        plantPtr_ = plantPtr;
        filename_ = "optimalStand_"+ std::to_string( static_cast<int>(height*100) ) +"cm";
        std::ifstream ifile;
        ifile.open(folder_+filename_);
        if(ifile){
            
            ifile.close();
            std::cout<<"Loading decision var from file, will fail if num dec vars changed" <<std::endl;
            dairlib::DirconTrajectory loaded_traj(folder_+filename_);
            fullstate_ = (loaded_traj.GetStateSamples(0) ).col(0);
        }else{
            fullstate_ = getSpiritOptimalStand();
        }
    }

    Eigen::VectorXd OptimalSpiritStand::getSpiritOptimalStand( double tol ){

        dairlib::ModeSequenceHelper msh;
        // Setup mode sequence
        auto sequence = dairlib::systems::trajectory_optimization::DirconModeSequence<double>(*plantPtr_);

        msh.addMode(
            (Eigen::Matrix<bool,1,4>() << true,  true,  true,  true).finished(),
            3,
            normal_,
            Eigen::Vector3d::Zero(),
            std::numeric_limits<double>::infinity(),
            1,
            1
            );

        auto [modeVector, toeEvals, toeEvalSets] = createSpiritModeSequence(*plantPtr_, msh);

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
                {0, 1, 2, 3}, {0, 1, 2}, 1.0);
            mode->SetKinPositionScale(
                {0, 1, 2, 3}, {0, 1, 2}, 200);
            }
            sequence.AddMode(mode.get());
        }
        ///Setup trajectory optimization
        auto trajopt = dairlib::systems::trajectory_optimization::Dircon<double>(sequence);
        // Set up Trajectory Optimization options
        trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(),
                                "Print file", "../snopt.out");
        trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(),
                                "Major iterations limit", 200000);
        trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(), "Iterations limit", 1000000);
        trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(),
                                "Major optimality tolerance",
                                tol);  // target optimality
        trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(), "Major feasibility tolerance", tol);
        trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(), "Verify level", 0);  // 0

        // Get position and velocity dictionaries
        auto positions_map =  dairlib::multibody::makeNameToPositionsMap(*plantPtr_);
        auto velocities_map = dairlib::multibody::makeNameToVelocitiesMap(*plantPtr_);

        dairlib::setSpiritJointLimits(*plantPtr_, trajopt);
        dairlib::setSpiritActuationLimits(*plantPtr_, trajopt);

        /// Setup all the optimization constraints
        int n_q = plantPtr_->num_positions();
        int n_v = plantPtr_->num_velocities();
        int n_u = plantPtr_->num_actuators();
        int n_x = n_q + n_v;

        int num_knotpoints = trajopt.N();
        for (int iKnot=0;iKnot<num_knotpoints;iKnot++){
            auto xi = trajopt.state(iKnot);
            trajopt.AddBoundingBoxConstraint(height_, height_, xi( positions_map.at("base_z")));
            for (int iVel = 0; iVel < n_v; iVel ++){
            trajopt.AddBoundingBoxConstraint(-0, 0, xi( n_q + iVel));
            }
        }

        // Four contacts so forces are 12 dimensional
        Eigen::VectorXd init_l_vec(12);
        // Initial guess
        init_l_vec << 0, 0, 3*9.81, 0, 0, 3*9.81, 0, 0, 3*9.81, 0, 0, 3*9.81; //gravity and mass distributed
        
        //Initialize force trajectories
        std::vector<Eigen::MatrixXd> init_x;
        std::vector<Eigen::MatrixXd> init_u;
        std::vector<Eigen::MatrixXd> init_l;
        std::vector<Eigen::MatrixXd> init_lc;
        std::vector<Eigen::MatrixXd> init_vc;
        std::vector<double> init_time;
        Eigen::VectorXd x_state(n_x);
        dairlib::nominalSpiritStand( *plantPtr_, x_state,  height_);
        double duration = 1;
        for (int iKnot = 0; iKnot < num_knotpoints; iKnot++) {
            init_time.push_back(iKnot*duration/(num_knotpoints-1));
            init_l.push_back(init_l_vec);
            init_lc.push_back(init_l_vec);
            init_vc.push_back(Eigen::VectorXd::Zero(12));
            init_x.push_back(x_state);
            init_u.push_back(Eigen::VectorXd::Zero(n_u));
        }

        auto init_x_traj = drake::trajectories::PiecewisePolynomial<double>::ZeroOrderHold(init_time, init_x);
        auto init_u_traj = drake::trajectories::PiecewisePolynomial<double>::ZeroOrderHold(init_time, init_u);
        auto init_l_traj = drake::trajectories::PiecewisePolynomial<double>::ZeroOrderHold(init_time,init_l);
        auto init_lc_traj = drake::trajectories::PiecewisePolynomial<double>::ZeroOrderHold(init_time,init_lc);
        auto init_vc_traj = drake::trajectories::PiecewisePolynomial<double>::ZeroOrderHold(init_time,init_vc);

            
        ///Setup the cost function
        const double R = 20;  // Cost on input effort
        auto u = trajopt.input();
        trajopt.AddRunningCost(u.transpose()*R*u);
        trajopt.SetInitialTrajectory(init_u_traj, init_x_traj);
        trajopt.SetInitialForceTrajectory(0, init_l_traj, init_lc_traj, init_vc_traj);

        std::cout<<"Get Optimized Stand"<<std::endl;

        /// Run the optimization using your initial guess
        auto start = std::chrono::high_resolution_clock::now();
        const auto result = Solve(trajopt, trajopt.initial_guess());
        auto finish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = finish - start;
        std::cout << "Solve time:" << elapsed.count() <<std::endl;
        std::cout << "Cost:" << result.get_optimal_cost() <<std::endl;
        std::cout << (result.is_success() ? "Stand Optimization Success" : "Stand Optimization Fail") << std::endl;
        Eigen::MatrixXd stateOut = trajopt.GetStateSamplesByMode(result,0);

        /// Save trajectory
        std::cout << "Outputting trajectories" << std::endl;


        dairlib::DirconTrajectory saved_traj(
            *plantPtr_, trajopt, result, "Jumping trajectory",
            "Decision variables and state/input trajectories "
            "for jumping");

        std::cout << "writing to file" << std::endl;
        saved_traj.WriteToFile(folder_ + filename_);

    
        return (stateOut.col(0)).head(n_q);

        }
}