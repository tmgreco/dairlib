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
using drake::solvers::VariableRefList;

namespace dairlib {
    
    using systems::trajectory_optimization::DirconModeSequence;
    using std::vector;
    using std::map;
    template <typename C, class Y> // C: confuguration struct
    class Behavior{
        public:
        
        virtual void addCost(
                    MultibodyPlant<Y>& plant, 
                    dairlib::systems::trajectory_optimization::Dircon<Y>& trajopt)=0;

        // addConstraints, adds constraints to the trajopt jump problem. See runSpiritJump for a description of the inputs
        virtual void config(C input_configuration) = 0; //Maybe it should load configuration directly from the path
        virtual void addConstraints(
                            MultibodyPlant<Y>& plant, 
                            dairlib::systems::trajectory_optimization::Dircon<Y>& trajopt
                            ) =0;


        virtual void loadOldTrajectory(std::string traj_dir)=0;

        /// runSpiritJump, runs a trajectory optimization problem for spirit jumping on flat ground
        virtual void run(MultibodyPlant<Y>& plant) = 0;

        



    protected:
        C configuration;

        std::vector<int> num_knot_points;
        double mu;

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
        
        /// getModeSequence, initializes the trajopt mode seqence for jump, see runSpiritJump for a def of inputs
        std::tuple<  std::vector<std::unique_ptr<dairlib::systems::trajectory_optimization::DirconMode<Y>>>,
                    std::vector<std::unique_ptr<multibody::WorldPointEvaluator<Y>>> ,
                    std::vector<std::unique_ptr<multibody::KinematicEvaluatorSet<Y>>>>
        getModeSequence(
                        MultibodyPlant<Y>& plant,
                        DirconModeSequence<Y>& sequence,
                        std::vector<std::string>& mode_vector){
        dairlib::ModeSequenceHelper msh;
        int counter=0;
        for (std::string mode_name : mode_vector){
            if (mode_name=="stance"){
            msh.addMode( // Stance
                (Eigen::Matrix<bool,1,4>() << true,  true,  true,  true).finished(), // contact bools
                num_knot_points[counter],  // number of knot points in the collocation
                Eigen::Vector3d::UnitZ(), // normal
                Eigen::Vector3d::Zero(),  // world offset
                mu //friction
            );
            }
            else if(mode_name=="flight"){
            msh.addMode( // Flight
            (Eigen::Matrix<bool,1,4>() << false, false, false, false).finished(), // contact bools
            num_knot_points[counter],  // number of knot points in the collocation
            Eigen::Vector3d::UnitZ(), // normal
            Eigen::Vector3d::Zero(),  // world offset
            mu //friction
            );
            }
            counter++;
        }
        
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
                {0, 1, 2, 3}, {0, 1, 2}, 1.0);
            mode->SetKinPositionScale(
                {0, 1, 2, 3}, {0, 1, 2}, 200);
            }
            sequence.AddMode(mode.get());
        }
        return {std::move(modeVector), std::move(toeEvals), std::move(toeEvalSets)};
        }

    };
} 

#endif