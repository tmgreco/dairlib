#include <memory>
#include <chrono>
#include <unistd.h>
#include <gflags/gflags.h>
#include <string.h>
#include <cmath>
#include <experimental/filesystem>
#include <Eigen/Geometry>

#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/ipopt_solver.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/multibody/parsing/parser.h"
#include <drake/multibody/inverse_kinematics/inverse_kinematics.h>
#include "drake/geometry/drake_visualizer.h"
#include "drake/solvers/solve.h"
#include "drake/solvers/decision_variable.h"
// #include "drake/math/orthonormal_basis.h"

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

#include "examples/Spirit/spirit_utils.h"
#include "examples/Spirit/spirit_optimal_stand.h"

DEFINE_double(duration, 1, "The stand duration");
DEFINE_double(standHeight, 0.23, "The standing height.");
DEFINE_double(foreAftDisplacement, 1, "The fore-aft displacement.");
DEFINE_double(apexGoal, 0.7, "Apex state goal");
DEFINE_double(inputCost, 5, "input cost scale.");
DEFINE_double(velocityCost, 10, "velocity cost scale.");
DEFINE_double(eps, 1e-2, "The wiggle room.");
DEFINE_double(tol, 1e-6, "Optimization Tolerance");
DEFINE_double(mu, 1.5, "coefficient of friction");
DEFINE_double(boxHeight, 0.3, "The height of the landing");
DEFINE_double(iterationAngResDeg, 5, "Angular Resolution for iterative optimization");

DEFINE_string(data_directory, "/home/feng/Downloads/dairlib/examples/Spirit/saved_trajectories/parkour/",
              "directory to save/read data");
DEFINE_string(distance_name, "iterativeBox45","name to describe distance");

DEFINE_bool(runAllOptimization, false, "rerun earlier optimizations?");
DEFINE_bool(skipInitialOptimization, false, "skip first optimizations?");
DEFINE_bool(minWork, false, "skip try to minimize work?");
DEFINE_bool(runIterative, true, "for angled runs, run multiple optimizations to approach");

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
namespace {

using systems::trajectory_optimization::DirconModeSequence;
using systems::trajectory_optimization::DirconMode;
using systems::trajectory_optimization::Dircon;
using systems::trajectory_optimization::KinematicConstraintType;

using std::vector;
using std::cout;
using std::endl;

/// Adds an offset constraint which allows the body to be anywhere on the normal vector
template <typename T>
void offsetConstraint(
        MultibodyPlant<T>& plant,
        dairlib::systems::trajectory_optimization::Dircon<T>& trajopt,
        const Eigen::Ref<const drake::solvers::VectorXDecisionVariable>& xb,
        Eigen::Vector3d normal,
        Eigen::Vector3d offset,
        double eps = 0.01
        ){
  // Get position and velocity dictionaries
  auto positions_map = multibody::makeNameToPositionsMap(plant);
  auto velocities_map = multibody::makeNameToVelocitiesMap(plant);
  Eigen::Vector3d nHat= normal/normal.norm();
  Eigen::Matrix3d rotMat = dairlib::normal2Rotation(nHat).matrix();
  std::cout << "Rot:\n" << rotMat << "\nNormal:\n" << nHat <<  "\nOffset:\n" << offset << std::endl;
  Eigen::Vector3d t1 = rotMat.col(0);
  Eigen::Vector3d t2 = rotMat.col(1);
  std::cout << "T1" << t1 <<std::endl;
  std::cout << "T2" << t2 <<std::endl;
  std::vector<int> posInds{positions_map.at("base_x"),positions_map.at("base_y"),positions_map.at("base_z")};
  double t1Toffset = t1.transpose()*offset;
  double t2Toffset = t2.transpose()*offset;
  std::cout << "pos1: " <<  posInds[0] <<std::endl;
  std::cout << "pos2: " <<  posInds[1] <<std::endl;
  std::cout << "pos3: " <<  posInds[2] <<std::endl;
  double upper1 = t1Toffset + eps;
  double lower1 = t1Toffset - eps;
  std::cout << "t1 offset +: " <<  upper1 << std::endl;
  std::cout << "t1 offset -: " <<  lower1 << std::endl;
  
  
  
  trajopt.AddLinearConstraint( t1(0) * xb(posInds[0]) + t1(1) * xb(posInds[1]) + t1(2) * xb(posInds[2])  <=  (t1Toffset));
  trajopt.AddLinearConstraint( t1(0) * xb(posInds[0]) + t1(1) * xb(posInds[1]) + t1(2) * xb(posInds[2])  >=  (t1Toffset));
  // trajopt.AddLinearConstraint( t2(0) * (xb(posInds[0])-offset(0)) + t2(1) * (xb(posInds[1])-offset(1)) + t2(2) * (xb(posInds[2])-offset(2))  <=  eps);
  // trajopt.AddLinearConstraint( t2(0) * (xb(posInds[0])-offset(0)) + t2(1) * (xb(posInds[1])-offset(1)) + t2(2) * (xb(posInds[2])-offset(2))  >= -eps);

}
/// addCost, adds the cost to the trajopt jump problem. See runSpiritBoxJump for a description of the inputs
template <typename T>
void addCost(MultibodyPlant<T>& plant,
             dairlib::systems::trajectory_optimization::Dircon<T>& trajopt,
             const double cost_actuation,
             const double cost_velocity,
             const double cost_work,
             const double work_constraint_scale = 1.0){


  auto velocities_map = multibody::makeNameToVelocitiesMap(plant);
  auto actuator_map = multibody::makeNameToActuatorsMap(plant);


  int n_q = plant.num_positions();
  int n_v = plant.num_velocities();
  auto u   = trajopt.input();
  auto x   = trajopt.state();

  // Setup the traditional cost function
  const double R = cost_actuation;  // Cost on input effort
  const MatrixXd Q = cost_velocity  * MatrixXd::Identity(n_v, n_v); // Cost on velocity

  trajopt.AddRunningCost( x.tail(n_v).transpose() * Q * x.tail(n_v) );
  trajopt.AddRunningCost( u.transpose()*R*u);

  for (int joint = 0; joint < 12; joint++){
    auto power_plus = trajopt.NewSequentialVariable(1, "joint_" + std::to_string(joint)+"_power_plus");
    auto power_minus = trajopt.NewSequentialVariable(1, "joint_" + std::to_string(joint)+"_power_minus");

    trajopt.AddRunningCost(cost_work * (power_plus + power_minus));

    for(int time_index = 0; time_index < trajopt.N(); time_index++){
      auto u_i   = trajopt.input(time_index);
      auto x_i   = trajopt.state(time_index);

      drake::symbolic::Variable actuation = u_i(actuator_map.at("motor_" + std::to_string(joint)));
      drake::symbolic::Variable velocity = x_i(n_q + velocities_map.at("joint_" + std::to_string(joint) +"dot"));
      drake::symbolic::Variable power_plus_i = trajopt.GetSequentialVariableAtIndex("joint_" + std::to_string(joint)+"_power_plus", time_index)[0];
      drake::symbolic::Variable power_minus_i = trajopt.GetSequentialVariableAtIndex("joint_" + std::to_string(joint)+"_power_minus", time_index)[0];


      if (cost_work > 0){
        trajopt.AddConstraint(actuation * velocity * work_constraint_scale == (power_plus_i - power_minus_i) * work_constraint_scale) ;
        trajopt.AddLinearConstraint(power_plus_i * work_constraint_scale >= 0);
        trajopt.AddLinearConstraint(power_minus_i * work_constraint_scale >= 0);
      }
      trajopt.SetInitialGuess(power_plus_i, 0);
      trajopt.SetInitialGuess(power_minus_i, 0);
    }
  }
}
// addConstraints, adds constraints to the trajopt jump problem. See runSpiritBoxJump for a description of the inputs
template <typename T>
void addConstraints(MultibodyPlant<T>& plant,
          dairlib::systems::trajectory_optimization::Dircon<T>& trajopt,
          dairlib::OptimalSpiritStand& initialStand,
          dairlib::OptimalSpiritStand& finalStand,          
          std::vector<std::tuple<Eigen::Vector3d,Eigen::Vector3d,double>> transitionSurfaces,//(normal,offset,mu)
          const double apex_height,
          const double initial_height,
          const bool lock_rotation,
          const bool lock_legs_apex,
          const bool force_symmetry,
          const bool use_nominal_stand,
          const bool stop_at_bottom,
          const double max_duration,
          const double eps){

  // Get position and velocity dictionaries
  auto positions_map = multibody::makeNameToPositionsMap(plant);
  auto velocities_map = multibody::makeNameToVelocitiesMap(plant);

  setSpiritJointLimits(plant, trajopt);
  setSpiritActuationLimits(plant, trajopt);
  int numJumps = transitionSurfaces.size() + 1;
  if (force_symmetry) {
    setSpiritSymmetry(plant, trajopt);
  }
  /// Setup all the optimization constraints
  int n_q = plant.num_positions();
  int n_v = plant.num_velocities();

  auto   x0  = trajopt.initial_state();
  auto   xlo = trajopt.state_vars(1, 0);
  // auto xapex = trajopt.state_vars(2, 0);
  auto   xtd = trajopt.state_vars(3, 0);
  auto   xf  = trajopt.final_state();

  // Add duration constraint, currently constrained not bounded
  trajopt.AddDurationBounds(0, max_duration);



  /// ***** Constrain the initial and final states. *****  
  // Constraining configuration  
  VectorXd standOffsetInit = initialStand.offset();
  VectorXd standOffsetFinal = finalStand.offset();
  VectorXd standJointsInit = initialStand.getJoints();
  VectorXd standJointsFinal = finalStand.getJoints();
  VectorXd standQuatInit = initialStand.getQuat();
  VectorXd standQuatFinal = finalStand.getQuat();
  // int numJoints = standJointsInit.size();
  // Init Standing XY Pos
  trajopt.AddBoundingBoxConstraint(
              standOffsetInit(0), 
              standOffsetInit(0), 
              x0(positions_map.at("base_x"))); 
  trajopt.AddBoundingBoxConstraint(
              standOffsetInit(1)-eps, 
              standOffsetInit(1)+eps, 
              x0(positions_map.at("base_y"))); 

  // Final Standing XY Pos
  trajopt.AddBoundingBoxConstraint(
              standOffsetFinal(0)-eps, 
              standOffsetFinal(0)+eps, 
              xf(positions_map.at("base_x")));
  trajopt.AddBoundingBoxConstraint(
              standOffsetFinal(1)-eps, 
              standOffsetFinal(1)+eps, 
              xf(positions_map.at("base_y")));
  
  // Joint constraints init and final
  trajopt.AddBoundingBoxConstraint(
              standJointsInit-Eigen::VectorXd::Constant(standJointsInit.size(), eps),
              standJointsInit+Eigen::VectorXd::Constant(standJointsInit.size(), eps),
              (x0.head(n_q)).tail(n_q-7));
  trajopt.AddBoundingBoxConstraint(
              standJointsFinal-Eigen::VectorXd::Constant(standJointsFinal.size(), eps),
              standJointsFinal+Eigen::VectorXd::Constant(standJointsFinal.size(), eps),
              (xf.head(n_q)).tail(n_q-7));

  // Body pose constraints at initial and final state
  trajopt.AddBoundingBoxConstraint(
        standQuatInit - Eigen::Vector4d::Constant(eps), 
        standQuatInit + Eigen::Vector4d::Constant(eps), 
        x0.head(4));
  trajopt.AddBoundingBoxConstraint(
        standQuatFinal - Eigen::Vector4d::Constant(eps), 
        standQuatFinal + Eigen::Vector4d::Constant(eps), 
        xf.head(4));

  // Initial and final velocity
  trajopt.AddBoundingBoxConstraint(
          VectorXd::Zero(n_v), 
          VectorXd::Zero(n_v), 
          x0.tail(n_v));
  trajopt.AddBoundingBoxConstraint(
          VectorXd::Zero(n_v), 
          VectorXd::Zero(n_v), 
          xf.tail(n_v));
  //  ****************************************************

  
  for (int iJump=0;iJump<numJumps;iJump++){
    // Apex height
    if(apex_height > 0){
      auto xapex = trajopt.state_vars(2 + iJump*3 , 0);
      trajopt.AddBoundingBoxConstraint(
            apex_height - eps, 
            apex_height + eps, 
            xapex(positions_map.at("base_z")) );
            
      trajopt.AddBoundingBoxConstraint(
            - eps, 
            + eps, 
            xapex(n_q + velocities_map.at("base_vy")) );
      trajopt.AddBoundingBoxConstraint(
            - eps, 
            + eps, 
            xapex(n_q + velocities_map.at("base_vz")) );
    }
    if(iJump != 0){
      std::cout << "Zero Velocity Bottoms activated" << std::endl;
      int interiorStanceModeIndex = iJump*3;
      int botKnotpoint = trajopt.mode_length(interiorStanceModeIndex)/2;
      auto xbot = trajopt.state_vars( interiorStanceModeIndex , botKnotpoint);
      Eigen::Vector3d sNormal;
      Eigen::Vector3d sOffset;
      std::tie(sNormal,sOffset,std::ignore) = transitionSurfaces[iJump-1];
      dairlib::offsetConstraint(
        plant, 
        trajopt, 
        xbot, 
        sNormal,//Eigen::Vector3d::UnitY()+Eigen::Vector3d::UnitZ(), 
        sOffset
        );

      // if(stop_at_bottom){
      //   trajopt.AddBoundingBoxConstraint(
      //           standOffsetFinal(0)-eps, 
      //           standOffsetFinal(0)+eps, 
      //           xbot(positions_map.at("base_x")));
      //   trajopt.AddBoundingBoxConstraint(
      //           VectorXd::Zero(n_v), 
      //           VectorXd::Zero(n_v), 
      //           xbot.tail(n_v));
      // }
    }

  }
  // The "leg angle" = (pi/2 - theta0)

  // double upperSet = 1;
  // double kneeSet = 2;

  // if(lock_legs_apex){
  //   //STATIC LEGS AT APEX
  //   trajopt.AddBoundingBoxConstraint(upperSet - eps, upperSet + eps, xapex(positions_map.at("joint_0") ) );
  //   trajopt.AddBoundingBoxConstraint(kneeSet - eps, kneeSet + eps, xapex(positions_map.at("joint_1") ) );

  //   trajopt.AddBoundingBoxConstraint(upperSet - eps, upperSet + eps, xapex(positions_map.at("joint_2") ) );
  //   trajopt.AddBoundingBoxConstraint(kneeSet - eps, kneeSet + eps, xapex(positions_map.at("joint_3") ) );

  //   trajopt.AddBoundingBoxConstraint(upperSet - eps, upperSet + eps, xapex(positions_map.at("joint_4") ) );
  //   trajopt.AddBoundingBoxConstraint(kneeSet - eps, kneeSet + eps, xapex(positions_map.at("joint_5") ) );

  //   trajopt.AddBoundingBoxConstraint(upperSet - eps, upperSet + eps, xapex(positions_map.at("joint_6") ) );
  //   trajopt.AddBoundingBoxConstraint(kneeSet - eps, kneeSet + eps, xapex(positions_map.at("joint_7") ) );

  //   trajopt.AddBoundingBoxConstraint(-0, 0, xapex( n_q + velocities_map.at("joint_0dot")));
  //   trajopt.AddBoundingBoxConstraint(-0, 0, xapex( n_q + velocities_map.at("joint_1dot")));
  //   trajopt.AddBoundingBoxConstraint(-0, 0, xapex( n_q + velocities_map.at("joint_2dot")));
  //   trajopt.AddBoundingBoxConstraint(-0, 0, xapex( n_q + velocities_map.at("joint_3dot")));
  //   trajopt.AddBoundingBoxConstraint(-0, 0, xapex( n_q + velocities_map.at("joint_4dot")));
  //   trajopt.AddBoundingBoxConstraint(-0, 0, xapex( n_q + velocities_map.at("joint_5dot")));
  //   trajopt.AddBoundingBoxConstraint(-0, 0, xapex( n_q + velocities_map.at("joint_6dot")));
  //   trajopt.AddBoundingBoxConstraint(-0, 0, xapex( n_q + velocities_map.at("joint_7dot")));
  //   trajopt.AddBoundingBoxConstraint(-0, 0, xapex( n_q + velocities_map.at("joint_8dot")));
  //   trajopt.AddBoundingBoxConstraint(-0, 0, xapex( n_q + velocities_map.at("joint_9dot")));
  //   trajopt.AddBoundingBoxConstraint(-0, 0, xapex( n_q + velocities_map.at("joint_10dot")));
  //   trajopt.AddBoundingBoxConstraint(-0, 0, xapex( n_q + velocities_map.at("joint_11dot")));
  // }

  for (int i = 0; i < trajopt.N(); i++){
    auto xi = trajopt.state(i);
    
    // Height
    trajopt.AddBoundingBoxConstraint( 0.15, 5, xi( positions_map.at("base_z")));
    
    double leg_angle_boundary_distance = M_PI_4;//Set the angular distance from horizontal that the toes are allowed 
    
    trajopt.AddLinearConstraint( xi(positions_map.at("joint_1"))/2 - xi(positions_map.at("joint_0")) <=   M_PI_2 - leg_angle_boundary_distance );
    trajopt.AddLinearConstraint( xi(positions_map.at("joint_1"))/2 - xi(positions_map.at("joint_0")) >= - M_PI_2 + leg_angle_boundary_distance );
    
    trajopt.AddLinearConstraint( xi(positions_map.at("joint_3"))/2 - xi(positions_map.at("joint_2")) <=   M_PI_2 - leg_angle_boundary_distance );
    trajopt.AddLinearConstraint( xi(positions_map.at("joint_3"))/2 - xi(positions_map.at("joint_2")) >= - M_PI_2 + leg_angle_boundary_distance );
    
    trajopt.AddLinearConstraint( xi(positions_map.at("joint_5"))/2 - xi(positions_map.at("joint_4")) <=   M_PI_2 - leg_angle_boundary_distance );
    trajopt.AddLinearConstraint( xi(positions_map.at("joint_5"))/2 - xi(positions_map.at("joint_4")) >= - M_PI_2 + leg_angle_boundary_distance );
    
    trajopt.AddLinearConstraint( xi(positions_map.at("joint_7"))/2 - xi(positions_map.at("joint_6")) <=   M_PI_2 - leg_angle_boundary_distance );
    trajopt.AddLinearConstraint( xi(positions_map.at("joint_7"))/2 - xi(positions_map.at("joint_6")) >= - M_PI_2 + leg_angle_boundary_distance );
    
  }
  for (int iMode = 0; iMode<trajopt.num_modes(); iMode++){
    if(iMode%3){
      for(int iKnot = 0; iKnot< trajopt.mode_length(iMode);iKnot++){
        auto xi = trajopt.state_vars(iMode,iKnot);
        if (lock_rotation)
        {
          trajopt.AddBoundingBoxConstraint(1 - eps, 1 + eps, xi(positions_map.at("base_qw")));
          trajopt.AddBoundingBoxConstraint(0 - eps, 0 + eps, xi(positions_map.at("base_qx")));
          trajopt.AddBoundingBoxConstraint(0 - eps, 0 + eps, xi(positions_map.at("base_qy")));
          trajopt.AddBoundingBoxConstraint(0 - eps, 0 + eps, xi(positions_map.at("base_qz")));
        }
      }
    }
  }
}


void concatVectorTraj(
      vector<PiecewisePolynomial<double>>& traj, 
      vector<PiecewisePolynomial<double>> otherTraj, 
      bool mergeInnerMode){
        

  double finalTime = traj.back().end_time();
  double startTime = otherTraj.front().start_time();
    std::cout<<"***  N:"<< otherTraj.size() << "**F:" <<finalTime<<" **S: "<<startTime<<std::endl;
  for (auto& otherTrajI : otherTraj){
    std::cout<<otherTrajI.start_time()<<std::endl;
    otherTrajI.shiftRight(finalTime-startTime);
    if(mergeInnerMode){
      mergeInnerMode = false;
      PiecewisePolynomial<double> lastMode = traj.back();
      traj.pop_back();
      lastMode.ConcatenateInTime(otherTrajI);
      traj.push_back(lastMode);
    }else{
      traj.push_back(otherTrajI);
      // std::cout<<finalTime<<std::endl;
    }
  }
}

template <typename T>
void badSpiritMultiJumpFromSimple(MultibodyPlant<T>& plant,
                    PiecewisePolynomial<double>& x_traj,
                    PiecewisePolynomial<double>& u_traj,
                    vector<PiecewisePolynomial<double>>& l_traj,
                    vector<PiecewisePolynomial<double>>& lc_traj,
                    vector<PiecewisePolynomial<double>>& vc_traj,
                    int nJumps=1){

  std::string file_name_in = FLAGS_data_directory+"simple_jump";
  dairlib::DirconTrajectory loaded_traj(file_name_in);

  
  // x_traj.clear(); 
  // u_traj.clear(); 
  // l_traj.clear(); 
  // lc_traj.clear(); 
  // vc_traj.clear(); 

  x_traj = loaded_traj.ReconstructStateTrajectory();
  u_traj = loaded_traj.ReconstructInputTrajectory();
  l_traj = loaded_traj.ReconstructLambdaTrajectory();
  lc_traj = loaded_traj.ReconstructLambdaCTrajectory();
  vc_traj = loaded_traj.ReconstructGammaCTrajectory();
  
  for(int iJump = 1; iJump < nJumps; iJump++){
    PiecewisePolynomial<double> x_traj_i = loaded_traj.ReconstructStateTrajectory();
    PiecewisePolynomial<double> u_traj_i = loaded_traj.ReconstructInputTrajectory();
    vector<PiecewisePolynomial<double>> l_traj_i = loaded_traj.ReconstructLambdaTrajectory();
    vector<PiecewisePolynomial<double>> lc_traj_i = loaded_traj.ReconstructLambdaCTrajectory();
    vector<PiecewisePolynomial<double>> vc_traj_i = loaded_traj.ReconstructGammaCTrajectory();
    
    std::cout<<"test"<< std::endl;
    x_traj_i.shiftRight(x_traj.end_time());
    u_traj_i.shiftRight(u_traj.end_time());
    std::cout<<"test"<< std::endl;

    x_traj.ConcatenateInTime(x_traj_i);
    u_traj.ConcatenateInTime(u_traj_i);
    std::cout<<"test"<< std::endl;

    concatVectorTraj(  l_traj,  l_traj_i , true);
    concatVectorTraj( lc_traj, lc_traj_i , true);
    concatVectorTraj( vc_traj, vc_traj_i , true);
  }
  /// Run animation of the final trajectory
  std::string full_name = dairlib::FindResourceOrThrow("examples/Spirit/spirit_drake.urdf");
  drake::systems::DiagramBuilder<double> builder;
  auto plant_vis = std::make_unique<MultibodyPlant<double>>(0.0);
  auto scene_graph_ptr = std::make_unique<SceneGraph<double>>();
  Parser parser_vis(plant_vis.get(), scene_graph_ptr.get());
  parser_vis.AddModelFromFile(full_name);
  // Add the transional surfaces
  int counter = 0;
  const drake::Vector4<double> orange(1.0, 0.55, 0.0, 1.0);
  
  plant_vis->Finalize();
  SceneGraph<double>& scene_graph = *builder.AddSystem(std::move(scene_graph_ptr));

  multibody::connectTrajectoryVisualizer(plant_vis.get(),
      &builder, &scene_graph, x_traj);
  auto diagram = builder.Build();
  std::cout << "animating 1 times." << std::endl;
  for (int i = 0; i <1; i++ ) {
    drake::systems::Simulator<double> simulator(*diagram);
    simulator.set_target_realtime_rate(0.25);
    simulator.Initialize();
    simulator.AdvanceTo(x_traj.end_time());
    sleep(2);
  }
}


/// badSpiritJump, generates a bad initial guess for the spirit jump traj opt
/// \param plant: robot model
/// \param x_traj[out]: initial and solution state trajectory
/// \param u_traj[out]: initial and solution control trajectory
/// \param l_traj[out]: initial and solution contact force trajectory
/// \param lc_traj[out]: initial and solution contact force slack variable trajectory
/// \param vc_traj[out]: initial and solution contact velocity slack variable trajectory
template <typename T>
void badSpiritMultiJump(MultibodyPlant<T>& plant,
                    PiecewisePolynomial<double>& x_traj,
                    PiecewisePolynomial<double>& u_traj,
                    vector<PiecewisePolynomial<double>>& l_traj,
                    vector<PiecewisePolynomial<double>>& lc_traj,
                    vector<PiecewisePolynomial<double>>& vc_traj,
                    double apexHeight,
                    int nJumps=1){

  int nu = plant.num_actuators();
  int nx = plant.num_positions() + plant.num_velocities();
  int nq = plant.num_positions();
  int nv = plant.num_velocities();
  int N = 21; // number of timesteps per jump

  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(nx);

  std::vector<MatrixXd> init_x;
  std::vector<MatrixXd> init_u;
  l_traj.clear();
  lc_traj.clear();
  vc_traj.clear();

  // Initialize state trajectory
  std::vector<double> init_time;

  VectorXd xInit(nx);
  VectorXd xMid(nx);
  VectorXd xState(nx);
  xInit = Eigen::VectorXd::Zero(nx);
  xMid = Eigen::VectorXd::Zero(nx);
  xState = Eigen::VectorXd::Zero(nx);

  auto positions_map = dairlib::multibody::makeNameToPositionsMap(plant);
  auto velocities_map = dairlib::multibody::makeNameToVelocitiesMap(plant);
  int num_joints = 12;

  // Print joint dictionary
  std::cout<<"**********************Joints***********************"<<std::endl;
  for (auto const& element : positions_map)
    std::cout << element.first << " = " << element.second << std::endl;
  for (auto const& element : velocities_map)
    std::cout << element.first << " = " << element.second << std::endl;
  std::cout<<"***************************************************"<<std::endl;
  double initHeight = 0.16;
  double nominalLegExtensionHeight = 0.35;
  double g = 9.81;
  dairlib::nominalSpiritStand( plant, xInit,  initHeight); //Update xInit
  dairlib::nominalSpiritStand( plant, xMid,  nominalLegExtensionHeight); //Update xMid

  xMid(positions_map.at("base_z"))=apexHeight;

  VectorXd deltaX(nx);
  VectorXd averageV(nx);
  double duration2Apex = sqrt(2*(apexHeight-initHeight)/g);
  deltaX = xMid-xInit;
  averageV = deltaX / (duration2Apex * 2);
  xInit.tail(nv-3) = (averageV.head(nq)).tail(nq-4); //Ignoring Orientation make velocity the average
  if(apexHeight<=initHeight){
    // Current bad jump cant handle this 
    std::cout<<"WARNING: CHOSEN APEX NOT COMPATIBLE WITH THIS FUNCTION"<<std::endl;
  }
  



    // Initial pose
  double dt = 2 * duration2Apex/(N+1);

  double time = 0;
  for (int iJump = 0; iJump<nJumps;iJump++){  
    bool hasSwitchDirection = false;
    xState = xInit;
    xState(nq + velocities_map.at("base_vz")) = g*duration2Apex;

    for (int i = 0; i < N; i++) {
      time=time+dt; // calculate iteration's time
      init_time.push_back(time);

      // Switch the direction of the stand to go back to the initial state (not actually properly periodic initial)
      if ( i >= (N-1)/2 && !hasSwitchDirection){
        hasSwitchDirection = true;
        double tempdz = xState(nq + velocities_map.at("base_vz"));
        xState.tail(nv) = -xInit.tail(nv);
        xState(nq + velocities_map.at("base_vz")) = tempdz;
      }

      // Integrate the positions based on constant velocity  for joints and xyz
      for (int j = 0; j < num_joints; j++){
        xState(positions_map.at("joint_" + std::to_string(j))) =
            xState(positions_map.at("joint_" + std::to_string(j))) + xState(nq + velocities_map.at("joint_" + std::to_string(j)+"dot" )) * dt;
      }
      xState(positions_map.at("base_x")) =
          xState(positions_map.at("base_x")) + xState(nq + velocities_map.at("base_vx")) * dt;

      xState(positions_map.at("base_y")) =
          xState(positions_map.at("base_y")) + xState(nq + velocities_map.at("base_vy")) * dt;

      xState(positions_map.at("base_z")) =
          xState(positions_map.at("base_z")) + xState(nq + velocities_map.at("base_vz")) * dt - (g / 2) * dt * dt ;
      xState(nq + velocities_map.at("base_vz")) = xState(nq + velocities_map.at("base_vz")) - g * dt;
      // Save timestep state into matrix
      init_x.push_back(xState);
      init_u.push_back(Eigen::VectorXd::Zero(nu));
    }
  }
  // Make matrix into trajectory
  x_traj = PiecewisePolynomial<double>::ZeroOrderHold(init_time, init_x);
  u_traj = PiecewisePolynomial<double>::ZeroOrderHold(init_time, init_u);
  // for (auto& init_xi:init_x){
  //   std::cout<<(init_xi).transpose()<<std::endl;
  // }

  
  // std::string file_name_in = "/home/jdcap/dairlib/examples/Spirit/saved_trajectories/simple_jump";
  // std::cout<<"Loading decision var from file." <<std::endl;
  // dairlib::DirconTrajectory loaded_traj(file_name_in);



  // l_traj = loaded_traj.ReconstructLambdaTrajectory();
  // lc_traj = loaded_traj.ReconstructLambdaCTrajectory();
  // vc_traj = loaded_traj.ReconstructGammaCTrajectory();
  
  
  // Four contacts so forces are 12 dimensional
  Eigen::VectorXd init_l_vec(12);
  // Initial guess
  init_l_vec << 0, 0, 3*9.81, 0, 0, 3*9.81, 0, 0, 3*9.81, 0, 0, 3*9.81; //gravity and mass distributed

  //Initialize force trajectories

  //Stance
  std::vector<MatrixXd> init_l_j;
  std::vector<MatrixXd> init_lc_j;
  std::vector<MatrixXd> init_vc_j;
  std::vector<double> init_time_j;
  for (int i = 0; i < N; i++) {
    init_time_j.push_back(i * 2 * duration2Apex / (N - 1));
    init_l_j.push_back(init_l_vec);
    init_lc_j.push_back(init_l_vec);
    init_vc_j.push_back(VectorXd::Zero(12));
  }

  auto init_l_traj_j = PiecewisePolynomial<double>::ZeroOrderHold(init_time_j,init_l_j);
  auto init_lc_traj_j = PiecewisePolynomial<double>::ZeroOrderHold(init_time_j,init_lc_j);
  auto init_vc_traj_j = PiecewisePolynomial<double>::ZeroOrderHold(init_time_j,init_vc_j);

  l_traj.push_back(init_l_traj_j);
  lc_traj.push_back(init_lc_traj_j);
  vc_traj.push_back(init_vc_traj_j);
  for (int iJump = 0; iJump<nJumps;iJump++){
    // Flight
    init_l_j.clear();
    init_lc_j.clear();
    init_vc_j.clear();
    init_time_j.clear();
    for (int i = 0; i < N; i++) {
      init_time_j.push_back(i * 2 * duration2Apex / (N - 1));
      init_l_j.push_back(VectorXd::Zero(12));
      init_lc_j.push_back(VectorXd::Zero(12));
      init_vc_j.push_back(VectorXd::Zero(12));
    }

    init_l_traj_j = PiecewisePolynomial<double>::ZeroOrderHold(init_time_j,init_l_j);
    init_lc_traj_j = PiecewisePolynomial<double>::ZeroOrderHold(init_time_j,init_lc_j);
    init_vc_traj_j = PiecewisePolynomial<double>::ZeroOrderHold(init_time_j,init_vc_j);

    l_traj.push_back(init_l_traj_j);
    lc_traj.push_back(init_lc_traj_j);
    vc_traj.push_back(init_vc_traj_j);
    l_traj.push_back(init_l_traj_j);
    lc_traj.push_back(init_lc_traj_j);
    vc_traj.push_back(init_vc_traj_j);

    // Stance
    init_l_j.clear();
    init_lc_j.clear();
    init_vc_j.clear();
    init_time_j.clear();
    for (int i = 0; i < N; i++) {
      init_time_j.push_back(i * 2 * duration2Apex / (N - 1));
      init_l_j.push_back(init_l_vec);
      init_lc_j.push_back(init_l_vec);
      init_vc_j.push_back(VectorXd::Zero(12));
    }

    init_l_traj_j = PiecewisePolynomial<double>::ZeroOrderHold(init_time_j,init_l_j);
    init_lc_traj_j = PiecewisePolynomial<double>::ZeroOrderHold(init_time_j,init_lc_j);
    init_vc_traj_j = PiecewisePolynomial<double>::ZeroOrderHold(init_time_j,init_vc_j);

    l_traj.push_back(init_l_traj_j);
    lc_traj.push_back(init_lc_traj_j);
    vc_traj.push_back(init_vc_traj_j);
  }
}

/// runSpiritBoxJump, runs a trajectory optimization problem for spirit jumping on flat ground
/// \param plant_ptr: robot model
/// \param plant_double_ptr: model used for animation
/// \param scene_graph_ptr: scene graph
/// \param x_traj[in, out]: initial and solution state trajectory
/// \param u_traj[in, out]: initial and solution control trajectory
/// \param l_traj[in, out]: initial and solution contact force trajectory
/// \param lc_traj[in, out]: initial and solution contact force slack variable trajectory
/// \param vc_traj[in, out]: initial and solution contact velocity slack variable trajectory
/// \param initialStand: OptimalSpiritStand obj of the inital state
/// \param finalStand: OptimalSpiritStand obj of the final state
/// \param animate: true if solution should be animated, false otherwise
/// \param num_knot_points: number of knot points used for each mode (vector)
/// \param apex_height: apex height of the jump, if 0, do not enforce and apex height
/// \param initial_height: initial and final height of the jump
/// \param fore_aft_displacement: fore-aft displacemnt after jump
/// \param lock_rotation: true if rotation is constrained at all knot points, false if just initial and final state
/// \param lock_legs_apex if true, legs have fixed pose at apex
/// \param force_symmetry forces saggital plane symmetry {todo} make it so it does not overdefine initial and final state
/// \param use_nominal_stand if true sets initial and final state to be a nominal stand
/// \param max_duration: maximum time allowed for jump
/// \param cost_actuation: Cost on actuation
/// \param cost_velocity: Cost on state velocity
/// \param cost_work: Cost on work
/// \param mu: coefficient of friction
/// \param eps: the tolerance for position constraints
/// \param tol: optimization solver constraint and optimality tolerence
/// \param work_constraint_scale: scale for the constraints for the power calculation
/// \param file_name_out: if empty, file is unsaved, if not empty saves the trajectory in the directory
template <typename T>
void runSpiritParkour(
      MultibodyPlant<T>& plant,
      PiecewisePolynomial<double>& x_traj,
      PiecewisePolynomial<double>& u_traj,
      vector<PiecewisePolynomial<double>>& l_traj,
      vector<PiecewisePolynomial<double>>& lc_traj,
      vector<PiecewisePolynomial<double>>& vc_traj,
      dairlib::OptimalSpiritStand& initialStand,
      dairlib::OptimalSpiritStand& finalStand,
      std::vector<std::tuple<Eigen::Vector3d,Eigen::Vector3d,double>> transitionSurfaces,//(normal,offset,mu)
      double nKnotpoints_flight,
      double nKnotpoints_stances,
      const double max_duration,
      const bool animate,
      const double apex_height,
      const bool lock_rotation,
      const bool lock_legs_apex,
      const bool force_symmetry,
      const bool use_nominal_stand,
      const double cost_actuation,
      const double cost_velocity,
      const double cost_work,
      const double eps,
      const double tol,
      const double work_constraint_scale,
      const std::string& file_name_out,
      const std::string& file_name_in= ""
    ) {

  // Setup mode sequence
  auto sequence = DirconModeSequence<T>(plant);
  dairlib::ModeSequenceHelper msh;
  msh.addMode( // Stance
      (Eigen::Matrix<bool,1,4>() << true,  true,  true,  true).finished(), // contact bools
      nKnotpoints_stances,  // number of knot points in the collocation
      initialStand.normal(), // normal
      initialStand.offset(),  // world offset
      1.5  //   initialStand.mu() //friction
  );
  msh.addFlight(nKnotpoints_flight);
  msh.addFlight(nKnotpoints_flight);


  for (auto& surface : transitionSurfaces){
    Eigen::Vector3d sNormal;
    Eigen::Vector3d sOffset;
    double sMu;
    std::tie(sNormal,sOffset,sMu) = surface;
    msh.addMode( // Stance
        (Eigen::Matrix<bool,1,4>() << true,  true,  true,  true).finished(), // contact bools
        nKnotpoints_stances,  // number of knot points in the collocation
        sNormal, // normal
        sOffset,  // world offset
        sMu //friction
    );
    msh.addFlight(nKnotpoints_flight);
    msh.addFlight(nKnotpoints_flight);
  }


  msh.addMode( // Stance
      (Eigen::Matrix<bool,1,4>() << true,  true,  true,  true).finished(), // contact bools
      nKnotpoints_stances,  // number of knot points in the collocation
      finalStand.normal(), // normal
      finalStand.offset(),  // world offset
      1.5  //finalStand.mu() //friction
  );

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

  ///Setup trajectory optimization
  auto trajopt = Dircon<T>(sequence);
  // Set up Trajectory Optimization options
  trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(),
                           "Print file", "../snopt.out");
  trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(),
                           "Major iterations limit", 200000);
  trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(), "Iterations limit", 200000);
  trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(),
                           "Major optimality tolerance",
                           tol);  // target optimality
  trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(), "Major feasibility tolerance", tol);
  trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(), "Verify level",
                           0);  // 0

  // Setting up cost
  addCost(plant, trajopt, cost_actuation, cost_velocity, cost_work, work_constraint_scale);
  // Setting up constraints
  bool stop_at_bottom = true;
  addConstraints(plant, trajopt, initialStand, finalStand, transitionSurfaces, apex_height, initialStand.height(), lock_rotation,
                    lock_legs_apex, force_symmetry, use_nominal_stand, stop_at_bottom, max_duration, eps);

  // Initialize the trajectory control state and forces
  if (file_name_in.empty()){
    trajopt.drake::systems::trajectory_optimization::MultipleShooting::
        SetInitialTrajectory(u_traj, x_traj);
    for (int j = 0; j < sequence.num_modes(); j++) {
      trajopt.SetInitialForceTrajectory(j, l_traj[j], lc_traj[j], vc_traj[j], l_traj[j].start_time());
    }
  }else{
    std::cout<<"Loading decision var from file." <<std::endl;
    dairlib::DirconTrajectory loaded_traj(file_name_in);
    trajopt.SetInitialGuessForAllVariables(loaded_traj.GetDecisionVariables());
  }


  /// Setup the visualization during the optimization
  int num_ghosts = 3;// Number of ghosts in visualization. NOTE: there are limitations on number of ghosts based on modes and knotpoints
  std::vector<unsigned int> visualizer_poses; // Ghosts for visualizing during optimization
  for(int i = 0; i < sequence.num_modes(); i++){
      visualizer_poses.push_back(num_ghosts); 
  }
  trajopt.CreateVisualizationCallback(
      dairlib::FindResourceOrThrow("examples/Spirit/spirit_drake.urdf"),
      visualizer_poses, 0.2); // setup which URDF, how many poses, and alpha transparency 

  
  /// Run the optimization using your initial guess
  auto start = std::chrono::high_resolution_clock::now();
  const auto result = Solve(trajopt, trajopt.initial_guess());
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  std::cout << "Solve time:" << elapsed.count() <<std::endl;
  std::cout << "Cost:" << result.get_optimal_cost() <<std::endl;
  std::cout << (result.is_success() ? "Optimization Success" : "Optimization Fail") << std::endl;

  /// Save trajectory
  std::cout << "Outputting trajectories" << std::endl;


  if(!file_name_out.empty()){
    dairlib::DirconTrajectory saved_traj(
        plant, trajopt, result, "Jumping trajectory",
        "Decision variables and state/input trajectories "
        "for jumping");

    std::cout << "writing to file" << std::endl;
    saved_traj.WriteToFile(file_name_out);

    dairlib::DirconTrajectory old_traj(file_name_out);
    x_traj = old_traj.ReconstructStateTrajectory();
    u_traj = old_traj.ReconstructInputTrajectory();
    l_traj = old_traj.ReconstructLambdaTrajectory();
    lc_traj = old_traj.ReconstructLambdaCTrajectory();
    vc_traj = old_traj.ReconstructGammaCTrajectory();
  } else{
    std::cout << "warning no file name provided, will not be able to return full solution" << std::endl;
    x_traj  = trajopt.ReconstructStateTrajectory(result);
    u_traj  = trajopt.ReconstructInputTrajectory(result);
    l_traj  = trajopt.ReconstructLambdaTrajectory(result);
  }

  if(animate){
    /// Run animation of the final trajectory
    std::string full_name = dairlib::FindResourceOrThrow("examples/Spirit/spirit_drake.urdf");
    drake::systems::DiagramBuilder<double> builder;
    auto plant_vis = std::make_unique<MultibodyPlant<double>>(0.0);
    auto scene_graph_ptr = std::make_unique<SceneGraph<double>>();
    Parser parser_vis(plant_vis.get(), scene_graph_ptr.get());
    parser_vis.AddModelFromFile(full_name);
    // Add the transional surfaces
    int counter = 0;
    const drake::Vector4<double> orange(1.0, 0.55, 0.0, 1.0);
    for (auto& surface : transitionSurfaces){
      Eigen::Vector3d sNormal;
      Eigen::Vector3d sOffset;
      std::tie(sNormal,sOffset,std::ignore) = surface;
      counter++;
      dairlib::visualizeSurface(
        plant_vis.get(),
        sNormal,
        sOffset,
        0.5,
        0.5,
        0.1,orange,"surface"+std::to_string(counter));
    }
      
    // Add the box surface to the final animation
    dairlib::visualizeSurface(
        plant_vis.get(),
        finalStand.normal(),
        finalStand.offset(),
        0.5,
        0.5,
        0.1,orange,"final");

    plant_vis->Finalize();
    SceneGraph<double>& scene_graph = *builder.AddSystem(std::move(scene_graph_ptr));

    const drake::trajectories::PiecewisePolynomial<double> pp_xtraj =
        trajopt.ReconstructStateTrajectory(result);
    multibody::connectTrajectoryVisualizer(plant_vis.get(),
        &builder, &scene_graph, pp_xtraj);
    auto diagram = builder.Build();
    std::cout << "animating, kill to end." << std::endl;
    while (true) {
      drake::systems::Simulator<double> simulator(*diagram);
      simulator.set_target_realtime_rate(0.25);
      simulator.Initialize();
      simulator.AdvanceTo(pp_xtraj.end_time());
      sleep(2);
    }
  }
}
}  //namespace
}  //dairlib namespace

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  std::srand(time(0));  // Initialize random number generator.


  auto plant = std::make_unique<MultibodyPlant<double>>(0.0);
  Parser parser(plant.get());
  std::string full_name = dairlib::FindResourceOrThrow("examples/Spirit/spirit_drake.urdf");
  parser.AddModelFromFile(full_name);

  plant->mutable_gravity_field().set_gravity_vector(-9.81 * Eigen::Vector3d::UnitZ());
  plant->Finalize();

  PiecewisePolynomial<double> x_traj;
  PiecewisePolynomial<double> u_traj;
  std::vector<PiecewisePolynomial<double>> l_traj;
  std::vector<PiecewisePolynomial<double>> lc_traj;
  std::vector<PiecewisePolynomial<double>> vc_traj;
  

  bool lock_rotation_flight  = true;
  bool lock_legs_apex = true;
  bool force_symmetry  = true;
  bool use_nominal_stand = true;
  bool animate = true;
  dairlib::OptimalSpiritStand initialStand(plant.get(), FLAGS_standHeight, Eigen::Vector3d::UnitZ(), Eigen::Vector3d::Zero(), false);
  // dairlib::OptimalSpiritStand   finalStand(plant.get(), FLAGS_standHeight, normal, offset, false)/;
  if(! FLAGS_skipInitialOptimization){
      std::cout<<"Running initial optimization"<<std::endl;
      std::vector<std::tuple<Eigen::Vector3d,Eigen::Vector3d,double>> inPlaceSurface = 
        {std::make_tuple(Eigen::Vector3d::UnitZ(),Eigen::Vector3d::Zero(), std::numeric_limits<double>::infinity())};
      
      // std::vector<std::tuple<Eigen::Vector3d,Eigen::Vector3d,double>> inPlaceSurface = 
      //     {};
      int nJumps = inPlaceSurface.size() +1;
      dairlib::badSpiritMultiJumpFromSimple(*plant, x_traj, u_traj, l_traj, lc_traj, vc_traj, nJumps);
      
      /// Initial Jump inplace for useful contact force profiles
      /// Jump inplace need the transitional surface 
      int nKnotpoints_flight = 7;
      int nKnotpoints_stances = 7;
      double max_duration = 2;
      double apex_height = .5;
      double initial_height = FLAGS_standHeight;
      double cost_actuation = 3;
      double cost_velocity = 15;
      double cost_work = 0;
      double tol = 1e-1;
      dairlib::runSpiritParkour(
            *plant,
            x_traj,
            u_traj,
            l_traj,
            lc_traj,
            vc_traj,
            initialStand,
            initialStand,
            inPlaceSurface,
            nKnotpoints_flight,
            nKnotpoints_stances,
            max_duration,
            animate,
            apex_height,
            lock_rotation_flight,
            lock_legs_apex,
            not force_symmetry,
            use_nominal_stand,
            cost_actuation,
            cost_velocity,
            cost_work,
            FLAGS_eps,
            tol,
            0,
            FLAGS_data_directory+"simple_parkour_2_1"
    );
  }
  std::vector<std::tuple<Eigen::Vector3d,Eigen::Vector3d,double>> middleSurface = 
    {std::make_tuple(Eigen::Vector3d::UnitZ(),Eigen::Vector3d::UnitX()*0.125, std::numeric_limits<double>::infinity())};
    dairlib::OptimalSpiritStand   finalStand(plant.get(), FLAGS_standHeight, Eigen::Vector3d::UnitZ(), 0.25*Eigen::Vector3d::UnitX(), false);
   
      int nJumps = middleSurface.size() +1;
      
      /// Initial Jump inplace for useful contact force profiles
      /// Jump inplace need the transitional surface 
      int nKnotpoints_flight = 7;
      int nKnotpoints_stances = 7;
      double max_duration = 2;
      double apex_height = .5;
      double initial_height = FLAGS_standHeight;
      double cost_actuation = 3;
      double cost_velocity = 10;
      double cost_work = 0;
      double tol = 1e-3;
      dairlib::runSpiritParkour(
            *plant,
            x_traj,
            u_traj,
            l_traj,
            lc_traj,
            vc_traj,
            initialStand,
            finalStand,
            middleSurface,
            nKnotpoints_flight,
            nKnotpoints_stances,
            max_duration,
            animate,
            apex_height,
            not lock_rotation_flight,
            lock_legs_apex,
            not force_symmetry,
            use_nominal_stand,
            cost_actuation,
            cost_velocity,
            cost_work,
            FLAGS_eps,
            tol,
            0,
            FLAGS_data_directory+"simple_parkour_2_2",
            FLAGS_data_directory+"simple_parkour_2_1"

    );

//// DEBUG   DEBUG   DEBUG   DEBUG   DEBUG   DEBUG   DEBUG
//// DEBUG   DEBUG   DEBUG   DEBUG   DEBUG   DEBUG   DEBUG
//// DEBUG   DEBUG   DEBUG   DEBUG   DEBUG   DEBUG   DEBUG
//// DEBUG   DEBUG   DEBUG   DEBUG   DEBUG   DEBUG   DEBUG
//// DEBUG   DEBUG   DEBUG   DEBUG   DEBUG   DEBUG   DEBUG
//// DEBUG   DEBUG   DEBUG   DEBUG   DEBUG   DEBUG   DEBUG

  // /// Run animation of the final trajectory
  drake::systems::DiagramBuilder<double> builder;
  auto plant_vis = std::make_unique<MultibodyPlant<double>>(0.0);
  auto scene_graph_ptr = std::make_unique<SceneGraph<double>>();
  Parser parser_vis(plant_vis.get(), scene_graph_ptr.get());
  parser_vis.AddModelFromFile(full_name);
  plant_vis->Finalize();
  dairlib::runAnimate(
    std::move(plant),
    plant_vis.get(),
    std::move(scene_graph_ptr),
    x_traj,
    .25
    ) ;
  


  // Eigen::Matrix<double, 3, 1> initialPos;
  // Eigen::Matrix<double, 3, 1>   finalPos;
  // initialPos<<0,0,0;
  // finalPos<<1,1,0;
  // double apex = 5;
  // int N = 5;
  // std::pair< std::vector<Eigen::Matrix<double, 7, 1>>, Eigen::Matrix<double, 7, 1>> ballastic = dairlib::calculateBallistic(initialPos,finalPos,N,apex);
  // std::cout<<ballastic.second<<std::endl;
  // for (int i; i<N; i++){
  //   std::cout<<"Ballastic"<<((ballastic.first)[i]).transpose()<<std::endl;
  // }

  
  // Eigen::Matrix<double, 4, 1> initQuat;
  // Eigen::Matrix<double, 4, 1>  finalQuat;
  // initQuat<<1,0,0,0;
  // finalQuat<<1,0,1,0;
  // finalQuat = finalQuat/finalQuat.norm();
  // std::vector<Eigen::Matrix<double,8,1>>quatstates = dairlib::calculateBallisticRotation( initQuat, finalQuat, 5, (ballastic.first).back()(1) );
  
  // for (int i; i<N; i++){
  //   std::cout<<"Rotations"<<(quatstates[i]).transpose()<<std::endl;
  // }
}