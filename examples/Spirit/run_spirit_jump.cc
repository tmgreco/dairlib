#include <memory>
#include <chrono>
#include <unistd.h>
#include <gflags/gflags.h>
#include <string.h>
#include <cmath>
#include <experimental/filesystem>

#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/ipopt_solver.h"
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

#include "examples/Spirit/animate_spirit.h"
#include "common/file_utils.h"
#include "lcm/dircon_saved_trajectory.h"
#include "solvers/optimization_utils.h"

#include "examples/Spirit/spirit_utils.h"

DEFINE_double(duration, 1, "The stand duration");
DEFINE_double(standHeight, 0.25, "The standing height.");
DEFINE_double(foreAftDisplacement, 1.3, "The fore-aft displacement.");
DEFINE_double(apexGoal, 0.5, "Apex state goal");
DEFINE_double(inputCost, 3, "The standing height.");
DEFINE_double(velocityCost, 10, "The standing height.");
DEFINE_double(eps, 1e-2, "The wiggle room.");
DEFINE_double(tol, 1e-6, "Optimization Tolerance");
DEFINE_double(mu, 1, "coefficient of friction");

DEFINE_string(data_directory, "/home/shane/Drake_ws/dairlib/examples/Spirit/saved_trajectories/",
              "directory to save/read data");
DEFINE_string(distance_name, "13m","name to describe distance");

DEFINE_bool(runAllOptimization, false, "rerun earlier optimizations?");
DEFINE_bool(skipInitialOptimization, false, "skip first optimizations?");
DEFINE_bool(minWork, true, "try to minimize work?");

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

/// badSpiritJump, generates a bad initial guess for the spirit jump traj opt
/// \param plant: robot model
/// \param x_traj[out]: initial and solution state trajectory
/// \param u_traj[out]: initial and solution control trajectory
/// \param l_traj[out]: initial and solution contact force trajectory
/// \param lc_traj[out]: initial and solution contact force slack variable trajectory
/// \param vc_traj[out]: initial and solution contact velocity slack variable trajectory
template <typename T>
void badSpiritJump(MultibodyPlant<T>& plant,
                    PiecewisePolynomial<double>& x_traj,
                    PiecewisePolynomial<double>& u_traj,
                    vector<PiecewisePolynomial<double>>& l_traj,
                    vector<PiecewisePolynomial<double>>& lc_traj,
                    vector<PiecewisePolynomial<double>>& vc_traj){
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(plant.num_positions() +
      plant.num_velocities());

  int nu = plant.num_actuators();
  int nx = plant.num_positions() + plant.num_velocities();
  int nq = plant.num_positions();
  int nv = plant.num_velocities();
  int N = 20; // number of timesteps

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
  xInit = Eigen::VectorXd::Zero(plant.num_positions() + plant.num_velocities());
  xMid = Eigen::VectorXd::Zero(plant.num_positions() + plant.num_velocities());
  xState = Eigen::VectorXd::Zero(plant.num_positions() + plant.num_velocities());

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

  dairlib::nominalSpiritStand( plant, xInit,  0.16); //Update xInit
  dairlib::nominalSpiritStand( plant, xMid,  0.35); //Update xMid

  xMid(positions_map.at("base_z"))=FLAGS_apexGoal;

  VectorXd deltaX(nx);
  VectorXd averageV(nx);
  deltaX = xMid-xInit;
  averageV = deltaX / FLAGS_duration;
  xInit.tail(nv-3) = (averageV.head(nq)).tail(nq-4); //Ignoring Orientation make velocity the average
  double time = 0;
  double dt = FLAGS_duration/(N-1)/2;

  // Initial pose
  xState = xInit;

  for (int i = 0; i < N; i++) {
    time=i*dt; // calculate iteration's time
    init_time.push_back(time);

    // Switch the direction of the stand to go back to the initial state (not actually properly periodic initial)
    if ( i > (N-1)/2 ){
      xState.tail(nv) = -xInit.tail(nv);
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
        xState(positions_map.at("base_z")) + xState(nq + velocities_map.at("base_vz")) * dt;
    // Save timestep state into matrix
    init_x.push_back(xState);
    init_u.push_back(Eigen::VectorXd::Zero(nu));
  }
  // Make matrix into trajectory
  x_traj = PiecewisePolynomial<double>::ZeroOrderHold(init_time, init_x);
  u_traj = PiecewisePolynomial<double>::ZeroOrderHold(init_time, init_u);


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
    init_time_j.push_back(i * FLAGS_duration / (N - 1));
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

  // Flight
  init_l_j.clear();
  init_lc_j.clear();
  init_vc_j.clear();
  init_time_j.clear();
  for (int i = 0; i < N; i++) {
    init_time_j.push_back(i * FLAGS_duration / (N - 1));
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
    init_time_j.push_back(i * FLAGS_duration / (N - 1));
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

/// addCost, adds the cost to the trajopt jump problem. See runSpiritJump for a description of the inputs
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
  int n_u = plant.num_actuators();
  auto u   = trajopt.input();
  auto x   = trajopt.state();

  // Setup the traditional cost function
  trajopt.AddRunningCost( u.transpose()*cost_actuation*u);
  double Q = 0;

  // Add velocity cost handleing discontinuities
  // Loop through each mode and each knot point and use trapezoidal integration
  for(int mode_index = 0; mode_index < trajopt.num_modes(); mode_index++){
    for(int knot_index = 0; knot_index < trajopt.mode_length(mode_index)-1; knot_index++){

      // Get lower and upper knot velocities
      drake::solvers::VectorXDecisionVariable xi  = trajopt.state_vars(mode_index, knot_index).tail(n_v);
      drake::solvers::VectorXDecisionVariable xip = trajopt.state_vars(mode_index, knot_index+1).tail(n_v);

      drake::symbolic::Expression hi = trajopt.timestep(trajopt.get_mode_start(mode_index) + knot_index)[0];

      // Loop through and calcuate sum of velocities squared
      drake::symbolic::Expression gi = 0;
      drake::symbolic::Expression gip = 0;
      for(int i = 0; i< n_v; i++){
        gi += cost_velocity * xi[i] * xi[i];
        gip += cost_velocity * xip[i] * xip[i];
      }

      // Add cost
      trajopt.AddCost(hi/2.0 * (gi + gip));
    }
  }

  // Vector of new decision variables
  std::vector<drake::symbolic::Variable> power_pluses;
  std::vector<drake::symbolic::Variable> power_minuses;

  // Loop through each joint
  for (int joint = 0; joint < 12; joint++) {
    if(joint == 1 or joint == 3 or joint == 5 or joint == 7)
      Q = 0.249;
    else
      Q = 0.561;
    // Loop through each mode
    for (int mode_index = 0; mode_index < trajopt.num_modes(); mode_index++) {
      for (int knot_index = 0; knot_index < trajopt.mode_length(mode_index); knot_index++) {
        // Create ith set of power variables
        power_pluses.push_back(trajopt.NewContinuousVariables(1, "joint_" + std::to_string(joint)+
            "_mode_"+ std::to_string(mode_index)+"_index_"+std::to_string(knot_index)+"_power_plus")[0]);
        power_minuses.push_back( trajopt.NewContinuousVariables(1, "joint_" + std::to_string(joint)+
            "_mode_"+ std::to_string(mode_index)+"_index_"+std::to_string(knot_index)+"_power_minus")[0]);

        // ith power variables
        drake::symbolic::Variable power_plus_i = power_pluses[power_pluses.size()-1];
        drake::symbolic::Variable power_minus_i = power_minuses[power_minuses.size()-1];

        // Get current actuation and state
        auto u_i = trajopt.input(trajopt.get_mode_start(mode_index) + knot_index);
        auto x_i   = trajopt.state_vars(mode_index, knot_index);
        drake::symbolic::Variable actuation = u_i(actuator_map.at("motor_" + std::to_string(joint)));
        drake::symbolic::Variable velocity = x_i(n_q + velocities_map.at("joint_" + std::to_string(joint) +"dot"));

        // Constrain newly power variables
        if (cost_work > 0){
          trajopt.AddConstraint((actuation * velocity + Q * actuation * actuation) * work_constraint_scale == (power_plus_i - power_minus_i) * work_constraint_scale) ;
          trajopt.AddLinearConstraint(power_plus_i * work_constraint_scale >= 0);
          trajopt.AddLinearConstraint(power_minus_i * work_constraint_scale >= 0);
        }
        trajopt.SetInitialGuess(power_plus_i, 0);
        trajopt.SetInitialGuess(power_minus_i, 0);

        // For 0th iteration, dont bother adding cost
        if(knot_index > 0){

          auto u_im = trajopt.input(trajopt.get_mode_start(mode_index) + knot_index-1);
          drake::symbolic::Variable actuation_m = u_im(actuator_map.at("motor_" + std::to_string(joint)));

          // ith-1 power variables
          drake::symbolic::Variable power_plus_im = power_pluses[power_pluses.size()-2];
          drake::symbolic::Variable power_minus_im = power_minuses[power_minuses.size()-2];

          // Get ith - 1 time step
          drake::symbolic::Expression him = trajopt.timestep(trajopt.get_mode_start(mode_index) + knot_index-1)[0];

          // abs of power at ith and ith+1
          drake::symbolic::Expression gi  = power_plus_i;
          drake::symbolic::Expression gim = power_plus_im;

          // add cost
          trajopt.AddCost(cost_work * him/2.0 * (gi + gim));
        }
      } // knot point loop
    } // Mode loop
  } // Joint loop
} // Function


// addConstraints, adds constraints to the trajopt jump problem. See runSpiritJump for a description of the inputs
template <typename T>
void addConstraints(MultibodyPlant<T>& plant,
             dairlib::systems::trajectory_optimization::Dircon<T>& trajopt,
                    const double apex_height,
                    const double initial_height,
                    const double fore_aft_displacement,
                    const bool lock_rotation,
                    const bool lock_legs_apex,
                    const bool force_symmetry,
                    const bool use_nominal_stand,
                    const double max_duration,
                    const double eps){

  // Get position and velocity dictionaries
  auto positions_map = multibody::makeNameToPositionsMap(plant);
  auto velocities_map = multibody::makeNameToVelocitiesMap(plant);

  setSpiritJointLimits(plant, trajopt);
  setSpiritActuationLimits(plant, trajopt);

  if (force_symmetry) {
    setSpiritSymmetry(plant, trajopt);
  }
  /// Setup all the optimization constraints
  int n_q = plant.num_positions();
  int n_v = plant.num_velocities();

  auto   x0  = trajopt.initial_state();
  auto   xlo = trajopt.state_vars(1, 0);
  auto xapex = trajopt.state_vars(2, 0);
  auto   xtd = trajopt.state_vars(3, 0);
  auto   xf  = trajopt.final_state();

  // Add duration constraint, currently constrained not bounded
  trajopt.AddDurationBounds(0, max_duration);

  /// Constraining xy position
  // Initial body positions
  trajopt.AddBoundingBoxConstraint(0, 0, x0(positions_map.at("base_x"))); // Give the initial condition room to choose the x_init position
  trajopt.AddBoundingBoxConstraint(-eps, eps, x0(positions_map.at("base_y")));
  // Lift off body positions conditions
  trajopt.AddBoundingBoxConstraint(-eps, eps, xlo(positions_map.at("base_y")));
  // Apex body positions conditions
  trajopt.AddBoundingBoxConstraint(-eps, eps, xapex(positions_map.at("base_y")));
  // Touchdown body positions conditions
  trajopt.AddBoundingBoxConstraint(-eps, eps, xtd(positions_map.at("base_y")));
  // Final body positions conditions
  trajopt.AddBoundingBoxConstraint(fore_aft_displacement - eps, fore_aft_displacement + eps, xf(positions_map.at("base_x"))); // Give the initial condition room to choose the x_init position (helps with positive knee constraint)
  trajopt.AddBoundingBoxConstraint(-eps, eps, xf(positions_map.at("base_y")));

  // Nominal stand or z and attitude
  if(use_nominal_stand){
    nominalSpiritStandConstraint(plant,trajopt,initial_height, {0,trajopt.N()-1}, eps);
  }else{
    trajopt.AddBoundingBoxConstraint(initial_height - eps, initial_height + eps, x0(positions_map.at("base_z")));
    trajopt.AddBoundingBoxConstraint(initial_height - eps, initial_height + eps, xf(positions_map.at("base_z")));
  }


  // Body pose constraints (keep the body flat) at initial state
  trajopt.AddBoundingBoxConstraint(1 - eps, 1 + eps, x0(positions_map.at("base_qw")));
  trajopt.AddBoundingBoxConstraint(0 - eps, 0 + eps, x0(positions_map.at("base_qx")));
  trajopt.AddBoundingBoxConstraint(0 - eps, 0 + eps, x0(positions_map.at("base_qy")));
  trajopt.AddBoundingBoxConstraint(0 - eps, 0 + eps, x0(positions_map.at("base_qz")));


  // Body pose constraints (keep the body flat) at final state
  trajopt.AddBoundingBoxConstraint(1 - eps, 1 + eps, xf(positions_map.at("base_qw")));
  trajopt.AddBoundingBoxConstraint(0 - eps, 0 + eps, xf(positions_map.at("base_qx")));
  trajopt.AddBoundingBoxConstraint(0 - eps, 0 + eps, xf(positions_map.at("base_qy")));
  trajopt.AddBoundingBoxConstraint(0 - eps, 0 + eps, xf(positions_map.at("base_qz")));

  // Initial and final velocity
  trajopt.AddBoundingBoxConstraint(VectorXd::Zero(n_v), VectorXd::Zero(n_v), x0.tail(n_v));
  trajopt.AddBoundingBoxConstraint(VectorXd::Zero(n_v), VectorXd::Zero(n_v), xf.tail(n_v));

  // Apex height
  if(apex_height > 0)
    trajopt.AddBoundingBoxConstraint(apex_height - eps, apex_height + eps, xapex(positions_map.at("base_z")) );

  double upperSet = 1;
  double kneeSet = 2;

  if(lock_legs_apex){
    //STATIC LEGS AT APEX
    trajopt.AddBoundingBoxConstraint(upperSet - eps, upperSet + eps, xapex(positions_map.at("joint_0") ) );
    trajopt.AddBoundingBoxConstraint(kneeSet - eps, kneeSet + eps, xapex(positions_map.at("joint_1") ) );

    trajopt.AddBoundingBoxConstraint(upperSet - eps, upperSet + eps, xapex(positions_map.at("joint_2") ) );
    trajopt.AddBoundingBoxConstraint(kneeSet - eps, kneeSet + eps, xapex(positions_map.at("joint_3") ) );

    trajopt.AddBoundingBoxConstraint(upperSet - eps, upperSet + eps, xapex(positions_map.at("joint_4") ) );
    trajopt.AddBoundingBoxConstraint(kneeSet - eps, kneeSet + eps, xapex(positions_map.at("joint_5") ) );

    trajopt.AddBoundingBoxConstraint(upperSet - eps, upperSet + eps, xapex(positions_map.at("joint_6") ) );
    trajopt.AddBoundingBoxConstraint(kneeSet - eps, kneeSet + eps, xapex(positions_map.at("joint_7") ) );

    trajopt.AddBoundingBoxConstraint(-0, 0, xapex( n_q + velocities_map.at("joint_0dot")));
    trajopt.AddBoundingBoxConstraint(-0, 0, xapex( n_q + velocities_map.at("joint_1dot")));
    trajopt.AddBoundingBoxConstraint(-0, 0, xapex( n_q + velocities_map.at("joint_2dot")));
    trajopt.AddBoundingBoxConstraint(-0, 0, xapex( n_q + velocities_map.at("joint_3dot")));
    trajopt.AddBoundingBoxConstraint(-0, 0, xapex( n_q + velocities_map.at("joint_4dot")));
    trajopt.AddBoundingBoxConstraint(-0, 0, xapex( n_q + velocities_map.at("joint_5dot")));
    trajopt.AddBoundingBoxConstraint(-0, 0, xapex( n_q + velocities_map.at("joint_6dot")));
    trajopt.AddBoundingBoxConstraint(-0, 0, xapex( n_q + velocities_map.at("joint_7dot")));
    trajopt.AddBoundingBoxConstraint(-0, 0, xapex( n_q + velocities_map.at("joint_8dot")));
    trajopt.AddBoundingBoxConstraint(-0, 0, xapex( n_q + velocities_map.at("joint_9dot")));
    trajopt.AddBoundingBoxConstraint(-0, 0, xapex( n_q + velocities_map.at("joint_10dot")));
    trajopt.AddBoundingBoxConstraint(-0, 0, xapex( n_q + velocities_map.at("joint_11dot")));
  }

  for (int i = 0; i < trajopt.N(); i++){
    auto xi = trajopt.state(i);
    //Orientation
    if (lock_rotation and i != 0 and i != (trajopt.N()-1))
    {
      trajopt.AddBoundingBoxConstraint(1 - eps, 1 + eps, xi(positions_map.at("base_qw")));
      trajopt.AddBoundingBoxConstraint(0 - eps, 0 + eps, xi(positions_map.at("base_qx")));
      trajopt.AddBoundingBoxConstraint(0 - eps, 0 + eps, xi(positions_map.at("base_qy")));
      trajopt.AddBoundingBoxConstraint(0 - eps, 0 + eps, xi(positions_map.at("base_qz")));
    }

    // Height
    trajopt.AddBoundingBoxConstraint( 0.15, 5, xi( positions_map.at("base_z")));
  }
}

/// getModeSequence, initializes the trajopt mode seqence for jump, see runSpiritJump for a def of inputs
template <typename T>
std::tuple<  std::vector<std::unique_ptr<dairlib::systems::trajectory_optimization::DirconMode<T>>>,
             std::vector<std::unique_ptr<multibody::WorldPointEvaluator<T>>> ,
             std::vector<std::unique_ptr<multibody::KinematicEvaluatorSet<T>>>>
getModeSequence(
    drake::multibody::MultibodyPlant<T>& plant, // multibodyPlant
    const double mu,
    std::vector<int> num_knot_points,
    DirconModeSequence<T>& sequence){

  dairlib::ModeSequenceHelper msh;
  msh.addMode( // Stance
      (Eigen::Matrix<bool,1,4>() << true,  true,  true,  true).finished(), // contact bools
      num_knot_points[0],  // number of knot points in the collocation
      Eigen::Vector3d::UnitZ(), // normal
      Eigen::Vector3d::Zero(),  // world offset
      mu //friction
  );
  msh.addMode( // Flight
      (Eigen::Matrix<bool,1,4>() << false, false, false, false).finished(), // contact bools
      num_knot_points[1],  // number of knot points in the collocation
      Eigen::Vector3d::UnitZ(), // normal
      Eigen::Vector3d::Zero(),  // world offset
      mu //friction
  );
  msh.addMode( // Flight
      (Eigen::Matrix<bool,1,4>() << false, false, false, false).finished(), // contact bools
      num_knot_points[2],  // number of knot points in the collocation
      Eigen::Vector3d::UnitZ(), // normal
      Eigen::Vector3d::Zero(),  // world offset
      mu //friction
  );
  msh.addMode( // Stance
      (Eigen::Matrix<bool,1,4>() << true,  true,  true,  true).finished(), // contact bools
      num_knot_points[3],  // number of knot points in the collocation
      Eigen::Vector3d::UnitZ(), // normal
      Eigen::Vector3d::Zero(),  // world offset
      mu //friction
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

  return {std::move(modeVector), std::move(toeEvals), std::move(toeEvalSets)};
}

/// runSpiritJump, runs a trajectory optimization problem for spirit jumping on flat ground
/// \param plant_ptr: robot model
/// \param plant_double_ptr: model used for animation
/// \param scene_graph_ptr: scene graph
/// \param x_traj[in, out]: initial and solution state trajectory
/// \param u_traj[in, out]: initial and solution control trajectory
/// \param l_traj[in, out]: initial and solution contact force trajectory
/// \param lc_traj[in, out]: initial and solution contact force slack variable trajectory
/// \param vc_traj[in, out]: initial and solution contact velocity slack variable trajectory
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
void runSpiritJump(
    MultibodyPlant<T>& plant,
    PiecewisePolynomial<double>& x_traj,
    PiecewisePolynomial<double>& u_traj,
    vector<PiecewisePolynomial<double>>& l_traj,
    vector<PiecewisePolynomial<double>>& lc_traj,
    vector<PiecewisePolynomial<double>>& vc_traj,
    const bool animate,
    std::vector<int> num_knot_points,
    const double apex_height,
    const double initial_height,
    const double fore_aft_displacement,
    const bool lock_rotation,
    const bool lock_legs_apex,
    const bool force_symmetry,
    const bool use_nominal_stand,
    const double max_duration,
    const double cost_actuation,
    const double cost_velocity,
    const double cost_work,
    const double mu,
    const double eps,
    const double tol,
    const double work_constraint_scale,
    const std::string& file_name_out,
    const std::string& file_name_in= ""
    ) {
  drake::systems::DiagramBuilder<double> builder;

  auto plant_vis = std::make_unique<MultibodyPlant<double>>(0.0);
  auto scene_graph_ptr = std::make_unique<SceneGraph<double>>();
  Parser parser_vis(plant_vis.get(), scene_graph_ptr.get());
  std::string full_name =
      dairlib::FindResourceOrThrow("examples/Spirit/spirit_drake.urdf");
  parser_vis.AddModelFromFile(full_name);
  plant_vis->Finalize();
  SceneGraph<double>& scene_graph =
      *builder.AddSystem(std::move(scene_graph_ptr));

  // Setup mode sequence
  auto sequence = DirconModeSequence<T>(plant);
  auto [modeVector, toeEvals, toeEvalSets] = getModeSequence(plant, mu, num_knot_points, sequence);

  ///Setup trajectory optimization
  auto trajopt = Dircon<T>(sequence);
  // Set up Trajectory Optimization options
  trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(),
                           "Print file", "../snopt.out");
  trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(),
                           "Major iterations limit", 10000);
  trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(), "Iterations limit", 1000000);
  trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(),
                           "Major optimality tolerance",
                           tol);  // target optimality
  trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(), "Major feasibility tolerance", tol);
  trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(), "Verify level",
                           0);  // 0

  // Setting up cost
  addCost(plant, trajopt, cost_actuation, cost_velocity, cost_work, work_constraint_scale);


  // Initialize the trajectory control state and forces
  if (file_name_in.empty()){
    trajopt.drake::systems::trajectory_optimization::MultipleShooting::
        SetInitialTrajectory(u_traj, x_traj);
    for (int j = 0; j < sequence.num_modes(); j++) {
      trajopt.SetInitialForceTrajectory(j, l_traj[j], lc_traj[j],
                                        vc_traj[j]);
    }
  }else{
    std::cout<<"Loading decision var from file, will fail if num dec vars changed" <<std::endl;
    dairlib::DirconTrajectory loaded_traj(file_name_in);
    trajopt.SetInitialGuessForAllVariables(loaded_traj.GetDecisionVariables());
  }

  addConstraints(plant, trajopt, apex_height, initial_height, fore_aft_displacement, lock_rotation,
                    lock_legs_apex, force_symmetry, use_nominal_stand, max_duration, eps);

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
  std::cout << "Solve time: " << elapsed.count() <<std::endl;
  std::cout << "Cost: " << result.get_optimal_cost() <<std::endl;
  std::cout << (result.is_success() ? "Optimization Success" : "Optimization Fail") << std::endl;

  /// Save trajectory

  if(!file_name_out.empty()){
    dairlib::DirconTrajectory saved_traj(
        plant, trajopt, result, "Jumping trajectory",
        "Decision variables and state/input trajectories "
        "for jumping");

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
  auto x_trajs = trajopt.ReconstructDiscontinuousStateTrajectory(result);
  std::cout<<"Work = " << dairlib::calcWork(plant, x_trajs, u_traj) << std::endl;
  std::cout<<"Integral of Actuation = " << dairlib::calcTorqueInt(plant, u_traj) << std::endl;
  std::cout<<"Integral of Velocity = " << dairlib::calcVelocityInt(plant, x_trajs) << std::endl;

  /// Run animation of the final trajectory
  const drake::trajectories::PiecewisePolynomial<double> pp_xtraj =
      trajopt.ReconstructStateTrajectory(result);

  multibody::connectTrajectoryVisualizer(plant_vis.get(),
      &builder, &scene_graph, pp_xtraj);
  auto diagram = builder.Build();
  while (animate) {
    drake::systems::Simulator<double> simulator(*diagram);
    simulator.set_target_realtime_rate(0.25);
    simulator.Initialize();
    simulator.AdvanceTo(pp_xtraj.end_time());
    sleep(2);
  }
}
}  // namespace
}  // namespace dairlib


int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  std::srand(time(0));  // Initialize random number generator.

  auto plant = std::make_unique<MultibodyPlant<double>>(0.0);
  auto plant_vis = std::make_unique<MultibodyPlant<double>>(0.0);
  auto scene_graph = std::make_unique<SceneGraph<double>>();
  Parser parser(plant.get());
  Parser parser_vis(plant_vis.get(), scene_graph.get());
  std::string full_name =
      dairlib::FindResourceOrThrow("examples/Spirit/spirit_drake.urdf");

  parser.AddModelFromFile(full_name);
  parser_vis.AddModelFromFile(full_name);

  plant->mutable_gravity_field().set_gravity_vector(-9.81 *
      Eigen::Vector3d::UnitZ());

  plant->Finalize();
  plant_vis->Finalize();

  PiecewisePolynomial<double> x_traj;
  PiecewisePolynomial<double> u_traj;
  std::vector<PiecewisePolynomial<double>> l_traj;
  std::vector<PiecewisePolynomial<double>> lc_traj;
  std::vector<PiecewisePolynomial<double>> vc_traj;

  if (FLAGS_runAllOptimization){
    if(! FLAGS_skipInitialOptimization){
      std::cout<<"Running initial optimization"<<std::endl;
      dairlib::badSpiritJump(*plant, x_traj, u_traj, l_traj, lc_traj, vc_traj);
      dairlib::runSpiritJump<double>(
          *plant,
          x_traj, u_traj, l_traj,
          lc_traj, vc_traj,
          false,
          {7, 7, 7, 7},
          FLAGS_apexGoal,
          FLAGS_standHeight,
          0,
          true,
          true,
          false,
          true,
          2,
          3,
          10,
          0,
          100,
          FLAGS_eps,
          1e-1,
          0,
          FLAGS_data_directory+"simple_jump");
    }
    else{
      dairlib::DirconTrajectory old_traj(FLAGS_data_directory+"simple_jump");
      x_traj = old_traj.ReconstructStateTrajectory();
      u_traj = old_traj.ReconstructInputTrajectory();
      l_traj = old_traj.ReconstructLambdaTrajectory();
      lc_traj = old_traj.ReconstructLambdaCTrajectory();
      vc_traj = old_traj.ReconstructGammaCTrajectory();
    }

    std::cout<<"Running 2nd optimization"<<std::endl;
    // Hopping correct distance, but heavily constrained
    dairlib::runSpiritJump<double>(
        *plant,
        x_traj, u_traj, l_traj,
        lc_traj, vc_traj,
        false,
        {7, 7, 7, 7} ,
        FLAGS_apexGoal,
        FLAGS_standHeight,
        FLAGS_foreAftDisplacement,
        true,
        true,
        false,
        true,
        2,
        3,
        10,
        0,
        100,
        0,
        1e-2,
        0,
        FLAGS_data_directory+"jump_"+FLAGS_distance_name);
  }
  std::cout<<"Running 3rd optimization"<<std::endl;
  // Fewer constraints, and higher tolerences
  dairlib::runSpiritJump<double>(
      *plant,
      x_traj, u_traj, l_traj,
      lc_traj, vc_traj,
      !FLAGS_minWork,
      {7, 7, 7, 7} ,
      FLAGS_apexGoal,
      FLAGS_standHeight,
      FLAGS_foreAftDisplacement,
      false,
      false,
      false,
      true,
      2,
      3,
      10,
      0,
      FLAGS_mu,
      FLAGS_eps,
      FLAGS_tol,
      0,
      FLAGS_data_directory+"jump_"+FLAGS_distance_name+"_hq",
      FLAGS_data_directory+"jump_"+FLAGS_distance_name);

  if (FLAGS_minWork){
    // Adding in work cost and constraints
    std::cout<<"Running 4th optimization"<<std::endl;
    dairlib::runSpiritJump<double>(
        *plant,
        x_traj, u_traj, l_traj,
        lc_traj, vc_traj,
        true,
        {7, 7, 7, 7} ,
        FLAGS_apexGoal,
        FLAGS_standHeight,
        FLAGS_foreAftDisplacement,
        false,
        false,
        false,
        true,
        1.5,
        FLAGS_inputCost/10,
        FLAGS_velocityCost/10,
        500,
        FLAGS_mu,
        FLAGS_eps,
        FLAGS_tol,
        1.0,
        FLAGS_data_directory+"jump_"+FLAGS_distance_name+"_hq_work_option3",
        FLAGS_data_directory+"jump_"+FLAGS_distance_name+"_hq");
  }
}

