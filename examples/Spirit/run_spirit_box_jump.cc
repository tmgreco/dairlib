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
#include "drake/geometry/geometry_visualization.h"
#include "drake/solvers/solve.h"
#include "drake/math/orthonormal_basis.h"

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

DEFINE_string(data_directory, "/home/jdcap/dairlib/examples/Spirit/saved_trajectories/",
              "directory to save/read data");
DEFINE_string(distance_name, "iterativeBox45","name to describe distance");

DEFINE_bool(runAllOptimization, false, "rerun earlier optimizations?");
DEFINE_bool(skipInitialOptimization, true, "skip first optimizations?");
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

  int nu = plant.num_actuators();
  int nx = plant.num_positions() + plant.num_velocities();
  int nq = plant.num_positions();
  int nv = plant.num_velocities();
  int N = 20; // number of timesteps

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
    // nominalSpiritStandConstraint(plant,trajopt,initial_height, {0,trajopt.N()-1}, eps);
    VectorXd standJointsInit = initialStand.getJoints();
    VectorXd standJointsFinal = finalStand.getJoints();
    VectorXd standQuatInit = initialStand.getQuat();
    VectorXd standQuatFinal = finalStand.getQuat();

    std::cout<<"\nInit:\n"<<standJointsInit<<"\nFinal:\n"<< standJointsFinal<<std::endl;

    trajopt.AddBoundingBoxConstraint(standJointsInit,standJointsInit,(x0.head(n_q)).tail(n_q-7));
    trajopt.AddBoundingBoxConstraint(standJointsFinal,standJointsFinal,(xf.head(n_q)).tail(n_q-7));

     // Body pose constraints (keep the body flat) at initial state
    trajopt.AddBoundingBoxConstraint(
          standQuatInit - Eigen::Vector4d::Constant(eps), 
          standQuatInit + Eigen::Vector4d::Constant(eps), 
          x0.head(4));
    // Body pose constraints (keep the body flat) at final state
    trajopt.AddBoundingBoxConstraint(
          standQuatFinal - Eigen::Vector4d::Constant(eps), 
          standQuatFinal + Eigen::Vector4d::Constant(eps), 
          xf.head(4));

  }else{
    trajopt.AddBoundingBoxConstraint(initial_height - eps, initial_height + eps, x0(positions_map.at("base_z")));
    trajopt.AddBoundingBoxConstraint(initial_height - eps, initial_height  + eps, xf(positions_map.at("base_z")));
  }

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

    // //Orientation
    // if (lock_rotation and i != 0 and i != (trajopt.N()-1))
    // {
    //   trajopt.AddBoundingBoxConstraint(1 - eps, 1 + eps, xi(positions_map.at("base_qw")));
    //   trajopt.AddBoundingBoxConstraint(0 - eps, 0 + eps, xi(positions_map.at("base_qx")));
    //   trajopt.AddBoundingBoxConstraint(0 - eps, 0 + eps, xi(positions_map.at("base_qy")));
    //   trajopt.AddBoundingBoxConstraint(0 - eps, 0 + eps, xi(positions_map.at("base_qz")));
    // }
    // Height
    trajopt.AddBoundingBoxConstraint( 0.15, 5, xi( positions_map.at("base_z")));
  }
  //Orientation lock rotation in flight
  for (int iMode = 1; iMode<=2; iMode++){
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


/// runSpiritBoxJump, runs a trajectory optimization problem for spirit jumping on flat ground
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
void runSpiritBoxJump(
      MultibodyPlant<T>& plant,
      PiecewisePolynomial<double>& x_traj,
      PiecewisePolynomial<double>& u_traj,
      vector<PiecewisePolynomial<double>>& l_traj,
      vector<PiecewisePolynomial<double>>& lc_traj,
      vector<PiecewisePolynomial<double>>& vc_traj,
      dairlib::OptimalSpiritStand& initialStand,
      dairlib::OptimalSpiritStand& finalStand,
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

  // Setup mode sequence
  auto sequence = DirconModeSequence<T>(plant);
  dairlib::ModeSequenceHelper msh;
  msh.addMode( // Stance
      (Eigen::Matrix<bool,1,4>() << true,  true,  true,  true).finished(), // contact bools
      num_knot_points[0],  // number of knot points in the collocation
      initialStand.normal(), // normal
      initialStand.offset(),  // world offset
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
      finalStand.normal(), // normal
      finalStand.offset(),  // world offset
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

  ///Setup trajectory optimization
  auto trajopt = Dircon<T>(sequence);
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

  // Setting up cost
  addCost(plant, trajopt, cost_actuation, cost_velocity, cost_work, work_constraint_scale);
  // Setting up constraints
  addConstraints(plant, trajopt, initialStand, finalStand,  apex_height, initial_height, fore_aft_displacement, lock_rotation,
                    lock_legs_apex, force_symmetry, use_nominal_stand, max_duration, eps);

  // Initialize the trajectory control state and forces
  if (file_name_in.empty()){
    trajopt.drake::systems::trajectory_optimization::MultipleShooting::
        SetInitialTrajectory(u_traj, x_traj);
    for (int j = 0; j < sequence.num_modes(); j++) {
      trajopt.SetInitialForceTrajectory(j, l_traj[j], lc_traj[j], vc_traj[j]);
    }
  }else{
    std::cout<<"Loading decision var from file, will fail if num dec vars changed" <<std::endl;
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

    dairlib::visualizeSurface(
        plant_vis.get(),
        finalStand.normal(),
        finalStand.offset(),
        0.5,
        0.5,
        0.1);

    plant_vis->Finalize();
    SceneGraph<double>& scene_graph = *builder.AddSystem(std::move(scene_graph_ptr));

    const drake::trajectories::PiecewisePolynomial<double> pp_xtraj =
        trajopt.ReconstructStateTrajectory(result);
    multibody::connectTrajectoryVisualizer(plant_vis.get(),
        &builder, &scene_graph, pp_xtraj);
    auto diagram = builder.Build();
    std::cout << "animating, kill to end." << std::endl;
    while (animate) {
      drake::systems::Simulator<double> simulator(*diagram);
      simulator.set_target_realtime_rate(0.25);
      simulator.Initialize();
      simulator.AdvanceTo(pp_xtraj.end_time());
      sleep(2);
    }
  }
}
}  // namespace
}  // namespace dairlib


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
  Eigen::Vector3d initialNormal = Eigen::Vector3d::UnitZ();
  Eigen::Vector3d normal = 1 *Eigen::Vector3d::UnitZ() + 1 * Eigen::Vector3d::UnitY();
  normal = normal/normal.norm();
  Eigen::Vector3d offset = Eigen::Vector3d::UnitX()*FLAGS_foreAftDisplacement + Eigen::Vector3d::UnitZ()*FLAGS_boxHeight;
  std::cout<<"Normal Set: "<<normal<<std::endl;
  dairlib::OptimalSpiritStand initialStand(plant.get(), FLAGS_standHeight, initialNormal, Eigen::Vector3d::Zero(),false);
  dairlib::OptimalSpiritStand   finalStand(plant.get(), FLAGS_standHeight, normal, offset, false);
  std::vector<dairlib::OptimalSpiritStand> stands;
  int numSteps = 1;
  stands.push_back(initialStand);
  if (FLAGS_runIterative){
    //Get the stands for the necessary angles
    std::cout<<"DotProd: "<<normal.dot(Eigen::Vector3d::UnitZ())<<std::endl;
    double angle = acos( normal.dot(Eigen::Vector3d::UnitZ()))*(180/M_PI);
    if (!(angle<1e-4)){
      Eigen::Vector3d axis =  normal.cross(Eigen::Vector3d::UnitZ());
      axis = axis/axis.norm();
      numSteps = ceil(angle/FLAGS_iterationAngResDeg);
      std::cout<<"Angle: "<<angle<<"    numsteps"<<numSteps<<std::endl;
      Eigen::Matrix3d initialFrame = Eigen::Matrix3d::Identity();

      for (int iStep = 1; iStep < numSteps-1; iStep++){
        Eigen::AngleAxis iRotation(iStep*angle*(M_PI/180)/numSteps,axis);
        std::cout<<"Rot Mat: "<< iRotation.toRotationMatrix()<< std::endl;
        std::cout<<"initialFrame: "<< initialFrame<< std::endl;
        std::cout<<"iNormal: "<< (initialFrame*(iRotation.toRotationMatrix()).transpose()).col(2)<< std::endl;
        std::cout<<"iTranspose: "<< (initialFrame*(iRotation.toRotationMatrix())).col(2)<< std::endl;
        Eigen::Vector3d iNormal = (initialFrame*(iRotation.toRotationMatrix()).transpose()).col(2);
        dairlib::OptimalSpiritStand iStand(plant.get(), FLAGS_standHeight, iNormal, offset, false);
        stands.push_back(iStand);
      }
    }
  }
  stands.push_back(finalStand);

  if (FLAGS_runAllOptimization){
    if(! FLAGS_skipInitialOptimization){
      std::cout<<"Running initial optimization"<<std::endl;
      dairlib::badSpiritJump(*plant, x_traj, u_traj, l_traj, lc_traj, vc_traj);
      dairlib::runSpiritBoxJump<double>(
          *plant,
          x_traj, 
          u_traj, 
          l_traj,
          lc_traj, 
          vc_traj,
          initialStand,
          finalStand,
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
          FLAGS_data_directory+"simple_boxjump");
    }
    else{
      dairlib::DirconTrajectory old_traj(FLAGS_data_directory+"simple_boxjump");
      x_traj = old_traj.ReconstructStateTrajectory();
      u_traj = old_traj.ReconstructInputTrajectory();
      l_traj = old_traj.ReconstructLambdaTrajectory();
      lc_traj = old_traj.ReconstructLambdaCTrajectory();
      vc_traj = old_traj.ReconstructGammaCTrajectory();
    }

    dairlib::OptimalSpiritStand   finalFlatStand(plant.get(), FLAGS_standHeight, initialNormal, offset, true);
    std::cout<<"Running 2nd optimization"<<std::endl;
    // Hopping correct distance, but heavily constrained
    dairlib::runSpiritBoxJump<double>(
        *plant,
        x_traj, u_traj, l_traj,
        lc_traj, vc_traj,
        initialStand,
        finalFlatStand,
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
        FLAGS_data_directory+"boxjump_"+FLAGS_distance_name + "_" + std::to_string(1));
  }
  for (int iStep = 1; iStep<numSteps;iStep++){

    std::cout<<" Running hq optimization step: " << iStep  <<"of "<<numSteps <<std::endl;
    std::cout<<" Normal for the iSurface " << stands[iStep].normal() << std::endl;
    // Fewer constraints, and higher tolerences
    dairlib::runSpiritBoxJump<double>(
        *plant,
        x_traj, u_traj, l_traj,
        lc_traj, vc_traj,
        initialStand,
        stands[iStep],
        (iStep+1 ==numSteps),
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
        FLAGS_data_directory+"boxjump_"+FLAGS_distance_name + "_" + std::to_string(iStep+1),
        FLAGS_data_directory+"boxjump_"+FLAGS_distance_name + "_" + std::to_string(iStep)
        );
  }
  // if (FLAGS_minWork){
  //   std::cout<<"Running 4th optimization"<<std::endl;
  //   // Adding in work cost and constraints

  //   dairlib::runSpiritBoxJump<double>(
  //       *plant,
  //       x_traj, u_traj, l_traj,
  //       lc_traj, vc_traj,
  //       initialStand,
  //       finalStand,
  //       false,
  //       {7, 7, 7, 7} ,
  //       FLAGS_apexGoal,
  //       FLAGS_standHeight,
  //       FLAGS_foreAftDisplacement,
  //       false,
  //       false,
  //       false,
  //       true,
  //       2,
  //       3,
  //       10,
  //       0.01,
  //       FLAGS_mu,
  //       FLAGS_eps,
  //       1e-3,
  //       1.0,
  //       FLAGS_data_directory+"boxjump_"+FLAGS_distance_name+"_hq_work",
  //       FLAGS_data_directory+"boxjump_"+FLAGS_distance_name+"_hq");

  //   // Adding more min work
  //   std::cout<<"Running 5th optimization"<<std::endl;
  //   dairlib::runSpiritBoxJump<double>(
  //       *plant,
  //       x_traj, u_traj, l_traj,
  //       lc_traj, vc_traj,
  //       initialStand,
  //       finalStand,
  //       false,
  //       {7, 7, 7, 7} ,
  //       FLAGS_apexGoal,
  //       FLAGS_standHeight,
  //       FLAGS_foreAftDisplacement,
  //       false,
  //       false,
  //       false,
  //       true,
  //       2,
  //       FLAGS_inputCost,
  //       FLAGS_velocityCost,
  //       5,
  //       FLAGS_mu,
  //       FLAGS_eps,
  //       FLAGS_tol,
  //       1.0/20,
  //       FLAGS_data_directory+"boxjump_"+FLAGS_distance_name+"_hq_work2",
  //       FLAGS_data_directory+"boxjump_"+FLAGS_distance_name+"_hq_work");
  //   std::cout<<"Running 6th optimization"<<std::endl;
  //   dairlib::runSpiritBoxJump<double>(
  //       *plant,
  //       x_traj, u_traj, l_traj,
  //       lc_traj, vc_traj,
  //       initialStand,
  //       finalStand,
  //       true,
  //       {7, 7, 7, 7} ,
  //       FLAGS_apexGoal,
  //       FLAGS_standHeight,
  //       FLAGS_foreAftDisplacement,
  //       false,
  //       false,
  //       false,
  //       true,
  //       2,
  //       FLAGS_inputCost,
  //       FLAGS_velocityCost,
  //       10,
  //       FLAGS_mu,
  //       FLAGS_eps,
  //       FLAGS_tol,
  //       1.0,
  //       FLAGS_data_directory+"boxjump_"+FLAGS_distance_name+"_hq_work3",
  //       FLAGS_data_directory+"boxjump_"+FLAGS_distance_name+"_hq_work2");
  // }
}

