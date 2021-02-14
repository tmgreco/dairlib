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
#include <drake/solvers/choose_best_solver.h>
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
#include "solvers/nonlinear_cost.h"


#include "examples/Spirit/spirit_utils.h"

DEFINE_double(standHeight, 0.25, "The standing height.");
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
DEFINE_double(foreAftDisplacement, 0.75  , "The fore-aft displacement.");
DEFINE_double(apexGoal, 0.5, "Apex state goal");
=======
DEFINE_double(foreAftDisplacement, 0.7, "The fore-aft displacement.");
DEFINE_double(apexGoal, 0.25, "Apex state goal");
>>>>>>> Not really making progress
=======
DEFINE_double(foreAftDisplacement, 0.2, "The fore-aft displacement.");
=======
DEFINE_double(foreAftDisplacement, 0.4, "The fore-aft displacement.");
>>>>>>> Seemed to be doing alright
DEFINE_double(apexGoal, 0.45, "Apex state goal");
>>>>>>> Sort of working first half jump
=======
DEFINE_double(foreAftDisplacement, 0.0, "The fore-aft displacement.");
DEFINE_double(apexGoal, 0.4, "Apex state goal");
>>>>>>> Code failing, moving to initial guess work
DEFINE_double(inputCost, 3, "The standing height.");
DEFINE_double(velocityCost, 10, "The standing height.");
DEFINE_double(eps, 1e-2, "The wiggle room.");
DEFINE_double(tol, 1e-3, "Optimization Tolerance");
DEFINE_double(mu, 1, "coefficient of friction");

DEFINE_string(data_directory, "/home/shane/Drake_ws/dairlib/examples/Spirit/saved_trajectories/",
              "directory to save/read data");
<<<<<<< HEAD
DEFINE_string(distance_name, "075cm","name to describe distance");
=======
DEFINE_string(distance_name, "90cm","name to describe distance");
>>>>>>> Not really making progress

DEFINE_bool(runAllOptimization, true, "rerun earlier optimizations?");
<<<<<<< HEAD
DEFINE_bool(skipInitialOptimization, true, "skip first optimizations?");
DEFINE_bool(minWork, true, "try to minimize work?");
DEFINE_bool(ipopt, true, "Use IPOPT as solver instead of SNOPT");
=======
DEFINE_bool(skipInitialOptimization, false, "skip first optimizations?");
DEFINE_bool(minWork, false, "try to minimize work?");
>>>>>>> Moving to simpler behaviors

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

  const double duration = 0.3;
  std::vector<MatrixXd> x_points;
  std::vector<double> time_vec = {0,duration/3.0};
  VectorXd x_const;
  dairlib::ikSpiritStand(plant, x_const, {true, true, true, true}, FLAGS_standHeight, 0.15, 0, 0);
  x_points.push_back(x_const);
  dairlib::ikSpiritStand(plant, x_const, {true, true, true, true}, FLAGS_standHeight, 0.2, 0, -0.2);
  x_points.push_back(x_const);
//  dairlib::ikSpiritStand(plant, x_const, {false, true, false, true}, (FLAGS_standHeight + FLAGS_apexGoal)/2.0, 0.25, 0, -0.35/2);
//  x_points.push_back(x_const);
//  dairlib::ikSpiritStand(plant, x_const, {false, false, false, false}, FLAGS_apexGoal, 0.25, 0, 0);
//  x_points.push_back(x_const);

  x_traj = PiecewisePolynomial<double>::FirstOrderHold(time_vec,x_points);

  std::vector<MatrixXd> u_points;
  u_points.push_back(MatrixXd::Zero(1, plant.num_actuators()));
  u_points.push_back(MatrixXd::Zero(1, plant.num_actuators()));

  u_traj = PiecewisePolynomial<double>::FirstOrderHold(time_vec,u_points);

  // Four contacts so forces are 12 dimensional
  Eigen::VectorXd init_l_vec(12);
  // Initial guess
  init_l_vec << 0, 0, 30*9.81, 0, 0, 30*9.81, 0, 0, 30*9.81, 0, 0, 30*9.81; //gravity and mass distributed

  Eigen::VectorXd init_l_vec_front_stance(12);
  init_l_vec_front_stance << 0, 0, 0, 0, 0, 600*9.81, 0, 0, 0, 0, 0, 600*9.81; //gravity and mass distributed

  Eigen::VectorXd init_l_vec_back_stance(12);
  init_l_vec_back_stance << 0, 0, 600*9.81, 0, 0, 0, 0, 0, 600*9.81, 0, 0, 0; //gravity and mass distributed

  //Initialize force trajectories

  //Stance
  int N = 10;
  std::vector<MatrixXd> init_l_j;
  std::vector<MatrixXd> init_lc_j;
  std::vector<MatrixXd> init_vc_j;
  std::vector<double> init_time_j;
  for (int i = 0; i < N; i++) {
    init_time_j.push_back(i * duration/6.0 / (N - 1));
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

  // front stance
  init_l_j.clear();
  init_lc_j.clear();
  init_vc_j.clear();
  init_time_j.clear();
  for (int i = 0; i < N; i++) {
    init_time_j.push_back(i * duration /6.0/ (N - 1) + duration/3.0);
    init_l_j.push_back(init_l_vec_front_stance);
    init_lc_j.push_back(init_l_vec_front_stance);
    init_vc_j.push_back(VectorXd::Zero(12));
  }

  init_l_traj_j = PiecewisePolynomial<double>::ZeroOrderHold(init_time_j,init_l_j);
  init_lc_traj_j = PiecewisePolynomial<double>::ZeroOrderHold(init_time_j,init_lc_j);
  init_vc_traj_j = PiecewisePolynomial<double>::ZeroOrderHold(init_time_j,init_vc_j);

  l_traj.push_back(init_l_traj_j);
  lc_traj.push_back(init_lc_traj_j);
  vc_traj.push_back(init_vc_traj_j);

  // Flight
  init_l_j.clear();
  init_lc_j.clear();
  init_vc_j.clear();
  init_time_j.clear();
  for (int i = 0; i < N; i++) {
    init_time_j.push_back(i * duration /6.0/ (N - 1) +2.0 * duration/3.0);
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
}

/// addCost, adds the cost to the trajopt jump problem. See runSpiritJump for a description of the inputs
template <typename T>
void addCost(MultibodyPlant<T>& plant,
             dairlib::systems::trajectory_optimization::Dircon<T>& trajopt,
             const double cost_actuation,
             const double cost_velocity,
<<<<<<< HEAD
             const double cost_work){
=======
             const double cost_work,
             const double work_constraint_scale = 1.0){


  auto velocities_map = multibody::makeNameToVelocitiesMap(plant);
  auto actuator_map = multibody::makeNameToActuatorsMap(plant);


  int n_q = plant.num_positions();
  int n_v = plant.num_velocities();
>>>>>>> Moving to simpler behaviors
  auto u   = trajopt.input();
  auto x   = trajopt.state();

  // Setup the traditional cost function
  trajopt.AddRunningCost( u.transpose()*cost_actuation*u);
<<<<<<< HEAD
<<<<<<< HEAD
  trajopt.AddVelocityCost(cost_velocity);
  AddWorkCost(plant, trajopt, cost_work);

<<<<<<< HEAD
=======
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
=======
  trajopt.AddRunningCost( x.tail(n_v).transpose()*cost_velocity*x.tail(n_v));

>>>>>>> Sort of working first half jump

  // Vector of new decision variables
  std::vector<drake::symbolic::Variable> power_pluses;
  std::vector<drake::symbolic::Variable> power_minuses;

  // Loop through each joint
  for (int joint = 0; joint < 12; joint++) {
    auto actuation = u(actuator_map.at("motor_" + std::to_string(joint)));
    auto velocity = x(n_q + velocities_map.at("joint_" + std::to_string(joint) +"dot"));
    trajopt.AddRunningCost(actuation * actuation * velocity * velocity * cost_work);
    // Loop through each mode
  } // Joint loop
<<<<<<< HEAD
  std::cout<<"foo"<<std::endl;
>>>>>>> Not really making progress
=======
>>>>>>> Sort of working first half jump
=======
  trajopt.AddVelocityCost(cost_velocity);
  AddWorkCost(plant, trajopt, cost_work, work_constraint_scale, 0.0);
>>>>>>> Seems to be working for basic pitch
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

  /// Setup all the optimization constraints
  int n_q = plant.num_positions();
  int n_v = plant.num_velocities();

  auto   x0  = trajopt.initial_state();
  auto xapex = trajopt.final_state();

  // Add duration constraint, currently constrained not bounded
  trajopt.AddDurationBounds(0, max_duration);

  /// Constraining xy position
  // Initial body positions
  trajopt.AddBoundingBoxConstraint(0, 0, x0(positions_map.at("base_x"))); // Give the initial condition room to choose the x_init position
  trajopt.AddBoundingBoxConstraint(0, 0, xapex(positions_map.at("base_x"))); // Give the initial condition room to choose the x_init position

  // Final body positions conditions
  trajopt.AddBoundingBoxConstraint( -eps, eps, x0( positions_map.at("base_y")));
  trajopt.AddBoundingBoxConstraint( -eps, eps, xapex( positions_map.at("base_y")));

  // Nominal stand or z and attitude
  nominalSpiritStandConstraint(plant,trajopt,initial_height, {0}, eps);

  // Body pose constraints (keep the body flat) at initial state
  trajopt.AddBoundingBoxConstraint(1 - eps, 1 + eps, x0(positions_map.at("base_qw")));
  trajopt.AddBoundingBoxConstraint(0 - eps, 0 + eps, x0(positions_map.at("base_qx")));
  trajopt.AddBoundingBoxConstraint(0 - eps, 0 + eps, x0(positions_map.at("base_qy")));
  trajopt.AddBoundingBoxConstraint(0 - eps, 0 + eps, x0(positions_map.at("base_qz")));

  double pitch = -0.2;
  // Body pose constraints (keep the body flat) at apex state
  trajopt.AddBoundingBoxConstraint(cos(pitch/2.0), cos(pitch/2.0), xapex(positions_map.at("base_qw")));
  trajopt.AddBoundingBoxConstraint(0, 0, xapex(positions_map.at("base_qx")));
  trajopt.AddBoundingBoxConstraint(sin(pitch/2.0) , sin(pitch/2.0), xapex(positions_map.at("base_qy")));
  trajopt.AddBoundingBoxConstraint(0, 0, xapex(positions_map.at("base_qz")));


  //trajopt.AddCost(-0.1 * xapex(positions_map.at("base_qw")));
  // Initial and final velocity
  trajopt.AddBoundingBoxConstraint(VectorXd::Zero(n_v), VectorXd::Zero(n_v), x0.tail(n_v));
  trajopt.AddBoundingBoxConstraint(VectorXd::Zero(n_v), VectorXd::Zero(n_v), xapex.tail(n_v));

  // Apex height
  trajopt.AddBoundingBoxConstraint(initial_height-eps, initial_height+eps, xapex(positions_map.at("base_z")) );

  for (int i = 0; i < trajopt.N(); i++){
    auto xi = trajopt.state(i);
    //Orientation
    // Height
    trajopt.AddBoundingBoxConstraint( 0.0, 5, xi( positions_map.at("base_z")));
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
      mu, //friction
      0.1
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
    else if (mode->evaluators().num_evaluators() == 2){
      mode->SetKinVelocityScale(
          {0, 1}, {0, 1, 2}, 1.0);
      mode->SetKinPositionScale(
          {0, 1}, {0, 1, 2}, 200);
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
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======

>>>>>>> Code failing, moving to initial guess work
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
<<<<<<< HEAD
<<<<<<< HEAD
                           0);  // 0
<<<<<<< HEAD
  trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(), "Total int workspace",
                          6322783);  // 0
  trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(), "Total real workspace",
                          6322783);  // 0
>>>>>>> Not really making progress

  if (FLAGS_ipopt) {
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
                            "Major iterations limit", 10000);
    trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(), "Iterations limit", 1000000);
    trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(),
                            "Major optimality tolerance",
                            tol);  // target optimality
    trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(), "Major feasibility tolerance", tol);
    trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(), "Verify level",
                            0);  // 0
  }
=======
=======
                           3);  // 0
  trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(), "Derivative option",
                          0);  // 1
>>>>>>> Code failing, moving to initial guess work
=======
                           0);  // 0
  trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(), "Derivative option",
                          1);  // 1
>>>>>>> Moving to simpler behaviors

>>>>>>> Sort of working first half jump
  // Setting up cost
  addCost(plant, trajopt, cost_actuation, cost_velocity, cost_work);

// Initialize the trajectory control state and forces
  if (file_name_in.empty()){
    trajopt.SetInitialGuessForAllVariables(
        VectorXd::Random(trajopt.decision_variables().size()));
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
  int num_ghosts = 1;// Number of ghosts in visualization. NOTE: there are limitations on number of ghosts based on modes and knotpoints
  std::vector<unsigned int> visualizer_poses; // Ghosts for visualizing during optimization
  for(int i = 0; i < sequence.num_modes(); i++){
      visualizer_poses.push_back(num_ghosts); 
  }
  trajopt.CreateVisualizationCallback(
      dairlib::FindResourceOrThrow("examples/Spirit/spirit_drake.urdf"),
      visualizer_poses, 0.2); // setup which URDF, how many poses, and alpha transparency 

  drake::solvers::SolverId solver_id("");
  if (FLAGS_ipopt) {
    solver_id = drake::solvers::IpoptSolver().id();
    cout << "\nChose manually: " << solver_id.name() << endl;
  } else {
    solver_id = drake::solvers::ChooseBestSolver(trajopt);
    cout << "\nChose the best solver: " << solver_id.name() << endl;
  }

  /// Run the optimization using your initial guess
  auto start = std::chrono::high_resolution_clock::now();
<<<<<<< HEAD
<<<<<<< HEAD
  auto solver = drake::solvers::MakeSolver(solver_id);
  drake::solvers::MathematicalProgramResult result;
  solver->Solve(trajopt, trajopt.initial_guess(), trajopt.solver_options(),
                &result);
=======
  std::cout << "num_vars: " << trajopt.num_vars() <<std::endl;
  std::cout << "decision variable 736: " << trajopt.decision_variable(736).get_name() <<std::endl;
  std::cout << "decision variable 737: " << trajopt.decision_variable(737).get_name() <<std::endl;
  std::cout << "decision variable 738: " << trajopt.decision_variable(738).get_name() <<std::endl;
  std::cout << "decision variable 739: " << trajopt.decision_variable(739).get_name() <<std::endl;
  std::cout << trajopt.timestep(1) << std::endl;
  std::cout << trajopt.timestep(2) << std::endl;
  std::cout << trajopt.timestep(3) << std::endl;
  std::cout << trajopt.timestep(4) << std::endl;

=======
>>>>>>> Seems to be working for basic pitch
  const auto result = Solve(trajopt, trajopt.initial_guess());
>>>>>>> Code failing, moving to initial guess work
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
<<<<<<< HEAD
  std::cout<<"Work = " << dairlib::calcElectricalWork(plant, x_trajs, u_traj) << std::endl;
//  double cost_work_acceleration = solvers::EvalCostGivenSolution(
//      result, cost_joint_work_bindings);
//  std::cout<<"Cost Work = " << cost_work_acceleration << std::endl;
=======

>>>>>>> Moving to simpler behaviors
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
<<<<<<< HEAD
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
=======
    dairlib::badSpiritJump(*plant,x_traj,u_traj,l_traj,lc_traj,vc_traj);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> Not really making progress
=======
    dairlib::DirconTrajectory old_traj(FLAGS_data_directory+"jump_05m_hq");
    x_traj = old_traj.ReconstructStateTrajectory();
    u_traj = old_traj.ReconstructInputTrajectory();
>>>>>>> Sort of working first half jump

=======
    dairlib::DirconTrajectory old_traj(FLAGS_data_directory+"simple_jump");
    //x_traj = old_traj.ReconstructStateTrajectory();
    //u_traj = old_traj.ReconstructInputTrajectory();
//    l_traj = old_traj.ReconstructLambdaTrajectory();
//    lc_traj = old_traj.ReconstructLambdaCTrajectory();
//    vc_traj = old_traj.ReconstructGammaCTrajectory();
>>>>>>> Code failing, moving to initial guess work
=======
>>>>>>> Seems to be working for basic pitch
    std::cout<<"Running 2nd optimization"<<std::endl;
    //Hopping correct distance, but heavily constrained

    dairlib::runSpiritJump<double>(
        *plant,
        x_traj, u_traj, l_traj,
        lc_traj, vc_traj,
        true,
        {10, 7, 5, 5, 5, 5} ,
        FLAGS_apexGoal,
        FLAGS_standHeight,
        FLAGS_foreAftDisplacement,
        false,
        false,
        false,
        true,
        0.8,
        3,
        10,
        0,
        100000,
        1e-2,
        1e-4,
        1.0,
        FLAGS_data_directory+"simple_yaw");

<<<<<<< HEAD
    dairlib::runSpiritJump<double>(
        *plant,
        x_traj, u_traj, l_traj,
        lc_traj, vc_traj,
        true,
        {5, 7, 5, 5, 5, 5} ,
        FLAGS_apexGoal,
        FLAGS_standHeight,
        FLAGS_foreAftDisplacement,
        false,
        true,
        false,
        true,
        0.6,
        3/1000.0,
        10/1000.0,
        100/1000.0,
        100000,
        1e-2,
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        FLAGS_data_directory+"jump_"+FLAGS_distance_name);
=======
        0,
        FLAGS_data_directory+"leap_"+FLAGS_distance_name+"_3",
        FLAGS_data_directory+"simple_leap");
=======
        1e-6,
=======
        1e-4,
>>>>>>> Code failing, moving to initial guess work
        1.0,
        FLAGS_data_directory+"simple_leap_3",
        FLAGS_data_directory+"simple_leap_2");
>>>>>>> Seemed to be doing alright
=======
>>>>>>> Seems to be working for basic pitch

>>>>>>> Not really making progress
  }
  std::cout<<"Running 3rd optimization"<<std::endl;
<<<<<<< HEAD
  // Fewer constraints, and higher tolerences
  dairlib::runSpiritJump<double>(
      *plant,
      x_traj, u_traj, l_traj,
      lc_traj, vc_traj,
      !FLAGS_minWork,
      {4, 4, 4, 4, 4} ,
      0,
      FLAGS_standHeight,
      FLAGS_foreAftDisplacement,
      false,
      true,
      false,
      true,
      2,
      3,
      10,
      0,
      1000,
      FLAGS_eps,
<<<<<<< HEAD
      FLAGS_tol,
      FLAGS_data_directory+"jump_"+FLAGS_distance_name+"_hq",
      FLAGS_data_directory+"jump_"+FLAGS_distance_name);
=======
      1e-4,
      0,
      FLAGS_data_directory+"leap_"+FLAGS_distance_name+"_hq");
>>>>>>> Not really making progress

  if (FLAGS_minWork){
    // Adding in work cost and constraints
    std::cout<<"Running 4th optimization"<<std::endl;
    dairlib::runSpiritJump<double>(
        *plant,
        x_traj, u_traj, l_traj,
        lc_traj, vc_traj,
        true,
        {7, 7, 7, 7, 7, 7} ,
        FLAGS_apexGoal,
        FLAGS_standHeight,
        FLAGS_foreAftDisplacement,
        false,
        false,
        false,
        true,
        1.5,
<<<<<<< HEAD
        FLAGS_inputCost/10,
=======
        500 * .5,
>>>>>>> Not really making progress
        FLAGS_velocityCost/10,
        100,
        FLAGS_mu,
        FLAGS_eps,
        FLAGS_tol,
<<<<<<< HEAD
        FLAGS_data_directory+"jump_"+FLAGS_distance_name+"_hq_work_option3",
        FLAGS_data_directory+"jump_"+FLAGS_distance_name+"_hq");
=======
        1.0,
        FLAGS_data_directory+"leap_"+FLAGS_distance_name+"_hq_work",
        FLAGS_data_directory+"leap_"+FLAGS_distance_name+"_hq");
>>>>>>> Not really making progress
  }
=======
>>>>>>> Seems to be working for basic pitch
}

