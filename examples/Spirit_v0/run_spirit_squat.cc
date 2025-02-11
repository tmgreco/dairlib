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
#include "drake/geometry/drake_visualizer.h"
#include "drake/solvers/solve.h"

#include "common/find_resource.h"
#include "systems/trajectory_optimization/dircon/dircon.h"
#include "multibody/kinematic/world_point_evaluator.h"
#include "solvers/nonlinear_constraint.h"
#include "multibody/kinematic/distance_evaluator.h"
#include "multibody/multibody_utils.h"
#include "multibody/visualization_utils.h"
#include "multibody/kinematic/kinematic_constraints.h"

#include "examples/Spirit_v0/animate_spirit.h"
#include "examples/Spirit_v0/spirit_utils.h"

DEFINE_double(duration, 1, "The stand duration");
DEFINE_double(front2BackToeDistance, 0.35, "Nominal distance between the back and front toes.");
DEFINE_double(side2SideToeDistance, 0.2, "Nominal distance between the back and front toes.");
DEFINE_double(bodyHeight, 0.104, "The spirit body start height (defined in URDF)");
DEFINE_double(lowerHeight,0.15, "The sitting height of the bottom of the robot");
DEFINE_double(upperHeight, 0.35, "The standing height.");
DEFINE_double(inputCost, 3, "The standing height.");
DEFINE_double(velocityCost, 10, "The standing height.");
DEFINE_double(eps, 1e-2, "The wiggle room.");
DEFINE_double(optTol, 1e-4,"Optimization Tolerance");
DEFINE_double(feasTol, 1e-4,"Feasibility Tolerance");
DEFINE_bool(autodiff, false, "Double or autodiff version");
DEFINE_bool(runInitTraj, false, "Animate initial conditions?");
// Parameters which enable dircon-improving features
DEFINE_bool(scale_constraint, true, "Scale the nonlinear constraint values");
DEFINE_bool(scale_variable, false, "Scale the decision variable");

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


template <typename T>
void addConstraints(const MultibodyPlant<T>& plant, Dircon<T>& trajopt){
  // Get position and velocity dictionaries 
  int num_knotpoints =10; ///DEBUG
  auto positions_map = multibody::makeNameToPositionsMap(plant);
  auto velocities_map = multibody::makeNameToVelocitiesMap(plant);
  auto x0 = trajopt.initial_state();
  auto xmid = trajopt.state_vars(0, (num_knotpoints - 1) / 2);
  // auto xf = trajopt.final_state();


  // Initial body positions
  trajopt.AddBoundingBoxConstraint(-FLAGS_eps, FLAGS_eps, x0(positions_map.at("base_x"))); // Give the initial condition room to choose the x_init position (helps with positive knee constraint)
  trajopt.AddBoundingBoxConstraint(-FLAGS_eps, FLAGS_eps, x0(positions_map.at("base_y")));
  trajopt.AddBoundingBoxConstraint(FLAGS_lowerHeight-FLAGS_eps, FLAGS_lowerHeight+FLAGS_eps, x0(positions_map.at("base_z")));
  
  // Mid body positions
  trajopt.AddBoundingBoxConstraint(-0, 0, xmid(positions_map.at("base_x"))); // Give the initial condition room to choose the x_init position (helps with positive knee constraint)
  trajopt.AddBoundingBoxConstraint(-0, 0, xmid(positions_map.at("base_y")));
  trajopt.AddBoundingBoxConstraint(FLAGS_upperHeight-FLAGS_eps, FLAGS_upperHeight+FLAGS_eps, xmid(positions_map.at("base_z")));

  return;
}

  /// See runSpiritSquat()
    // std::unique_ptr<MultibodyPlant<T>> plant_ptr,
    // MultibodyPlant<double>* plant_double_ptr,
    // std::unique_ptr<SceneGraph<double>> scene_graph_ptr,
    // double duration,
    // PiecewisePolynomial<double> init_x_traj,
    // PiecewisePolynomial<double> init_u_traj,
    // vector<PiecewisePolynomial<double>> init_l_traj,
    // vector<PiecewisePolynomial<double>> init_lc_traj,
    // vector<PiecewisePolynomial<double>> init_vc_traj
  ///
  /// Given an initial guess for state, control, and forces optimizes a standing behaviour for the spirit robot
template <typename T>
void runSpiritSquat(
    std::unique_ptr<MultibodyPlant<T>> plant_ptr,
    MultibodyPlant<double>* plant_double_ptr,
    std::unique_ptr<SceneGraph<double>> scene_graph_ptr,
    double duration,
    PiecewisePolynomial<double> init_x_traj,
    PiecewisePolynomial<double> init_u_traj,
    vector<PiecewisePolynomial<double>> init_l_traj,
    vector<PiecewisePolynomial<double>> init_lc_traj,
    vector<PiecewisePolynomial<double>> init_vc_traj
    ) {

  drake::systems::DiagramBuilder<double> builder;
  MultibodyPlant<T>& plant = *plant_ptr;
  SceneGraph<double>& scene_graph =
      *builder.AddSystem(std::move(scene_graph_ptr));
  
  // Get position and velocity dictionaries 
  auto positions_map = multibody::makeNameToPositionsMap(plant);
  auto velocities_map = multibody::makeNameToVelocitiesMap(plant);
  /// For Spirit front left leg->0, back left leg->1, front right leg->2, back right leg->3
  /// Get the frame of each toe and attach a world point to the toe tip (frame is at toe ball center).

  int num_legs = 4;
  double toeRadius = 0.02; // Radius of toe ball
  Vector3d toeOffset(toeRadius,0,0); // vector to "contact point"
  double mu = 1; // Coeff of friction
  

  int num_knotpoints_per_mode = 10; 
  

  auto sequence = DirconModeSequence<T>(plant);

  dairlib::ModeSequenceHelper msh;
  
  msh.addMode( // FIRST MODE 1
    (Eigen::Matrix<bool,1,4>() << 1,1,1,1 ).finished(), // contact bools 
    num_knotpoints_per_mode,  // number of knot points in the collocation
    Eigen::Vector3d::UnitZ(), // normal
    Eigen::Vector3d::Zero(),  // world offset
    mu //friction
    );

  // auto [modeVector, toeEvals, toeEvalSets] = createSpiritModeSequence(plant, msh.modes , msh.knots , msh.normals , msh.offsets, msh.mus, msh.minTs, msh.maxTs);
  auto [modeVector, toeEvals, toeEvalSets] = createSpiritModeSequence(plant, msh);
  
  for (auto& mode : modeVector){
    for (int i = 0; i < num_legs; i++ ){
      mode->MakeConstraintRelative(i,0);
      mode->MakeConstraintRelative(i,1);
    }
    mode->SetDynamicsScale(
      {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17}, 1.0 / 150.0);
    mode->SetKinVelocityScale(
      {0, 1, 2, 3}, {0, 1, 2}, 1.0 / 500.0 * 500 * 1 / 1);
    sequence.AddMode(mode.get());
  }          


 
  

  ///Setup trajectory optimization
  auto trajopt = Dircon<T>(sequence);

  // Set up Trajectory Optimization options
  trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(),
                           "Print file", "../snopt.out");
  trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(),
                           "Major iterations limit", 20000);
  trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(), "Iterations limit", 100000);
  trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(),
                           "Major optimality tolerance",
                           FLAGS_optTol);  // target optimality
  trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(), "Major feasibility tolerance", FLAGS_feasTol);
  trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(), "Verify level",
                           0);  // 0

    // Add duration constraint, currently constrained not bounded
  trajopt.AddDurationBounds(0, duration*2);
  // Initialize the trajectory control state and forces
  for (int j = 0; j < sequence.num_modes(); j++) {
    trajopt.drake::systems::trajectory_optimization::MultipleShooting::
        SetInitialTrajectory(init_u_traj, init_x_traj);
    trajopt.SetInitialForceTrajectory(j, init_l_traj[j], init_lc_traj[j],
                                      init_vc_traj[j]);
  }

  /// Setup all the optimization constraints 
  int num_knotpoints = trajopt.N(); // number of knot points total in the collocation
  int n_v = plant.num_velocities();
  // int n_q = plant.num_positions();
  auto u = trajopt.input();
  auto x = trajopt.state();
  auto x0 = trajopt.initial_state();
  auto xmid = trajopt.state_vars(0, (num_knotpoints - 1) / 2);
  auto xf = trajopt.final_state();
  addConstraints(plant,trajopt);

  // // Initial body positions
  // trajopt.AddBoundingBoxConstraint(-FLAGS_eps, FLAGS_eps, x0(positions_map.at("base_x"))); // Give the initial condition room to choose the x_init position (helps with positive knee constraint)
  // trajopt.AddBoundingBoxConstraint(-FLAGS_eps, FLAGS_eps, x0(positions_map.at("base_y")));
  // trajopt.AddBoundingBoxConstraint(FLAGS_lowerHeight-FLAGS_eps, FLAGS_lowerHeight+FLAGS_eps, x0(positions_map.at("base_z")));
  
  // // Mid body positions
  // trajopt.AddBoundingBoxConstraint(-0, 0, xmid(positions_map.at("base_x"))); // Give the initial condition room to choose the x_init position (helps with positive knee constraint)
  // trajopt.AddBoundingBoxConstraint(-0, 0, xmid(positions_map.at("base_y")));
  // trajopt.AddBoundingBoxConstraint(FLAGS_upperHeight-FLAGS_eps, FLAGS_upperHeight+FLAGS_eps, xmid(positions_map.at("base_z")));

  // Final Position Constraints
  bool isPeriodic = 1;
  if (isPeriodic){
    trajopt.AddLinearConstraint( x0(positions_map.at("base_x"))==xf(positions_map.at("base_x")) );
    trajopt.AddLinearConstraint( x0(positions_map.at("base_y"))==xf(positions_map.at("base_y")) );
    trajopt.AddLinearConstraint( x0(positions_map.at("base_z"))==xf(positions_map.at("base_z")) );
  }else{
    trajopt.AddBoundingBoxConstraint(-FLAGS_eps, FLAGS_eps, xf(positions_map.at("base_x"))); // Give the initial condition room to choose the x_init position (helps with positive knee constraint)
    trajopt.AddBoundingBoxConstraint(-FLAGS_eps, FLAGS_eps, xf(positions_map.at("base_y")));
    trajopt.AddBoundingBoxConstraint(FLAGS_lowerHeight-FLAGS_eps, FLAGS_lowerHeight+FLAGS_eps, xf(positions_map.at("base_z")));
  }

  
  
  /// Decide whether or not to constrain the body orientation at all knotpoints or only at the beginning and end
  bool isConstrainOrientation = false;
  if (isConstrainOrientation){
    // Body pose constraints (keep the body flat) at all the knotpoints including the ends
    for (int i = 0; i < num_knotpoints; i++) {
      auto xi = trajopt.state(i);
      trajopt.AddBoundingBoxConstraint(1, 1, xi(positions_map.at("base_qw")));
      trajopt.AddBoundingBoxConstraint(0, 0, xi(positions_map.at("base_qx")));
      trajopt.AddBoundingBoxConstraint(0, 0, xi(positions_map.at("base_qy")));
      trajopt.AddBoundingBoxConstraint(0, 0, xi(positions_map.at("base_qz")));
    }
  }else{
    // Body pose constraints (keep the body flat) at initial state
    trajopt.AddBoundingBoxConstraint(1, 1, x0(positions_map.at("base_qw")));
    trajopt.AddBoundingBoxConstraint(0, 0, x0(positions_map.at("base_qx")));
    trajopt.AddBoundingBoxConstraint(0, 0, x0(positions_map.at("base_qy")));
    trajopt.AddBoundingBoxConstraint(0, 0, x0(positions_map.at("base_qz")));

    // Body pose constraints (keep the body flat) at mid state
    trajopt.AddBoundingBoxConstraint(1, 1, xmid(positions_map.at("base_qw")));
    trajopt.AddBoundingBoxConstraint(0, 0, xmid(positions_map.at("base_qx")));
    trajopt.AddBoundingBoxConstraint(0, 0, xmid(positions_map.at("base_qy")));
    trajopt.AddBoundingBoxConstraint(0, 0, xmid(positions_map.at("base_qz")));

    // Body pose constraints (keep the body flat) at final state
    trajopt.AddBoundingBoxConstraint(1, 1, xf(positions_map.at("base_qw")));
    trajopt.AddBoundingBoxConstraint(0, 0, xf(positions_map.at("base_qx")));
    trajopt.AddBoundingBoxConstraint(0, 0, xf(positions_map.at("base_qy")));
    trajopt.AddBoundingBoxConstraint(0, 0, xf(positions_map.at("base_qz")));
  }


  /// Start/End velocity constraints of the behavior
  trajopt.AddBoundingBoxConstraint(VectorXd::Zero(n_v), VectorXd::Zero(n_v),
                                   x0.tail(n_v));
  trajopt.AddBoundingBoxConstraint(VectorXd::Zero(n_v), VectorXd::Zero(n_v),
                                   xmid.tail(n_v));
  trajopt.AddBoundingBoxConstraint(VectorXd::Zero(n_v), VectorXd::Zero(n_v),
                                   xf.tail(n_v));

  

  for (int i = 0; i < num_knotpoints; i++){
    auto xi = trajopt.state(i);  
    // legs lined up (front and back hips equal)
    trajopt.AddLinearConstraint( xi( positions_map.at("joint_8") ) == xi( positions_map.at("joint_9") ) );
    trajopt.AddLinearConstraint( xi( positions_map.at("joint_10") ) == xi( positions_map.at("joint_11") )  );
  }
  setSpiritSymmetry(plant, trajopt, "sagittal");
  setSpiritJointLimits(plant, trajopt);
  setSpiritActuationLimits(plant,trajopt);

 double upperLegLength = 0.206; // length of the upper leg link

  
// Quick check for allowed heights for Spirit
  try{
    if (FLAGS_lowerHeight<FLAGS_bodyHeight/2){
        throw "Body to close to floor" ;
    }
    if (FLAGS_upperHeight>=upperLegLength*2){
        throw "Body can't stand to that height without leaving the ground" ;
    }
  }catch(char const* exc){
      std::cerr<< exc <<std::endl;
  }

  ///Setup the traditional cost function
  const double R = FLAGS_inputCost;  // Cost on input effort
  const MatrixXd Q = FLAGS_velocityCost  * MatrixXd::Identity(n_v, n_v); // Cost on velocity
  trajopt.AddRunningCost( x.tail(n_v).transpose() * Q * x.tail(n_v) );
  trajopt.AddRunningCost( u.transpose()*R*u );
  
  ///Add regularization costs
  trajopt.AddRunningCost( x0(positions_map.at("base_x")) * 10 * x0(positions_map.at("base_x")) ); // x position cost
  // trajopt.AddRunningCost( (x.segment(12,2)-x.segment(14,2)).transpose() * 1 * (x.segment(12,2)-x.segment(14,2)) );
  // trajopt.AddRunningCost( (x.segment(16,2)-x.segment(18,2)).transpose() * 1 * (x.segment(16,2)-x.segment(18,2)) );

  

  /// Setup the visualization during the optimization
  int num_ghosts = 3;// Number of ghosts in visualization. NOTE: there are limitations on number of ghosts based on modes and knotpoints
  std::vector<unsigned int> visualizer_poses; // Ghosts for visualizing during optimization
  visualizer_poses.push_back(num_ghosts); 

  trajopt.CreateVisualizationCallback(
      dairlib::FindResourceOrThrow("examples/Spirit_v0/spirit_drake.urdf"),
      visualizer_poses, 0.2); // setup which URDF, how many poses, and alpha transparency 

  /// Run the optimization using your initial guess
  auto start = std::chrono::high_resolution_clock::now();
  const auto result = Solve(trajopt, trajopt.initial_guess());
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  std::cout << "Solve time:" << elapsed.count() <<std::endl;
  std::cout << "Cost:" << result.get_optimal_cost() <<std::endl;
  std::cout << (result.is_success() ? "Optimization Success" : "Optimization Fail") << std::endl;

  /// Run animation of the final trajectory
  const drake::trajectories::PiecewisePolynomial<double> pp_xtraj =
      trajopt.ReconstructStateTrajectory(result);
  multibody::connectTrajectoryVisualizer(plant_double_ptr,
      &builder, &scene_graph, pp_xtraj);
  auto diagram = builder.Build();
  while (true) {
    
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
      dairlib::FindResourceOrThrow("examples/Spirit_v0/spirit_drake.urdf");

  parser.AddModelFromFile(full_name);
  parser_vis.AddModelFromFile(full_name);
  
  plant->mutable_gravity_field().set_gravity_vector(-9.81 *
      Eigen::Vector3d::UnitZ());

  plant->Finalize();
  plant_vis->Finalize();
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(plant->num_positions() +
                       plant->num_velocities());

  int nu = plant->num_actuators();
  int nx = plant->num_positions() + plant->num_velocities();
  int nq = plant->num_positions();
  int nv = plant->num_velocities();
  int N = 20; // number of timesteps

  std::vector<MatrixXd> init_x;
  std::vector<MatrixXd> init_u;
  std::vector<PiecewisePolynomial<double>> init_l_traj;
  std::vector<PiecewisePolynomial<double>> init_lc_traj;
  std::vector<PiecewisePolynomial<double>> init_vc_traj;

  // Initialize state trajectory
  std::vector<double> init_time;

  VectorXd xInit(nx);
  VectorXd xMid(nx);
  VectorXd xState(nx);
  xInit = Eigen::VectorXd::Zero(plant->num_positions() + plant->num_velocities());
  xMid = Eigen::VectorXd::Zero(plant->num_positions() + plant->num_velocities());
  xState = Eigen::VectorXd::Zero(plant->num_positions() + plant->num_velocities());

  auto positions_map = dairlib::multibody::makeNameToPositionsMap(*plant);
  auto velocities_map = dairlib::multibody::makeNameToVelocitiesMap(*plant);
  auto actuators_map = dairlib::multibody::makeNameToActuatorsMap(*plant);
  int num_joints = 12;

  // Print joint dictionary
  std::cout<<"**********************Joints***********************"<<std::endl;
  for (auto const& element : positions_map)
    std::cout << element.first << " = " << element.second << std::endl;
  for (auto const& element : velocities_map)
    std::cout << element.first << " = " << element.second << std::endl;
  for (auto const& element : actuators_map)
    std::cout << element.first << " = " << element.second << std::endl;
  std::cout<<"***************************************************"<<std::endl;
    
  dairlib::nominalSpiritStand( *plant, xInit,  FLAGS_lowerHeight); //Update xInit
  dairlib::nominalSpiritStand( *plant, xMid,  FLAGS_upperHeight); //Update xMid
  
  VectorXd deltaX(nx);
  VectorXd averageV(nx);
  deltaX = xMid-xInit;
  averageV = 2* deltaX / FLAGS_duration;
  xInit.tail(nv-3) = (averageV.head(nq)).tail(nq-4); //Ignoring Orientation make velocity the average

  double time = 0;
  double dt = FLAGS_duration/(N-1);

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
  auto init_x_traj = PiecewisePolynomial<double>::ZeroOrderHold(init_time, init_x);
  auto init_u_traj = PiecewisePolynomial<double>::ZeroOrderHold(init_time, init_u);

  
  // Four contacts so forces are 12 dimensional
  Eigen::VectorXd init_l_vec(12);
  // Initial guess
  init_l_vec << 0, 0, 3*9.81, 0, 0, 3*9.81, 0, 0, 3*9.81, 0, 0, 3*9.81; //gravity and mass distributed
  
  //Initialize force trajectories
  int num_modes = 1; // MAKE DYNAMIC DEBUG
  for (int j = 0; j < num_modes; j++) {    
    std::vector<MatrixXd> init_l_j;
    std::vector<MatrixXd> init_lc_j;
    std::vector<MatrixXd> init_vc_j;
    std::vector<double> init_time_j;
    for (int i = 0; i < N; i++) {
      init_time_j.push_back(i*FLAGS_duration/(N-1));
      init_l_j.push_back(init_l_vec);
      init_lc_j.push_back(init_l_vec);
      init_vc_j.push_back(VectorXd::Zero(12));
    }

    auto init_l_traj_j = PiecewisePolynomial<double>::ZeroOrderHold(init_time_j,init_l_j);
    auto init_lc_traj_j = PiecewisePolynomial<double>::ZeroOrderHold(init_time_j,init_lc_j);
    auto init_vc_traj_j = PiecewisePolynomial<double>::ZeroOrderHold(init_time_j,init_vc_j);

    init_l_traj.push_back(init_l_traj_j);
    init_lc_traj.push_back(init_lc_traj_j);
    init_vc_traj.push_back(init_vc_traj_j);
  }

  // if (FLAGS_autodiff) {
  //   std::unique_ptr<MultibodyPlant<drake::AutoDiffXd>> plant_autodiff =
  //       drake::systems::System<double>::ToAutoDiffXd(*plant);
  //   dairlib::runSpiritSquat<drake::AutoDiffXd>(
  //     std::move(plant_autodiff), plant_vis.get(), std::move(scene_graph),
  //     FLAGS_duration, init_x_traj, init_u_traj, init_l_traj,
  //     init_lc_traj, init_vc_traj);
  // } else 
  if (FLAGS_runInitTraj){
    dairlib::runAnimate<double>(
      std::move(plant), plant_vis.get(), std::move(scene_graph), init_x_traj);
  }else {
    dairlib::runSpiritSquat<double>(
      std::move(plant), plant_vis.get(), std::move(scene_graph),
      FLAGS_duration, init_x_traj, init_u_traj, init_l_traj,
      init_lc_traj, init_vc_traj);
  }

}

