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
#include "drake/math/orthonormal_basis.h"
#include <drake/solvers/choose_best_solver.h>
#include "drake/solvers/mathematical_program.h"

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

DEFINE_double(standHeight, 0.2, "The standing height.");
DEFINE_double(foreAftDisplacement, 1.0, "The fore-aft displacement.");
DEFINE_double(apexGoal, 0.45, "Apex state goal");
DEFINE_double(eps, 1e-2, "The wiggle room.");
DEFINE_double(tol, 2e-1, "Optimization Tolerance");
DEFINE_double(mu, 1, "coefficient of friction");

DEFINE_string(data_directory, "/home/jdcap/dairlib/examples/Spirit/saved_trajectories/parkour_experiments/",
              "directory to save/read data");
DEFINE_bool(skipInitialOptimization, true, "skip first optimizations?"); 

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
//TODO: finish fixing the inplaceParkourBound
/// Uses IK to generate a bad initial guess for an inplace bound
/// @param plant the multibody plant
/// @param x_traj[out] the vector for state trajectories for each mode
/// @param u_traj[out] control trajectory
/// @param l_traj[out] the contact force trajectory
/// @param lc_traj[out] slack contact force trajectory
/// @param vc_traj[out] slack contact velocity trajectory
template <typename T>
void badInplaceBound(MultibodyPlant<T>& plant,
                    vector<PiecewisePolynomial<double>>& x_traj,
                    PiecewisePolynomial<double>& u_traj,
                    vector<PiecewisePolynomial<double>>& l_traj,
                    vector<PiecewisePolynomial<double>>& lc_traj,
                    vector<PiecewisePolynomial<double>>& vc_traj) {

  double stand_height = FLAGS_standHeight;
  double pitch_lo = -0.3;
  double apex_height = FLAGS_apexGoal;
  double duration = 0.3;

  std::vector<MatrixXd> x_points;
  VectorXd x_const;
  //Stance 
  dairlib::ikSpiritStand(plant, x_const, {true, true, true, true}, stand_height, 0.2, 0, 0);
  x_points.push_back(x_const);
  dairlib::ikSpiritStand(plant, x_const, {true, true, true, true}, stand_height+0.02, 0.2, 0, pitch_lo/2.0);
  x_points.push_back(x_const);

  x_traj.push_back(PiecewisePolynomial<double>::FirstOrderHold({0,duration/3.0},x_points));
  x_points.clear();

  //Rear stance
  dairlib::ikSpiritStand(plant, x_const, {true, true, true, true}, stand_height+0.02, 0.2, 0, pitch_lo/2.0);
  x_points.push_back(x_const);
  dairlib::ikSpiritStand(plant, x_const, {false, true, false, true}, stand_height+0.05, 0.2, 0, pitch_lo);
  x_points.push_back(x_const);

  x_traj.push_back(PiecewisePolynomial<double>::FirstOrderHold({duration/3.0, 2 * duration/3.0},x_points));
  x_points.clear();

  //Flight 1
  dairlib::ikSpiritStand(plant, x_const, {false, true, false, true}, stand_height+0.05, 0.2, 0, pitch_lo);
  x_points.push_back(x_const);
  dairlib::ikSpiritStand(plant, x_const, {false, false, false, false}, apex_height, 0.2, 0, 0);
  x_points.push_back(x_const);

  x_traj.push_back(PiecewisePolynomial<double>::FirstOrderHold({2 * duration/3.0, 3 * duration/3.0},x_points));
  x_points.clear();

  //Flight 2
  dairlib::ikSpiritStand(plant, x_const, {false, false, false, false}, apex_height, 0.2, 0, 0);
  x_points.push_back(x_const);
  dairlib::ikSpiritStand(plant, x_const, {true, false, true, false}, stand_height+0.05, 0.2, 0, -pitch_lo);
  x_points.push_back(x_const);

  x_traj.push_back(PiecewisePolynomial<double>::FirstOrderHold({3 * duration/3.0, 4 * duration/3.0},x_points));
  x_points.clear();

  //Front stance
  dairlib::ikSpiritStand(plant, x_const, {true, false, true, false}, stand_height+0.05, 0.2, 0, -pitch_lo);
  x_points.push_back(x_const);
  dairlib::ikSpiritStand(plant, x_const, {true, true, true, true}, stand_height+0.02, 0.2, 0, -pitch_lo/2.0);
  x_points.push_back(x_const);

  x_traj.push_back(PiecewisePolynomial<double>::FirstOrderHold({4 * duration/3.0, 5 * duration/3.0},x_points));
  x_points.clear();

  //stance
  dairlib::ikSpiritStand(plant, x_const, {true, true, true, true}, stand_height+0.02, 0.2, 0, -pitch_lo/2.0);
  x_points.push_back(x_const);
  dairlib::ikSpiritStand(plant, x_const, {true, true, true, true}, stand_height, 0.2, 0, 0);
  x_points.push_back(x_const);

  x_traj.push_back(PiecewisePolynomial<double>::FirstOrderHold({5 * duration/3.0, 6 * duration/3.0},x_points));
  x_points.clear();

  //END OF NORMAL BOUND

  //Rear stance
  dairlib::ikSpiritStand(plant, x_const, {true, true, true, true}, stand_height+0.02, 0.2, 0, pitch_lo/2.0);
  x_points.push_back(x_const);
  dairlib::ikSpiritStand(plant, x_const, {false, true, false, true}, stand_height+0.05, 0.2, 0, pitch_lo);
  x_points.push_back(x_const);

  x_traj.push_back(PiecewisePolynomial<double>::FirstOrderHold({6 * duration/3.0, 7 * duration/3.0},x_points));
  x_points.clear();

  //Flight 1
  dairlib::ikSpiritStand(plant, x_const, {false, true, false, true}, stand_height+0.05, 0.2, 0, pitch_lo);
  x_points.push_back(x_const);
  dairlib::ikSpiritStand(plant, x_const, {false, false, false, false}, apex_height, 0.2, 0, 0);
  x_points.push_back(x_const);

  x_traj.push_back(PiecewisePolynomial<double>::FirstOrderHold({7 * duration/3.0, 8 * duration/3.0},x_points));
  x_points.clear();

  //Flight 2
  dairlib::ikSpiritStand(plant, x_const, {false, false, false, false}, apex_height, 0.2, 0, 0);
  x_points.push_back(x_const);
  dairlib::ikSpiritStand(plant, x_const, {true, false, true, false}, stand_height+0.05, 0.2, 0, -pitch_lo);
  x_points.push_back(x_const);

  x_traj.push_back(PiecewisePolynomial<double>::FirstOrderHold({8 * duration/3.0, 9 * duration/3.0},x_points));
  x_points.clear();
  
  //Front stance
  dairlib::ikSpiritStand(plant, x_const, {true, false, true, false}, stand_height+0.05, 0.2, 0, -pitch_lo);
  x_points.push_back(x_const);
  dairlib::ikSpiritStand(plant, x_const, {true, true, true, true}, stand_height+0.02, 0.2, 0, -pitch_lo/2.0);
  x_points.push_back(x_const);

  x_traj.push_back(PiecewisePolynomial<double>::FirstOrderHold({9 * duration/3.0, 10 * duration/3.0},x_points));
  x_points.clear();

  //stance
  dairlib::ikSpiritStand(plant, x_const, {true, true, true, true}, stand_height+0.02, 0.2, 0, -pitch_lo/2.0);
  x_points.push_back(x_const);
  dairlib::ikSpiritStand(plant, x_const, {true, true, true, true}, stand_height, 0.2, 0, 0);
  x_points.push_back(x_const);

  x_traj.push_back(PiecewisePolynomial<double>::FirstOrderHold({10 * duration/3.0, 11 * duration/3.0},x_points));
  x_points.clear();


  // Create control initial guess (zeros)
  std::vector<MatrixXd> u_points;
  u_points.push_back(MatrixXd::Zero( plant.num_actuators(), 1));
  u_points.push_back(MatrixXd::Zero( plant.num_actuators(), 1));

  u_traj = PiecewisePolynomial<double>::FirstOrderHold({0, 11 * duration/3.0},u_points);

  // Contact force initial guess
  // Four contacts so forces are 12 dimensional
  Eigen::VectorXd init_l_vec(12);
  // Initial guess
  init_l_vec << 0, 0, 12*9.81, 0, 0, 12*9.81, 0, 0, 12*9.81, 0, 0, 12*9.81; //gravity and mass distributed
  //FIXME: needs testing Figure out the correct leg pattern for this section and above
  Eigen::VectorXd init_l_vec_rear_stance(12);
  init_l_vec_rear_stance << 0, 0, 0, 0, 0, 24*9.81, 0, 0, 0, 0, 0, 24*9.81; //gravity and mass distributed

  Eigen::VectorXd init_l_vec_front_stance(12);
  init_l_vec_front_stance << 0, 0, 24*9.81, 0, 0, 0, 0, 0, 24*9.81, 0, 0, 0; //gravity and mass distributed

  //Stance
  int N = 10;
  std::vector<MatrixXd> init_l_j;
  std::vector<MatrixXd> init_lc_j;
  std::vector<MatrixXd> init_vc_j;
  std::vector<double> init_time_j;

  // stance
  init_l_j.clear();
  init_lc_j.clear();
  init_vc_j.clear();
  init_time_j.clear();
  for (int i = 0; i < N; i++) {
    init_time_j.push_back(i * duration /3.0/ (N - 1));
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

  // rear stance
  init_l_j.clear();
  init_lc_j.clear();
  init_vc_j.clear();
  init_time_j.clear();
  for (int i = 0; i < N; i++) {
    init_time_j.push_back(i * duration /3.0/ (N - 1) + duration/3.0);
    init_l_j.push_back(init_l_vec_rear_stance);
    init_lc_j.push_back(init_l_vec_rear_stance);
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
    init_time_j.push_back(i * duration /3.0/ (N - 1) +2.0 * duration/3.0);
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

  // Flight
  init_l_j.clear();
  init_lc_j.clear();
  init_vc_j.clear();
  init_time_j.clear();
  for (int i = 0; i < N; i++) {
    init_time_j.push_back(i * duration /3.0/ (N - 1) +3.0 * duration/3.0);
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

  // front stance
  init_l_j.clear();
  init_lc_j.clear();
  init_vc_j.clear();
  init_time_j.clear();
  for (int i = 0; i < N; i++) {
    init_time_j.push_back(i * duration /3.0/ (N - 1) + 4 * duration/3.0);
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

  
  // stance
  init_l_j.clear();
  init_lc_j.clear();
  init_vc_j.clear();
  init_time_j.clear();
  for (int i = 0; i < N; i++) {
    init_time_j.push_back(i * duration /3.0/ (N - 1)+ 5 * duration/3.0);
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

  // rear stance
  init_l_j.clear();
  init_lc_j.clear();
  init_vc_j.clear();
  init_time_j.clear();
  for (int i = 0; i < N; i++) {
    init_time_j.push_back(i * duration /3.0/ (N - 1) + 6 * duration/3.0);
    init_l_j.push_back(init_l_vec_rear_stance);
    init_lc_j.push_back(init_l_vec_rear_stance);
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
    init_time_j.push_back(i * duration /3.0/ (N - 1) +7.0 * duration/3.0);
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

  // Flight
  init_l_j.clear();
  init_lc_j.clear();
  init_vc_j.clear();
  init_time_j.clear();
  for (int i = 0; i < N; i++) {
    init_time_j.push_back(i * duration /3.0/ (N - 1) +8.0 * duration/3.0);
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

  // front stance
  init_l_j.clear();
  init_lc_j.clear();
  init_vc_j.clear();
  init_time_j.clear();
  for (int i = 0; i < N; i++) {
    init_time_j.push_back(i * duration /3.0/ (N - 1) + 9* duration/3.0);
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

  // stance
  init_l_j.clear();
  init_lc_j.clear();
  init_vc_j.clear();
  init_time_j.clear();
  for (int i = 0; i < N; i++) {
    init_time_j.push_back(i * duration /3.0/ (N - 1) + 10  * duration/3.0);
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
/// Add a cost on a specific mode for a specific subset of joints, to be used to add a cost on the legs in flight
/// @param plant the plant
/// @param trajopt the trajectory optimization object
/// @param cost_actuation a cost on actuation squared
/// @param cost_velocity a cost on velocity squared
/// @param joints a vector of joints corresponding to which joints to apply the cost to
/// @param mode_index the mode to apply the cost to
template <typename T>
void addCostLegs(MultibodyPlant<T>& plant,
                 dairlib::systems::trajectory_optimization::Dircon<T>& trajopt,
                 const double cost_actuation,
                 const double cost_velocity,
                 const std::vector<double>& joints,
                 const int mode_index){
  auto velocities_map = multibody::makeNameToVelocitiesMap(plant);
  auto actuator_map = multibody::makeNameToActuatorsMap(plant);
  int n_q = plant.num_positions();

  for(int knot_index = 0; knot_index < trajopt.mode_length(mode_index)-1; knot_index++){
    for(int joint_index : joints){
      // Get lower and upper knot velocities
      auto vel  = trajopt.state_vars(mode_index, knot_index)(n_q + velocities_map.at("joint_" + std::to_string(joint_index) +"dot"));
      auto vel_up = trajopt.state_vars(mode_index, knot_index+1)(n_q + velocities_map.at("joint_" + std::to_string(joint_index) +"dot"));

      drake::symbolic::Expression hi = trajopt.timestep(trajopt.get_mode_start(mode_index) + knot_index)[0];

      // Loop through and calculate sum of velocities squared
      drake::symbolic::Expression vel_sq = vel * vel;
      drake::symbolic::Expression vel_sq_up = vel_up * vel_up;

      // Add cost
      trajopt.AddCost(hi/2.0 * cost_velocity * (vel_sq + vel_sq_up));

      auto act  = trajopt.input(trajopt.get_mode_start(mode_index) + knot_index)(actuator_map.at("motor_" + std::to_string(joint_index)));
      auto act_up = trajopt.input(trajopt.get_mode_start(mode_index) + knot_index+1)(actuator_map.at("motor_" + std::to_string(joint_index)));

      drake::symbolic::Expression act_sq = act * act;
      drake::symbolic::Expression act_sq_up = act_up * act_up;

      trajopt.AddCost(hi/2.0 * cost_actuation * (act_sq + act_sq_up));
    }
  }
}



template <typename T>
void addOffsetConstraint(MultibodyPlant<T>& plant,
                dairlib::systems::trajectory_optimization::Dircon<T>& trajopt,
                const Eigen::Ref<const drake::solvers::VectorXDecisionVariable>& vars, 
                Eigen::Vector3d offset,
                Eigen::Vector3d normal = Eigen::Vector3d::UnitZ(),
                double dist_along_normal = -1, 
                double eps = 0.01 ){
  
  Eigen::Matrix3d wRc = drake::math::ComputeBasisFromAxis(2, normal);//Rotation world to contact frame
  Eigen::Vector3d offset_c = wRc.transpose()*offset;
  Eigen::Vector3d lb; //Lower Bound
  Eigen::Vector3d ub; //Upper Bound
  Eigen::Vector3d tangent_error = Eigen::Vector3d::UnitX() * eps + Eigen::Vector3d::UnitY() * eps;
  if (dist_along_normal==std::numeric_limits<double>::infinity() || dist_along_normal < 0 ){
    Eigen::Vector3d no_upper_height;
    no_upper_height <<0 , 0, 100;
    lb = offset_c - tangent_error;
    ub = offset_c + tangent_error + no_upper_height;
  }
  else{
    Eigen::Vector3d height_vect = Eigen::Vector3d::UnitZ()*dist_along_normal;
    lb = offset_c + height_vect - tangent_error - Eigen::Vector3d::UnitZ()*eps;
    ub = offset_c + height_vect + tangent_error + Eigen::Vector3d::UnitZ()*eps;
  }
  std::cout<<"LINEAR CONSTRAINT?"<<std::endl;
  std::cout<<wRc.transpose() <<std::endl<<lb<<std::endl <<ub<<std::endl;
  trajopt.AddLinearConstraint(wRc.transpose(), lb, ub, vars);
  std::cout<<"LINEAR CONSTRAINT?"<<std::endl;
  
}



//TODO: Write Front leg based/ shoulder midpoint constraints
//      Can we write a linear, stay on vector constraint? So far require magnitude/square (not linear) 
//      I think a small cube around the position will do
//      or find the tangent directions (function exists) and make both dot products < epsilon


/// addCost, adds the cost to the trajopt jump problem. See runSpiritJump for a description of the inputs
template <typename T>
std::vector<drake::solvers::Binding<drake::solvers::Cost>> addCost(MultibodyPlant<T>& plant,
             dairlib::systems::trajectory_optimization::Dircon<T>& trajopt,
             const double cost_actuation,
             const double cost_velocity,
             const double cost_velocity_legs_flight,
             const double cost_actuation_legs_flight,
             const double cost_time,
             const double cost_work){
  auto u   = trajopt.input();
  auto x   = trajopt.state();

  // Setup the traditional cost function
  trajopt.AddRunningCost( u.transpose()*cost_actuation*u);
  trajopt.AddVelocityCost(cost_velocity);

  // trajopt.AddRunningCost(cost_time);

  // Hard code which joints are in flight for which mode //FIXME using quad_mod_seq
  addCostLegs(plant, trajopt, cost_velocity_legs_flight, cost_actuation_legs_flight, {0, 1, 4, 5, 8, 10}, 1);
  addCostLegs(plant, trajopt, cost_velocity_legs_flight, cost_actuation_legs_flight, {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}, 2);
  addCostLegs(plant, trajopt, cost_velocity_legs_flight, cost_actuation_legs_flight, {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}, 3);
  addCostLegs(plant, trajopt, cost_velocity_legs_flight, cost_actuation_legs_flight, {2, 3, 6, 7, 10, 11}, 4);
  addCostLegs(plant, trajopt, cost_velocity_legs_flight, cost_actuation_legs_flight, {0, 1, 4, 5, 8, 10}, 6);
  addCostLegs(plant, trajopt, cost_velocity_legs_flight, cost_actuation_legs_flight, {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}, 7);
  addCostLegs(plant, trajopt, cost_velocity_legs_flight, cost_actuation_legs_flight, {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}, 8);
  addCostLegs(plant, trajopt, cost_velocity_legs_flight, cost_actuation_legs_flight, {2, 3, 6, 7, 10, 11}, 9);

  return AddWorkCost(plant, trajopt, cost_work);
} // Function


// addConstraints, adds constraints to the trajopt jump problem. See runSpiritJump for a description of the inputs
template <typename T>
void addConstraints(MultibodyPlant<T>& plant,
             dairlib::systems::trajectory_optimization::Dircon<T>& trajopt,
                    const double initial_height,
                    const double apex_height,
                    const double fore_aft_displacement,
                    const double pitch_magnitude_lo,
                    const double pitch_magnitude_apex,
                    const double max_duration,
                    const double eps){

  // Get position and velocity dictionaries
  auto positions_map = multibody::makeNameToPositionsMap(plant);
  auto velocities_map = multibody::makeNameToVelocitiesMap(plant);


  // General constraints
  setSpiritJointLimits(plant, trajopt);
  setSpiritActuationLimits(plant, trajopt);
  trajopt.AddDurationBounds(0, max_duration);

  /// Setup all the optimization constraints
  int n_q = plant.num_positions();
  int n_v = plant.num_velocities();
  auto          x0  = trajopt.initial_state();
  auto xlo_front_1  = trajopt.state_vars(1,0);
  auto  xlo_rear_1  = trajopt.state_vars(2,0);
  auto     xapex_1  = trajopt.state_vars(3,0);
  auto xtd_front_1  = trajopt.state_vars(4,0);
  auto  xtd_rear_1  = trajopt.state_vars(5,0);
  auto xlo_front_2  = trajopt.state_vars(6,0);
  auto  xlo_rear_2  = trajopt.state_vars(7,0);
  auto     xapex_2  = trajopt.state_vars(8,0);
  auto xtd_front_2  = trajopt.state_vars(9,0);
  auto  xtd_rear_2  = trajopt.state_vars(10,0);
  auto          xf  = trajopt.final_state();
  double pitch;


  /// Initial constraint
  // xy position
  trajopt.AddBoundingBoxConstraint(0, 0, x0(positions_map.at("base_x"))); // Give the initial condition room to choose the x_init position
  trajopt.AddBoundingBoxConstraint( -eps, eps, x0( positions_map.at("base_y")));
  // Nominal stand
  nominalSpiritStandConstraint(plant,trajopt,initial_height, {0}, eps);
  // Body pose constraints (keep the body flat) at initial state
  trajopt.AddBoundingBoxConstraint(1, 1,  x0(positions_map.at("base_qw")));
  trajopt.AddBoundingBoxConstraint(0, 0,  x0(positions_map.at("base_qx")));
  trajopt.AddBoundingBoxConstraint(0, 0,  x0(positions_map.at("base_qy")));
  trajopt.AddBoundingBoxConstraint(0, 0,  x0(positions_map.at("base_qz")));
  // Initial  velocity
  trajopt.AddBoundingBoxConstraint(VectorXd::Zero(n_v), VectorXd::Zero(n_v), x0.tail(n_v));

  std::cout<<positions_map.at("base_x")<<positions_map.at("base_y")<<positions_map.at("base_z")<<std::endl;


  /// Final constraints
  // x,y position
  trajopt.AddBoundingBoxConstraint(0-eps, 0+eps, xf(positions_map.at("base_y")));
  if (fore_aft_displacement >= 0){
    trajopt.AddBoundingBoxConstraint(fore_aft_displacement-eps, fore_aft_displacement+eps, xf(positions_map.at("base_x")));
  }
  // Nominal stand
  nominalSpiritStandConstraint(plant,trajopt,initial_height, {trajopt.N()-1}, eps);
  // Body pose constraints (keep the body flat) at final state
  trajopt.AddBoundingBoxConstraint(1, 1, xf(positions_map.at("base_qw")));
  trajopt.AddBoundingBoxConstraint(0, 0, xf(positions_map.at("base_qx")));
  trajopt.AddBoundingBoxConstraint(0, 0, xf(positions_map.at("base_qy")));
  trajopt.AddBoundingBoxConstraint(0, 0, xf(positions_map.at("base_qz")));
  // Zero velocity
  trajopt.AddBoundingBoxConstraint(VectorXd::Zero(n_v), VectorXd::Zero(n_v), xf.tail(n_v));




  /// Middle Constraint
  // x,y position
  // trajopt.AddBoundingBoxConstraint(0-eps, 0+eps, xtd_front_1(positions_map.at("base_y")));
  // if (fore_aft_displacement >= 0){
  //   trajopt.AddBoundingBoxConstraint(fore_aft_displacement/2-eps, fore_aft_displacement/2+eps, xtd_front_1(positions_map.at("base_x")));
  // }

  pitch = abs(pitch_magnitude_apex);
  addOffsetConstraint(plant, trajopt, xtd_rear_1.head(7).tail(3), Eigen::Vector3d::UnitX()*fore_aft_displacement/2,Eigen::Vector3d::UnitZ());
  trajopt.AddBoundingBoxConstraint( cos(pitch/2.0),              1, xtd_rear_1(positions_map.at("base_qw")));
  trajopt.AddBoundingBoxConstraint(           -eps,            eps, xtd_rear_1(positions_map.at("base_qx")));
  trajopt.AddBoundingBoxConstraint(-sin(pitch/2.0), sin(pitch/2.0), xtd_rear_1(positions_map.at("base_qy")));
  trajopt.AddBoundingBoxConstraint(           -eps,            eps, xtd_rear_1(positions_map.at("base_qz")));


  /// First Front Legs LO constraints
  // Limit magnitude of pitch
  pitch = abs(pitch_magnitude_lo);
  trajopt.AddBoundingBoxConstraint( cos(pitch/2.0),              1, xlo_rear_1(positions_map.at("base_qw")));
  trajopt.AddBoundingBoxConstraint(           -eps,            eps, xlo_rear_1(positions_map.at("base_qx")));
  trajopt.AddBoundingBoxConstraint(-sin(pitch/2.0), sin(pitch/2.0), xlo_rear_1(positions_map.at("base_qy")));
  trajopt.AddBoundingBoxConstraint(           -eps,            eps, xlo_rear_1(positions_map.at("base_qz")));



  /// First Apex Flight constraints
  // Height
  if (apex_height > 0.15)
    trajopt.AddBoundingBoxConstraint(apex_height-1e-2, 1, xapex_1(positions_map.at("base_z")));
  // Limit magnitude of pitch
  pitch = abs(pitch_magnitude_apex);
  trajopt.AddBoundingBoxConstraint( cos(pitch/2.0),              1, xapex_1(positions_map.at("base_qw")));
  trajopt.AddBoundingBoxConstraint(           -eps,            eps, xapex_1(positions_map.at("base_qx")));
  trajopt.AddBoundingBoxConstraint(-sin(pitch/2.0), sin(pitch/2.0), xapex_1(positions_map.at("base_qy")));
  trajopt.AddBoundingBoxConstraint(           -eps,            eps, xapex_1(positions_map.at("base_qz")));
  
  // Velocity
  trajopt.AddBoundingBoxConstraint(0, 0, xapex_1(n_q + velocities_map.at("base_vz")));



  /// First Front Legs LO constraints
  // Limit magnitude of pitch
  pitch = abs(pitch_magnitude_lo);
  trajopt.AddBoundingBoxConstraint( cos(pitch/2.0),              1, xtd_front_1(positions_map.at("base_qw")));
  trajopt.AddBoundingBoxConstraint(           -eps,            eps, xtd_front_1(positions_map.at("base_qx")));
  trajopt.AddBoundingBoxConstraint(-sin(pitch/2.0), sin(pitch/2.0), xtd_front_1(positions_map.at("base_qy")));
  trajopt.AddBoundingBoxConstraint(           -eps,            eps, xtd_front_1(positions_map.at("base_qz")));




  /// First Front Legs LO constraints
  // Limit magnitude of pitch
  pitch = abs(pitch_magnitude_lo);
  trajopt.AddBoundingBoxConstraint( cos(pitch/2.0),              1, xlo_rear_2(positions_map.at("base_qw")));
  trajopt.AddBoundingBoxConstraint(           -eps,            eps, xlo_rear_2(positions_map.at("base_qx")));
  trajopt.AddBoundingBoxConstraint(-sin(pitch/2.0), sin(pitch/2.0), xlo_rear_2(positions_map.at("base_qy")));
  trajopt.AddBoundingBoxConstraint(           -eps,            eps, xlo_rear_2(positions_map.at("base_qz")));



  /// First Apex Flight constraints
  // Height
  if (apex_height > 0.15)
    trajopt.AddBoundingBoxConstraint(apex_height-1e-2, 1, xapex_2(positions_map.at("base_z")));
  // Limit magnitude of pitch
  pitch = abs(pitch_magnitude_apex);
  trajopt.AddBoundingBoxConstraint( cos(pitch/2.0),              1, xapex_2(positions_map.at("base_qw")));
  trajopt.AddBoundingBoxConstraint(           -eps,            eps, xapex_2(positions_map.at("base_qx")));
  trajopt.AddBoundingBoxConstraint(-sin(pitch/2.0), sin(pitch/2.0), xapex_2(positions_map.at("base_qy")));
  trajopt.AddBoundingBoxConstraint(           -eps,            eps, xapex_2(positions_map.at("base_qz")));
  
  // Velocity
  trajopt.AddBoundingBoxConstraint(0, 0, xapex_1(n_q + velocities_map.at("base_vz")));



  /// First Front Legs LO constraints
  // Limit magnitude of pitch
  pitch = abs(pitch_magnitude_lo);
  trajopt.AddBoundingBoxConstraint( cos(pitch/2.0),              1, xtd_front_2(positions_map.at("base_qw")));
  trajopt.AddBoundingBoxConstraint(           -eps,            eps, xtd_front_2(positions_map.at("base_qx")));
  trajopt.AddBoundingBoxConstraint(-sin(pitch/2.0), sin(pitch/2.0), xtd_front_2(positions_map.at("base_qy")));
  trajopt.AddBoundingBoxConstraint(           -eps,            eps, xtd_front_2(positions_map.at("base_qz")));







  /// Constraints on all points
  for (int i = 0; i < trajopt.N(); i++){
    auto xi = trajopt.state(i);
    // Height
    trajopt.AddBoundingBoxConstraint( 0.15, 2, xi( positions_map.at("base_z")));
    trajopt.AddBoundingBoxConstraint( -eps, eps, xi( n_q+velocities_map.at("base_vy")));
  }

}



//TODO: remove this mode sequence helper and add the new one. reimplement/overlaod current function.
/// Generates a mode sequence helper
/// @param[out] the msh
/// @param mu the coefficient of friction for each mode
/// @param num_knot_points[in] vector of num knot points per mode
void getModeSequenceHelper(dairlib::ModeSequenceHelper& msh,
                           const double mu,
                           std::vector<int> num_knot_points){
  msh.addMode( // Stance
      (Eigen::Matrix<bool,1,4>() << true,  true,  true,  true).finished(), // contact bools
      num_knot_points[0],  // number of knot points in the collocation
      Eigen::Vector3d::UnitZ(), // normal
      Eigen::Vector3d::Zero(),  // world offset
      mu, //friction
      0.02,
      0.5
  );
  msh.addMode( // Rear stance
      (Eigen::Matrix<bool,1,4>() << false,  true,  false,  true).finished(), // contact bools
      num_knot_points[1],  // number of knot points in the collocation
      Eigen::Vector3d::UnitZ(), // normal
      Eigen::Vector3d::Zero(),  // world offset
      mu, //friction
      0.02,
      1.0
  );
  msh.addMode( // Flight
      (Eigen::Matrix<bool,1,4>() << false,  false,  false,  false).finished(), // contact bools
      num_knot_points[2],  // number of knot points in the collocation
      Eigen::Vector3d::UnitZ(), // normal
      Eigen::Vector3d::Zero(),  // world offset
      mu, //friction
      0.02,
      1.0
  );
  msh.addMode( // Flight
      (Eigen::Matrix<bool,1,4>() << false,  false,  false,  false).finished(), // contact bools
      num_knot_points[3],  // number of knot points in the collocation
      Eigen::Vector3d::UnitZ(), // normal
      Eigen::Vector3d::Zero(),  // world offset
      mu, //friction
      0.02,
      1.0
  );
  msh.addMode( // Front Stance
      (Eigen::Matrix<bool,1,4>() << true,  false,  true,  false).finished(), // contact bools
      num_knot_points[4],  // number of knot points in the collocation
      Eigen::Vector3d::UnitZ(), // normal
      Eigen::Vector3d::Zero(),  // world offset
      mu, //friction
      0.02,
      0.1
  );
  msh.addMode( // Stance
      (Eigen::Matrix<bool,1,4>() << true,  true,  true,  true).finished(), // contact bools
      num_knot_points[5],  // number of knot points in the collocation
      Eigen::Vector3d::UnitZ(), // normal
      Eigen::Vector3d::Zero(),  // world offset
      mu, //friction
      0.02,
      0.1
  );
  msh.addMode( // Rear stance
      (Eigen::Matrix<bool,1,4>() << false,  true,  false,  true).finished(), // contact bools
      num_knot_points[6],  // number of knot points in the collocation
      Eigen::Vector3d::UnitZ(), // normal
      Eigen::Vector3d::Zero(),  // world offset
      mu, //friction
      0.02,
      1.0
  );
  msh.addMode( // Flight
      (Eigen::Matrix<bool,1,4>() << false,  false,  false,  false).finished(), // contact bools
      num_knot_points[7],  // number of knot points in the collocation
      Eigen::Vector3d::UnitZ(), // normal
      Eigen::Vector3d::Zero(),  // world offset
      mu, //friction
      0.02,
      1.0
  );
  msh.addMode( // Flight
      (Eigen::Matrix<bool,1,4>() << false,  false,  false,  false).finished(), // contact bools
      num_knot_points[8],  // number of knot points in the collocation
      Eigen::Vector3d::UnitZ(), // normal
      Eigen::Vector3d::Zero(),  // world offset
      mu, //friction
      0.02,
      1.0
  );
  msh.addMode( // Front Stance
      (Eigen::Matrix<bool,1,4>() << true,  false,  true,  false).finished(), // contact bools
      num_knot_points[9],  // number of knot points in the collocation
      Eigen::Vector3d::UnitZ(), // normal
      Eigen::Vector3d::Zero(),  // world offset
      mu, //friction
      0.02,
      0.1
  );
  msh.addMode( // Stance
      (Eigen::Matrix<bool,1,4>() << true,  true,  true,  true).finished(), // contact bools
      num_knot_points[10],  // number of knot points in the collocation
      Eigen::Vector3d::UnitZ(), // normal
      Eigen::Vector3d::Zero(),  // world offset
      mu, //friction
      0.02,
      0.1
  );
}
//TODO: Reimplement
/// getModeSequenceRear, initializes the trajopt mode seqence for jump, see runSpiritJump for a def of inputs
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

  getModeSequenceHelper(msh, mu, num_knot_points);

  auto [modeVector, toeEvals, toeEvalSets] = createSpiritModeSequence(plant, msh);

  for (auto& mode : modeVector){
    for (int i = 0; i < mode->evaluators().num_evaluators(); i++ ){
      mode->MakeConstraintRelative(i,0);
      mode->MakeConstraintRelative(i,1);
    }
    mode->SetDynamicsScale(
        {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19}, 150.0);
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
/// \param ipopt true if should solve with ipopt
/// \param num_knot_points: number of knot points used for each mode (vector)
/// \param initial_height: initial and final height of the jump
/// \param pitch_magnitude_lo the max magnitude of pitch at lo and td
/// \param pitch_magnitude_apex the max magnitude of pitch at apex
/// \param fore_aft_displacement: fore-aft displacemnt after jump
/// \param max_duration: maximum time allowed for jump
/// \param cost_actuation: Cost on actuation
/// \param cost_velocity: Cost on state velocity
/// \param cost_velocity_legs_flight cost on velocity of legs in flight
/// \param cost_actuation_legs_flight cost on actuation of legs in flight
/// \param cost_time cost on duration
/// \param cost_work: Cost on work
/// \param mu: coefficient of friction
/// \param eps: the tolerance for position constraints
/// \param tol: optimization solver constraint and optimality tolerence
/// \param work_constraint_scale: scale for the constraints for the power calculation
/// \param file_name_out: if empty, file is unsaved, if not empty saves the trajectory in the directory
template <typename T>
void runSpiritJump(
    MultibodyPlant<T>& plant,
    vector<PiecewisePolynomial<double>>& x_traj,
    PiecewisePolynomial<double>& u_traj,
    vector<PiecewisePolynomial<double>>& l_traj,
    vector<PiecewisePolynomial<double>>& lc_traj,
    vector<PiecewisePolynomial<double>>& vc_traj,
    const bool animate,
    const bool ipopt,
    std::vector<int> num_knot_points,
    const double initial_height,
    const double pitch_magnitude_lo,
    const double pitch_magnitude_apex,
    const double apex_height,
    const double fore_aft_displacement,
    const double max_duration,
    const double cost_actuation,
    const double cost_velocity,
    const double cost_velocity_legs_flight,
    const double cost_actuation_legs_flight,
    const double cost_time,
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
  auto [modeVector, toeEvals, toeEvalSets] =  getModeSequence(plant, mu, num_knot_points, sequence);

  ///Setup trajectory optimization
  auto trajopt = Dircon<T>(sequence);

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
    trajopt.SetSolverOption(id, "acceptable_tol", tol * 10);
    trajopt.SetSolverOption(id, "acceptable_iter", 5);
  } else {
    // Set up Trajectory Optimization options
    trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(),
                            "Print file", "../snopt.out");//FIXME probably need absolute directory
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

  // Setting up cost
  auto work_binding = addCost(plant, trajopt, cost_actuation, cost_velocity, cost_velocity_legs_flight, cost_actuation_legs_flight, cost_time, cost_work);

  //TODO: Figure out useful file system for repeating runs
  // Initialize the trajectory control state and forces
  if (file_name_in.empty()){
    //trajopt.SetInitialGuessForAllVariables(
    //    VectorXd::Zero(trajopt.decision_variables().size()));
    for (int j = 0; j < sequence.num_modes(); j++) {
      trajopt.SetInitialTrajectoryForMode(j, x_traj[j], u_traj, x_traj[j].start_time(), x_traj[j].end_time());
      trajopt.SetInitialForceTrajectory(j, l_traj[j], lc_traj[j],
                                        vc_traj[j], l_traj[j].start_time());
    }
  }else{
    std::cout<<"Loading decision var from file, will fail if num dec vars changed" <<std::endl;
    dairlib::DirconTrajectory loaded_traj(file_name_in);
    trajopt.SetInitialGuessForAllVariables(loaded_traj.GetDecisionVariables());
  }

  addConstraints(plant, trajopt,
                 initial_height, apex_height, fore_aft_displacement, pitch_magnitude_lo, pitch_magnitude_apex, max_duration, eps);

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
  if (ipopt) {
    solver_id = drake::solvers::IpoptSolver().id();
    cout << "\nChose manually: " << solver_id.name() << endl;
  } else {
    solver_id = drake::solvers::ChooseBestSolver(trajopt);
    cout << "\nChose the best solver: " << solver_id.name() << endl;
  }

  /// Run the optimization using your initial guess
  auto start = std::chrono::high_resolution_clock::now();
  auto solver = drake::solvers::MakeSolver(solver_id);
  drake::solvers::MathematicalProgramResult result;
  solver->Solve(trajopt, trajopt.initial_guess(), trajopt.solver_options(),
                &result);  auto finish = std::chrono::high_resolution_clock::now();
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
    x_traj = old_traj.ReconstructStateDiscontinuousTrajectory();
    u_traj = old_traj.ReconstructInputTrajectory();
    l_traj = old_traj.ReconstructLambdaTrajectory();
    lc_traj = old_traj.ReconstructLambdaCTrajectory();
    vc_traj = old_traj.ReconstructGammaCTrajectory();
  } else{
    std::cout << "warning no file name provided, will not be able to return full solution" << std::endl;
    x_traj  = trajopt.ReconstructDiscontinuousStateTrajectory(result);
    u_traj  = trajopt.ReconstructInputTrajectory(result);
    l_traj  = trajopt.ReconstructLambdaTrajectory(result);
  }
  auto x_trajs = trajopt.ReconstructDiscontinuousStateTrajectory(result);
  std::cout<<"Work = " << dairlib::calcElectricalWork(plant, x_trajs, u_traj) << std::endl;

  if(cost_work > 0){
    double cost_work_val = solvers::EvalCostGivenSolution(
        result, work_binding);
    std::cout<<"ReLu Work = " << cost_work_val/cost_work << std::endl;
  }

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

template <typename T>
void runSpiritParkour(
    MultibodyPlant<T>& plant,
    vector<PiecewisePolynomial<double>>& x_traj,
    PiecewisePolynomial<double>& u_traj,
    vector<PiecewisePolynomial<double>>& l_traj,
    vector<PiecewisePolynomial<double>>& lc_traj,
    vector<PiecewisePolynomial<double>>& vc_traj,
    QuadrupedalModeSequence quad_mod_seq,
    const bool animate,
    const bool ipopt,
    const double initial_height,
    const double pitch_magnitude_lo,
    const double pitch_magnitude_apex,
    const double apex_height,
    const double max_duration,
    const double cost_actuation,
    const double cost_velocity,
    const double cost_velocity_legs_flight,
    const double cost_actuation_legs_flight,
    const double cost_time,
    const double cost_work,
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
  auto [modeVector, toeEvals, toeEvalSets] =  getModeSequence(plant, quad_mod_seq, sequence); // FIXME:getModeSequence

  ///Setup trajectory optimization
  auto trajopt = Dircon<T>(sequence);

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
    trajopt.SetSolverOption(id, "acceptable_tol", tol * 10);
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

  // Setting up cost// FIXME:addCost
  auto work_binding = addCost(plant, trajopt, quad_mod_seq, cost_actuation, cost_velocity, cost_velocity_legs_flight, cost_actuation_legs_flight, cost_time, cost_work);

  //TODO: Figure out useful file system for repeating runs
  // Initialize the trajectory control state and forces
  if (file_name_in.empty()){
    //trajopt.SetInitialGuessForAllVariables(
    //    VectorXd::Zero(trajopt.decision_variables().size()));
    for (int j = 0; j < sequence.num_modes(); j++) {
      trajopt.SetInitialTrajectoryForMode(j, x_traj[j], u_traj, x_traj[j].start_time(), x_traj[j].end_time());
      trajopt.SetInitialForceTrajectory(j, l_traj[j], lc_traj[j],
                                        vc_traj[j], l_traj[j].start_time());
    }
  }else{
    std::cout<<"Loading decision var from file, will fail if num dec vars changed" <<std::endl;
    dairlib::DirconTrajectory loaded_traj(file_name_in);
    trajopt.SetInitialGuessForAllVariables(loaded_traj.GetDecisionVariables());
  }
  // FIXME:addConstraints 
  addConstraints(plant, trajopt, quad_mod_seq,
                 initial_height, apex_height, fore_aft_displacement, pitch_magnitude_lo, pitch_magnitude_apex, max_duration, eps);

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
  if (ipopt) {
    solver_id = drake::solvers::IpoptSolver().id();
    cout << "\nChose manually: " << solver_id.name() << endl;
  } else {
    solver_id = drake::solvers::ChooseBestSolver(trajopt);
    cout << "\nChose the best solver: " << solver_id.name() << endl;
  }

  /// Run the optimization using your initial guess
  auto start = std::chrono::high_resolution_clock::now();
  auto solver = drake::solvers::MakeSolver(solver_id);
  drake::solvers::MathematicalProgramResult result;
  solver->Solve(trajopt, trajopt.initial_guess(), trajopt.solver_options(),
                &result);  auto finish = std::chrono::high_resolution_clock::now();
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
    x_traj = old_traj.ReconstructStateDiscontinuousTrajectory();
    u_traj = old_traj.ReconstructInputTrajectory();
    l_traj = old_traj.ReconstructLambdaTrajectory();
    lc_traj = old_traj.ReconstructLambdaCTrajectory();
    vc_traj = old_traj.ReconstructGammaCTrajectory();
  } else{
    std::cout << "warning no file name provided, will not be able to return full solution" << std::endl;
    x_traj  = trajopt.ReconstructDiscontinuousStateTrajectory(result);
    u_traj  = trajopt.ReconstructInputTrajectory(result);
    l_traj  = trajopt.ReconstructLambdaTrajectory(result);
  }
  auto x_trajs = trajopt.ReconstructDiscontinuousStateTrajectory(result);
  std::cout<<"Work = " << dairlib::calcElectricalWork(plant, x_trajs, u_traj) << std::endl;

  if(cost_work > 0){
    double cost_work_val = solvers::EvalCostGivenSolution(
        result, work_binding);
    std::cout<<"ReLu Work = " << cost_work_val/cost_work << std::endl;
  }

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
}// namespace dairlib

class QuadrupedalMode {
  private:
    bool contact_set_[4];
    int num_knots_;
    Eigen::Vector3d normal_;
    Eigen::Vector3d offset_;
    double mu_;
    double minT_;
    double maxT_;

  public:
    QuadrupedalMode(  bool contact_set[4],
                      Eigen::Vector3d normal,
                      Eigen::Vector3d offset,
                      double mu = std::numeric_limits<double>::infinity() ,
                      int num_knots = 7,
                      double minT = 0,
                      double maxT = std::numeric_limits<double>::infinity()   ) {
      for (size_t i = 0; i < 4; i++)
      {
        contact_set_[i] = contact_set[i];
      }
      num_knots_ = num_knots;
      normal_ = normal;
      offset_ = offset;
      mu_ = mu;
      minT_ = minT;
      maxT_ = maxT;
      }
      
    QuadrupedalMode(std::string contact_set_str,
                    Eigen::Vector3d normal,
                    Eigen::Vector3d offset,
                    double mu = std::numeric_limits<double>::infinity(),
                    int num_knots = 7,
                    double minT = 0,
                    double maxT = std::numeric_limits<double>::infinity()) {
      assert(contact_set_str.length() == 4  && "String must be four characters long");
      transform(contact_set_str.begin(), contact_set_str.end(), contact_set_str.begin(), ::tolower);
      for( int i = 0; i<4; i++){
        assert(contact_set_str[i]=='t'||contact_set_str[i]=='f'||contact_set_str[i]=='0'||contact_set_str[i]=='1' && "String must be made up of t's and f's or 1's and 0's");
        contact_set_[i] = (contact_set_str[i]!='f'&&contact_set_str[i]!='0');
        }      
      num_knots_ = num_knots;
      normal_ = normal;
      normal.normalize();
      offset_ = offset;
      mu_ = mu;
      minT_ = minT;
      maxT_ = maxT;
      }

    static QuadrupedalMode flight(int num_knots,
                                 double minT = 0,
                                  double maxT = std::numeric_limits<double>::infinity()){
      return QuadrupedalMode( {false,false,false,false},
                              Eigen::Vector3d::UnitZ(), 
                              Eigen::Vector3d::Zero(), 
                              std::numeric_limits<double>::infinity(),
                              num_knots,
                              minT,
                              maxT);
    }
    // Simple verbose printing method
    void print(){
      std::cout << "************** Quadrupedal Mode *****" << std::endl;
      std::cout << std::boolalpha;
      std::cout << "Active Toe Contacts: " << contact_set_[0] << ", " << contact_set_[1] << ", " << contact_set_[2] << ", " << contact_set_[3] << std::endl;
      std::cout << "Number of Knots: " << num_knots_ << std::endl;
      std::cout << "Contact Normal: " << "(" << normal_.transpose() << ")^T" << std::endl;
      std::cout << "Contact Offset: " << "(" << offset_.transpose() << ")^T" << std::endl;
      std::cout << "Friction coefficient, mu: " << mu_ << std::endl;
      std::cout << "Time Range: " << "min: " << minT_ << " max: "<<  maxT_ << std::endl;
      std::cout << "*************************************" << std::endl;
    }
    // Get and Set methods
    Eigen::Vector3d normal(){return normal_;}
    Eigen::Vector3d offset(){return offset_;}
    double mu(){return mu_;}
    double minT(){return minT_;}
    double maxT(){return maxT_;}
    int num_knots(){return num_knots_;}

    void set_normal(Eigen::Vector3d new_normal){ normal_ = new_normal;}
    void set_offset(Eigen::Vector3d new_offset){ offset_ = new_offset;}
    void set_mu(double mu){ mu_ = mu;}
    void set_minT(double minT){ minT_ = minT;}
    void set_maxT(double maxT){ maxT_ = maxT;}
    void set_num_knots(int num_knots){num_knots_ = num_knots;}

    Eigen::Vector3d affinePosition(double height){ return (offset() + (normal() * height)); }
    
};


class QuadrupedalModeSequence {
  public:
    std::vector<dairlib::QuadrupedalMode> mode_sequence;
    void addMode(dairlib::QuadrupedalMode qMode){ mode_sequence.push_back(qMode); }
    void addFlight(int num_knots,
                   double minT = 0,
                   double maxT = std::numeric_limits<double>::infinity()){
      mode_sequence.push_back(dairlib::QuadrupedalMode::flight(num_knots,
                                                               minT,
                                                               maxT));
    }
    void print(){ for(auto mode : mode_sequence){mode.print();} }
    void set_mu(double mu){for(auto mode : mode_sequence){mode.set_mu(mu);}
};
//TODO: set up to run only a couple experiments to start



void getParkourBoundModeSequence(QuadrupedalModeSequence quad_mode_seq,
                                 Surface init_surf,
                                 Surface mid_surf,
                                 Surface final_surf,
                                 std::vector<int> num_knots, 
                                 std::vector<double> min_T,
                                 std::vector<double> max_T){

  //Front Left, Back Left, Front Right, Back Right
  quad_mode_seq.addMode( QuadrupedalMode( "1111", init_surf.normal, init_surf.offset, init_surf.mu, num_knots[0], min_T[0], max_T[0] ) );
  quad_mode_seq.addMode( QuadrupedalMode( "0101", init_surf.normal, init_surf.offset, init_surf.mu, num_knots[1], min_T[1], max_T[1] ) );
  quad_mode_seq.addFlight(num_knots[2], min_T[2], max_T[2] );
  quad_mode_seq.addFlight(num_knots[3], min_T[3], max_T[3] );
  quad_mode_seq.addMode( QuadrupedalMode( "1010", mid_surf.normal, mid_surf.offset, mid_surf.mu, num_knots[4], min_T[4], max_T[4] ) );
  quad_mode_seq.addMode( QuadrupedalMode( "1111", mid_surf.normal, mid_surf.offset, mid_surf.mu, num_knots[5], min_T[5], max_T[5] ) );
  quad_mode_seq.addMode( QuadrupedalMode( "0101", mid_surf.normal, mid_surf.offset, mid_surf.mu, num_knots[6], min_T[6], max_T[6] ) );
  quad_mode_seq.addFlight(num_knots[7], min_T[7], max_T[7] );
  quad_mode_seq.addFlight(num_knots[8], min_T[8], max_T[8] );
  quad_mode_seq.addMode( QuadrupedalMode( "1010", final_surf.normal, final_surf.offset, final_surf.mu, num_knots[9], min_T[9], max_T[9] ) );
  quad_mode_seq.addMode( QuadrupedalMode( "1111", final_surf.normal, final_surf.offset, final_surf.mu, num_knots[10], min_T[10], max_T[10] ) );
}
void getParkourBoundModeSequence(QuadrupedalModeSequence quad_mode_seq,
                                 Surface init_surf,
                                 Surface mid_surf,
                                 Surface final_surf,
                                 int num_knots, 
                                 double min_T,
                                 double max_T){

  //Front Left, Back Left, Front Right, Back Right
  quad_mode_seq.addMode( QuadrupedalMode( "1111", init_surf.normal, init_surf.offset, init_surf.mu, num_knots, min_T, max_T ) );
  quad_mode_seq.addMode( QuadrupedalMode( "0101", init_surf.normal, init_surf.offset, init_surf.mu, num_knots, min_T, max_T ) );
  quad_mode_seq.addFlight(num_knots, min_T, max_T );
  quad_mode_seq.addFlight(num_knots, min_T, max_T );
  quad_mode_seq.addMode( QuadrupedalMode( "1010", mid_surf.normal, mid_surf.offset, mid_surf.mu, num_knots, min_T, max_T ) );
  quad_mode_seq.addMode( QuadrupedalMode( "1111", mid_surf.normal, mid_surf.offset, mid_surf.mu, num_knots, min_T, max_T ) );
  quad_mode_seq.addMode( QuadrupedalMode( "0101", mid_surf.normal, mid_surf.offset, mid_surf.mu, num_knots, min_T, max_T ) );
  quad_mode_seq.addFlight(num_knots, min_T, max_T);
  quad_mode_seq.addFlight(num_knots, min_T, max_T);
  quad_mode_seq.addMode( QuadrupedalMode( "1010", final_surf.normal, final_surf.offset, final_surf.mu, num_knots, min_T, max_T ) );
  quad_mode_seq.addMode( QuadrupedalMode( "1111", final_surf.normal, final_surf.offset, final_surf.mu, num_knots, min_T, max_T ) );
}

}  // namespace
}  





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

  std::vector<PiecewisePolynomial<double>> x_traj;
  PiecewisePolynomial<double> u_traj;
  std::vector<PiecewisePolynomial<double>> l_traj;
  std::vector<PiecewisePolynomial<double>> lc_traj;
  std::vector<PiecewisePolynomial<double>> vc_traj;

  /// Set up the surface
  Eigen::Vector3d init_surf_normal; init_surf_normal << 0.0, 0.0, 1.0;
  Eigen::Vector3d init_surf_offset; init_surf_offset << 0.0, 0.0, 0.0;

  Eigen::Vector3d mid_surf_normal; mid_surf_normal << 0.0, 0.0, 1.0;
  Eigen::Vector3d mid_surf_offset; mid_surf_offset << 0.5, 0.0, 0.1;

  Eigen::Vector3d final_surf_normal; final_surf_normal << 0.0, 0.0, 1.0;
  Eigen::Vector3d final_surf_offset; final_surf_offset << 1.0, 0.0, 0.0;


  // Set up the surface set for 2nd optimization
 
  int num_knots = 7;
  mu = 10;
  
  Surface  init_surface(  init_surf_normal,  init_surf_offset, mu);
  Surface   mid_surface(   mid_surf_normal,   mid_surf_offset, mu);
  Surface final_surface( final_surf_normal, final_surf_offset, mu);
  QuadrupedalModeSequence parkour_mode_seq;
  getParkourBoundModeSequence(  parkour_mode_seq, 
                                init_surface,
                                mid_surface,
                                final_surface,
                                num_knots,
                                min_T,
                                max_T
                                );


  std::string distance_name = std::to_string(int(floor(100*FLAGS_foreAftDisplacement)))+"cm";
  

if(!FLAGS_skipInitialOptimization){
    std::cout<<"Running 1st optimization"<<std::endl;
    //Hopping correct distance, but heavily constrained
    dairlib::badInplaceBound(*plant, x_traj, u_traj, l_traj, lc_traj, vc_traj);
    std::cout<<"Inplace Calculated"<<std::endl;
    dairlib::runSpiritJump<double>(
        *plant,
        x_traj, u_traj, l_traj,
        lc_traj, vc_traj,
        false,
        true,
        {7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7} ,
        FLAGS_standHeight,
        0.6,
        0.1,
        FLAGS_apexGoal,       // Ignored if small
        0,
        1.8,
        3,
        10,
        10/5.0,
        5/5.0,
        1000,
        0,
        100,
        1e-3,
        1e-2,
        FLAGS_data_directory+"in_place_parkour");
  }

  std::cout<<"Running 2nd optimization"<<std::endl;
  parkour_mode_seq.set_mu(100);
  dairlib::runSpiritParkour<double>(
      *plant,
      x_traj, u_traj, l_traj,
      lc_traj, vc_traj,
      parkour_mode_seq,
      false,
      true,
      FLAGS_standHeight,
      1.0,
      0.3,
      FLAGS_apexGoal,       // Ignored if small
      1.8,
      3,
      10,
      10/5.0,
      5/5.0,
      1000,
      0,
      1e-3,
      1e0,
      FLAGS_data_directory+"parkour_"+distance_name,
      FLAGS_data_directory+"in_place_parkour");
  std::cout<<"Running 3rd optimization"<<std::endl;

  dairlib::runSpiritParkour<double>(
      *plant,
      x_traj, u_traj, l_traj,
      lc_traj, vc_traj,
      parkour_mode_seq,
      false,//DEBUGtesting 3rd opti
      true,
      FLAGS_standHeight,
      1.0,
      0.3,
      FLAGS_apexGoal,       // Ignored if small
      1.8,
      3,
      10,
      10/5.0,
      5/5.0,
      1000,
      0,
      1e-3,
      1e0,
      FLAGS_data_directory+"parkour_"+distance_name+"low_mu",
      FLAGS_data_directory+"parkour_"+distance_name);

  std::cout<<"Running 4th optimization"<<std::endl;

  dairlib::runSpiritParkour<double>(
      *plant,
      x_traj, u_traj, l_traj,
      lc_traj, vc_traj,
      parkour_mode_seq,
      true,
      true,
      FLAGS_standHeight,
      1.0,
      0.3,
      FLAGS_apexGoal,       // Ignored if small
      1.8,
      3/100.0,
      10/100.0,
      10/5.0,
      5/5.0,
      0,
      100,
      FLAGS_eps,
      FLAGS_tol,
      FLAGS_data_directory+"parkour_"+distance_name+"min_work",
      FLAGS_data_directory+"parkour_"+distance_name+"low_mu");
  return 0;
}






//   if(!FLAGS_skipInitialOptimization){
//     std::cout<<"Running 1st optimization"<<std::endl;
//     //Hopping correct distance, but heavily constrained
//     dairlib::badInplaceBound(*plant, x_traj, u_traj, l_traj, lc_traj, vc_traj);
//     std::cout<<"Inplace Calculated"<<std::endl;
//     dairlib::runSpiritJump<double>(
//         *plant,
//         x_traj, u_traj, l_traj,
//         lc_traj, vc_traj,
//         false,
//         true,
//         {7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7} ,
//         FLAGS_standHeight,
//         0.6,
//         0.1,
//         FLAGS_apexGoal,       // Ignored if small
//         0,
//         1.8,
//         3,
//         10,
//         10/5.0,
//         5/5.0,
//         1000,
//         0,
//         100,
//         1e-3,
//         1e-2,
//         FLAGS_data_directory+"in_place_parkour");
//   }

//   std::cout<<"Running 2nd optimization"<<std::endl;

//   dairlib::runSpiritJump<double>(
//       *plant,
//       x_traj, u_traj, l_traj,
//       lc_traj, vc_traj,
//       false,
//       true,
//         {7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7} ,
//       FLAGS_standHeight,
//       1.0,
//       0.3,
//       FLAGS_apexGoal,       // Ignored if small
//       FLAGS_foreAftDisplacement,
//       1.8,
//       3,
//       10,
//       10/5.0,
//       5/5.0,
//       1000,
//       0,
//       10,
//       1e-3,
//       1e0,
//       FLAGS_data_directory+"parkour_"+distance_name,
//       FLAGS_data_directory+"in_place_parkour");
//   std::cout<<"Running 3rd optimization"<<std::endl;

//   dairlib::runSpiritJump<double>(
//       *plant,
//       x_traj, u_traj, l_traj,
//       lc_traj, vc_traj,
//       false,//DEBUGtesting 3rd opti
//       true,
//         {7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7} ,
//       FLAGS_standHeight,
//       1.0,
//       0.3,
//       FLAGS_apexGoal,       // Ignored if small
//       FLAGS_foreAftDisplacement,
//       1.8,
//       3,
//       10,
//       10/5.0,
//       5/5.0,
//       1000,
//       0,
//       FLAGS_mu,
//       1e-3,
//       1e0,
//       FLAGS_data_directory+"parkour_"+distance_name+"low_mu",
//       FLAGS_data_directory+"parkour_"+distance_name);

//   std::cout<<"Running 4th optimization"<<std::endl;

//   dairlib::runSpiritJump<double>(
//       *plant,
//       x_traj, u_traj, l_traj,
//       lc_traj, vc_traj,
//       true,
//       true,
//         {7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7} ,
//       FLAGS_standHeight,
//       1.0,
//       0.3,
//       FLAGS_apexGoal,       // Ignored if small
//       FLAGS_foreAftDisplacement,
//       1.8,
//       3/100.0,
//       10/100.0,
//       10/5.0,
//       5/5.0,
//       0,
//       100,
//       FLAGS_mu,
//       FLAGS_eps,
//       FLAGS_tol,
//       FLAGS_data_directory+"parkour_"+distance_name+"min_work",
//       FLAGS_data_directory+"parkour_"+distance_name+"low_mu");
//   return 0;
// }