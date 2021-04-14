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
#include <drake/solvers/choose_best_solver.h>

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
DEFINE_double(foreAftDisplacement, 0.0, "The fore-aft displacement.");
DEFINE_double(apexGoal, 0.3, "Apex state goal");
DEFINE_double(inputCost, 3, "The standing height.");
DEFINE_double(velocityCost, 10, "The standing height.");
DEFINE_double(eps, 1e-2, "The wiggle room.");
DEFINE_double(tol, 1e-6, "Optimization Tolerance");
DEFINE_double(mu, 1, "coefficient of friction");

DEFINE_string(data_directory, "/home/shane/Drake_ws/dairlib/examples/Spirit/saved_trajectories/",
              "directory to save/read data");
DEFINE_string(distance_name, "90cm","name to describe distance");

DEFINE_bool(runAllOptimization, true, "rerun earlier optimizations?");
DEFINE_bool(skipInitialOptimization, true, "skip first optimizations?");
DEFINE_bool(minWork, false, "try to minimize work?");
DEFINE_bool(ipopt, true, "Use IPOPT as solver instead of SNOPT");

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

/// badSpiritRear, generates a bad initial guess for the spirit jump traj opt
/// \param plant: robot model
/// \param x_traj[out]: initial and solution state trajectory
/// \param u_traj[out]: initial and solution control trajectory
/// \param l_traj[out]: initial and solution contact force trajectory
/// \param lc_traj[out]: initial and solution contact force slack variable trajectory
/// \param vc_traj[out]: initial and solution contact velocity slack variable trajectory
template <typename T>
void badSpiritRear(MultibodyPlant<T>& plant,
<<<<<<< HEAD
<<<<<<< HEAD
                   vector<PiecewisePolynomial<double>>& x_traj,
=======
                   PiecewisePolynomial<double>& x_traj,
>>>>>>> Leap is working fairly well
=======
                   vector<PiecewisePolynomial<double>>& x_traj,
>>>>>>> Making some progress with new initial guess load in
                   PiecewisePolynomial<double>& u_traj,
                   vector<PiecewisePolynomial<double>>& l_traj,
                   vector<PiecewisePolynomial<double>>& lc_traj,
                   vector<PiecewisePolynomial<double>>& vc_traj){

  const double duration = 0.3;
  std::vector<MatrixXd> x_points;
  dairlib::DirconTrajectory old_traj(FLAGS_data_directory+"simple_yaw2");

  std::vector<double> time_vec = {0, duration/3.0};

  auto positions_map = multibody::makeNameToPositionsMap(plant);
  auto velocities_map = multibody::makeNameToVelocitiesMap(plant);
  VectorXd x_const;
  dairlib::ikSpiritStand(plant, x_const, {true, true, true, true}, FLAGS_standHeight, 0.15, 0, 0);
  x_points.push_back(x_const);
  dairlib::ikSpiritStand(plant, x_const, {true, true, true, true}, FLAGS_standHeight, 0.2, 0, -0.3);
  x_points.push_back(x_const);
  x_traj.push_back(PiecewisePolynomial<double>::FirstOrderHold(time_vec,x_points));

  x_points[0]=x_const;
  dairlib::ikSpiritStand(plant, x_const, {false, true, false, true},  (FLAGS_standHeight + FLAGS_apexGoal)/2.0, 0.15, 0, -0.35/2);
  x_points[1]=x_const;
  time_vec[0] = time_vec[1];
  time_vec[1] = 2* time_vec[1];
  x_traj.push_back(PiecewisePolynomial<double>::FirstOrderHold(time_vec,x_points));

  std::vector<MatrixXd> u_points;
  u_points.push_back(MatrixXd::Zero( plant.num_actuators(), 1));
  u_points.push_back(MatrixXd::Zero(plant.num_actuators(), 1));
  u_points.push_back(MatrixXd::Zero(plant.num_actuators(), 1));
  time_vec = {0, duration/3.0, 2.0 * duration/3.0};
//  time_vec.push_back(x_traj.end_time());
  u_traj = PiecewisePolynomial<double>::FirstOrderHold(time_vec,u_points);

  // Four contacts so forces are 12 dimensional
  Eigen::VectorXd init_l_vec(12);
  // Initial guess
  init_l_vec << 0, 0, 60*9.81, 0, 0, 10*9.81, 0, 0, 60*9.81, 0, 0, 10*9.81; //gravity and mass distributed

  Eigen::VectorXd init_l_vec_front_stance(12);
  init_l_vec_front_stance << 0, 0, 0, 0, 0, 60*9.81, 0, 0, 0, 0, 0, 60*9.81; //gravity and mass distributed

  Eigen::VectorXd init_l_vec_back_stance(12);
  init_l_vec_back_stance << 0, 0, 60*9.81, 0, 0, 0, 0, 0, 60*9.81, 0, 0, 0; //gravity and mass distributed

  //Initialize force trajectories

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
    init_time_j.push_back(i * duration /6.0/ (N - 1));
    init_l_j.push_back(init_l_vec);
    init_lc_j.push_back(init_l_vec);
    init_vc_j.push_back(VectorXd::Zero(12));
  }
<<<<<<< HEAD

  auto init_l_traj_j = PiecewisePolynomial<double>::ZeroOrderHold(init_time_j,init_l_j);
  auto init_lc_traj_j = PiecewisePolynomial<double>::ZeroOrderHold(init_time_j,init_lc_j);
  auto init_vc_traj_j = PiecewisePolynomial<double>::ZeroOrderHold(init_time_j,init_vc_j);

=======

  auto init_l_traj_j = PiecewisePolynomial<double>::ZeroOrderHold(init_time_j,init_l_j);
  auto init_lc_traj_j = PiecewisePolynomial<double>::ZeroOrderHold(init_time_j,init_lc_j);
  auto init_vc_traj_j = PiecewisePolynomial<double>::ZeroOrderHold(init_time_j,init_vc_j);

>>>>>>> I really should have commited this a long time ago
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

template <typename T>
void appendFlight(MultibodyPlant<T>& plant,
<<<<<<< HEAD
<<<<<<< HEAD
                  vector<PiecewisePolynomial<double>>& x_traj,
=======
                  PiecewisePolynomial<double>& x_traj,
>>>>>>> Leap is working fairly well
=======
                  vector<PiecewisePolynomial<double>>& x_traj,
>>>>>>> Making some progress with new initial guess load in
                  PiecewisePolynomial<double>& u_traj,
                  vector<PiecewisePolynomial<double>>& l_traj,
                  vector<PiecewisePolynomial<double>>& lc_traj,
                  vector<PiecewisePolynomial<double>>& vc_traj){
<<<<<<< HEAD
<<<<<<< HEAD
  auto xlo = x_traj[1].value(x_traj[1].end_time());
=======
  auto xlo = x_traj.value(x_traj.end_time());
>>>>>>> Leap is working fairly well
=======
  auto xlo = x_traj[1].value(x_traj[1].end_time());
>>>>>>> Making some progress with new initial guess load in
  auto positions_map = multibody::makeNameToPositionsMap(plant);
  auto velocities_map = multibody::makeNameToVelocitiesMap(plant);
  int n_q = plant.num_positions();

  double apex_height = xlo(positions_map.at("base_z")) + xlo(n_q+velocities_map.at("base_vz"))/2.0/9.81;
<<<<<<< HEAD
<<<<<<< HEAD
  double apex_time = x_traj[1].end_time() + xlo(n_q+velocities_map.at("base_vz"))/9.81;
=======
  double apex_time = x_traj.end_time() + xlo(n_q+velocities_map.at("base_vz"))/9.81;
>>>>>>> Leap is working fairly well
=======
  double apex_time = x_traj[1].end_time() + xlo(n_q+velocities_map.at("base_vz"))/9.81;
>>>>>>> Making some progress with new initial guess load in
  double initial_pitch = 2 * asin(xlo(positions_map.at("base_qy")));
  double apex_pitch = initial_pitch + xlo(n_q+velocities_map.at("base_vz"))/9.81 * xlo(n_q + velocities_map.at("base_wy"));
  double apex_foreaft_pos = xlo(positions_map.at("base_x")) + xlo(n_q+velocities_map.at("base_vz"))/9.81 * xlo(n_q + velocities_map.at("base_vx"));
  VectorXd x_const;
  dairlib::ikSpiritStand(plant, x_const, {false, false, false, false}, apex_height, 0.15, 0, apex_pitch);
  x_const(positions_map.at("base_x")) = apex_foreaft_pos;
  std::vector<MatrixXd> x_points = {xlo, x_const};
<<<<<<< HEAD
<<<<<<< HEAD
  std::vector<double> time_vec = {x_traj[1].end_time(), apex_time};
  x_traj.push_back(PiecewisePolynomial<double>::FirstOrderHold(time_vec,x_points));
=======
  std::vector<double> time_vec = {x_traj.end_time(), apex_time};
  x_traj.ConcatenateInTime(PiecewisePolynomial<double>::FirstOrderHold(time_vec,x_points));
>>>>>>> Leap is working fairly well
=======
  std::vector<double> time_vec = {x_traj[1].end_time(), apex_time};
  x_traj.push_back(PiecewisePolynomial<double>::FirstOrderHold(time_vec,x_points));
>>>>>>> Making some progress with new initial guess load in

  std::vector<MatrixXd> u_points = {u_traj.value(u_traj.end_time()), u_traj.value(u_traj.end_time())};
  u_traj.ConcatenateInTime(PiecewisePolynomial<double>::FirstOrderHold(time_vec,u_points));

  auto zero_vec = VectorXd::Zero(12);
  std::vector<MatrixXd> zero_traj = {zero_vec, zero_vec};
  auto zero_poly = PiecewisePolynomial<double>::ZeroOrderHold(time_vec, zero_traj);
  l_traj.push_back(zero_poly);
  lc_traj.push_back(zero_poly);
  vc_traj.push_back(zero_poly);
}
<<<<<<< HEAD

template <typename T>
void appendFrontTD(MultibodyPlant<T>& plant,
                  vector<PiecewisePolynomial<double>>& x_traj,
                  PiecewisePolynomial<double>& u_traj,
                  vector<PiecewisePolynomial<double>>& l_traj,
                  vector<PiecewisePolynomial<double>>& lc_traj,
                  vector<PiecewisePolynomial<double>>& vc_traj,
                  const double td_height){
  auto xapex = x_traj[2].value(x_traj[2].end_time());
  auto positions_map = multibody::makeNameToPositionsMap(plant);
  auto velocities_map = multibody::makeNameToVelocitiesMap(plant);
  int n_q = plant.num_positions();

  double apex_height = xapex(positions_map.at("base_z"));
  double apex_time = x_traj[2].end_time();
  double td_time = apex_time + sqrt((apex_height-td_height)*2/9.81);
  double initial_pitch = 2 * asin(xapex(positions_map.at("base_qy")));
  double td_pitch = initial_pitch + (td_time - apex_time) * xapex(n_q + velocities_map.at("base_wy"));
  double td_foreaft_pos = xapex(positions_map.at("base_x")) + (td_time - apex_time) * xapex(n_q + velocities_map.at("base_vx"));
  VectorXd x_const;
  dairlib::ikSpiritStand(plant, x_const, {true, false, true, false}, td_height, 0.15, 0, td_pitch);
  x_const(positions_map.at("base_x")) = td_foreaft_pos;
  std::vector<MatrixXd> x_points = {xapex, x_const};
  std::vector<double> time_vec = {apex_time, td_time};
  x_traj.push_back(PiecewisePolynomial<double>::FirstOrderHold(time_vec,x_points));
  x_points[0] = x_const;
  dairlib::ikSpiritStand(plant, x_const, {true, true, true, true}, td_height-0.02, 0.15, 0, td_pitch/2.0);
  x_const(positions_map.at("base_x")) = td_foreaft_pos;
  x_points[1] = x_const;

  time_vec[0] = x_traj[2].end_time();
  time_vec[1] = td_time+0.03;
  x_traj.push_back(PiecewisePolynomial<double>::FirstOrderHold(time_vec,x_points));

  std::vector<MatrixXd> u_points = {u_traj.value(u_traj.end_time()), u_traj.value(u_traj.end_time())};

  u_traj.ConcatenateInTime(PiecewisePolynomial<double>::FirstOrderHold(time_vec,u_points));

  auto zero_vec = VectorXd::Zero(12);
  std::vector<MatrixXd> zero_traj = {zero_vec, zero_vec};
  time_vec ={apex_time, td_time/2.0 + apex_time/2.0};
  auto zero_poly = PiecewisePolynomial<double>::ZeroOrderHold(time_vec, zero_traj);
  l_traj.push_back(zero_poly);
  lc_traj.push_back(zero_poly);
  vc_traj.push_back(zero_poly);

  VectorXd front_stance_vec(12);
  front_stance_vec << 0, 0, 10*9.81, 0, 0, 0, 0, 0, 10*9.81, 0, 0, 0;
  std::vector<MatrixXd> front_traj = {front_stance_vec, front_stance_vec};
  time_vec = {td_time/2.0 + apex_time/2.0, td_time};
  auto front_poly = PiecewisePolynomial<double>::ZeroOrderHold(time_vec, front_traj);
  l_traj.push_back(front_poly);
  lc_traj.push_back(front_poly);
  vc_traj.push_back(front_poly);
}
=======
>>>>>>> Leap is working fairly well

template <typename T>
<<<<<<< HEAD
void appendStance(MultibodyPlant<T>& plant,
                  vector<PiecewisePolynomial<double>>& x_traj,
=======
void appendFrontTD(MultibodyPlant<T>& plant,
<<<<<<< HEAD
                  PiecewisePolynomial<double>& x_traj,
>>>>>>> Jump seems to be working alright
=======
                  vector<PiecewisePolynomial<double>>& x_traj,
>>>>>>> Making some progress with new initial guess load in
                  PiecewisePolynomial<double>& u_traj,
                  vector<PiecewisePolynomial<double>>& l_traj,
                  vector<PiecewisePolynomial<double>>& lc_traj,
                  vector<PiecewisePolynomial<double>>& vc_traj,
<<<<<<< HEAD
                  double final_height){
  auto xtd = x_traj[4].value(x_traj[4].end_time());
  auto positions_map = multibody::makeNameToPositionsMap(plant);
  auto velocities_map = multibody::makeNameToVelocitiesMap(plant);
  int n_q = plant.num_positions();
=======
                  const double td_height){
  auto xapex = x_traj[2].value(x_traj[2].end_time());
  auto positions_map = multibody::makeNameToPositionsMap(plant);
  auto velocities_map = multibody::makeNameToVelocitiesMap(plant);
  int n_q = plant.num_positions();

  double apex_height = xapex(positions_map.at("base_z"));
  double apex_time = x_traj[2].end_time();
  double td_time = apex_time + sqrt((apex_height-td_height)*2/9.81);
  double initial_pitch = 2 * asin(xapex(positions_map.at("base_qy")));
  double td_pitch = initial_pitch + (td_time - apex_time) * xapex(n_q + velocities_map.at("base_wy"));
  double td_foreaft_pos = xapex(positions_map.at("base_x")) + (td_time - apex_time) * xapex(n_q + velocities_map.at("base_vx"));
  VectorXd x_const;
  dairlib::ikSpiritStand(plant, x_const, {true, false, true, false}, td_height, 0.15, 0, td_pitch);
  x_const(positions_map.at("base_x")) = td_foreaft_pos;
  std::vector<MatrixXd> x_points = {xapex, x_const};
  std::vector<double> time_vec = {apex_time, td_time};
  x_traj.push_back(PiecewisePolynomial<double>::FirstOrderHold(time_vec,x_points));
  x_points[0] = x_const;
  dairlib::ikSpiritStand(plant, x_const, {true, true, true, true}, td_height-0.02, 0.15, 0, td_pitch/2.0);
  x_const(positions_map.at("base_x")) = td_foreaft_pos;
  x_points[1] = x_const;

  time_vec[0] = x_traj[2].end_time();
  time_vec[1] = td_time+0.03;
  x_traj.push_back(PiecewisePolynomial<double>::FirstOrderHold(time_vec,x_points));

  std::vector<MatrixXd> u_points = {u_traj.value(u_traj.end_time()), u_traj.value(u_traj.end_time())};

  u_traj.ConcatenateInTime(PiecewisePolynomial<double>::FirstOrderHold(time_vec,u_points));

  auto zero_vec = VectorXd::Zero(12);
  std::vector<MatrixXd> zero_traj = {zero_vec, zero_vec};
  time_vec ={apex_time, td_time/2.0 + apex_time/2.0};
  auto zero_poly = PiecewisePolynomial<double>::ZeroOrderHold(time_vec, zero_traj);
  l_traj.push_back(zero_poly);
  lc_traj.push_back(zero_poly);
  vc_traj.push_back(zero_poly);

  VectorXd front_stance_vec(12);
  front_stance_vec << 0, 0, 10*9.81, 0, 0, 0, 0, 0, 10*9.81, 0, 0, 0;
  std::vector<MatrixXd> front_traj = {front_stance_vec, front_stance_vec};
  time_vec = {td_time/2.0 + apex_time/2.0, td_time};
  auto front_poly = PiecewisePolynomial<double>::ZeroOrderHold(time_vec, front_traj);
  l_traj.push_back(front_poly);
  lc_traj.push_back(front_poly);
  vc_traj.push_back(front_poly);
}

<<<<<<< HEAD
>>>>>>> Jump seems to be working alright
=======
template <typename T>
void appendStance(MultibodyPlant<T>& plant,
                  vector<PiecewisePolynomial<double>>& x_traj,
                  PiecewisePolynomial<double>& u_traj,
                  vector<PiecewisePolynomial<double>>& l_traj,
                  vector<PiecewisePolynomial<double>>& lc_traj,
                  vector<PiecewisePolynomial<double>>& vc_traj,
                  double final_height){
  auto xtd = x_traj[4].value(x_traj[4].end_time());
  auto positions_map = multibody::makeNameToPositionsMap(plant);
  auto velocities_map = multibody::makeNameToVelocitiesMap(plant);
  int n_q = plant.num_positions();
>>>>>>> Things are working ish

  VectorXd x_const;
  dairlib::ikSpiritStand(plant, x_const, {true, true, true, true}, final_height, 0.15, 0, 0);
  x_const(positions_map.at("base_x")) = xtd(positions_map.at("base_x"));
  std::vector<MatrixXd> x_points = {xtd, x_const};
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  std::vector<double> time_vec = {x_traj[4].end_time(), x_traj[4].end_time() + 0.02};
  x_traj.push_back(PiecewisePolynomial<double>::FirstOrderHold(time_vec,x_points));
=======
  std::vector<double> time_vec = {x_traj.end_time(), x_traj.end_time() + 0.1};
  x_traj.ConcatenateInTime(PiecewisePolynomial<double>::FirstOrderHold(time_vec,x_points));
>>>>>>> Things are working ish
=======
  std::vector<double> time_vec = {x_traj[4].end_time(), x_traj[4].end_time() + 0.1};
=======
  std::vector<double> time_vec = {x_traj[4].end_time(), x_traj[4].end_time() + 0.02};
>>>>>>> I really should have commited this a long time ago
  x_traj.push_back(PiecewisePolynomial<double>::FirstOrderHold(time_vec,x_points));
>>>>>>> Making some progress with new initial guess load in

  std::vector<MatrixXd> u_points = {u_traj.value(u_traj.end_time()), u_traj.value(u_traj.end_time())};
  u_traj.ConcatenateInTime(PiecewisePolynomial<double>::FirstOrderHold(time_vec,u_points));

  VectorXd full_stance_force(12);
  full_stance_force << 0, 0, 10*9.81, 0, 0, 10*9.81, 0, 0, 10*9.81, 0, 0, 10*9.81;
  std::vector<MatrixXd> full_stance_vec = {full_stance_force, full_stance_force};
  auto full_stance_poly = PiecewisePolynomial<double>::ZeroOrderHold(time_vec, full_stance_vec);
  l_traj.push_back(full_stance_poly);
  lc_traj.push_back(full_stance_poly);
  vc_traj.push_back(full_stance_poly);
}
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

/// addCost, adds the cost to the trajopt jump problem. See runSpiritJump for a description of the inputs
template <typename T>
void addCost(MultibodyPlant<T>& plant,
             dairlib::systems::trajectory_optimization::Dircon<T>& trajopt,
             const double cost_actuation,
             const double cost_velocity,
             const double cost_velocity_legs_flight,
             const double cost_actuation_legs_flight,
             const double cost_time,
             const double cost_work,
             const double work_constraint_scale = 1.0){
  auto u   = trajopt.input();
  auto x   = trajopt.state();

  // Setup the traditional cost function
  trajopt.AddRunningCost( u.transpose()*cost_actuation*u);
  trajopt.AddVelocityCost(cost_velocity);
  AddWorkCost(plant, trajopt, cost_work, work_constraint_scale, 0.0);

  trajopt.AddRunningCost(cost_time);

  addCostLegs(plant, trajopt, cost_velocity_legs_flight, cost_actuation_legs_flight, {0, 1, 4, 5, 8, 10}, 1);
  if(trajopt.num_modes() > 2){
    addCostLegs(plant, trajopt, cost_velocity_legs_flight, cost_actuation_legs_flight, {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}, 2);
  }
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> Jump seems to be working alright
  if(trajopt.num_modes() > 3){
    addCostLegs(plant, trajopt, cost_velocity_legs_flight, cost_actuation_legs_flight, {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}, 3);
  }
  if(trajopt.num_modes() > 4){
    addCostLegs(plant, trajopt, cost_velocity_legs_flight, cost_actuation_legs_flight, {2, 3, 6, 7, 10, 11}, 4);
  }
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> Jump seems to be working alright

>>>>>>> Leap is working fairly well
=======
>>>>>>> It sort of works
} // Function

template <typename T>
void addConstraintsFlight(MultibodyPlant<T>& plant,
                    dairlib::systems::trajectory_optimization::Dircon<T>& trajopt,
                    double apex_height,
                    double fore_aft_displacement,
                    double eps = 0){
  auto positions_map = multibody::makeNameToPositionsMap(plant);
  auto velocities_map = multibody::makeNameToVelocitiesMap(plant);
  auto   xapex  = trajopt.num_modes() > 3 ? trajopt.state_vars(3,0) : trajopt.final_state();
  trajopt.AddBoundingBoxConstraint(0, 0, xapex(plant.num_positions()+velocities_map.at("base_vz")));

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
template <typename T>
void addConstraintsFlight(MultibodyPlant<T>& plant,
                    dairlib::systems::trajectory_optimization::Dircon<T>& trajopt,
                    double apex_height,
                    double fore_aft_displacement,
                    double eps = 0){
  auto positions_map = multibody::makeNameToPositionsMap(plant);
  auto velocities_map = multibody::makeNameToVelocitiesMap(plant);
  auto   xapex  = trajopt.num_modes() > 3 ? trajopt.state_vars(3,0) : trajopt.final_state();
  trajopt.AddBoundingBoxConstraint(0, 0, xapex(plant.num_positions()+velocities_map.at("base_vz")));

  if (trajopt.num_modes() <= 3){
    nominalSpiritStandConstraint(plant, trajopt, FLAGS_standHeight, {trajopt.N()-1}, eps);
<<<<<<< HEAD
  }
  if (apex_height > 0.15)
    trajopt.AddBoundingBoxConstraint(apex_height-1e-2, apex_height+1e-2, xapex(positions_map.at("base_z")));

  if (fore_aft_displacement >= 0)
    trajopt.AddBoundingBoxConstraint(fore_aft_displacement-eps, fore_aft_displacement, xapex(positions_map.at("base_x")));

  double pitch = abs(0.2);
  // Body pose constraints (keep the body flat) at apex state
  trajopt.AddBoundingBoxConstraint(cos(pitch/2.0), 1, xapex(positions_map.at("base_qw")));
  trajopt.AddBoundingBoxConstraint(0, 0, xapex(positions_map.at("base_qx")));
  trajopt.AddBoundingBoxConstraint(-sin(pitch/2.0) , sin(pitch/2.0), xapex(positions_map.at("base_qy")));
  trajopt.AddBoundingBoxConstraint(0, 0, xapex(positions_map.at("base_qz")));
  std::cout<<"Adding flight constraints" << std::endl;

}

template <typename T>
void addConstraintsTD(MultibodyPlant<T>& plant,
                          dairlib::systems::trajectory_optimization::Dircon<T>& trajopt,
                          const double td_height,
                          const double fore_aft_displacement,
                          const double eps = 0){
  auto positions_map = multibody::makeNameToPositionsMap(plant);
  auto velocities_map = multibody::makeNameToVelocitiesMap(plant);
  auto   xtd  =  trajopt.final_state();

  std::cout<<"Adding td constraints" << std::endl;

  if(td_height > 0)
    trajopt.AddBoundingBoxConstraint(td_height-eps, td_height+eps, xtd(positions_map.at("base_z")));
<<<<<<< HEAD
  double pitch = abs(0.6);
  // Body pose constraints (keep the body flat) at apex state
  trajopt.AddBoundingBoxConstraint(cos(pitch/2.0), 1, xtd(positions_map.at("base_qw")));
  trajopt.AddBoundingBoxConstraint(0, 0, xtd(positions_map.at("base_qx")));
  trajopt.AddBoundingBoxConstraint(-sin(pitch/2.0) , sin(pitch/2.0), xtd(positions_map.at("base_qy")));
  trajopt.AddBoundingBoxConstraint(0, 0, xtd(positions_map.at("base_qz")));

}

template <typename T>
void addConstraintsStance(MultibodyPlant<T>& plant,
                      dairlib::systems::trajectory_optimization::Dircon<T>& trajopt,
                      const double initial_height,
                      const double fore_aft_displacement,
                      const double eps = 0){
  auto positions_map = multibody::makeNameToPositionsMap(plant);
  auto velocities_map = multibody::makeNameToVelocitiesMap(plant);
  auto   xf  = trajopt.final_state();
  std::cout<<"Adding stance constraints" << std::endl;
  nominalSpiritStandConstraint(plant,trajopt,initial_height, {trajopt.N()-1}, eps);

  std::cout<< fore_aft_displacement << std::endl;
  if (fore_aft_displacement >= 0){
    trajopt.AddBoundingBoxConstraint(fore_aft_displacement-eps, fore_aft_displacement, xf(positions_map.at("base_x")));
  }

  double n_v = plant.num_velocities();

  trajopt.AddBoundingBoxConstraint(VectorXd::Zero(n_v), VectorXd::Zero(n_v), xf.tail(n_v));
  trajopt.AddBoundingBoxConstraint(0-eps, 0+eps, xf(positions_map.at("base_y")));

  // Body pose constraints (keep the body flat) at initial state
  trajopt.AddBoundingBoxConstraint(1-eps, 1 + eps, xf(positions_map.at("base_qw")));
  trajopt.AddBoundingBoxConstraint(0-eps , 0+ eps, xf(positions_map.at("base_qx")));
  trajopt.AddBoundingBoxConstraint(0-eps, 0+ eps, xf(positions_map.at("base_qy")));
  trajopt.AddBoundingBoxConstraint(0-eps, 0 + eps, xf(positions_map.at("base_qz")));
=======
=======
  double eps = 1e-2;

=======
>>>>>>> Cleaned up apex with constraints on position
  if (trajopt.num_modes() > 3){
    nominalSpiritStandConstraint(plant, trajopt, FLAGS_standHeight, {trajopt.get_mode_start(3)}, eps);
  }else{
=======
  if (trajopt.num_modes() <= 3){
>>>>>>> Trying to go all the way
    nominalSpiritStandConstraint(plant, trajopt, FLAGS_standHeight, {trajopt.N()-1}, eps);
    trajopt.AddBoundingBoxConstraint(fore_aft_displacement-eps, fore_aft_displacement, xapex(positions_map.at("base_x")));
=======
>>>>>>> Getting full leap succsessful
  }
>>>>>>> Working getting to apex
  if (apex_height > 0.15)
    trajopt.AddBoundingBoxConstraint(apex_height-1e-2, apex_height+1e-2, xapex(positions_map.at("base_z")));

  if (fore_aft_displacement >= 0)
    trajopt.AddBoundingBoxConstraint(fore_aft_displacement-eps, fore_aft_displacement, xapex(positions_map.at("base_x")));

  double pitch = abs(0.2);
  // Body pose constraints (keep the body flat) at apex state
  trajopt.AddBoundingBoxConstraint(cos(pitch/2.0), 1, xapex(positions_map.at("base_qw")));
  trajopt.AddBoundingBoxConstraint(0, 0, xapex(positions_map.at("base_qx")));
  trajopt.AddBoundingBoxConstraint(-sin(pitch/2.0) , sin(pitch/2.0), xapex(positions_map.at("base_qy")));
  trajopt.AddBoundingBoxConstraint(0, 0, xapex(positions_map.at("base_qz")));
  std::cout<<"Adding flight constraints" << std::endl;

}

template <typename T>
void addConstraintsTD(MultibodyPlant<T>& plant,
                          dairlib::systems::trajectory_optimization::Dircon<T>& trajopt,
                          const double td_height,
                          const double fore_aft_displacement,
                          const double eps = 0){
  auto positions_map = multibody::makeNameToPositionsMap(plant);
  auto velocities_map = multibody::makeNameToVelocitiesMap(plant);
<<<<<<< HEAD
  auto   xtd  = trajopt.num_modes() > 5 ? trajopt.state_vars(5,0) : trajopt.final_state();
<<<<<<< HEAD
<<<<<<< HEAD
  trajopt.AddBoundingBoxConstraint(min_final_height/2 + initial_height/2, 100, xtd(positions_map.at("base_z")));
>>>>>>> It sort of works
=======
  trajopt.AddBoundingBoxConstraint(initial_height, min_final_height, xtd(positions_map.at("base_z")));
<<<<<<< HEAD
>>>>>>> It converges
=======
=======
=======
  auto   xtd  =  trajopt.final_state();
>>>>>>> Trying to go all the way

  std::cout<<"Adding td constraints" << std::endl;
  //trajopt.AddBoundingBoxConstraint(fore_aft_displacement-eps, fore_aft_displacement, xtd(positions_map.at("base_x")));
  //trajopt.AddBoundingBoxConstraint(td_height-eps, td_height+eps, xtd(positions_map.at("base_z")));
=======
>>>>>>> Making some progress with new initial guess load in
  double pitch = abs(0.6);
  // Body pose constraints (keep the body flat) at apex state
  trajopt.AddBoundingBoxConstraint(cos(pitch/2.0), 1, xtd(positions_map.at("base_qw")));
  trajopt.AddBoundingBoxConstraint(0, 0, xtd(positions_map.at("base_qx")));
  trajopt.AddBoundingBoxConstraint(-sin(pitch/2.0) , sin(pitch/2.0), xtd(positions_map.at("base_qy")));
  trajopt.AddBoundingBoxConstraint(0, 0, xtd(positions_map.at("base_qz")));

<<<<<<< HEAD
  Eigen::VectorXd end_state_nominal;
  dairlib::nominalSpiritStand(plant, end_state_nominal, 0.3);
  for(int joint_index: {0, 1, 4, 5, 8, 9}){
    double joint_val = end_state_nominal[positions_map.at("joint_"+std::to_string(joint_index))];
    trajopt.AddBoundingBoxConstraint(joint_val - 1e-2, joint_val + 1e-2, xtd(positions_map.at("joint_"+std::to_string(joint_index))));
  }

>>>>>>> Working getting to apex
  double eps = 1e-2;
  trajopt.AddBoundingBoxConstraint(-eps, eps, xtd(positions_map.at("base_qx")));
  trajopt.AddBoundingBoxConstraint(-eps, eps, xtd(positions_map.at("base_qz")));
=======
>>>>>>> Trying to go all the way
}

<<<<<<< HEAD
>>>>>>> Working much better
=======
template <typename T>
void addConstraintsStance(MultibodyPlant<T>& plant,
                      dairlib::systems::trajectory_optimization::Dircon<T>& trajopt,
                      const double initial_height,
                      const double fore_aft_displacement,
                      const double eps = 0){
  auto positions_map = multibody::makeNameToPositionsMap(plant);
  auto velocities_map = multibody::makeNameToVelocitiesMap(plant);
  auto   xf  = trajopt.final_state();
  std::cout<<"Adding stance constraints" << std::endl;
  nominalSpiritStandConstraint(plant,trajopt,initial_height, {trajopt.N()-1}, eps);

  std::cout<< fore_aft_displacement << std::endl;
  if (fore_aft_displacement >= 0){
    trajopt.AddBoundingBoxConstraint(fore_aft_displacement-eps, fore_aft_displacement, xf(positions_map.at("base_x")));
  }

  double n_v = plant.num_velocities();

  trajopt.AddBoundingBoxConstraint(VectorXd::Zero(n_v), VectorXd::Zero(n_v), xf.tail(n_v));
  trajopt.AddBoundingBoxConstraint(0-eps, 0+eps, xf(positions_map.at("base_y")));

  // Body pose constraints (keep the body flat) at initial state
<<<<<<< HEAD
  //trajopt.AddBoundingBoxConstraint(1, 1 , xf(positions_map.at("base_qw")));
  //trajopt.AddBoundingBoxConstraint(0 , 0, xf(positions_map.at("base_qx")));
  //trajopt.AddBoundingBoxConstraint(0, 0, xf(positions_map.at("base_qy")));
  //trajopt.AddBoundingBoxConstraint(0, 0, xf(positions_map.at("base_qz")));
>>>>>>> Working getting to apex
=======
  trajopt.AddBoundingBoxConstraint(1-eps, 1 + eps, xf(positions_map.at("base_qw")));
  trajopt.AddBoundingBoxConstraint(0-eps , 0+ eps, xf(positions_map.at("base_qx")));
  trajopt.AddBoundingBoxConstraint(0-eps, 0+ eps, xf(positions_map.at("base_qy")));
  trajopt.AddBoundingBoxConstraint(0-eps, 0 + eps, xf(positions_map.at("base_qz")));
>>>>>>> Getting full leap succsessful
}
// addConstraints, adds constraints to the trajopt jump problem. See runSpiritJump for a description of the inputs
template <typename T>
void addConstraints(MultibodyPlant<T>& plant,
             dairlib::systems::trajectory_optimization::Dircon<T>& trajopt,
                    const double min_final_height,
                    const double initial_height,
                    const double fore_aft_displacement,
                    const double liftoff_velocity,
                    const double pitch_magnitude,
                    const bool lock_rotation,
                    const bool lock_knees_stance,
                    const double max_duration,
                    const double eps){

  // Get position and velocity dictionaries
  auto positions_map = multibody::makeNameToPositionsMap(plant);
  auto velocities_map = multibody::makeNameToVelocitiesMap(plant);

  setSpiritJointLimits(plant, trajopt, true);
  setSpiritActuationLimits(plant, trajopt);

  /// Setup all the optimization constraints
  int n_q = plant.num_positions();
  int n_v = plant.num_velocities();

<<<<<<< HEAD
<<<<<<< HEAD
  auto x0  = trajopt.initial_state();
  auto xlo = trajopt.num_modes()> 2 ? trajopt.state_vars(2,0):trajopt.final_state();
=======
  auto   x0  = trajopt.initial_state();
  auto xlo = trajopt.final_state();
>>>>>>> Seems to be working, messing with 3rd op
=======
  auto x0  = trajopt.initial_state();
  auto xlo = trajopt.num_modes()> 2 ? trajopt.state_vars(2,0):trajopt.final_state();
>>>>>>> Leap is working fairly well
  auto x_pitch = trajopt.state_vars(1,0);

  // Add duration constraint, currently constrained not bounded
  trajopt.AddDurationBounds(0, max_duration);

  /// Constraining xy position
  // Initial body positions
  trajopt.AddBoundingBoxConstraint(0, 0, x0(positions_map.at("base_x"))); // Give the initial condition room to choose the x_init position
<<<<<<< HEAD
<<<<<<< HEAD
=======
  trajopt.AddBoundingBoxConstraint(fore_aft_displacement-eps, fore_aft_displacement+eps, xlo(positions_map.at("base_x"))); // Give the initial condition room to choose the x_init position
>>>>>>> Seems to be working, messing with 3rd op

  if(trajopt.num_modes() < 3){
    trajopt.AddBoundingBoxConstraint(fore_aft_displacement-eps, fore_aft_displacement+eps, xlo(positions_map.at("base_x"))); // Give the initial condition room to choose the x_init position
    trajopt.AddBoundingBoxConstraint(liftoff_velocity, 100, xlo(n_q+velocities_map.at("base_vz")));
    trajopt.AddBoundingBoxConstraint(liftoff_velocity/2, 100, x_pitch(n_q+velocities_map.at("base_vz")));
    trajopt.AddBoundingBoxConstraint(min_final_height, 100, xlo(positions_map.at("base_z")) );
  }

=======

  if(trajopt.num_modes() < 3){
    trajopt.AddBoundingBoxConstraint(fore_aft_displacement-eps, fore_aft_displacement+eps, xlo(positions_map.at("base_x"))); // Give the initial condition room to choose the x_init position
    trajopt.AddBoundingBoxConstraint(liftoff_velocity, 100, xlo(n_q+velocities_map.at("base_vz")));
    trajopt.AddBoundingBoxConstraint(liftoff_velocity/2, 100, x_pitch(n_q+velocities_map.at("base_vz")));
    trajopt.AddBoundingBoxConstraint(min_final_height, 100, xlo(positions_map.at("base_z")) );
  }

>>>>>>> Working getting to apex
  trajopt.AddBoundingBoxConstraint( -eps, eps, x0( positions_map.at("base_y")));
  trajopt.AddBoundingBoxConstraint(-eps, eps, xlo(positions_map.at("base_y")));

  // Nominal stand or z and attitude
  nominalSpiritStandConstraint(plant,trajopt,initial_height, {0}, eps);

  // Body pose constraints (keep the body flat) at initial state
  trajopt.AddBoundingBoxConstraint(1, 1 , x0(positions_map.at("base_qw")));
  trajopt.AddBoundingBoxConstraint(0 , 0, x0(positions_map.at("base_qx")));
  trajopt.AddBoundingBoxConstraint(0, 0, x0(positions_map.at("base_qy")));
  trajopt.AddBoundingBoxConstraint(0, 0, x0(positions_map.at("base_qz")));

  double pitch = abs(pitch_magnitude);
  // Body pose constraints (keep the body flat) at apex state
  trajopt.AddBoundingBoxConstraint(cos(pitch/2.0), 1, xlo(positions_map.at("base_qw")));
  trajopt.AddBoundingBoxConstraint(0, 0, xlo(positions_map.at("base_qx")));
  trajopt.AddBoundingBoxConstraint(-sin(pitch/2.0) , sin(pitch/2.0), xlo(positions_map.at("base_qy")));
  trajopt.AddBoundingBoxConstraint(0, 0, xlo(positions_map.at("base_qz")));


  // Initial and final velocity
  trajopt.AddBoundingBoxConstraint(VectorXd::Zero(n_v), VectorXd::Zero(n_v), x0.tail(n_v));
<<<<<<< HEAD
<<<<<<< HEAD
=======
  trajopt.AddBoundingBoxConstraint(liftoff_velocity, 100, xlo(n_q+velocities_map.at("base_vz")));
  trajopt.AddBoundingBoxConstraint(liftoff_velocity/2, 100, x_pitch(n_q+velocities_map.at("base_vz")));

  // Apex height
<<<<<<< HEAD
  trajopt.AddBoundingBoxConstraint(.3, 100, xlo(positions_map.at("base_z")) );
>>>>>>> Seems to be working, messing with 3rd op
=======
  trajopt.AddBoundingBoxConstraint(min_final_height, 100, xlo(positions_map.at("base_z")) );
>>>>>>> Refactored joint limits
=======
>>>>>>> Working getting to apex

  for (int i = 0; i < trajopt.N(); i++){
    auto xi = trajopt.state(i);
    //Orientation
    if(lock_rotation){
      trajopt.AddBoundingBoxConstraint(0, 0, xi(positions_map.at("base_qx")));
      trajopt.AddBoundingBoxConstraint(0, 0, xi(positions_map.at("base_qz")));
    }
    // Height
    trajopt.AddBoundingBoxConstraint( 0.15, 5, xi( positions_map.at("base_z")));
    trajopt.AddBoundingBoxConstraint( -eps, eps, xi( n_q+velocities_map.at("base_vy")));
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> Seems to be working, messing with 3rd op
=======
>>>>>>> Making some progress with new initial guess load in
  }
  Eigen::VectorXd end_state_nominal;
  dairlib::nominalSpiritStand(plant, end_state_nominal, initial_height);
  vector<double> joint_vec;
  if (trajopt.num_modes() > 2)
    joint_vec =  {};
  else
    joint_vec =  {0, 1, 4, 5, 8, 10};
  for(int joint_index: joint_vec){
    double joint_val = end_state_nominal[positions_map.at("joint_"+std::to_string(joint_index))];
    trajopt.AddBoundingBoxConstraint(joint_val - eps, joint_val + eps, xlo(positions_map.at("joint_"+std::to_string(joint_index))));
  }
}

<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> Things are working ish
void getModeSequenceHelper(dairlib::ModeSequenceHelper& msh,
                           const double mu,
                           std::vector<int> num_knot_points,
                          const bool optimize_flight,
                           const bool optimize_td,
                           const bool optimize_stance){
<<<<<<< HEAD
=======
/// getModeSequenceRear, initializes the trajopt mode seqence for jump, see runSpiritJump for a def of inputs
template <typename T>
std::tuple<  std::vector<std::unique_ptr<dairlib::systems::trajectory_optimization::DirconMode<T>>>,
             std::vector<std::unique_ptr<multibody::WorldPointEvaluator<T>>> ,
             std::vector<std::unique_ptr<multibody::KinematicEvaluatorSet<T>>>>
getModeSequenceRear(
    drake::multibody::MultibodyPlant<T>& plant, // multibodyPlant
    const double mu,
    std::vector<int> num_knot_points,
    DirconModeSequence<T>& sequence){
  dairlib::ModeSequenceHelper msh;
>>>>>>> Leap is working fairly well
=======
>>>>>>> Things are working ish
  msh.addMode( // Stance
      (Eigen::Matrix<bool,1,4>() << true,  true,  true,  true).finished(), // contact bools
      num_knot_points[0],  // number of knot points in the collocation
      Eigen::Vector3d::UnitZ(), // normal
      Eigen::Vector3d::Zero(),  // world offset
      mu, //friction
      0.03,
      0.5
  );
  msh.addMode( // Stance
      (Eigen::Matrix<bool,1,4>() << false,  true,  false,  true).finished(), // contact bools
      num_knot_points[1],  // number of knot points in the collocation
      Eigen::Vector3d::UnitZ(), // normal
      Eigen::Vector3d::Zero(),  // world offset
      mu, //friction
      0.03,
      1.0
  );

  if(optimize_flight){
    msh.addMode( // Stance
        (Eigen::Matrix<bool,1,4>() << false,  false,  false,  false).finished(), // contact bools
        num_knot_points[2],  // number of knot points in the collocation
        Eigen::Vector3d::UnitZ(), // normal
        Eigen::Vector3d::Zero(),  // world offset
        mu, //friction
        0.03,
        1.0
    );
    if(optimize_td){
      msh.addMode( // Stance
          (Eigen::Matrix<bool,1,4>() << false,  false,  false,  false).finished(), // contact bools
          num_knot_points[3],  // number of knot points in the collocation
          Eigen::Vector3d::UnitZ(), // normal
          Eigen::Vector3d::Zero(),  // world offset
          mu, //friction
          0.03,
          1.0
      );

      msh.addMode( // Stance
          (Eigen::Matrix<bool,1,4>() << true,  false,  true,  false).finished(), // contact bools
          num_knot_points[4],  // number of knot points in the collocation
          Eigen::Vector3d::UnitZ(), // normal
          Eigen::Vector3d::Zero(),  // world offset
          mu, //friction
<<<<<<< HEAD
<<<<<<< HEAD
          0.03,
<<<<<<< HEAD
          0.1
=======
          1.0
>>>>>>> Things are working ish
=======
          0.01,
=======
          0.03,
<<<<<<< HEAD
>>>>>>> Making some progress with new initial guess load in
          0.4
>>>>>>> Getting full leap succsessful
=======
          0.1
>>>>>>> I really should have commited this a long time ago
      );
      if(optimize_stance){
        msh.addMode( // Stance
            (Eigen::Matrix<bool,1,4>() << true,  true,  true,  true).finished(), // contact bools
            num_knot_points[5],  // number of knot points in the collocation
            Eigen::Vector3d::UnitZ(), // normal
            Eigen::Vector3d::Zero(),  // world offset
            mu, //friction
<<<<<<< HEAD
<<<<<<< HEAD
            0.03,
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> I really should have commited this a long time ago
            0.1
        );
      }
    }
  }
}
/// getModeSequenceRear, initializes the trajopt mode seqence for jump, see runSpiritJump for a def of inputs
template <typename T>
std::tuple<  std::vector<std::unique_ptr<dairlib::systems::trajectory_optimization::DirconMode<T>>>,
             std::vector<std::unique_ptr<multibody::WorldPointEvaluator<T>>> ,
             std::vector<std::unique_ptr<multibody::KinematicEvaluatorSet<T>>>>
getModeSequence(
    drake::multibody::MultibodyPlant<T>& plant, // multibodyPlant
    const double mu,
    std::vector<int> num_knot_points,
    DirconModeSequence<T>& sequence,
    const bool optimize_flight,
    const bool optimize_td,
    const bool optimize_stance){
  dairlib::ModeSequenceHelper msh;

  getModeSequenceHelper(msh, mu, num_knot_points, optimize_flight, optimize_td, optimize_stance);

  auto [modeVector, toeEvals, toeEvalSets] = createSpiritModeSequence(plant, msh);

  for (auto& mode : modeVector){
    for (int i = 0; i < mode->evaluators().num_evaluators(); i++ ){
      mode->MakeConstraintRelative(i,0);
      mode->MakeConstraintRelative(i,1);
    }
    mode->SetDynamicsScale(
<<<<<<< HEAD
<<<<<<< HEAD
        {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19}, 500.0);
=======
        {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19}, 150.0);
>>>>>>> Optimization success with pipeline
=======
        {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19}, 500.0);
>>>>>>> I really should have commited this a long time ago
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
<<<<<<< HEAD
=======
            0.5
=======
=======
            0.01,
>>>>>>> Getting full leap succsessful
=======
            0.03,
>>>>>>> Making some progress with new initial guess load in
            0.4
>>>>>>> Working getting to apex
        );
      }
>>>>>>> Things are working ish
=======
>>>>>>> Optimization success with pipeline
    }
  }
<<<<<<< HEAD
<<<<<<< HEAD
=======
  return {std::move(modeVector), std::move(toeEvals), std::move(toeEvalSets)};
=======
>>>>>>> Things are working ish
}
/// getModeSequenceRear, initializes the trajopt mode seqence for jump, see runSpiritJump for a def of inputs
template <typename T>
std::tuple<  std::vector<std::unique_ptr<dairlib::systems::trajectory_optimization::DirconMode<T>>>,
             std::vector<std::unique_ptr<multibody::WorldPointEvaluator<T>>> ,
             std::vector<std::unique_ptr<multibody::KinematicEvaluatorSet<T>>>>
getModeSequence(
    drake::multibody::MultibodyPlant<T>& plant, // multibodyPlant
    const double mu,
    std::vector<int> num_knot_points,
<<<<<<< HEAD
    DirconModeSequence<T>& sequence){

  dairlib::ModeSequenceHelper msh;
  msh.addMode( // Stance
      (Eigen::Matrix<bool,1,4>() << true,  true,  true,  true).finished(), // contact bools
      num_knot_points[0],  // number of knot points in the collocation
      Eigen::Vector3d::UnitZ(), // normal
      Eigen::Vector3d::Zero(),  // world offset
      mu, //friction
      0.03,
      0.5
  );
  msh.addMode( // Stance
      (Eigen::Matrix<bool,1,4>() << false,  true,  false,  true).finished(), // contact bools
      num_knot_points[1],  // number of knot points in the collocation
      Eigen::Vector3d::UnitZ(), // normal
      Eigen::Vector3d::Zero(),  // world offset
      mu, //friction
      0.03,
      1.0
  );
  msh.addMode( // Stance
      (Eigen::Matrix<bool,1,4>() << false,  false,  false,  false).finished(), // contact bools
      num_knot_points[2],  // number of knot points in the collocation
      Eigen::Vector3d::UnitZ(), // normal
      Eigen::Vector3d::Zero(),  // world offset
      mu, //friction
      0.03,
      1.0
  );
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

>>>>>>> Leap is working fairly well
  return {std::move(modeVector), std::move(toeEvals), std::move(toeEvalSets)};
}

/// getModeSequenceRear, initializes the trajopt mode seqence for jump, see runSpiritJump for a def of inputs
template <typename T>
std::tuple<  std::vector<std::unique_ptr<dairlib::systems::trajectory_optimization::DirconMode<T>>>,
             std::vector<std::unique_ptr<multibody::WorldPointEvaluator<T>>> ,
             std::vector<std::unique_ptr<multibody::KinematicEvaluatorSet<T>>>>
getModeSequenceTouchdown(
    drake::multibody::MultibodyPlant<T>& plant, // multibodyPlant
    const double mu,
    std::vector<int> num_knot_points,
    DirconModeSequence<T>& sequence){

=======
    DirconModeSequence<T>& sequence,
    const bool optimize_flight,
    const bool optimize_td,
    const bool optimize_stance){
>>>>>>> Things are working ish
  dairlib::ModeSequenceHelper msh;

  getModeSequenceHelper(msh, mu, num_knot_points, optimize_flight, optimize_td, optimize_stance);

  auto [modeVector, toeEvals, toeEvalSets] = createSpiritModeSequence(plant, msh);

  for (auto& mode : modeVector){
    for (int i = 0; i < mode->evaluators().num_evaluators(); i++ ){
      mode->MakeConstraintRelative(i,0);
      mode->MakeConstraintRelative(i,1);
    }
    mode->SetDynamicsScale(
        {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19}, 200.0);
    if (mode->evaluators().num_evaluators() == 4)
    {
      mode->SetKinVelocityScale(
          {0, 1, 2, 3}, {0, 1, 2}, 1.0);
      mode->SetKinPositionScale(
          {0, 1, 2, 3}, {0, 1, 2}, 200.0);
    }
    else if (mode->evaluators().num_evaluators() == 2){
      mode->SetKinVelocityScale(
          {0, 1}, {0, 1, 2}, 1.0);
      mode->SetKinPositionScale(
          {0, 1}, {0, 1, 2}, 200.0);
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
/// \param min_final_height: apex height of the jump, if 0, do not enforce and apex height
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
    vector<PiecewisePolynomial<double>>& x_traj,
    PiecewisePolynomial<double>& u_traj,
    vector<PiecewisePolynomial<double>>& l_traj,
    vector<PiecewisePolynomial<double>>& lc_traj,
    vector<PiecewisePolynomial<double>>& vc_traj,
    const bool animate,
    const bool ipopt,
    std::vector<int> num_knot_points,
    const double min_final_height,
    const double initial_height,
    const double fore_aft_displacement,
    const double liftoff_velocity,
    const double pitch_magnitude,
    const double apex_height,
<<<<<<< HEAD
<<<<<<< HEAD
    const double apex_displacement,
    const double td_height,
    const double td_displacement,
<<<<<<< HEAD
=======
>>>>>>> It sort of works
=======
    const double apex_displacement,
>>>>>>> Cleaned up apex with constraints on position
=======
>>>>>>> Trying to go all the way
    const bool lock_rotation,
    const bool optimize_flight,
<<<<<<< HEAD
<<<<<<< HEAD
    const bool optimize_td,
    const bool optimize_stance,
=======
>>>>>>> Leap is working fairly well
=======
    const bool optimize_td,
<<<<<<< HEAD
>>>>>>> Jump seems to be working alright
=======
    const bool optimize_stance,
>>>>>>> Working getting to apex
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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  auto [modeVector, toeEvals, toeEvalSets] =  getModeSequence(plant, mu, num_knot_points, sequence, optimize_flight, optimize_td, optimize_stance);
=======
  auto [modeVector, toeEvals, toeEvalSets] =  optimize_flight ? getModeSequenceFlight(plant, mu, num_knot_points, sequence)
                                                              : getModeSequenceRear(plant, mu, num_knot_points, sequence);
>>>>>>> Leap is working fairly well
=======
  auto [modeVector, toeEvals, toeEvalSets] =  optimize_flight ? ( optimize_td ? getModeSequenceTouchdown(plant, mu, num_knot_points, sequence)
                                                                : getModeSequenceFlight(plant, mu, num_knot_points, sequence))
                                                                : getModeSequenceRear(plant, mu, num_knot_points, sequence);
>>>>>>> It sort of works
=======
  auto [modeVector, toeEvals, toeEvalSets] =  getModeSequence(plant, mu, num_knot_points, sequence, optimize_flight, optimize_td, false);
>>>>>>> Things are working ish
=======
  auto [modeVector, toeEvals, toeEvalSets] =  getModeSequence(plant, mu, num_knot_points, sequence, optimize_flight, optimize_td, optimize_stance);
>>>>>>> Working getting to apex

  ///Setup trajectory optimization
  auto trajopt = Dircon<T>(sequence);

<<<<<<< HEAD
<<<<<<< HEAD
  if (ipopt) {
=======
  if (FLAGS_ipopt) {
>>>>>>> Using IPOPT and first 4 optimizations working fine
=======
  if (ipopt) {
>>>>>>> Lowering acceptable tol seems to be pretty nice
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
<<<<<<< HEAD
<<<<<<< HEAD
    trajopt.SetSolverOption(id, "acceptable_obj_change_tol", tol);
    trajopt.SetSolverOption(id, "acceptable_tol", tol);
=======
    trajopt.SetSolverOption(id, "acceptable_obj_change_tol", 1e-3);
<<<<<<< HEAD
    trajopt.SetSolverOption(id, "acceptable_tol", 1e2);
>>>>>>> Using IPOPT and first 4 optimizations working fine
=======
    trajopt.SetSolverOption(id, "acceptable_tol", 1e-4);
>>>>>>> Lowering acceptable tol seems to be pretty nice
=======
    trajopt.SetSolverOption(id, "acceptable_obj_change_tol", tol);
    trajopt.SetSolverOption(id, "acceptable_tol", tol);
>>>>>>> Final opt working, uncommenting and testing everyhting
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

  // Setting up cost
  addCost(plant, trajopt, cost_actuation, cost_velocity, cost_velocity_legs_flight, cost_actuation_legs_flight, cost_time, cost_work, work_constraint_scale);


  // Initialize the trajectory control state and forces
  if (file_name_in.empty()){
    //trajopt.SetInitialGuessForAllVariables(
    //    VectorXd::Zero(trajopt.decision_variables().size()));
<<<<<<< HEAD
<<<<<<< HEAD
=======
    trajopt.drake::systems::trajectory_optimization::MultipleShooting::
        SetInitialTrajectory(u_traj, x_traj);
>>>>>>> Leap is working fairly well
    for (int j = 0; j < sequence.num_modes(); j++) {
      trajopt.SetInitialTrajectoryForMode(j, x_traj[j], u_traj, x_traj[j].start_time(), x_traj[j].end_time());
=======
    for (int j = 0; j < sequence.num_modes(); j++) {
<<<<<<< HEAD
      trajopt.SetInitialTrajectory(j, x_traj[j], u_traj, x_traj[j].start_time(), x_traj[j].end_time());
>>>>>>> Making some progress with new initial guess load in
=======
      trajopt.SetInitialTrajectoryForMode(j, x_traj[j], u_traj, x_traj[j].start_time(), x_traj[j].end_time());
>>>>>>> Using IPOPT and first 4 optimizations working fine
      trajopt.SetInitialForceTrajectory(j, l_traj[j], lc_traj[j],
                                        vc_traj[j], l_traj[j].start_time());
    }
  }else{
    std::cout<<"Loading decision var from file, will fail if num dec vars changed" <<std::endl;
    dairlib::DirconTrajectory loaded_traj(file_name_in);
    trajopt.SetInitialGuessForAllVariables(loaded_traj.GetDecisionVariables());
  }

  addConstraints(plant, trajopt, min_final_height,
                 initial_height, fore_aft_displacement, liftoff_velocity, pitch_magnitude,
                 lock_rotation, false, max_duration, eps);
<<<<<<< HEAD
<<<<<<< HEAD
  if(optimize_flight)
    addConstraintsFlight(plant, trajopt, apex_height, apex_displacement, eps);
  if(optimize_td)
    addConstraintsTD(plant, trajopt, td_height, td_displacement, eps);
  if(optimize_stance)
    addConstraintsStance(plant, trajopt, initial_height, td_displacement, eps);
=======
 if(optimize_flight)
<<<<<<< HEAD
    addConstraintsFlight(plant, trajopt);
<<<<<<< HEAD

>>>>>>> Leap is working fairly well
=======
=======
    addConstraintsFlight(plant, trajopt, apex_height);
<<<<<<< HEAD
>>>>>>> It converges
 if(optimize_flight)
=======
 if(optimize_td)
>>>>>>> Things are working ish
   addConstraintsTD(plant, trajopt, min_final_height, initial_height);
>>>>>>> It sort of works
=======
  if(optimize_flight)
    addConstraintsFlight(plant, trajopt, apex_height, apex_displacement, eps);
  if(optimize_td)
    addConstraintsTD(plant, trajopt, td_height, td_displacement, eps);
  if(optimize_stance)
<<<<<<< HEAD
    addConstraintsStance(plant, trajopt, initial_height);
>>>>>>> Working getting to apex
=======
    addConstraintsStance(plant, trajopt, initial_height, td_displacement, eps);
>>>>>>> Getting full leap succsessful

  /// Setup the visualization during the optimization
  int num_ghosts = 1;// Number of ghosts in visualization. NOTE: there are limitations on number of ghosts based on modes and knotpoints
  std::vector<unsigned int> visualizer_poses; // Ghosts for visualizing during optimization
  for(int i = 0; i < sequence.num_modes(); i++){
      visualizer_poses.push_back(num_ghosts); 
  }
  trajopt.CreateVisualizationCallback(
      dairlib::FindResourceOrThrow("examples/Spirit/spirit_drake.urdf"),
      visualizer_poses, 0.2); // setup which URDF, how many poses, and alpha transparency 

<<<<<<< HEAD
<<<<<<< HEAD
  drake::solvers::SolverId solver_id("");
  if (ipopt) {
=======
  drake::solvers::SolverId solver_id("");
<<<<<<< HEAD
  if (FLAGS_ipopt) {
>>>>>>> Using IPOPT and first 4 optimizations working fine
=======
  if (ipopt) {
>>>>>>> Lowering acceptable tol seems to be pretty nice
    solver_id = drake::solvers::IpoptSolver().id();
    cout << "\nChose manually: " << solver_id.name() << endl;
  } else {
    solver_id = drake::solvers::ChooseBestSolver(trajopt);
    cout << "\nChose the best solver: " << solver_id.name() << endl;
  }

<<<<<<< HEAD
=======
>>>>>>> It sort of works
=======
>>>>>>> Using IPOPT and first 4 optimizations working fine
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

  std::vector<PiecewisePolynomial<double>> x_traj;
  PiecewisePolynomial<double> u_traj;
  std::vector<PiecewisePolynomial<double>> l_traj;
  std::vector<PiecewisePolynomial<double>> lc_traj;
  std::vector<PiecewisePolynomial<double>> vc_traj;

  if (FLAGS_runAllOptimization){
    if(!FLAGS_skipInitialOptimization){
      std::cout<<"Running 1st optimization"<<std::endl;
      //Hopping correct distance, but heavily constrained
<<<<<<< HEAD
<<<<<<< HEAD
      dairlib::badSpiritRear(*plant, x_traj, u_traj, l_traj, lc_traj, vc_traj);
<<<<<<< HEAD
=======
      dairlib::badSpiritJump(*plant,x_traj,u_traj,l_traj,lc_traj,vc_traj);
>>>>>>> Refactored joint limits
=======
      dairlib::badSpiritRear(*plant, x_traj, u_traj, l_traj, lc_traj, vc_traj);
<<<<<<< HEAD
>>>>>>> Leap is working fairly well
=======
>>>>>>> Using IPOPT and first 4 optimizations working fine
      dairlib::runSpiritJump<double>(
          *plant,
          x_traj, u_traj, l_traj,
          lc_traj, vc_traj,
          false,
<<<<<<< HEAD
<<<<<<< HEAD
          true,
=======
>>>>>>> Using IPOPT and first 4 optimizations working fine
=======
          true,
>>>>>>> Lowering acceptable tol seems to be pretty nice
          {10, 7, 5, 5, 5, 5} ,
          0.3,
          0.2,
          0,
          0,
          0.4,
          0,
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          0,
          0,
          0,
<<<<<<< HEAD
=======
>>>>>>> It sort of works
=======
          0,
>>>>>>> Cleaned up apex with constraints on position
=======
          0,
          0,
>>>>>>> Trying to go all the way
          true,
<<<<<<< HEAD
<<<<<<< HEAD
          false,
          false,
<<<<<<< HEAD
          false,
<<<<<<< HEAD
=======
          true, //Does not do anything
>>>>>>> Seems to be working, messing with 3rd op
=======
          false,
>>>>>>> Leap is working fairly well
=======
>>>>>>> Jump seems to be working alright
=======
          false,
>>>>>>> Working getting to apex
=======
          0,
          0,
          0,
          true,
          false,
          false,
          false,
>>>>>>> Using IPOPT and first 4 optimizations working fine
          0.5,
          0.3,
          1,
          5,
          10,
          2000,
          0,
          100000,
          0,
          1e-4,
          1.0,
          FLAGS_data_directory+"simple_rear");

      std::cout<<"Running 2nd optimization"<<std::endl;

<<<<<<< HEAD
=======
//      dairlib::runSpiritJump<double>(
//          *plant,
//          x_traj, u_traj, l_traj,
//          lc_traj, vc_traj,
//          false,
//          {10, 7, 5, 5, 5, 5} ,
//          0.3,
//          0.2,
//          0,
//          0,
//          0.4,
//          0,
//          0,
//          0,
//          0,
//          true,
//          false,
//          false,
//          false,
//          0.5,
//          0.3,
//          1,
//          5,
//          10,
//          2000,
//          0,
//          100000,
//          0,
//          1e-4,
//          1.0,
//          FLAGS_data_directory+"simple_rear");
//
//      std::cout<<"Running 2nd optimization"<<std::endl;
//
>>>>>>> I really should have commited this a long time ago
=======
>>>>>>> Using IPOPT and first 4 optimizations working fine
      dairlib::runSpiritJump<double>(
          *plant,
          x_traj, u_traj, l_traj,
          lc_traj, vc_traj,
          false,
          true,
          {10, 7, 5, 5, 5, 5} ,
          0.3,
          0.2,
          0,
          0,
          0.4,
          0,
<<<<<<< HEAD
<<<<<<< HEAD
          0,
          0,
          0,
=======
>>>>>>> It sort of works
=======
          0,
<<<<<<< HEAD
>>>>>>> Cleaned up apex with constraints on position
=======
          0,
          0,
>>>>>>> Trying to go all the way
          false,
          false,
          false,
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
          false,
<<<<<<< HEAD
=======
          true, //Does not do anything
>>>>>>> Seems to be working, messing with 3rd op
=======
>>>>>>> Leap is working fairly well
=======
>>>>>>> Jump seems to be working alright
=======
          false,
>>>>>>> Working getting to apex
          0.8,
          3,
          20,
          5,
          10,
          2000,
          0,
          100000,
          1e-2,
          1e-4,
          1.0,
          FLAGS_data_directory+"simple_rear2");
    }
//
//    std::cout<<"Running 3rd optimization"<<std::endl;
//
//    dairlib::runSpiritJump<double>(
//        *plant,
//        x_traj, u_traj, l_traj,
//        lc_traj, vc_traj,
//        false,
//        true,
//        {10, 7, 5, 5, 5, 5} ,
//        0.35,
//        FLAGS_standHeight,
//        0.15,
//        0.5,
//        0.4,
//        0,
//        0,
//        0,
//        0,
//        false,
//        false,
//        false,
//        false,
//        0.8,
//        3,
//        20,
//        5,
//        10,
//        4000,
//        0,
//        100000,
//        1e-2,
//        1e-4,
//        1.0,
//        FLAGS_data_directory+"simple_rear3",
//        FLAGS_data_directory+"simple_rear2");
//
//    dairlib::DirconTrajectory old_traj(FLAGS_data_directory+"simple_rear3");
//    x_traj = old_traj.ReconstructStateDiscontinuousTrajectory();
//    u_traj = old_traj.ReconstructInputTrajectory();
//    l_traj = old_traj.ReconstructLambdaTrajectory();
//    lc_traj = old_traj.ReconstructLambdaCTrajectory();
//    vc_traj = old_traj.ReconstructGammaCTrajectory();
//
//    dairlib::appendFlight(*plant, x_traj, u_traj, l_traj, lc_traj, vc_traj);
//
//    std::cout<<"Running 4th optimization"<<std::endl;
//    dairlib::runSpiritJump<double>(
//        *plant,
//        x_traj, u_traj, l_traj,
//        lc_traj, vc_traj,
//        true,
//        true,
//        {10, 7, 7, 7, 7, 7} ,
//        0.35,
//        FLAGS_standHeight,
//        0.06,
//        1.8,
//        0.6,
//        0.4,
//        0.5,
//        0,
//        0,
//        false,
//        true,
//        false,
//        false,
//        1.8,
//        3,
//        10,
//        5,
//        10,
//        4000,
//        0,
//        100000,
//        1e-2,
//        1e-6,
//        1.0,
//        FLAGS_data_directory+"half_leap");


//  dairlib::DirconTrajectory old_traj2(FLAGS_data_directory+"half_leap");
//  x_traj = old_traj2.ReconstructStateDiscontinuousTrajectory();
//  u_traj = old_traj2.ReconstructInputTrajectory();
//  l_traj = old_traj2.ReconstructLambdaTrajectory();
//  lc_traj = old_traj2.ReconstructLambdaCTrajectory();
//  vc_traj = old_traj2.ReconstructGammaCTrajectory();
//
//    dairlib::appendFrontTD(*plant, x_traj, u_traj, l_traj, lc_traj, vc_traj, 0.25);
//    dairlib::appendStance(*plant, x_traj, u_traj, l_traj, lc_traj, vc_traj, FLAGS_standHeight);
//
//    std::cout<<"Running 5th optimization"<<std::endl;
//    dairlib::runSpiritJump<double>(
//        *plant,
//        x_traj, u_traj, l_traj,
//        lc_traj, vc_traj,
//        false,
//        true,
//        {7, 7, 7, 7, 7, 7} ,
//        0.35,     // Only active small number modes
//        FLAGS_standHeight,
//        0.06, // Only active small number modes
//        1.8,       // Only active small number modes
//        0.6,
//        0.43,
//        0.5,
//        -1.00,
//        1,
//        false,
//        true,
//        true,
//        true,
//        1.8,
//        3,
//        10,
//        10/5.0,
//        5/5.0,
//        50000,
//        0,
//        100000,
//        1e-2,
//        1e-3,
//        1.0,
//        FLAGS_data_directory+"three_quarter_leap");

//    std::cout<<"Running 6th optimization"<<std::endl;
//    dairlib::runSpiritJump<double>(
//        *plant,
//        x_traj, u_traj, l_traj,
//        lc_traj, vc_traj,
//        true,
//        true,
//        {7, 7, 7, 7, 7, 7} ,
//        0.35,     // Only active small number modes
//        FLAGS_standHeight,
//        0.06, // Only active small number modes
//        1.8,       // Only active small number modes
//        0.6,
//        0.43,       // Ignored if small
//        0.5,   // Ignored if negative
//        -1.00,       // Ignored if negative
//        1,
//        false,
//        true,
//        true,
//        true,
//        1.8,
//        3,
//        10,
//        10/5.0,
//        5/5.0,
//        50000,
//        10,
//        100000,
//        1e-2,
//        1e-3,
//        1.0,
//        FLAGS_data_directory+"full_leap",
//        FLAGS_data_directory+"three_quarter_leap");

    std::cout<<"Running 6th optimization"<<std::endl;
    dairlib::runSpiritJump<double>(
        *plant,
        x_traj, u_traj, l_traj,
        lc_traj, vc_traj,
        true,
        true,
        {7, 7, 7, 7, 7, 7} ,
        0.35,     // Only active small number modes
        FLAGS_standHeight,
        0.06, // Only active small number modes
        1.8,       // Only active small number modes
        0.6,
        0.43,       // Ignored if small
        -0.5,   // Ignored if negative
        -1.00,       // Ignored if negative
        1,
        false,
        true,
        true,
        true,
        1.8,
        3,
        10,
        10/5.0,
        5/5.0,
        50000,
        100,
        100000,
        1e-2,
        1e-4,
        1.0,
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        FLAGS_data_directory+"half_leap");


  dairlib::DirconTrajectory old_traj2(FLAGS_data_directory+"half_leap");
  x_traj = old_traj2.ReconstructStateTrajectory();
  u_traj = old_traj2.ReconstructInputTrajectory();
  l_traj = old_traj2.ReconstructLambdaTrajectory();
  lc_traj = old_traj2.ReconstructLambdaCTrajectory();
  vc_traj = old_traj2.ReconstructGammaCTrajectory();

  dairlib::appendFrontTD(*plant, x_traj, u_traj, l_traj, lc_traj, vc_traj, 0.32);

  std::cout<<"Running 5th optimization"<<std::endl;
  dairlib::runSpiritJump<double>(
      *plant,
      x_traj, u_traj, l_traj,
      lc_traj, vc_traj,
      true,
      {5, 5, 3, 3, 5, 5} ,
      0.35,
      FLAGS_standHeight,
      0.1,
      0.8,
      0.4,
      0,
      0,
      false,
      true,
      true,
      false,
      1.8,
      .3,
      .5,
      15,
      20,
      500,
      0,
      100000,
      1e-2,
      1e-3,
      1.0,
      FLAGS_data_directory+"three_quarter_leap");
<<<<<<< HEAD
<<<<<<< HEAD

    std::cout<<"Running 6th optimization"<<std::endl;
    dairlib::runSpiritJump<double>(
        *plant,
        x_traj, u_traj, l_traj,
        lc_traj, vc_traj,
        false,
        true,
        {10, 7, 5, 5, 5, 5} ,
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        0.35,
=======
        0.4,
>>>>>>> Seems to be working, messing with 3rd op
        FLAGS_standHeight,
        0.15,
        0.5,
=======
        0.35,
        FLAGS_standHeight,
<<<<<<< HEAD
        FLAGS_foreAftDisplacement,
        2.0,
>>>>>>> Refactored joint limits
=======
        0.15,
        0.5,
>>>>>>> Leap start
=======
        0.4,
        FLAGS_standHeight,
        0.02,
        1.5,
>>>>>>> Leap is working fairly well
=======
        0.35,
        FLAGS_standHeight,
        0.02,
        1,
>>>>>>> Jump seems to be working alright
        0.4,
<<<<<<< HEAD
        0,
        0,
        0,
        0,
        false,
        false,
        false,
<<<<<<< HEAD
        false,
<<<<<<< HEAD
=======
        false, //Does not do anything
>>>>>>> Seems to be working, messing with 3rd op
        0.8,
        3,
        20,
        5,
        10,
<<<<<<< HEAD
        4000,
=======
        2000,
>>>>>>> Seems to be working, messing with 3rd op
=======
=======
        0.4,
        true,
        true,
>>>>>>> It sort of works
        true,
        1.8,
        .3,
        .5,
        5,
        10,
        100,
<<<<<<< HEAD
<<<<<<< HEAD
        500,
>>>>>>> Leap is working fairly well
        0,
        100000,
        1e-2,
        1e-4,
        1.0,
<<<<<<< HEAD
        FLAGS_data_directory+"simple_rear3",
        FLAGS_data_directory+"simple_rear2");

    dairlib::DirconTrajectory old_traj(FLAGS_data_directory+"simple_rear3");
    x_traj = old_traj.ReconstructStateDiscontinuousTrajectory();
    u_traj = old_traj.ReconstructInputTrajectory();
    l_traj = old_traj.ReconstructLambdaTrajectory();
    lc_traj = old_traj.ReconstructLambdaCTrajectory();
    vc_traj = old_traj.ReconstructGammaCTrajectory();

    dairlib::appendFlight(*plant, x_traj, u_traj, l_traj, lc_traj, vc_traj);

    std::cout<<"Running 4th optimization"<<std::endl;
    dairlib::runSpiritJump<double>(
        *plant,
        x_traj, u_traj, l_traj,
        lc_traj, vc_traj,
        false,
        true,
        {10, 7, 7, 7, 7, 7} ,
        0.35,
        FLAGS_standHeight,
        0.06,
        1.8,
        0.6,
        0.4,
        0.5,
        0,
        0,
        false,
        true,
        false,
        false,
        1.8,
        3,
        10,
        5,
        10,
        4000,
=======
        1000,
>>>>>>> Jump seems to be working alright
        0,
        100000,
        1e-2,
        1e-3,
        1.0,
        FLAGS_data_directory+"half_leap");


  dairlib::DirconTrajectory old_traj2(FLAGS_data_directory+"half_leap");
  x_traj = old_traj2.ReconstructStateDiscontinuousTrajectory();
  u_traj = old_traj2.ReconstructInputTrajectory();
  l_traj = old_traj2.ReconstructLambdaTrajectory();
  lc_traj = old_traj2.ReconstructLambdaCTrajectory();
  vc_traj = old_traj2.ReconstructGammaCTrajectory();

    dairlib::appendFrontTD(*plant, x_traj, u_traj, l_traj, lc_traj, vc_traj, 0.25);
    dairlib::appendStance(*plant, x_traj, u_traj, l_traj, lc_traj, vc_traj, FLAGS_standHeight);

    std::cout<<"Running 5th optimization"<<std::endl;
    dairlib::runSpiritJump<double>(
        *plant,
        x_traj, u_traj, l_traj,
        lc_traj, vc_traj,
        false,
        true,
        {7, 7, 7, 7, 7, 7} ,
        0.35,     // Only active small number modes
        FLAGS_standHeight,
        0.06, // Only active small number modes
        1.8,       // Only active small number modes
        0.6,
        0.43,
        0.5,
        -1.00,
        1,
        false,
        true,
        true,
        true,
        1.8,
        3,
        10,
        10/5.0,
        5/5.0,
        50000,
=======
>>>>>>> It sort of works
        0,
        100000,
        1e-2,
        1e-3,
        1.0,
        FLAGS_data_directory+"three_quarter_leap");

    std::cout<<"Running 6th optimization"<<std::endl;
    dairlib::runSpiritJump<double>(
        *plant,
        x_traj, u_traj, l_traj,
        lc_traj, vc_traj,
        true,
        true,
        {7, 7, 7, 7, 7, 7} ,
        0.35,     // Only active small number modes
        FLAGS_standHeight,
        0.06, // Only active small number modes
        1.8,       // Only active small number modes
        0.6,
        0.43,       // Ignored if small
        0.5,   // Ignored if negative
        -1.00,       // Ignored if negative
        1,
        false,
        true,
        true,
        true,
        1.8,
        3,
        10,
        10/5.0,
        5/5.0,
        50000,
        10,
        100000,
        1e-2,
        1e-3,
        1.0,
        FLAGS_data_directory+"full_leap",
        FLAGS_data_directory+"three_quarter_leap");

    std::cout<<"Running 6th optimization"<<std::endl;
    dairlib::runSpiritJump<double>(
        *plant,
        x_traj, u_traj, l_traj,
        lc_traj, vc_traj,
        true,
        true,
        {7, 7, 7, 7, 7, 7} ,
        0.35,     // Only active small number modes
        FLAGS_standHeight,
        0.06, // Only active small number modes
        1.8,       // Only active small number modes
        0.6,
        0.43,       // Ignored if small
        -0.5,   // Ignored if negative
        -1.00,       // Ignored if negative
        1,
        false,
        true,
        true,
        true,
        1.8,
        3,
        10,
        10/5.0,
        5/5.0,
        50000,
        100,
        100000,
        1e-2,
        1e-4,
        1.0,
<<<<<<< HEAD
        FLAGS_data_directory+"full_leap_min_work",
        FLAGS_data_directory+"full_leap");
=======
        FLAGS_data_directory+"half_leap");
>>>>>>> Leap is working fairly well
=======
        FLAGS_data_directory+"three_quarter_leap2",
        FLAGS_data_directory+"three_quarter_leap");
>>>>>>> It sort of works
=======
>>>>>>> Things are working ish
=======
=======
        FLAGS_data_directory+"three_quarter_leap");
>>>>>>> Trying to go all the way

    dairlib::DirconTrajectory old_traj3(FLAGS_data_directory+"three_quarter_leap");
    x_traj = old_traj3.ReconstructStateTrajectory();
    u_traj = old_traj3.ReconstructInputTrajectory();
    l_traj = old_traj3.ReconstructLambdaTrajectory();
    lc_traj = old_traj3.ReconstructLambdaCTrajectory();
    vc_traj = old_traj3.ReconstructGammaCTrajectory();


    std::cout<<"Running 6th optimization"<<std::endl;
  dairlib::runSpiritJump<double>(
      *plant,
      x_traj, u_traj, l_traj,
      lc_traj, vc_traj,
      true,
      {5, 5, 3, 3, 5, 5} ,
      0.35,
      FLAGS_standHeight,
      0.1,
      0.8,
      0.4,
      0,
      0,
      0,
      0,
      false,
      true,
      true,
      true,
      1.8,
      .3,
      .5,
      150,
      20,
      500,
      0,
      100000,
      1e-2,
      1e-3,
      1.0,
      FLAGS_data_directory+"full_leap");
>>>>>>> Working getting to apex
=======
        FLAGS_data_directory+"full_leap",
        FLAGS_data_directory+"three_quarter_leap");
>>>>>>> Starting work on full leap
=======
        FLAGS_data_directory+"full_leap");
>>>>>>> Making some progress with new initial guess load in
=======
        FLAGS_data_directory+"full_leap",
        FLAGS_data_directory+"three_quarter_leap");
>>>>>>> Optimization success with pipeline
=======
=======
        FLAGS_data_directory+"full_leap_min_work",
>>>>>>> Lowering acceptable tol seems to be pretty nice
        FLAGS_data_directory+"full_leap");
>>>>>>> I really should have commited this a long time ago
  }
}

