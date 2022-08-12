#include "examples/Spirit_spine/spirit_turn.h"
#include <yaml-cpp/yaml.h>


using drake::multibody::MultibodyPlant;
using drake::trajectories::PiecewisePolynomial;
using Eigen::VectorXd;
using Eigen::MatrixXd;

namespace dairlib {
using systems::trajectory_optimization::DirconModeSequence;
using systems::trajectory_optimization::Dircon;

template <class Y> 
SpiritTurn<Y>::SpiritTurn(){
}

/// Assigns values to member variables according to input yaml file
/// \param yaml_path path of the yaml file
/// \param saved_directory directory for saving the output trajectories (namely for setting file_name_out)
/// \param index indicates that we are using the index^th configuration
/// \param plant: robot model
template <class Y>
void SpiritTurn<Y>::config(std::string yaml_path, std::string saved_directory, int index,MultibodyPlant<Y>* plant){
  YAML::Node config = YAML::LoadFile(yaml_path);
  this->index=index;
  this->ipopt=config[index]["ipopt"].as<bool>();
  this->num_knot_points=config[index]["num_knot_points"].as<std::vector<int>>();
  this->initial_height=config[index]["initial_height"].as<double>();
  this->pitch_magnitude_lo=config[index]["pitch_magnitude_lo"].as<double>();
  this->max_roll=config[index]["max_roll"].as<double>();
  this->max_pitch=config[index]["max_pitch"].as<double>();
  this->pitch_magnitude_apex=config[index]["pitch_magnitude_apex"].as<double>();
  this->orientation_diff=config[index]["orientation_diff"].as<double>();
  this->lock_spine=config[index]["lock_spine"].as<bool>();
  this->apex_height=config[index]["apex_height"].as<double>();
  this->speed=config[index]["speed"].as<double>();
  this->max_duration=config[index]["max_duration"].as<double>();
  this->lock_leg_apex=config[index]["lock_leg_apex"].as<bool>();
  this->cost_actuation=config[index]["cost_actuation"].as<double>();
  this->cost_velocity=config[index]["cost_velocity"].as<double>();
  this->cost_velocity_legs_flight=config[index]["cost_velocity_legs_flight"].as<double>();
  this->cost_actuation_legs_flight=config[index]["cost_actuation_legs_flight"].as<double>();
  this->cost_power=config[index]["cost_power"].as<double>();
  this->cost_time=config[index]["cost_time"].as<double>();
  this->mu=config[index]["mu"].as<double>();
  this->eps=config[index]["eps"].as<double>();
  this->tol=config[index]["tol"].as<double>();

  if(!config[index]["file_name_out"].as<std::string>().empty()) this->file_name_out=saved_directory+config[index]["file_name_out"].as<std::string>();
  if(!config[index]["file_name_in"].as<std::string>().empty()) this->file_name_in= saved_directory+config[index]["file_name_in"].as<std::string>();

}


/// Generate a bad initial guess for the spirit bound traj opt
/// \param plant: robot model
template <class Y>
void SpiritTurn<Y>::generateInitialGuess(MultibodyPlant<Y>& plant){
  double stand_height = this->initial_height;
  double pitch_lo = -0.1;
  double apex_height = this->apex_height;
  double duration = 0.2;
  std::vector<MatrixXd> x_points;
  VectorXd x_const;

  
  //Flight 1
  dairlib::ikSpiritStand(plant, x_const, {false, false, false, false}, apex_height, 0.2, 0, 0);
  x_points.push_back(x_const);
  dairlib::ikSpiritStand(plant, x_const, {true, false, true, false}, stand_height+0.05, 0.2, 0, -pitch_lo);
  x_points.push_back(x_const);

  this->x_traj.push_back(PiecewisePolynomial<double>::FirstOrderHold({0 * duration/3.0, 1 * duration/3.0},x_points));
  x_points.clear();

  //Front stance
  dairlib::ikSpiritStand(plant, x_const, {true, false, true, false}, stand_height+0.05, 0.2, 0, -pitch_lo);
  x_points.push_back(x_const);
  dairlib::ikSpiritStand(plant, x_const, {true, false, true, false}, stand_height+0.02, 0.2, 0, -pitch_lo/2.0);
  x_points.push_back(x_const);

  this->x_traj.push_back(PiecewisePolynomial<double>::FirstOrderHold({1 * duration/3.0, 2 * duration/3.0},x_points));
  x_points.clear();

  //Flight 2
  dairlib::ikSpiritStand(plant, x_const, {true, false, true, false}, stand_height+0.02, 0.2, 0, -pitch_lo/2.0);
  x_points.push_back(x_const);
  dairlib::ikSpiritStand(plant, x_const, {false, false, false, false}, stand_height+0.05, 0.2, 0, 0);
  x_points.push_back(x_const);
  this->x_traj.push_back(PiecewisePolynomial<double>::FirstOrderHold({2 * duration/3.0, 3 * duration/3.0},x_points));
  x_points.clear();

  //Flight 3
  dairlib::ikSpiritStand(plant, x_const, {false, false, false, false}, stand_height+0.05, 0.2, 0, 0);
  x_points.push_back(x_const);
  dairlib::ikSpiritStand(plant, x_const, {false, true, false, true}, stand_height+0.02, 0.2, 0, pitch_lo/2.0);
  x_points.push_back(x_const);

  this->x_traj.push_back(PiecewisePolynomial<double>::FirstOrderHold({3 * duration/3.0, 4 * duration/3.0},x_points));
  x_points.clear();

  //Rare stance
  dairlib::ikSpiritStand(plant, x_const, {false, true, false, true}, stand_height+0.02, 0.2, 0, pitch_lo/2.0);
  x_points.push_back(x_const);
  dairlib::ikSpiritStand(plant, x_const, {false, true, false, true}, stand_height+0.05, 0.2, 0, pitch_lo);
  x_points.push_back(x_const);

  this->x_traj.push_back(PiecewisePolynomial<double>::FirstOrderHold({4 * duration/3.0, 5* duration/3.0},x_points));
  x_points.clear();

  //Flight 4
  dairlib::ikSpiritStand(plant, x_const, {false, true, false, true}, stand_height+0.05, 0.2, 0, pitch_lo);
  x_points.push_back(x_const);
  dairlib::ikSpiritStand(plant, x_const, {false, false, false, false}, apex_height, 0.2, 0, 0);
  x_points.push_back(x_const);

  this->x_traj.push_back(PiecewisePolynomial<double>::FirstOrderHold({5 * duration/3.0, 6 * duration/3.0},x_points));
  x_points.clear();


  // Create control initial guess (zeros)
  std::vector<MatrixXd> u_points;
  u_points.push_back(MatrixXd::Zero( plant.num_actuators(), 1));
  u_points.push_back(MatrixXd::Zero( plant.num_actuators(), 1));

  this->u_traj = PiecewisePolynomial<double>::FirstOrderHold({0, 6 * duration/3.0},u_points);

  // Contact force initial guess
  // Four contacts so forces are 12 dimensional
  Eigen::VectorXd init_l_vec(12);
  // Initial guess
  init_l_vec << 0, 0, 12*9.81, 0, 0, 12*9.81, 0, 0, 12*9.81, 0, 0, 12*9.81; //gravity and mass distributed

  Eigen::VectorXd init_l_vec_front_stance(12);
  init_l_vec_front_stance << 0, 0, 0, 0, 0, 24*9.81, 0, 0, 0, 0, 0, 24*9.81; //gravity and mass distributed

  Eigen::VectorXd init_l_vec_back_stance(12);
  init_l_vec_back_stance << 0, 0, 24*9.81, 0, 0, 0, 0, 0, 24*9.81, 0, 0, 0; //gravity and mass distributed

  //Stance
  int N = 10;
  std::vector<MatrixXd> init_l_j;
  std::vector<MatrixXd> init_lc_j;
  std::vector<MatrixXd> init_vc_j;
  std::vector<double> init_time_j;

  // flight 1
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

  this->l_traj.push_back(init_l_traj_j);
  this->lc_traj.push_back(init_lc_traj_j);
  this->vc_traj.push_back(init_vc_traj_j);

  // front stance
  init_l_j.clear();
  init_lc_j.clear();
  init_vc_j.clear();
  init_time_j.clear();
  for (int i = 0; i < N; i++) {
    init_time_j.push_back(i * duration /3.0/ (N - 1) + duration/3.0);
    init_l_j.push_back(init_l_vec_front_stance);
    init_lc_j.push_back(init_l_vec_front_stance);
    init_vc_j.push_back(VectorXd::Zero(12));
  }

  init_l_traj_j = PiecewisePolynomial<double>::ZeroOrderHold(init_time_j,init_l_j);
  init_lc_traj_j = PiecewisePolynomial<double>::ZeroOrderHold(init_time_j,init_lc_j);
  init_vc_traj_j = PiecewisePolynomial<double>::ZeroOrderHold(init_time_j,init_vc_j);

  this->l_traj.push_back(init_l_traj_j);
  this->lc_traj.push_back(init_lc_traj_j);
  this->vc_traj.push_back(init_vc_traj_j);

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

  this->l_traj.push_back(init_l_traj_j);
  this->lc_traj.push_back(init_lc_traj_j);
  this->vc_traj.push_back(init_vc_traj_j);

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

  this->l_traj.push_back(init_l_traj_j);
  this->lc_traj.push_back(init_lc_traj_j);
  this->vc_traj.push_back(init_vc_traj_j);

  // back stance
  init_l_j.clear();
  init_lc_j.clear();
  init_vc_j.clear();
  init_time_j.clear();
  for (int i = 0; i < N; i++) {
    init_time_j.push_back(i * duration /3.0/ (N - 1) +4.0 * duration/3.0);
    init_l_j.push_back(VectorXd::Zero(12));
    init_lc_j.push_back(VectorXd::Zero(12));
    init_vc_j.push_back(VectorXd::Zero(12));
  }

  init_l_traj_j = PiecewisePolynomial<double>::ZeroOrderHold(init_time_j,init_l_j);
  init_lc_traj_j = PiecewisePolynomial<double>::ZeroOrderHold(init_time_j,init_lc_j);
  init_vc_traj_j = PiecewisePolynomial<double>::ZeroOrderHold(init_time_j,init_vc_j);

  this->l_traj.push_back(init_l_traj_j);
  this->lc_traj.push_back(init_lc_traj_j);
  this->vc_traj.push_back(init_vc_traj_j);

  // flight
  init_l_j.clear();
  init_lc_j.clear();
  init_vc_j.clear();
  init_time_j.clear();
  for (int i = 0; i < N; i++) {
    init_time_j.push_back(i * duration /3.0/ (N - 1) + 5 * duration/3.0);
    init_l_j.push_back(init_l_vec_back_stance);
    init_lc_j.push_back(init_l_vec_back_stance);
    init_vc_j.push_back(VectorXd::Zero(12));
  }

  init_l_traj_j = PiecewisePolynomial<double>::ZeroOrderHold(init_time_j,init_l_j);
  init_lc_traj_j = PiecewisePolynomial<double>::ZeroOrderHold(init_time_j,init_lc_j);
  init_vc_traj_j = PiecewisePolynomial<double>::ZeroOrderHold(init_time_j,init_vc_j);

  this->l_traj.push_back(init_l_traj_j);
  this->lc_traj.push_back(init_lc_traj_j);
  this->vc_traj.push_back(init_vc_traj_j);
  
}


/// add cost for legs
/// \param joints add cost for which joint of the legs
/// \param mode_index add most for which mode 
template <class Y>
void SpiritTurn<Y>::addCostLegs(MultibodyPlant<Y>& plant,
                 dairlib::systems::trajectory_optimization::Dircon<Y>& trajopt,
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
      trajopt.AddCost(hi/2.0 * this->cost_velocity_legs_flight* (vel_sq + vel_sq_up));

      auto act  = trajopt.input(trajopt.get_mode_start(mode_index) + knot_index)(actuator_map.at("motor_" + std::to_string(joint_index)));
      auto act_up = trajopt.input(trajopt.get_mode_start(mode_index) + knot_index+1)(actuator_map.at("motor_" + std::to_string(joint_index)));

      drake::symbolic::Expression act_sq = act * act;
      drake::symbolic::Expression act_sq_up = act_up * act_up;

      trajopt.AddCost(hi/2.0 * this->cost_actuation_legs_flight * (act_sq + act_sq_up));
    }
  }
}

/// Adds cost to the trajopt problem
/// \param plant robot model
/// \param trajopt trajectory optimization problem to be solved
template <class Y>
std::vector<drake::solvers::Binding<drake::solvers::Cost>> SpiritTurn<Y>::addCost(
            MultibodyPlant<Y>& plant,
            dairlib::systems::trajectory_optimization::Dircon<Y>& trajopt){
  auto u   = trajopt.input();
  auto x   = trajopt.state();
  
  // Setup the traditional cost function
  trajopt.AddRunningCost( u.transpose()*this->cost_actuation*u);
  trajopt.AddVelocityCost(this->cost_velocity);
  trajopt.AddRunningCost(this->cost_time);

  // Hard code which joints are in flight for which mode

  addCostLegs(plant, trajopt,  {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}, 0);
  addCostLegs(plant, trajopt,  {2, 3, 6, 7, 10, 11}, 1);
  addCostLegs(plant, trajopt,  {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}, 2);
  addCostLegs(plant, trajopt, {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}, 3);
  addCostLegs(plant, trajopt,  {0, 1, 4, 5, 8, 10}, 4);
  addCostLegs(plant, trajopt, {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}, 5);
  return AddPowerCost(plant, trajopt, this->cost_power);
}
/// Adds constraints to the trajopt bound problem
/// \param plant robot model
/// \param trajopt trajectory optimization problem to be solved
template <class Y>
void SpiritTurn<Y>::addConstraints(
                    MultibodyPlant<Y>& plant,
                    dairlib::systems::trajectory_optimization::Dircon<Y>& trajopt){
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
  auto x0  = trajopt.initial_state();
  auto xtd = trajopt.state_vars(1,0);
  auto xapex  = trajopt.state_vars(3,0) ;
  auto xlo  =  trajopt.state_vars(4,0);
  auto xf  = trajopt.final_state();


  /// Initial constraint

  // xyz position
  trajopt.AddBoundingBoxConstraint(0, 0, x0(positions_map.at("base_x"))); // Give the initial condition room to choose the x_init position
  trajopt.AddBoundingBoxConstraint( -eps, eps, x0( positions_map.at("base_y")));
  if (apex_height>0) trajopt.AddBoundingBoxConstraint( this->apex_height-eps, this->apex_height+eps, x0( positions_map.at("base_z")));


  // Set initial state to be identical with the input traj
  for (int i=0;i<n_q+n_v;i++){
    trajopt.AddBoundingBoxConstraint(states_traj.value(0)(i,0)-0.1*eps, states_traj.value(0)(i,0)+0.1*eps, x0(i));
  }

  // velocities
  // trajopt.AddBoundingBoxConstraint(this->speed-eps, this->speed+eps, x0(plant.num_positions()+velocities_map.at("base_vx")));
  // trajopt.AddBoundingBoxConstraint(-eps, eps, x0(plant.num_positions()+velocities_map.at("base_vy")));
  trajopt.AddBoundingBoxConstraint(-eps, eps, x0(plant.num_positions()+velocities_map.at("base_vz")));

  // // Limit magnitude of pitch
  // trajopt.AddBoundingBoxConstraint(cos(this->pitch_magnitude_apex/2.0), 1, x0(positions_map.at("base_qw")));
  // trajopt.AddBoundingBoxConstraint(-eps, eps, x0(positions_map.at("base_qx")));
  // trajopt.AddBoundingBoxConstraint(-sin(this->pitch_magnitude_apex/2.0) , sin(this->pitch_magnitude_apex/2.0), x0(positions_map.at("base_qy")));
  // trajopt.AddBoundingBoxConstraint(-eps, eps, x0(positions_map.at("base_qz")));


  if (lock_leg_apex){
    double upperSet = 1;
    double kneeSet = 2;
    trajopt.AddBoundingBoxConstraint(upperSet - eps, upperSet + eps, x0(positions_map.at("joint_0") ) );
    trajopt.AddBoundingBoxConstraint(kneeSet - eps, kneeSet + eps, x0(positions_map.at("joint_1") ) );

    trajopt.AddBoundingBoxConstraint(upperSet - eps, upperSet + eps, x0(positions_map.at("joint_2") ) );
    trajopt.AddBoundingBoxConstraint(kneeSet - eps, kneeSet + eps, x0(positions_map.at("joint_3") ) );

    trajopt.AddBoundingBoxConstraint(upperSet - eps, upperSet + eps, x0(positions_map.at("joint_4") ) );
    trajopt.AddBoundingBoxConstraint(kneeSet - eps, kneeSet + eps, x0(positions_map.at("joint_5") ) );

    trajopt.AddBoundingBoxConstraint(upperSet - eps, upperSet + eps, x0(positions_map.at("joint_6") ) );
    trajopt.AddBoundingBoxConstraint(kneeSet - eps, kneeSet + eps, x0(positions_map.at("joint_7") ) );

    trajopt.AddBoundingBoxConstraint(upperSet - eps, upperSet + eps, xapex(positions_map.at("joint_0") ) );
    trajopt.AddBoundingBoxConstraint(kneeSet - eps, kneeSet + eps, xapex(positions_map.at("joint_1") ) );

    trajopt.AddBoundingBoxConstraint(upperSet - eps, upperSet + eps, xapex(positions_map.at("joint_2") ) );
    trajopt.AddBoundingBoxConstraint(kneeSet - eps, kneeSet + eps, xapex(positions_map.at("joint_3") ) );

    trajopt.AddBoundingBoxConstraint(upperSet - eps, upperSet + eps, xapex(positions_map.at("joint_4") ) );
    trajopt.AddBoundingBoxConstraint(kneeSet - eps, kneeSet + eps, xapex(positions_map.at("joint_5") ) );

    trajopt.AddBoundingBoxConstraint(upperSet - eps, upperSet + eps, xapex(positions_map.at("joint_6") ) );
    trajopt.AddBoundingBoxConstraint(kneeSet - eps, kneeSet + eps, xapex(positions_map.at("joint_7") ) );
  }



  /// Touch down constraint
  // trajopt.AddBoundingBoxConstraint(cos(pitch_magnitude_lo/2.0)*cos(orientation_diff/6), 1, xtd(positions_map.at("base_qw")));
  // trajopt.AddBoundingBoxConstraint(-sin(orientation_diff/6)*sin(pitch_magnitude_lo/2.0)-eps, sin(orientation_diff/6)*sin(pitch_magnitude_lo/2.0)+ eps, xtd(positions_map.at("base_qx")));
  // trajopt.AddBoundingBoxConstraint(-sin(pitch_magnitude_lo/2.0)*cos(orientation_diff/6)-eps, sin(pitch_magnitude_lo/2.0)*cos(orientation_diff/6)+eps, xtd(positions_map.at("base_qy")));
  // trajopt.AddBoundingBoxConstraint(-cos(pitch_magnitude_lo/2.0)*sin(orientation_diff/6)-eps, cos(pitch_magnitude_lo/2.0)*sin(orientation_diff/6)+eps, xtd(positions_map.at("base_qz")));
  auto ttd1=2.0 * (xtd(0) * xtd(3) - xtd(1)* xtd(2));
  auto ttd2=1 - 2* (xtd(2) * xtd(2) + xtd(3)* xtd(3));
  auto yawtd = atan2(ttd1, ttd2);
  trajopt.AddConstraint(yawtd,0,orientation_diff/2);
  // /// Flight 2 constraint
  
  // // Limit magnitude of pitch
  // trajopt.AddBoundingBoxConstraint(cos(pitch_magnitude_apex/2.0)*cos(orientation_diff/3), 1, xapex(positions_map.at("base_qw")));
  // trajopt.AddBoundingBoxConstraint(-sin(orientation_diff/3)*sin(pitch_magnitude_apex/2.0)-eps,sin(orientation_diff/3)*sin(pitch_magnitude_apex/2.0)+ eps, xapex(positions_map.at("base_qx")));
  // trajopt.AddBoundingBoxConstraint(-sin(pitch_magnitude_apex/2.0)*cos(orientation_diff/3)-eps,sin(pitch_magnitude_apex/2.0)*cos(orientation_diff/3)+eps, xapex(positions_map.at("base_qy")));
  // trajopt.AddBoundingBoxConstraint(-cos(pitch_magnitude_apex/2.0)*sin(orientation_diff/3)-eps, cos(pitch_magnitude_apex/2.0)*sin(orientation_diff/3)+eps, xapex(positions_map.at("base_qz")));
  auto tapex1=2.0 * (xapex(0) * xapex(3) - xapex(1)* xapex(2));
  auto tapex2=1 - 2* (xapex(2) * xapex(2) + xapex(3)* xapex(3));
  auto yawapex = atan2(tapex1, tapex2);
  trajopt.AddConstraint(yawapex,orientation_diff/5,orientation_diff*0.7);

  // Velocity
  trajopt.AddBoundingBoxConstraint(-eps, eps, xapex(plant.num_positions()+velocities_map.at("base_vz")));
  // Apex height
  if (apex_height>0) trajopt.AddBoundingBoxConstraint(apex_height-eps,apex_height+ eps, xapex(positions_map.at("base_z")));


  /// LO constraints
  // Limit magnitude of pitch
  // trajopt.AddBoundingBoxConstraint(cos(pitch_magnitude_lo/2.0)*cos(orientation_diff/2), 1, xlo(positions_map.at("base_qw")));
  // trajopt.AddBoundingBoxConstraint(-sin(orientation_diff/2)*sin(pitch_magnitude_lo/2.0)-eps,sin(orientation_diff/2)*sin(pitch_magnitude_lo/2.0)+ eps, xlo(positions_map.at("base_qx")));
  // trajopt.AddBoundingBoxConstraint(-sin(pitch_magnitude_lo/2.0)*cos(orientation_diff/2)-eps , sin(pitch_magnitude_lo/2.0)*cos(orientation_diff/2)+eps, xlo(positions_map.at("base_qy")));
  // trajopt.AddBoundingBoxConstraint(-cos(pitch_magnitude_lo/2.0)*sin(orientation_diff/2)-eps, cos(pitch_magnitude_lo/2.0)*sin(orientation_diff/2)+eps, xlo(positions_map.at("base_qz")));
  auto tlo1=2.0 * (xlo(0) * xlo(3) - xlo(1)* xlo(2));
  auto tlo2=1 - 2* (xlo(2) * xlo(2) + xlo(3)* xlo(3));
  auto yawlo = atan2(tlo1, tlo2);
  trajopt.AddConstraint(yawlo,orientation_diff*0.6,orientation_diff);



  /// Final constraints
  // Final orientation
  // Limit magnitude of yaw
  // TO DO: Compute yaw  yaw diff=0.1
  //        Compute roll, pitch
  // auto t0=2.0 * (x0(0) * x0(3) + x0(1)* x0(2));
  // auto t1=1 - 2* (x0(2) * x0(2) + x0(3)* x0(3));
  // auto yaw0 = atan2(t0, t1);
  // auto t2=2.0 * (xf(0) * xf(3) + xf(1)* xf(2));
  // auto t3=1 - 2* (xf(2) * xf(2) + xf(3)* xf(3));
  // auto yawf = atan2(t2, t3);
  // trajopt.AddConstraint(yawf-yaw0-orientation_diff,-eps,eps);

  
  // auto roll0 = atan2(2.0 * (x0(0) * x0(1) + x0(2) * x0(3)), 1.0 - 2.0 * (x0(1) * x0(1) + x0(2) * x0(2)));
  // auto rollf = atan2(2.0 * (xf(0) * xf(1) + xf(2) * xf(3)), 1.0 - 2.0 * (xf(1) * xf(1) + xf(2) * xf(2)));
  // trajopt.AddConstraint(rollf-roll0,-eps,eps);

  // auto pitch0 = asin(2.0 * (x0(0) * x0(2) - x0(3) * x0(1)));
  // auto pitchf = asin(2.0 * (xf(0) * xf(2) - xf(3) * xf(1)));
  // trajopt.AddConstraint(pitchf-pitch0,-eps,eps);


  auto t0=2.0 * (x0(0) * x0(3) - x0(1)* x0(2));
  auto t1=1 - 2* (x0(2) * x0(2) + x0(3)* x0(3));
  auto yaw0 = atan2(t0, t1);
  auto t2=2.0 * (xf(0) * xf(3) - xf(1)* xf(2));
  auto t3=1 - 2* (xf(2) * xf(2) + xf(3)* xf(3));
  auto yawf = atan2(t2, t3);
  trajopt.AddConstraint(yawf-yaw0-orientation_diff,-eps,eps);

  
  auto roll0 = atan2(2.0 * (x0(0) * x0(1) - x0(2) * x0(3)), 1.0 - 2.0 * (x0(1) * x0(1) + x0(2) * x0(2)));
  auto rollf = atan2(2.0 * (xf(0) * xf(1) - xf(2) * xf(3)), 1.0 - 2.0 * (xf(1) * xf(1) + xf(2) * xf(2)));
  trajopt.AddConstraint(rollf-roll0,-eps,eps);

  auto pitch0 = asin(2.0 * (x0(0) * x0(2) + x0(3) * x0(1)));
  auto pitchf = asin(2.0 * (xf(0) * xf(2) + xf(3) * xf(1)));
  trajopt.AddConstraint(pitchf-pitch0,-eps,eps);

  ///constraint initial roll yaw pitch
  trajopt.AddConstraint(yaw0,-eps,eps);
  trajopt.AddConstraint(roll0,-0.3,0.3);
  trajopt.AddConstraint(pitch0,-0.4,0.4);

  
  
  if (this->spine_type=="rigid") {
    for (int i=7;i<n_q;i++) {
      trajopt.AddConstraint( xf(i)-x0(i),-10*eps, 10*eps);
    } // Same joint positions
    trajopt.AddConstraint( xf(6)-x0(6),-eps, eps);// same z

    trajopt.AddConstraint( xf(n_q)-x0(n_q),-eps, eps); // same wx
    trajopt.AddConstraint( xf(n_q+1)-x0(n_q+1),-eps, eps); // same wy
    trajopt.AddConstraint( x0(n_q+2),-eps, eps); // zero initial wz
    trajopt.AddConstraint( xf(n_q+3)-x0(n_q+3),-eps, eps); // same vx
    trajopt.AddConstraint( x0(n_q+4),-eps, eps); // zero initial vy
    trajopt.AddConstraint( xf(n_q+4),-speed*sin(orientation_diff)*0.1-eps, speed*sin(orientation_diff)*0.1+eps); // small final vy
    trajopt.AddConstraint( xf(n_q+5)-x0(n_q+5),-eps, eps); // same zero vz
    for (int i=n_q+6;i<n_q+n_v;i++) {
      trajopt.AddConstraint( xf(i)-x0(i),-10*eps, 10*eps);
    }
  }
  else if (this->spine_type=="twisting") {
    for (int i=8;i<n_q;i++) {
      trajopt.AddConstraint( xf(i)-x0(i),-10*eps, 10*eps);
    } // Almost the same joint positions
    trajopt.AddConstraint( xf(6)-x0(6),-eps, eps);// same z

    trajopt.AddConstraint( xf(n_q)-x0(n_q),-eps, eps); // same wx
    trajopt.AddConstraint( xf(n_q+1)-x0(n_q+1),-eps, eps); // same wy
    trajopt.AddConstraint( x0(n_q+2),-eps, eps); // zero initial wz
    trajopt.AddConstraint( xf(n_q+3)-x0(n_q+3),-eps, eps); // same vx
    trajopt.AddConstraint( x0(n_q+4),-eps, eps); // zero initial vy
    trajopt.AddConstraint( xf(n_q+4),-speed*sin(orientation_diff)*0.1-eps, speed*sin(orientation_diff)*0.1+eps); // small final vy
    trajopt.AddConstraint( xf(n_q+5)-x0(n_q+5),-eps, eps); // same zero vz

    // Contraints on spine
    // trajopt.AddBoundingBoxConstraint(-0.2,0.2, x0(7));
    trajopt.AddConstraint( xf(7)-x0(7),-10*eps, 10*eps);
    // trajopt.AddBoundingBoxConstraint(-0.5,0.5, x0(n_q+7));
    trajopt.AddConstraint( xf(n_q+7)-x0(n_q+7),-30*eps, 30*eps);

    for (int i=n_q+7;i<n_q+n_v;i++) {
      trajopt.AddConstraint( xf(i)-x0(i),-10*eps, 10*eps); // Almost the same joint velocities
    }
  }

  // //Average Velocity
  // auto times  = trajopt.time_vars();
  // auto total_time=times(0)+times(1);
  // for (int i=2;i<trajopt.N()-1;i++) total_time+=times(i);
  // trajopt.AddConstraint(  this->speed * total_time+x0(positions_map.at("base_x")) -xf(positions_map.at("base_x")),-eps, eps );

  //Average Velocity
  auto times  = trajopt.time_vars();
  auto total_time=times(0)+times(1);
  int vx_index=n_q + velocities_map.at("base_vx");
  auto total_distance=(times(0)*trajopt.state(0)(vx_index)+times(1)*trajopt.state(1)(vx_index))*cos(pitch0);
  
  /// Constraints on all points
  for (int i = 0; i < trajopt.N(); i++){
    auto xi = trajopt.state(i);

    auto rolli = atan2(2.0 * (xi(0) * xi(1) - xi(2) * xi(3)), 1.0 - 2.0 * (xi(1) * xi(1) + xi(2) * xi(2)));
    trajopt.AddConstraint(rolli,-max_roll-eps, max_roll+eps);
    auto pitchi = asin(2.0 * (xi(0) * xi(2) + xi(3) * xi(1)));
    trajopt.AddConstraint(pitchi,-max_pitch-eps, max_pitch+eps);

    // x velocity almost constant
    trajopt.AddBoundingBoxConstraint(this->speed*0.9, this->speed*1.1, xi(n_q + velocities_map.at("base_vx")));
    // Height
    trajopt.AddBoundingBoxConstraint( 0.15, 2, xi( positions_map.at("base_z")));
    if (lock_spine && this->spine_type=="twisting") trajopt.AddBoundingBoxConstraint( -eps, eps, xi( positions_map.at("joint_12")));

    //Average velocity
    if (i>=2 && i<(trajopt.N()-1)) {
      total_time+=times(i);
      total_distance+=times(i)*trajopt.state(i)(vx_index)*cos(rolli);
    }
  }
  trajopt.AddConstraint(  this->speed * total_time-total_distance, -eps, eps );
  // Final state y  FOR TESTING PURPOSE
  trajopt.AddConstraint( xf( positions_map.at("base_y"))/(total_distance*sin(orientation_diff)),0.4,1.4);
}


template <class Y>
void SpiritTurn<Y>::setUpModeSequence(){
  this->mode_vector.clear();
  this->normal_vector.clear();
  this->offset_vector.clear();
  this->minT_vector.clear();
  this->maxT_vector.clear();
  //                          mode name   normal                  world offset            minT  maxT
  this->addModeToSequenceVector("flight",Eigen::Vector3d::UnitZ(),Eigen::Vector3d::Zero(),0.04, 0.5);
  this->addModeToSequenceVector("front_stance",Eigen::Vector3d::UnitZ(),Eigen::Vector3d::Zero(),0.02, 1);
  this->addModeToSequenceVector("flight",Eigen::Vector3d::UnitZ(),Eigen::Vector3d::Zero(),0.04, 1);
  this->addModeToSequenceVector("flight",Eigen::Vector3d::UnitZ(),Eigen::Vector3d::Zero(),0.04, 1);
  this->addModeToSequenceVector("rear_stance",Eigen::Vector3d::UnitZ(),Eigen::Vector3d::Zero(),0.02, 1);
  this->addModeToSequenceVector("flight",Eigen::Vector3d::UnitZ(),Eigen::Vector3d::Zero(),0.04, 0.1);
}

/// Runs a trajectory optimization problem for spirit bounding on flat ground
/// \param plant: robot model
/// \param pp_xtraj: state trajectory passed by pointer for spirit animation
/// \param surface_vector vector of surfaces in the scene (for animation)
template <class Y>
void SpiritTurn<Y>::run(MultibodyPlant<Y>& plant,
                          PiecewisePolynomial<Y>* pp_xtraj,
                          std::vector<SurfaceConf>* surface_vector) {
  // Setup mode sequence
  auto sequence = DirconModeSequence<Y>(plant);
  setUpModeSequence();
  auto [modeVector, toeEvals, toeEvalSets] = getModeSequence(plant, sequence);
  
  ///Setup trajectory optimization
  auto trajopt = Dircon<Y>(sequence);

  if (this->ipopt) {
    // Ipopt settings adapted from CaSaDi and FROST
    auto id = drake::solvers::IpoptSolver::id();
    trajopt.SetSolverOption(id, "tol", this->tol);
    trajopt.SetSolverOption(id, "dual_inf_tol", this->tol);
    trajopt.SetSolverOption(id, "constr_viol_tol", this->tol);
    trajopt.SetSolverOption(id, "compl_inf_tol", this->tol);
    trajopt.SetSolverOption(id, "max_iter", 800);
    trajopt.SetSolverOption(id, "nlp_lower_bound_inf", -1e6);
    trajopt.SetSolverOption(id, "nlp_upper_bound_inf", 1e6);
    trajopt.SetSolverOption(id, "print_timing_statistics", "yes");
    trajopt.SetSolverOption(id, "print_level", 5);

    ///// WARM START /////////////////////////////////
    trajopt.SetSolverOption(id, "bound_push", 1e-8);
    trajopt.SetSolverOption(id, "warm_start_bound_push", 1e-8);
    trajopt.SetSolverOption(id, "warm_start_init_point", "yes");
    // Set to ignore overall tolerance/dual infeasibility, but terminate when
    // primal feasible and objective fails to increase over 5 iterations.
    trajopt.SetSolverOption(id, "acceptable_compl_inf_tol", this->tol);
    trajopt.SetSolverOption(id, "acceptable_constr_viol_tol", this->tol);
    trajopt.SetSolverOption(id, "acceptable_obj_change_tol", this->tol);
    trajopt.SetSolverOption(id, "acceptable_tol", this->tol * 10);
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
                            this->tol);  // target optimality
    trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(), "Major feasibility tolerance", this->tol);
    trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(), "Verify level",
                            0);  // 0
  }

  // Setting up cost
  auto work_binding = addCost(plant, trajopt);


  // Initialize the trajectory control state and forces
  if (this->file_name_in.empty()){
    //trajopt.SetInitialGuessForAllVariables(
    //    VectorXd::Zero(trajopt.decision_variables().size()));
    for (int j = 0; j < sequence.num_modes(); j++) {
      trajopt.SetInitialTrajectoryForMode(j, this->x_traj[j], this->u_traj, this->x_traj[j].start_time(), this->x_traj[j].end_time());
      trajopt.SetInitialForceTrajectory(j, this->l_traj[j], this->lc_traj[j],
                                        this->vc_traj[j], this->l_traj[j].start_time());
    }
  }else{
    // std::cout<<"Loading decision var from file, will fail if num dec vars changed" <<std::endl;
    dairlib::DirconTrajectory loaded_traj(this->file_name_in);
    // std::cout<<loaded_traj.GetStateDerivativeSamples(0)<<std::endl;
    // std::cout<<loaded_traj.GetDecisionVariables()<<std::endl;
    states_traj=loaded_traj.ReconstructStateTrajectory();
    trajopt.SetInitialGuessForAllVariables(loaded_traj.GetDecisionVariables());
  }

  
  addConstraints(plant, trajopt);

  /// Setup the visualization during the optimization
  int num_ghosts = 1;// Number of ghosts in visualization. NOTE: there are limitations on number of ghosts based on modes and knotpoints
  std::vector<unsigned int> visualizer_poses; // Ghosts for visualizing during optimization
  for(int i = 0; i < sequence.num_modes(); i++){
      visualizer_poses.push_back(num_ghosts); 
  }
  trajopt.CreateVisualizationCallback(
      dairlib::FindResourceOrThrow(this->urdf_path),
      visualizer_poses, 0.2); // setup which URDF, how many poses, and alpha transparency 

  drake::solvers::SolverId solver_id("");
  if (this->ipopt) {
    solver_id = drake::solvers::IpoptSolver().id();
    std::cout << "\nChose manually: " << solver_id.name() << std::endl;
  } else {
    solver_id = drake::solvers::ChooseBestSolver(trajopt);
    std::cout << "\nChose the best solver: " << solver_id.name() << std::endl;
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
  this->saveTrajectory(plant,trajopt,result);

  // std::cout<<result.get_x_val()<<std::endl;
  // Writing contact force data
  std::string contact_force_fname="/home/feng/Downloads/dairlib/examples/Spirit_spine/data/turn/rigid/turn_"+std::to_string(this->speed)+"_"+
                std::to_string(this->orientation_diff)+".csv";
  this->saveContactForceData(this->orientation_diff,contact_force_fname,result.is_success() && result.get_optimal_cost()<10000);
  
  // auto x_trajs = trajopt.ReconstructDiscontinuousStateTrajectory(result);
  // std::cout<<"Work = " << dairlib::calcElectricalWork(plant, x_trajs, this->u_traj) << std::endl;

  // if(this->cost_power > 0){
  //   double cost_work_val = solvers::EvalCostGivenSolution(
  //       result, work_binding);
  //   std::cout<<"ReLu Work = " << cost_work_val/this->cost_power << std::endl;
  // }
  /// Run animation of the final trajectory
  *pp_xtraj =trajopt.ReconstructStateTrajectory(result);
  // // Create offset polynomial
  // std::vector<double> breaks=pp_xtraj->get_breaks();
  // std::vector<Eigen::MatrixXd> samples(breaks.size());
  // for (int i = 0; i < static_cast<int>(breaks.size()); ++i) {
  //   samples[i].resize(39, 1);
  //   for (int j=0;j<39;j++) samples[i](j, 0) = 0;
  //   samples[i](4, 0) = pp_xtraj->value(pp_xtraj->end_time())(4,0); // x offset
  //   // samples[i](5, 0) = pp_xtraj->value(pp_xtraj->end_time())(5,0); // y offset
  //   // for (int j=0;j<4;j++)  samples[i](j, 0) = pp_xtraj->value(pp_xtraj->end_time())(j,0);
  // }
  // PiecewisePolynomial<double> ini_offset_pp = PiecewisePolynomial<double>::FirstOrderHold(breaks, samples);
  // PiecewisePolynomial<double> offset_pp=ini_offset_pp;

  // for (int i=0;i<10;i++){
  //   PiecewisePolynomial<Y> x_traj_i=trajopt.ReconstructStateTrajectory(result)+offset_pp;
  //   offset_pp+=ini_offset_pp;
  //   x_traj_i.shiftRight(pp_xtraj->end_time());
  //   pp_xtraj->ConcatenateInTime(x_traj_i);
  // }
  
  // }
}

template class SpiritTurn<double>;
}  // namespace dairlib

