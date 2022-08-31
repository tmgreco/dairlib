#include "examples/Spirit_spine/spirit_bounding_gait.h"
#include <yaml-cpp/yaml.h>


DEFINE_bool(ghosts, true, "If ghosts are shown"); 

using drake::multibody::MultibodyPlant;
using drake::trajectories::PiecewisePolynomial;
using Eigen::VectorXd;
using Eigen::MatrixXd;

namespace dairlib {
using systems::trajectory_optimization::DirconModeSequence;
using systems::trajectory_optimization::Dircon;

template <class Y> 
SpiritBoundingGait<Y>::SpiritBoundingGait(){
}

/// Assigns values to member variables according to input yaml file
/// \param yaml_path path of the yaml file
/// \param saved_directory directory for saving the output trajectories (namely for setting file_name_out)
/// \param index indicates that we are using the index^th configuration
/// \param plant: robot model
template <class Y>
void SpiritBoundingGait<Y>::config(std::string yaml_path, std::string saved_directory, int index,MultibodyPlant<Y>* plant){
  YAML::Node config = YAML::LoadFile(yaml_path);
  this->index=index;
  this->ipopt=config[index]["ipopt"].as<bool>();
  this->num_knot_points=config[index]["num_knot_points"].as<std::vector<int>>();
  this->initial_height=config[index]["initial_height"].as<double>();
  this->pitch_magnitude=config[index]["pitch_magnitude"].as<double>();
  this->pitch_magnitude_apex=config[index]["pitch_magnitude_apex"].as<double>();
  this->lock_spine=config[index]["lock_spine"].as<bool>();
  this->time_symmetry=config[index]["time_symmetry"].as<bool>();
  this->force_symmetry=config[index]["force_symmetry"].as<bool>();
  this->apex_height=config[index]["apex_height"].as<double>();
  this->speed=config[index]["speed"].as<double>();
  this->max_duration=config[index]["max_duration"].as<double>();
  this->min_duration=config[index]["min_duration"].as<double>();
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
  if (config[index]["warm_up"]) this->warm_up=config[index]["warm_up"].as<bool>();
  else this->warm_up=false;

  if(!config[index]["file_name_out"].as<std::string>().empty()) this->file_name_out=saved_directory+config[index]["file_name_out"].as<std::string>();
  if(!config[index]["file_name_in"].as<std::string>().empty()) this->file_name_in= saved_directory+config[index]["file_name_in"].as<std::string>();
  if(config[index]["action"]) this->action=config[index]["action"].as<std::string>();
  else this->action="";
}


/// Generate a bad initial guess for the spirit bound traj opt
/// \param plant: robot model
template <class Y>
void SpiritBoundingGait<Y>::generateInitialGuess(MultibodyPlant<Y>& plant){
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
void SpiritBoundingGait<Y>::addCostLegs(MultibodyPlant<Y>& plant,
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
std::vector<drake::solvers::Binding<drake::solvers::Cost>> SpiritBoundingGait<Y>::addCost(
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
void SpiritBoundingGait<Y>::addConstraints(
                    MultibodyPlant<Y>& plant,
                    dairlib::systems::trajectory_optimization::Dircon<Y>& trajopt){
// Get position and velocity dictionaries
  auto positions_map = multibody::makeNameToPositionsMap(plant);
  auto velocities_map = multibody::makeNameToVelocitiesMap(plant);


  // General constraints
  setSpiritJointLimits(plant, trajopt);
  setSpiritActuationLimits(plant, trajopt);
  trajopt.AddDurationBounds(min_duration, max_duration);

  /// Setup all the optimization constraints
  int n_q = plant.num_positions();
  int n_v = plant.num_velocities();
  auto x0  = trajopt.initial_state();
  auto xtd = trajopt.state_vars(1,0);
  auto xapex  = trajopt.state_vars(3,0) ;
  auto xlo  =  trajopt.state_vars(4,0);
  auto xf  = trajopt.final_state();


  if (time_symmetry) trajopt.AddPeriodicTimeIntervalsConstraints();
  if (force_symmetry) setSpiritSymmetry(plant, trajopt);

  /// Initial constraint

  // xyz position
  trajopt.AddBoundingBoxConstraint(0, 0, x0(positions_map.at("base_x"))); 
  trajopt.AddBoundingBoxConstraint( -eps, eps, x0( positions_map.at("base_y")));
  if (apex_height>0) trajopt.AddBoundingBoxConstraint( this->apex_height-eps, this->apex_height+eps, x0( positions_map.at("base_z")));

  // velocities
  // trajopt.AddBoundingBoxConstraint(this->speed-eps, this->speed+eps, x0(plant.num_positions()+velocities_map.at("base_vx")));
  trajopt.AddBoundingBoxConstraint(-eps, eps, x0(plant.num_positions()+velocities_map.at("base_vy")));
  trajopt.AddBoundingBoxConstraint(-eps, eps, x0(plant.num_positions()+velocities_map.at("base_vz")));

  // Limit magnitude of pitch
  trajopt.AddBoundingBoxConstraint(cos(this->pitch_magnitude_apex/2.0), 1, x0(positions_map.at("base_qw")));
  trajopt.AddBoundingBoxConstraint(-eps, eps, x0(positions_map.at("base_qx")));
  trajopt.AddBoundingBoxConstraint(-sin(this->pitch_magnitude_apex/2.0) , sin(this->pitch_magnitude_apex/2.0), x0(positions_map.at("base_qy")));
  trajopt.AddBoundingBoxConstraint(-eps, eps, x0(positions_map.at("base_qz")));


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

  /// Flight 2 constraint
  
  // Limit magnitude of pitch
  trajopt.AddBoundingBoxConstraint(cos(pitch_magnitude_apex/2.0), 1, xapex(positions_map.at("base_qw")));
  trajopt.AddBoundingBoxConstraint(-eps, eps, xapex(positions_map.at("base_qx")));
  trajopt.AddBoundingBoxConstraint(-sin(pitch_magnitude_apex/2.0) , sin(pitch_magnitude_apex/2.0), xapex(positions_map.at("base_qy")));
  trajopt.AddBoundingBoxConstraint(-eps, eps, xapex(positions_map.at("base_qz")));
  // Velocity
  trajopt.AddBoundingBoxConstraint(-eps, eps, xapex(plant.num_positions()+velocities_map.at("base_vz")));
  // // Apex height
  // if (apex_height>0) trajopt.AddBoundingBoxConstraint(apex_height-eps,apex_height+ eps, xapex(positions_map.at("base_z")));



  /// Final constraints
  for (int i=0;i<n_q+n_v;i++) {
    if (i!=4) trajopt.AddConstraint( xf(i)-x0(i),-eps, eps);
  }

  // //Average Velocity   NOT WORKING BECAUSE OF PITCH
  // auto times  = trajopt.time_vars();
  // auto total_time=times(0)+times(1);
  // auto total_distance=times(0)*trajopt.state(0)(23)+times(1)*trajopt.state(1)(23);
  // for (int i=2;i<trajopt.N()-1;i++) {
  //   total_time+=times(i);
  //   total_distance+=times(i)*trajopt.state(i)(23);
  // }
  // trajopt.AddConstraint(  this->speed * total_time-total_distance, -eps, eps );

  //Average Velocity
  auto times  = trajopt.time_vars();
  auto total_time=times(0)+times(1);
  for (int i=2;i<trajopt.N()-1;i++) total_time+=times(i);
  trajopt.AddConstraint(  this->speed * total_time+x0(positions_map.at("base_x")) -xf(positions_map.at("base_x")),-eps, eps );

  double a_knee_max=1000;
  double bodyLength=0.33;
  double upperLegLength=0.206;
  double lowerLegLength=0.206;
  double bodyWidth=0.24;
  double abOffs=0.10098;
  double minToeHeight=0.03;
  double minElbowHeight=0.05;

  pitch_magnitude_new=pitch_magnitude;
  if (this->var!=0) {
    double suggested_magnitude=0.3+0.25*sqrt(this->speed);
    // auto normal_dist = std::bind(std::normal_distribution<double>{1, this->var*10},
    //                             std::mt19937(std::random_device{}()));
    // pitch_magnitude=suggested_magnitude*normal_dist();
    pitch_magnitude_new=suggested_magnitude*(((double)rand()) / ((double)RAND_MAX) +0.5);
    
  }
  std::cout<<"SPINE TYPE: "<<this->spine_type<<" MAX PITCH MAGNITUDE: "<<pitch_magnitude_new<<std::endl;
  
  /// Constraints on all points
  for (int i = 0; i < trajopt.N(); i++){
    auto xi = trajopt.state(i);
    // Height
    trajopt.AddBoundingBoxConstraint( 0.25, 2, xi( positions_map.at("base_z")));
    trajopt.AddBoundingBoxConstraint( -eps, eps, xi( n_q+velocities_map.at("base_vy")));
    if (lock_spine && this->spine_type=="twisting") trajopt.AddBoundingBoxConstraint( -eps, eps, xi( positions_map.at("joint_12")));
    // Limit vx at all knot points
    trajopt.AddBoundingBoxConstraint( 0.6*this->speed, 1.4*this->speed, xi( n_q+velocities_map.at("base_vx")));
    // Limit magnitude of pitch
    trajopt.AddBoundingBoxConstraint(cos(pitch_magnitude_new/2.0), 1, xi(positions_map.at("base_qw")));
    trajopt.AddBoundingBoxConstraint(-eps, eps, xi(positions_map.at("base_qx")));
    trajopt.AddBoundingBoxConstraint(-sin(pitch_magnitude_new/2.0) , sin(pitch_magnitude_new/2.0), xi(positions_map.at("base_qy")));
    trajopt.AddBoundingBoxConstraint(-eps, eps, xi(positions_map.at("base_qz")));

    // Limit knee joints' angular accelerations
    if(i>0){
        auto xim1=trajopt.state(i-1);
        trajopt.AddConstraint((xi(n_q+velocities_map.at("joint_1dot"))-xim1(n_q+velocities_map.at("joint_1dot")))/times(i-1),-a_knee_max,a_knee_max);
        trajopt.AddConstraint((xi(n_q+velocities_map.at("joint_3dot"))-xim1(n_q+velocities_map.at("joint_3dot")))/times(i-1),-a_knee_max,a_knee_max);
        trajopt.AddConstraint((xi(n_q+velocities_map.at("joint_5dot"))-xim1(n_q+velocities_map.at("joint_5dot")))/times(i-1),-a_knee_max,a_knee_max);
        trajopt.AddConstraint((xi(n_q+velocities_map.at("joint_7dot"))-xim1(n_q+velocities_map.at("joint_7dot")))/times(i-1),-a_knee_max,a_knee_max);
    }

    // Torso height

    auto pitch = asin(2.0 * min(max((xi(0) * xi(2) + xi(3) * xi(1)),-0.5),0.5));   // clip the value in asin(*) to be with in (-1,1) 
                                                                                   // Up -> negative    Down -> positive
    trajopt.AddConstraint(  xi( positions_map.at("base_z"))+0.5*(bodyLength+0.1)*sin(pitch),0.15, 2);
    trajopt.AddConstraint(  xi( positions_map.at("base_z"))-0.5*(bodyLength+0.1)*sin(pitch),0.15, 2);


    // Elbow 1
    if (this->spine_type=="twisting")
      trajopt.AddConstraint(xi(positions_map.at("base_z"))+0.5*bodyWidth*sin(xi(positions_map.at("joint_12")))-0.5*bodyLength*sin(pitch)+abOffs*sin(xi(positions_map.at("joint_8")))
                            -cos(xi(positions_map.at("joint_8"))+xi(positions_map.at("joint_12")))*(upperLegLength*sin(xi(positions_map.at("joint_0"))-pitch)
                            ),minElbowHeight, 1);
    else 
      trajopt.AddConstraint(xi(positions_map.at("base_z"))-0.5*bodyLength*sin(pitch)+abOffs*sin(xi(positions_map.at("joint_8")))
                            -cos(xi(positions_map.at("joint_8")))*(upperLegLength*sin(xi(positions_map.at("joint_0"))-pitch)
                            ),minElbowHeight, 1);
    // Elbow 3
    if (this->spine_type=="twisting")
      trajopt.AddConstraint(xi(positions_map.at("base_z"))-0.5*bodyWidth*sin(xi(positions_map.at("joint_12")))-0.5*bodyLength*sin(pitch)-abOffs*sin(xi(positions_map.at("joint_10")))
                            -cos(xi(positions_map.at("joint_10"))+xi(positions_map.at("joint_12")))*(upperLegLength*sin(xi(positions_map.at("joint_4"))-pitch)
                            ),minElbowHeight, 1);
    else 
      trajopt.AddConstraint(xi(positions_map.at("base_z"))-0.5*bodyLength*sin(pitch)-abOffs*sin(xi(positions_map.at("joint_10")))
                            -cos(xi(positions_map.at("joint_10")))*(upperLegLength*sin(xi(positions_map.at("joint_4"))-pitch)
                            ),minElbowHeight, 1);

    // Elbow 2
    trajopt.AddConstraint(xi(positions_map.at("base_z"))+abOffs*sin(xi(positions_map.at("joint_9")))+0.5*bodyLength*sin(pitch)-
                          cos(xi(positions_map.at("joint_9")))*upperLegLength*sin(xi(positions_map.at("joint_2"))+pitch),minElbowHeight, 1);
    
    // Elbow 4
    trajopt.AddConstraint(xi(positions_map.at("base_z"))+0.5*bodyLength*sin(pitch)-abOffs*sin(xi(positions_map.at("joint_11")))-
                        cos(xi(positions_map.at("joint_11")))*upperLegLength*sin(xi(positions_map.at("joint_6"))+pitch),minElbowHeight, 1);

    // Toe 1
    if (this->spine_type=="twisting")
      trajopt.AddConstraint(xi(positions_map.at("base_z"))+0.5*bodyWidth*sin(xi(positions_map.at("joint_12")))-0.5*bodyLength*sin(pitch)+abOffs*sin(xi(positions_map.at("joint_8")))
                            -cos(xi(positions_map.at("joint_8"))+xi(positions_map.at("joint_12")))*(upperLegLength*sin(xi(positions_map.at("joint_0"))-pitch)
                            +lowerLegLength*sin(xi(positions_map.at("joint_1"))-xi(positions_map.at("joint_0"))+pitch)),0, 1.5);
    else 
      trajopt.AddConstraint(xi(positions_map.at("base_z"))-0.5*bodyLength*sin(pitch)+abOffs*sin(xi(positions_map.at("joint_8")))
                            -cos(xi(positions_map.at("joint_8")))*(upperLegLength*sin(xi(positions_map.at("joint_0"))-pitch)
                            +lowerLegLength*sin(xi(positions_map.at("joint_1"))-xi(positions_map.at("joint_0"))+pitch)),0, 1.5);
    // Toe 2
    trajopt.AddConstraint(xi(positions_map.at("base_z"))+abOffs*sin(xi(positions_map.at("joint_9")))+0.5*bodyLength*sin(pitch)-cos(xi(positions_map.at("joint_9")))*
                          (upperLegLength*sin(xi(positions_map.at("joint_2"))+pitch)+lowerLegLength*sin(xi(positions_map.at("joint_3"))
                          -xi(positions_map.at("joint_2"))-pitch)),0, 1.5);
    // Toe 3
    if (this->spine_type=="twisting")
      trajopt.AddConstraint(xi(positions_map.at("base_z"))-0.5*bodyWidth*sin(xi(positions_map.at("joint_12")))-0.5*bodyLength*sin(pitch)-abOffs*sin(xi(positions_map.at("joint_10")))
                            -cos(xi(positions_map.at("joint_10"))+xi(positions_map.at("joint_12")))*(upperLegLength*sin(xi(positions_map.at("joint_4"))-pitch)
                            +lowerLegLength*sin(xi(positions_map.at("joint_5"))-xi(positions_map.at("joint_4"))+pitch)),0, 1.5);
    else 
      trajopt.AddConstraint(xi(positions_map.at("base_z"))-0.5*bodyLength*sin(pitch)-abOffs*sin(xi(positions_map.at("joint_10")))
                            -cos(xi(positions_map.at("joint_10")))*(upperLegLength*sin(xi(positions_map.at("joint_4"))-pitch)
                            +lowerLegLength*sin(xi(positions_map.at("joint_5"))-xi(positions_map.at("joint_4"))+pitch)),0, 1.5);
    // Toe 4
    trajopt.AddConstraint(xi(positions_map.at("base_z"))+0.5*bodyLength*sin(pitch)-abOffs*sin(xi(positions_map.at("joint_11")))-cos(xi(positions_map.at("joint_11")))*
                          (upperLegLength*sin(xi(positions_map.at("joint_6"))+pitch)+lowerLegLength*sin(xi(positions_map.at("joint_7"))
                          -xi(positions_map.at("joint_6"))-pitch)),0, 1.5);
  }

}


template <class Y>
void SpiritBoundingGait<Y>::setUpModeSequence(){
  this->mode_vector.clear();
  this->normal_vector.clear();
  this->offset_vector.clear();
  this->minT_vector.clear();
  this->maxT_vector.clear();
  //                          mode name   normal                  world offset            minT  maxT
  this->addModeToSequenceVector("flight",Eigen::Vector3d::UnitZ(),Eigen::Vector3d::Zero(),0.02, 0.5);
  this->addModeToSequenceVector("front_stance",Eigen::Vector3d::UnitZ(),Eigen::Vector3d::Zero(),0.02, 1);
  this->addModeToSequenceVector("flight",Eigen::Vector3d::UnitZ(),Eigen::Vector3d::Zero(),0.02, 1);
  this->addModeToSequenceVector("flight",Eigen::Vector3d::UnitZ(),Eigen::Vector3d::Zero(),0.02, 1);
  this->addModeToSequenceVector("rear_stance",Eigen::Vector3d::UnitZ(),Eigen::Vector3d::Zero(),0.02, 1);
  this->addModeToSequenceVector("flight",Eigen::Vector3d::UnitZ(),Eigen::Vector3d::Zero(),0.02, 0.1);
}

/// Runs a trajectory optimization problem for spirit bounding on flat ground
/// \param plant: robot model
/// \param pp_xtraj: state trajectory passed by pointer for spirit animation
/// \param surface_vector vector of surfaces in the scene (for animation)
template <class Y>
void SpiritBoundingGait<Y>::run(MultibodyPlant<Y>& plant,
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
    drake::solvers::SolverId solver_id = drake::solvers::IpoptSolver().id();
    trajopt.SetSolverOption(solver_id, "tol", this->tol);
    trajopt.SetSolverOption(solver_id, "dual_inf_tol", this->tol*100);
    trajopt.SetSolverOption(solver_id, "constr_viol_tol", this->tol);
    trajopt.SetSolverOption(solver_id, "compl_inf_tol", this->tol);
    trajopt.SetSolverOption(solver_id, "max_iter", 1500);
    trajopt.SetSolverOption(solver_id, "nlp_lower_bound_inf", -1e6);
    trajopt.SetSolverOption(solver_id, "nlp_upper_bound_inf", 1e6);
    trajopt.SetSolverOption(solver_id, "print_timing_statistics", "yes");
    trajopt.SetSolverOption(solver_id, "print_level", 5);

    if (warm_up){
      std::cout<<"Warm start"<<std::endl;
      // trajopt.SetSolverOption(solver_id, "nlp_scaling_method", "none"); 
      
      // trajopt.SetSolverOption(solver_id, "mu_init", 10); 
      trajopt.SetSolverOption(solver_id, "warm_start_bound_frac", 1e-12);
      trajopt.SetSolverOption(solver_id, "warm_start_bound_push", 1e-12);
      trajopt.SetSolverOption(solver_id, "warm_start_mult_bound_push", 1e-12);
      trajopt.SetSolverOption(solver_id, "warm_start_slack_bound_frac", 1e-12);
      trajopt.SetSolverOption(solver_id, "warm_start_slack_bound_push", 1e-12);
      trajopt.SetSolverOption(solver_id, "warm_start_init_point", "yes");
    }
    trajopt.SetSolverOption(solver_id, "acceptable_compl_inf_tol", this->tol);
    trajopt.SetSolverOption(solver_id, "acceptable_constr_viol_tol", this->tol);
    trajopt.SetSolverOption(solver_id, "acceptable_obj_change_tol", this->tol);
    trajopt.SetSolverOption(solver_id, "acceptable_tol", this->tol * 10);
    trajopt.SetSolverOption(solver_id, "acceptable_iter", 5);
    std::cout << "\nChose manually: " << solver_id.name() << std::endl;
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
    std::cout<<"Loading decision var from file, will fail if num dec vars changed" <<std::endl;
    dairlib::DirconTrajectory loaded_traj(this->file_name_in);
    trajopt.SetInitialGuessForAllVariables(loaded_traj.GetDecisionVariables());
    this->addGaussionNoiseToInputTraj(plant,trajopt,loaded_traj);
    this->addGaussionNoiseToVelocitiesTraj(plant,trajopt,loaded_traj);
    this->addGaussionNoiseToStateTraj(plant,trajopt,loaded_traj);
  }

  addConstraints(plant, trajopt);

  /// Setup the visualization during the optimization
  if (FLAGS_ghosts){
    int num_ghosts = 1;// Number of ghosts in visualization. NOTE: there are limitations on number of ghosts based on modes and knotpoints
    std::vector<unsigned int> visualizer_poses; // Ghosts for visualizing during optimization
    for(int i = 0; i < sequence.num_modes(); i++){
        visualizer_poses.push_back(num_ghosts); 
    }
    trajopt.CreateVisualizationCallback(
        dairlib::FindResourceOrThrow(this->urdf_path),
        visualizer_poses, 0.2); // setup which URDF, how many poses, and alpha transparency 
  }

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
  // Writing contact force data
  int beginIdx = this->file_name_out.rfind('/');
  std::string filename = this->file_name_out.substr(beginIdx + 1);
  std::string contact_force_fname=this->data_directory+filename+".csv";
  this->saveContactForceData(trajopt,result,this->speed,pitch_magnitude_new,contact_force_fname,result.is_success());
  
  // auto x_trajs = trajopt.ReconstructDiscontinuousStateTrajectory(result);
  // std::cout<<"Work = " << dairlib::calcElectricalWork(plant, x_trajs, this->u_traj) << std::endl;

  // if(this->cost_power > 0){
  //   double cost_work_val = solvers::EvalCostGivenSolution(
  //       result, work_binding);
  //   std::cout<<"ReLu Work = " << cost_work_val/this->cost_power << std::endl;
  // }

  /// Run animation of the final trajectory
  *pp_xtraj =trajopt.ReconstructStateTrajectory(result);
  /// Create offset polynomial
  std::vector<double> breaks=pp_xtraj->get_breaks();
  std::vector<Eigen::MatrixXd> samples(breaks.size());
  for (int i = 0; i < static_cast<int>(breaks.size()); ++i) {
    samples[i].resize(39, 1);
    for (int j=0;j<39;j++) samples[i](j, 0) = 0;
    samples[i](4, 0) = pp_xtraj->value(pp_xtraj->end_time())(4,0);
  }
  PiecewisePolynomial<double> ini_offset_pp = PiecewisePolynomial<double>::FirstOrderHold(breaks, samples);
  PiecewisePolynomial<double> offset_pp=ini_offset_pp;

  for (int i=0;i<10;i++){
    PiecewisePolynomial<Y> x_traj_i=trajopt.ReconstructStateTrajectory(result)+offset_pp;
    offset_pp+=ini_offset_pp;
    x_traj_i.shiftRight(pp_xtraj->end_time());
    pp_xtraj->ConcatenateInTime(x_traj_i);
  }
  
  // }
}

template class SpiritBoundingGait<double>;
}  // namespace dairlib

