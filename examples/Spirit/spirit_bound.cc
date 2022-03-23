#include "examples/Spirit/spirit_bound.h"
#include <yaml-cpp/yaml.h>


using drake::multibody::MultibodyPlant;
using drake::trajectories::PiecewisePolynomial;
using Eigen::VectorXd;
using Eigen::MatrixXd;

namespace dairlib {
using systems::trajectory_optimization::DirconModeSequence;
using systems::trajectory_optimization::Dircon;

template <class Y> 
SpiritBound<Y>::SpiritBound(){
}

/// Assigns values to member variables according to input yaml file
/// \param yaml_path path of the yaml file
/// \param saved_directory directory for saving the output trajectories (namely for setting file_name_out)
/// \param index indicates that we are using the index^th configuration
/// \param plant: robot model
template <class Y>
void SpiritBound<Y>::config(std::string yaml_path, std::string saved_directory, int index,MultibodyPlant<Y>* plant){
  YAML::Node config = YAML::LoadFile(yaml_path);

  this->ipopt=config[index]["ipopt"].as<bool>();
  this->num_knot_points=config[index]["num_knot_points"].as<std::vector<int>>();
  this->initial_height=config[index]["initial_height"].as<double>();
  this->pitch_magnitude_lo=config[index]["pitch_magnitude_lo"].as<double>();
  this->pitch_magnitude_apex=config[index]["pitch_magnitude_apex"].as<double>();
  this->apex_height=config[index]["apex_height"].as<double>();
  this->fore_aft_displacement=config[index]["fore_aft_displacement"].as<double>();
  this->max_duration=config[index]["max_duration"].as<double>();
  this->cost_actuation=config[index]["cost_actuation"].as<double>();
  this->cost_velocity=config[index]["cost_velocity"].as<double>();
  this->cost_velocity_legs_flight=config[index]["cost_velocity_legs_flight"].as<double>();
  this->cost_actuation_legs_flight=config[index]["cost_actuation_legs_flight"].as<double>();
  this->cost_work=config[index]["cost_work"].as<double>();
  this->mu=config[index]["mu"].as<double>();
  this->eps=config[index]["eps"].as<double>();
  this->tol=config[index]["tol"].as<double>();

  if(!config[index]["file_name_out"].as<std::string>().empty()) this->file_name_out=saved_directory+config[index]["file_name_out"].as<std::string>();
  if(!config[index]["file_name_in"].as<std::string>().empty()) this->file_name_in= saved_directory+config[index]["file_name_in"].as<std::string>();

}


/// Generate a bad initial guess for the spirit bound traj opt
/// \param plant: robot model
template <class Y>
void SpiritBound<Y>::generateInitialGuess(MultibodyPlant<Y>& plant){
  double stand_height = this->initial_height;
  double pitch_lo = -0.3;
  double apex_height = this->apex_height;
  double duration = 0.3;
std::vector<MatrixXd> x_points;
  VectorXd x_const;
  //Stance
  dairlib::ikSpiritStand(plant, x_const, {true, true, true, true}, stand_height, 0.2, 0, 0);
  x_points.push_back(x_const);
  dairlib::ikSpiritStand(plant, x_const, {true, true, true, true}, stand_height+0.02, 0.2, 0, pitch_lo/2.0);
  x_points.push_back(x_const);

  this->x_traj.push_back(PiecewisePolynomial<double>::FirstOrderHold({0,duration/3.0},x_points));
  x_points.clear();

  //Rear stance
  dairlib::ikSpiritStand(plant, x_const, {true, true, true, true}, stand_height+0.02, 0.2, 0, pitch_lo/2.0);
  x_points.push_back(x_const);
  dairlib::ikSpiritStand(plant, x_const, {false, true, false, true}, stand_height+0.05, 0.2, 0, pitch_lo);
  x_points.push_back(x_const);

  this->x_traj.push_back(PiecewisePolynomial<double>::FirstOrderHold({duration/3.0, 2 * duration/3.0},x_points));
  x_points.clear();

  //Flight 1
  dairlib::ikSpiritStand(plant, x_const, {false, true, false, true}, stand_height+0.05, 0.2, 0, pitch_lo);
  x_points.push_back(x_const);
  dairlib::ikSpiritStand(plant, x_const, {false, false, false, false}, apex_height, 0.2, 0, 0);
  x_points.push_back(x_const);

  this->x_traj.push_back(PiecewisePolynomial<double>::FirstOrderHold({2 * duration/3.0, 3 * duration/3.0},x_points));
  x_points.clear();

  //Flight 2
  dairlib::ikSpiritStand(plant, x_const, {false, false, false, false}, apex_height, 0.2, 0, 0);
  x_points.push_back(x_const);
  dairlib::ikSpiritStand(plant, x_const, {true, false, true, false}, stand_height+0.05, 0.2, 0, -pitch_lo);
  x_points.push_back(x_const);

  this->x_traj.push_back(PiecewisePolynomial<double>::FirstOrderHold({3 * duration/3.0, 4 * duration/3.0},x_points));
  x_points.clear();

  //Front stance
  dairlib::ikSpiritStand(plant, x_const, {true, false, true, false}, stand_height+0.05, 0.2, 0, -pitch_lo);
  x_points.push_back(x_const);
  dairlib::ikSpiritStand(plant, x_const, {true, true, true, true}, stand_height+0.02, 0.2, 0, -pitch_lo/2.0);
  x_points.push_back(x_const);

  this->x_traj.push_back(PiecewisePolynomial<double>::FirstOrderHold({4 * duration/3.0, 5 * duration/3.0},x_points));
  x_points.clear();

  //stance
  dairlib::ikSpiritStand(plant, x_const, {true, true, true, true}, stand_height+0.02, 0.2, 0, -pitch_lo/2.0);
  x_points.push_back(x_const);
  dairlib::ikSpiritStand(plant, x_const, {true, true, true, true}, stand_height, 0.2, 0, 0);
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

  this->l_traj.push_back(init_l_traj_j);
  this->lc_traj.push_back(init_lc_traj_j);
  this->vc_traj.push_back(init_vc_traj_j);

  // back stance
  init_l_j.clear();
  init_lc_j.clear();
  init_vc_j.clear();
  init_time_j.clear();
  for (int i = 0; i < N; i++) {
    init_time_j.push_back(i * duration /3.0/ (N - 1) + 4 * duration/3.0);
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

  // stance
  init_l_j.clear();
  init_lc_j.clear();
  init_vc_j.clear();
  init_time_j.clear();
  for (int i = 0; i < N; i++) {
    init_time_j.push_back(i * duration /3.0/ (N - 1) + 5 * duration/3.0);
    init_l_j.push_back(init_l_vec);
    init_lc_j.push_back(init_l_vec);
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
void SpiritBound<Y>::addCostLegs(MultibodyPlant<Y>& plant,
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
std::vector<drake::solvers::Binding<drake::solvers::Cost>> SpiritBound<Y>::addCost(
            MultibodyPlant<Y>& plant,
            dairlib::systems::trajectory_optimization::Dircon<Y>& trajopt){
  auto u   = trajopt.input();
  auto x   = trajopt.state();
  
  // Setup the traditional cost function
  trajopt.AddRunningCost( u.transpose()*this->cost_actuation*u);
  trajopt.AddVelocityCost(this->cost_velocity);
  trajopt.AddRunningCost(this->cost_time);

  // Hard code which joints are in flight for which mode
  addCostLegs(plant, trajopt,  {0, 1, 4, 5, 8, 10}, 1);
  addCostLegs(plant, trajopt,  {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}, 2);
  addCostLegs(plant, trajopt, {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}, 3);
  addCostLegs(plant, trajopt,  {2, 3, 6, 7, 10, 11}, 4);

  return AddWorkCost(plant, trajopt, this->cost_work);
}
/// Adds constraints to the trajopt bound problem
/// \param plant robot model
/// \param trajopt trajectory optimization problem to be solved
template <class Y>
void SpiritBound<Y>::addConstraints(
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
  auto xlo = trajopt.state_vars(2,0);
  auto xapex  = trajopt.state_vars(3,0) ;
  auto xtd  =  trajopt.state_vars(4,0);
  auto xf  = trajopt.final_state();


  /// Initial constraint

  // xy position
  trajopt.AddBoundingBoxConstraint(0, 0, x0(positions_map.at("base_x"))); // Give the initial condition room to choose the x_init position
  trajopt.AddBoundingBoxConstraint( -eps, eps, x0( positions_map.at("base_y")));
  // Nominal stand
  nominalSpiritStandConstraint(plant,trajopt,initial_height, {0}, eps);
  // Body pose constraints (keep the body flat) at initial state
  trajopt.AddBoundingBoxConstraint(1, 1 , x0(positions_map.at("base_qw")));
  trajopt.AddBoundingBoxConstraint(0 , 0, x0(positions_map.at("base_qx")));
  trajopt.AddBoundingBoxConstraint(0, 0, x0(positions_map.at("base_qy")));
  trajopt.AddBoundingBoxConstraint(0, 0 , x0(positions_map.at("base_qz")));

  // Initial  velocity
  trajopt.AddBoundingBoxConstraint(VectorXd::Zero(n_v), VectorXd::Zero(n_v), x0.tail(n_v));

  /// LO constraints
  // Limit magnitude of pitch
  double pitch = abs(pitch_magnitude_lo);
  trajopt.AddBoundingBoxConstraint(cos(pitch/2.0), 1, xtd(positions_map.at("base_qw")));
  trajopt.AddBoundingBoxConstraint(-eps, eps, xtd(positions_map.at("base_qx")));
  trajopt.AddBoundingBoxConstraint(-sin(pitch/2.0) , sin(pitch/2.0), xtd(positions_map.at("base_qy")));
  trajopt.AddBoundingBoxConstraint(-eps, eps, xtd(positions_map.at("base_qz")));

  /// Apex Flight constraints
  // Velocity
  trajopt.AddBoundingBoxConstraint(0, 0, xapex(plant.num_positions()+velocities_map.at("base_vz")));
  // Height
  if (apex_height > 0.15)
    trajopt.AddBoundingBoxConstraint(apex_height-1e-2, 1, xapex(positions_map.at("base_z")));
  // Limit magnitude of pitch
  pitch = abs(pitch_magnitude_apex);
  trajopt.AddBoundingBoxConstraint(cos(pitch/2.0), 1, xapex(positions_map.at("base_qw")));
  trajopt.AddBoundingBoxConstraint(-eps, eps, xapex(positions_map.at("base_qx")));
  trajopt.AddBoundingBoxConstraint(-sin(pitch/2.0) , sin(pitch/2.0), xapex(positions_map.at("base_qy")));
  trajopt.AddBoundingBoxConstraint(-eps, eps, xapex(positions_map.at("base_qz")));

  /// TD constraints
  // Limit magnitude of pitch
  pitch = abs(pitch_magnitude_lo);
  trajopt.AddBoundingBoxConstraint(cos(pitch/2.0), 1, xtd(positions_map.at("base_qw")));
  trajopt.AddBoundingBoxConstraint(-eps, eps, xtd(positions_map.at("base_qx")));
  trajopt.AddBoundingBoxConstraint(-sin(pitch/2.0) , sin(pitch/2.0), xtd(positions_map.at("base_qy")));
  trajopt.AddBoundingBoxConstraint(-eps, eps, xtd(positions_map.at("base_qz")));


  /// Final constraints
  // x,y position
  trajopt.AddBoundingBoxConstraint(0-eps, 0+eps, xf(positions_map.at("base_y")));
  if (fore_aft_displacement >= 0){
    trajopt.AddBoundingBoxConstraint(fore_aft_displacement-eps, fore_aft_displacement, xf(positions_map.at("base_x")));
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

  /// Constraints on all points
  for (int i = 0; i < trajopt.N(); i++){
    auto xi = trajopt.state(i);
    // Height
    trajopt.AddBoundingBoxConstraint( 0.15, 2, xi( positions_map.at("base_z")));
    trajopt.AddBoundingBoxConstraint( -eps, eps, xi( n_q+velocities_map.at("base_vy")));
  }

}


template <class Y>
void SpiritBound<Y>::setUpModeSequence(){
  this->mode_vector.clear();
  this->normal_vector.clear();
  this->offset_vector.clear();
  this->minT_vector.clear();
  this->maxT_vector.clear();
  //                          mode name   normal                  world offset            minT  maxT
  this->addModeToSequenceVector("stance",Eigen::Vector3d::UnitZ(),Eigen::Vector3d::Zero(),0.02, 0.5);
  this->addModeToSequenceVector("rear_stance",Eigen::Vector3d::UnitZ(),Eigen::Vector3d::Zero(),0.02, 1);
  this->addModeToSequenceVector("flight",Eigen::Vector3d::UnitZ(),Eigen::Vector3d::Zero(),0.02, 1);
  this->addModeToSequenceVector("flight",Eigen::Vector3d::UnitZ(),Eigen::Vector3d::Zero(),0.02, 1);
  this->addModeToSequenceVector("front_stance",Eigen::Vector3d::UnitZ(),Eigen::Vector3d::Zero(),0.02, 0.1);
  this->addModeToSequenceVector("stance",Eigen::Vector3d::UnitZ(),Eigen::Vector3d::Zero(),0.02, 0.1);
}

/// Runs a trajectory optimization problem for spirit bounding on flat ground
/// \param plant: robot model
/// \param pp_xtraj: state trajectory passed by pointer for spirit animation
/// \param surface_vector vector of surfaces in the scene (for animation)
template <class Y>
void SpiritBound<Y>::run(MultibodyPlant<Y>& plant,
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
    trajopt.SetSolverOption(id, "max_iter", 1000000);
    trajopt.SetSolverOption(id, "nlp_lower_bound_inf", -1e6);
    trajopt.SetSolverOption(id, "nlp_upper_bound_inf", 1e6);
    trajopt.SetSolverOption(id, "print_timing_statistics", "yes");
    trajopt.SetSolverOption(id, "print_level", 5);

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
    std::cout<<"Loading decision var from file, will fail if num dec vars changed" <<std::endl;
    dairlib::DirconTrajectory loaded_traj(this->file_name_in);
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
      dairlib::FindResourceOrThrow("examples/Spirit/spirit_drake.urdf"),
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

  auto x_trajs = trajopt.ReconstructDiscontinuousStateTrajectory(result);
  std::cout<<"Work = " << dairlib::calcElectricalWork(plant, x_trajs, this->u_traj) << std::endl;

  if(this->cost_work > 0){
    double cost_work_val = solvers::EvalCostGivenSolution(
        result, work_binding);
    std::cout<<"ReLu Work = " << cost_work_val/this->cost_work << std::endl;
  }

  /// Run animation of the final trajectory
  *pp_xtraj =trajopt.ReconstructStateTrajectory(result);
}

template class SpiritBound<double>;
}  // namespace dairlib

