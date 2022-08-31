#include "examples/Spirit_spine/spirit_trot_half.h"
#include <yaml-cpp/yaml.h>



using drake::multibody::MultibodyPlant;
using drake::trajectories::PiecewisePolynomial;
using Eigen::VectorXd;
using Eigen::MatrixXd;

using drake::multibody::Parser;
namespace dairlib {
using systems::trajectory_optimization::DirconModeSequence;
using systems::trajectory_optimization::Dircon;

template <class Y> 
SpiritTrotHalf<Y>::SpiritTrotHalf(){
}

/// Assigns values to member variables according to input yaml file
/// \param yaml_path path of the yaml file
/// \param saved_directory directory for saving the output trajectories (namely for setting file_name_out)
/// \param index indicates that we are using the index^th configuration
/// \param plant: robot model
template <class Y>
void SpiritTrotHalf<Y>::config(std::string yaml_path, std::string saved_directory, int index,MultibodyPlant<Y>* plant){
  YAML::Node config = YAML::LoadFile(yaml_path);
  this->index=index;
  this->ipopt=config[index]["ipopt"].as<bool>();
  this->num_knot_points=config[index]["num_knot_points"].as<std::vector<int>>();
  this->pitch_magnitude_lo=config[index]["pitch_magnitude_lo"].as<double>();
  this->max_spine_magnitude=config[index]["max_spine_magnitude"].as<double>();
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

  if(!config[index]["file_name_out"].as<std::string>().empty()) this->file_name_out=saved_directory+config[index]["file_name_out"].as<std::string>();
  if(!config[index]["file_name_in"].as<std::string>().empty()) this->file_name_in= saved_directory+config[index]["file_name_in"].as<std::string>();
  if(config[index]["action"]) this->action=config[index]["action"].as<std::string>();
  else this->action="";

}


/// Generate a bad initial guess for the spirit trot traj opt
/// \param plant: robot model
template <class Y>
void SpiritTrotHalf<Y>::generateInitialGuess(MultibodyPlant<Y>& plant){
  double stand_height = 0.35;
  double pitch_lo = -0.05;
  double apex_height = 0.38;
  double duration = 0.6;
  std::vector<MatrixXd> x_points;
  VectorXd x_const;

  
  //Flight 1
  dairlib::ikSpiritStand(plant, x_const, {false, false, false, false}, apex_height, stand_height, 0, 0);
  x_points.push_back(x_const);
  dairlib::ikSpiritStand(plant, x_const, {true, false, false, true}, stand_height+0.02, stand_height, 0, -pitch_lo);
  x_points.push_back(x_const);

  this->x_traj.push_back(PiecewisePolynomial<double>::FirstOrderHold({0 * duration/3.0, 1 * duration/3.0},x_points));
  x_points.clear();

  //Front stance
  dairlib::ikSpiritStand(plant, x_const, {true, false, false, true}, stand_height+0.02, stand_height, 0, -pitch_lo);
  x_points.push_back(x_const);
  dairlib::ikSpiritStand(plant, x_const, {true, false, false, true}, stand_height+0.01, stand_height, 0, -pitch_lo/2.0);
  x_points.push_back(x_const);

  this->x_traj.push_back(PiecewisePolynomial<double>::FirstOrderHold({1 * duration/3.0, 2 * duration/3.0},x_points));
  x_points.clear();

  //Flight 2
  dairlib::ikSpiritStand(plant, x_const, {true, false, false, true}, stand_height+0.01, stand_height, 0, -pitch_lo/2.0);
  x_points.push_back(x_const);
  dairlib::ikSpiritStand(plant, x_const, {false, false, false, false}, stand_height+0.02, stand_height, 0, 0);
  x_points.push_back(x_const);
  this->x_traj.push_back(PiecewisePolynomial<double>::FirstOrderHold({2 * duration/3.0, 3 * duration/3.0},x_points));
  x_points.clear();


  // Create control initial guess (zeros)
  std::vector<MatrixXd> u_points;
  u_points.push_back(MatrixXd::Zero( plant.num_actuators(), 1));
  u_points.push_back(MatrixXd::Zero( plant.num_actuators(), 1));

  this->u_traj = PiecewisePolynomial<double>::FirstOrderHold({0,  duration},u_points);

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

}




/// Adds cost to the trajopt problem
/// \param plant robot model
/// \param trajopt trajectory optimization problem to be solved
template <class Y>
std::vector<drake::solvers::Binding<drake::solvers::Cost>> SpiritTrotHalf<Y>::addCost(
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
  addCostLegs(plant, trajopt,  {2, 3, 4, 5, 9, 10}, 1);
  addCostLegs(plant, trajopt,  {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}, 2);

  return AddPowerCost(plant, trajopt, this->cost_power);
}
/// Adds constraints to the trajopt bound problem
/// \param plant robot model
/// \param trajopt trajectory optimization problem to be solved
template <class Y>
void SpiritTrotHalf<Y>::addConstraints(
                    MultibodyPlant<Y>& plant,
                    dairlib::systems::trajectory_optimization::Dircon<Y>& trajopt){
  // Get position and velocity dictionaries
  auto positions_map = multibody::makeNameToPositionsMap(plant);
  auto velocities_map = multibody::makeNameToVelocitiesMap(plant);
  
  auto actuators_map = multibody::makeNameToActuatorsMap(plant);
  // // Print joint dictionary
  // std::cout<<"**********************Actuators***********************"<<std::endl;
  // for (auto const& element : actuators_map)
  //   std::cout << element.first << " = " << element.second << std::endl;
  // // for (auto const& element : velocities_map)
  // //   std::cout << element.first << " = " << element.second << std::endl;
  // std::cout<<"***************************************************"<<std::endl;

  // General constraints
  setSpiritJointLimits(plant, trajopt);
  setSpiritActuationLimits(plant, trajopt);
  trajopt.AddDurationBounds(min_duration, max_duration);

  /// Setup all the optimization constraints
  int n_q = plant.num_positions();
  int n_v = plant.num_velocities();
  auto x0  = trajopt.initial_state();
  auto xtd = trajopt.state_vars(1,0);
  auto xll1 = trajopt.state_vars(2,0);
  auto xf  = trajopt.final_state();


  // xyz position
  trajopt.AddBoundingBoxConstraint(0, 0, x0(positions_map.at("base_x"))); // Give the initial condition room to choose the x_init position
  trajopt.AddBoundingBoxConstraint( -eps, eps, x0( positions_map.at("base_y")));
  if (apex_height>0) trajopt.AddBoundingBoxConstraint( this->apex_height-eps, this->apex_height+eps, x0( positions_map.at("base_z")));

  // velocities
  // trajopt.AddBoundingBoxConstraint(this->speed-eps, this->speed+eps, x0(n_q+velocities_map.at("base_vx")));
  trajopt.AddBoundingBoxConstraint(-eps, eps, x0(n_q+velocities_map.at("base_vy")));
  trajopt.AddBoundingBoxConstraint(-eps, eps, x0(n_q+velocities_map.at("base_vz")));


  if (lock_leg_apex){
    double upperSet = 1;
    double kneeSet = 2;
    double upperSet2 = 0.7;
    double kneeSet2 = 1.4;
    double hipSet=0;
    double apex_eps=0.1;

    trajopt.AddBoundingBoxConstraint(upperSet - apex_eps, upperSet + apex_eps, x0(positions_map.at("joint_0") ) );
    trajopt.AddBoundingBoxConstraint(kneeSet - apex_eps, kneeSet + apex_eps, x0(positions_map.at("joint_1") ) );

    trajopt.AddBoundingBoxConstraint(upperSet2 - apex_eps, upperSet2 + apex_eps, x0(positions_map.at("joint_2") ) );
    trajopt.AddBoundingBoxConstraint(kneeSet2 - apex_eps, kneeSet2 + apex_eps, x0(positions_map.at("joint_3") ) );

    trajopt.AddBoundingBoxConstraint(upperSet2 - apex_eps, upperSet2 + apex_eps, x0(positions_map.at("joint_4") ) );
    trajopt.AddBoundingBoxConstraint(kneeSet2 - apex_eps, kneeSet2 + apex_eps, x0(positions_map.at("joint_5") ) );

    trajopt.AddBoundingBoxConstraint(upperSet - apex_eps, upperSet + apex_eps, x0(positions_map.at("joint_6") ) );
    trajopt.AddBoundingBoxConstraint(kneeSet - apex_eps, kneeSet + apex_eps, x0(positions_map.at("joint_7") ) );

    trajopt.AddBoundingBoxConstraint(hipSet- apex_eps, hipSet + apex_eps, x0(positions_map.at("joint_8") ) );
    trajopt.AddBoundingBoxConstraint(hipSet - apex_eps, hipSet + apex_eps, x0(positions_map.at("joint_9") ) );
    trajopt.AddBoundingBoxConstraint(hipSet - apex_eps, hipSet + apex_eps, x0(positions_map.at("joint_10") ) );
    trajopt.AddBoundingBoxConstraint(hipSet - apex_eps, hipSet + apex_eps, x0(positions_map.at("joint_11") ) );
    if (this->spine_type=="twisting") trajopt.AddBoundingBoxConstraint( -apex_eps, apex_eps, x0(positions_map.at("joint_12") ) );
  }

  /// Touch down constraint
  /// Limit magnitude of pitch
  max_spine_new=max_spine_magnitude;
  pitch_magnitude=pitch_magnitude_lo;
  if (this->var!=0) {
    double suggested_magnitude=0.2+0.25*sqrt(this->speed);
    max_spine_new=suggested_magnitude*((((double)rand()) / ((double)RAND_MAX)) +0.5);
    if (pitch_magnitude_lo>0.3) pitch_magnitude=pitch_magnitude_lo*(((double)rand()) / ((double)RAND_MAX));
  }
  if (this->spine_type=="rigid") max_spine_new*=0.3;
  std::cout<<"MAX SPINE: "<<max_spine_new<<"  MAX PITCH: "<< pitch_magnitude<<"SPINE TYPE: "<<this->spine_type<<std::endl;
  

  // /// Toes constraints
  toe_height=0.05;
  double upperLegLength=0.206;
  double lowerLegLength=0.206;
  double bodyWidth=0.24-0.05*2;
  double abOffs=0.10098;
  double toeRadius=0.02;
  double bodyLength=0.335+0.055*2;
  // Compute roll angle
  int midKnotpoint = trajopt.mode_length(1)/2;
  // Constraints multiple knot points in the apex to prevent shaky legs problem
  for (int k=midKnotpoint-1;k<midKnotpoint+2;k++){
    auto xmid = trajopt.state_vars( 1 , k);
    auto roll = atan2(2.0 * (xmid(0) * xmid(1) + xmid(2) * xmid(3)), 1.0 - 2.0 * (xmid(1) * xmid(1) + xmid(2) * xmid(2)));
    auto pitch = asin(2.0 * (xmid(0) * xmid(2) + xmid(3) * xmid(1)));
    // Toe 2
    trajopt.AddConstraint(xmid(positions_map.at("base_z"))+0.5*bodyWidth*sin(roll)+0.5*(bodyLength)*sin(pitch)-cos(xmid(positions_map.at("joint_9"))+roll)*
                          (upperLegLength*sin(xmid(positions_map.at("joint_2"))-pitch)+lowerLegLength*sin(xmid(positions_map.at("joint_3"))
                          -xmid(positions_map.at("joint_2"))+pitch))+abOffs*sin(xmid(positions_map.at("joint_9"))+roll),toe_height, toe_height+0.3);
    // Toe 3
    if (this->spine_type=="twisting") 
      trajopt.AddConstraint(xmid(positions_map.at("base_z"))-0.5*bodyWidth*sin(xmid(positions_map.at("joint_12"))+roll)-0.5*bodyLength*sin(pitch)
                          -cos(xmid(positions_map.at("joint_10"))+xmid(positions_map.at("joint_12"))+roll)*(upperLegLength*sin(xmid(positions_map.at("joint_4"))-pitch)
                          +lowerLegLength*sin(pitch+xmid(positions_map.at("joint_5"))-xmid(positions_map.at("joint_4"))))
                          -abOffs*sin(xmid(positions_map.at("joint_12"))+xmid(positions_map.at("joint_10"))+roll),toe_height, toe_height+0.3);
    else 
      trajopt.AddConstraint(xmid(positions_map.at("base_z"))-0.5*bodyWidth*sin(roll)-0.5*bodyLength*sin(pitch)
                          -cos(xmid(positions_map.at("joint_10"))+roll)*(upperLegLength*sin(xmid(positions_map.at("joint_4"))-pitch)
                          +lowerLegLength*sin(pitch+xmid(positions_map.at("joint_5"))-xmid(positions_map.at("joint_4"))))
                          -abOffs*sin(xmid(positions_map.at("joint_10"))+roll),toe_height, toe_height+0.3);
  }


  /// Final constraints
  //Constraint orientation and position
  for (int i=0;i<n_q+n_v;i++) {
    /// Flip quaternion
    trajopt.AddConstraint( xf(0)-x0(0),-eps, eps);
    trajopt.AddConstraint( xf(1)+x0(1),-eps, eps); // opposite qx
    trajopt.AddConstraint( xf(2)-x0(2),-eps, eps);
    trajopt.AddConstraint( xf(3)+x0(3),-eps, eps); // opposite qz
    /// Position
    trajopt.AddConstraint( xf(5)+x0(5),-eps, eps); // opposite y
    trajopt.AddConstraint( xf(6)-x0(6),-eps, eps);
    /// Joint positions
    trajopt.AddConstraint( xf(positions_map.at("joint_0"))-x0(positions_map.at("joint_4")),-eps,eps);
    trajopt.AddConstraint( xf(positions_map.at("joint_1"))-x0(positions_map.at("joint_5")),-eps,eps);
    trajopt.AddConstraint( xf(positions_map.at("joint_2"))-x0(positions_map.at("joint_6")),-eps,eps);
    trajopt.AddConstraint( xf(positions_map.at("joint_3"))-x0(positions_map.at("joint_7")),-eps,eps);
    trajopt.AddConstraint( xf(positions_map.at("joint_4"))-x0(positions_map.at("joint_0")),-eps,eps);
    trajopt.AddConstraint( xf(positions_map.at("joint_5"))-x0(positions_map.at("joint_1")),-eps,eps);
    trajopt.AddConstraint( xf(positions_map.at("joint_6"))-x0(positions_map.at("joint_2")),-eps,eps);
    trajopt.AddConstraint( xf(positions_map.at("joint_7"))-x0(positions_map.at("joint_3")),-eps,eps);
    trajopt.AddConstraint( xf(positions_map.at("joint_8"))+x0(positions_map.at("joint_10")),-eps,eps); // Opposite
    trajopt.AddConstraint( xf(positions_map.at("joint_9"))+x0(positions_map.at("joint_11")),-eps,eps); // Opposite
    trajopt.AddConstraint( xf(positions_map.at("joint_10"))+x0(positions_map.at("joint_8")),-eps,eps); // Opposite
    trajopt.AddConstraint( xf(positions_map.at("joint_11"))+x0(positions_map.at("joint_9")),-eps,eps); // Opposite
    if (this->spine_type=="twisting") trajopt.AddConstraint( xf(positions_map.at("joint_12"))+x0(positions_map.at("joint_12")),-eps,eps); //opposite joint 12
    /// Base Velocity
    trajopt.AddConstraint(xf(n_q+velocities_map.at("base_vx"))-x0(n_q+velocities_map.at("base_vx")),-eps,eps);
    trajopt.AddConstraint(xf(n_q+velocities_map.at("base_vy"))+x0(n_q+velocities_map.at("base_vy")),-eps,eps); //opposite vy
    trajopt.AddConstraint(xf(n_q+velocities_map.at("base_vz"))-x0(n_q+velocities_map.at("base_vz")),-eps,eps);
    /// angular velocity
    trajopt.AddConstraint(xf(n_q+velocities_map.at("base_wx"))+x0(n_q+velocities_map.at("base_wx")),-eps,eps); //opposite wx
    trajopt.AddConstraint(xf(n_q+velocities_map.at("base_wy"))-x0(n_q+velocities_map.at("base_wy")),-eps,eps); 
    trajopt.AddConstraint(xf(n_q+velocities_map.at("base_wz"))+x0(n_q+velocities_map.at("base_wz")),-eps,eps); //opposite wz
    /// joint velocities
    trajopt.AddConstraint(xf(n_q+velocities_map.at("joint_0dot"))-x0(n_q+velocities_map.at("joint_4dot")),-eps,eps);
    trajopt.AddConstraint(xf(n_q+velocities_map.at("joint_1dot"))-x0(n_q+velocities_map.at("joint_5dot")),-eps,eps);
    trajopt.AddConstraint(xf(n_q+velocities_map.at("joint_2dot"))-x0(n_q+velocities_map.at("joint_6dot")),-eps,eps);
    trajopt.AddConstraint(xf(n_q+velocities_map.at("joint_3dot"))-x0(n_q+velocities_map.at("joint_7dot")),-eps,eps);
    trajopt.AddConstraint(xf(n_q+velocities_map.at("joint_4dot"))-x0(n_q+velocities_map.at("joint_0dot")),-eps,eps);
    trajopt.AddConstraint(xf(n_q+velocities_map.at("joint_5dot"))-x0(n_q+velocities_map.at("joint_1dot")),-eps,eps);
    trajopt.AddConstraint(xf(n_q+velocities_map.at("joint_6dot"))-x0(n_q+velocities_map.at("joint_2dot")),-eps,eps);
    trajopt.AddConstraint(xf(n_q+velocities_map.at("joint_7dot"))-x0(n_q+velocities_map.at("joint_3dot")),-eps,eps);
    trajopt.AddConstraint(xf(n_q+velocities_map.at("joint_8dot"))+x0(n_q+velocities_map.at("joint_10dot")),-eps,eps); // Opposite
    trajopt.AddConstraint(xf(n_q+velocities_map.at("joint_9dot"))+x0(n_q+velocities_map.at("joint_11dot")),-eps,eps); // Opposite
    trajopt.AddConstraint(xf(n_q+velocities_map.at("joint_10dot"))+x0(n_q+velocities_map.at("joint_8dot")),-eps,eps); // Opposite
    trajopt.AddConstraint(xf(n_q+velocities_map.at("joint_11dot"))+x0(n_q+velocities_map.at("joint_9dot")),-eps,eps); // Opposite
    if (this->spine_type=="twisting") trajopt.AddConstraint(xf(n_q+velocities_map.at("joint_12dot"))+x0(n_q+velocities_map.at("joint_12dot")),-eps,eps); //opposite v joint 12
  }
  ///Constraint joints

  // For knee angular acceleration
  auto times  = trajopt.time_vars();
  double a_knee_max=1000;

  /// Constraints on all points
  for (int i = 0; i < trajopt.N(); i++){
    auto xi = trajopt.state(i);
    // Limit roll and yaw
    trajopt.AddBoundingBoxConstraint(cos(pitch_magnitude/2.0)*cos(max_spine_new/4), 1, xi(positions_map.at("base_qw")));
    trajopt.AddBoundingBoxConstraint(-sin(max_spine_new/4)*cos(pitch_magnitude/2.0), sin(max_spine_new/4)*cos(pitch_magnitude/2.0), xi(positions_map.at("base_qx")));
    trajopt.AddBoundingBoxConstraint(-cos(max_spine_new/4)*sin(pitch_magnitude/2.0) , cos(max_spine_new/4)*sin(pitch_magnitude/2.0), xi(positions_map.at("base_qy")));
    trajopt.AddBoundingBoxConstraint(-sin(max_spine_new/4)*sin(pitch_magnitude/2.0) , sin(max_spine_new/4)*sin(pitch_magnitude/2.0), xi(positions_map.at("base_qz")));
    // Height
    trajopt.AddBoundingBoxConstraint( 0.25, 2, xi( positions_map.at("base_z")));
    trajopt.AddBoundingBoxConstraint( -eps, eps, xi( n_q+velocities_map.at("base_vy")));
    if (this->spine_type=="twisting") trajopt.AddBoundingBoxConstraint( -max_spine_new-eps, max_spine_new+eps, xi( positions_map.at("joint_12")));

    //Toes
    
    // Compute roll angle
    auto roll_i = atan2(2.0 * (xi(0) * xi(1) + xi(2) * xi(3)), 1.0 - 2.0 * (xi(1) * xi(1) + xi(2) * xi(2)));
    auto pitch_i = asin(2.0 * (xi(0) * xi(2) + xi(3) * xi(1)));
    // Toe 2 
    trajopt.AddConstraint(xi(positions_map.at("base_z"))+0.5*bodyWidth*sin(roll_i)+0.5*(bodyLength)*sin(pitch_i)-cos(xi(positions_map.at("joint_9"))+roll_i)*
                          (upperLegLength*sin(xi(positions_map.at("joint_2"))-pitch_i)+lowerLegLength*sin(xi(positions_map.at("joint_3"))
                        -xi(positions_map.at("joint_2"))+pitch_i)) +abOffs*cos(pitch_i)*sin(xi(positions_map.at("joint_9"))+roll_i),toeRadius, 0.3);
    // Toe 3
    if (this->spine_type=="twisting")
      trajopt.AddConstraint(xi(positions_map.at("base_z"))-0.5*bodyWidth*sin(xi(positions_map.at("joint_12"))+roll_i)-0.5*bodyLength*sin(pitch_i)
                            -cos(xi(positions_map.at("joint_10"))+xi(positions_map.at("joint_12"))+roll_i)*(upperLegLength*sin(xi(positions_map.at("joint_4"))-pitch_i)
                            +lowerLegLength*sin(pitch_i+xi(positions_map.at("joint_5"))-xi(positions_map.at("joint_4"))))
                            -abOffs*cos(pitch_i)*sin(xi(positions_map.at("joint_12"))+xi(positions_map.at("joint_10"))+roll_i),toeRadius, 0.3);
    else 
      trajopt.AddConstraint(xi(positions_map.at("base_z"))-0.5*bodyWidth*sin(roll_i)-0.5*bodyLength*sin(pitch_i)
                            -cos(xi(positions_map.at("joint_10"))+roll_i)*(upperLegLength*sin(xi(positions_map.at("joint_4"))-pitch_i)
                            +lowerLegLength*sin(pitch_i+xi(positions_map.at("joint_5"))-xi(positions_map.at("joint_4"))))
                            -abOffs*cos(pitch_i)*sin(xi(positions_map.at("joint_10"))+roll_i),toeRadius, 0.3);

    //// Toes should not cross the central line
    // Toe 2
    trajopt.AddConstraint(sin(xi(positions_map.at("joint_9"))+roll_i)*
                          (upperLegLength*sin(xi(positions_map.at("joint_2")))+lowerLegLength*sin(xi(positions_map.at("joint_3"))
                          -xi(positions_map.at("joint_2")))),-0.5*bodyWidth, 0.15);
    // Toe 3
    if (this->spine_type=="twisting") 
      trajopt.AddConstraint(sin(xi(positions_map.at("joint_10"))+xi(positions_map.at("joint_12"))+roll_i)*(upperLegLength*sin(xi(positions_map.at("joint_4")))
                          +lowerLegLength*sin(xi(positions_map.at("joint_5"))-xi(positions_map.at("joint_4")))),-0.15, 0.5*bodyWidth);
    else 
      trajopt.AddConstraint(sin(xi(positions_map.at("joint_10"))+roll_i)*(upperLegLength*sin(xi(positions_map.at("joint_4")))
                          +lowerLegLength*sin(xi(positions_map.at("joint_5"))-xi(positions_map.at("joint_4")))),-0.15, 0.5*bodyWidth);
    
    // Limit knee joints' angular accelerations
    if(i>0){
        // auto fi=trajopt.input(i);
        // auto fim1=trajopt.input(i-1);
        auto xim1=trajopt.state(i-1);
        trajopt.AddConstraint((xi(n_q+velocities_map.at("joint_1dot"))-xim1(n_q+velocities_map.at("joint_1dot")))/times(i-1),-a_knee_max,a_knee_max);
        trajopt.AddConstraint((xi(n_q+velocities_map.at("joint_3dot"))-xim1(n_q+velocities_map.at("joint_3dot")))/times(i-1),-a_knee_max,a_knee_max);
        trajopt.AddConstraint((xi(n_q+velocities_map.at("joint_5dot"))-xim1(n_q+velocities_map.at("joint_5dot")))/times(i-1),-a_knee_max,a_knee_max);
        trajopt.AddConstraint((xi(n_q+velocities_map.at("joint_7dot"))-xim1(n_q+velocities_map.at("joint_7dot")))/times(i-1),-a_knee_max,a_knee_max);
    }

  }

  //Average Velocity
  auto total_time=times(0)+times(1);
  for (int i=2;i<trajopt.N()-1;i++) total_time+=times(i);
  trajopt.AddConstraint(  this->speed * total_time+x0(positions_map.at("base_x")) -xf(positions_map.at("base_x")),-eps, eps );
  // trajopt.AddConstraint(  this->speed * times(0)+ times(1),-eps, eps );
  // drake::solvers::VectorXDecisionVariable C(times.rows(), times.cols()+x0(positions_map.at("joint_4")));
}

/// add cost for legs
/// \param joints add cost for which joint of the legs
/// \param mode_index add most for which mode 
template <class Y>
void SpiritTrotHalf<Y>::addCostLegs(MultibodyPlant<Y>& plant,
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

template <class Y>
void SpiritTrotHalf<Y>::setUpModeSequence(){
  this->mode_vector.clear();
  this->normal_vector.clear();
  this->offset_vector.clear();
  this->minT_vector.clear();
  this->maxT_vector.clear();
  //                          mode name   normal                  world offset            minT  maxT
  this->addModeToSequenceVector("flight",Eigen::Vector3d::UnitZ(),Eigen::Vector3d::Zero(),0.001,0.2);
  this->addModeToSequenceVector("diag1",Eigen::Vector3d::UnitZ(),Eigen::Vector3d::Zero(),0.3, 0.7);
  this->addModeToSequenceVector("flight",Eigen::Vector3d::UnitZ(),Eigen::Vector3d::Zero(),0.001, 0.2);
}

/// Runs a trajectory optimization problem for spirit bounding on flat ground
/// \param plant: robot model
/// \param pp_xtraj: state trajectory passed by pointer for spirit animation
/// \param surface_vector vector of surfaces in the scene (for animation)
template <class Y>
void SpiritTrotHalf<Y>::run(MultibodyPlant<Y>& plant,
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
    this->addGaussionNoiseToInputTraj(plant,trajopt,loaded_traj);
    this->addGaussionNoiseToVelocitiesTraj(plant,trajopt,loaded_traj);
    this->addGaussionNoiseToStateTraj(plant,trajopt,loaded_traj);
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
  // Writing contact force data
  // std::string contect_force_fname="/home/feng/Downloads/dairlib/examples/Spirit_spine/data/trot_half/twisting/"+this->file_name_out+".csv";
  // this->saveContactForceData(this->speed,contect_force_fname);
  // Writing contact force data
  // std::stringstream stream;
  // stream << std::fixed << std::setprecision(2) << this->speed;
  // std::string contect_force_fname="/home/feng/Downloads/dairlib/examples/Spirit_spine/data/trot_half/twisting_p5/trot_"+stream.str()+".csv";
  int beginIdx = this->file_name_out.rfind('/');
  std::string filename = this->file_name_out.substr(beginIdx + 1);
  std::string contact_force_fname=this->data_directory+filename +".csv";
  this->saveContactForceData(trajopt,result,this->speed,max_spine_new,contact_force_fname,result.is_success());
  
  // auto x_trajs = trajopt.ReconstructDiscontinuousStateTrajectory(result);
  // std::cout<<"Work = " << dairlib::calcElectricalWork(plant, x_trajs, this->u_traj) << std::endl;

  // if(this->cost_power > 0){
  //   double cost_power_val = solvers::EvalCostGivenSolution(
  //       result, work_binding);
  //   std::cout<<"ReLu Work = " << cost_power_val/this->cost_power << std::endl;
  // }

  
  /////////////////////////////////////// Build up animation ///////////////////////
  /////// Create whole-period trajectory
  PiecewisePolynomial<Y> org_x_traj=trajopt.ReconstructStateTrajectory(result);
  PiecewisePolynomial<Y> flipped_x_traj=trajopt.ReconstructFlippedStateTrajectory(result,this->spine_type);
  // Create half 
  std::vector<double> breaks_half=flipped_x_traj.get_breaks();
  std::vector<Eigen::MatrixXd> samples_half(breaks_half.size());
  int nq;
  if (this->spine_type=="twisting") nq=39;
  else if (this->spine_type=="rigid") nq=37;
  for (int i = 0; i < static_cast<int>(breaks_half.size()); ++i) {
    samples_half[i].resize(nq, 1);
    for (int j=0;j<nq;j++) samples_half[i](j, 0) = 0;
    samples_half[i](4, 0) = org_x_traj.value(org_x_traj.end_time())(4,0);
  }
  PiecewisePolynomial<double> half_offset_pp = PiecewisePolynomial<double>::FirstOrderHold(breaks_half, samples_half);
  flipped_x_traj+=half_offset_pp;
  flipped_x_traj.shiftRight(org_x_traj.end_time());
  org_x_traj.ConcatenateInTime(flipped_x_traj);
  PiecewisePolynomial<Y> whole_x_traj=org_x_traj; // Get the whole-period trajectory
  *pp_xtraj=whole_x_traj;
  // Create offset piecewise polynomial
  std::vector<double> breaks=whole_x_traj.get_breaks();
  std::vector<Eigen::MatrixXd> samples(breaks.size());
  for (int i = 0; i < static_cast<int>(breaks.size()); ++i) {
    samples[i].resize(nq, 1);
    for (int j=0;j<nq;j++) samples[i](j, 0) = 0;
    samples[i](4, 0) = whole_x_traj.value(whole_x_traj.end_time())(4,0);
  }
  PiecewisePolynomial<double> ini_offset_pp = PiecewisePolynomial<double>::FirstOrderHold(breaks, samples);
  PiecewisePolynomial<double> offset_pp=ini_offset_pp;

  /// Concatenate 10 periods
  for (int i=0;i<10;i++){
    PiecewisePolynomial<Y> x_traj_i=whole_x_traj+offset_pp;
    offset_pp+=ini_offset_pp;
    x_traj_i.shiftRight(pp_xtraj->end_time());
    pp_xtraj->ConcatenateInTime(x_traj_i);
  }

  
  
}

template class SpiritTrotHalf<double>;
}  // namespace dairlib

