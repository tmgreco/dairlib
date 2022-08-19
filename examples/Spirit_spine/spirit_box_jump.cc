#include "examples/Spirit_spine/spirit_box_jump.h"
#include "examples/Spirit_spine/surface_conf.h"
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
SpiritBoxJump<Y>::SpiritBoxJump():plant (std::make_unique<MultibodyPlant<Y>>(0.0))
{
    Parser parser(plant.get());
    std::string full_name =
        dairlib::FindResourceOrThrow("examples/Spirit_spine/spirit_with_spine_drake.urdf");
    parser.AddModelFromFile(full_name);
    plant->mutable_gravity_field().set_gravity_vector(-9.81 *
        Eigen::Vector3d::UnitZ());
    plant->Finalize();
    initialStand.init(plant.get(), 0.2, Eigen::Vector3d::UnitZ(), Eigen::Vector3d::Zero(),false);
    finalStand.init(plant.get(), 0.2, Eigen::Vector3d::UnitZ(), Eigen::Vector3d::Zero(),false);
}

template <class Y>
void SpiritBoxJump<Y>::config(
  std::string yaml_path, std::string saved_directory, int index,MultibodyPlant<Y>* plant)
  {

  YAML::Node config = YAML::LoadFile(yaml_path);
  this->num_knot_points =config[index]["num_knot_points"].as<std::vector<int>>();
  this->apex_height =config[index]["apex_height"].as<double>();
  this->initial_height = config[index]["initial_height"].as<double>();
  this->fore_aft_displacement =config[index]["fore_aft_displacement"].as<double>();
  this->lock_rotation =config[index]["lock_rotation"].as<bool>();
  this->lock_legs_apex =config[index]["lock_legs_apex"].as<bool>();
  this->force_symmetry =config[index]["force_symmetry"].as<bool>();
  this->use_nominal_stand =config[index]["use_nominal_stand"].as<bool>();
  this->max_duration =config[index]["max_duration"].as<double>();
  this->cost_actuation =config[index]["cost_actuation"].as<double>();
  this->cost_velocity =config[index]["cost_velocity"].as<double>();
  this->cost_work =config[index]["cost_work"].as<double>();
  this->mu =config[index]["mu"].as<double>();
  this->eps =config[index]["eps"].as<double>();
  this->tol =config[index]["tol"].as<double>();
  this->work_constraint_scale =config[index]["work_constraint_scale"].as<double>();
  if(!config[index]["file_name_out"].as<std::string>().empty()) this->file_name_out=saved_directory+config[index]["file_name_out"].as<std::string>();
  if(!config[index]["file_name_in"].as<std::string>().empty()) this->file_name_in= saved_directory+config[index]["file_name_in"].as<std::string>();
  std::cout<<config[index]["initial_stand"][0].as<std::string>() <<std::endl;
  Eigen::Vector3d initial_normal =  config[index]["initial_stand"][0].as<double>() *Eigen::Vector3d::UnitX()+
                            config[index]["initial_stand"][1].as<double>() *Eigen::Vector3d::UnitY()+
                            config[index]["initial_stand"][2].as<double>()  *Eigen::Vector3d::UnitZ();
  initial_normal = initial_normal/initial_normal.norm();
  Eigen::Vector3d initial_offset = config[index]["initial_stand"][3].as<double>() *Eigen::Vector3d::UnitX()+ 
                           config[index]["initial_stand"][4].as<double>() *Eigen::Vector3d::UnitY()+
                           config[index]["initial_stand"][5].as<double>() *Eigen::Vector3d::UnitZ();

  Eigen::Vector3d final_normal =  config[index]["final_stand"][0].as<double>() *Eigen::Vector3d::UnitX()+
                            config[index]["final_stand"][1].as<double>() *Eigen::Vector3d::UnitY()+
                            config[index]["final_stand"][2].as<double>()  *Eigen::Vector3d::UnitZ();
  final_normal = final_normal/final_normal.norm();
  Eigen::Vector3d final_offset = config[index]["final_stand"][3].as<double>() *Eigen::Vector3d::UnitX()+ 
                           config[index]["final_stand"][4].as<double>() *Eigen::Vector3d::UnitY()+
                           config[index]["final_stand"][5].as<double>() *Eigen::Vector3d::UnitZ();
  this->initialStand.init(plant,config[index]["stand_height"].as<double>(),initial_normal, initial_offset,config[index]["initial_stand"][6].as<bool>());
  this->finalStand.init(plant, config[index]["stand_height"].as<double>(), final_normal, final_offset, config[index]["final_stand"][6].as<bool>());


}



template <class Y>
void SpiritBoxJump<Y>::generateInitialGuess(MultibodyPlant<Y>& plant){
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(plant.num_positions() +
      plant.num_velocities());

  int nu = plant.num_actuators();
  int nx = plant.num_positions() + plant.num_velocities();
  int nq = plant.num_positions();
  int nv = plant.num_velocities();
  int N = 20; // number of timesteps

  std::vector<MatrixXd> init_x;
  std::vector<MatrixXd> init_u;
  this->l_traj.clear();
  this->lc_traj.clear();
  this->vc_traj.clear();

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

  xMid(positions_map.at("base_z"))=apex_height;

  VectorXd deltaX(nx);
  VectorXd averageV(nx);
  deltaX = xMid-xInit;
  double duration=1;
  averageV = deltaX / duration;
  xInit.tail(nv-3) = (averageV.head(nq)).tail(nq-4); //Ignoring Orientation make velocity the average
  double time = 0;
  double dt = duration/(N-1)/2;

  // Initial pose
  xState = xInit;

  for (int i = 0; i < N; i++) {
    time=i*dt; // calculate iteration's time
    init_time.push_back(time);

    // Switch the direction of the stand to go back to the initial state (not actually properly periodic initial)
    if ( i > (N-1)/2 ){
      xState.tail(nv) = -xInit.tail(nv);
    }
    // Integrate the positions based on constant velocity for joints and xyz
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
  this->x_traj = PiecewisePolynomial<double>::ZeroOrderHold(init_time, init_x);
  this->u_traj = PiecewisePolynomial<double>::ZeroOrderHold(init_time, init_u);


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
    init_time_j.push_back(i * duration / (N - 1));
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

  // Flight
  init_l_j.clear();
  init_lc_j.clear();
  init_vc_j.clear();
  init_time_j.clear();
  for (int i = 0; i < N; i++) {
    init_time_j.push_back(i * duration / (N - 1));
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
  this->l_traj.push_back(init_l_traj_j);
  this->lc_traj.push_back(init_lc_traj_j);
  this->vc_traj.push_back(init_vc_traj_j);

  // Stance
  init_l_j.clear();
  init_lc_j.clear();
  init_vc_j.clear();
  init_time_j.clear();
  for (int i = 0; i < N; i++) {
    init_time_j.push_back(i * duration / (N - 1));
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

template <class Y>
void SpiritBoxJump<Y>::addCost(
            MultibodyPlant<Y>& plant,
            dairlib::systems::trajectory_optimization::Dircon<Y>& trajopt){
  auto velocities_map = multibody::makeNameToVelocitiesMap(plant);
  auto actuator_map = multibody::makeNameToActuatorsMap(plant);


  int n_q = plant.num_positions();
  int n_v = plant.num_velocities();
  auto u   = trajopt.input();
  auto x   = trajopt.state();

  // Setup the traditional cost function
  const double R = this->cost_actuation;  // Cost on input effort
  const MatrixXd Q = this->cost_velocity  * MatrixXd::Identity(n_v, n_v); // Cost on velocity

  trajopt.AddRunningCost( x.tail(n_v).transpose() * Q * x.tail(n_v) );
  trajopt.AddRunningCost( u.transpose()*R*u);

  for (int joint = 0; joint < 12; joint++){
    auto power_plus = trajopt.NewSequentialVariable(1, "joint_" + std::to_string(joint)+"_power_plus");
    auto power_minus = trajopt.NewSequentialVariable(1, "joint_" + std::to_string(joint)+"_power_minus");

    trajopt.AddRunningCost(this->cost_work * (power_plus + power_minus));

    for(int time_index = 0; time_index < trajopt.N(); time_index++){
      auto u_i   = trajopt.input(time_index);
      auto x_i   = trajopt.state(time_index);

      drake::symbolic::Variable actuation = u_i(actuator_map.at("motor_" + std::to_string(joint)));
      drake::symbolic::Variable velocity = x_i(n_q + velocities_map.at("joint_" + std::to_string(joint) +"dot"));
      drake::symbolic::Variable power_plus_i = trajopt.GetSequentialVariableAtIndex("joint_" + std::to_string(joint)+"_power_plus", time_index)[0];
      drake::symbolic::Variable power_minus_i = trajopt.GetSequentialVariableAtIndex("joint_" + std::to_string(joint)+"_power_minus", time_index)[0];


      if (this->cost_work > 0){
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
/// \param plant: robot model
/// \param trajopt: trajectory to be optimized
template <class Y>
void SpiritBoxJump<Y>::addConstraints(
                    MultibodyPlant<Y>& plant,
                    dairlib::systems::trajectory_optimization::Dircon<Y>& trajopt){

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
  // Initial velocity
  trajopt.AddBoundingBoxConstraint(VectorXd::Zero(n_v), VectorXd::Zero(n_v), x0.tail(n_v));

  // Lift off body positions conditions
  trajopt.AddBoundingBoxConstraint(-eps, eps, xlo(positions_map.at("base_y")));
  // Apex body positions conditions
  trajopt.AddBoundingBoxConstraint(-eps, eps, xapex(positions_map.at("base_y")));
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


  // Touchdown body positions conditions
  trajopt.AddBoundingBoxConstraint(-eps, eps, xtd(positions_map.at("base_y")));
  // Final body positions conditions
  trajopt.AddBoundingBoxConstraint(fore_aft_displacement - eps, fore_aft_displacement + eps, xf(positions_map.at("base_x"))); // Give the initial condition room to choose the x_init position (helps with positive knee constraint)
  trajopt.AddBoundingBoxConstraint(-eps, eps, xf(positions_map.at("base_y")));
  // Final velocity
  trajopt.AddBoundingBoxConstraint(VectorXd::Zero(n_v), VectorXd::Zero(n_v), xf.tail(n_v));
  
  // Nominal stand or z and attitude
  if(use_nominal_stand){
    // nominalSpiritStandConstraint(plant,trajopt,initial_height, {0,trajopt.N()-1}, eps);
    VectorXd standJointsInit = initialStand.getJoints();
    VectorXd standJointsFinal = finalStand.getJoints();
    VectorXd standQuatInit = initialStand.getQuat();
    VectorXd standQuatFinal = finalStand.getQuat();

    // std::cout<<"\nInit:\n"<<standJointsInit<<"\nFinal:\n"<< standJointsFinal<<std::endl;

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



template <class Y>
void SpiritBoxJump<Y>::setUpModeSequence(){
  this->mode_vector.clear();
  this->normal_vector.clear();
  this->offset_vector.clear();
  this->minT_vector.clear();
  this->maxT_vector.clear();       
  //                          mode name   normal                  world offset            minT  maxT
  this->addModeToSequenceVector("stance",initialStand.normal(),initialStand.offset(),     0,    std::numeric_limits<double>::infinity());
  this->addModeToSequenceVector("flight",Eigen::Vector3d::UnitZ(),Eigen::Vector3d::Zero(),0,    std::numeric_limits<double>::infinity());
  this->addModeToSequenceVector("flight",Eigen::Vector3d::UnitZ(),Eigen::Vector3d::Zero(),0,    std::numeric_limits<double>::infinity());
  this->addModeToSequenceVector("stance",finalStand.normal(),    finalStand.offset(),     0,    std::numeric_limits<double>::infinity());
}


/// runSpiritBoxJump, runs a trajectory optimization problem for spirit jumping on flat ground
/// \param plant: robot model
/// \param pp_xtraj: trajectory passed by pointer for spirit animation
template <class Y>
void SpiritBoxJump<Y>::run(MultibodyPlant<Y>& plant,
                          PiecewisePolynomial<Y>* pp_xtraj,
                          std::vector<SurfaceConf>* surface_vector) {

  // Setup mode sequence
  auto sequence = DirconModeSequence<Y>(plant);
  setUpModeSequence();
  auto [modeVector, toeEvals, toeEvalSets] = getModeSequence(plant, sequence);

  ///Setup trajectory optimization
  auto trajopt = Dircon<Y>(sequence);

  // Set up Trajectory Optimization options
  this->ipopt=false; //using snopt solver
  this->setSolver(trajopt);
  // Setting up cost
  addCost(plant, trajopt);
  // Setting up constraints
  addConstraints(plant, trajopt);

  // Initialize the trajectory control state and forces
  this->initTrajectory(plant,trajopt,sequence);


  /// Setup the visualization during the optimization
  this->setupVisualization(trajopt,sequence);
  
  /// Run the optimization using your initial guess
  auto start = std::chrono::high_resolution_clock::now();
  auto result = Solve(trajopt, trajopt.initial_guess());
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  std::cout << "Solve time:" << elapsed.count() <<std::endl;
  std::cout << "Cost:" << result.get_optimal_cost() <<std::endl;
  std::cout << (result.is_success() ? "Optimization Success" : "Optimization Fail") << std::endl;

  /// Save trajectory
  this->saveTrajectory(plant,trajopt,result);

  struct SurfaceConf new_surface={finalStand.normal(),finalStand.offset(),0.5,0.5,0.1};
  if (this->get_animate_info) surface_vector->push_back(new_surface);
  *pp_xtraj =trajopt.ReconstructStateTrajectory(result);
}

template class SpiritBoxJump<double>;
}   // namespace dairlib