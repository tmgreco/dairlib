#include "examples/Spirit/spirit_parkour.h"
#include "examples/Spirit/surface_conf.h"
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
SpiritParkourJump<Y>::SpiritParkourJump():plant (std::make_unique<MultibodyPlant<Y>>(0.0))
{
    Parser parser(plant.get());
    std::string full_name =
        dairlib::FindResourceOrThrow("examples/Spirit/spirit_drake.urdf");
    parser.AddModelFromFile(full_name);
    plant->mutable_gravity_field().set_gravity_vector(-9.81 *
        Eigen::Vector3d::UnitZ());
    plant->Finalize();
    initialStand.init(plant.get(), 0.2, Eigen::Vector3d::UnitZ(), Eigen::Vector3d::Zero(),false);
    finalStand.init(plant.get(), 0.2, Eigen::Vector3d::UnitZ(), Eigen::Vector3d::Zero(),false);
}

template <class Y>
void SpiritParkourJump<Y>::config(
  std::string yaml_path, std::string saved_directory, int index,MultibodyPlant<Y>* plant)
  {
  YAML::Node config = YAML::LoadFile(yaml_path);
  this->nKnotpoints_flight =config[index]["nKnotpoints_flight"].as<double>();
  this->nKnotpoints_stances =config[index]["nKnotpoints_stances"].as<double>();
  this->apex_height =config[index]["apex_height"].as<double>();
  this->max_pitch_magnitude =config[index]["max_pitch_magnitude"].as<double>();
  this->max_apex_pitch_magnitude =config[index]["max_apex_pitch_magnitude"].as<double>();
  this->lock_rotation =config[index]["lock_rotation"].as<bool>();
  this->lock_legs_apex =config[index]["lock_legs_apex"].as<bool>();
  this->force_symmetry =config[index]["force_symmetry"].as<bool>();
  this->max_duration =config[index]["max_duration"].as<double>();
  this->cost_actuation =config[index]["cost_actuation"].as<double>();
  this->cost_velocity =config[index]["cost_velocity"].as<double>();
  this->cost_actuation_legs_flight =config[index]["cost_actuation_legs_flight"].as<double>();
  this->cost_velocity_legs_flight =config[index]["cost_velocity_legs_flight"].as<double>();
  this->cost_work =config[index]["cost_work"].as<double>();
  this->mu =config[index]["mu"].as<double>();
  this->eps =config[index]["eps"].as<double>();
  this->tol =config[index]["tol"].as<double>();
  this->work_constraint_scale =config[index]["work_constraint_scale"].as<double>();
  this->animate=config[index]["animate"].as<bool>();
  this->nJumps=config[index]["n_jumps"].as<int>();
  this->stand_height=config[index]["stand_height"].as<double>();
  if(!config[index]["file_name_out"].as<std::string>().empty()) this->file_name_out=saved_directory+config[index]["file_name_out"].as<std::string>();
  if(!config[index]["file_name_in"].as<std::string>().empty()) this->file_name_in= saved_directory+config[index]["file_name_in"].as<std::string>();
  

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
  this->initial_height=this->initialStand.height();

  this->transitionSurfaces={std::make_tuple(config[index]["transition_surface"][0].as<double>()*Eigen::Vector3d::UnitX()+
                                              config[index]["transition_surface"][1].as<double>()*Eigen::Vector3d::UnitY()+
                                              config[index]["transition_surface"][2].as<double>()*Eigen::Vector3d::UnitZ(),
                                            config[index]["transition_surface"][3].as<double>()*Eigen::Vector3d::UnitX()+
                                              config[index]["transition_surface"][4].as<double>()*Eigen::Vector3d::UnitY()+
                                              config[index]["transition_surface"][5].as<double>()*Eigen::Vector3d::UnitZ(),
                                            std::numeric_limits<double>::infinity())};
  
  
}

/// Adds an offset constraint which allows the body to be anywhere on the normal vector
template <typename Y>
void SpiritParkourJump<Y>::offsetConstraint(
        MultibodyPlant<Y>& plant,
        dairlib::systems::trajectory_optimization::Dircon<Y>& trajopt,
        const Eigen::Ref<const drake::solvers::VectorXDecisionVariable>& xb,
        Eigen::Vector3d normal,
        Eigen::Vector3d offset,
        double eps2
        ){
  // Get position and velocity dictionaries
  auto positions_map = multibody::makeNameToPositionsMap(plant);
  auto velocities_map = multibody::makeNameToVelocitiesMap(plant);
  Eigen::Vector3d nHat= normal/normal.norm();
  Eigen::Matrix3d rotMat = dairlib::normal2Rotation(nHat).matrix();
  std::cout << "Rot:\n" << rotMat << "\nNormal:\n" << nHat <<  "\nOffset:\n" << offset << std::endl;
  Eigen::Vector3d t1 = rotMat.col(0);
  Eigen::Vector3d t2 = rotMat.col(1);
  Eigen::Vector3d t3 = rotMat.col(2);
  std::cout << "T1" << t1 <<std::endl;
  std::cout << "T2" << t2 <<std::endl;
  std::vector<int> posInds{positions_map.at("base_x"),positions_map.at("base_y"),positions_map.at("base_z")};
  double t1Toffset = t1.transpose()*offset;
  double t2Toffset = t2.transpose()*offset;
  double t3Toffset = t3.transpose()*offset;
  std::cout << "pos1: " <<  posInds[0] <<std::endl;
  std::cout << "pos2: " <<  posInds[1] <<std::endl;
  std::cout << "pos3: " <<  posInds[2] <<std::endl;
  double upper1 = t1Toffset + eps2;
  double lower1 = t1Toffset - eps2;
  std::cout << "t1 offset +: " <<  upper1 << std::endl;
  std::cout << "t1 offset -: " <<  lower1 << std::endl;
  
  
  
  trajopt.AddLinearConstraint( t1(0) * xb(posInds[0]) + t1(1) * xb(posInds[1]) + t1(2) * xb(posInds[2])  <=  (t1Toffset));
  trajopt.AddLinearConstraint( t1(0) * xb(posInds[0]) + t1(1) * xb(posInds[1]) + t1(2) * xb(posInds[2])  >=  (t1Toffset));
  // trajopt.AddLinearConstraint( t2(0) * (xb(posInds[0])-offset(0)) + t2(1) * (xb(posInds[1])-offset(1)) + t2(2) * (xb(posInds[2])-offset(2))  <=  eps);
  // trajopt.AddLinearConstraint( t2(0) * (xb(posInds[0])-offset(0)) + t2(1) * (xb(posInds[1])-offset(1)) + t2(2) * (xb(posInds[2])-offset(2))  >= -eps);
  // trajopt.AddLinearConstraint( t3(0) * (xb(posInds[0])-offset(0)) + t3(1) * (xb(posInds[1])-offset(1)) + t3(2) * (xb(posInds[2])-offset(2))  <=  0.3);
  // trajopt.AddLinearConstraint( t3(0) * (xb(posInds[0])-offset(0)) + t3(1) * (xb(posInds[1])-offset(1)) + t3(2) * (xb(posInds[2])-offset(2))  >= 0.15);

}

template <class Y>
void SpiritParkourJump<Y>::generateInitialGuess(MultibodyPlant<Y>& plant){
  // Generate initial guess from two from inplace bound
  std::string file_name_in = "/home/feng/Downloads/dairlib/examples/Spirit/saved_trajectories/bound_test1/in_place_bound";
  // std::string file_name_in = "/home/feng/Downloads/dairlib/examples/Spirit/saved_trajectories/jump_test1/simple_jump";
  dairlib::DirconTrajectory loaded_traj(file_name_in);

  this->x_traj = loaded_traj.ReconstructStateTrajectory();
  this->u_traj = loaded_traj.ReconstructInputTrajectory();
  this->l_traj = loaded_traj.ReconstructLambdaTrajectory();
  this->lc_traj = loaded_traj.ReconstructLambdaCTrajectory();
  this->vc_traj = loaded_traj.ReconstructGammaCTrajectory();
  
  for(int iJump = 1; iJump < nJumps; iJump++){
    PiecewisePolynomial<double> x_traj_i = loaded_traj.ReconstructStateTrajectory();
    PiecewisePolynomial<double> u_traj_i = loaded_traj.ReconstructInputTrajectory();
    vector<PiecewisePolynomial<double>> l_traj_i = loaded_traj.ReconstructLambdaTrajectory();
    vector<PiecewisePolynomial<double>> lc_traj_i = loaded_traj.ReconstructLambdaCTrajectory();
    vector<PiecewisePolynomial<double>> vc_traj_i = loaded_traj.ReconstructGammaCTrajectory();
    
    std::cout<<"test"<< std::endl;
    x_traj_i.shiftRight(this->x_traj.end_time());
    u_traj_i.shiftRight(this->u_traj.end_time());
    std::cout<<"test"<< std::endl;

    this->x_traj.ConcatenateInTime(x_traj_i);
    this->u_traj.ConcatenateInTime(u_traj_i);
    std::cout<<"test"<< std::endl;

    concatVectorTraj(  this->l_traj,  l_traj_i , true);
    concatVectorTraj( this->lc_traj, lc_traj_i , true);
    concatVectorTraj( this->vc_traj, vc_traj_i , true);
  }
  // Run animation of the final trajectory
  std::string full_name = dairlib::FindResourceOrThrow("examples/Spirit/spirit_drake.urdf");
  drake::systems::DiagramBuilder<double> builder;
  auto plant_vis = std::make_unique<MultibodyPlant<double>>(0.0);
  auto scene_graph_ptr = std::make_unique<SceneGraph<double>>();
  Parser parser_vis(plant_vis.get(), scene_graph_ptr.get());
  parser_vis.AddModelFromFile(full_name);
  // Add the transional surfaces
  int counter = 0;
  const drake::Vector4<double> orange(1.0, 0.55, 0.0, 1.0);
  
  plant_vis->Finalize();
  SceneGraph<double>& scene_graph = *builder.AddSystem(std::move(scene_graph_ptr));

  multibody::connectTrajectoryVisualizer(plant_vis.get(),
      &builder, &scene_graph, this->x_traj);
  auto diagram = builder.Build();
  std::cout << "animating 1 times." << std::endl;
  for (int i = 0; i <1; i++ ) {
    drake::systems::Simulator<double> simulator(*diagram);
    simulator.set_target_realtime_rate(0.25);
    simulator.Initialize();
    simulator.AdvanceTo(this->x_traj.end_time());
    sleep(2);
  }
}

template <class Y>
void SpiritParkourJump<Y>::concatVectorTraj(
      vector<PiecewisePolynomial<Y>>& traj, 
      vector<PiecewisePolynomial<Y>> otherTraj, 
      bool mergeInnerMode){
        

  double finalTime = traj.back().end_time();
  double startTime = otherTraj.front().start_time();
    std::cout<<"***  N:"<< otherTraj.size() << "**F:" <<finalTime<<" **S: "<<startTime<<std::endl;
  for (auto& otherTrajI : otherTraj){
    std::cout<<otherTrajI.start_time()<<std::endl;
    otherTrajI.shiftRight(finalTime-startTime);
    if(mergeInnerMode){
      mergeInnerMode = false;
      PiecewisePolynomial<double> lastMode = traj.back();
      traj.pop_back();
      lastMode.ConcatenateInTime(otherTrajI);
      traj.push_back(lastMode);
    }else{
      traj.push_back(otherTrajI);
      // std::cout<<finalTime<<std::endl;
    }
  }
}
  
template <typename Y>
void SpiritParkourJump<Y>::addCostLegs(MultibodyPlant<Y>& plant,
                 dairlib::systems::trajectory_optimization::Dircon<Y>& trajopt,
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

template <class Y>
void SpiritParkourJump<Y>::addCost(
            MultibodyPlant<Y>& plant,
            dairlib::systems::trajectory_optimization::Dircon<Y>& trajopt){
  auto u   = trajopt.input();
  auto x   = trajopt.state();

  // Setup the traditional cost function
  trajopt.AddRunningCost( u.transpose()*this->cost_actuation*u);
  trajopt.AddVelocityCost(this->cost_velocity);

  trajopt.AddRunningCost(1000);

  // // Hard code which joints are in flight for which mode //FIXME using quad_mod_seq
  addCostLegs(plant, trajopt, cost_velocity_legs_flight, cost_actuation_legs_flight, {0, 1, 4, 5, 8, 10}, 1);
  addCostLegs(plant, trajopt, cost_velocity_legs_flight, cost_actuation_legs_flight, {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}, 2);
  addCostLegs(plant, trajopt, cost_velocity_legs_flight, cost_actuation_legs_flight, {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}, 3);
  addCostLegs(plant, trajopt, cost_velocity_legs_flight, cost_actuation_legs_flight, {2, 3, 6, 7, 10, 11}, 4);
  addCostLegs(plant, trajopt, cost_velocity_legs_flight, cost_actuation_legs_flight, {0, 1, 4, 5, 8, 10}, 6);
  addCostLegs(plant, trajopt, cost_velocity_legs_flight, cost_actuation_legs_flight, {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}, 7);
  addCostLegs(plant, trajopt, cost_velocity_legs_flight, cost_actuation_legs_flight, {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}, 8);
  addCostLegs(plant, trajopt, cost_velocity_legs_flight, cost_actuation_legs_flight, {2, 3, 6, 7, 10, 11}, 9);
}

// addConstraints, adds constraints to the trajopt jump problem. See runSpiritParkourJump for a description of the inputs
/// \param plant: robot model
/// \param trajopt: trajectory to be optimized
template <class Y>
void SpiritParkourJump<Y>::addConstraints(
                    MultibodyPlant<Y>& plant,
                    dairlib::systems::trajectory_optimization::Dircon<Y>& trajopt){
  // Get position and velocity dictionaries
  auto positions_map = multibody::makeNameToPositionsMap(plant);
  auto velocities_map = multibody::makeNameToVelocitiesMap(plant);

  setSpiritJointLimits(plant, trajopt);
  setSpiritActuationLimits(plant, trajopt);
  int numJumps = transitionSurfaces.size() + 1;
  if (force_symmetry) {
    setSpiritSymmetry(plant, trajopt);
  }
  /// Setup all the optimization constraints
  int n_q = plant.num_positions();
  int n_v = plant.num_velocities();

  // States variables for the first two modes
  auto   x0  = trajopt.initial_state();
  auto xlo_front_1  = trajopt.state_vars(1,0);
  auto  xlo_rear_1  = trajopt.state_vars(2,0);
  // States variables for the last two modes
  auto xtd_front_2  = trajopt.state_vars(numJumps*5-1,0);
  auto  xtd_rear_2  = trajopt.state_vars(numJumps*5,0);
  auto   xf  = trajopt.final_state();

  // Add duration constraint, currently constrained not bounded
  trajopt.AddDurationBounds(0, max_duration);


  /// ***** Constrain the initial and final states. *****  
  // Constraining configuration  
  VectorXd standOffsetInit = initialStand.offset();
  VectorXd standOffsetFinal = finalStand.offset();
  VectorXd standJointsInit = initialStand.getJoints();
  VectorXd standJointsFinal = finalStand.getJoints();
  VectorXd standQuatInit = initialStand.getQuat();
  VectorXd standQuatFinal = finalStand.getQuat();
  // int numJoints = standJointsInit.size();
  // Init Standing XY Pos
  trajopt.AddBoundingBoxConstraint(standOffsetInit(0), standOffsetInit(0), x0(positions_map.at("base_x"))); 
  trajopt.AddBoundingBoxConstraint(standOffsetInit(1)-eps, standOffsetInit(1)+eps, x0(positions_map.at("base_y"))); 
  // Nominal stand
  nominalSpiritStandConstraint(plant,trajopt,initialStand.height(), {0}, eps);

  // Body pose constraints (keep the body flat) at initial state
  trajopt.AddBoundingBoxConstraint(1, 1,  x0(positions_map.at("base_qw")));
  trajopt.AddBoundingBoxConstraint(0, 0,  x0(positions_map.at("base_qx")));
  trajopt.AddBoundingBoxConstraint(0, 0,  x0(positions_map.at("base_qy")));
  trajopt.AddBoundingBoxConstraint(0, 0,  x0(positions_map.at("base_qz")));

  // Initial  velocity
  trajopt.AddBoundingBoxConstraint(VectorXd::Zero(n_v), VectorXd::Zero(n_v), x0.tail(n_v));

  // Final Standing XY Pos 
  std::cout<<"FINAL Stand OFFSET: "<<standOffsetFinal(0)<<", "<<standOffsetFinal(1)<<std::endl;
  trajopt.AddBoundingBoxConstraint(standOffsetFinal(0)-eps, standOffsetFinal(0)+eps, xf(positions_map.at("base_x")));
  trajopt.AddBoundingBoxConstraint(standOffsetFinal(1)-eps, standOffsetFinal(1)+eps, xf(positions_map.at("base_y")));
  // Nominal stand
  nominalSpiritStandConstraint(plant,trajopt,finalStand.height(), {trajopt.N()-1}, eps);
  // Body pose constraints (keep the body flat) at final state
  trajopt.AddBoundingBoxConstraint(1, 1, xf(positions_map.at("base_qw")));
  trajopt.AddBoundingBoxConstraint(-eps, eps, xf(positions_map.at("base_qx")));
  trajopt.AddBoundingBoxConstraint(-eps, eps, xf(positions_map.at("base_qy")));
  trajopt.AddBoundingBoxConstraint(-eps, eps, xf(positions_map.at("base_qz")));
  // Zero velocity
  trajopt.AddBoundingBoxConstraint(VectorXd::Zero(n_v), VectorXd::Zero(n_v), xf.tail(n_v));
 
  //  *********************initial lift off and final touch down *******************************
  trajopt.AddBoundingBoxConstraint( cos(max_pitch_magnitude/2.0),              1, xlo_rear_1(positions_map.at("base_qw")));
  trajopt.AddBoundingBoxConstraint(           -eps,            eps, xlo_rear_1(positions_map.at("base_qx")));
  trajopt.AddBoundingBoxConstraint(-sin(max_pitch_magnitude/2.0), sin(max_pitch_magnitude/2.0), xlo_rear_1(positions_map.at("base_qy")));
  trajopt.AddBoundingBoxConstraint(           -eps,            eps, xlo_rear_1(positions_map.at("base_qz")));

  // trajopt.AddBoundingBoxConstraint( cos(max_pitch_magnitude/2.0),              1, xlo_front_1(positions_map.at("base_qw")));
  // trajopt.AddBoundingBoxConstraint(           -eps,            eps, xlo_front_1(positions_map.at("base_qx")));
  // trajopt.AddBoundingBoxConstraint(-sin(max_pitch_magnitude/2.0), sin(max_pitch_magnitude/2.0), xlo_front_1(positions_map.at("base_qy")));
  // trajopt.AddBoundingBoxConstraint(           -eps,            eps, xlo_front_1(positions_map.at("base_qz")));

  // trajopt.AddBoundingBoxConstraint( cos(max_pitch_magnitude/2.0),              1, xtd_front_2(positions_map.at("base_qw")));
  // trajopt.AddBoundingBoxConstraint(           -eps,            eps, xtd_front_2(positions_map.at("base_qx")));
  // trajopt.AddBoundingBoxConstraint(-sin(max_pitch_magnitude/2.0), sin(max_pitch_magnitude/2.0), xtd_front_2(positions_map.at("base_qy")));
  // trajopt.AddBoundingBoxConstraint(           -eps,            eps, xtd_front_2(positions_map.at("base_qz")));
  trajopt.AddBoundingBoxConstraint( cos(max_pitch_magnitude/2.0),              1, xtd_front_2(positions_map.at("base_qw")));
  trajopt.AddBoundingBoxConstraint(           -eps,            eps, xtd_front_2(positions_map.at("base_qx")));
  trajopt.AddBoundingBoxConstraint(0, sin(max_pitch_magnitude/2.0), xtd_front_2(positions_map.at("base_qy")));
  trajopt.AddBoundingBoxConstraint(           -eps,            eps, xtd_front_2(positions_map.at("base_qz")));

  // trajopt.AddBoundingBoxConstraint( cos(max_pitch_magnitude/2.0),              1, xtd_rear_2(positions_map.at("base_qw")));
  // trajopt.AddBoundingBoxConstraint(           -eps,            eps, xtd_rear_2(positions_map.at("base_qx")));
  // trajopt.AddBoundingBoxConstraint(-sin(max_pitch_magnitude/2.0), sin(max_pitch_magnitude/2.0), xtd_rear_2(positions_map.at("base_qy")));
  // trajopt.AddBoundingBoxConstraint(           -eps,            eps, xtd_rear_2(positions_map.at("base_qz")));

  //  ****************************************************
  for (int iJump=0;iJump<numJumps;iJump++){
    // Apex height
    if(apex_height > 0){
      auto xapex = trajopt.state_vars(3 + iJump*5 , 0);
      trajopt.AddBoundingBoxConstraint(apex_height - eps, apex_height + eps, xapex(positions_map.at("base_z")) );
      // Apex max_pitch_magnitude constraints
      trajopt.AddBoundingBoxConstraint( cos(max_apex_pitch_magnitude/2.0),              1, xapex(positions_map.at("base_qw")));
      trajopt.AddBoundingBoxConstraint(           -eps,            eps, xapex(positions_map.at("base_qx")));
      trajopt.AddBoundingBoxConstraint(-sin(max_apex_pitch_magnitude/2.0), sin(max_apex_pitch_magnitude/2.0), xapex(positions_map.at("base_qy")));
      trajopt.AddBoundingBoxConstraint(           -eps,            eps, xapex(positions_map.at("base_qz")));

      trajopt.AddBoundingBoxConstraint(- eps, + eps, xapex(n_q + velocities_map.at("base_vy")) );
      trajopt.AddBoundingBoxConstraint(- eps, + eps, xapex(n_q + velocities_map.at("base_vz")) );

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

    }
    if(iJump != 0){
      std::cout << "Zero Velocity Bottoms activated" << std::endl;
      for (auto const &pair: positions_map) {
        std::cout << "{" << pair.first << ": " << pair.second << "}\n";
      }
      int interiorStanceModeIndex = iJump*5;
      int botKnotpoint = trajopt.mode_length(interiorStanceModeIndex)/2;
      auto xbot = trajopt.state_vars( interiorStanceModeIndex , botKnotpoint);
      Eigen::Vector3d sNormal;
      Eigen::Vector3d sOffset;
      auto xtd_front_s = trajopt.state_vars(interiorStanceModeIndex-1, 0);
      auto xtd_rear_s = trajopt.state_vars(interiorStanceModeIndex, 0);
      auto xlo_front_s = trajopt.state_vars(interiorStanceModeIndex+1, 0);
      auto xlo_rear_s = trajopt.state_vars(interiorStanceModeIndex+2, 0);
      std::tie(sNormal,sOffset,std::ignore) = transitionSurfaces[iJump-1];

      this->offsetConstraint(plant, trajopt, xbot, sNormal,sOffset);

      trajopt.AddBoundingBoxConstraint(- eps, + eps, xbot(n_q + velocities_map.at("base_vz")) );
      // trajopt.AddBoundingBoxConstraint(0.2, 0.25, xbot(positions_map.at("base_z")) );
      //  front legs touch-down max_pitch_magnitude angle
      trajopt.AddBoundingBoxConstraint( cos(max_pitch_magnitude/2.0),              1, xtd_front_s(positions_map.at("base_qw")));
      trajopt.AddBoundingBoxConstraint(           -eps,            eps, xtd_front_s(positions_map.at("base_qx")));
      trajopt.AddBoundingBoxConstraint(-sin(max_pitch_magnitude/2.0), sin(max_pitch_magnitude/2.0), xtd_front_s(positions_map.at("base_qy")));
      trajopt.AddBoundingBoxConstraint(           -eps,            eps, xtd_front_s(positions_map.at("base_qz")));

      //  rear legs touch-down max_pitch_magnitude angle
      trajopt.AddBoundingBoxConstraint( cos(max_apex_pitch_magnitude/2/2.0),              1, xtd_rear_s(positions_map.at("base_qw")));
      trajopt.AddBoundingBoxConstraint(           -eps,            eps, xtd_rear_s(positions_map.at("base_qx")));
      trajopt.AddBoundingBoxConstraint(-sin(max_apex_pitch_magnitude/2/2.0), sin(max_apex_pitch_magnitude/2/2.0), xtd_rear_s(positions_map.at("base_qy")));
      trajopt.AddBoundingBoxConstraint(           -eps,            eps, xtd_rear_s(positions_map.at("base_qz")));
      
      //  rear legs lift-off max_pitch_magnitude angle
      trajopt.AddBoundingBoxConstraint( cos(max_pitch_magnitude/2.0),              1, xlo_rear_s(positions_map.at("base_qw")));
      trajopt.AddBoundingBoxConstraint(           -eps,            eps, xlo_rear_s(positions_map.at("base_qx")));
      trajopt.AddBoundingBoxConstraint(-sin(max_pitch_magnitude/2.0), sin(max_pitch_magnitude/2.0), xlo_rear_s(positions_map.at("base_qy")));
      trajopt.AddBoundingBoxConstraint(           -eps,            eps, xlo_rear_s(positions_map.at("base_qz")));
    }

  }
  // The "leg angle" = (pi/2 - theta0)

  
  /// Constraints on all points
  for (int i = 0; i < trajopt.N(); i++){
    auto xi = trajopt.state(i);
    // Height
    trajopt.AddBoundingBoxConstraint( 0.15, 2, xi( positions_map.at("base_z")));
    trajopt.AddBoundingBoxConstraint( -eps, eps, xi( n_q+velocities_map.at("base_vy")));
  }
 
}

/// runSpiritParkourJump, runs a trajectory optimization problem for spirit jumping on flat ground
/// \param plant: robot model
/// \param pp_xtraj: trajectory passed by pointer for spirit animation
template <class Y>
void SpiritParkourJump<Y>::run(MultibodyPlant<Y>& plant,
                          PiecewisePolynomial<Y>* pp_xtraj,
                          std::vector<SurfaceConf>* surface_vector) {

  
  // Setup mode sequence
  auto sequence = DirconModeSequence<Y>(plant);

  std::vector<std::string> mode_vector{"stance","flight","flight","stance"};
  std::vector<double> minT_vector{0,0,0,0};
  double inf=std::numeric_limits<double>::infinity();
  std::vector<double> maxT_vector{inf,inf,inf,inf};
  auto [modeVector, toeEvals, toeEvalSets] = this->getModeSequence(plant, sequence,mode_vector,minT_vector,maxT_vector);


  auto trajopt = Dircon<Y>(sequence);
  // Set up Trajectory Optimization options
  // trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(),
  //                          "Print file", "../snopt.out");
  // trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(),
  //                          "Major iterations limit", 200000);
  // trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(), "Iterations limit", 200000);
  // trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(),
  //                          "Major optimality tolerance",
  //                          this->tol);  // target optimality
  // trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(), "Major feasibility tolerance", this->tol);
  // trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(), "Verify level",
  //                          0);  // 0

  // Ipopt settings adapted from CaSaDi and FROST
  auto id = drake::solvers::IpoptSolver::id();
  trajopt.SetSolverOption(id, "tol", this->tol);
  trajopt.SetSolverOption(id, "dual_inf_tol", this->tol);
  trajopt.SetSolverOption(id, "constr_viol_tol", this->tol);
  trajopt.SetSolverOption(id, "compl_inf_tol", this->tol);
  trajopt.SetSolverOption(id, "max_iter", 10000);
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

  drake::solvers::SolverId solver_id("");
  solver_id = drake::solvers::IpoptSolver().id();
  std::cout << "\nChose manually: " << solver_id.name() << std::endl;
  
  // Setting up cost

  addCost(plant, trajopt);
  // Setting up constraints
  bool stop_at_bottom = true;

  addConstraints(plant, trajopt);

  // Initialize the trajectory control state and forces
  if (this->file_name_in.empty()){
  //  for (int j = 0; j < sequence.num_modes(); j++) {
  //     trajopt.SetInitialTrajectoryForMode(j, this->x_traj[j], this->u_traj, this->x_traj[j].start_time(), this->x_traj[j].end_time());
  //     trajopt.SetInitialForceTrajectory(j, this->l_traj[j], this->lc_traj[j],
  //                                       this->vc_traj[j], this->l_traj[j].start_time());
  //   }
    trajopt.drake::systems::trajectory_optimization::MultipleShooting::
        SetInitialTrajectory(this->u_traj, this->x_traj);
    for (int j = 0; j < sequence.num_modes(); j++) {
      trajopt.SetInitialForceTrajectory(j, this->l_traj[j], this->lc_traj[j], this->vc_traj[j], this->l_traj[j].start_time());
    }
  }else{
    std::cout<<"Loading decision var from file." <<std::endl;
    dairlib::DirconTrajectory loaded_traj(this->file_name_in);
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
  auto solver = drake::solvers::MakeSolver(solver_id);
  drake::solvers::MathematicalProgramResult result;
  solver->Solve(trajopt, trajopt.initial_guess(), trajopt.solver_options(),
                &result);  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  std::cout << "Solve time: " << elapsed.count() <<std::endl;
  std::cout << "Cost: " << result.get_optimal_cost() <<std::endl;
  std::cout << (result.is_success() ? "Optimization Success" : "Optimization Fail") << std::endl;

  /// Save trajectory
  std::cout << "Outputting trajectories" << std::endl;


  this->saveTrajectory(plant,trajopt,result);

  const drake::Vector4<double> orange(1.0, 0.55, 0.0, 1.0);
  struct SurfaceConf new_surface={finalStand.normal(),finalStand.offset(),0.5,0.5,0.1};
  if (animate) {
    int counter = 0;
    for (auto& surface : transitionSurfaces){
      Eigen::Vector3d sNormal;
      Eigen::Vector3d sOffset;
      std::tie(sNormal,sOffset,std::ignore) = surface;
      counter++;
      struct SurfaceConf new_surface={sNormal,sOffset,0.5,0.5,0.1,orange,"surface"+std::to_string(counter)};
      surface_vector->push_back(new_surface);
    }
  }
  *pp_xtraj =trajopt.ReconstructStateTrajectory(result);
}

template class SpiritParkourJump<double>;
}   // namespace dairlib