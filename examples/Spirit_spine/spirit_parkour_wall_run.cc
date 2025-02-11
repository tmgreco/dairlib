#include "examples/Spirit_spine/spirit_parkour_wall_run.h"
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
SpiritParkourWallRun<Y>::SpiritParkourWallRun():plant (std::make_unique<MultibodyPlant<Y>>(0.0))
{
    
}

template <class Y>
void SpiritParkourWallRun<Y>::config(
  std::string yaml_path, std::string saved_directory, int index,MultibodyPlant<Y>* plant)
  {
  YAML::Node config = YAML::LoadFile(yaml_path);
  this->index=index;
  this->nKnotpoints_flight =config[index]["nKnotpoints_flight"].as<double>();
  this->nKnotpoints_stances =config[index]["nKnotpoints_stances"].as<double>();
  this->apex_heights =config[index]["apex_heights"].as<std::vector<double>>();
  this->max_duration =config[index]["max_duration"].as<double>();
  this->cost_actuation =config[index]["cost_actuation"].as<double>();
  this->cost_velocity =config[index]["cost_velocity"].as<double>();
  this->cost_actuation_legs_flight =config[index]["cost_actuation_legs_flight"].as<double>();
  this->cost_velocity_legs_flight =config[index]["cost_velocity_legs_flight"].as<double>();
  this->cost_time=config[index]["cost_time"].as<double>();
  this->cost_power =config[index]["cost_power"].as<double>();
  this->mu =config[index]["mu"].as<double>();
  this->eps =config[index]["eps"].as<double>();
  this->tol =config[index]["tol"].as<double>();
  this->xtol =config[index]["xtol"].as<double>();
  this->pose_ref =config[index]["pose_ref"].as<bool>();
  this->roll_ref =config[index]["roll_ref"].as<double>();
  this->pitch_ref =config[index]["pitch_ref"].as<double>();
  this->yaw_ref =config[index]["yaw_ref"].as<double>();
  this->distance =config[index]["distance"].as<double>();
  this->time_symmetry=config[index]["time_symmetry"].as<bool>();
  this->nominal_stance=config[index]["nominal_stance"].as<bool>();
  this->lock_legs_apex=config[index]["lock_legs_apex"].as<bool>();
  this->lock_spine=config[index]["lock_spine"].as<bool>();
  if (config[index]["warm_up"]) this->warm_up=config[index]["warm_up"].as<bool>();
  else this->warm_up=false;
  if(!config[index]["file_name_out"].as<std::string>().empty()) this->file_name_out=saved_directory+config[index]["file_name_out"].as<std::string>();
  if(!config[index]["file_name_in"].as<std::string>().empty()) this->file_name_in= saved_directory+config[index]["file_name_in"].as<std::string>();
  
  if(config[index]["action"]) this->action=config[index]["action"].as<std::string>();
  else this->action="";

  // Eigen::Vector3d initial_normal =  config[index]["initial_stand"][0].as<double>() *Eigen::Vector3d::UnitX()+
  //                           config[index]["initial_stand"][1].as<double>() *Eigen::Vector3d::UnitY()+
  //                           config[index]["initial_stand"][2].as<double>()  *Eigen::Vector3d::UnitZ();
  // initial_normal = initial_normal/initial_normal.norm();
  // Eigen::Vector3d initial_offset = config[index]["initial_stand"][3].as<double>() *Eigen::Vector3d::UnitX()+ 
  //                          config[index]["initial_stand"][4].as<double>() *Eigen::Vector3d::UnitY()+
  //                          config[index]["initial_stand"][5].as<double>() *Eigen::Vector3d::UnitZ();

  // Eigen::Vector3d final_normal =  config[index]["final_stand"][0].as<double>() *Eigen::Vector3d::UnitX()+
  //                           config[index]["final_stand"][1].as<double>() *Eigen::Vector3d::UnitY()+
  //                           config[index]["final_stand"][2].as<double>()  *Eigen::Vector3d::UnitZ();
  // final_normal = final_normal/final_normal.norm();
  // Eigen::Vector3d final_offset = config[index]["final_stand"][3].as<double>() *Eigen::Vector3d::UnitX()+ 
  //                          config[index]["final_stand"][4].as<double>() *Eigen::Vector3d::UnitY()+
  //                          config[index]["final_stand"][5].as<double>() *Eigen::Vector3d::UnitZ();
  // this->initialStand.init(plant,config[index]["stand_height"].as<double>(),initial_normal, initial_offset,config[index]["initial_stand"][6].as<bool>());
  // this->finalStand.init(plant, config[index]["stand_height"].as<double>(), final_normal, final_offset, config[index]["final_stand"][6].as<bool>());
  // this->initial_height=this->initialStand.height();

  this->transitionSurfaces.clear();
  if (config[index]["surfaces"]){
    for (std::size_t i=0;i<config[index]["surfaces"].size();i++){
      Eigen::Vector3d surface_normal=config[index]["surfaces"][i][0].as<double>()*Eigen::Vector3d::UnitX()+
                                                config[index]["surfaces"][i][1].as<double>()*Eigen::Vector3d::UnitY()+
                                                config[index]["surfaces"][i][2].as<double>()*Eigen::Vector3d::UnitZ();
      surface_normal = surface_normal/surface_normal.norm();                                          
      this->transitionSurfaces.push_back(std::make_tuple(surface_normal,
                                              config[index]["surfaces"][i][3].as<double>()*Eigen::Vector3d::UnitX()+
                                                config[index]["surfaces"][i][4].as<double>()*Eigen::Vector3d::UnitY()+
                                                config[index]["surfaces"][i][5].as<double>()*Eigen::Vector3d::UnitZ(),
                                              std::numeric_limits<double>::infinity()));
    }
  }
  else{
    double fore_aft_displacement=config[index]["final_stand"][3].as<double>();
    Eigen::Vector3d surface_normal=Eigen::Vector3d::UnitY();
    Eigen::Vector3d surface_offset=fore_aft_displacement*0.5*Eigen::Vector3d::UnitX()+
                                  (fore_aft_displacement*(-0.25)-0.3)*Eigen::Vector3d::UnitY()
                                  +(0.25+fore_aft_displacement*0.2)*Eigen::Vector3d::UnitZ();
    std::cout<<"NORMAL: "<<surface_normal<<"\n OFFSET: "<<surface_offset<<std::endl;
    this->transitionSurfaces.push_back(std::make_tuple(surface_normal,surface_offset,std::numeric_limits<double>::infinity()));
    this->apex_heights.clear();
    this->apex_heights.push_back(0.25+fore_aft_displacement*0.25);
    this->apex_heights.push_back(0.25+fore_aft_displacement*0.25);
  }
  
}

/// Adds an offset constraint which allows the body to be anywhere on the normal vector
template <typename Y>
void SpiritParkourWallRun<Y>::offsetConstraint(
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
  
  
  trajopt.AddLinearConstraint( t1(0) * xb(posInds[0]) + t1(1) * xb(posInds[1]) + t1(2) * xb(posInds[2])  <=  (t1Toffset+eps2));
  trajopt.AddLinearConstraint( t1(0) * xb(posInds[0]) + t1(1) * xb(posInds[1]) + t1(2) * xb(posInds[2])  >=  (t1Toffset-eps2));

  trajopt.AddLinearConstraint( t2(0) * xb(posInds[0]) + t2(1) * xb(posInds[1]) + t2(2) * xb(posInds[2])  <=  (t2Toffset+eps2));
  trajopt.AddLinearConstraint( t2(0) * xb(posInds[0]) + t2(1) * xb(posInds[1]) + t2(2) * xb(posInds[2])  >=  (t2Toffset-eps2));

  trajopt.AddLinearConstraint( t3(0) * xb(posInds[0]) + t3(1) * xb(posInds[1]) + t3(2) * xb(posInds[2])  >=  (t3Toffset+0.15));
  
  // trajopt.AddLinearConstraint( t3(0) * xb(posInds[0]) + t3(1) * xb(posInds[1]) + t3(2) * xb(posInds[2])  <=  (t3Toffset+0.25));
  // trajopt.AddLinearConstraint( t3(0) * xb(posInds[0]) + t3(1) * xb(posInds[1]) + t3(2) * xb(posInds[2])  >=  (t3Toffset+0.2));
  // trajopt.AddLinearConstraint( t2(0) * (xb(posInds[0])-offset(0)) + t2(1) * (xb(posInds[1])-offset(1)) + t2(2) * (xb(posInds[2])-offset(2))  <=  eps);
  // trajopt.AddLinearConstraint( t2(0) * (xb(posInds[0])-offset(0)) + t2(1) * (xb(posInds[1])-offset(1)) + t2(2) * (xb(posInds[2])-offset(2))  >= -eps);
  // trajopt.AddLinearConstraint( t3(0) * (xb(posInds[0])-offset(0)) + t3(1) * (xb(posInds[1])-offset(1)) + t3(2) * (xb(posInds[2])-offset(2))  <=  0.3);
  // trajopt.AddLinearConstraint( t3(0) * (xb(posInds[0])-offset(0)) + t3(1) * (xb(posInds[1])-offset(1)) + t3(2) * (xb(posInds[2])-offset(2))  >= 0.15);
  // trajopt.AddBoundingBoxConstraint( cos(max_pitch_magnitude/2.0),              1, xb(positions_map.at("base_qw")));
  // trajopt.AddBoundingBoxConstraint(           -eps,            eps, xb(positions_map.at("base_qx")));
  // trajopt.AddBoundingBoxConstraint(-sin(max_pitch_magnitude/2.0), sin(max_pitch_magnitude/2.0), xb(positions_map.at("base_qy")));
  // trajopt.AddBoundingBoxConstraint(           -eps,            eps, xb(positions_map.at("base_qz")));

}

template <class Y>
void SpiritParkourWallRun<Y>::generateInitialGuess(MultibodyPlant<Y>& plant){
  // Generate initial guess from two from inplace bound
  std::string file_name_in;
  if (this->spine_type=="twisting") file_name_in= "/home/feng/Downloads/dairlib/examples/Spirit_spine/saved_trajectories/bound_test1/in_place_bound";
  else if (this->spine_type=="rigid") file_name_in = "/home/feng/Downloads/dairlib/examples/Spirit_spine/saved_trajectories/bound_test1_without_spine/in_place_bound";
  // std::string file_name_in = "/home/feng/Downloads/dairlib/examples/Spirit/saved_trajectories/jump_test1/simple_jump";
  dairlib::DirconTrajectory loaded_traj(file_name_in);

  this->x_traj = loaded_traj.ReconstructStateTrajectory();
  this->u_traj = loaded_traj.ReconstructInputTrajectory();
  this->l_traj = loaded_traj.ReconstructLambdaTrajectory();
  this->lc_traj = loaded_traj.ReconstructLambdaCTrajectory();
  this->vc_traj = loaded_traj.ReconstructGammaCTrajectory();
  
  for(int iJump = 1; iJump < 2; iJump++){
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
  std::string full_name = dairlib::FindResourceOrThrow(this->urdf_path);
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
  // std::cout << "animating 1 times." << std::endl;
  // for (int i = 0; i <1; i++ ) {
  //   drake::systems::Simulator<double> simulator(*diagram);
  //   simulator.set_target_realtime_rate(0.25);
  //   simulator.Initialize();
  //   simulator.AdvanceTo(this->x_traj.end_time());
  //   sleep(2);
  // }
}

template <class Y>
void SpiritParkourWallRun<Y>::concatVectorTraj(
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
void SpiritParkourWallRun<Y>::addCostLegs(MultibodyPlant<Y>& plant,
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
void SpiritParkourWallRun<Y>::addCost(
            MultibodyPlant<Y>& plant,
            dairlib::systems::trajectory_optimization::Dircon<Y>& trajopt){
  auto u   = trajopt.input();
  auto x   = trajopt.state();
  int n_q = plant.num_positions();

  auto velocities_map = multibody::makeNameToVelocitiesMap(plant);
  auto actuator_map = multibody::makeNameToActuatorsMap(plant);

  // Setup the traditional cost function
  trajopt.AddRunningCost( u.transpose()*this->cost_actuation*u);
  trajopt.AddVelocityCost(this->cost_velocity);

  trajopt.AddRunningCost(this->cost_time);

  // // Hard code which joints are in flight for which mode //FIXME using quad_mod_seq
  addCostLegs(plant, trajopt, this->cost_velocity_legs_flight, this->cost_actuation_legs_flight, {0, 1, 4, 5, 8, 10}, 1);
  addCostLegs(plant, trajopt, this->cost_velocity_legs_flight, this->cost_actuation_legs_flight, {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}, 2);
  addCostLegs(plant, trajopt, this->cost_velocity_legs_flight, this->cost_actuation_legs_flight, {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}, 3);
  addCostLegs(plant, trajopt, this->cost_velocity_legs_flight, this->cost_actuation_legs_flight, {2, 3, 6, 7, 10, 11}, 4);
  addCostLegs(plant, trajopt, this->cost_velocity_legs_flight, this->cost_actuation_legs_flight, {0, 1, 4, 5, 8, 10}, 6);
  addCostLegs(plant, trajopt, this->cost_velocity_legs_flight, this->cost_actuation_legs_flight, {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}, 7);
  addCostLegs(plant, trajopt, this->cost_velocity_legs_flight, this->cost_actuation_legs_flight, {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11}, 8);
  addCostLegs(plant, trajopt, this->cost_velocity_legs_flight, this->cost_actuation_legs_flight, {2, 3, 6, 7, 10, 11}, 9);

  AddPowerCost(plant, trajopt, this->cost_power);

}

// addConstraints, adds constraints to the trajopt jump problem. See runSpiritParkourWallRun for a description of the inputs
/// \param plant: robot model
/// \param trajopt: trajectory to be optimized
template <class Y>
void SpiritParkourWallRun<Y>::addConstraints(
                    MultibodyPlant<Y>& plant,
                    dairlib::systems::trajectory_optimization::Dircon<Y>& trajopt){
  // Get position and velocity dictionaries
  auto positions_map = multibody::makeNameToPositionsMap(plant);
  auto velocities_map = multibody::makeNameToVelocitiesMap(plant);
  auto actuators_map=multibody::makeNameToActuatorsMap(plant);


  setSpiritJointLimits(plant, trajopt);
  setSpiritActuationLimits(plant, trajopt);
  int numJumps = 2;
  

  int n_q = plant.num_positions();
  int n_v = plant.num_velocities();

  // States variables for the first two modes
  auto   x0  = trajopt.initial_state();
  auto  xlo_1  = trajopt.state_vars(2,0);
  // States variables for the last two modes
  auto xtd_f  = trajopt.state_vars(numJumps*5-1,0);
  auto   xf  = trajopt.final_state();

  // Add duration constraint, currently constrained not bounded
  trajopt.AddDurationBounds(0, max_duration);


  /// ***** Constrain the initial and final states. *****  
  if (time_symmetry) trajopt.AddPeriodicTimeIntervalsConstraints();
  // Nominal stance
  if (nominal_stance) {
    nominalSpiritStandConstraint(plant,trajopt,0.2, {0}, eps);
    nominalSpiritStandConstraint(plant,trajopt,0.2, {trajopt.N()-1}, eps);
    // nominalSpiritStandConstraint(plant,trajopt,0.25, {trajopt.N()/2}, eps);
  }
  // std::cout<<trajopt.N()/2<<"@!@@@"<<std::endl;
  // int numJoints = standJointsInit.size();
  // Init Standing XY Pos
  // trajopt.AddBoundingBoxConstraint(0, 0, x0(positions_map.at("base_x"))); 
  // trajopt.AddBoundingBoxConstraint(-eps, +eps, x0(positions_map.at("base_y"))); 
  Eigen::Vector3d sNormal0;
  Eigen::Vector3d sOffset0;
  std::tie(sNormal0,sOffset0,std::ignore) = transitionSurfaces[0];
  this->offsetConstraint(plant, trajopt, x0, sNormal0,sOffset0,xtol);

  double s1_y=std::get<0>(this->transitionSurfaces[0])[1];
  double s1_z=std::get<0>(this->transitionSurfaces[0])[2];
  double s2_y=std::get<0>(this->transitionSurfaces[1])[1];
  double s2_z=std::get<0>(this->transitionSurfaces[1])[2];
  
  double roll_surface_0=-asin(s1_y/sqrt(pow(s1_y,2)+pow(s1_z,2)));
  double roll_surface_1=-asin(s2_y/sqrt(pow(s2_y,2)+pow(s2_z,2)));
  // std::cout<<"1!!!!!!!!: "<<roll_surface_0<<" "<<roll_surface_1<<std::endl;
  // Body pose constraints (keep the body flat) at initial state
  auto t0_0=2.0 * (x0(0) * x0(3) - x0(1)* x0(2));
  auto t0_1=1 - 2* (x0(2) * x0(2) + x0(3)* x0(3));
  auto yaw_0 = atan2(t0_0, t0_1);
  auto roll_0 = atan2(2.0 * (x0(0) * x0(1) - x0(2) * x0(3)), 1.0 - 2.0 * (x0(1) * x0(1) + x0(2) * x0(2)));
  auto pitch_0 = asin(2.0 * (x0(0) * x0(2) + x0(3) * x0(1)));
  if (pose_ref){
    trajopt.AddConstraint(yaw_0,-yaw_ref,eps);
    trajopt.AddConstraint(roll_0,roll_surface_0-roll_ref,roll_surface_0+roll_ref/2);
    trajopt.AddConstraint(pitch_0,-pitch_ref/3,pitch_ref/3);
  }

  // Final Standing X Pos 
  trajopt.AddBoundingBoxConstraint(this->distance-eps, this->distance+eps, xf(positions_map.at("base_x")));
 

  ///constraint the first lift-off roll yaw pitch
  auto tlo1_0=2.0 * (xlo_1(0) * xlo_1(3) - xlo_1(1)* xlo_1(2));
  auto tlo1_1=1 - 2* (xlo_1(2) * xlo_1(2) + xlo_1(3)* xlo_1(3));
  auto yaw_lo1 = atan2(tlo1_0, tlo1_1);
  auto roll_lo1 = atan2(2.0 * (xlo_1(0) * xlo_1(1) - xlo_1(2) * xlo_1(3)), 1.0 - 2.0 * (xlo_1(1) * xlo_1(1) + xlo_1(2) * xlo_1(2)));
  auto pitch_lo1 = asin(2.0 * (xlo_1(0) * xlo_1(2) + xlo_1(3) * xlo_1(1)));
  if (pose_ref){
    trajopt.AddConstraint(yaw_lo1,-yaw_ref, yaw_ref/2);
    trajopt.AddConstraint(roll_lo1,roll_surface_0-roll_ref,roll_surface_0+roll_ref);
    trajopt.AddConstraint(pitch_lo1,-pitch_ref,eps);
  }
  ///constraint final touch-down roll yaw pitch
  auto ttd_f_0=2.0 * (xtd_f(0) * xtd_f(3) - xtd_f(1)* xtd_f(2));
  auto ttd_f_1=1 - 2* (xtd_f(2) * xtd_f(2) + xtd_f(3)* xtd_f(3));
  auto yaw_tdf = atan2(ttd_f_0, ttd_f_1);
  auto roll_tdf = atan2(2.0 * (xtd_f(0) * xtd_f(1) - xtd_f(2) * xtd_f(3)), 1.0 - 2.0 * (xtd_f(1) * xtd_f(1) + xtd_f(2) * xtd_f(2)));
  auto pitch_tdf = asin(2.0 * (xtd_f(0) * xtd_f(2) + xtd_f(3) * xtd_f(1)));
  if (pose_ref){
    trajopt.AddConstraint(yaw_tdf,-yaw_ref/2, yaw_ref);
    trajopt.AddConstraint(roll_tdf,roll_surface_0-roll_ref,roll_surface_0+roll_ref);
    trajopt.AddConstraint(pitch_tdf,-eps,pitch_ref*0.8);
  }

  

  //  ****************************************************
  auto xapex1 = trajopt.state_vars(3 , 0);
  trajopt.AddBoundingBoxConstraint(apex_heights[0] - eps, apex_heights[0] + eps, xapex1(positions_map.at("base_z")) );
  trajopt.AddBoundingBoxConstraint(- eps, eps, xapex1(n_q + velocities_map.at("base_vz")) );
  auto xapex2 = trajopt.state_vars(8 , 0);
  trajopt.AddBoundingBoxConstraint(apex_heights[1] - eps, apex_heights[1] + eps, xapex2(positions_map.at("base_z")) );
  trajopt.AddBoundingBoxConstraint(- eps, eps, xapex2(n_q + velocities_map.at("base_vz")) );

  if(lock_legs_apex){
    //STATIC LEGS AT APEX
    double upperSet = 1;
    double kneeSet = 2;
    trajopt.AddBoundingBoxConstraint(upperSet - eps, upperSet + eps, xapex1(positions_map.at("joint_0") ) );
    trajopt.AddBoundingBoxConstraint(kneeSet - eps, kneeSet + eps, xapex1(positions_map.at("joint_1") ) );

    trajopt.AddBoundingBoxConstraint(upperSet - eps, upperSet + eps, xapex1(positions_map.at("joint_2") ) );
    trajopt.AddBoundingBoxConstraint(kneeSet - eps, kneeSet + eps, xapex1(positions_map.at("joint_3") ) );

    trajopt.AddBoundingBoxConstraint(upperSet - eps, upperSet + eps, xapex1(positions_map.at("joint_4") ) );
    trajopt.AddBoundingBoxConstraint(kneeSet - eps, kneeSet + eps, xapex1(positions_map.at("joint_5") ) );

    trajopt.AddBoundingBoxConstraint(upperSet - eps, upperSet + eps, xapex1(positions_map.at("joint_6") ) );
    trajopt.AddBoundingBoxConstraint(kneeSet - eps, kneeSet + eps, xapex1(positions_map.at("joint_7") ) );

    trajopt.AddBoundingBoxConstraint(-0, 0, xapex1( n_q + velocities_map.at("joint_0dot")));
    trajopt.AddBoundingBoxConstraint(-0, 0, xapex1( n_q + velocities_map.at("joint_1dot")));
    trajopt.AddBoundingBoxConstraint(-0, 0, xapex1( n_q + velocities_map.at("joint_2dot")));
    trajopt.AddBoundingBoxConstraint(-0, 0, xapex1( n_q + velocities_map.at("joint_3dot")));
    trajopt.AddBoundingBoxConstraint(-0, 0, xapex1( n_q + velocities_map.at("joint_4dot")));
    trajopt.AddBoundingBoxConstraint(-0, 0, xapex1( n_q + velocities_map.at("joint_5dot")));
    trajopt.AddBoundingBoxConstraint(-0, 0, xapex1( n_q + velocities_map.at("joint_6dot")));
    trajopt.AddBoundingBoxConstraint(-0, 0, xapex1( n_q + velocities_map.at("joint_7dot")));
    trajopt.AddBoundingBoxConstraint(-0, 0, xapex1( n_q + velocities_map.at("joint_8dot")));
    trajopt.AddBoundingBoxConstraint(-0, 0, xapex1( n_q + velocities_map.at("joint_9dot")));
    trajopt.AddBoundingBoxConstraint(-0, 0, xapex1( n_q + velocities_map.at("joint_10dot")));
    trajopt.AddBoundingBoxConstraint(-0, 0, xapex1( n_q + velocities_map.at("joint_11dot")));
    trajopt.AddBoundingBoxConstraint(-0, 0, xapex1( n_q + velocities_map.at("joint_12dot")));
  }

  // Constraint the state during flight
  for (int iJump=0;iJump<2;iJump++){
    for (int j=2;j<4;j++){
      for (int k=0;k<this->nKnotpoints_flight;k++){
        auto x_flight = trajopt.state_vars(iJump*5+j, k);

        auto t0=2.0 * (x_flight(0) * x_flight(3) - x_flight(1)* x_flight(2));
        auto t1=1 - 2* (x_flight(2) * x_flight(2) + x_flight(3)* x_flight(3));
        auto yaw_flight = atan2(t0, t1);
        auto roll_flight = atan2(2.0 * (x_flight(0) * x_flight(1) - x_flight(2) * x_flight(3)), 1.0 - 2.0 * (x_flight(1) * x_flight(1) + x_flight(2) * x_flight(2)));
        auto pitch_flight = asin(2.0 * (x_flight(0) * x_flight(2) + x_flight(3) * x_flight(1)));
        if (pose_ref){
          if (iJump%2==0){
            trajopt.AddConstraint(yaw_flight,-yaw_ref*2,yaw_ref/2);
            trajopt.AddConstraint(roll_flight,roll_surface_1*1.5,roll_surface_0*1.2);
            trajopt.AddConstraint(pitch_flight,-pitch_ref,pitch_ref);
          }
          else{
            trajopt.AddConstraint(yaw_flight,-yaw_ref/2,yaw_ref*2);
            trajopt.AddConstraint(roll_flight,roll_surface_1*1.5,roll_surface_0*1.2);
            trajopt.AddConstraint(pitch_flight,-pitch_ref,pitch_ref);
          }
        }
      }
    }
  }
  


  int interiorStanceModeIndex = 5;
  int botKnotpoint = trajopt.mode_length(interiorStanceModeIndex)/2;
  auto xbot = trajopt.state_vars( interiorStanceModeIndex , botKnotpoint);
  if(this->spine_type=="twisting") {
    for (int i=8;i<n_q;i++) trajopt.AddConstraint( xbot(i)-x0(i),-eps, eps);
    trajopt.AddConstraint( xbot(7)+x0(7),-eps, eps);
  }
  else for (int i=7;i<n_q;i++) trajopt.AddConstraint( xbot(i)-x0(i),-eps, eps);

  Eigen::Vector3d sNormal;
  Eigen::Vector3d sOffset;
  for (int j=interiorStanceModeIndex-1;j<interiorStanceModeIndex+3;j++){
    for (int k=0;k<nKnotpoints_stances;k++){
      auto xi = trajopt.state_vars(j,k);
      auto pitchi = asin(2.0 * (xi(0) * xi(2) + xi(3) * xi(1)));
      // Elbows above the ground
      trajopt.AddConstraint(pitchi+xi(positions_map.at("joint_1"))-xi(positions_map.at("joint_0")),eps,M_PI );
      trajopt.AddConstraint(pitchi+xi(positions_map.at("joint_3"))-xi(positions_map.at("joint_2")),eps,M_PI );
      trajopt.AddConstraint(pitchi+xi(positions_map.at("joint_5"))-xi(positions_map.at("joint_4")),eps,M_PI );
      trajopt.AddConstraint(pitchi+xi(positions_map.at("joint_7"))-xi(positions_map.at("joint_6")),eps,M_PI );
      if (pose_ref){
        trajopt.AddConstraint(pitchi,-pitch_ref,pitch_ref);
      }
    }
  }
  std::tie(sNormal,sOffset,std::ignore) = transitionSurfaces[1];

  this->offsetConstraint(plant, trajopt, xbot, sNormal,sOffset,xtol);
  trajopt.AddBoundingBoxConstraint(-eps, + eps, xbot(n_q + velocities_map.at("base_vz")) );
  // Apex pose constraints
  auto tbot_0=2.0 * (xbot(0) * xbot(3) - xbot(1)* xbot(2));
  auto tbot_1=1 - 2* (xbot(2) * xbot(2) + xbot(3)* xbot(3));
  auto yaw_bot = atan2(tbot_0, tbot_1);
  auto roll_bot = atan2(2.0 * (xbot(0) * xbot(1) - xbot(2) * xbot(3)), 1.0 - 2.0 * (xbot(1) * xbot(1) + xbot(2) * xbot(2)));
  auto pitch_bot = asin(2.0 * (xbot(0) * xbot(2) + xbot(3) * xbot(1)));
  if (pose_ref){
    trajopt.AddConstraint(yaw_bot+yaw_0,-eps,eps);  //opposite yaw 
    trajopt.AddConstraint(roll_bot+roll_0,-eps,eps); //opposite yaw 
    trajopt.AddConstraint(pitch_bot-pitch_0,-eps,eps);
  }


  // Final state = initial state
  for (int i=0;i<n_q+n_v;i++) {
    if (i!=4) trajopt.AddConstraint( xf(i)-x0(i),-eps, eps);
  }

  // The "leg angle" = (pi/2 - theta0)

  auto times  = trajopt.time_vars();
  double a_knee_max=800;
  /// Constraints on all points
  for (int i = 0; i < trajopt.N(); i++){
    auto xi = trajopt.state(i);
    // Height  /// Need to be modified
    trajopt.AddBoundingBoxConstraint( 0.15, 2, xi( positions_map.at("base_z")));

    // Restrict upper leg angles  FOR THE STABILITY OF SEQUENTIAL OPTIMIZATION
    trajopt.AddBoundingBoxConstraint(-1-eps, M_PI+ eps, xi(positions_map.at("joint_0") ) );
    trajopt.AddBoundingBoxConstraint(-1-eps, M_PI+ eps, xi(positions_map.at("joint_2") ) );
    trajopt.AddBoundingBoxConstraint(-1-eps, M_PI+ eps, xi(positions_map.at("joint_4") ) );
    trajopt.AddBoundingBoxConstraint(-1-eps, M_PI+ eps, xi(positions_map.at("joint_6") ) );

    if(this->lock_spine && this->spine_type=="twisting") trajopt.AddBoundingBoxConstraint(-eps,  eps, xi(positions_map.at("joint_12") ) );
    // Address shaky legs
    // Limit knee joints' angular accelerations
    if(i>0){
      auto xim1=trajopt.state(i-1);
      trajopt.AddConstraint((xi(n_q+velocities_map.at("joint_1dot"))-xim1(n_q+velocities_map.at("joint_1dot")))/times(i-1),-a_knee_max,a_knee_max);
      trajopt.AddConstraint((xi(n_q+velocities_map.at("joint_3dot"))-xim1(n_q+velocities_map.at("joint_3dot")))/times(i-1),-a_knee_max,a_knee_max);
      trajopt.AddConstraint((xi(n_q+velocities_map.at("joint_5dot"))-xim1(n_q+velocities_map.at("joint_5dot")))/times(i-1),-a_knee_max,a_knee_max);
      trajopt.AddConstraint((xi(n_q+velocities_map.at("joint_7dot"))-xim1(n_q+velocities_map.at("joint_7dot")))/times(i-1),-a_knee_max,a_knee_max);
    }
  }
}

template <class Y>
void SpiritParkourWallRun<Y>::setUpModeSequence(){
  this->mode_vector.clear();
  this->normal_vector.clear();
  this->offset_vector.clear();
  this->minT_vector.clear();
  this->maxT_vector.clear();
  //                          mode name   normal                  world offset            minT  maxT
  this->addModeToSequenceVector("stance",std::get<0>(transitionSurfaces[0]),std::get<1>(transitionSurfaces[0]),     0.02, 0.5);
  this->addModeToSequenceVector("rear_stance",std::get<0>(transitionSurfaces[0]),std::get<1>(transitionSurfaces[0]),0.02, 1  ); //0.04
  this->addModeToSequenceVector("flight",Eigen::Vector3d::UnitZ(),Eigen::Vector3d::Zero(),0.02, 1  );

  this->addModeToSequenceVector("flight",Eigen::Vector3d::UnitZ(),Eigen::Vector3d::Zero(),0.02, 1  );
  this->addModeToSequenceVector("front_stance",std::get<0>(transitionSurfaces[1]),std::get<1>(transitionSurfaces[1]),0.02, 1  ); //0.04
  this->addModeToSequenceVector("stance",std::get<0>(transitionSurfaces[1]),std::get<1>(transitionSurfaces[1]),0.04, 1  );
  this->addModeToSequenceVector("rear_stance",std::get<0>(transitionSurfaces[1]),std::get<1>(transitionSurfaces[1]),0.02, 1  ); //0.04
  this->addModeToSequenceVector("flight",Eigen::Vector3d::UnitZ(),Eigen::Vector3d::Zero(),0.02, 1  );

  this->addModeToSequenceVector("flight",Eigen::Vector3d::UnitZ(),Eigen::Vector3d::Zero(),0.02, 1  );
  this->addModeToSequenceVector("front_stance",std::get<0>(transitionSurfaces[2]),std::get<1>(transitionSurfaces[2]),0.02, 1  ); //0.04
  this->addModeToSequenceVector("stance",      std::get<0>(transitionSurfaces[2]),std::get<1>(transitionSurfaces[2]),0.02, 1  );
}
  

/// runSpiritParkourWallRun, runs a trajectory optimization problem for spirit jumping on flat ground
/// \param plant: robot model
/// \param pp_xtraj: trajectory passed by pointer for spirit animation
template <class Y>
void SpiritParkourWallRun<Y>::run(MultibodyPlant<Y>& plant,
                          PiecewisePolynomial<Y>* pp_xtraj,
                          std::vector<SurfaceConf>* surface_vector) {

  
  // Setup mode sequence
  auto sequence = DirconModeSequence<Y>(plant);
  setUpModeSequence();
  auto [modeVector, toeEvals, toeEvalSets] = getModeSequence(plant, sequence);


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
  trajopt.SetSolverOption(id, "dual_inf_tol", this->tol*100);
  trajopt.SetSolverOption(id, "constr_viol_tol", this->tol);
  trajopt.SetSolverOption(id, "compl_inf_tol", this->tol);
  trajopt.SetSolverOption(id, "max_iter", 10000);
  trajopt.SetSolverOption(id, "nlp_lower_bound_inf", -1e6);
  trajopt.SetSolverOption(id, "nlp_upper_bound_inf", 1e6);
  trajopt.SetSolverOption(id, "print_timing_statistics", "yes");
  trajopt.SetSolverOption(id, "print_level", 5);
  if (warm_up){
    std::cout<<"Warm start"<<std::endl;
    trajopt.SetSolverOption(id, "bound_push", 1e-8);
    trajopt.SetSolverOption(id, "warm_start_bound_push", 1e-8);
    trajopt.SetSolverOption(id, "warm_start_init_point", "yes");
  }

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
    this->addGaussionNoiseToInputTraj(plant,trajopt,loaded_traj);
    this->addGaussionNoiseToVelocitiesTraj(plant,trajopt,loaded_traj);
    this->addGaussionNoiseToStateTraj(plant,trajopt,loaded_traj);
  }


  /// Setup the visualization during the optimization
  int num_ghosts = 1;// Number of ghosts in visualization. NOTE: there are limitations on number of ghosts based on modes and knotpoints
  std::vector<unsigned int> visualizer_poses; // Ghosts for visualizing during optimization
  for(int i = 0; i < sequence.num_modes(); i++){
      if (i==2 || i==8) num_ghosts = 3;
      else num_ghosts = 1;
      visualizer_poses.push_back(num_ghosts); 
  }
  trajopt.CreateVisualizationCallback(
      dairlib::FindResourceOrThrow(this->urdf_path),
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
  int beginIdx = this->file_name_out.rfind('/');
  std::string filename = this->file_name_out.substr(beginIdx + 1);
  std::string contact_force_fname=this->data_directory+filename+".csv";
  this->saveContactForceData(std::get<0>(transitionSurfaces[2])[0],contact_force_fname,result.is_success());


  // For animation
  *pp_xtraj =trajopt.ReconstructStateTrajectory(result);
  const drake::Vector4<double> orange(1.0, 0.55, 0.0, 1.0);
  double x_offset_1= pp_xtraj->value(pp_xtraj->end_time())(4,0);
  if (this->get_animate_info) {
    int counter = 0;
    double xoffset=0;
    for (int i=0;i<11;i++){
      for (auto& surface : transitionSurfaces){
        Eigen::Vector3d sNormal;
        Eigen::Vector3d sOffset;
        std::tie(sNormal,sOffset,std::ignore) = surface;
        sOffset[0]+=xoffset;
        counter++;
        struct SurfaceConf new_surface={sNormal,sOffset,0.5,0.5,0.1,orange,"surface"+std::to_string(counter)};
        surface_vector->push_back(new_surface);
      }
      xoffset+=x_offset_1;
    }
  }
  
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
}

template class SpiritParkourWallRun<double>;
}   // namespace dairlib