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
  this->animate=config[index]["animate"].as<bool>();
  this->nJumps=config[index]["n_jumps"].as<int>();
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

  this->transitionSurfaces.push_back(std::make_tuple(config[index]["transition_surface"][1].as<double>()*Eigen::Vector3d::UnitZ(),
                                                    config[index]["transition_surface"][0].as<double>()*Eigen::Vector3d::UnitX(), std::numeric_limits<double>::infinity()));
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
  std::cout << "T1" << t1 <<std::endl;
  std::cout << "T2" << t2 <<std::endl;
  std::vector<int> posInds{positions_map.at("base_x"),positions_map.at("base_y"),positions_map.at("base_z")};
  double t1Toffset = t1.transpose()*offset;
  double t2Toffset = t2.transpose()*offset;
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

}

template <class Y>
void SpiritParkourJump<Y>::generateInitialGuess(MultibodyPlant<Y>& plant){
  std::string file_name_in = "/home/feng/Downloads/dairlib/examples/Spirit/saved_trajectories/jump_test1/simple_jump";
  dairlib::DirconTrajectory loaded_traj(file_name_in);

  
  // this->x_traj.clear(); 
  // this->u_traj.clear(); 
  // this->l_traj.clear(); 
  // this->lc_traj.clear(); 
  // this->vc_traj.clear(); 

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
  /// Run animation of the final trajectory
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
  

template <class Y>
void SpiritParkourJump<Y>::addCost(
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

  auto   x0  = trajopt.initial_state();
  auto   xlo = trajopt.state_vars(1, 0);
  // auto xapex = trajopt.state_vars(2, 0);
  auto   xtd = trajopt.state_vars(3, 0);
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
  trajopt.AddBoundingBoxConstraint(
              standOffsetInit(0), 
              standOffsetInit(0), 
              x0(positions_map.at("base_x"))); 
  trajopt.AddBoundingBoxConstraint(
              standOffsetInit(1)-eps, 
              standOffsetInit(1)+eps, 
              x0(positions_map.at("base_y"))); 

  // Final Standing XY Pos
  trajopt.AddBoundingBoxConstraint(
              standOffsetFinal(0)-eps, 
              standOffsetFinal(0)+eps, 
              xf(positions_map.at("base_x")));
  trajopt.AddBoundingBoxConstraint(
              standOffsetFinal(1)-eps, 
              standOffsetFinal(1)+eps, 
              xf(positions_map.at("base_y")));
  
  // Joint constraints init and final
  trajopt.AddBoundingBoxConstraint(
              standJointsInit-Eigen::VectorXd::Constant(standJointsInit.size(), eps),
              standJointsInit+Eigen::VectorXd::Constant(standJointsInit.size(), eps),
              (x0.head(n_q)).tail(n_q-7));
  trajopt.AddBoundingBoxConstraint(
              standJointsFinal-Eigen::VectorXd::Constant(standJointsFinal.size(), eps),
              standJointsFinal+Eigen::VectorXd::Constant(standJointsFinal.size(), eps),
              (xf.head(n_q)).tail(n_q-7));

  // Body pose constraints at initial and final state
  trajopt.AddBoundingBoxConstraint(
        standQuatInit - Eigen::Vector4d::Constant(eps), 
        standQuatInit + Eigen::Vector4d::Constant(eps), 
        x0.head(4));
  trajopt.AddBoundingBoxConstraint(
        standQuatFinal - Eigen::Vector4d::Constant(eps), 
        standQuatFinal + Eigen::Vector4d::Constant(eps), 
        xf.head(4));

  // Initial and final velocity
  trajopt.AddBoundingBoxConstraint(
          VectorXd::Zero(n_v), 
          VectorXd::Zero(n_v), 
          x0.tail(n_v));
  trajopt.AddBoundingBoxConstraint(
          VectorXd::Zero(n_v), 
          VectorXd::Zero(n_v), 
          xf.tail(n_v));
  //  ****************************************************

  
  for (int iJump=0;iJump<numJumps;iJump++){
    // Apex height
    if(apex_height > 0){
      auto xapex = trajopt.state_vars(2 + iJump*3 , 0);
      trajopt.AddBoundingBoxConstraint(
            apex_height - eps, 
            apex_height + eps, 
            xapex(positions_map.at("base_z")) );
            
      trajopt.AddBoundingBoxConstraint(
            - eps, 
            + eps, 
            xapex(n_q + velocities_map.at("base_vy")) );
      trajopt.AddBoundingBoxConstraint(
            - eps, 
            + eps, 
            xapex(n_q + velocities_map.at("base_vz")) );
    }
    if(iJump != 0){
      std::cout << "Zero Velocity Bottoms activated" << std::endl;
      int interiorStanceModeIndex = iJump*3;
      int botKnotpoint = trajopt.mode_length(interiorStanceModeIndex)/2;
      auto xbot = trajopt.state_vars( interiorStanceModeIndex , botKnotpoint);
      Eigen::Vector3d sNormal;
      Eigen::Vector3d sOffset;
      std::tie(sNormal,sOffset,std::ignore) = transitionSurfaces[iJump-1];
      this->offsetConstraint(
        plant, 
        trajopt, 
        xbot, 
        sNormal,//Eigen::Vector3d::UnitY()+Eigen::Vector3d::UnitZ(), 
        sOffset
        );

      // if(stop_at_bottom){
      //   trajopt.AddBoundingBoxConstraint(
      //           standOffsetFinal(0)-eps, 
      //           standOffsetFinal(0)+eps, 
      //           xbot(positions_map.at("base_x")));
      //   trajopt.AddBoundingBoxConstraint(
      //           VectorXd::Zero(n_v), 
      //           VectorXd::Zero(n_v), 
      //           xbot.tail(n_v));
      // }
    }

  }
  // The "leg angle" = (pi/2 - theta0)

  // double upperSet = 1;
  // double kneeSet = 2;

  // if(lock_legs_apex){
  //   //STATIC LEGS AT APEX
  //   trajopt.AddBoundingBoxConstraint(upperSet - eps, upperSet + eps, xapex(positions_map.at("joint_0") ) );
  //   trajopt.AddBoundingBoxConstraint(kneeSet - eps, kneeSet + eps, xapex(positions_map.at("joint_1") ) );

  //   trajopt.AddBoundingBoxConstraint(upperSet - eps, upperSet + eps, xapex(positions_map.at("joint_2") ) );
  //   trajopt.AddBoundingBoxConstraint(kneeSet - eps, kneeSet + eps, xapex(positions_map.at("joint_3") ) );

  //   trajopt.AddBoundingBoxConstraint(upperSet - eps, upperSet + eps, xapex(positions_map.at("joint_4") ) );
  //   trajopt.AddBoundingBoxConstraint(kneeSet - eps, kneeSet + eps, xapex(positions_map.at("joint_5") ) );

  //   trajopt.AddBoundingBoxConstraint(upperSet - eps, upperSet + eps, xapex(positions_map.at("joint_6") ) );
  //   trajopt.AddBoundingBoxConstraint(kneeSet - eps, kneeSet + eps, xapex(positions_map.at("joint_7") ) );

  //   trajopt.AddBoundingBoxConstraint(-0, 0, xapex( n_q + velocities_map.at("joint_0dot")));
  //   trajopt.AddBoundingBoxConstraint(-0, 0, xapex( n_q + velocities_map.at("joint_1dot")));
  //   trajopt.AddBoundingBoxConstraint(-0, 0, xapex( n_q + velocities_map.at("joint_2dot")));
  //   trajopt.AddBoundingBoxConstraint(-0, 0, xapex( n_q + velocities_map.at("joint_3dot")));
  //   trajopt.AddBoundingBoxConstraint(-0, 0, xapex( n_q + velocities_map.at("joint_4dot")));
  //   trajopt.AddBoundingBoxConstraint(-0, 0, xapex( n_q + velocities_map.at("joint_5dot")));
  //   trajopt.AddBoundingBoxConstraint(-0, 0, xapex( n_q + velocities_map.at("joint_6dot")));
  //   trajopt.AddBoundingBoxConstraint(-0, 0, xapex( n_q + velocities_map.at("joint_7dot")));
  //   trajopt.AddBoundingBoxConstraint(-0, 0, xapex( n_q + velocities_map.at("joint_8dot")));
  //   trajopt.AddBoundingBoxConstraint(-0, 0, xapex( n_q + velocities_map.at("joint_9dot")));
  //   trajopt.AddBoundingBoxConstraint(-0, 0, xapex( n_q + velocities_map.at("joint_10dot")));
  //   trajopt.AddBoundingBoxConstraint(-0, 0, xapex( n_q + velocities_map.at("joint_11dot")));
  // }

  for (int i = 0; i < trajopt.N(); i++){
    auto xi = trajopt.state(i);
    
    // Height
    trajopt.AddBoundingBoxConstraint( 0.15, 5, xi( positions_map.at("base_z")));
    
    double leg_angle_boundary_distance = M_PI_4;//Set the angular distance from horizontal that the toes are allowed 
    
    trajopt.AddLinearConstraint( xi(positions_map.at("joint_1"))/2 - xi(positions_map.at("joint_0")) <=   M_PI_2 - leg_angle_boundary_distance );
    trajopt.AddLinearConstraint( xi(positions_map.at("joint_1"))/2 - xi(positions_map.at("joint_0")) >= - M_PI_2 + leg_angle_boundary_distance );
    
    trajopt.AddLinearConstraint( xi(positions_map.at("joint_3"))/2 - xi(positions_map.at("joint_2")) <=   M_PI_2 - leg_angle_boundary_distance );
    trajopt.AddLinearConstraint( xi(positions_map.at("joint_3"))/2 - xi(positions_map.at("joint_2")) >= - M_PI_2 + leg_angle_boundary_distance );
    
    trajopt.AddLinearConstraint( xi(positions_map.at("joint_5"))/2 - xi(positions_map.at("joint_4")) <=   M_PI_2 - leg_angle_boundary_distance );
    trajopt.AddLinearConstraint( xi(positions_map.at("joint_5"))/2 - xi(positions_map.at("joint_4")) >= - M_PI_2 + leg_angle_boundary_distance );
    
    trajopt.AddLinearConstraint( xi(positions_map.at("joint_7"))/2 - xi(positions_map.at("joint_6")) <=   M_PI_2 - leg_angle_boundary_distance );
    trajopt.AddLinearConstraint( xi(positions_map.at("joint_7"))/2 - xi(positions_map.at("joint_6")) >= - M_PI_2 + leg_angle_boundary_distance );
    
  }
  for (int iMode = 0; iMode<trajopt.num_modes(); iMode++){
    if(iMode%3){
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
  dairlib::ModeSequenceHelper msh;
  msh.addMode( // Stance
      (Eigen::Matrix<bool,1,4>() << true,  true,  true,  true).finished(), // contact bools
      nKnotpoints_stances,  // number of knot points in the collocation
      initialStand.normal(), // normal
      initialStand.offset(),  // world offset
      1.5  //   initialStand.mu() //friction
  );
  msh.addFlight(nKnotpoints_flight);
  msh.addFlight(nKnotpoints_flight);


  for (auto& surface : transitionSurfaces){
    Eigen::Vector3d sNormal;
    Eigen::Vector3d sOffset;
    double sMu;
    std::tie(sNormal,sOffset,sMu) = surface;
    msh.addMode( // Stance
        (Eigen::Matrix<bool,1,4>() << true,  true,  true,  true).finished(), // contact bools
        nKnotpoints_stances,  // number of knot points in the collocation
        sNormal, // normal
        sOffset,  // world offset
        sMu //friction
    );
    msh.addFlight(nKnotpoints_flight);
    msh.addFlight(nKnotpoints_flight);
  }


  msh.addMode( // Stance
      (Eigen::Matrix<bool,1,4>() << true,  true,  true,  true).finished(), // contact bools
      nKnotpoints_stances,  // number of knot points in the collocation
      finalStand.normal(), // normal
      finalStand.offset(),  // world offset
      1.5  //finalStand.mu() //friction
  );

  auto [modeVector, toeEvals, toeEvalSets] = createSpiritModeSequence(plant, msh);

  for (auto& mode : modeVector){
    for (int i = 0; i < mode->evaluators().num_evaluators(); i++ ){
      mode->MakeConstraintRelative(i,0);
      mode->MakeConstraintRelative(i,1);
    }
    mode->SetDynamicsScale(
        {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17}, 200);
    if (mode->evaluators().num_evaluators() > 0)
    {
      mode->SetKinVelocityScale(
          {0, 1, 2, 3}, {0, 1, 2}, 1.0);
      mode->SetKinPositionScale(
          {0, 1, 2, 3}, {0, 1, 2}, 200);
    }
    sequence.AddMode(mode.get());
  }

  ///Setup trajectory optimization
  auto trajopt = Dircon<Y>(sequence);
  // Set up Trajectory Optimization options
  trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(),
                           "Print file", "../snopt.out");
  trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(),
                           "Major iterations limit", 200000);
  trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(), "Iterations limit", 200000);
  trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(),
                           "Major optimality tolerance",
                           this->tol);  // target optimality
  trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(), "Major feasibility tolerance", this->tol);
  trajopt.SetSolverOption(drake::solvers::SnoptSolver::id(), "Verify level",
                           0);  // 0

  // Setting up cost
  addCost(plant, trajopt);
  // Setting up constraints
  bool stop_at_bottom = true;
  addConstraints(plant, trajopt);

  // Initialize the trajectory control state and forces
  if (this->file_name_in.empty()){
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
  const auto result = Solve(trajopt, trajopt.initial_guess());
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  std::cout << "Solve time:" << elapsed.count() <<std::endl;
  std::cout << "Cost:" << result.get_optimal_cost() <<std::endl;
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