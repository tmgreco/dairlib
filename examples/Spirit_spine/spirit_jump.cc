#include "examples/Spirit_spine/spirit_jump.h"
#include <yaml-cpp/yaml.h>


using drake::multibody::MultibodyPlant;
using drake::trajectories::PiecewisePolynomial;
using Eigen::VectorXd;
using Eigen::MatrixXd;

namespace dairlib {
using systems::trajectory_optimization::DirconModeSequence;
using systems::trajectory_optimization::Dircon;

template <class Y> 
SpiritJump<Y>::SpiritJump(){
}

//Configuration for the next optimization (jump)
/// \param yaml_path: path of yaml file
/// \param saved_directory: directory to save trajectories and yaml duplication
/// \param index: which configuration to load
template <class Y>
void SpiritJump<Y>::config(std::string yaml_path, std::string saved_directory, int index,MultibodyPlant<Y>* plant){
  YAML::Node config = YAML::LoadFile(yaml_path);
  this->index=index;
  this->duration=config[index]["duration"].as<double>();
  this->ipopt=config[index]["ipopt"].as<bool>();
  this->num_knot_points=config[index]["num_knot_points"].as<std::vector<int>>();
  this->apex_height=config[index]["apex_height"].as<double>();
  this->initial_height=config[index]["initial_height"].as<double>();
  this->fore_aft_displacement=config[index]["fore_aft_displacement"].as<double>();
  this->lock_rotation=config[index]["lock_rotation"].as<bool>();
  this->lock_legs_apex=config[index]["lock_legs_apex"].as<bool>();
  this->force_symmetry=config[index]["force_symmetry"].as<bool>();
  this->use_nominal_stand=config[index]["use_nominal_stand"].as<bool>();
  this->max_duration=config[index]["max_duration"].as<double>();
  this->cost_actuation=config[index]["cost_actuation"].as<double>();
  this->cost_velocity=config[index]["cost_velocity"].as<double>();
  this->cost_work=config[index]["cost_work"].as<double>();
  this->cost_time=config[index]["cost_time"].as<double>();
  this->mu=config[index]["mu"].as<double>();
  this->eps=config[index]["eps"].as<double>();
  this->tol=config[index]["tol"].as<double>();

  if(!config[index]["file_name_out"].as<std::string>().empty()) this->file_name_out=saved_directory+config[index]["file_name_out"].as<std::string>();
  if(!config[index]["file_name_in"].as<std::string>().empty()) this->file_name_in= saved_directory+config[index]["file_name_in"].as<std::string>();

}


/// generateInitialGuess, generates a bad initial guess for the spirit jump traj opt
/// \param plant: robot model
template <class Y>
void SpiritJump<Y>::generateInitialGuess(MultibodyPlant<Y>& plant){
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
  int num_joints = 13;

  // Print joint dictionary
  std::cout<<"**********************Joints***********************"<<std::endl;
  for (auto const& element : positions_map)
    std::cout << element.first << " = " << element.second << std::endl;
  for (auto const& element : velocities_map)
    std::cout << element.first << " = " << element.second << std::endl;
  std::cout<<"***************************************************"<<std::endl;

  dairlib::nominalSpiritStand( plant, xInit,  0.16); //Update xInit
  dairlib::nominalSpiritStand( plant, xMid,  0.35); //Update xMid

  if (apex_height>0) xMid(positions_map.at("base_z"))=apex_height;

  VectorXd deltaX(nx);
  VectorXd averageV(nx);
  deltaX = xMid-xInit;
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


// addConstraints, adds constraints to the trajopt jump problem. See runSpiritJump for a description of the inputs
/// \param plant: robot model
/// \param trajopt: trajectory to be optimized
template <class Y>
void SpiritJump<Y>::addConstraints(
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
  trajopt.AddBoundingBoxConstraint(-eps, eps, x0(positions_map.at("joint_12")));
  // Body pose constraints (keep the body flat) at initial state
  this->addPoseConstraints(trajopt,x0,positions_map,0,0,0,1,eps);
  // Initial velocity
  trajopt.AddBoundingBoxConstraint(VectorXd::Zero(n_v), VectorXd::Zero(n_v), x0.tail(n_v));
  

  // Lift off body positions conditions
  trajopt.AddBoundingBoxConstraint(-eps, eps, xlo(positions_map.at("base_y")));

  // Apex body positions conditions
  trajopt.AddBoundingBoxConstraint(-eps, eps, xapex(positions_map.at("base_y")));
  trajopt.AddBoundingBoxConstraint(- eps, + eps, xapex(n_q + velocities_map.at("base_vz")) );
  // trajopt.AddBoundingBoxConstraint(-eps, eps, xapex(positions_map.at("joint_12")));
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
    trajopt.AddBoundingBoxConstraint(-0, 0, xapex( n_q + velocities_map.at("joint_12dot")));
  }


  // Touchdown body positions conditions
  trajopt.AddBoundingBoxConstraint(-eps, eps, xtd(positions_map.at("base_y")));
  

  // Final body positions conditions
  trajopt.AddBoundingBoxConstraint(fore_aft_displacement - eps, fore_aft_displacement + eps, xf(positions_map.at("base_x"))); // Give the initial condition room to choose the x_init position (helps with positive knee constraint)
  trajopt.AddBoundingBoxConstraint(-eps, eps, xf(positions_map.at("base_y")));
  trajopt.AddBoundingBoxConstraint(-eps, eps, xf(positions_map.at("joint_12")));
  // Nominal stand or z and attitude
  if(use_nominal_stand){
    nominalSpiritStandConstraint(plant,trajopt,initial_height, {0,trajopt.N()-1}, eps);
  }else{
    trajopt.AddBoundingBoxConstraint(initial_height - eps, initial_height + eps, x0(positions_map.at("base_z")));
    trajopt.AddBoundingBoxConstraint(initial_height - eps, initial_height + eps, xf(positions_map.at("base_z")));
  }


  // Body pose constraints (keep the body flat) at final state
  this->addPoseConstraints(trajopt,xf,positions_map,0,0,0,1,eps);
  // Final velocity
  trajopt.AddBoundingBoxConstraint(VectorXd::Zero(n_v), VectorXd::Zero(n_v), xf.tail(n_v));

  for (int i = 0; i < trajopt.N(); i++){
    auto xi = trajopt.state(i);
    //Orientation
    if (lock_rotation and i != 0 and i != (trajopt.N()-1)) this->addPoseConstraints(trajopt,xi,positions_map,0,0,0,1,eps);
    // Height
    trajopt.AddBoundingBoxConstraint( 0.15, 5, xi( positions_map.at("base_z")));
  }
}


template <class Y>
void SpiritJump<Y>::setUpModeSequence(){
  this->mode_vector.clear();
  this->normal_vector.clear();
  this->offset_vector.clear();
  this->minT_vector.clear();
  this->maxT_vector.clear();
  //                          mode name   normal                  world offset            minT  maxT
  this->addModeToSequenceVector("stance",Eigen::Vector3d::UnitZ(),Eigen::Vector3d::Zero(),0,    std::numeric_limits<double>::infinity());
  this->addModeToSequenceVector("flight",Eigen::Vector3d::UnitZ(),Eigen::Vector3d::Zero(),0,    std::numeric_limits<double>::infinity());
  this->addModeToSequenceVector("flight",Eigen::Vector3d::UnitZ(),Eigen::Vector3d::Zero(),0,    std::numeric_limits<double>::infinity());
  this->addModeToSequenceVector("stance",Eigen::Vector3d::UnitZ(),Eigen::Vector3d::Zero(),0,    std::numeric_limits<double>::infinity());
}

/// runSpiritJump, runs a trajectory optimization problem for spirit jumping on flat ground
/// \param plant: robot model
/// \param pp_xtraj: trajectory passed by pointer for spirit animation
template <class Y>
void SpiritJump<Y>::run(MultibodyPlant<Y>& plant,
                          PiecewisePolynomial<Y>* pp_xtraj,
                          std::vector<SurfaceConf>* surface_vector) {
  // Setup mode sequence
  auto sequence = DirconModeSequence<Y>(plant);
  setUpModeSequence();
  auto [modeVector, toeEvals, toeEvalSets] = getModeSequence(plant, sequence);

  ///Setup trajectory optimization
  auto trajopt = Dircon<Y>(sequence); 

  this->setSolver(trajopt);
  // Setting up cost
  this->addCost(plant,trajopt);

  // Initialize the trajectory control state and forces
  this->initTrajectory(trajopt,sequence);

  // Setting constraints
  addConstraints(plant,trajopt);

  /// Setup the visualization during the optimization
  int num_ghosts = 3;// Number of ghosts in visualization. NOTE: there are limitations on number of ghosts based on modes and knotpoints
  std::vector<unsigned int> visualizer_poses; // Ghosts for visualizing during optimization
  for(int i = 0; i < sequence.num_modes(); i++){
      visualizer_poses.push_back(num_ghosts); 
  }
  trajopt.CreateVisualizationCallback(
      dairlib::FindResourceOrThrow("examples/Spirit_spine/spirit_with_spine_drake.urdf"),
      visualizer_poses, 0.2); // setup which URDF, how many poses, and alpha transparency 

  /// Run the optimization using your initial guess
  drake::solvers::SolverId solver_id("");
  if (this->ipopt) {
    solver_id = drake::solvers::IpoptSolver().id();
    trajopt.SetSolverOption(solver_id, "tol", this->tol);
    trajopt.SetSolverOption(solver_id, "dual_inf_tol", this->tol*100);
    trajopt.SetSolverOption(solver_id, "constr_viol_tol", this->tol);
    trajopt.SetSolverOption(solver_id, "compl_inf_tol", this->tol);
    trajopt.SetSolverOption(solver_id, "max_iter", 2000);
    trajopt.SetSolverOption(solver_id, "nlp_lower_bound_inf", -1e6);
    trajopt.SetSolverOption(solver_id, "nlp_upper_bound_inf", 1e6);
    trajopt.SetSolverOption(solver_id, "print_timing_statistics", "yes");
    trajopt.SetSolverOption(solver_id, "print_level", 5);

    // Set to ignore overall tolerance/dual infeasibility, but terminate when
    // primal feasible and objective fails to increase over 5 iterations.
    trajopt.SetSolverOption(solver_id, "acceptable_compl_inf_tol", this->tol);
    trajopt.SetSolverOption(solver_id, "acceptable_constr_viol_tol", this->tol);
    trajopt.SetSolverOption(solver_id, "acceptable_obj_change_tol", this->tol);
    trajopt.SetSolverOption(solver_id, "acceptable_tol", this->tol * 10);
    trajopt.SetSolverOption(solver_id, "acceptable_iter", 5);
    std::cout << "\nChose manually: " << solver_id.name() << std::endl;
  } else {
    solver_id = drake::solvers::ChooseBestSolver(trajopt);
    std::cout << "\nChose the best solver: " << solver_id.name() << std::endl;
  }
  
  auto start = std::chrono::high_resolution_clock::now();
  auto solver = drake::solvers::MakeSolver(solver_id);
  drake::solvers::MathematicalProgramResult result;
  solver->Solve(trajopt, trajopt.initial_guess(), trajopt.solver_options(),
                &result);
  auto finish = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = finish - start;
  std::cout << "Solve time: " << elapsed.count() <<std::endl;
  std::cout << "Cost: " << result.get_optimal_cost() <<std::endl;
  
  std::cout << (result.is_success() ? "Optimization Success" : "Optimization Fail. Running again") << std::endl;
  
  // If the optimization failed, recover by imposing and relaxing the constraints again
  
  /// Save trajectory
  this->saveTrajectory(plant,trajopt,result);

  // Writing contact force data
  std::string contect_force_fname="/home/feng/Downloads/dairlib/examples/Spirit_spine/data/test"+std::to_string(this->index)+".csv";
  this->saveContactForceData(this->fore_aft_displacement,contect_force_fname);
  

  // auto context = plant.CreateDefaultContext();
  // Eigen::VectorXd q_s=this->x_traj.value(0.4);
  // plant.SetPositionsAndVelocities(context.get(),q_s);
  // const auto& toe_frame = dairlib::getSpiritToeFrame(plant, 1);
  // std::cout<<"!!!!!!!!!!!"<<toe_frame.is_body_frame()<<std::endl;
  
  // std::cout<<toe_frame.CalcRotationMatrixInWorld(*context).matrix()<<std::endl;
  if (!result.is_success() || result.get_optimal_cost()>20000){
    this-> lock_legs_apex=true;
    this->run(plant,pp_xtraj,surface_vector);
    this-> lock_legs_apex=false;
  }
  

  /// pass the final trajectory back to spirit for animation
  *pp_xtraj =trajopt.ReconstructStateTrajectory(result);
  

  // std::vector<MatrixXd> x_points;
  // VectorXd x_const;

  // dairlib::ikSpiritStand(plant, x_const, {false, false, false, false}, apex_height, 0.2, 0, 0);
  // x_points.push_back(x_const);
  // dairlib::ikSpiritStand(plant, x_const, {true, false, true, false}, stand_height+0.05, 0.2, 0, -pitch_lo);
  // x_points.push_back(x_const);

  // this->x_traj.push_back(PiecewisePolynomial<double>::FirstOrderHold({0 * duration/3.0, 1 * duration/3.0},x_points));
  // x_points.clear();
  

  
}

template class SpiritJump<double>;
}  // namespace dairlib

