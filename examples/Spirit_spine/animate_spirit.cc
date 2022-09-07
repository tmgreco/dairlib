
#include "examples/Spirit_spine/animate_spirit.h"
#include <gflags/gflags.h>
DEFINE_string(traj_path, "temp", "path to trajectory to be viz"); 
DEFINE_int32(num_period, 1, "How many periods to be visualized"); 
DEFINE_double(real_time_rate,0.25,"Display speed");
DEFINE_string(behavior, "jump", "behavior"); 
DEFINE_string(spine_type, "rigid", "spine_type"); 
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

using systems::trajectory_optimization::DirconModeSequence;
using systems::trajectory_optimization::DirconMode;
using systems::trajectory_optimization::Dircon;
using systems::trajectory_optimization::KinematicConstraintType;

using std::vector;
using std::cout;
using std::endl;

/// See runAnimate(
///    std::unique_ptr<MultibodyPlant<T>> plant_ptr,
///    MultibodyPlant<double>* plant_double_ptr,
///    std::unique_ptr<SceneGraph<double>> scene_graph_ptr,
///    PiecewisePolynomial<double> pp_xtraj )
///
/// Takes the plants and scenegraph and a trajectory and
/// creates a visualization of that trajectory (example
/// built in the main file).
template<typename T>
void runAnimate(
    std::unique_ptr<MultibodyPlant<T>> plant_ptr,
    MultibodyPlant<double> *plant_double_ptr,
    std::unique_ptr<SceneGraph<double>> scene_graph_ptr,
    PiecewisePolynomial<double> pp_xtraj,
    const double real_time_factor
) {

  drake::systems::DiagramBuilder<double> builder;
  MultibodyPlant<T> &plant = *plant_ptr;
  SceneGraph<double> &scene_graph =
      *builder.AddSystem(std::move(scene_graph_ptr));

  auto positions_map = multibody::makeNameToPositionsMap(plant);
  auto velocities_map = multibody::makeNameToVelocitiesMap(plant);

  multibody::connectTrajectoryVisualizer(plant_double_ptr,
                                         &builder, &scene_graph, pp_xtraj);
  auto diagram = builder.Build();
  while (true) {

    drake::systems::Simulator<double> simulator(*diagram);
    simulator.set_target_realtime_rate(real_time_factor);
    simulator.Initialize();
    simulator.AdvanceTo(pp_xtraj.end_time());
    sleep(2);
  }
}

void animateTraj(std::string& urdf_path) {

  auto plant = std::make_unique<MultibodyPlant<double>>(0.0);
  auto plant_vis = std::make_unique<MultibodyPlant<double>>(0.0);
  auto scene_graph_ptr = std::make_unique<SceneGraph<double>>();
  drake::systems::DiagramBuilder<double> builder;
  Parser parser(plant.get());
  Parser parser_vis(plant_vis.get(), scene_graph_ptr.get());
  std::string full_name;
  if (FLAGS_spine_type=="rigid") full_name = dairlib::FindResourceOrThrow("examples/Spirit_spine/spirit_drake.urdf");
  else full_name = dairlib::FindResourceOrThrow("examples/Spirit_spine/spirit_with_spine_drake.urdf");

  parser.AddModelFromFile(full_name);
  parser_vis.AddModelFromFile(full_name);

  plant->mutable_gravity_field().set_gravity_vector(-9.81 *
      Eigen::Vector3d::UnitZ());

  plant->Finalize();
  plant_vis->Finalize();

  dairlib::DirconTrajectory old_traj(FLAGS_traj_path);
  PiecewisePolynomial<double> pp_xtraj = old_traj.ReconstructStateTrajectory();


  PiecewisePolynomial<double> x_traj_1;
  if (FLAGS_behavior=="trot"){
    x_traj_1=old_traj.ReconstructStateTrajectory();
    PiecewisePolynomial<double> x_traj_2=old_traj.ReconstructStateTrajectory();
    std::vector<double> breaks=x_traj_1.get_breaks();
    // Reconstruct flipped traj
    for (int i = 0; i < static_cast<int>(breaks.size())-1; ++i) {
      drake::MatrixX<drake::Polynomial<double>> org_matrix=x_traj_1.getPolynomialMatrix(i);
      drake::MatrixX<drake::Polynomial<double>> new_matrix=x_traj_2.getPolynomialMatrix(i);
      if (FLAGS_spine_type=="twisting"){
        new_matrix(1)*=-1; //opposite qx
        new_matrix(3)*=-1; //opposite qz
        new_matrix(4)+=x_traj_1.value(x_traj_1.end_time())(4,0);
        new_matrix(5)*=-1; //opposite y
        new_matrix(14)=org_matrix(15); //joint 0 = jiont 4
        new_matrix(18)=org_matrix(19); //joint 1 = jiont 5
        new_matrix(12)=org_matrix(13); //joint 2 = jiont 6
        new_matrix(16)=org_matrix(17); //joint 3 = jiont 7
        new_matrix(15)=org_matrix(14); //joint 4 = jiont 0
        new_matrix(19)=org_matrix(18); //joint 5 = jiont 1
        new_matrix(13)=org_matrix(12); //joint 6 = jiont 2
        new_matrix(17)=org_matrix(16); //joint 7 = jiont 3
        new_matrix(10)=-org_matrix(11); //joint 8 = - jiont 10
        new_matrix(8)=-org_matrix(9); //joint 9 = -jiont 11
        new_matrix(11)=-org_matrix(10); //joint 10 = -jiont 8
        new_matrix(9)=-org_matrix(8); //joint 11 = -jiont 9
        new_matrix(7)*=-1;  //Opposite Joint 12

        new_matrix(20)*=-1; //opposite wx
        new_matrix(22)*=-1; //opposite wz
        new_matrix(24)*=-1; //opposite vy
        new_matrix(33)=org_matrix(34); //Velocities: joint 0 = jiont 4
        new_matrix(37)=org_matrix(38); //Velocities: joint 1 = jiont 5
        new_matrix(31)=org_matrix(32); //Velocities: joint 2 = jiont 6
        new_matrix(35)=org_matrix(36); //Velocities: joint 3 = jiont 7
        new_matrix(34)=org_matrix(33); //Velocities: joint 4 = jiont 0
        new_matrix(38)=org_matrix(37); //Velocities: joint 5 = jiont 1
        new_matrix(32)=org_matrix(31); //Velocities: joint 6 = jiont 2
        new_matrix(36)=org_matrix(35); //Velocities: joint 7 = jiont 3
        new_matrix(29)=-org_matrix(30); //Velocities: joint 8 = -jiont 10
        new_matrix(27)=-org_matrix(28); //Velocities: joint 9 = -jiont 11
        new_matrix(30)=-org_matrix(29); //Velocities: joint 10 = -jiont 8
        new_matrix(28)=-org_matrix(27); //Velocities: joint 11 = -jiont 9
        new_matrix(26)*=-1;  //Velocities: Opposite Joint 12
      }
      else if (FLAGS_spine_type=="rigid"){
        new_matrix(1)*=-1; //opposite qx
        new_matrix(3)*=-1; //opposite qz
        new_matrix(4)+=x_traj_1.value(x_traj_1.end_time())(4,0);
        new_matrix(5)*=-1; //opposite y
        new_matrix(11)=org_matrix(13); //joint 0 = jiont 4
        new_matrix(15)=org_matrix(17); //joint 1 = jiont 5
        new_matrix(12)=org_matrix(14); //joint 2 = jiont 6
        new_matrix(16)=org_matrix(18); //joint 3 = jiont 7
        new_matrix(13)=org_matrix(11); //joint 4 = jiont 0
        new_matrix(17)=org_matrix(15); //joint 5 = jiont 1
        new_matrix(14)=org_matrix(12); //joint 6 = jiont 2
        new_matrix(18)=org_matrix(16); //joint 7 = jiont 3
        new_matrix(7)=-org_matrix(9); //joint 8 = - jiont 10
        new_matrix(8)=-org_matrix(10); //joint 9 = -jiont 11
        new_matrix(9)=-org_matrix(7); //joint 10 = -jiont 8
        new_matrix(10)=-org_matrix(8); //joint 11 = -jiont 9

        new_matrix(19)*=-1; //opposite wx
        new_matrix(21)*=-1; //opposite wz
        new_matrix(23)*=-1; //opposite vy
        new_matrix(31)=org_matrix(32); //Velocities: joint 0 = jiont 4
        new_matrix(35)=org_matrix(36); //Velocities: joint 1 = jiont 5
        new_matrix(29)=org_matrix(30); //Velocities: joint 2 = jiont 6
        new_matrix(33)=org_matrix(34); //Velocities: joint 3 = jiont 7
        new_matrix(32)=org_matrix(31); //Velocities: joint 4 = jiont 0
        new_matrix(36)=org_matrix(35); //Velocities: joint 5 = jiont 1
        new_matrix(30)=org_matrix(29); //Velocities: joint 6 = jiont 2
        new_matrix(34)=org_matrix(33); //Velocities: joint 7 = jiont 3
        new_matrix(27)=-org_matrix(28); //Velocities: joint 8 = -jiont 10
        new_matrix(25)=-org_matrix(26); //Velocities: joint 9 = -jiont 11
        new_matrix(28)=-org_matrix(27); //Velocities: joint 10 = -jiont 8
        new_matrix(26)=-org_matrix(25); //Velocities: joint 11 = -jiont 9
      }
      x_traj_2.setPolynomialMatrixBlock(new_matrix,i);
    }   
    x_traj_2.shiftRight(x_traj_1.end_time());
    x_traj_1.ConcatenateInTime(x_traj_2);
    pp_xtraj=x_traj_1;
  }
  


  
  if (FLAGS_num_period>1){
    /// Create offset polynomial
    std::vector<double> breaks=pp_xtraj.get_breaks();
    std::vector<Eigen::MatrixXd> samples(breaks.size());
    for (int i = 0; i < static_cast<int>(breaks.size()); ++i) {
      samples[i].resize(39, 1);
      for (int j=0;j<39;j++) samples[i](j, 0) = 0;
      samples[i](4, 0) = pp_xtraj.value(pp_xtraj.end_time())(4,0);
    }
    PiecewisePolynomial<double> ini_offset_pp = PiecewisePolynomial<double>::FirstOrderHold(breaks, samples);
    PiecewisePolynomial<double> offset_pp=ini_offset_pp;
    for (int i=0;i<FLAGS_num_period;i++){
      PiecewisePolynomial<double> x_traj_i;
      if (FLAGS_behavior=="trot") x_traj_i=x_traj_1+offset_pp;
      else x_traj_i=old_traj.ReconstructStateTrajectory()+offset_pp;
      offset_pp+=ini_offset_pp;
      x_traj_i.shiftRight(pp_xtraj.end_time());
      pp_xtraj.ConcatenateInTime(x_traj_i);
    }
  }

  SceneGraph<double>& scene_graph = *builder.AddSystem(std::move(scene_graph_ptr));
  multibody::connectTrajectoryVisualizer(plant_vis.get(),
      &builder, &scene_graph, pp_xtraj);
  auto diagram = builder.Build();
  // bool save=false;
  std::cout<<"Start running animation with real time factor = "<< FLAGS_real_time_rate<<std::endl;
  while (1) {
    drake::systems::Simulator<double> simulator(*diagram);
    simulator.set_target_realtime_rate(FLAGS_real_time_rate);
    simulator.Initialize();
    simulator.AdvanceTo(pp_xtraj.end_time());
    sleep(1);
  }
}

template <typename T>
void addGaussionNoiseToStateTraj(MultibodyPlant<T>& plant,
                                drake::trajectories::PiecewisePolynomial<T>& state_traj){

  drake::trajectories::PiecewisePolynomial<T> org_pp=state_traj;

  /// Create offset polynomial
  double mean=1;
  double var=1;
  std::vector<double> breaks=state_traj.get_breaks();
  std::vector<Eigen::MatrixXd> samples(breaks.size());
  int num_states=plant.num_positions()+plant.num_velocities();
  auto normal_dist = std::bind(std::normal_distribution<double>{mean, var},
                      std::mt19937(std::random_device{}())); // Normal distribution
  auto normal_dist_joints = std::bind(std::normal_distribution<double>{mean, var},
                      std::mt19937(std::random_device{}())); // Normal distribution
  
  for (int i = 0; i < static_cast<int>(breaks.size())-1; ++i) {
    drake::MatrixX<drake::Polynomial<T>> org_matrix=state_traj.getPolynomialMatrix(i);
    std::cout<<"old matrix "<<i<<"\n"<<org_matrix<<std::endl;
    drake::Polynomial<T> scalar_poly(2);
    std::cout<<"!!!\n"<<org_matrix(4)<<"\n"<<scalar_poly<<std::endl;
    org_matrix(4)*=scalar_poly;
    org_matrix(23)*=scalar_poly;
    
    state_traj.setPolynomialMatrixBlock(org_matrix,i);
    std::cout<<"new matrix "<<i<<"\n"<<org_matrix<<std::endl;
  }
  std::cout<<"FINISHED"<<std::endl;
}

template void runAnimate(
    std::unique_ptr<drake::multibody::MultibodyPlant<double>> plant_ptr,
    drake::multibody::MultibodyPlant<double> *plant_double_ptr,
    std::unique_ptr<drake::geometry::SceneGraph<double>> scene_graph_ptr,
    drake::trajectories::PiecewisePolynomial<double> pp_xtraj,
    double real_time_factor
); //NOLINT

template void addGaussionNoiseToStateTraj(MultibodyPlant<double>& plant,
                                drake::trajectories::PiecewisePolynomial<double>& state_traj);
}// namespace dairlib
