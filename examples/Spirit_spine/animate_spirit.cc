
#include "examples/Spirit_spine/animate_spirit.h"
#include <gflags/gflags.h>
DEFINE_string(traj_path, "temp", "path to trajectory to be viz"); 

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

void animateTraj(std::string& urdf_path,int num_period,double real_time_rate) {
  // auto plant_vis = std::make_unique<drake::multibody::MultibodyPlant<double>>(0.0);
  // auto scene_graph_ptr = std::make_unique<SceneGraph<double>>();
  // drake::systems::DiagramBuilder<double> builder;

  // drake::multibody::Parser parser_vis(plant_vis.get(), scene_graph_ptr.get());
  // std::string full_name =dairlib::FindResourceOrThrow("examples/Spirit_spine/spirit_with_spine_drake.urdf");

  // parser_vis.AddModelFromFile(full_name);
  // plant_vis.Finalize();


  auto plant = std::make_unique<MultibodyPlant<double>>(0.0);
  auto plant_vis = std::make_unique<MultibodyPlant<double>>(0.0);
  auto scene_graph_ptr = std::make_unique<SceneGraph<double>>();
  drake::systems::DiagramBuilder<double> builder;
  Parser parser(plant.get());
  Parser parser_vis(plant_vis.get(), scene_graph_ptr.get());
  std::string full_name =
      dairlib::FindResourceOrThrow("examples/Spirit_spine/spirit_drake.urdf");

  parser.AddModelFromFile(full_name);
  parser_vis.AddModelFromFile(full_name);

  plant->mutable_gravity_field().set_gravity_vector(-9.81 *
      Eigen::Vector3d::UnitZ());

  plant->Finalize();
  plant_vis->Finalize();

  dairlib::DirconTrajectory old_traj(FLAGS_traj_path);
  PiecewisePolynomial<double> pp_xtraj = old_traj.ReconstructStateTrajectory();
  
  if (num_period>1){
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
    for (int i=0;i<10;i++){
      PiecewisePolynomial<double> x_traj_i=old_traj.ReconstructStateTrajectory()+offset_pp;
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
  std::cout<<"Start running animation with real time factor = "<< real_time_rate<<std::endl;
  while (1) {
    drake::systems::Simulator<double> simulator(*diagram);
    simulator.set_target_realtime_rate(real_time_rate);
    simulator.Initialize();
    simulator.AdvanceTo(pp_xtraj.end_time());
    sleep(2);
  }
}

template void runAnimate(
    std::unique_ptr<drake::multibody::MultibodyPlant<double>> plant_ptr,
    drake::multibody::MultibodyPlant<double> *plant_double_ptr,
    std::unique_ptr<drake::geometry::SceneGraph<double>> scene_graph_ptr,
    drake::trajectories::PiecewisePolynomial<double> pp_xtraj,
    double real_time_factor
); //NOLINT

}// namespace dairlib
