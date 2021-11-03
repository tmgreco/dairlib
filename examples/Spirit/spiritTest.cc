#include "examples/Spirit/spirit_jump.h"
using drake::multibody::Parser;

DEFINE_double(duration, 1, "The stand duration");
DEFINE_double(standHeight, 0.2, "The standing height.");
DEFINE_double(foreAftDisplacement, 1.0  , "The fore-aft displacement.");
DEFINE_double(apexGoal, 0.45, "Apex state goal");
DEFINE_double(inputCost, 3, "The standing height.");
DEFINE_double(velocityCost, 10, "The standing height.");
DEFINE_double(eps, 1e-2, "The wiggle room.");
DEFINE_double(tol, 1e-3, "Optimization Tolerance");
DEFINE_double(mu, 1, "coefficient of friction");

DEFINE_string(data_directory, "/home/feng/Downloads/dairlib/examples/Spirit/saved_trajectories/",
              "directory to save/read data");

DEFINE_bool(runAllOptimization, true, "rerun earlier optimizations?");
DEFINE_bool(skipInitialOptimization, false, "skip first optimizations?");
DEFINE_bool(minWork, true, "try to minimize work?");
DEFINE_bool(ipopt, true, "Use IPOPT as solver instead of SNOPT");

int main(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  std::srand(time(0));  // Initialize random number generator.

  auto plant = std::make_unique<MultibodyPlant<double>>(0.0);
  auto plant_vis = std::make_unique<MultibodyPlant<double>>(0.0);
  auto scene_graph = std::make_unique<SceneGraph<double>>();
  Parser parser(plant.get());
  Parser parser_vis(plant_vis.get(), scene_graph.get());
  std::string full_name =
      dairlib::FindResourceOrThrow("examples/Spirit/spirit_drake.urdf");

  parser.AddModelFromFile(full_name);
  parser_vis.AddModelFromFile(full_name);

  plant->mutable_gravity_field().set_gravity_vector(-9.81 *
      Eigen::Vector3d::UnitZ());

  plant->Finalize();
  plant_vis->Finalize();

  std::string distance_name = std::to_string(int(floor(100*FLAGS_foreAftDisplacement)))+"cm";

  dairlib::SpiritJump<double> jump_helper(FLAGS_apexGoal,FLAGS_duration,FLAGS_ipopt);
  // dairlib::SpiritJump<double> jump_helper;

  if (FLAGS_runAllOptimization){
    if(! FLAGS_skipInitialOptimization){
      std::cout<<"Running initial optimization"<<std::endl;
      jump_helper.badSpiritJump(*plant);
      jump_helper.run(
          *plant,
          false,
          {7, 7, 7, 7},
          FLAGS_apexGoal,
          FLAGS_standHeight,
          0,
          true,
          true,
          false,
          true,
          2,
          3,
          10,
          0,
          100,
          FLAGS_eps,
          1e-1,
          FLAGS_data_directory+"simple_jump");
    }
    else{
      jump_helper.loadOldTrajectory(FLAGS_data_directory+"simple_jump");
    }
    
    std::cout<<"Running 2nd optimization"<<std::endl;
    // Hopping correct distance, but heavily constrained
    jump_helper.run(
        *plant,
        false,
        {7, 7, 7, 7} ,
        FLAGS_apexGoal,
        FLAGS_standHeight,
        FLAGS_foreAftDisplacement,
        true,
        true,
        false,
        true,
        2,
        3,
        10,
        0,
        100,
        0,
        1e-2,
        FLAGS_data_directory+"jump_"+distance_name);
    }
    std::cout<<"Running 3rd optimization"<<std::endl;
    // Fewer constraints, and higher tolerences
    jump_helper.run(
      *plant,
      !FLAGS_minWork,
      {7, 7, 7, 7} ,
      FLAGS_apexGoal,
      FLAGS_standHeight,
      FLAGS_foreAftDisplacement,
      false,
      false,
      false,
      true,
      2,
      3,
      10,
      0,
      FLAGS_mu,
      FLAGS_eps,
      FLAGS_tol,
      FLAGS_data_directory+"jump_"+distance_name+"_hq",
      FLAGS_data_directory+"jump_"+distance_name);

  if (FLAGS_minWork){
    // Adding in work cost and constraints
    std::cout<<"Running 4th optimization"<<std::endl;
    jump_helper.run(
        *plant,
        true,
        {7, 7, 7, 7} ,
        FLAGS_apexGoal,
        FLAGS_standHeight,
        FLAGS_foreAftDisplacement,
        false,
        false,
        false,
        true,
        1.5,
        FLAGS_inputCost/10,
        FLAGS_velocityCost/10,
        100,
        FLAGS_mu,
        FLAGS_eps,
        FLAGS_tol,
        FLAGS_data_directory+"jump_"+distance_name+"_hq_work_option3",
        FLAGS_data_directory+"jump_"+distance_name+"_hq");
  }
}

