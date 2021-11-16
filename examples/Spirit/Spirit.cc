#include "examples/Spirit/Spirit.h"
#include "examples/Spirit/spirit_jump.h"
#include <yaml-cpp/yaml.h>
using drake::multibody::Parser;
using drake::multibody::MultibodyPlant;
using drake::trajectories::PiecewisePolynomial;
using drake::geometry::SceneGraph;
namespace dairlib {

template <template<class> class B,class T>
Spirit<B,T>::Spirit(std::string yaml_path) :plant (std::make_unique<MultibodyPlant<T>>(0.0)),
                    plant_vis (std::make_unique<MultibodyPlant<T>>(0.0)),
                    scene_graph_ptr (std::make_unique<SceneGraph<T>>()),
                    apex_goal(0.45),duration(1),ipopt(true),
                    behavior()
    {
    data_directory= "/home/feng/Downloads/dairlib/examples/Spirit/saved_trajectories/";
    this->yaml_path=yaml_path;
    YAML::Node config = YAML::LoadFile(yaml_path);
    runAllOptimization=config[0]["runAllOptimization"].as<bool>();
    skipInitialOptimization=config[0]["skipInitialOptimization"].as<bool>();
    minWork=config[0]["minWork"].as<bool>();


    ///init plant
    Parser parser(plant.get());
    Parser parser_vis(plant_vis.get(), scene_graph_ptr.get());
    std::string full_name =
        dairlib::FindResourceOrThrow("examples/Spirit/spirit_drake.urdf");

    parser.AddModelFromFile(full_name);
    parser_vis.AddModelFromFile(full_name);

    plant->mutable_gravity_field().set_gravity_vector(-9.81 *
        Eigen::Vector3d::UnitZ());

    plant->Finalize();
    plant_vis->Finalize();

    distance_name = std::to_string(int(floor(100*foreAftDisplacement)))+"cm";
    // behavior(0.45,1,true);
    // behavior(apex_goal,duration,ipopt);
    }

template <template<class> class B,class T>
void Spirit<B,T>::animate(){
  SceneGraph<double>& scene_graph = *builder.AddSystem(std::move(scene_graph_ptr));
  multibody::connectTrajectoryVisualizer(plant_vis.get(),
      &builder, &scene_graph, pp_xtraj);
  auto diagram = builder.Build();
  while (1) {
    drake::systems::Simulator<double> simulator(*diagram);
    simulator.set_target_realtime_rate(0.25);
    simulator.Initialize();
    simulator.AdvanceTo(pp_xtraj.end_time());
    sleep(2);
  }
}

template <template<class> class B,class T>
void Spirit<B,T>::run(){
  
  if (runAllOptimization){
    if(! skipInitialOptimization){
      std::cout<<"Running initial optimization"<<std::endl;   
      behavior.config(yaml_path,1);
      behavior.badSpiritJump(*plant);
      behavior.run(*plant,&pp_xtraj);
    }
    else{
      behavior.loadOldTrajectory(data_directory+"simple_jump");
    }
    

    std::cout<<"Running 2nd optimization"<<std::endl;
    // Hopping correct distance, but heavily constrained
    behavior.config(yaml_path,2);
    behavior.run(*plant,&pp_xtraj);
    }
  std::cout<<"Running 3rd optimization"<<std::endl;
  // Fewer constraints, and higher tolerences
  behavior.config(yaml_path,3);
  behavior.run(*plant,&pp_xtraj);

  if (minWork){
    // Adding in work cost and constraints
    std::cout<<"Running 4th optimization"<<std::endl;
    behavior.config(yaml_path,4);
    behavior.run(*plant,&pp_xtraj);
  }

  animate();
  
}
template class Spirit<dairlib::SpiritJump,double>;
}