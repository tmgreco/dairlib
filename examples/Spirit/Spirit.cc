#include "examples/Spirit/Spirit.h"
#include "examples/Spirit/spirit_jump.h"
#include "examples/Spirit/spirit_bound.h"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <gflags/gflags.h>

DEFINE_double(skip_to, 1, "skip to ith optimization"); //set it not 1 after running the optimization once

using drake::multibody::Parser;
using drake::multibody::MultibodyPlant;
using drake::trajectories::PiecewisePolynomial;
using drake::geometry::SceneGraph;
namespace dairlib {
template <template<class> class B,class T>
Spirit<B,T>::Spirit(std::string yaml_path) :plant (std::make_unique<MultibodyPlant<T>>(0.0)),
                    plant_vis (std::make_unique<MultibodyPlant<T>>(0.0)),
                    scene_graph_ptr (std::make_unique<SceneGraph<T>>()),
                    behavior()
    {
    // Load yaml configurations for spirit
    this->yaml_path=yaml_path;
    YAML::Node config = YAML::LoadFile(yaml_path);
    num_optimizations=config[0]["num_optimizations"].as<int>();
    initial_guess=config[0]["initial_guess"].as<std::string>();

    saved_directory=config[0]["saved_directory"].as<std::string>();
    // Create saved directory if it doesn't exist
    if (!std::experimental::filesystem::is_directory(saved_directory) || !std::experimental::filesystem::exists(saved_directory)) { 
        std::experimental::filesystem::create_directory(saved_directory); 
    }
    // Copy current yaml to saved directory
    std::ifstream  src(yaml_path, std::ios::binary);
    std::ofstream  dst(saved_directory+"config.yaml",   std::ios::binary);
    dst << src.rdbuf();

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
  for (int i=FLAGS_skip_to;i<=num_optimizations;i++){
      std::cout<<"Running optimization "<<i<<std::endl;   
      behavior.config(yaml_path,saved_directory,i,plant.get());
      if (i==FLAGS_skip_to){
        if(initial_guess=="") behavior.generateInitialGuess(*plant); //If we don't have a file for initial guess, then generate one.
        else behavior.loadOldTrajectory(initial_guess); //Otherwise, use the trajectory file we have.
      } 
      behavior.run(*plant,&pp_xtraj,&surface_vector);
  }
}
template class Spirit<dairlib::SpiritJump,double>;
template class Spirit<dairlib::SpiritBound,double>;
}