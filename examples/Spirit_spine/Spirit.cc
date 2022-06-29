#include "examples/Spirit_spine/Spirit.h"
#include "examples/Spirit_spine/spirit_jump.h"
#include "examples/Spirit_spine/spirit_bound.h"
#include "examples/Spirit_spine/spirit_bounds2s.h"
#include "examples/Spirit_spine/spirit_box_jump.h"
#include "examples/Spirit_spine/spirit_parkour.h"
#include "examples/Spirit_spine/spirit_parkour_wall.h"
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
    this->yaml_path=yaml_path;
    YAML::Node config = YAML::LoadFile(yaml_path);
    
    initial_guess=config[0]["initial_guess"].as<std::string>();
    saved_directory=config[0]["saved_directory"].as<std::string>();
    behavior.urdf_path=config[0]["urdf_path"].as<std::string>();
    behavior.spine_type=config[0]["spine_type"].as<std::string>();
    // Create saved directory if it doesn't exist
    if (!std::experimental::filesystem::is_directory(saved_directory) || !std::experimental::filesystem::exists(saved_directory)) { 
        std::experimental::filesystem::create_directory(saved_directory); 
    }
    // Copy current yaml to saved directory
    std::ifstream  src(yaml_path, std::ios::binary);
    std::ofstream  dst(saved_directory+"config.yaml",   std::ios::binary);
    dst << src.rdbuf();


    // generate expanded yaml config
    std::cout<<"Start Generating Expanded Yaml File "<<std::endl;
    YAML::Node OPTIMIZATIONS;
    OPTIMIZATIONS.push_back(Clone(config[0]));
    for (std::size_t i=1;i<config.size();i++){
      if(config[0]["iterate"]["name"]){
        if(config[i]["name"].as<std::string>()==config[0]["iterate"]["name"].as<std::string>()){
          std::cout<<"find iterative node: "<< config[i]["name"].as<std::string>()<<std::endl;
          std::string name_in=config[i]["file_name_in"].as<std::string>();
          for (std::size_t j =0; j<config[0]["iterate"]["values"].size();j++){
            OPTIMIZATIONS.push_back(Clone(config[i]));
            OPTIMIZATIONS[OPTIMIZATIONS.size()-1][config[0]["iterate"]["iteration_variable"].as<std::string>()]=config[0]["iterate"]["values"][j];
            // Generate correct file names in and out
            if (j!=0) OPTIMIZATIONS[OPTIMIZATIONS.size()-1]["file_name_in"]= name_in+"_"+std::to_string(j-1);
            if (j<config[0]["iterate"]["values"].size()-1) OPTIMIZATIONS[OPTIMIZATIONS.size()-1]["file_name_out"]= name_in+"_"+std::to_string(j);
          }
          if (config[0]["iterate"]["for"].size()>0){
            int j=0;
            for (double v =config[0]["iterate"]["for"]["start"].as<double>(); v < config[0]["iterate"]["for"]["end"].as<double>();
                  v += config[0]["iterate"]["for"]["step_size"].as<double>()){
              OPTIMIZATIONS.push_back(Clone(config[i]));
              OPTIMIZATIONS[OPTIMIZATIONS.size()-1][config[0]["iterate"]["iteration_variable"].as<std::string>()]=v;
              // Generate correct file names in and out
              if (v!=config[0]["iterate"]["for"]["start"].as<double>()) OPTIMIZATIONS[OPTIMIZATIONS.size()-1]["file_name_in"]= name_in+"_"+std::to_string(j-1);
              if (v+config[0]["iterate"]["for"]["step_size"].as<double>() < config[0]["iterate"]["for"]["end"].as<double>()) {
                OPTIMIZATIONS[OPTIMIZATIONS.size()-1]["file_name_out"]= name_in+"_"+std::to_string(j);
              }
              j++;
          }
          }
          
        }
        else{
          OPTIMIZATIONS.push_back(Clone(config[i]));
        }
      }
      else OPTIMIZATIONS.push_back(Clone(config[i]));
    }
    num_optimizations=OPTIMIZATIONS.size()-1;
    std::ofstream fout(saved_directory+"expanded_config.yaml");
    fout << OPTIMIZATIONS;
    this->yaml_path=saved_directory+"expanded_config.yaml"; //set yaml path to the new generated one

    ///init plant
    Parser parser(plant.get());
    Parser parser_vis(plant_vis.get(), scene_graph_ptr.get());
    std::string full_name =
        dairlib::FindResourceOrThrow(behavior.urdf_path);

    parser.AddModelFromFile(full_name);
    parser_vis.AddModelFromFile(full_name);

    plant->mutable_gravity_field().set_gravity_vector(-9.81 *
        Eigen::Vector3d::UnitZ());
    plant->Finalize();
    // plant_vis->Finalize();
    }

template <template<class> class B,class T>
void Spirit<B,T>::animate(){
  // Add the box surface to the final animation
  for(SurfaceConf surface : surface_vector){
    dairlib::visualizeSurface(
      plant_vis.get(),
      surface.surface_normal,
      surface.surface_offset,
      surface.length_surf,
      surface.width_surf,
      surface.thickness_surf,
      surface.color,
      surface.name);
  } 

  
  plant_vis->Finalize();


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
      if (i==num_optimizations) behavior.enable_animate();   
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
template class Spirit<dairlib::SpiritBoundS2s,double>;
template class Spirit<dairlib::SpiritBoxJump,double>;
template class Spirit<dairlib::SpiritParkourJump,double>;
template class Spirit<dairlib::SpiritParkourWallPronk,double>;
}