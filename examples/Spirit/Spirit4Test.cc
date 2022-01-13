#include "examples/Spirit/Spirit4Test.h"
#include "examples/Spirit/spirit_box_jump.h"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <gflags/gflags.h>
DEFINE_double(skip_to, 1, "skip to ith optimization"); //set it not 1 after running the optimization once

DEFINE_double(duration, 1, "The stand duration");
DEFINE_double(standHeight, 0.23, "The standing height.");
DEFINE_double(foreAftDisplacement, 1, "The fore-aft displacement.");
DEFINE_double(apexGoal, 0.7, "Apex state goal");
DEFINE_double(inputCost, 5, "input cost scale.");
DEFINE_double(velocityCost, 10, "velocity cost scale.");
DEFINE_double(eps, 1e-2, "The wiggle room.");
DEFINE_double(tol, 1e-6, "Optimization Tolerance");
DEFINE_double(mu, 1.5, "coefficient of friction");
DEFINE_double(boxHeight, 0.3, "The height of the landing");
DEFINE_double(iterationAngResDeg, 5, "Angular Resolution for iterative optimization");

DEFINE_string(data_directory, "/home/feng/Downloads/dairlib/examples/Spirit/saved_trajectories/",
              "directory to save/read data");
DEFINE_string(distance_name, "iterativeBox45","name to describe distance");

DEFINE_bool(runAllOptimization, true, "rerun earlier optimizations?");
DEFINE_bool(skipInitialOptimization, false, "skip first optimizations?");
DEFINE_bool(minWork, false, "skip try to minimize work?");
DEFINE_bool(runIterative, true, "for angled runs, run multiple optimizations to approach");

using drake::multibody::Parser;
using drake::multibody::MultibodyPlant;
using drake::trajectories::PiecewisePolynomial;
using drake::geometry::SceneGraph;
namespace dairlib {
template <template<class> class B,class T>
Spirit4Test<B,T>::Spirit4Test(std::string yaml_path) :plant (std::make_unique<MultibodyPlant<T>>(0.0)),
                    plant_vis (std::make_unique<MultibodyPlant<T>>(0.0)),
                    scene_graph_ptr (std::make_unique<SceneGraph<T>>()),
                    behavior()
    {
    this->yaml_path=yaml_path;
    YAML::Node config = YAML::LoadFile(yaml_path);
    
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


    // generate expanded yaml config
    std::cout<<"Start Generating Expanded Yaml File "<<std::endl;
    YAML::Node OPTIMIZATIONS;
    OPTIMIZATIONS.push_back(Clone(config[0]));
    for (std::size_t i=1;i<config.size();i++){
      std::cout<<config[i]["name"]<<"    "<<config[0]["iterate"]["name"]<<std::endl;
      if(config[i]["name"].as<std::string>()==config[0]["iterate"]["name"].as<std::string>()){
        std::cout<<"find iterative node: "<< config[i]["name"].as<std::string>()<<std::endl;
        for (std::size_t j =0; j<config[0]["iterate"]["values"].size();j++){
          OPTIMIZATIONS.push_back(Clone(config[i]));
          OPTIMIZATIONS[OPTIMIZATIONS.size()-1][config[0]["iterate"]["iteration_variable"].as<std::string>()]=config[0]["iterate"]["values"][j];
          OPTIMIZATIONS[OPTIMIZATIONS.size()-1]["animate"]=false;
        }
      }
      else{
        OPTIMIZATIONS.push_back(Clone(config[i]));
      }
    }
    OPTIMIZATIONS[OPTIMIZATIONS.size()-1]["animate"]=true;
    num_optimizations=OPTIMIZATIONS.size()-1;
    std::ofstream fout(saved_directory+"expanded_config.yaml");
    fout << OPTIMIZATIONS;
    this->yaml_path=saved_directory+"expanded_config.yaml";

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
    // plant_vis->Finalize();
    }

template <template<class> class B,class T>
void Spirit4Test<B,T>::animate(){

  Eigen::Vector3d normal = 1 *Eigen::Vector3d::UnitZ() + 0 * Eigen::Vector3d::UnitY();
  normal = normal/normal.norm();
  Eigen::Vector3d offset = Eigen::Vector3d::UnitX()*FLAGS_foreAftDisplacement + Eigen::Vector3d::UnitZ()*FLAGS_boxHeight;
  // Add the box surface to the final animation
  for(SurfaceConf surface : surface_vector){
    dairlib::visualizeSurface(
      plant_vis.get(),
      surface.surface_normal,
      surface.surface_offset,
      surface.length_surf,
      surface.width_surf,
      surface.thickness_surf);
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
void Spirit4Test<B,T>::run(){
  
  // // Set up the iteration
  // int numSteps = 1;
  // stands.push_back(initialStand);
  // if (FLAGS_runIterative){
  //   /// Get the stands for the necessary angles
  //   // Debug prints
  //   // std::cout<<"DotProd: "<<normal.dot(Eigen::Vector3d::UnitZ())<<std::endl;
  //   double angle = acos(normal.dot(Eigen::Vector3d::UnitZ()))*(180/M_PI);
  //   if (!(angle<1e-4)){
  //     Eigen::Vector3d axis =  normal.cross(Eigen::Vector3d::UnitZ());
  //     axis = axis/axis.norm();
  //     numSteps = ceil(angle/FLAGS_iterationAngResDeg);
  //     std::cout<<"Angle: "<<angle<<"    numsteps"<<numSteps<<std::endl;
  //     Eigen::Matrix3d initialFrame = Eigen::Matrix3d::Identity();

  //     for (int iStep = 1; iStep < numSteps; iStep++){
  //       Eigen::AngleAxis iRotation(iStep*angle*(M_PI/180)/numSteps,axis);
  //       Eigen::Vector3d iNormal = (initialFrame*(iRotation.toRotationMatrix()).transpose()).col(2);

  //       bool rerun = false;
  //       dairlib::OptimalSpiritStand iStand(plant.get(), FLAGS_standHeight, iNormal, offset, rerun);
  //       stands.push_back(iStand);
  //     }
  //   }
  // }
  // stands.push_back(finalStand);

  // std::cout<<"Running initial optimization"<<std::endl;
  // behavior.generateInitialGuess(*plant);
  
  // behavior.config(yaml_path,saved_directory,1,plant.get());
  // behavior.run(*plant,&pp_xtraj,&surface_vector);


// /// Get a finalFlatStand for getting a feasible leap and trajectory
// dairlib::OptimalSpiritStand   finalFlatStand(plant.get(), FLAGS_standHeight, initialNormal, offset, true);
// std::cout<<"Running 2nd optimization"<<std::endl;
// // Hopping correct distance, but heavily constrained

// behavior.config(yaml_path,saved_directory,2,plant.get());
// behavior.run(*plant,&pp_xtraj,&surface_vector);

//   /// Run all the the steps to feed a higher angle
//   for (int iStep = 0; iStep<numSteps;iStep++){
//     std::cout<<" Running hq optimization step: " << iStep +1 <<" of "<< numSteps <<std::endl;
//     // Fewer constraints, and higher tolerences
//     behavior.initialStand=initialStand;
//     behavior.finalStand=stands[iStep+1];
//     behavior.config(yaml_path,saved_directory,3+iStep,plant.get());
//     behavior.run(*plant,&pp_xtraj,&surface_vector);
//   }
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

template class Spirit4Test<dairlib::SpiritBoxJump,double>;

}