#include "examples/Spirit/Spirit.h"
#include "examples/Spirit/spirit_jump.h"
using drake::multibody::Parser;
using drake::multibody::MultibodyPlant;
using drake::trajectories::PiecewisePolynomial;
using drake::geometry::SceneGraph;
namespace dairlib {

template <class T>
Spirit<T>::Spirit() :plant (std::make_unique<MultibodyPlant<T>>(0.0)),
                    plant_vis (std::make_unique<MultibodyPlant<T>>(0.0)),
                    scene_graph (std::make_unique<SceneGraph<T>>()),
                    apex_goal(0.45),duration(1),ipopt(true),
                    jump_behavior(apex_goal,duration,ipopt)
    {
    ///init parameters
    // apex_goal=0.45;
    // duration=1;
    // ipopt=true;
    standHeight=0.2;
    foreAftDisplacement=1.0;
    inputCost=3;
    velocityCost=10;
    eps=1e-2;
    tol=1e-3;
    mu=1;

    data_directory= "/home/feng/Downloads/dairlib/examples/Spirit/saved_trajectories/";
    
    runAllOptimization=true;
    skipInitialOptimization=false;
    minWork=true;


    ///init plant
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

    distance_name = std::to_string(int(floor(100*foreAftDisplacement)))+"cm";
    // jump_behavior(0.45,1,true);
    // jump_behavior(apex_goal,duration,ipopt);
    std::cout<<"Initialize successfully"<<std::endl;
    }

template <class T>
void Spirit<T>::jump(){
    std::cout<<"Going to jump"<<std::endl;
    if (runAllOptimization){
    if(! skipInitialOptimization){
      std::cout<<"Running initial optimization"<<std::endl;
      jump_behavior.badSpiritJump(*plant);
      jump_behavior.run(
          *plant,
          false,
          {7, 7, 7, 7},
          apex_goal,
          standHeight,
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
          eps,
          1e-1,
          data_directory+"simple_jump");
    }
    else{
      jump_behavior.loadOldTrajectory(data_directory+"simple_jump");
    }
    

    std::cout<<"Running 2nd optimization"<<std::endl;
    // Hopping correct distance, but heavily constrained
    jump_behavior.run(
        *plant,
        false,
        {7, 7, 7, 7} ,
        apex_goal,
        standHeight,
        foreAftDisplacement,
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
        data_directory+"jump_"+distance_name);
    }
    std::cout<<"Running 3rd optimization"<<std::endl;
    // Fewer constraints, and higher tolerences
    jump_behavior.run(
      *plant,
      !minWork,
      {7, 7, 7, 7} ,
      apex_goal,
      standHeight,
      foreAftDisplacement,
      false,
      false,
      false,
      true,
      2,
      3,
      10,
      0,
      mu,
      eps,
      tol,
      data_directory+"jump_"+distance_name+"_hq",
      data_directory+"jump_"+distance_name);

  if (minWork){
    // Adding in work cost and constraints
    std::cout<<"Running 4th optimization"<<std::endl;
    jump_behavior.run(
        *plant,
        true,
        {7, 7, 7, 7} ,
        apex_goal,
        standHeight,
        foreAftDisplacement,
        false,
        false,
        false,
        true,
        1.5,
        inputCost/10,
        velocityCost/10,
        100,
        mu,
        eps,
        tol,
        data_directory+"jump_"+distance_name+"_hq_work_option3",
        data_directory+"jump_"+distance_name+"_hq");
  }
  
}
template class Spirit<double>;
}