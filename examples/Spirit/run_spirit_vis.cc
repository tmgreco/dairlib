#include <memory>
#include <chrono>
#include <unistd.h>
#include <gflags/gflags.h>
#include <string.h>

#include "drake/solvers/snopt_solver.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/solvers/solve.h"

#include "common/find_resource.h"
#include "systems/trajectory_optimization/dircon/dircon.h"
#include "multibody/kinematic/world_point_evaluator.h"
#include "multibody/multibody_utils.h"
#include "multibody/visualization_utils.h"
#include "multibody/kinematic/kinematic_constraints.h"

#include "examples/Spirit/animate_spirit.h"
#include "examples/Spirit/spirit_utils.h"

DEFINE_double(duration, 3, "The squat duration");

using drake::AutoDiffXd;
using drake::multibody::MultibodyPlant;
using drake::geometry::SceneGraph;
using drake::multibody::Parser;
using drake::trajectories::PiecewisePolynomial;

using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::Matrix3d;
using Eigen::MatrixXd;

// drake::math::RotationMatrix<double> normal2Rotation(Eigen::Vector3d nHat){
//   const double eps  = 1e-6;
//   drake::math::RotationMatrix<double> rotMat;
//   assert(std::abs(nHat.squaredNorm() - 1) < eps );
//   if       ( std::abs(nHat.dot(Eigen::Vector3d::UnitZ()) - 1) < eps){ //If close to +unitZ dont rotate
//     std::cout<<"A"<<std::endl;
//     rotMat = drake::math::RotationMatrix<double>();
//   }else if ( std::abs(nHat.dot(Eigen::Vector3d::UnitZ()) + 1) < eps){ //If close to -unitZ dont rotate
//     std::cout<<"B"<<std::endl;
//     Eigen::Matrix3d R;
//     R <<  1,  0,  0,
//           0, -1,  0,
//           0,  0, -1;
//     rotMat = drake::math::RotationMatrix<double>(R);
//   }else if ( std::abs(nHat.dot(Eigen::Vector3d::UnitX())) <= std::abs(nHat.dot(Eigen::Vector3d::UnitY())) ){
//     std::cout<<"C"<<std::endl;
//     Eigen::Vector3d nyHat =  nHat.cross(Eigen::Vector3d::UnitX());
//     Eigen::Vector3d nxHat =  nyHat.cross(nHat);
//     Eigen::Matrix3d R;
//     R.col(0) << nxHat; 
//     R.col(1) << nyHat; 
//     R.col(2) << nHat; 
//     rotMat = drake::math::RotationMatrix<double>(R);
//   }else if ( std::abs(nHat.dot(Eigen::Vector3d::UnitX())) > std::abs(nHat.dot(Eigen::Vector3d::UnitY())) ){
//     std::cout<<"D"<<std::endl;
//     Eigen::Vector3d nxHat = -nHat.cross(Eigen::Vector3d::UnitY());
//     Eigen::Vector3d nyHat =  nHat.cross(nxHat);
//     Eigen::Matrix3d R;
//     R.col(0) << nxHat; 
//     R.col(1) << nyHat; 
//     R.col(2) << nHat; 
//     rotMat = drake::math::RotationMatrix<double>(R);
//   }else{
//     std::cout<<"Something went wrong check here"<<std::endl;
//     rotMat = drake::math::RotationMatrix<double>();
//   }
//   return rotMat;
// }

// void visualizeSurface(MultibodyPlant<double>* plant_vis, 
//   Eigen::Vector3d surface_normal = Eigen::Vector3d::UnitY(),
//   Eigen::Vector3d surface_offset = Eigen::Vector3d::UnitY()*.5 + Eigen::Vector3d::UnitZ()*0,
//   double length_surf = 0.3, 
//   double width_surf = 0.3,
//   double thickness_surf = 0.05
//   ){

//   const drake::Vector4<double> orange(1.0, 0.55, 0.0, 1.0);
//   Eigen::Vector3d bodyOffset(0,0,thickness_surf/2);
//   drake::math::RotationMatrix<double> rot = normal2Rotation(surface_normal);
//   drake::math::RigidTransformd bodyToSurfaceTransform(bodyOffset);
//   drake::math::RigidTransformd worldToBodyTransform(rot,surface_offset);
  
//   double lx = length_surf;
//   double ly = width_surf;
//   double lz = thickness_surf;

//   std::cout << rot.matrix() << std::endl;
//   plant_vis->RegisterVisualGeometry( 
//     plant_vis->world_body(),
//     worldToBodyTransform*bodyToSurfaceTransform,     /* Pose X_BG of the geometry frame G in the cylinder body frame B. */
//     drake::geometry::Box(lx, ly, lz), 
//     "box", orange);
// }
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
  dairlib::visualizeSurface(plant_vis.get());
  plant->Finalize();
  plant_vis->Finalize();

  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(plant->num_positions() +
                       plant->num_velocities());

  int nx = plant->num_positions() + plant->num_velocities();
  int nq = plant->num_positions();
  int N = 100; // number of timesteps

  std::vector<MatrixXd> init_x;

  // Initialize state trajectory
  std::vector<double> init_time;
  VectorXd xState(nx);
  xState = Eigen::VectorXd::Zero(plant->num_positions() + plant->num_velocities());
  auto positions_map = dairlib::multibody::makeNameToPositionsMap(*plant);
  auto velocities_map = dairlib::multibody::makeNameToVelocitiesMap(*plant);
  int num_joints = 12;

  //Intialize the quaternion position of the body
  xState(positions_map.at("base_qw")) = 1;
  xState(positions_map.at("base_qx")) = 0;
  xState(positions_map.at("base_qy")) = 0;
  xState(positions_map.at("base_qz")) = 0;

  //Set Joint Velocities
  for (int j = 0; j < num_joints; j++){   
    xState(nq + velocities_map.at( "joint_"+std::to_string(j)+"dot" )) = 0.2;
  }
  

  double time = 0;
  for (int i = 0; i < N; i++) {
    time=i*FLAGS_duration/(N-1);
    init_time.push_back(time);

    // Integrate joint positions based on velocities
    for (int j =0; j < num_joints; j++){
        xState(positions_map.at("joint_"+std::to_string(j))) = xState(nq + velocities_map.at("joint_" + std::to_string(j)+"dot" )) * time;
    }
    
    //Add to knotpoint state matrix
    init_x.push_back(xState);
  }
  auto init_x_traj = PiecewisePolynomial<double>::ZeroOrderHold(init_time, init_x);

  dairlib::runAnimate( std::move(plant), plant_vis.get(), std::move(scene_graph), init_x_traj);
}

