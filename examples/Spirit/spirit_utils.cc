#include <memory>
#include <chrono>
#include <tuple>
#include <unistd.h>
#include <gflags/gflags.h>
#include <string.h>
#include <assert.h>
#include <Eigen/StdVector>

#include "drake/solvers/snopt_solver.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/multibody/parsing/parser.h"
#include <drake/multibody/inverse_kinematics/inverse_kinematics.h>
#include "drake/geometry/geometry_visualization.h"
#include "drake/solvers/solve.h"

#include "common/find_resource.h"
#include "systems/trajectory_optimization/dircon/dircon.h"
#include "multibody/kinematic/world_point_evaluator.h"
#include "solvers/nonlinear_constraint.h"
#include "multibody/kinematic/distance_evaluator.h"
#include "multibody/multibody_utils.h"
#include "multibody/visualization_utils.h"
#include "multibody/kinematic/kinematic_constraints.h"

#include "examples/Spirit/spirit_utils.h"


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

/// Get a nominal Spirit Stand (i.e. zero hip ad/abduction motor torque, toes below motors) for initializing
template <typename T>
void nominalSpiritStand( MultibodyPlant<T>& plant, Eigen::VectorXd& xState, double height){
  //Get joint name dictionaries
  auto positions_map = dairlib::multibody::makeNameToPositionsMap(plant);
  auto velocities_map = dairlib::multibody::makeNameToVelocitiesMap(plant);
  
  // Initialize state vector and add orienation and goal height
  xState = Eigen::VectorXd::Zero(plant.num_positions() + plant.num_velocities());
  xState(positions_map.at("base_qw")) = 1;
  xState(positions_map.at("base_z")) = height;
  
  //Calculate the inverse kinematics to get the joint angles for toe placement
  double upperLegLength = 0.206; // length of the upper leg link
  double hipLength = 0.10098; //absOffset ToeLocation
  double hip2toeZLength = sqrt(height*height - hipLength*hipLength);// length of lower legs (2dof)
  double theta1 = asin(hip2toeZLength/(2*upperLegLength )) ; //upper angle
  double theta2 = 2*theta1 ; //lower angle
  double alpha = asin(hipLength/(height)); //hip angle

  int upperInd, kneeInd, hipInd;
  int mirroredFlag;
  for (int j = 0; j < 4; j++){
    upperInd = j * 2;
    kneeInd = j * 2 + 1;
    hipInd = j + 8;
    xState(positions_map.at("joint_" + std::to_string(upperInd))) = theta1 ;
    xState(positions_map.at("joint_" + std::to_string(kneeInd))) = theta2 ;
    mirroredFlag = -1;
    if ( hipInd > 9 ){
      mirroredFlag = 1;
    }
    xState(positions_map.at("joint_" + std::to_string(hipInd))) = mirroredFlag * alpha;
  }
}

template <typename T>
const drake::multibody::Frame<T>& getSpiritToeFrame( MultibodyPlant<T>& plant, u_int8_t toeIndex ){
  assert(toeIndex<4);
  return plant.GetFrameByName( "toe" + std::to_string(toeIndex) );
}

template <typename T>
std::unique_ptr<dairlib::multibody::WorldPointEvaluator<T>> getSpiritToeEvaluator( 
                      MultibodyPlant<T>& plant, 
                      const Eigen::Vector3d toePoint ,
                      u_int8_t toeIndex,
                      const Eigen::Vector3d normal ,
                      const Eigen::Vector3d offset ,
                      bool xy_active, 
                      double mu 
                      ){
  assert(toeIndex<4); // Check that toeIndex is an actual Spirit leg
  auto toe_eval =  std::make_unique<dairlib::multibody::WorldPointEvaluator<T>>(
        plant, 
        toePoint, 
        getSpiritToeFrame(plant, toeIndex ) , 
        normal, 
        offset, 
        xy_active );
  if(mu){
    toe_eval->set_frictional(); toe_eval->set_mu(mu);
  }
  return toe_eval;
}

template <typename T>
std::tuple<
                std::vector<std::unique_ptr<DirconMode<T>>>,
                std::vector<std::unique_ptr<dairlib::multibody::WorldPointEvaluator<T>>> ,
                std::vector<std::unique_ptr<dairlib::multibody::KinematicEvaluatorSet<T>>>
          > createSpiritModeSequence( 
          MultibodyPlant<T>& plant, // multibodyPlant
          std::vector<Eigen::Matrix<bool,1,4>> modeSeqVect, // bool matrix describing toe contacts as true or false e.g. {{1,1,1,1},{0,0,0,0}} would be a full support mode and flight mode
          std::vector<int> knotpointVect, // Matrix of knot points for each mode  
          std::vector<Eigen::Vector3d> normals,
          std::vector<Eigen::Vector3d> offsets, 
          std::vector<double> mus ){

  // std::cout<<modeSeqMat<<std::endl;
  // std::cout<<knotpointVect<<std::endl;  

  const double toeRadius = 0.02;
  const Vector3d toeOffset(toeRadius,0,0); // vector to "contact point"
  int num_modes = modeSeqVect.size();
  assert( num_modes == knotpointVect.size() );
  assert( num_modes == mus.size() );

  std::vector<std::unique_ptr<dairlib::multibody::WorldPointEvaluator<T>>> toeEvals;
  std::vector<std::unique_ptr<dairlib::multibody::KinematicEvaluatorSet<T>>> toeEvalSets;
  std::vector<std::unique_ptr<DirconMode<T>>> modeVector;
  // DirconModeSequence<T> sequence = DirconModeSequence<T>(plant);

  for (int iMode = 0; iMode<num_modes; iMode++)
  {
    
    toeEvalSets.push_back( std::move( std::make_unique<dairlib::multibody::KinematicEvaluatorSet<T>>(plant) ));
    for ( int iLeg = 0; iLeg < 4; iLeg++ ){
      if (modeSeqVect.at(iMode)(iLeg)){
        toeEvals.push_back( std::move( getSpiritToeEvaluator(plant, toeOffset, iLeg, normals.at(iMode), offsets.at(iMode), mus.at(iMode) )) );//Default Normal (z), offset, and xy_active=true
        (toeEvalSets.back())->add_evaluator(  (toeEvals.back()).get()  ); //add evaluator to the set if active //Works ish
      }
    }
    auto dumbToeEvalPtr = (toeEvalSets.back()).get() ;
    int num_knotpoints = knotpointVect.at(iMode);
    modeVector.push_back(std::move( std::make_unique<DirconMode<T>>( *dumbToeEvalPtr , num_knotpoints )));
    // DirconMode<T> modeDum = DirconMode<T>( *dumbToeEvalPtr , num_knotpoints );
    // sequence.AddMode(  &modeDum  ); // Add the evaluator set to the mode sequence
    
  }
  
  return {std::move(modeVector), std::move(toeEvals), std::move(toeEvalSets)};
  // return {std::move(toeEvals), std::move(toeEvalSets)};
}
// //Overload function to allow the use of a equal number of knotpoints for every mode.
// template <typename T>
// std::tuple<
//                 DirconModeSequence<T>,
//                 std::vector<std::unique_ptr<multibody::WorldPointEvaluator<T>>> ,
//                 std::vector<std::unique_ptr<multibody::KinematicEvaluatorSet<T>>>
//           > createSpiritModeSequence( 
//   MultibodyPlant<T>& plant, // multibodyPlant
//   Eigen::Matrix<bool,-1,4> modeSeqMat, // bool matrix describing toe contacts as true or false e.g. {{1,1,1,1},{0,0,0,0}} would be a full support mode and flight mode
//   uint16_t knotpoints, // Number of knot points per mode
//   double mu = 1){
//   int numModes = modeSeqMat.rows(); 
//   std::vector<int> knotpointVect = Eigen::MatrixXi::Constant(numModes,1,knotpoints);
//   return createSpiritModeSequence(plant, modeSeqMat,knotpointVect, mu);
// }











// **********************************************************
//   SETSPIRITJOINTLIMITS 


template <typename T> 
void setSpiritJointLimits(drake::multibody::MultibodyPlant<T> & plant, 
                          dairlib::systems::trajectory_optimization::Dircon<T>& trajopt,  
                          int iJoint, 
                          double minVal, 
                          double maxVal  ){

  auto positions_map = multibody::makeNameToPositionsMap(plant);
  auto velocities_map = multibody::makeNameToVelocitiesMap(plant);
  // std::cout<<"joint_" + std::to_string(iJoint)<<std::endl;
  int N_knotpoints = trajopt.N();
  for(int i = 0; i<N_knotpoints;i++){
    auto xi = trajopt.state(i);
    trajopt.AddBoundingBoxConstraint(minVal,maxVal,xi(positions_map.at("joint_" + std::to_string(iJoint))));
  }
}

template <typename T> 
void setSpiritJointLimits(drake::multibody::MultibodyPlant<T> & plant, 
                          dairlib::systems::trajectory_optimization::Dircon<T>& trajopt,  
                          std::vector<int> iJoints, 
                          std::vector<double> minVals, 
                          std::vector<double> maxVals  ){
  int numJoints = iJoints.size();
  assert(numJoints==minVals.size());
  assert(numJoints==maxVals.size());
  for (int i = 0; i<numJoints; i++){
    setSpiritJointLimits( plant, 
                          trajopt,  
                          iJoints[i], 
                          minVals[i], 
                          maxVals[i] );
  }
}
template <typename T> 
void setSpiritJointLimits(drake::multibody::MultibodyPlant<T> & plant, 
                          dairlib::systems::trajectory_optimization::Dircon<T>& trajopt,  
                          std::vector<int> iJoints, 
                          double minVal, 
                          double maxVal  ){

  for (int iJoint: iJoints){
    setSpiritJointLimits( plant, 
                          trajopt,  
                          iJoint, 
                          minVal, 
                          maxVal );
  }
}
template <typename T> 
void setSpiritJointLimits(drake::multibody::MultibodyPlant<T> & plant, 
                          dairlib::systems::trajectory_optimization::Dircon<T>& trajopt ){
  // Upper doesn't need a joint limit for now, but we may eventually want to
  // change this to help the optimizations
  // double minValUpper = -M_PI ;
  // double maxValUpper =  M_PI ;

  // Lower limits are set to 0 and pi in the URDF might be better to include
  // the few degrees that come with the body collision and to remove a few to stay
  // away from singularity
  double minValLower =  0;
  double maxValLower =  M_PI;
  
  // The URDF defines symmetric limits if asymmetric constraints we need to
  // add a mirror since the hips are positive in the same direction
  double abKinLimit = 0.707 ; //From the URDF
  double minValAbduc = -abKinLimit ;
  double maxValAbduc =  abKinLimit ;
  
  // Upper 0,2,4,6
  std::vector<int> indicesUpper = {0,2,4,6};
  // Lower 1,3,5,7
  std::vector<int> indicesLower = {1,3,5,7};
  // Hips  8,9,10,11
  std::vector<int> indicesAbduc = {8,9,10,11};

//See above: Upper doesn't need joint limit since there isnt a meaningful set
// setSpiritJointLimits( plant, 
//                      trajopt,  
//                      indicesUpper, 
//                      minValUpper, 
//                      maxValUpper  );
                     
setSpiritJointLimits( plant, 
                     trajopt,  
                     indicesLower, 
                     minValLower, 
                     maxValLower  );
                     
setSpiritJointLimits( plant, 
                     trajopt,  
                     indicesAbduc, 
                     minValAbduc, 
                     maxValAbduc  );

}
//   \SETSPIRITJOINTLIMITS 


// **********************************************************
//   SETSPIRITACTUATIONLIMITS
template <typename T>  
void setSpiritActuationLimits(drake::multibody::MultibodyPlant<T> & plant, 
                          dairlib::systems::trajectory_optimization::Dircon<T>& trajopt,
                          double actuatorLimit){
  auto actuators_map = multibody::makeNameToActuatorsMap(plant);
  // std::cout<<"joint_" + std::to_string(iJoint)<<std::endl;
  int N_knotpoints = trajopt.N();
  for(int iKnot = 0; iKnot<N_knotpoints;iKnot++){
    auto ui = trajopt.input(iKnot);
    for (int iMotor = 0; iMotor<12; iMotor++){
      trajopt.AddBoundingBoxConstraint(-actuatorLimit,actuatorLimit,ui(actuators_map.at("motor_" + std::to_string(iMotor))));
    }
  }

}
//   \SETSPIRITACTUATIONLIMITS


// **********************************************************
//   SETSPIRITSYMMETRY

template <typename T>
void makeSaggitalSymmetric(
        drake::multibody::MultibodyPlant<T> & plant, 
        dairlib::systems::trajectory_optimization::Dircon<T>& trajopt){

  // Get position and velocity dictionaries 
  auto positions_map = multibody::makeNameToPositionsMap(plant);
  auto velocities_map = multibody::makeNameToVelocitiesMap(plant);
  int num_knotpoints = trajopt.N();
  /// Symmetry constraints
  for (int i = 0; i < num_knotpoints; i++){
    
    auto xi = trajopt.state(i);  
    // Symmetry constraints (mirrored hips all else equal across)
    trajopt.AddLinearConstraint( xi( positions_map.at("joint_8") ) == -xi( positions_map.at("joint_10") ) );
    trajopt.AddLinearConstraint( xi( positions_map.at("joint_9") ) == -xi( positions_map.at("joint_11") ) );
    // // Front legs equal and back legs equal constraints 
    trajopt.AddLinearConstraint( xi( positions_map.at("joint_0") ) ==  xi( positions_map.at("joint_4") ) );
    trajopt.AddLinearConstraint( xi( positions_map.at("joint_1") ) ==  xi( positions_map.at("joint_5") ) );
    trajopt.AddLinearConstraint( xi( positions_map.at("joint_2") ) ==  xi( positions_map.at("joint_6") ) );
    trajopt.AddLinearConstraint( xi( positions_map.at("joint_3") ) ==  xi( positions_map.at("joint_7") ) );
  }
}
template <typename T>
void setSpiritSymmetry(
        drake::multibody::MultibodyPlant<T> & plant, 
        dairlib::systems::trajectory_optimization::Dircon<T>& trajopt,
        std::vector<std::string> symmetries){
  
  
  static bool hasRun = false;
  if(hasRun){
    std::cerr << "Warning: Running different symmetries seperately could cause system to be overconstrained. Use vector overload to avoid this." 
              << std::endl;
  }else{
    hasRun = true;
  }
  bool hasSaggital = false;
  bool hasForeAft  = false;
  hasSaggital = std::any_of(symmetries.begin(), symmetries.end(), [](const std::string & str) {
                                                        return str == "sagittal";
                                                    });
                                                    
  hasForeAft = std::any_of(symmetries.begin(), symmetries.end(), [](const std::string & str) {
                                                        return str == "foreaft";
                                                    });
  if (hasSaggital&&hasForeAft){
    // Avoids having overconstrained system of symmetry
    std::cerr << "Warning: Simultaneous Saggital and ForeAft not implemented yet. Only adding saggital constraints." << std::endl;
    makeSaggitalSymmetric(plant,trajopt);
  }else if (hasSaggital){
    makeSaggitalSymmetric(plant,trajopt);
  }else if (hasForeAft){
    std::cerr << "Warning: ForeAft not implemented yet. No constraints added" << std::endl;
  }else{
    std::cerr << "Warning: Failed to add symmetric constraint. No constraints added" << std::endl;
  }

}

template <typename T>
void setSpiritSymmetry(drake::multibody::MultibodyPlant<T> & plant, 
                       dairlib::systems::trajectory_optimization::Dircon<T>& trajopt,
                       std::string symmetry){
                         
  std::vector<std::string> symmetries;
  symmetries.push_back(symmetry);
  setSpiritSymmetry(
                    plant,
                    trajopt,
                    symmetries);
}
//   \SETSPIRITSYMMETRY



template void nominalSpiritStand(
    drake::multibody::MultibodyPlant<double>& plant, 
    Eigen::VectorXd& xState, 
    double height); // NOLINT 
    
template const drake::multibody::Frame<double>& getSpiritToeFrame( 
    drake::multibody::MultibodyPlant<double>& plant, 
    u_int8_t toeIndex ); // NOLINT 

template std::unique_ptr<multibody::WorldPointEvaluator<double>> getSpiritToeEvaluator( 
                      MultibodyPlant<double>& plant, 
                      const Eigen::Vector3d toePoint ,
                      u_int8_t toeIndex,
                      const Eigen::Vector3d normal ,
                      const Eigen::Vector3d offset ,
                      bool xy_active, 
                      double mu 
                      ); // NOLINT
                      
template std::tuple<  std::vector<std::unique_ptr<dairlib::systems::trajectory_optimization::DirconMode<double>>>,
             std::vector<std::unique_ptr<multibody::WorldPointEvaluator<double>>> ,
             std::vector<std::unique_ptr<multibody::KinematicEvaluatorSet<double>>>   
          >     
    createSpiritModeSequence( 
          drake::multibody::MultibodyPlant<double>& plant, // multibodyPlant
          std::vector<Eigen::Matrix<bool,1,4>> modeSeqVect, // bool matrix describing toe contacts as true or false e.g. {{1,1,1,1},{0,0,0,0}} would be a full support mode and flight mode
          std::vector<int> knotpointVect, // Matrix of knot points for each mode  
          std::vector<Eigen::Vector3d> normals,
          std::vector<Eigen::Vector3d> offsets, 
          std::vector<double> mus ); // NOLINT
          
  
template void setSpiritJointLimits(
          drake::multibody::MultibodyPlant<double> & plant, 
          dairlib::systems::trajectory_optimization::Dircon<double>& trajopt,  
          int iJoint, 
          double minVal, 
          double maxVal  );

template void setSpiritJointLimits(
          drake::multibody::MultibodyPlant<double> & plant, 
          dairlib::systems::trajectory_optimization::Dircon<double>& trajopt,  
          std::vector<int> iJoints, 
          std::vector<double> minVals, 
          std::vector<double> maxVals  );

template void setSpiritJointLimits(
          drake::multibody::MultibodyPlant<double> & plant, 
          dairlib::systems::trajectory_optimization::Dircon<double>& trajopt,  
          std::vector<int> iJoints, 
          double minVal, 
          double maxVal  );

template void setSpiritJointLimits(
          drake::multibody::MultibodyPlant<double> & plant, 
          dairlib::systems::trajectory_optimization::Dircon<double>& trajopt );

template void setSpiritActuationLimits(
          drake::multibody::MultibodyPlant<double> & plant, 
          dairlib::systems::trajectory_optimization::Dircon<double>& trajopt,
          double actuatorLimit);

template void setSpiritSymmetry(
        drake::multibody::MultibodyPlant<double> & plant, 
        dairlib::systems::trajectory_optimization::Dircon<double>& trajopt,
        std::string symmetry = "sagittal");

template void setSpiritSymmetry(
        drake::multibody::MultibodyPlant<double> & plant, 
        dairlib::systems::trajectory_optimization::Dircon<double>& trajopt,
        std::vector<std::string> symmetries);

}//namespace dairlib