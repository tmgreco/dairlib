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
#include "drake/geometry/drake_visualizer.h"
#include "drake/solvers/solve.h"

#include "common/find_resource.h"
#include "systems/trajectory_optimization/dircon/dircon.h"
#include "multibody/kinematic/world_point_evaluator.h"
#include "solvers/nonlinear_constraint.h"
#include "multibody/kinematic/distance_evaluator.h"
#include "multibody/multibody_utils.h"
#include "multibody/visualization_utils.h"
#include "multibody/kinematic/kinematic_constraints.h"
#include "solvers/nonlinear_cost.h"
#include "examples/Spirit_spine/spirit_utils.h"


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

void ikSpiritStand(drake::multibody::MultibodyPlant<double>& plant,
                   Eigen::VectorXd& xState,
                   Eigen::Matrix<bool,1,4> contactSequence,
                   double com_height,
                   double leg_height,
                   double roll,
                   double pitch,
                   double eps){
  drake::multibody::InverseKinematics ik(plant);
  const auto& world_frame = plant.world_frame();
  const auto& body_frame  = plant.GetFrameByName("body");

  // Position of the com
  Vector3d pos = {0, 0, com_height};
  // Orientation of body
  drake::math::RotationMatrix<double> orientation;
  orientation = drake::math::RotationMatrix<double>::MakeXRotation(-roll) *
                drake::math::RotationMatrix<double>::MakeYRotation(-pitch) ;

  const double n_x = plant.num_positions() + plant.num_velocities();
  const double n_q = plant.num_positions();


  // Constrain body location and pose
  ik.AddPositionConstraint(body_frame, Vector3d(0, 0, 0), world_frame,
                           pos,
                           pos);
  ik.AddOrientationConstraint(body_frame, orientation,
                              world_frame, drake::math::RotationMatrix<double>(), 0);

  // Constrain toes
  for(int toe = 0; toe < 4; toe ++){
    constrainToe(plant, ik, toe, contactSequence[toe], orientation, com_height, leg_height, eps);
  }

  // Add initial guess and solve
  Eigen::VectorXd initial_guess;
  dairlib::nominalSpiritStand(plant, initial_guess, com_height);
  ik.get_mutable_prog()->SetInitialGuess(ik.q(), initial_guess.head(n_q));
  const auto result = Solve(ik.prog());
  const auto q_sol = result.GetSolution(ik.q());
  std::cout << (result.is_success() ? "IK Success" : "IK Fail") << std::endl;

  // Return
  VectorXd x_const = VectorXd::Zero(n_x);
  x_const.head(n_q) = q_sol;
  xState = x_const;
}

void constrainToe(drake::multibody::MultibodyPlant<double> & plant,
                  drake::multibody::InverseKinematics &ik,
                  int toe, bool inContact, drake::math::RotationMatrix<double> orientation,
                  double com_height, double leg_height, double eps){
  // Get frames
  const auto& toe_frame = dairlib::getSpiritToeFrame(plant, toe);
  const auto& world_frame = plant.world_frame();
  const auto& body_frame  = plant.GetFrameByName("body");
  Vector3d eps_vec = {eps, eps, eps};

  // Chose distance between toe and com
  const double hip_width = 0.07;
  const double body_length = 0.2263;
  Vector3d toe_offset;
  switch (toe) {
    case 0:
      toe_offset = {body_length, hip_width, -abs(leg_height)};
      break;
    case 1:
      toe_offset = {-body_length, hip_width, -abs(leg_height)};
      break;
    case 2:
      toe_offset = {body_length, -hip_width, -abs(leg_height)};
      break;
    case 3:
      toe_offset = {-body_length, -hip_width, -abs(leg_height)};
      break;
    default:
      break;
  }

  if(inContact){
    // Toe is in contact with ground

    // Constrain toe to be under hip with distance constraint
    toe_offset(2) = 0; // vector between body and hip in body frame
    // Location hip in world frame
    Vector3d hip_in_world = orientation.transpose() * toe_offset + Vector3d({0, 0, abs(com_height)});
    // Vector between hip and toe in world frame
    Vector3d  hip_to_toe_world = {0, 0, -hip_in_world(2)};
    // Vector between body and toe in body frame
    Vector3d body_to_toe_body = toe_offset + orientation * hip_to_toe_world;
    //ik.AddPositionConstraint(body_frame, body_to_toe_body, toe_frame, -eps_vec, eps_vec);

    // Toe is in contact with ground
    ik.AddPositionConstraint(toe_frame, Vector3d(0, 0, 0), world_frame,
                             toe_offset,
                             toe_offset);

  }
  else{
    // Toe is under hip in body frame
    ik.AddPositionConstraint(body_frame, toe_offset, toe_frame,
                             Vector3d(0, 0, 0),
                             Vector3d(0, 0, 0));
  }
}

template <typename T>
void nominalSpiritStandConstraint(
    drake::multibody::MultibodyPlant<T>& plant,
    dairlib::systems::trajectory_optimization::Dircon<T>& trajopt,
    double height,
    std::vector<int> knotPoints,
    double eps){
  int nx = plant.num_positions() + plant.num_velocities();
  VectorXd x_state(nx);
  dairlib::nominalSpiritStand( plant, x_state,  height); //get desired x_state

  auto positions_map = dairlib::multibody::makeNameToPositionsMap(plant);

  for(int knot_index : knotPoints){
    auto xi = trajopt.state(knot_index);
    for(int joint = 0; joint< 12; joint ++){
      std::string joint_string = "joint_"+std::to_string(joint);
      double joint_val = x_state[positions_map.at(joint_string)];
      trajopt.AddBoundingBoxConstraint(joint_val-eps, joint_val+eps, xi(positions_map.at(joint_string)));
    }
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
  if(!std::isinf(mu)){
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
          std::vector<double> mus,
          std::vector<double> minTs,
          std::vector<double> maxTs ){

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
        toeEvals.push_back( std::move( getSpiritToeEvaluator(plant, toeOffset, iLeg, normals.at(iMode), offsets.at(iMode), true, mus.at(iMode) )) );
        (toeEvalSets.back())->add_evaluator(  (toeEvals.back()).get()  ); //add evaluator to the set if active //Works ish
      }
    }
    auto dumbToeEvalPtr = (toeEvalSets.back()).get() ;
    int num_knotpoints = knotpointVect.at(iMode);
    modeVector.push_back(std::move( std::make_unique<DirconMode<T>>( *dumbToeEvalPtr , num_knotpoints, minTs.at(iMode), maxTs.at(iMode) )));
    // DirconMode<T> modeDum = DirconMode<T>( *dumbToeEvalPtr , num_knotpoints );
    // sequence.AddMode(  &modeDum  ); // Add the evaluator set to the mode sequence

  }

  return {std::move(modeVector), std::move(toeEvals), std::move(toeEvalSets)};
  // return {std::move(toeEvals), std::move(toeEvalSets)};
}


template <typename T>
std::tuple<
                std::vector<std::unique_ptr<DirconMode<T>>>,
                std::vector<std::unique_ptr<dairlib::multibody::WorldPointEvaluator<T>>> ,
                std::vector<std::unique_ptr<dairlib::multibody::KinematicEvaluatorSet<T>>>
          > createSpiritModeSequence(
          MultibodyPlant<T>& plant, // multibodyPlant
          const dairlib::ModeSequenceHelper& msh ){

return createSpiritModeSequence(plant, msh.modes , msh.knots , msh.normals , msh.offsets, msh.mus, msh.minTs, msh.maxTs);
}




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
                          dairlib::systems::trajectory_optimization::Dircon<T>& trajopt,
                          float knee_limit){
  // Upper doesn't need a joint limit for now, but we may eventually want to
  // change this to help the optimizations
  double minValUpper = -2 * M_PI ;
  double maxValUpper =  2 * M_PI ;

  // Lower limits are set to 0 and pi in the URDF might be better to include
  // the few degrees that come with the body collision and to remove a few to stay
  // away from singularity
  double minValLower =  0.2;
  double maxValLower =  knee_limit;

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
setSpiritJointLimits( plant,
                     trajopt,
                     indicesUpper,
                     minValUpper,
                     maxValUpper  );

setSpiritJointLimits( plant,
                     trajopt,
                     indicesLower,
                     minValLower ,
                     maxValLower );

setSpiritJointLimits( plant,
                     trajopt,
                     indicesAbduc,
                     minValAbduc,
                     maxValAbduc  );
if (plant.num_actuators()==13) setSpiritJointLimits( plant,
                     trajopt,
                     {12},
                     -M_PI/2,
                     M_PI/2 );
}
//   \SETSPIRITJOINTLIMITS

template <typename T>
void setSpiritActuationSpeedCurveLimits(
          drake::multibody::MultibodyPlant<T> & plant,
          dairlib::systems::trajectory_optimization::Dircon<T>& trajopt){
  auto actuators_map = multibody::makeNameToActuatorsMap(plant);
  auto velocities_map = multibody::makeNameToVelocitiesMap(plant);
  int n_q = plant.num_positions();

  std::vector<int> indicesNotKnee = {0,2,4,6,8,9,10,11};
  std::vector<int> indicesKnee = {1,3,5,7};

  int N_knotpoints = trajopt.N();
  for(int iKnot = 0; iKnot<N_knotpoints;iKnot++){
    auto ui = trajopt.input(iKnot);

    for (int iMotor : indicesNotKnee){
      int vel_int = n_q + velocities_map.at("joint_" + std::to_string(iMotor) +"dot");
      auto vi = trajopt.state(iKnot)(vel_int);
      trajopt.AddConstraint(max(-torque_stall_not_knee/omega_noload_not_knee*vi+torque_stall_not_knee,0)-ui(actuators_map.at("motor_" + std::to_string(iMotor)))>=0);
      trajopt.AddConstraint(ui(actuators_map.at("motor_" + std::to_string(iMotor)))-min(-torque_stall_not_knee/omega_noload_not_knee*vi-torque_stall_not_knee,0)>=0);
    }
    for (int iMotor : indicesKnee){
      int vel_int = n_q + velocities_map.at("joint_" + std::to_string(iMotor) +"dot");
      auto vi = trajopt.state(iKnot)(vel_int);
      trajopt.AddConstraint(max(-torque_stall_knee/omega_noload_knee*vi+torque_stall_knee,0)-ui(actuators_map.at("motor_" + std::to_string(iMotor)))>=0);
      trajopt.AddConstraint(ui(actuators_map.at("motor_" + std::to_string(iMotor)))-min(-torque_stall_knee/omega_noload_knee*vi-torque_stall_knee,0)>=0);
    }

  }
 }

// **********************************************************
//   SETSPIRITACTUATIONLIMITS
template <typename T>
void setSpiritActuationLimits(drake::multibody::MultibodyPlant<T> & plant,
                          dairlib::systems::trajectory_optimization::Dircon<T>& trajopt,
                          double actuatorLimit){
  auto actuators_map = multibody::makeNameToActuatorsMap(plant);

  // Upper 0,2,4,6
  std::vector<int> indicesUpper = {0,2,4,6};
  // Lower 1,3,5,7
  std::vector<int> indicesLower = {1,3,5,7};
  // Hips  8,9,10,11
  std::vector<int> indicesAbduc = {8,9,10,11};

  double mechanicalReductionKnee = 1.5;


  int N_knotpoints = trajopt.N();
  for(int iKnot = 0; iKnot<N_knotpoints;iKnot++){
    auto ui = trajopt.input(iKnot);
    for (int iMotor : indicesUpper){
      trajopt.AddBoundingBoxConstraint(
        -actuatorLimit,
         actuatorLimit,
         ui(actuators_map.at("motor_" + std::to_string(iMotor))));
    }
    for (int iMotor : indicesLower){
      trajopt.AddBoundingBoxConstraint(
        -actuatorLimit*mechanicalReductionKnee,
         actuatorLimit*mechanicalReductionKnee,
         ui(actuators_map.at("motor_" + std::to_string(iMotor))));
    }
    for (int iMotor : indicesAbduc){
      trajopt.AddBoundingBoxConstraint(
        -actuatorLimit,
         actuatorLimit,
         ui(actuators_map.at("motor_" + std::to_string(iMotor))));
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
    std::cerr << "Warning: Failed to add symmetric constraint. Check spelling or implemenation. No constraints added" << std::endl;
  }

}

template <typename T>
void setSpiritSymmetry(drake::multibody::MultibodyPlant<T> & plant,
                       dairlib::systems::trajectory_optimization::Dircon<T>& trajopt,
                       std::string symmetry,
                       bool ignoreWarning){

  static bool hasRun = false;
  if(hasRun && !ignoreWarning){
    std::cerr << "Warning: Running different symmetries seperately could cause system to be overconstrained. Use vector overload to avoid this or set ignoreWarning flag if not a problem."
              << std::endl;
  }else{
    hasRun = true;
  }

  std::vector<std::string> symmetries;
  symmetries.push_back(symmetry);
  setSpiritSymmetry(
                    plant,
                    trajopt,
                    symmetries);
}
//   \SETSPIRITSYMMETRY

// drake::symbolic::Expression getYawFromQuaternion(
//       Eigen::VectorBlock<const drake::solvers::VectorXDecisionVariable> w,
//       Eigen::VectorBlock<const drake::solvers::VectorXDecisionVariable> x,
//       Eigen::VectorBlock<const drake::solvers::VectorXDecisionVariable> y,
//       Eigen::VectorBlock<const drake::solvers::VectorXDecisionVariable> z){
//         // drake::symbolic::Expression t0=2.0 * (w * z + x * y);
//         // drake::symbolic::Expression t1=1 - 2* (y * y + z * z);
//         // drake::symbolic::Expression yaw = atan2(t0, t1);
//         return yaw;
//       }

// drake::symbolic::Expression getRollFromQuaternion(
//       Eigen::VectorBlock<const drake::solvers::VectorXDecisionVariable> w,
//       Eigen::VectorBlock<const drake::solvers::VectorXDecisionVariable> x,
//       Eigen::VectorBlock<const drake::solvers::VectorXDecisionVariable> y,
//       Eigen::VectorBlock<const drake::solvers::VectorXDecisionVariable> z){
//     // drake::symbolic::Expression roll = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
//     drake::symbolic::Expression roll =x+y;
//     return roll;
//     }

// drake::symbolic::Expression getPitchFromQuaternion(
//       Eigen::VectorBlock<const drake::solvers::VectorXDecisionVariable> w,
//       Eigen::VectorBlock<const drake::solvers::VectorXDecisionVariable> x,
//       Eigen::VectorBlock<const drake::solvers::VectorXDecisionVariable> y,
//       Eigen::VectorBlock<const drake::solvers::VectorXDecisionVariable> z) {
//         // drake::symbolic::Expression pitch = asin(2.0 * (w * y - z * x));
//         drake::symbolic::Expression pitch =x+y;
//         return pitch;
//       }

drake::math::RotationMatrix<double> normal2Rotation(Eigen::Vector3d nHat){
  const double eps  = 1e-6;
  drake::math::RotationMatrix<double> rotMat;
  assert(std::abs(nHat.squaredNorm() - 1) < eps );
  if       ( std::abs(nHat.dot(Eigen::Vector3d::UnitZ()) - 1) < eps){ //If close to +unitZ dont rotate
    rotMat = drake::math::RotationMatrix<double>();
  }else if ( std::abs(nHat.dot(Eigen::Vector3d::UnitZ()) + 1) < eps){ //If close to -unitZ dont rotate
    Eigen::Matrix3d R;
    R <<  1,  0,  0,
          0, -1,  0,
          0,  0, -1;
    rotMat = drake::math::RotationMatrix<double>(R);
  }else if ( std::abs(nHat.dot(Eigen::Vector3d::UnitX())) <= std::abs(nHat.dot(Eigen::Vector3d::UnitY())) ){
    Eigen::Vector3d nyHat =  nHat.cross(Eigen::Vector3d::UnitX());
    Eigen::Vector3d nxHat =  nyHat.cross(nHat);
    Eigen::Matrix3d R;
    R.col(0) << nxHat;
    R.col(1) << nyHat;
    R.col(2) << nHat;
    rotMat = drake::math::RotationMatrix<double>(R);
  }else if ( std::abs(nHat.dot(Eigen::Vector3d::UnitX())) > std::abs(nHat.dot(Eigen::Vector3d::UnitY())) ){
    Eigen::Vector3d nxHat = -nHat.cross(Eigen::Vector3d::UnitY());
    Eigen::Vector3d nyHat =  nHat.cross(nxHat);
    Eigen::Matrix3d R;
    R.col(0) << nxHat;
    R.col(1) << nyHat;
    R.col(2) << nHat;
    rotMat = drake::math::RotationMatrix<double>(R);
  }else{
    std::cout<<"Something went wrong check here"<<std::endl;
    rotMat = drake::math::RotationMatrix<double>();
  }
  return rotMat;
}

template <typename T>
double calcWork(
    drake::multibody::MultibodyPlant<T> & plant,
    drake::trajectories::PiecewisePolynomial<double>& x_traj,
    drake::trajectories::PiecewisePolynomial<double>& u_traj){

  auto velocities_map = multibody::makeNameToVelocitiesMap(plant);
  auto actuator_map = multibody::makeNameToActuatorsMap(plant);
  int n_q = plant.num_positions();

  double work = 0;
  std::vector<double> knot_points = x_traj.get_segment_times();

  for(int knot_index = 0; knot_index < knot_points.size()-1; knot_index++){
      auto u_low = u_traj.value(knot_points[knot_index]);
      auto u_up  = u_traj.value(knot_points[knot_index + 1]);
      auto x_low = x_traj.value(knot_points[knot_index]);
      auto x_up  = x_traj.value(knot_points[knot_index + 1]);

    for (int joint = 0; joint < plant.num_actuators(); joint++){

      double actuation_low = u_low(actuator_map.at("motor_" + std::to_string(joint)));
      double actuation_up  =  u_up(actuator_map.at("motor_" + std::to_string(joint)));

      double velocity_low = x_low(n_q + velocities_map.at("joint_" + std::to_string(joint) +"dot"));
      double velocity_up  =  x_up(n_q + velocities_map.at("joint_" + std::to_string(joint) +"dot"));

      double pow_low = abs(actuation_low*velocity_low);
      double pow_up  = abs(actuation_up*velocity_up);

      // trapazoidal integration
      work += (knot_points[knot_index + 1] - knot_points[knot_index])/2.0 * (pow_low + pow_up);
    }
  }
  return work;
}

template <typename T>
double calcMechanicalWork(
    drake::multibody::MultibodyPlant<T> & plant,
    std::vector<drake::trajectories::PiecewisePolynomial<double>>& x_trajs,
    drake::trajectories::PiecewisePolynomial<double>& u_traj){

  auto velocities_map = multibody::makeNameToVelocitiesMap(plant);
  auto actuator_map = multibody::makeNameToActuatorsMap(plant);
  int n_q = plant.num_positions();

  double work = 0;
  for(const auto& x_traj : x_trajs) {
    std::vector<double> knot_points = x_traj.get_segment_times();
    for (int knot_index = 0; knot_index < knot_points.size() - 1; knot_index++) {
      auto u_low = u_traj.value(knot_points[knot_index]);
      auto u_up = u_traj.value(knot_points[knot_index + 1]);
      auto x_low = x_traj.value(knot_points[knot_index]);
      auto x_up = x_traj.value(knot_points[knot_index + 1]);

      for (int joint = 0; joint < plant.num_actuators(); joint++) {

        double actuation_low = u_low(actuator_map.at("motor_" + std::to_string(joint)));
        double actuation_up = u_up(actuator_map.at("motor_" + std::to_string(joint)));

        double velocity_low = x_low(n_q + velocities_map.at("joint_" + std::to_string(joint) + "dot"));
        double velocity_up = x_up(n_q + velocities_map.at("joint_" + std::to_string(joint) + "dot"));

        double pow_low = abs(actuation_low * velocity_low);
        double pow_up = abs(actuation_up * velocity_up);

        // trapazoidal integration
        work += (knot_points[knot_index + 1] - knot_points[knot_index]) / 2.0 * (pow_low + pow_up);
      }
    }
  }
  return work;
}


template <typename T>
double calcElectricalWork(
    drake::multibody::MultibodyPlant<T> & plant,
    std::vector<drake::trajectories::PiecewisePolynomial<double>>& x_trajs,
    drake::trajectories::PiecewisePolynomial<double>& u_traj,
    double efficiency){

  auto velocities_map = multibody::makeNameToVelocitiesMap(plant);
  auto actuator_map = multibody::makeNameToActuatorsMap(plant);
  auto positions_map = multibody::makeNameToPositionsMap(plant);
  int n_q = plant.num_positions();

  double work = 0;
  double thermal_loss=0;
  double Q;
  for(const auto& x_traj : x_trajs) {
    std::vector<double> knot_points = x_traj.get_segment_times();
    for (int knot_index = 0; knot_index < knot_points.size() - 1; knot_index++) {
      auto u_low = u_traj.value(knot_points[knot_index]);
      auto u_up = u_traj.value(knot_points[knot_index + 1]);
      auto x_low = x_traj.value(knot_points[knot_index]);
      auto x_up = x_traj.value(knot_points[knot_index + 1]);

      for (int joint = 0; joint < plant.num_actuators(); joint++) {
        if(joint == 1 or joint == 3 or joint == 5 or joint == 7){
          Q = Q_knee;
        }
        else{
          Q = Q_not_knee;
        }

        double actuation_low = u_low(actuator_map.at("motor_" + std::to_string(joint)));
        double actuation_up = u_up(actuator_map.at("motor_" + std::to_string(joint)));

        double position_low = x_low(positions_map.at("joint_" + std::to_string(joint)));
        double position_up = x_up(positions_map.at("joint_" + std::to_string(joint)));

        double velocity_low = x_low(n_q + velocities_map.at("joint_" + std::to_string(joint) + "dot"));
        double velocity_up = x_up(n_q + velocities_map.at("joint_" + std::to_string(joint) + "dot"));
        if (joint == 12) {
            double k = 45.01;
            double b = 0;
            actuation_low += b*velocity_low + k*position_low;
            actuation_up += b*velocity_up + k*position_up;
        }

        double pow_low = actuation_low * velocity_low + Q * actuation_low * actuation_low;
        double pow_up = actuation_up * velocity_up+ Q * actuation_up * actuation_up;

        double battery_pow_low = positivePart(pow_low) + efficiency * negativePart(pow_low);
        double battery_pow_up = positivePart(pow_up) + efficiency * negativePart(pow_up);

        // trapazoidal integration
        work += (knot_points[knot_index + 1] - knot_points[knot_index]) / 2.0 * (battery_pow_low + battery_pow_up);


        double pow_loss_low = Q * actuation_low * actuation_low;
        double pow_loss_up =  Q * actuation_up * actuation_up;
        thermal_loss += (knot_points[knot_index + 1] - knot_points[knot_index]) / 2.0 * (pow_loss_low + pow_loss_up);
      }
    }
  }
  std::cout<<"Thermal loss: "<< thermal_loss<< std::endl;
  return work;
}


template <typename T>
double calcVelocityInt(
    drake::multibody::MultibodyPlant<T> & plant,
    drake::trajectories::PiecewisePolynomial<double>& x_traj){

  auto velocities_map = multibody::makeNameToVelocitiesMap(plant);
  int n_v = plant.num_velocities();

  double vel_int = 0;
  std::vector<double> knot_points = x_traj.get_segment_times();

  for(int knot_index = 0; knot_index < knot_points.size()-1; knot_index++){
    Eigen::VectorXd x_low = x_traj.value(knot_points[knot_index]);
    Eigen::VectorXd x_up  = x_traj.value(knot_points[knot_index + 1]);

    Eigen::VectorXd velocity_low = x_low.tail(n_v);
    Eigen::VectorXd velocity_up  =  x_up.tail(n_v);

    double vel_sq_low = velocity_low.transpose() * velocity_low;
    double vel_sq_up  = velocity_up.transpose()  * velocity_up;

    // trapazoidal integration
    vel_int += (knot_points[knot_index + 1] - knot_points[knot_index])/2.0 * (vel_sq_low  + vel_sq_up);
  }

  return vel_int;
}


template <typename T>
double calcVelocityInt(
    drake::multibody::MultibodyPlant<T> & plant,
    std::vector<drake::trajectories::PiecewisePolynomial<double>>& x_trajs){

  auto velocities_map = multibody::makeNameToVelocitiesMap(plant);
  int n_v = plant.num_velocities();

  double vel_int = 0;

  for(const auto& x_traj : x_trajs){
    std::vector<double> knot_points = x_traj.get_segment_times();

    for(int knot_index = 0; knot_index < knot_points.size()-1; knot_index++){
      Eigen::VectorXd x_low = x_traj.value(knot_points[knot_index]);
      Eigen::VectorXd x_up  = x_traj.value(knot_points[knot_index + 1]);

      Eigen::VectorXd velocity_low = x_low.tail(n_v);
      Eigen::VectorXd velocity_up  =  x_up.tail(n_v);

      double vel_sq_low = velocity_low.transpose() * velocity_low;
      double vel_sq_up  = velocity_up.transpose()  * velocity_up;

      // trapazoidal integration
      vel_int += (knot_points[knot_index + 1] - knot_points[knot_index])/2.0 * (vel_sq_low  + vel_sq_up);
    }

  }
  return vel_int;
}

template <typename T>
double calcTorqueInt(
    drake::multibody::MultibodyPlant<T> & plant,
    drake::trajectories::PiecewisePolynomial<double>& u_traj){

  auto actuator_map = multibody::makeNameToActuatorsMap(plant);

  double act_int = 0;
  std::vector<double> knot_points = u_traj.get_segment_times();

  for(int knot_index = 0; knot_index < knot_points.size()-1; knot_index++){
    Eigen::VectorXd u_low = u_traj.value(knot_points[knot_index]);
    Eigen::VectorXd u_up  = u_traj.value(knot_points[knot_index + 1]);

    double act_sq_low = u_low.transpose() * u_low;
    double act_sq_up  = u_up.transpose() * u_up;

    // trapazoidal integration
    act_int += (knot_points[knot_index + 1] - knot_points[knot_index])/2.0 * (act_sq_low + act_sq_up);
  }
  return act_int;
}

template <typename T>
std::vector<drake::solvers::Binding<drake::solvers::Cost>> AddWorkCost(drake::multibody::MultibodyPlant<T> & plant,
                 dairlib::systems::trajectory_optimization::Dircon<T>& trajopt,
                 double cost_work_gain,
                 std::string work_type){
  std::vector<drake::solvers::Binding<drake::solvers::Cost>> cost_joint_work_bindings;
  int n_q = plant.num_positions();

  auto velocities_map = multibody::makeNameToVelocitiesMap(plant);
  auto actuator_map = multibody::makeNameToActuatorsMap(plant);
  double Q = 0;
  // Loop through each joint
  for (int joint = 0; joint < plant.num_actuators(); joint++) {
    if(joint == 1 or joint == 3 or joint == 5 or joint == 7)
      Q = Q_knee;
    else
      Q = Q_not_knee;
    auto joint_work_cost = std::make_shared<JointWorkCost>(plant,  Q, cost_work_gain,4,work_type);
    int act_int = actuator_map.at("motor_" + std::to_string(joint));
    int vel_int = n_q + velocities_map.at("joint_" + std::to_string(joint) +"dot");
    // Loop through each mode
    for (int mode_index = 0; mode_index < trajopt.num_modes(); mode_index++) {
      for (int knot_index = 0; knot_index < trajopt.mode_length(mode_index)-1; knot_index++) {
        auto u_i = trajopt.input(trajopt.get_mode_start(mode_index) + knot_index)(act_int);
        auto x_i   = trajopt.state_vars(mode_index, knot_index)(vel_int);
        auto u_ip = trajopt.input(trajopt.get_mode_start(mode_index) + knot_index+1)(act_int);
        auto x_ip   = trajopt.state_vars(mode_index, knot_index+1)(vel_int);

        auto hi = trajopt.timestep(trajopt.get_mode_start(mode_index) + knot_index);
        drake::solvers::VectorXDecisionVariable variables(5);
        variables(0) = hi[0];
        variables(1) = u_i;
        variables(2) = u_ip;
        variables(3) = x_i;
        variables(4) = x_ip;
        cost_joint_work_bindings.push_back(trajopt.AddCost(joint_work_cost, variables));
      } // knot point loop
    } // Mode loop
  } // Joint loop
  return cost_joint_work_bindings;
}
template <typename T>
std::vector<drake::solvers::Binding<drake::solvers::Cost>> AddPowerCost(drake::multibody::MultibodyPlant<T> & plant,
                 dairlib::systems::trajectory_optimization::Dircon<T>& trajopt,
                 double cost_power_gain, double k, double b){
  std::vector<drake::solvers::Binding<drake::solvers::Cost>> cost_joint_power_bindings;
  int n_q = plant.num_positions();
  auto velocities_map = multibody::makeNameToVelocitiesMap(plant);
  auto actuator_map = multibody::makeNameToActuatorsMap(plant);
  // adding positions_map, necessary for calculating the spring forces
  auto positions_map = multibody::makeNameToPositionsMap(plant);
  double Q = 0;
  // Loop through each joint
  for (int joint = 0; joint < plant.num_actuators(); joint++) {
    if(joint == 1 or joint == 3 or joint == 5 or joint == 7)
      Q = Q_knee;
    else
      Q = Q_not_knee;

    if(joint == 12) { // DIFFERENT COST FOR SPINE
        auto compliant_joint_power_cost = std::make_shared<CompliantJointPowerCost>(plant, trajopt, Q, cost_power_gain,4);
        int act_int = actuator_map.at("motor_" + std::to_string(joint));
        int vel_int = n_q + velocities_map.at("joint_" + std::to_string(joint) +"dot");
        // what is n_q and why is it here?
        // n_q is the number of positions.  So we DON'T want it in pos_int
        // Assuming sensible naming of joints
        int pos_int = positions_map.at("joint_" + std::to_string(joint));
        compliant_joint_power_cost->set_k(k);
        compliant_joint_power_cost->set_b(b);
        // std::cout << "Pos_int: " << pos_int << std::endl;
        // Loop through each mode
        for (int mode_index = 0; mode_index < trajopt.num_modes(); mode_index++) {
          for (int knot_index = 0; knot_index < trajopt.mode_length(mode_index)-1; knot_index++) {
            // Renamed velocities to v_i from x_i, because for this we need position and velocity.
            //
            auto u_i = trajopt.input(trajopt.get_mode_start(mode_index) + knot_index)(act_int);
            auto x_i   = trajopt.state(trajopt.get_mode_start(mode_index) + knot_index)(pos_int);
            auto v_i   = trajopt.state(trajopt.get_mode_start(mode_index) + knot_index)(vel_int);
            auto u_ip = trajopt.input(trajopt.get_mode_start(mode_index) + knot_index+1)(act_int);
            auto x_ip   = trajopt.state(trajopt.get_mode_start(mode_index) + knot_index+1)(pos_int);
            auto v_ip   = trajopt.state(trajopt.get_mode_start(mode_index) + knot_index+1)(vel_int);
            // std::cout << "Got x_i, x_ip" << std::endl;
            auto hi = trajopt.time_vars()(trajopt.get_mode_start(mode_index) + knot_index);

            drake::solvers::VectorXDecisionVariable variables(trajopt.num_modes()+7);
            variables(0) = u_i;
            variables(1) = u_ip;
            variables(2) = v_i;
            variables(3) = v_ip;
            variables(4) = hi;
            variables(5) = x_i;
            variables(6) = x_ip;

            // std::cout << "Defined variables" << std::endl;
            //
            // std::cout<<"Hindex: "<<trajopt.get_mode_start(mode_index)+ knot_index<<" N: "<<trajopt.N()<<std::endl;
            for (int i=0;i < trajopt.num_modes();i++){
              variables((7+i))=trajopt.time_vars()(trajopt.get_mode_start(i) + 2);
              compliant_joint_power_cost->addKnotPoints(trajopt.mode_length(i)-1);
            }
            // std::cout << "After for loop" << std::endl;

            // THIS IS WHERE IT FAILS
            // std::cout << "trajopt.num_modes(): " << trajopt.num_modes() << std::endl;
            // std::cout << "num_rows: " << variables.rows() << std::endl;
            // std::cout << "num_vars: " << compliant_joint_power_cost->num_vars() << std::endl;
            cost_joint_power_bindings.push_back(trajopt.AddCost(compliant_joint_power_cost, variables));
            // std::cout << "Finished adding spine cost" << std::endl;
          } // knot point loop
        } // Mode loop
    } else { // Regular joints get regular cost
        auto joint_power_cost = std::make_shared<JointPowerCost>(plant, trajopt, Q, cost_power_gain,4);
        int act_int = actuator_map.at("motor_" + std::to_string(joint));
        int vel_int = n_q + velocities_map.at("joint_" + std::to_string(joint) +"dot");
        // Loop through each mode
        for (int mode_index = 0; mode_index < trajopt.num_modes(); mode_index++) {
          for (int knot_index = 0; knot_index < trajopt.mode_length(mode_index)-1; knot_index++) {
            auto u_i = trajopt.input(trajopt.get_mode_start(mode_index) + knot_index)(act_int);
            auto x_i   = trajopt.state(trajopt.get_mode_start(mode_index) + knot_index)(vel_int);
            auto u_ip = trajopt.input(trajopt.get_mode_start(mode_index) + knot_index+1)(act_int);
            auto x_ip   = trajopt.state(trajopt.get_mode_start(mode_index) + knot_index+1)(vel_int);

            auto hi = trajopt.time_vars()(trajopt.get_mode_start(mode_index) + knot_index);

            drake::solvers::VectorXDecisionVariable variables(trajopt.num_modes()+5);
            variables(0) = u_i;
            variables(1) = u_ip;
            variables(2) = x_i;
            variables(3) = x_ip;
            variables(4) = hi;
            //
            // std::cout<<"Hindex: "<<trajopt.get_mode_start(mode_index)+ knot_index<<" N: "<<trajopt.N()<<std::endl;
            for (int i=0;i < trajopt.num_modes();i++){
              variables((5+i))=trajopt.time_vars()(trajopt.get_mode_start(i) + 2);
              joint_power_cost->addKnotPoints(trajopt.mode_length(i)-1);
            }
            cost_joint_power_bindings.push_back(trajopt.AddCost(joint_power_cost, variables));
          } // knot point loop
        } // Mode loop
    }
  } // Joint loop
  return cost_joint_power_bindings;
}

// Add power by stride length of segital steady state behavior, which may help to make the Hessian nicer
template <typename T>
std::vector<drake::solvers::Binding<drake::solvers::Cost>> AddDeltaTorqueRegularizationCost(
                 drake::multibody::MultibodyPlant<T> & plant,
                 dairlib::systems::trajectory_optimization::Dircon<T>& trajopt,
                 double regularization_gain){
  std::vector<drake::solvers::Binding<drake::solvers::Cost>> cost_joint_force_bindings;
  int n_q = plant.num_positions();
  auto actuator_map = multibody::makeNameToActuatorsMap(plant);

  auto x0=trajopt.initial_state();
  auto xf=trajopt.final_state();


  // Loop through each joint
  for (int joint = 0; joint < plant.num_actuators(); joint++) {
    auto joint_force_cost = std::make_shared<JointDeltaTorqueRegularizationCost>(plant, trajopt, regularization_gain);
    int act_int = actuator_map.at("motor_" + std::to_string(joint));
    // Loop through each mode
    for (int mode_index = 0; mode_index < trajopt.num_modes(); mode_index++) {
      for (int knot_index = 0; knot_index < trajopt.mode_length(mode_index)-1; knot_index++) {
        auto u_i = trajopt.input(trajopt.get_mode_start(mode_index) + knot_index)(act_int);
        auto u_ip = trajopt.input(trajopt.get_mode_start(mode_index) + knot_index+1)(act_int);
        auto hi = trajopt.time_vars()(trajopt.get_mode_start(mode_index) + knot_index);

        drake::solvers::VectorXDecisionVariable variables(3);
        variables(0) = u_i;
        variables(1) = u_ip;
        variables(2) = hi;
        cost_joint_force_bindings.push_back(trajopt.AddCost(joint_force_cost, variables));
      } // knot point loop
    } // Mode loop
  } // Joint loop
  return cost_joint_force_bindings;
}

JointWorkCost::JointWorkCost(const drake::multibody::MultibodyPlant<double>& plant,
                             const double &Q,
                             const double &cost_work,
                             const double &alpha,
                             const std::string &description)
    :solvers::NonlinearCost<double>(5,description), plant_(plant){
  Q_ = Q;
  cost_work_ = cost_work;
  work_type_=description;
  alpha_ = alpha;
  n_q_ = plant_.num_positions();
  n_v_ = plant_.num_velocities();
  n_u_ = plant_.num_actuators();
  DRAKE_DEMAND(alpha_ > 0);
}

double JointWorkCost::relu_smooth(const double x) const{
  return  x>5 ? x : 1/alpha_ * log(1 + exp(alpha_ * x));
}

void JointWorkCost::EvaluateCost(const Eigen::Ref<const drake::VectorX<double>> &x,
                  drake::VectorX<double> *y) const{

double efficiency_=0;

double h_i = x(0);

double u_i = x(1);
double u_ip = x(2);
double v_i = x(3);
double v_ip = x(4);
double work_low = cost_work_ * relu_smooth(u_i * v_i + Q_ * u_i * u_i);
double work_up = cost_work_ * relu_smooth(u_ip * v_ip + Q_ * u_ip * u_ip);

if (work_type_=="mech_thermal") {
  work_low = cost_work_ * (abs(u_i * v_i) + Q_ * u_i * u_i);
  work_up = cost_work_ * (abs(u_ip * v_ip) + Q_ * u_ip * u_ip);
}

// double work_low = cost_work_ * (u_i * v_i + Q_ * u_i * u_i);
// double work_up = cost_work_ * (u_ip * v_ip + Q_ * u_ip * u_ip);
double battery_pow_low = positivePart(work_low) + efficiency_ * negativePart(work_low);
double battery_pow_up = positivePart(work_up) + efficiency_ * negativePart(work_up);
drake::VectorX<double> rv(1);
rv[0] = 1/2.0 * h_i *(battery_pow_low + battery_pow_up);
(*y) = rv;
};


JointPowerCost::JointPowerCost(const drake::multibody::MultibodyPlant<double>& plant,
                            dairlib::systems::trajectory_optimization::Dircon<double>& trajopt,
                             const double &Q,
                             const double &cost_work,
                             const double &alpha,
                             const std::string &description,
                             const int n_vars)
    :solvers::NonlinearCost<double>(n_vars+trajopt.num_modes(),description), plant_(plant){
  N_=trajopt.num_modes();
  Q_ = Q;
  cost_work_ = cost_work;
  alpha_ = alpha;
  n_q_ = plant_.num_positions();
  n_v_ = plant_.num_velocities();
  n_u_ = plant_.num_actuators();
  DRAKE_DEMAND(alpha_ > 0);
}
// COMMENT
void JointPowerCost::EvaluateCost(const Eigen::Ref<const drake::VectorX<double>> &x,
                  drake::VectorX<double> *y) const{
  double u_i = x(0);
  double u_ip = x(1);
  double v_i = x(2);
  double v_ip = x(3);
  double h_i = x(4);

  double ht = 0;
  for (int i =0;i<N_;i++){
    ht+=x(i+5)*num_nodes[i];
  }
  ////////////////////////////////////////// FOR MECHANICAL POWER ///////////////////////////////////////////
  // // double work_low = cost_work_ * relu_smooth(u_i * v_i + Q_ * u_i * u_i);
  // // double work_up = cost_work_ * relu_smooth(u_ip * v_ip + Q_ * u_ip * u_ip);
  // double work_low = cost_work_ * abs(u_i * v_i + Q_ * u_i * u_i);
  // double work_up = cost_work_ * abs(u_ip * v_ip + Q_ * u_ip * u_ip);

  // drake::VectorX<double> rv(1);
  // // rv[0] = 1/2.0 * h_i/ht *(work_low + work_up);
  // rv[0] = 1/2.0 * h_i/ht *(work_low + work_up);
  // (*y) = rv;
  //////////////////////////////////////////// FOR MECHANICAL POWER ///////////////////////////////////////////

  //////////////////////////////////////////// FOR ELETRICAL POWER ///////////////////////////////////////////
  double efficiency=0;

  double pow_low =  (u_i * v_i + Q_ * u_i * u_i);
  double pow_up =  (u_ip * v_ip + Q_ * u_ip * u_ip);

  double battery_pow_low = positivePart(pow_low) + efficiency * negativePart(pow_low);
  double battery_pow_up = positivePart(pow_up) + efficiency * negativePart(pow_up);

  // double battery_pow_low = relu_smooth(pow_low);
  // double battery_pow_up = relu_smooth(pow_up);
  drake::VectorX<double> rv(1);
  rv[0]= cost_work_ /2.0 * h_i/ht *(battery_pow_low + battery_pow_up);
  // std::cout<<"cost power gain: "<<cost_work_<<std::endl;
  (*y) = rv;
  //////////////////////////////////////////// FOR ELETRICAL POWER ///////////////////////////////////////////
};
double JointPowerCost::relu_smooth(const double x) const{
  return  x>5 ? x : 1/alpha_ * log(1 + exp(alpha_ * x));
}
double positivePart(double x){
  return(std::max(x,0.0));
}

double negativePart(double x){
  return(std::min(x,0.0));
}

JointDeltaTorqueRegularizationCost::JointDeltaTorqueRegularizationCost(
                            const drake::multibody::MultibodyPlant<double>& plant,
                            dairlib::systems::trajectory_optimization::Dircon<double>& trajopt,
                             const double &cost_delta)
    :solvers::NonlinearCost<double>(3,"Force regularization term "), plant_(plant){
  N_=trajopt.num_modes();
  cost_delta_ = cost_delta;
  n_q_ = plant_.num_positions();
  n_v_ = plant_.num_velocities();
  n_u_ = plant_.num_actuators();
}
// COMMENT
void JointDeltaTorqueRegularizationCost::EvaluateCost(const Eigen::Ref<const drake::VectorX<double>> &x,
                  drake::VectorX<double> *y) const{
  double u_i = x(0);
  double u_ip = x(1);
  double h_i = x(2);
  drake::VectorX<double> rv(1);
  rv[0]= cost_delta_ *(u_i-u_ip)*(u_i-u_ip)/ h_i/h_i;
  (*y) = rv;

};

void CompliantJointPowerCost::EvaluateCost(const Eigen::Ref<const drake::VectorX<double>> &x,
                  drake::VectorX<double> *y) const{
  double u_i = x(0);
  double u_ip = x(1);
  double v_i = x(2);
  double v_ip = x(3);
  double h_i = x(4);
  double x_i = x(5);
  double x_ip = x(6);

  double ht = 0;
  for (int i =0;i<N_;i++){
    ht+=x(i+7)*num_nodes[i];
  }
  ////////////////////////////////////////// FOR MECHANICAL POWER ///////////////////////////////////////////
  // // double work_low = cost_work_ * relu_smooth(u_i * v_i + Q_ * u_i * u_i);
  // // double work_up = cost_work_ * relu_smooth(u_ip * v_ip + Q_ * u_ip * u_ip);
  // double work_low = cost_work_ * abs(u_i * v_i + Q_ * u_i * u_i);
  // double work_up = cost_work_ * abs(u_ip * v_ip + Q_ * u_ip * u_ip);

  // drake::VectorX<double> rv(1);
  // // rv[0] = 1/2.0 * h_i/ht *(work_low + work_up);
  // rv[0] = 1/2.0 * h_i/ht *(work_low + work_up);
  // (*y) = rv;
  //////////////////////////////////////////// FOR MECHANICAL POWER ///////////////////////////////////////////

  //////////////////////////////////////////// FOR ELETRICAL POWER ///////////////////////////////////////////
  double efficiency=0; // electrical regen efficiency, set to zero for no regeneration

  // Since there's now a spring-damper system,
  // u_i = u_m - k*x_i - b*v_i
  // where u_m is the motor torque, and k and b are constants.
  // At some point, k and b will need to be variables, but let's hardcode for now.
  // u_m = u_i + k*x_i + b*v_i
  // double k = 4.0;//
  // double b = 0.7; //
  double pow_low =  ((u_i + k*x_i + b*v_i) * v_i + Q_ * (u_i + k*x_i + b*v_i) * (u_i + k*x_i + b*v_i));
  double pow_up =  ((u_ip + k*x_ip + b*v_ip) * v_ip + Q_ * (u_ip + k*x_ip + b*v_ip) * (u_ip + k*x_ip + b*v_ip));

  double battery_pow_low = positivePart(pow_low) + efficiency * negativePart(pow_low);
  double battery_pow_up = positivePart(pow_up) + efficiency * negativePart(pow_up);
  // double battery_pow_low = relu_smooth(pow_low);
  // double battery_pow_up = relu_smooth(pow_up);
  drake::VectorX<double> rv(1);
  rv[0]= cost_work_ /2.0 * h_i/ht *(battery_pow_low + battery_pow_up);
  // std::cout<<"cost power gain: "<<cost_work_<<std::endl;
  (*y) = rv;
  //////////////////////////////////////////// FOR ELETRICAL POWER ///////////////////////////////////////////
};


// void visualizeSurface(drake::multibody::MultibodyPlant<double>* plant_vis,
//   Eigen::Vector3d surface_normal,
//   Eigen::Vector3d surface_offset,
//   double length_surf,
//   double width_surf,
//   double thickness_surf,
//   const drake::Vector4<double> color
//   ){
//   Eigen::Vector3d unit_normal = surface_normal/surface_normal.norm();
//   Eigen::Vector3d bodyOffset(0,0,-thickness_surf/2);
//   drake::math::RotationMatrix<double> rot = normal2Rotation(unit_normal);
//   drake::math::RigidTransformd bodyToSurfaceTransform(bodyOffset);
//   drake::math::RigidTransformd worldToBodyTransform(rot,surface_offset);

//   double lx = length_surf;
//   double ly = width_surf;
//   double lz = thickness_surf;

//   // std::cout << rot.matrix() << std::endl;
//   plant_vis->RegisterVisualGeometry(
//     plant_vis->world_body(),
//     worldToBodyTransform*bodyToSurfaceTransform,     /* Pose X_BG of the geometry frame G in the cylinder body frame B. */
//     drake::geometry::Box(lx, ly, lz),
//     "box", color);
// }
void visualizeSurface(drake::multibody::MultibodyPlant<double>* plant_vis,
  Eigen::Vector3d surface_normal,
  Eigen::Vector3d surface_offset,
  double length_surf,
  double width_surf,
  double thickness_surf,
  const drake::Vector4<double> color,
  std::string name
  ){
  Eigen::Vector3d unit_normal = surface_normal/surface_normal.norm();
  Eigen::Vector3d bodyOffset(0,0,-thickness_surf/2);
  drake::math::RotationMatrix<double> rot = normal2Rotation(unit_normal);
  drake::math::RigidTransformd bodyToSurfaceTransform(bodyOffset);
  drake::math::RigidTransformd worldToBodyTransform(rot,surface_offset);

  double lx = length_surf;
  double ly = width_surf;
  double lz = thickness_surf;

  // std::cout << rot.matrix() << std::endl;
  plant_vis->RegisterVisualGeometry(
    plant_vis->world_body(),
    worldToBodyTransform*bodyToSurfaceTransform,     /* Pose X_BG of the geometry frame G in the cylinder body frame B. */
    drake::geometry::Box(lx, ly, lz),
    name, color);
}

void visualizeSurface(drake::multibody::MultibodyPlant<double>* plant_vis,
  Eigen::Vector3d surface_normal,
  Eigen::Vector3d surface_offset,
  double length_surf,
  double width_surf,
  double thickness_surf){
  const drake::Vector4<double> orange(1.0, 0.55, 0.0, 1.0);
  visualizeSurface(
    plant_vis,
    surface_normal,
    surface_offset,
    length_surf,
    width_surf,
    thickness_surf,
    orange
  );
  }

template void nominalSpiritStand(
    drake::multibody::MultibodyPlant<double>& plant,
    Eigen::VectorXd& xState,
    double height); // NOLINT

template void nominalSpiritStandConstraint(
    drake::multibody::MultibodyPlant<double>& plant,
    dairlib::systems::trajectory_optimization::Dircon<double>& trajopt,
    double height,
    std::vector<int> knotPoints,
    const double eps); // NOLINT

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
          std::vector<double> mus ,
          std::vector<double> minTs,
          std::vector<double> maxTs); // NOLINT


template std::tuple<
                std::vector<std::unique_ptr<DirconMode<double>>>,
                std::vector<std::unique_ptr<dairlib::multibody::WorldPointEvaluator<double>>> ,
                std::vector<std::unique_ptr<dairlib::multibody::KinematicEvaluatorSet<double>>>
          > createSpiritModeSequence(
          MultibodyPlant<double>& plant, // multibodyPlant
          const dairlib::ModeSequenceHelper& msh );//NOLINT


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
          dairlib::systems::trajectory_optimization::Dircon<double>& trajopt,
          float knee_limit);

template void setSpiritActuationLimits(
          drake::multibody::MultibodyPlant<double> & plant,
          dairlib::systems::trajectory_optimization::Dircon<double>& trajopt,
          double actuatorLimit);


template void setSpiritActuationSpeedCurveLimits(
          drake::multibody::MultibodyPlant<double> & plant,
          dairlib::systems::trajectory_optimization::Dircon<double>& trajopt);

template void setSpiritSymmetry(
        drake::multibody::MultibodyPlant<double> & plant,
        dairlib::systems::trajectory_optimization::Dircon<double>& trajopt,
        std::string symmetry,
        bool ignoreWarning);

template void setSpiritSymmetry(
        drake::multibody::MultibodyPlant<double> & plant,
        dairlib::systems::trajectory_optimization::Dircon<double>& trajopt,
        std::vector<std::string> symmetries);

template double calcWork(
    drake::multibody::MultibodyPlant<double> & plant,
    drake::trajectories::PiecewisePolynomial<double>& x_traj,
    drake::trajectories::PiecewisePolynomial<double>& u_traj);

template double calcMechanicalWork(
    drake::multibody::MultibodyPlant<double> & plant,
    std::vector<drake::trajectories::PiecewisePolynomial<double>>& x_traj,
    drake::trajectories::PiecewisePolynomial<double>& u_traj);

template double calcElectricalWork(
    drake::multibody::MultibodyPlant<double> & plant,
    std::vector<drake::trajectories::PiecewisePolynomial<double>>& x_traj,
    drake::trajectories::PiecewisePolynomial<double>& u_traj,
    double efficiency);

template double calcVelocityInt(
    drake::multibody::MultibodyPlant<double> & plant,
    drake::trajectories::PiecewisePolynomial<double>& x_traj);

template double calcVelocityInt(
    drake::multibody::MultibodyPlant<double> & plant,
    std::vector<drake::trajectories::PiecewisePolynomial<double>>& x_trajs);

template double calcTorqueInt(
    drake::multibody::MultibodyPlant<double> & plant,
    drake::trajectories::PiecewisePolynomial<double>& u_traj);

template std::vector<drake::solvers::Binding<drake::solvers::Cost>> AddWorkCost(drake::multibody::MultibodyPlant<double> & plant,
                 dairlib::systems::trajectory_optimization::Dircon<double>& trajopt,
                 double cost_work_gain,
                 std::string work_type);

template std::vector<drake::solvers::Binding<drake::solvers::Cost>> AddPowerCost(drake::multibody::MultibodyPlant<double> & plant,
                 dairlib::systems::trajectory_optimization::Dircon<double>& trajopt,
                 double cost_work_gain, double k, double b);

template std::vector<drake::solvers::Binding<drake::solvers::Cost>> AddDeltaTorqueRegularizationCost(drake::multibody::MultibodyPlant<double> & plant,
                 dairlib::systems::trajectory_optimization::Dircon<double>& trajopt,
                 double regularization_gain);


}//namespace dairlib
