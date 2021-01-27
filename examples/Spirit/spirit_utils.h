#pragma once

namespace dairlib {

class ModeSequenceHelper {
  public:
    std::vector<Eigen::Matrix<bool,1,4>> modes; // bool matrix describing toe contacts as true or false e.g. {{1,1,1,1},{0,0,0,0}} would be a full support mode and flight mode
    std::vector<int> knots; // Matrix of knot points for each mode  
    std::vector<Eigen::Vector3d> normals;
    std::vector<Eigen::Vector3d> offsets;
    std::vector<double> mus;
    void addMode(
      Eigen::Matrix<bool,1,4> activeContactVector, 
      int num_knots, 
      Eigen::Vector3d normal, 
      Eigen::Vector3d offset,
      double mu){
        modes.push_back(activeContactVector);
        knots.push_back(num_knots);
        normals.push_back(normal);
        offsets.push_back(offset);
        mus.push_back(mu);
    }
};

/// Outputs a nominal stand state into the xState vector pointer based on the 
/// height. This is an approximation for use in initial conditions.
///   @param plant a pointer to the MultibodyPlant
///   @param xState a pointer to a vector for the state to be saved in
///   @param height the nominal height for calculating approx inv kinematics

template <typename T>
void nominalSpiritStand(
    drake::multibody::MultibodyPlant<T>& plant, 
    Eigen::VectorXd& xState, 
    double height);

/// Returns a pointer to the toe frame by index. On Spirit the toes are as follows
/// Front Left (0), Back Left (1), Front Right (2), Back Right(3)
/// @param plant a point to the MultibodyPlant 
/// @param toeIndex The index of the desired toe (0-3)
template <typename T>
const drake::multibody::Frame<T>& getSpiritToeFrame( 
    drake::multibody::MultibodyPlant<T>& plant, 
    u_int8_t toeIndex );

/// Returns a unique WorldPointEvaluator for the toe at toeIndex. Uses the normal
/// vector constructor to allow for non-world-aligned ground/surface contact.
/// Also includes friction, and a offset from the frame if the toes have a radius
///   @param plant a point to the MultibodyPlant
///   @param toeIndex the toeIndex of the desired evaluator
///   @param toePoint the toeOffset in the toe frame
///   @param mu frictional coefficient
///   @param normal the contact normal of this evaluator
///   @param xy_active are the tangent directions active
///              )
///
template <typename T>
std::unique_ptr<multibody::WorldPointEvaluator<T>> getSpiritToeEvaluator( 
                      drake::multibody::MultibodyPlant<T>& plant, 
                      const Eigen::Vector3d toePoint,
                      u_int8_t toeIndex,
                      const Eigen::Vector3d normal = Eigen::Vector3d::UnitZ(),
                      const Eigen::Vector3d offset = Eigen::Vector3d::Zero(),
                      bool xy_active = true, 
                      double mu = 0.0
                      );


/// Takes in a matrix of bools that define the mode sequence (contact/no contact)
/// and outputs a tuple of the mode vector, toe evaluator sets, and toe evaluators
/// the latter two are only used to avoid descoping of the references used by
/// the mode sequence
///    @param plant a pointer to a multibodyPlant
///    @param modeSeqMat a bool matrix describing toe contacts as true or false 
///             e.g. {{1,1,1,1},{0,0,0,0}} would be a full support mode and flight mode
///    @param knotpointVect  Vector of knot points for each mode  
///    @param mu Friction coefficient

template <typename T>
std::tuple<  std::vector<std::unique_ptr<dairlib::systems::trajectory_optimization::DirconMode<T>>>,
             std::vector<std::unique_ptr<multibody::WorldPointEvaluator<T>>> ,
             std::vector<std::unique_ptr<multibody::KinematicEvaluatorSet<T>>>   
          >     
    createSpiritModeSequence( 
          drake::multibody::MultibodyPlant<T>& plant, // multibodyPlant
          std::vector<Eigen::Matrix<bool,1,4>> modeSeqVect, // bool matrix describing toe contacts as true or false e.g. {{1,1,1,1},{0,0,0,0}} would be a full support mode and flight mode
          std::vector<int> knotpointVect, // Matrix of knot points for each mode  
          std::vector<Eigen::Vector3d> normals,
          std::vector<Eigen::Vector3d> offsets, 
          std::vector<double> mus );



/// Add joint kinematic limits to the spirit joints

// template <typename T>
// void setSpiritJointLimits(
//     drake::multibody::MultibodyPlant<T> & plant
// )



template <typename T>
void setSpiritJointLimits(
                    drake::multibody::MultibodyPlant<T> & plant, 
                    dairlib::systems::trajectory_optimization::Dircon<T>& trajopt,  
                    int iJoint, 
                    double minVal, 
                    double maxVal  );

template <typename T>
void setSpiritJointLimits(
                    drake::multibody::MultibodyPlant<T> & plant, 
                    dairlib::systems::trajectory_optimization::Dircon<T>& trajopt,  
                    std::vector<int> iJoints, 
                    std::vector<double> minVals, 
                    std::vector<double> maxVals  );

template <typename T>
void setSpiritJointLimits(
                    drake::multibody::MultibodyPlant<T> & plant, 
                    dairlib::systems::trajectory_optimization::Dircon<T>& trajopt,  
                    std::vector<int> iJoints, 
                    double minVal, 
                    double maxVal  );

template <typename T>
void setSpiritJointLimits(
                    drake::multibody::MultibodyPlant<T> & plant, 
                    dairlib::systems::trajectory_optimization::Dircon<T>& trajopt );

template <typename T> 
void setSpiritSymmetry(
        drake::multibody::MultibodyPlant<T> & plant, 
        dairlib::systems::trajectory_optimization::Dircon<T>& trajopt,
        std::string symmetry = "sagittal");

template <typename T>
void setSpiritSymmetry(
        drake::multibody::MultibodyPlant<T> & plant, 
        dairlib::systems::trajectory_optimization::Dircon<T>& trajopt,
        std::vector<std::string> symmetries);

} //namespace dairlib