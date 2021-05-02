#pragma once
#include "solvers/nonlinear_cost.h"
#include "drake/multibody/inverse_kinematics/inverse_kinematics.h"

namespace dairlib {

class ModeSequenceHelper {
  public:
    std::vector<Eigen::Matrix<bool,1,4>> modes; // bool matrix describing toe contacts as true or false e.g. {{1,1,1,1},{0,0,0,0}} would be a full support mode and flight mode
    std::vector<int> knots; // Matrix of knot points for each mode  
    std::vector<Eigen::Vector3d> normals;
    std::vector<Eigen::Vector3d> offsets;
    std::vector<double> mus;
    std::vector<double> minTs;
    std::vector<double> maxTs;
    void addMode(
      Eigen::Matrix<bool,1,4> activeContactVector, 
      int num_knots, 
      Eigen::Vector3d normal, 
      Eigen::Vector3d offset,
      double mu,
      double minT = 0,
      double maxT = std::numeric_limits<double>::infinity() ){
        modes.push_back(activeContactVector);
        knots.push_back(num_knots);
        normals.push_back(normal);
        offsets.push_back(offset);
        mus.push_back(mu);
        minTs.push_back(minT);
        maxTs.push_back(maxT);
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

/// Solves an IK problem for the spirit stand. If the feet are in contact with the ground they are constrained to be
/// under the hips in the world frame. If they are not in contact with the ground they are constrained to be under the
/// hips in the body frame a distance of leg_height.
///     @param plant, reference to the robot plant
///     @param xState[out], the state vector from solving the IK problem
///     @param contactSequence, vector of booleans describing which legs are in contact with the ground
///     @param com_height, height of the com of spirit
///     @param leg_height, distance between hip and toe for legs not on the ground
///     @param roll, roll of body in radians
///     @param pitch, pitch of body in radians
///     @param spine, does the robot have a spine?
///     @param eps, tolerance on legs in contact with ground
void ikSpiritStand(
    drake::multibody::MultibodyPlant<double>& plant,
    Eigen::VectorXd& xState,
    Eigen::Matrix<bool,1,4> contactSequence,
    double com_height,
    double leg_height,
    double roll = 0,
    double pitch = 0,
    const bool spine = false,
    double eps = 0.01);

/// Adds constraints to a toe for ik problem. If the toe are in contact with the ground they are constrained to be
/// under the hips in the world frame. If they are not in contact with the ground they are constrained to be under the
/// hips in the body frame a distance of leg_height.
///     @param plant, reference to the robot plant
///     @param ik, ik problem to add constraints to
///     @param toe, the number of the toe for which to constraint
///     @param inContact, true if the toe is in contact with the ground
///     @param orientation, rotation matrix describing orientation of body in world frame
///     @param com_height, height of the com of spirit
///     @param leg_height, distance between hip and toe for legs not on the ground
///     @param eps, tolerance on legs in contact with ground
void constrainToe(drake::multibody::MultibodyPlant<double> & plant,
                  drake::multibody::InverseKinematics &ik,
                  int toe, bool inContact, drake::math::RotationMatrix<double> orientation,
                  double com_height, double leg_height, double eps);

/// Constrians j0-j1 so nominal stand is at specified height
///   @param[in]  MultibodyPlant
///   @param[in]  trajectory optimization object
///   @param height the nominal height for calculating approx inv kinematics
///   @param knotPoints vector of the knot points to constrain
///   @param eps tolerance on constraint
template <typename T>
void nominalSpiritStandConstraint(
    drake::multibody::MultibodyPlant<T>& plant,
    dairlib::systems::trajectory_optimization::Dircon<T>& trajopt,
    double height,
    std::vector<int> knotPoints,
    const double eps = 0);


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
///   @param toePoint the toeOffset in the toe frame
///   @param toeIndex the toeIndex of the desired evaluator
///   @param normal the contact normal of this evaluator
///   @param offset the contact normal's nominal location in the world
///   @param xy_active are the tangent directions active
///   @param mu frictional coefficient
///              )
///
template <typename T>
std::unique_ptr<dairlib::multibody::WorldPointEvaluator<T>> getSpiritToeEvaluator( 
                      drake::multibody::MultibodyPlant<T>& plant, 
                      const Eigen::Vector3d toePoint,
                      u_int8_t toeIndex,
                      const Eigen::Vector3d normal = Eigen::Vector3d::UnitZ(),
                      const Eigen::Vector3d offset = Eigen::Vector3d::Zero(),
                      bool xy_active = true, 
                      double mu = std::numeric_limits<double>::infinity()
                      );


/// Takes in a matrix of bools that define the mode sequence (contact/no contact)
/// and outputs a tuple of the mode vector, toe evaluator sets, and toe evaluators
/// the latter two are only used to avoid descoping of the references used by
/// the mode sequence
///    @param plant a pointer to a multibodyPlant
///    @param modeSeqMat a bool matrix describing toe contacts as true or false 
///             e.g. {{1,1,1,1},{0,0,0,0}} would be a full support mode and flight mode
///    @param knotpointVect  Vector of knot points for each mode  
///    @param normals vector of contact normals of each mode
///    @param offsets vector of contact normals' nominal locations in the world
///    @param mus vector of friction coefficients

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
          std::vector<double> mus, 
          std::vector<double> minTs, 
          std::vector<double> maxTs );

/// Overload allows the use of the ModeSequenceHelper Class so we dont need to feed in all the arguments
template <typename T>
std::tuple<  std::vector<std::unique_ptr<dairlib::systems::trajectory_optimization::DirconMode<T>>>,
             std::vector<std::unique_ptr<multibody::WorldPointEvaluator<T>>> ,
             std::vector<std::unique_ptr<multibody::KinematicEvaluatorSet<T>>>   
          >     
    createSpiritModeSequence( 
          drake::multibody::MultibodyPlant<T>& plant, // multibodyPlant
          const dairlib::ModeSequenceHelper& msh );

/// This overload sets all the joints to their nominal limit's
///    @param plant a pointer to a multibodyPlant
///    @param trajopt a ponter to a Dircon<T> object
///    @param spine, does the robot have a spine?
template <typename T>
void setSpiritJointLimits(
                    drake::multibody::MultibodyPlant<T> & plant,
                    dairlib::systems::trajectory_optimization::Dircon<T>& trajopt,
                    const bool spine = false);

/// This overload sets an individual joint's position limit
///    @param plant a pointer to a multibodyPlant
///    @param trajopt a ponter to a Dircon<T> object  
///    @param iJoint the integer index of the joint
///    @param minVal joint's minimum position
///    @param minVal joint's maximum position
template <typename T>
void setSpiritJointLimits(
                    drake::multibody::MultibodyPlant<T> & plant,
                    dairlib::systems::trajectory_optimization::Dircon<T>& trajopt,
                    int iJoint, 
                    double minVal, 
                    double maxVal  );

/// This overload sets an a set of joints' position limits
///    @param plant a pointer to a multibodyPlant
///    @param trajopt a ponter to a Dircon<T> object  
///    @param iJoints the integer indices (vector) of the joints to be limited
///    @param minVals vector of joints' minimum positions
///    @param minVals vector of joints' maximum positions
template <typename T>
void setSpiritJointLimits(
                    drake::multibody::MultibodyPlant<T> & plant,
                    dairlib::systems::trajectory_optimization::Dircon<T>& trajopt,
                    std::vector<int> iJoints, 
                    std::vector<double> minVals, 
                    std::vector<double> maxVals  );

/// This overload sets an a set of joints' position limits to the same thing
///    @param plant a pointer to a multibodyPlant
///    @param trajopt a ponter to a Dircon<T> object  
///    @param iJoints the integer indices (vector) of the joints to be limited
///    @param minVal joints' minimum position
///    @param minVal joints' maximum position
template <typename T>
void setSpiritJointLimits(
                    drake::multibody::MultibodyPlant<T> & plant,
                    dairlib::systems::trajectory_optimization::Dircon<T>& trajopt,
                    std::vector<int> iJoints, 
                    double minVal, 
                    double maxVal  );



/// Sets all the joints' actuator limits to the same thing
///    @param plant a pointer to a multibodyPlant
///    @param trajopt a ponter to a Dircon<T> object  
///    @param actuatorLimit the (symmetric) effort limit
///    @param spine, does the robot have a spine?
template <typename T> 
void setSpiritActuationLimits(
          drake::multibody::MultibodyPlant<T> & plant, 
          dairlib::systems::trajectory_optimization::Dircon<T>& trajopt,
          const bool spine = false,
          double actuatorLimit = 3.5 * 6);// URDF has 40 this is more realistic based on the modules 

/// Constrains the system to a single symmetric leg behavior
///    @param plant a pointer to a multibodyPlant
///    @param trajopt a ponter to a Dircon<T> object  
///    @param symmetry a string of the desired symmetry
template <typename T> 
void setSpiritSymmetry(
        drake::multibody::MultibodyPlant<T> & plant, 
        dairlib::systems::trajectory_optimization::Dircon<T>& trajopt,
        std::string symmetry = "sagittal",
        bool ignoreWarning = false);


/// Constrains the system to a set symmetric leg behaviors handling internal
/// over contraining (TODO)
///    @param plant a pointer to a multibodyPlant
///    @param trajopt a ponter to a Dircon<T> object  
///    @param symmetry a vector of symmetries (vector)
template <typename T>
void setSpiritSymmetry(
        drake::multibody::MultibodyPlant<T> & plant, 
        dairlib::systems::trajectory_optimization::Dircon<T>& trajopt,
        std::vector<std::string> symmetries);

/// Calculates the mechanical work done during trajectory
///     @param plont, a pointer to the robot's model
///     @param x_traj the state trajectory
///     @param u_traj the control trajectory
template <typename T>
double calcWork(
    drake::multibody::MultibodyPlant<T> & plant,
    drake::trajectories::PiecewisePolynomial<double>& x_traj,
    drake::trajectories::PiecewisePolynomial<double>& u_traj);

/// Calculates the mechanical work done during trajectory, handles discontinuities
///     @param plont, a pointer to the robot's model
///     @param x_trajs a vector of the state trajectory for each mode
///     @param u_traj the control trajectory
template <typename T>
double calcMechanicalWork(
    drake::multibody::MultibodyPlant<T> & plant,
    std::vector<drake::trajectories::PiecewisePolynomial<double>>& x_trajs,
    drake::trajectories::PiecewisePolynomial<double>& u_traj);

/// Calculates the electrical work done during trajectory, handles discontinuities
///     @param plont, a pointer to the robot's model
///     @param x_trajs a vector of the state trajectory for each mode
///     @param u_traj the control trajectory
///     @param spine, does the robot have a spine?
///     @param efficiency, gain on what percent of negative power is useable by the battery
template <typename T>
double calcElectricalWork(
    drake::multibody::MultibodyPlant<T> & plant,
    std::vector<drake::trajectories::PiecewisePolynomial<double>>& x_trajs,
    drake::trajectories::PiecewisePolynomial<double>& u_traj,
    const bool spine = false,
    double efficiency = 0);

/// Calculates the integral of velocities squared
///     @param plont, a pointer to the robot's model
///     @param x_traj the state trajectory
template <typename T>
double calcVelocityInt(
    drake::multibody::MultibodyPlant<T> & plant,
    drake::trajectories::PiecewisePolynomial<double>& x_traj);

/// Calculates the integral of velocities squared
///     @param plont, a pointer to the robot's model
///     @param x_trajs a vector of the state trajectory for each mode
template <typename T>
double calcVelocityInt(
    drake::multibody::MultibodyPlant<T> & plant,
    std::vector<drake::trajectories::PiecewisePolynomial<double>>& x_trajs);

/// Calculates the integral of torques squared
///     @param plont, a pointer to the robot's model
///     @param u_traj the control trajectory
template <typename T>
double calcTorqueInt(
    drake::multibody::MultibodyPlant<T> & plant,
    drake::trajectories::PiecewisePolynomial<double>& u_traj);

/// Adds a cost on the integral of electrical power
///     @param plant, the robot model
///     @param trajopt the dircon object
///     @param cost_work_gain, the gain on the electrical work
///     @param spine, does the robot have a spine?
template <typename T>
std::vector<drake::solvers::Binding<drake::solvers::Cost>> AddWorkCost(drake::multibody::MultibodyPlant<T> & plant,
                 dairlib::systems::trajectory_optimization::Dircon<T>& trajopt,
                 double cost_work_gain,
                 const bool spine = false);


/// JointWorkCost object for adding smooth relu without slack variables to cost
class JointWorkCost : public solvers::NonlinearCost<double> {
 public:
  JointWorkCost(const drake::multibody::MultibodyPlant<double>& plant,
                const double &Q,
                const double &cost_work,
                const double &alpha,
                const std::string &description = "");

 private:
  /// Smooth relu
  double relu_smooth(const double x) const;
  void EvaluateCost(const Eigen::Ref<const drake::VectorX<double>> &x,
                    drake::VectorX<double> *y) const override;
  const drake::multibody::MultibodyPlant<double>& plant_;
  double Q_; /// Gain on actuation squared
  double cost_work_;
  double alpha_; /// Gain on smoothing for relu, higher is less smooth
  int n_q_;
  int n_v_;
  int n_u_;
};

double positivePart(double x);
double negativePart(double x);

// Gains on resistive losses for knees and for other motors based on resistance, torque constant, and gear ratio
const double Q_u8 = 20.196;
const double gear_not_knee = 6;
const double gear_spine = gear_not_knee * 3;
const double gear_knee = gear_not_knee * 1.5;
const double Q_knee = Q_u8/ pow(gear_knee , 2);
const double Q_not_knee = Q_u8/ pow(gear_not_knee,2);
const double Q_spine = Q_u8 / pow(gear_spine,2);

} //namespace dairlib