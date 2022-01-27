#pragma once

namespace dairlib {
/// Containter for an optimized stand behavior, useful for initial and final conditions TODO add templating
class OptimalSpiritStand {
  private: 
    drake::multibody::MultibodyPlant<double>* plantPtr_;
    double height_;
    Eigen::Vector3d normal_;
    Eigen::Vector3d offset_;
    Eigen::VectorXd fullstate_;
    std::string filename_;
    Eigen::Vector3d rpy_;
    std::string folder_= "/home/feng/Downloads/dairlib/examples/Spirit/saved_trajectories/stand_trajectories/";
    
    /// Runs the stand optimization for the object, can be used to update the stand
    /// @param tol tolerance for the optimization
    /// @param animate boolean to enable looping animation visualization
    Eigen::VectorXd getSpiritOptimalStand( double tol = 1e-4, bool animate = false);

  public:
    /// The constructor for the Optimal Stand defined using the surface normal 
    /// and saving the offset. If a file of a nearby stand is in folder_ it will 
    /// not rerun the optimization by default
    /// @param plantPtr pointer to plant used in stand
    /// @param height the normal distance of the COM from the surface
    /// @param normal the surface normal
    /// @param offset the nominal location of the surface normal (not used in the optimization)
    /// @param rerun boolean to rerun an optimization that has a saved file already
    /// @param tol tolerance for the optimization
    /// @param animate boolean to enable looping animation visualization
    /// @param filenameBase filenameBase for saving the optimization
    OptimalSpiritStand(){};
    OptimalSpiritStand(
          drake::multibody::MultibodyPlant<double>* plantPtr, 
          double height, 
          Eigen::Vector3d normal,
          Eigen::Vector3d offset,
          bool rerun = false,
          double tol = 1e-4,
          bool animate = false,
          std::string filenameBase = "optimalStand");  // Constructor
    
    void init(
          drake::multibody::MultibodyPlant<double>* plantPtr, 
          double height, 
          Eigen::Vector3d normal,
          Eigen::Vector3d offset,
          bool rerun = false,
          double tol = 1e-4,
          bool animate = false,
          std::string filenameBase = "optimalStand");  // Constructor
    // Get the surface normal 
    Eigen::VectorXd normal(){ return normal_; }
    // Get the surface nominal offset 
    Eigen::VectorXd offset(){ return offset_; }
    // Get the fullstate of the stand (velocities should be zero)
    Eigen::VectorXd getState(){ return fullstate_; }
    // Get the roll pitch yaw as a vector
    Eigen::Vector3d getRPY(){ return rpy_; }
    // Get the position of the optimal stand
    Eigen::VectorXd getPosition(){ return fullstate_.head(plantPtr_->num_positions()); }
    // Get quaternion of the body orientation in optimal stand
    Eigen::VectorXd getQuat(){ return fullstate_.head(4); }
    // Get the COM position in the optimal stand (first 7 are [quat; COM] )
    Eigen::VectorXd getCOM(){ return (fullstate_.head(7)).tail(3); }
    // Get all the joints (positions that aren't Quat or COM pose)
    Eigen::VectorXd getJoints(){ return (getPosition()).tail(plantPtr_->num_positions()-7); }
    double height(){return height_;};
};
}