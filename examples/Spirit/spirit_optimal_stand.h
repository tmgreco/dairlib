#pragma once

namespace dairlib {
class OptimalSpiritStand {
  private: 
    drake::multibody::MultibodyPlant<double>* plantPtr_;
    double height_;
    Eigen::Vector3d normal_;
    Eigen::Vector3d offset_;
    Eigen::VectorXd fullstate_;
    std::string filename_;
    Eigen::Vector3d rpy_;
    int nq_ = 19; //TODO make dynamic
    std::string folder_= "/home/jdcap/dairlib/examples/Spirit/saved_trajectories/stand_trajectories/";

  public:
    OptimalSpiritStand(
          drake::multibody::MultibodyPlant<double>* plantPtr, 
          double height, 
          Eigen::Vector3d normal,
          Eigen::Vector3d offset,
          bool rerun = false,
          double tol = 1e-4,
          bool animate = false);  // Constructor
          
    Eigen::VectorXd normal(){ return normal_; }
    Eigen::VectorXd offset(){ return offset_; }
    Eigen::VectorXd getJoints(){ return (getPosition()).tail(12); }
    Eigen::VectorXd getState(){ return fullstate_; }
    Eigen::Vector3d getRPY(){ return rpy_; }
    Eigen::VectorXd getPosition(){ return fullstate_.head(nq_); }
    Eigen::VectorXd getQuat(){ return fullstate_.head(4); }
    Eigen::VectorXd getCOM(){ return (fullstate_.head(7)).tail(3); }
    void loadOptimalStand(std::string filename);
    Eigen::VectorXd getSpiritOptimalStand( double tol = 1e-4, bool animate = false);
   
};
}