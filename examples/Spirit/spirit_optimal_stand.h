#pragma once

namespace dairlib {
class OptimalSpiritStand {
  private: 
    drake::multibody::MultibodyPlant<double>* plantPtr_;
    double height_;
    Eigen::Vector3d normal_;
    Eigen::VectorXd fullstate_;
    std::string filename_;
    std::string folder_= "/home/jdcap/dairlib/examples/Spirit/saved_trajectories/stand_trajectories/";
    bool animate_ = false;

  public:
    OptimalSpiritStand(
          drake::multibody::MultibodyPlant<double>* plantPtr, 
          double height, 
          Eigen::Vector3d normal,
          bool rerun = false,
          double tol = 1e-4,
          bool animate = false);  // Constructor

    Eigen::VectorXd getJoints(){ return fullstate_.tail(fullstate_.rows()-7); }
    void loadOptimalStand(std::string filename);
    Eigen::VectorXd getSpiritOptimalStand( double tol = 1e-4);
   
    // static void interrupt_handler(int signum);
};
}