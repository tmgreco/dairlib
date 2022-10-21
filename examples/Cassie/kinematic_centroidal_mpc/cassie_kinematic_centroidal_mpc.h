#pragma once
#include <iostream>
#include "systems/controllers/kinematic_centroidal_mpc/kinematic_centroidal_mpc.h"
#include "external/drake/tools/install/libdrake/_virtual_includes/drake_shared_library/drake/multibody/plant/multibody_plant.h"
#include "examples/Cassie/cassie_utils.h"

/*!
 * @brief Cassie specific child class for kinematic centroidal mpc.
 */
class CassieKinematicCentroidalMPC : public KinematicCentroidalMPC {
 public:
  CassieKinematicCentroidalMPC(const drake::multibody::MultibodyPlant<double>& plant, int n_knot_points, double dt) :
      KinematicCentroidalMPC(plant, n_knot_points, dt, CreateContactPoints(plant)),
      l_loop_evaluator_(dairlib::LeftLoopClosureEvaluator(Plant())),
      r_loop_evaluator_(dairlib::RightLoopClosureEvaluator(Plant())),
      loop_closure_evaluators(Plant()){
    AddPlantJointLimits(dairlib::JointNames());
    AddLoopClosure();
  }
 private:
  std::vector<dairlib::multibody::WorldPointEvaluator<double>> CreateContactPoints(const drake::multibody::MultibodyPlant<double>& plant);
  void AddLoopClosure();

  dairlib::multibody::DistanceEvaluator<double> l_loop_evaluator_;
  dairlib::multibody::DistanceEvaluator<double> r_loop_evaluator_;
  dairlib::multibody::KinematicEvaluatorSet<double> loop_closure_evaluators;

};

