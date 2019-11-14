//
// Created by yangwill on 11/13/19.
//
#include <gflags/gflags.h>
#include <chrono>
#include <fstream>

#include "attic/multibody/rigidbody_utils.h"

#include "drake/multibody/rigid_body_tree_construction.h"

#include "drake/lcm/drake_lcm.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/snopt_solver.h"
#include "drake/solvers/solve.h"
#include "drake/solvers/solution_result.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/systems/rendering/multibody_position_to_geometry_pose.h"
#include "drake/geometry/geometry_visualization.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/multibody/parsers/urdf_parser.h"

#include "common/find_resource.h"

#include "examples/jumping/traj_logger.h"
#include "lcm/lcm_trajectory.h"

using drake::multibody::Parser;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using drake::trajectories::PiecewisePolynomial;
using std::cout;
using std::endl;
using drake::multibody::MultibodyPlant;

DEFINE_double(time_offset, 0.0,
              "Length of time (s) to remain in neutral state");
DEFINE_string(file_name, "",
              "Name of file to write COM traj to");
DEFINE_int32(resolution, 200, "Number of timesteps per states");

namespace dairlib {

namespace examples {
namespace jumping {

int doMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  std::string filename = "examples/jumping/five_link_biped.urdf";
  RigidBodyTree<double> tree;
  drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
      FindResourceOrThrow(filename), drake::multibody::joints::kFixed, &tree);

  KinematicsCache<double> cache = tree.CreateKinematicsCache();
  const LcmTrajectory& loaded_traj =
      LcmTrajectory(LcmTrajectory::loadFromFile
                        ("examples/jumping/saved_trajs/jumping_11_13"));
  const LcmTrajectory::Trajectory& jumping_traj = loaded_traj.getTrajectory
                                                                 ("jumping_trajectory_x_u");
  int num_positions = tree.get_num_positions();
  int num_states = num_positions + tree
      .get_num_velocities();
  cout << jumping_traj.datapoints
                      .topRows(num_states).cols() << endl;
  cout << jumping_traj.time_vector.size() << endl;
  const PiecewisePolynomial<double>& datapoints =
      PiecewisePolynomial<double>::Pchip(jumping_traj.time_vector,
                                         jumping_traj.datapoints
                                                     .topRows(num_states));

  for (auto const& element : multibody::makeNameToPositionsMap(tree)) {
    cout << element.first << " = " << element.second << endl;
  }

  VectorXd q(num_positions);
  VectorXd center_of_mass(3);

  std::vector<double> points;
  std::vector<double> times;
  points.reserve(10000);
  times.reserve(10000);

  for (int i = 0; i < FLAGS_resolution; ++i) {
    //    cout << traj.value(0).transpose().topLeftCorner(1, tree.get_num_positions())
    //         << endl;
    //    q << traj.value(0).transpose().topLeftCorner(1, tree.get_num_positions());
    //    std::cout << datapoints.value(i * FLAGS_time_offset / FLAGS_resolution)
    //    .topRows(num_positions) <<
    //              std::endl;
    q << datapoints.value(0).topRows(num_positions);
    //     Order of states are not the same for multibody and rigid bodies
    dairlib::multibody::SetZeroQuaternionToIdentity(&q);
    cache.initialize(q);
    tree.doKinematics(cache);
    center_of_mass = tree.centerOfMass(cache);
    times.push_back(i * FLAGS_time_offset / FLAGS_resolution);
    for (int j = 0; j < 3; ++j) {
      points.push_back(center_of_mass(j));
    }
  }
  cout << "Adding more points to trajectory: " << endl;
  double time_offset = times.empty() ? 0 : times.back() +
      datapoints.end_time() / FLAGS_resolution;
  double end_time = datapoints.end_time();
  for (int i = 0; i < FLAGS_resolution; ++i) {
    //    cout << traj.value(0).transpose().topLeftCorner(1, tree.get_num_positions())
    //         << endl;
    //    q << traj.value(0).transpose().topLeftCorner(1, tree.get_num_positions());
    //    std::cout << datapoints.value(i * FLAGS_time_offset / FLAGS_resolution)
    //    .topRows(num_positions) <<
    //              std::endl;
    q << datapoints.value(i * end_time / FLAGS_resolution)
                   .topRows(num_positions);
    //     Order of states are not the same for multibody and rigid bodies
    dairlib::multibody::SetZeroQuaternionToIdentity(&q);
    cache.initialize(q);
    tree.doKinematics(cache);
    center_of_mass = tree.centerOfMass(cache);
    std::cout << center_of_mass.transpose() << std::endl;
    times.push_back(i * end_time / FLAGS_resolution + time_offset);
    for (int j = 0; j < 3; ++j) {
      points.push_back(center_of_mass(j));
    }
  }

  std::cout << "Creating matrix " << std::endl;

  MatrixXd com_pos_matrix =
      Eigen::Map<const Matrix<double, Dynamic, Dynamic>>(
          points.data(),
          3, points.size() / 3);
  VectorXd com_pos_time_matrix =
      Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(times.data(),
                                                          times.size());

  LcmTrajectory::Trajectory traj_block;
  traj_block.traj_name = "center_of_mass_trajectory";
  traj_block.datapoints = com_pos_matrix;
  traj_block.time_vector = com_pos_time_matrix;
  traj_block.datatypes = {"com_x", "com_y", "com_z"};
  std::vector<LcmTrajectory::Trajectory> trajectories;
  trajectories.push_back(traj_block);
  std::vector<std::string> trajectory_names;
  trajectory_names.push_back("center_of_mass_trajectory");
  LcmTrajectory saved_traj(trajectories, trajectory_names, "COM_trajectory",
                           "Center of mass trajectory for jumping");
  saved_traj.writeToFile("examples/jumping/saved_trajs/" + FLAGS_file_name);
  //
  //  {
  //    std::ofstream fout;
  //    fout.open("examples/jumping/saved_trajs/com_traj/squat.csv");
  //    int count = 0;
  //    for (double pt : points) {
  //      fout << pt;
  //      if (count++ % 4 == 2) {
  //        fout << "\n";
  //      } else {
  //        fout << ", ";
  //      }
  //    }
  //    fout.close();
  //    fout.open("examples/jumping/saved_trajs/com_traj/squat_times");
  //    for (double t : times) {
  //      fout << t << " ";
  //    }
  //    fout.flush();
  //    fout.close();
  //    std::cout << "Wrote Matrix " << std::endl;
  //  }
  return 0;
}

}
}
}

int main(int argc, char* argv[]) {
  return dairlib::examples::jumping::doMain(argc, argv);
}
