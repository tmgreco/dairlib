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
              "Number of timesteps to remain in neutral state");
const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision,
                                       Eigen::DontAlignCols, ", ", "\n");

namespace dairlib {

namespace examples {
namespace jumping {

int doMain(int argc, char* argv[]) {
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  PiecewisePolynomial<double> traj =
    loadTrajToPP("examples/jumping/saved_trajs/", "states.csv", "times", 3);
  std::string filename = "examples/jumping/five_link_biped.urdf";
  RigidBodyTree<double> tree;
  drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
    FindResourceOrThrow(filename), drake::multibody::joints::kFixed, &tree);

  KinematicsCache<double> cache = tree.CreateKinematicsCache();
  VectorXd q(tree.get_num_positions());
  Vector3d center_of_mass(3);
  // std::vector<double> knots;
  // std::vector<double> breaks;
  std::vector<double> points;
  std::vector<double> times;
  points.reserve(10000);
  for (auto const& element : multibody::makeNameToPositionsMap(tree))
    cout << element.first << " = " << element.second << endl;

  for (double i = 0; i < FLAGS_time_offset; i += FLAGS_time_offset / 200) {
    q << traj.value(0);
    // Order of states are not the same for multibody and rigid bodies
    dairlib::multibody::SetZeroQuaternionToIdentity(&q);
    cache.initialize(q);
    // std::cout << cache.getQ() << std::endl;
    tree.doKinematics(cache);
    center_of_mass = tree.centerOfMass(cache);
    times.push_back(i);
    // breaks.push_back(i);
    for (int j = 0; j < 3; ++j) {
      points.push_back(center_of_mass(j));
      // knots.push_back(center_of_mass(j));
    }
  }
  double time_offset = times.empty() ? 0 : times.back() + traj.end_time() /
                       200;
  for (double i = 0; i < traj.end_time(); i += traj.end_time() / 500) {
    q << traj.value(i);
    // Order of states are not the same for multibody and rigid bodies
    dairlib::multibody::SetZeroQuaternionToIdentity(&q);
    cache.initialize(q);
    // std::cout << cache.getQ() << std::endl;
    tree.doKinematics(cache);
    center_of_mass = tree.centerOfMass(cache);
    times.push_back(i + time_offset);
    // breaks.push_back(i);
    for (int j = 0; j < 3; ++j) {
      points.push_back(center_of_mass(j));
      // knots.push_back(center_of_mass(j));
    }
  }
  // MatrixXd com_pos_matrix =
  //   Eigen::Map<const Matrix<double, Dynamic, Dynamic, RowMajor>>(points.data(),
  //       points.size() / 3, 3);
  // MatrixXd com_pos_time_matrix =
  //   Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(times.data(),
  //       times.size());
  // MatrixXd knots_matrix = MatrixXd::Zero(3, knots.size()/3);
  // MatrixXd breaks_vector = VectorXd::Zero(breaks.size());
  // MatrixXd knots_matrix =
  //   Eigen::Map<const Matrix<double, Dynamic, Dynamic, RowMajor>>
  //   (knots.data(), 3, knots.size() / 3);
  // VectorXd breaks_vector =
  //   Eigen::Map<const Eigen::VectorXd, Eigen::Unaligned>(breaks.data(),
  //       breaks.size());
  // VectorXd breaks_vector = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(breaks.data(), breaks.size());
  // std::cout << "breaks: " << breaks_vector << std::endl;
  // std::cout << "knots_matrix: " << knots_matrix << std::endl;
  // PiecewisePolynomial<double> com_traj =
  //   PiecewisePolynomial<double>::FirstOrderHold(breaks_vector, knots_matrix);
  std::cout << "Creating matrix " << std::endl;
  // writePPTrajToFile(com_traj, "examples/jumping/saved_trajs/com_traj/",
  //                   "com_traj");
  // MatrixXd copy = com_pos_matrix;
  std::ofstream fout;
  fout.open("examples/jumping/saved_trajs/com_traj/squat.csv");
  int count = 0;
  for (double pt : points) {
    fout << pt;
    if (count++ % 3 == 2) {
      fout << "\n";
    } else {
      fout << ", ";
    }
  }
  // fout << Eigen::Map<const Matrix<double, Dynamic, Dynamic, RowMajor>>
  //      (points.data(),
  //       points.size() / 3, 3).format(CSVFormat);
  fout.close();
  // writeCSV("examples/jumping/saved_trajs/com_traj/com_pos_matrix.csv", com_pos_matrix);

  // std::ofstream fout;
  fout.open("examples/jumping/saved_trajs/com_traj/squat_times");
  for (double t : times) {
    fout << t << " ";
  }
  fout.flush();
  fout.close();
  std::cout << "Wrote Matrix " << std::endl;
  return 0;
}

}
}
}

int main(int argc, char* argv[]) {
  return dairlib::examples::jumping::doMain(argc, argv);
}
