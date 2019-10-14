#include <chrono>
#include <thread>

#include <gflags/gflags.h>
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


namespace dairlib{

namespace examples{
namespace jumping{

int doMain(int argc, char* argv[]){
	PiecewisePolynomial<double> traj = loadStateTrajToPP("examples/jumping/saved_trajs/", 3);
	std::string filename = "examples/jumping/five_link_biped.urdf";
	RigidBodyTree<double> tree;
	drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
		FindResourceOrThrow(filename), drake::multibody::joints::kFixed, &tree);
	KinematicsCache<double> cache = tree.CreateKinematicsCache();

	VectorXd q(tree.get_num_positions());
	Vector3d center_of_mass(3);
	std::vector<double> knots;
	std::vector<double> breaks;
	for(double i = 0; i < traj.end_time(); i += traj.end_time() / 500){
		q << traj.value(i);
		cache.initialize(q);
		tree.doKinematics(cache);
		center_of_mass = tree.centerOfMass(cache);
		// points.push_back(i);
		breaks.push_back(i);
		for(int j = 0; j < 3; ++j){
			// points.push_back(center_of_mass(j));
			knots.push_back(center_of_mass(j));
		}
	}
	// std::cout << "points size: " << points.size() << std::endl;
	MatrixXd com_pos_matrix = Eigen::Map<const Matrix<double, Dynamic, Dynamic, RowMajor>>(points.data(), points.size()/4, 4);
	MatrixXd knots_matrix = MatrixXd::Zero(3, knots.size()/3);
	MatrixXd breaks_vector = VectorXd::Zero(breaks.size());
	knots_matrix = Eigen::Map<const Matrix<double, Dynamic, Dynamic>>(knots.data(), 3, knots.size()/3);
	breaks_vector = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(breaks.data(), breaks.size());
	// VectorXd breaks_vector = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(breaks.data(), breaks.size());
	// std::cout << "breaks: " << breaks_vector << std::endl;
	// std::cout << "knots_matrix: " << knots_matrix << std::endl;
	PiecewisePolynomial<double> com_traj = PiecewisePolynomial<double>::FirstOrderHold(breaks_vector, knots_matrix);
	std::cout << "Creating matrix " << std::endl;
	writePPTrajToFile(com_traj, "examples/jumping/saved_trajs/com_traj/", "com_traj");
	MatrixXd copy = com_pos_matrix;
	writeCSV("com_pos_matrix.csv", copy);
	return 0;
}

}
}
}
int main(int argc, char* argv[]) { 
	return dairlib::examples::jumping::doMain(argc, argv); 
}
