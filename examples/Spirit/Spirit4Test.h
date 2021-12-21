#ifndef _spirit4test
#define _spirit4test

#include <cmath>
#include <experimental/filesystem>

#include "common/find_resource.h"
#include "systems/trajectory_optimization/dircon/dircon.h"
#include "multibody/kinematic/world_point_evaluator.h"

#include "multibody/kinematic/distance_evaluator.h"
#include "multibody/multibody_utils.h"
#include "multibody/visualization_utils.h"
#include "multibody/kinematic/kinematic_constraints.h"

#include "examples/Spirit/animate_spirit.h"
#include "common/file_utils.h"
#include "lcm/dircon_saved_trajectory.h"
#include "examples/Spirit/spirit_box_jump.h"
#include "examples/Spirit/surface_conf.h"


using drake::multibody::MultibodyPlant;
using drake::trajectories::PiecewisePolynomial;
using drake::geometry::SceneGraph;

namespace dairlib {
template <template<class> class B,class T>
class Spirit4Test {
public:
    Spirit4Test();
    void run();
    void animate();

private:
    int num_optimizations;
    std::string initial_guess;
    std::string yaml_path;
    std::string saved_directory;

    std::unique_ptr<MultibodyPlant<T>> plant;
    std::unique_ptr<MultibodyPlant<T>> plant_vis;
    std::unique_ptr<SceneGraph<T>> scene_graph_ptr;
    drake::systems::DiagramBuilder<double> builder;
    
    B<T> behavior;
    PiecewisePolynomial<T> pp_xtraj; //For animation use
    std::vector<SurfaceConf> surface_vector;
};
}


#endif
