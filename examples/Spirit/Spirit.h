#ifndef _spirit
#define _spirit

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
#include "examples/Spirit/spirit_jump.h"

using drake::multibody::MultibodyPlant;
using drake::trajectories::PiecewisePolynomial;
using drake::geometry::SceneGraph;

namespace dairlib {
    using systems::trajectory_optimization::DirconModeSequence;
    using systems::trajectory_optimization::DirconMode;
    using systems::trajectory_optimization::Dircon;
    using std::vector;

template <template<class> class B,class T>
class Spirit {

public:
    Spirit(std::string yaml_path);
    void run();
    void animate();

private:
    int num_optimizations;
    bool initial_guess;
    std::string yaml_path;
    std::string saved_directory;

    std::unique_ptr<MultibodyPlant<T>> plant;
    std::unique_ptr<MultibodyPlant<T>> plant_vis;
    std::unique_ptr<SceneGraph<T>> scene_graph_ptr;
    drake::systems::DiagramBuilder<double> builder;
    
    B<T> behavior;
    PiecewisePolynomial<T> pp_xtraj; //For animation use

};
}


#endif
