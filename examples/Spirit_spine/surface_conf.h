#ifndef surface_conf
#define surface_conf

#include <Eigen/StdVector>
#include "drake/geometry/drake_visualizer.h"

struct SurfaceConf
{
    Eigen::Vector3d surface_normal;
    Eigen::Vector3d surface_offset;
    double length_surf;
    double width_surf;
    double thickness_surf;
    const drake::Vector4<double> color;
    std::string name;
};


#endif