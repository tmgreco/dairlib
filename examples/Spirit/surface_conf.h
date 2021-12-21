#ifndef surface_conf
#define surface_conf

#include <Eigen/StdVector>

struct SurfaceConf
{
    Eigen::Vector3d surface_normal;
    Eigen::Vector3d surface_offset;
    double length_surf;
    double width_surf;
    double thickness_surf;
    const drake::Vector4<double> color;
};


#endif