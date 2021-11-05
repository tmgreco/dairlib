#ifndef _jump_configuration
#define _jump_configuration

#include <vector>
#include <string>

struct JumpConfiguration
{  
    double apex_goal; //for bad spirit jump
    double duration;
    bool ipopt;
    const bool animate;
    std::vector<int> num_knot_points;
    const double apex_height;
    const double initial_height;
    const double fore_aft_displacement;
    const bool lock_rotation;
    const bool lock_legs_apex;
    const bool force_symmetry;
    const bool use_nominal_stand;
    const double max_duration;
    const double cost_actuation;
    const double cost_velocity;
    const double cost_work;
    const double mu;
    const double eps;
    const double tol;
    const std::string file_name_out;
    const std::string file_name_in="";
}  ;

#endif