#ifndef _jump_configuration
#define _jump_configuration

#include <vector>
#include <string>

struct JumpConfiguration
{  
    double apex_goal; //for bad spirit jump
    double duration;
    bool ipopt;
    bool animate;
    std::vector<int> num_knot_points;
    double apex_height;
    double initial_height;
    double fore_aft_displacement;
    bool lock_rotation;
    bool lock_legs_apex;
    bool force_symmetry;
    bool use_nominal_stand;
    double max_duration;
    double cost_actuation;
    double cost_velocity;
    double cost_work;
    double mu;
    double eps;
    double tol;
    std::string file_name_out;
    std::string file_name_in="";
}  ;

#endif