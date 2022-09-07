#include "animate_spirit.h"
#include <gflags/gflags.h>


int main(int argc, char* argv[]) {
    google::ParseCommandLineFlags(&argc, &argv, true);
    std::string urdf_path="/home/kodlab/dairlib/examples/Spirit_spine/spirit_with_spine_drake.urdf";
    dairlib::animateTraj(urdf_path);
}