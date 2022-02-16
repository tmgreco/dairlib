#include "examples/Spirit/Spirit.h"

int main(int argc, char* argv[]) {
    google::ParseCommandLineFlags(&argc, &argv, true);
    // dairlib::Spirit<dairlib::SpiritBound,double> my_spirit("/home/feng/Downloads/dairlib/examples/Spirit/bound_config.yaml");
    // dairlib::Spirit<dairlib::SpiritBoxJump,double> my_spirit("/home/feng/Downloads/dairlib/examples/Spirit/box_jump_config.yaml");
    dairlib::Spirit<dairlib::SpiritParkourJump,double> my_spirit("/home/feng/Downloads/dairlib/examples/Spirit/parkour_jump_config.yaml");
    my_spirit.run();
    my_spirit.animate();
}