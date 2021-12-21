#include "examples/Spirit/Spirit.h"
#include "examples/Spirit/Spirit4Test.h"
int main(int argc, char* argv[]) {
    google::ParseCommandLineFlags(&argc, &argv, true);
    // dairlib::Spirit<dairlib::SpiritJump,double> my_spirit("/home/feng/Downloads/dairlib/examples/Spirit/config.yaml");
    // my_spirit.run();
    // my_spirit.animate();
    dairlib::Spirit4Test<dairlib::SpiritBoxJump,double> my_spirit;
    my_spirit.run();
    my_spirit.animate();
}