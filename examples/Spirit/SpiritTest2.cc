#include "examples/Spirit/Spirit.h"

int main(int argc, char* argv[]) {
    dairlib::Spirit<dairlib::SpiritJump,double> my_spirit("/home/feng/Downloads/dairlib/examples/Spirit/config.yaml");
    my_spirit.run();
}