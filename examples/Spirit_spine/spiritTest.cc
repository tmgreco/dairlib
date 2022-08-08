#include "examples/Spirit_spine/Spirit.h"

int main(int argc, char* argv[]) {
    google::ParseCommandLineFlags(&argc, &argv, true);
    // dairlib::Spirit<dairlib::SpiritJump,double> my_spirit("/home/feng/Downloads/dairlib/examples/Spirit_spine/yaml_configs/config.yaml");
    // dairlib::Spirit<dairlib::SpiritBound,double> my_spirit("/home/feng/Downloads/dairlib/examples/Spirit_spine/yaml_configs/bound_config.yaml");
    // dairlib::Spirit<dairlib::SpiritTrot,double> my_spirit("/home/feng/Downloads/dairlib/examples/Spirit_spine/yaml_configs/trot.yaml");
    dairlib::Spirit<dairlib::SpiritTrotHalf,double> my_spirit("/home/feng/Downloads/dairlib/examples/Spirit_spine/yaml_configs/trot_half.yaml");
    // dairlib::Spirit<dairlib::SpiritTurn,double> my_spirit("/home/feng/Downloads/dairlib/examples/Spirit_spine/yaml_configs/turn.yaml");
    // dairlib::Spirit<dairlib::SpiritBoundingGait,double> my_spirit("/home/feng/Downloads/dairlib/examples/Spirit_spine/yaml_configs/bounding_gait.yaml");
    // dairlib::Spirit<dairlib::SpiritBoxJump,double> my_spirit("/home/feng/Downloads/dairlib/examples/Spirit_spine/yaml_configs/box_jump_config.yaml");
    // dairlib::Spirit<dairlib::SpiritParkourJump,double> my_spirit("/home/feng/Downloads/dairlib/examples/Spirit_spine/yaml_configs/parkour_jump_config.yaml");
    // dairlib::Spirit<dairlib::SpiritParkourWallPronk,double> my_spirit("/home/feng/Downloads/dairlib/examples/Spirit_spine/yaml_configs/parkour_3wall_config.yaml");
    // dairlib::Spirit<dairlib::SpiritParkourWallPronk,double> my_spirit("/home/feng/Downloads/dairlib/examples/Spirit_spine/yaml_configs/parkour_wall_config.yaml");
    // dairlib::Spirit<dairlib::SpiritParkourWallPronk,double> my_spirit("/home/feng/Downloads/dairlib/examples/Spirit_spine/yaml_configs/parkour_wall_config_without_spine.yaml");
    my_spirit.run();
    my_spirit.animate();
}