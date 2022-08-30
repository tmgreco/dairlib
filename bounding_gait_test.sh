# !/bin/bash
sleep 1s
gnome-terminal --title="rigid 1-43" --command="bazel run //examples/Spirit_spine:spirit_test -- --skip_to=1 --end_at=43 --yaml_path=/examples/Spirit_spine/yaml_configs/bounding_gait.yaml"
sleep 1s
gnome-terminal --title="twist 1-43" --command="bazel run //examples/Spirit_spine:spirit_test -- --skip_to=1 --end_at=43 --yaml_path=/examples/Spirit_spine/yaml_configs/bounding_gait_twist.yaml"


sleep 20m
gnome-terminal --title="rigid 44-84" --command="bazel run //examples/Spirit_spine:spirit_test -- --skip_to=44 --end_at=84 --yaml_path=/examples/Spirit_spine/yaml_configs/bounding_gait.yaml"
sleep 1s
gnome-terminal --title="twist 44-84" --command="bazel run //examples/Spirit_spine:spirit_test -- --skip_to=44 --end_at=84 --yaml_path=/examples/Spirit_spine/yaml_configs/bounding_gait_twist.yaml"



sleep 1h
gnome-terminal --title="rigid 85-97" --command="bazel run //examples/Spirit_spine:spirit_test -- --skip_to=85 --end_at=97 --yaml_path=/examples/Spirit_spine/yaml_configs/bounding_gait.yaml"
sleep 1s
gnome-terminal --title="rigid 98-110" --command="bazel run //examples/Spirit_spine:spirit_test -- --skip_to=98 --end_at=110 --yaml_path=/examples/Spirit_spine/yaml_configs/bounding_gait.yaml"
sleep 1s
gnome-terminal --title="rigid 111-125" --command="bazel run //examples/Spirit_spine:spirit_test -- --skip_to=111 --end_at=125 --yaml_path=/examples/Spirit_spine/yaml_configs/bounding_gait.yaml"
sleep 1s
gnome-terminal --title="twist 85-97" --command="bazel run //examples/Spirit_spine:spirit_test -- --skip_to=85 --end_at=97 --yaml_path=/examples/Spirit_spine/yaml_configs/bounding_gait_twist.yaml"
sleep 1s
gnome-terminal --title="twist 98-110" --command="bazel run //examples/Spirit_spine:spirit_test -- --skip_to=98 --end_at=110 --yaml_path=/examples/Spirit_spine/yaml_configs/bounding_gait_twist.yaml"
sleep 1s
gnome-terminal --title="twist 111-125" --command="bazel run //examples/Spirit_spine:spirit_test -- --skip_to=111 --end_at=125 --yaml_path=/examples/Spirit_spine/yaml_configs/bounding_gait_twist.yaml"

sleep 1h
gnome-terminal --title="rigid 126-138" --command="bazel run //examples/Spirit_spine:spirit_test -- --skip_to=129 --end_at=138 --yaml_path=/examples/Spirit_spine/yaml_configs/bounding_gait.yaml"
sleep 1s
gnome-terminal --title="rigid 139-151" --command="bazel run //examples/Spirit_spine:spirit_test -- --skip_to=139 --end_at=151 --yaml_path=/examples/Spirit_spine/yaml_configs/bounding_gait.yaml"
sleep 1s
gnome-terminal --title="rigid 152-166" --command="bazel run //examples/Spirit_spine:spirit_test -- --skip_to=152 --end_at=166 --yaml_path=/examples/Spirit_spine/yaml_configs/bounding_gait.yaml"

sleep 1s
gnome-terminal --title="twist 126-138" --command="bazel run //examples/Spirit_spine:spirit_test -- --skip_to=126 --end_at=138 --yaml_path=/examples/Spirit_spine/yaml_configs/bounding_gait_twist.yaml"
sleep 1s
gnome-terminal --title="twist 139-151" --command="bazel run //examples/Spirit_spine:spirit_test -- --skip_to=139 --end_at=151 --yaml_path=/examples/Spirit_spine/yaml_configs/bounding_gait_twist.yaml"
sleep 1s
gnome-terminal --title="twist 152-166" --command="bazel run //examples/Spirit_spine:spirit_test -- --skip_to=152 --end_at=166 --yaml_path=/examples/Spirit_spine/yaml_configs/bounding_gait_twist.yaml"

