#include "so3_control/NetworkControl.h"
int main(int argc, char **argv) {
    ros::init(argc, argv, "network_controller_node");
    NetworkControl controller;
    controller.init();
    // Single callback thread preserves the ROS2 mutually exclusive callback semantics.
    ros::spin();
    return 0;
}
