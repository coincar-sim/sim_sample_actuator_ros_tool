#include "actuator_single_track_model.hpp"

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "actuator_single_track_model_node");

    sim_sample_actuator_ros_tool::ActuatorSingleTrackModel actuator_single_track_model(ros::NodeHandle(), ros::NodeHandle("~"));

    ros::spin();
    return 0;
}
