#include "actuator_single_track_model.hpp"
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace sim_sample_actuator_ros_tool {

class ActuatorSingleTrackModelNodelet : public nodelet::Nodelet {

    virtual void onInit();
    boost::shared_ptr<ActuatorSingleTrackModel> m_;
};

void ActuatorSingleTrackModelNodelet::onInit() {
    m_.reset(new ActuatorSingleTrackModel(getNodeHandle(), getPrivateNodeHandle()));
}

} // namespace sim_sample_actuator_ros_tool

PLUGINLIB_DECLARE_CLASS(sim_sample_actuator_ros_tool, ActuatorSingleTrackModelNodelet, sim_sample_actuator_ros_tool::ActuatorSingleTrackModelNodelet, nodelet::Nodelet);
