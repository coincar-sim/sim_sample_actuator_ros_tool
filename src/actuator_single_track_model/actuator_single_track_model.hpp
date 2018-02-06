#pragma once

#include <list>
#include <memory>
#include <string.h>
#include <time.h>
#include <dynamic_reconfigure/server.h>
#include <message_filters/cache.h>
#include <message_filters/subscriber.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Header.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <automated_driving_msgs/MotionState.h>
#include <automated_driving_msgs/ObjectState.h>
#include <automated_driving_msgs/ObjectStateArray.h>
#include <automated_driving_msgs/StampedFloat64.h>
#include <simulation_only_msgs/DeltaTrajectoryWithID.h>
#include <simulation_utils/util_perception.hpp>
#include <simulation_utils/util_planner.hpp>
#include <simulation_utils/util_single_track_model.hpp>

#include "sim_sample_actuator_ros_tool/ActuatorSingleTrackModelInterface.h"


namespace sim_sample_actuator_ros_tool {

class ActuatorSingleTrackModel {

    using Subscriber = message_filters::Subscriber<automated_driving_msgs::StampedFloat64>;
    using Cache = message_filters::Cache<automated_driving_msgs::StampedFloat64>;

public:
    ActuatorSingleTrackModel(ros::NodeHandle, ros::NodeHandle);

private:
    ros::Publisher vehicleMotionPublisher_;
    ros::Subscriber objectStateArraySub_;
    ros::Timer actuatorTimer_;
    std::unique_ptr<Subscriber> steeringSubscriber_;
    std::unique_ptr<Subscriber> accelerationSubscriber_;
    std::unique_ptr<Cache> cacheSteering_;
    std::unique_ptr<Cache> cacheAcceleration_;

    int objectID_;
    double accelerationBuffer_;
    double steeringBuffer_;
    double currentVelocity_;
    bool velocityValid_;
    bool controlCommandsReceived_;

    automated_driving_msgs::MotionState latestMotionState_;

    util_single_track_model::VehicleModelParams vehicleParams_;
    util_single_track_model::VehicleModel vehicleModel_;

    dynamic_reconfigure::Server<ActuatorSingleTrackModelConfig> reconfigSrv_; // Dynamic reconfiguration service

    ActuatorSingleTrackModelInterface params_;

    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    tf2_ros::TransformBroadcaster tfBroadcaster_;

    void actuatorTimerCallback(const ros::TimerEvent&);
    void steeringCallback(const automated_driving_msgs::StampedFloat64::ConstPtr&);
    void accelerationCallback(const automated_driving_msgs::StampedFloat64::ConstPtr&);
    void objectStateArraySubCallback(const automated_driving_msgs::ObjectStateArray::ConstPtr& msg);

    void publishDeltaTraj();

    void reconfigureRequest(ActuatorSingleTrackModelConfig&, uint32_t);
};

} // namespace sim_sample_actuator_ros_tool
