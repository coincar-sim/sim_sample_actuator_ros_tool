/*
 * Copyright (c) 2017
 * FZI Forschungszentrum Informatik, Karlsruhe, Germany (www.fzi.de)
 * KIT, Institute of Measurement and Control, Karlsruhe, Germany (www.mrt.kit.edu)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <dynamic_reconfigure/server.h>
#include <message_filters/cache.h>
#include <message_filters/subscriber.h>
#include <ros/ros.h>

#include <automated_driving_msgs/MotionState.h>
#include <automated_driving_msgs/ObjectState.h>
#include <automated_driving_msgs/ObjectStateArray.h>
#include <automated_driving_msgs/StampedFloat64.h>
#include <simulation_only_msgs/DeltaTrajectoryWithID.h>

#include "util_single_track_model.hpp"

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

    void actuatorTimerCallback(const ros::TimerEvent&);
    void steeringCallback(const automated_driving_msgs::StampedFloat64::ConstPtr&);
    void accelerationCallback(const automated_driving_msgs::StampedFloat64::ConstPtr&);
    void objectStateArraySubCallback(const automated_driving_msgs::ObjectStateArray::ConstPtr& msg);

    void publishDeltaTraj();

    void reconfigureRequest(ActuatorSingleTrackModelConfig&, uint32_t);
};

} // namespace sim_sample_actuator_ros_tool
