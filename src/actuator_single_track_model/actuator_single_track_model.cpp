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

#include <stdio.h>
#include <util_automated_driving_msgs/util_automated_driving_msgs.hpp>
#include <util_geometry_msgs/util_geometry_msgs.hpp>

#include "actuator_single_track_model.hpp"

namespace sim_sample_actuator_ros_tool {

ActuatorSingleTrackModel::ActuatorSingleTrackModel(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle)
        : reconfigSrv_{private_node_handle}, params_{private_node_handle}, tfListener_{tfBuffer_} {

    /**
     * Initialization
     */
    params_.fromParamServer();

    vehicleParams_.c_v = params_.c_v;
    vehicleParams_.c_h = params_.c_h;
    vehicleParams_.l_v = params_.l_v;
    vehicleParams_.l_h = params_.l_h;
    vehicleParams_.m = params_.m;
    vehicleParams_.i_sq = params_.i_sq;
    vehicleParams_.J = vehicleParams_.i_sq * vehicleParams_.m;
    vehicleModel_.initializeModel(params_.vehicle_id, vehicleParams_);

    accelerationBuffer_ = 0.0;
    steeringBuffer_ = 0.0;
    latestMotionState_.pose.covariance = util_geometry_msgs::checks::covarianceUnkownValues;
    latestMotionState_.twist.covariance = util_geometry_msgs::checks::covarianceUnkownValues;
    controlCommandsReceived_ = false;
    /**
     * Set up dynamic reconfiguration
     */
    reconfigSrv_.setCallback(boost::bind(&ActuatorSingleTrackModel::reconfigureRequest, this, _1, _2));

    /**
     * Publishers & subscriber
     */
    vehicleMotionPublisher_ = node_handle.advertise<simulation_only_msgs::DeltaTrajectoryWithID>(
        params_.desired_motion_out_topic, params_.msg_queue_size);

    actuatorTimer_ = node_handle.createTimer(
        ros::Duration(1. / params_.actuator_frequency), &ActuatorSingleTrackModel::actuatorTimerCallback, this);
    objectStateArraySub_ = node_handle.subscribe(params_.objects_ground_truth_topic_with_ns,
                                                 params_.msg_queue_size,
                                                 &ActuatorSingleTrackModel::objectStateArraySubCallback,
                                                 this);

    steeringSubscriber_ = std::make_unique<Subscriber>(
        node_handle, params_.steering_angle_in_topic, params_.msg_queue_size, ros::TransportHints().tcpNoDelay());
    accelerationSubscriber_ = std::make_unique<Subscriber>(
        node_handle, params_.acceleration_in_topic, params_.msg_queue_size, ros::TransportHints().tcpNoDelay());
    cacheSteering_ = std::make_unique<Cache>(*steeringSubscriber_, params_.cache_size);
    cacheSteering_->registerCallback(boost::bind(&ActuatorSingleTrackModel::steeringCallback, this, _1));
    cacheAcceleration_ = std::make_unique<Cache>(*accelerationSubscriber_, params_.cache_size);
    cacheAcceleration_->registerCallback(boost::bind(&ActuatorSingleTrackModel::accelerationCallback, this, _1));
}

void ActuatorSingleTrackModel::steeringCallback(const automated_driving_msgs::StampedFloat64::ConstPtr& msg) {

    const automated_driving_msgs::StampedFloat64::ConstPtr msg_cached{
        cacheSteering_->getElemBeforeTime(ros::Time(ros::Time::now().toSec() - params_.delay_steering))};
    if (msg_cached == NULL)
        return;

    controlCommandsReceived_ = true;
    steeringBuffer_ = msg_cached->data;
}

void ActuatorSingleTrackModel::accelerationCallback(const automated_driving_msgs::StampedFloat64::ConstPtr& msg) {

    const automated_driving_msgs::StampedFloat64::ConstPtr msg_cached{
        cacheAcceleration_->getElemBeforeTime(ros::Time(ros::Time::now().toSec() - params_.delay_acceleration))};
    if (msg_cached == NULL)
        return;

    controlCommandsReceived_ = true;
    accelerationBuffer_ = msg_cached->data;
}

void ActuatorSingleTrackModel::objectStateArraySubCallback(
    const automated_driving_msgs::ObjectStateArray::ConstPtr& msg) {
    // taken from sim_sample_perception_ros_tool/exact_localization_sensor

    bool foundAndUnique;
    automated_driving_msgs::ObjectState egoObjectState =
        util_automated_driving_msgs::conversions::objectStateFromObjectStateArray(
            msg, params_.vehicle_id, foundAndUnique);

    if (!foundAndUnique) {
        return;
    } else {
        ros::Duration deltaTime = egoObjectState.header.stamp - latestMotionState_.header.stamp;

        if (!util_automated_driving_msgs::checks::poseValid(egoObjectState.motion_state)) {
            // ROS_DEBUG("Received MotionState.pose is marked as unreliable. Forwarding it
            // anyway.");
        }

        util_automated_driving_msgs::computations::incorporatePrecedingDataToMotionstate(latestMotionState_,
                                                                                         egoObjectState.motion_state);

        if (util_automated_driving_msgs::checks::twistValid(egoObjectState.motion_state)) {
            double v_x = egoObjectState.motion_state.twist.twist.linear.x;
            double v_y = egoObjectState.motion_state.twist.twist.linear.y;
            double v_z = egoObjectState.motion_state.twist.twist.linear.z;
            currentVelocity_ = std::sqrt(v_x * v_x + v_y * v_y + v_z * v_z);
            velocityValid_ = true;
        } else {
            velocityValid_ = false;
            ROS_DEBUG_THROTTLE(1, "Could not calculate current velocity as latest pose not valid.");
        }
    }

    // set new EgoObjectState state as latest EgoObjectState state
    latestMotionState_ = egoObjectState.motion_state;
}

/**
  * This callback is called whenever a change was made in the dynamic_reconfigure window
*/
void ActuatorSingleTrackModel::reconfigureRequest(ActuatorSingleTrackModelConfig& config, uint32_t level) {
    params_.fromConfig(config);
    actuatorTimer_.setPeriod(ros::Duration(1. / params_.actuator_frequency));
}

void ActuatorSingleTrackModel::actuatorTimerCallback(const ros::TimerEvent& event) {
    publishDeltaTraj();
}

void ActuatorSingleTrackModel::publishDeltaTraj() {

    if (velocityValid_ && controlCommandsReceived_) {
        // ROS_INFO_STREAM("current velocity: " << currentVelocity_);
        simulation_only_msgs::DeltaTrajectoryWithID msg;
        msg = vehicleModel_.toDeltaTrajectoryMsg(currentVelocity_,
                                                 accelerationBuffer_,
                                                 steeringBuffer_,
                                                 params_.delta_t,
                                                 params_.horizon_t,
                                                 ros::Time::now());

        vehicleMotionPublisher_.publish(msg);
    } else {
        ROS_WARN_THROTTLE(1, "Currently not publishing any desired motion");
    }
}

} // namespace sim_sample_actuator_ros_tool
