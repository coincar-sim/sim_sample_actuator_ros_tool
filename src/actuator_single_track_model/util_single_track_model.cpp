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

#include <util_geometry_msgs/util_geometry_msgs.hpp>
#include <ros/ros.h>

#include "util_single_track_model.hpp"

namespace util_single_track_model {

VehicleModel::VehicleModel() {
}

void VehicleModel::initializeModel(const int objectID,
                                   const VehicleModelParams params,
                                   const double min_acceleration,
                                   const double min_velocity) {
    objectID_ = objectID;
    vehicleParams_ = params;
    min_acceleration_ = min_acceleration;
    min_velocity_ = min_velocity;
    if (min_velocity_ < 0.0) {
        throw std::logic_error("Single track model only implemented for positive velocity!");
    }
}

simulation_only_msgs::DeltaTrajectoryWithID VehicleModel::toDeltaTrajectoryMsg(const double v,
                                                                               const double a,
                                                                               const double delta,
                                                                               const double delta_t,
                                                                               const double horizon_t,
                                                                               const ros::Time& timestamp) {

    double initial_beta = 0.0;
    double initial_theta_d = 0.0;
    if (lastTimestamp_.isValid()) {
        initial_beta = getBetaByTimestamp(timestamp, lastTimestamp_, states_, delta_t_);
        initial_theta_d = getThetaDByTimestamp(timestamp, lastTimestamp_, states_, delta_t_);
    }

    delta_t_ = delta_t;
    horizon_t_ = horizon_t;
    bool early_stop = false;
    double v_init = v;
    if (v < 0.5 * min_velocity_) {
        v_init = 0.5 * min_velocity_;
        if (v < 0.0) {
            ROS_ERROR_THROTTLE(1,
                               "Current velocity is v=%.2f < 0 but this implementation only allows positive velocity. "
                               "Setting it to v=%.2f",
                               v,
                               v_init);
        }
    }
    initializeState(v_init, a, delta, initial_beta, initial_theta_d);
    for (double t = delta_t; t < horizon_t; t += delta_t) {
        if (states_.back().a < min_acceleration_ && states_.back().v < min_velocity_) {
            // if min_velocity and min_acceleration are deceeded at the same time, a full stop is assumed
            early_stop = true;
            states_.push_back(states_.back());
            states_.back().beta = 0.0;
            if (t > delta_t) {
                ROS_INFO_THROTTLE(2,
                                  "Current control commands lead to full stop in %.2fs (as velocity and acceleration "
                                  "do not exceed the given bound).",
                                  t - delta_t);
            } else {
                ROS_INFO_THROTTLE(
                    2, "Currently at a full stop (as velocity and acceleration do not exceed the given bound).");
            }

            break;
        }
        states_.push_back(calcNextState(states_.back()));
    }

    // create the message
    simulation_only_msgs::DeltaTrajectoryWithID deltaTraj;
    deltaTraj.object_id = objectID_;
    deltaTraj.header.stamp = timestamp;

    for (size_t i = 0; i < states_.size(); i++) {

        // delta_pose with orientation of previous section
        if (i > 0) {
            automated_driving_msgs::DeltaPoseWithDeltaTime dpwdt_p;
            ros::Duration duration;
            dpwdt_p.delta_time = duration.fromSec(double(i) * delta_t);
            dpwdt_p.delta_pose.position.x = states_[i].x;
            dpwdt_p.delta_pose.position.y = states_[i].y;
            dpwdt_p.delta_pose.position.z = 0.0;
            dpwdt_p.delta_pose.orientation = util_geometry_msgs::conversions::quaternionFromYaw(states_[i].theta);
            deltaTraj.delta_poses_with_delta_time.push_back(dpwdt_p);
        }

        // delta_pose with orientation of next section
        if (i < states_.size() - 1) {
            automated_driving_msgs::DeltaPoseWithDeltaTime dpwdt;
            ros::Duration duration;
            dpwdt.delta_time = duration.fromSec(double(i) * delta_t);
            dpwdt.delta_pose.position.x = states_[i].x;
            dpwdt.delta_pose.position.y = states_[i].y;
            dpwdt.delta_pose.position.z = 0.0;
            dpwdt.delta_pose.orientation = util_geometry_msgs::conversions::quaternionFromYaw(states_[i + 1].theta);
            deltaTraj.delta_poses_with_delta_time.push_back(dpwdt);
        }
    }

    if (early_stop) {
        automated_driving_msgs::DeltaPoseWithDeltaTime dpwdt;
        ros::Duration duration;
        dpwdt.delta_time = duration.fromSec(horizon_t);
        dpwdt.delta_pose = deltaTraj.delta_poses_with_delta_time.back().delta_pose;
        deltaTraj.delta_poses_with_delta_time.push_back(dpwdt);
    }

    lastTimestamp_ = timestamp;

    return deltaTraj;
}


double VehicleModel::getBetaByTimestamp(const ros::Time currentTimestamp,
                                        const ros::Time lastTimestamp,
                                        const std::vector<VehicleModelState> states,
                                        const double delta_t) {
    double currentDeltaT = currentTimestamp.toSec() - lastTimestamp.toSec();
    double beta = 0.0;
    if (currentDeltaT < 0.0) {
        return beta;
    }
    if (currentDeltaT > double(states.size()) * delta_t) {
        return beta;
    }


    for (size_t i = 0; i < states.size(); i++) {
        if (currentDeltaT < double(i) * delta_t) {
            beta = states.at(i).beta;

            break;
        }
    }
    return beta;
}

double VehicleModel::getThetaDByTimestamp(const ros::Time currentTimestamp,
                                          const ros::Time lastTimestamp,
                                          const std::vector<VehicleModelState> states,
                                          const double delta_t) {
    double currentDeltaT = currentTimestamp.toSec() - lastTimestamp.toSec();
    double theta_d = 0.0;
    if (currentDeltaT < 0.0) {
        return theta_d;
    }
    if (currentDeltaT > double(states.size()) * delta_t) {
        return theta_d;
    }


    for (size_t i = 0; i < states.size(); i++) {
        if (currentDeltaT < double(i) * delta_t) {
            theta_d = states.at(i).theta_d;

            break;
        }
    }
    return theta_d;
}

void VehicleModel::initializeState(
    const double v, const double a, const double delta, const double beta, const double theta_d) {
    states_.clear();
    VehicleModelState initialState;
    initialState.x = 0.0;
    initialState.y = 0.0;
    initialState.theta = 0.0;
    initialState.theta_d = theta_d;
    initialState.v = v;
    initialState.a = a;
    initialState.delta = delta;
    initialState.beta = beta;
    states_.push_back(initialState);
}


VehicleModelState VehicleModel::calcNextState(const VehicleModelState currState) {

    double x_d_ = currState.v * cos(currState.theta + currState.beta);
    double y_d_ = currState.v * sin(currState.theta + currState.beta);
    double theta_d_ = currState.theta_d;
    double v_d_ = currState.a;
    double a_d_ = 0.0;     // const. acceleration
    double delta_d_ = 0.0; // const. steering angle

    double theta_d_d_, beta_d_;

    const double v_dynamic_min = 3.0;
    if (currState.v > v_dynamic_min) {
        // source : https: // de.wikipedia.org/wiki/Einspurmodell#Bewegungsgleichungen
        theta_d_d_ = -(vehicleParams_.c_h * vehicleParams_.l_h - vehicleParams_.c_v * vehicleParams_.l_v) /
                         vehicleParams_.J * currState.beta -
                     (vehicleParams_.c_h * vehicleParams_.l_h * vehicleParams_.l_h +
                      vehicleParams_.c_v * vehicleParams_.l_v * vehicleParams_.l_v) /
                         (vehicleParams_.J * currState.v) * currState.theta_d +
                     vehicleParams_.c_v * vehicleParams_.l_v / vehicleParams_.J * currState.delta;

        beta_d_ = -(vehicleParams_.c_v + vehicleParams_.c_h) / (vehicleParams_.m * currState.v) * currState.beta +
                  (vehicleParams_.m * currState.v * currState.v -
                   (vehicleParams_.c_h * vehicleParams_.l_h - vehicleParams_.c_v * vehicleParams_.l_v)) /
                      (vehicleParams_.m * currState.v * currState.v) * currState.theta_d -
                  vehicleParams_.c_v / (vehicleParams_.m * currState.v) * currState.delta;
    } else {
        double beta_kinematic =
            atan(vehicleParams_.l_h / (vehicleParams_.l_h + vehicleParams_.l_v) * tan(currState.delta));
        double theta_d_kinematic =
            currState.v * cos(beta_kinematic) * tan(currState.delta) / (vehicleParams_.l_h + vehicleParams_.l_v);

        theta_d_d_ = 1. / delta_t_ * double(theta_d_kinematic - theta_d_);
        beta_d_ = 1. / delta_t_ * double(beta_kinematic - currState.beta);
    }


    VehicleModelState nextState;
    nextState.x = currState.x + delta_t_ * x_d_;
    nextState.y = currState.y + delta_t_ * y_d_;
    nextState.theta = currState.theta + delta_t_ * theta_d_;
    nextState.theta_d = currState.theta_d + delta_t_ * theta_d_d_;
    nextState.v = currState.v + delta_t_ * v_d_;
    nextState.a = currState.a + delta_t_ * a_d_;
    nextState.delta = currState.delta + delta_t_ * delta_d_;
    nextState.beta = currState.beta + delta_t_ * beta_d_;

    return nextState;
}

} // namespace util_single_track_model
