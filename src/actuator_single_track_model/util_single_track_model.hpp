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

#include <stdexcept>

#include <ros/ros.h>
#include <ros/time.h>

#include "simulation_only_msgs/DeltaTrajectoryWithID.h"

namespace util_single_track_model {

struct VehicleModelParams {
    double c_v;  // tire slip stiffness front [N/rad = N]
    double c_h;  // tire slip stiffness rear [N/rad = N]
    double l_v;  // length center of gravity - front axle [m]
    double l_h;  // length center of gravity - rear axle [m]
    double m;    // vehicle mass [kg]
    double i_sq; // square of effective radius for moment of intertia [m^2]
    double J;    // moment of intertia J = m i^2 = m i_sq [kg m^2]
};

struct VehicleModelState {
    double x;       // x-position [m]
    double y;       // y-position [m]
    double theta;   // yaw angle [rad] (0 = x-direction)
    double theta_d; // first time derivative of yaw angle [rad/s]
    double v;       // absolute velocity [m/s]
    double a;       // longitudinal acceleration [m/s^2]
    double beta;    // side slip angle [rad]
    double delta;   // steering angle [rad]
};

class VehicleModel {
public:
    VehicleModel();

    void initializeModel(const int objectID,
                         const VehicleModelParams params,
                         const double min_acceleration = 0.01,
                         const double min_velocity = 0.1);
    // if min_velocity and min_acceleration are deceeded at the same time, a full stop is assumed

    simulation_only_msgs::DeltaTrajectoryWithID toDeltaTrajectoryMsg(const double v,
                                                                     const double a,
                                                                     const double delta,
                                                                     const double delta_t,
                                                                     const double horizon_t,
                                                                     const ros::Time& timestamp);


private:
    int objectID_;
    double delta_t_;
    double horizon_t_;
    double min_acceleration_;
    double min_velocity_;
    VehicleModelParams vehicleParams_;
    std::vector<VehicleModelState> states_;
    ros::Time lastTimestamp_;

    static double getBetaByTimestamp(const ros::Time currentTimestamp,
                                     const ros::Time lastTimestamp,
                                     const std::vector<VehicleModelState> states,
                                     const double delta_t);
    static double getThetaDByTimestamp(const ros::Time currentTimestamp,
                                       const ros::Time lastTimestamp,
                                       const std::vector<VehicleModelState> states,
                                       const double delta_t);
    void initializeState(const double v, const double a, const double delta, const double beta, const double theta);
    VehicleModelState calcNextState(const VehicleModelState currState);
};

} // namespace util_single_track_model
