[![Build Status](https://api.travis-ci.org/coincar-sim/sim_sample_actuator_ros_tool.svg)](https://travis-ci.org/coincar-sim/sim_sample_actuator_ros_tool)

# sim_sample_actuator_ros_tool
Sample actuator module for a vehicle in the simulation framework.

#### actuator_single_track_model
* computes a DeltaTrajectory from motion_state and the control inputs steering angle and acceleration, based on the single track model, considering a deadtime.

## Installation
* this package is part of the simulation framework
* see [coincarsim_getting_started](https://github.com/coincar-sim/coincarsim_getting_started) for installation and more details

## Usage
* started within the a vehicle launchfile of the simulation_initialization_ros_tool

#### Parameters
* parameters that need to be passed to the launchfile `sample_actuator.launch`:
  * **vehicle_id**: Id of the vehicle, needs to be unique within the framework
  * **vehicle_ns**: Namespace of the vehicle, needs to be unique within the framework

  * **objects_ground_truth_topic_with_ns**: Topic under which the ground truth states of the objects are received
  * **steering_angle_in_topic**: Topic under which the current steering angle is received
  * **acceleration_in_topic**: Topic under which the current acceleration is received
  * **desired_motion_topic_with_ns**: Topic under which the desired motion of the vehicle is published

  * **vehicle_parameter_file**: Path to the file that contains the parameters for the single track model

* parameters that can be dynamically reconfigured
  * **horizon_t**: Time horizon for motion calculation [s]
  * **delay_steering**: Steering time delay [s]
  * **delay_acceleration**: Acceleration time delay [s]
  * **actuator_frequency**: Actuator frequency [Hz]

## Contribution
* fork this repo
* implement your own vehicle model
* ensure that
  * `$(arg desired_motion_topic_with_ns)` is published
  * all internal ROS communication stays within the perception namespace

## Contributors
Nick Engelhardt, Maximilian Naumann, Sascha Wirges

## License
This package is distributed under the 3-Clause BSD License, see [LICENSE](LICENSE).
