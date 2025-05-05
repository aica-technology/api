# Joint Trajectory Controller

## Overview

A Joint Trajectory Controller (JTC) is designed to track and execute smooth, time-parameterized trajectories for 
multiple joints simultaneously. Unlike simple position, velocity, acceleration, or effort controllers that command 
joints to move directly to a target state without regard to time or smoothness, JTC follows a full **_trajectory_**, 
_i.e._, a sequence of desired positions, velocities, acclerations, or forces each associated with specific timestamps. 

This ensures coordinated, continuous, and physically realistic joint motion. Compared to simple linear movement (where
joints might move individually or proportionally from one point to another), a joint trajectory approach offers several
key advantages:

- **Timing awareness**: Movements happen over a defined duration, allowing for synchronized motion across multiple
joints.
- **Smoothness**: In some cases, _e.g._ when velocity and acceleration are considered, transitions between waypoints 
are smooth, reducing mechanical stress and improving control performance.
- **Predictability**: The entire motion profile is predefined, allowing better anticipation of the system’s future
states, which is important for planning and safety.
- **Flexibility**: It can handle complex, multi-joint coordination tasks (*e.g*., following a curve) that simple
setpoint controllers cannot achieve easily (without further logic embedded in them).

In contrast to purely linear or direct controllers, JTC is ideal for applications requiring continuous, planned motion,
such as robotic arms or any system where joint coordination and motion quality are critical.

## Using it in AICA Core

A JTC is included in AICA Core by default and can be used via:

<!-- todo: no 1 to be updated once we start using our new Trajectory types -->
1. a **signal** (in this case, a single message) containing the joint trajectory; *i.e.*, one of joint positions,
velocities, accelerations, or effort, and their corresponding times from the start of the trajectory (refer to
[trajectory_msgs/JointTrajectoryPoint](https://docs.ros.org/en/noetic/api/trajectory_msgs/html/msg/JointTrajectoryPoint.html)). 
   
2. a **service** by providing one of the following:
   
   - **Cartesian frame names** (published as transformations; TFs) indicating the poses that should be traversed in a
joint-space motion.
   - **Joint position names** (published as joint frames; JFs) indicating the exact joint waypoints that JTC should
traverse.
  
Additional parameters can be modified to fine tune how JTC behaves (for example, PID gains for velocity control,
constraints for time and goal accuracy, and more). 
<!-- You may find JTC usage examples in **todo** and an advanced guide on
how to put an application together using AICA Studion in **todo**. -->