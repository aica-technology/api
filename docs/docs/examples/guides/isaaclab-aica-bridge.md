---
sidebar_position: 11
title: A guide for using Isaac Lab as a simulator for AICA System
---

import application from './assets/isaaclab-aica-bridge-application.png'

# A guide for using Isaac Lab as a simulator for AICA System
By interfacing the AICA System with Isaac Lab, we establish a workflow for developing, testing, and deploying robotic
applications. This connection provides several key benefits:

1. **RL Policy Testing**: AICA’s RL Policy Component SDK allows developers to deploy Reinforcement Learning (RL) models
   directly onto real hardware through components. These models can be trained in Isaac Lab, and with the AICA System
   interacting directly with Isaac Lab, users can validate trained policies under the same conditions in which they were
   learned.

2. **Physics-Based Evaluation**: Running the AICA System within a physics-based simulation such as Isaac Lab allows
   developers to observe how control algorithms respond to realistic dynamics, friction, collisions, and sensor noise.
   This ensures that behaviors tested in simulation mirror real-world performance, reducing the risk of unexpected
   failures during deployment and enabling safer, more reliable policy tuning before engaging with physical robots.

3. **Digital Twin Control**: Beyond RL, running the AICA System with Isaac Lab provides users with ways to interact with
   digital twins of their robots. Applications can be authored, tested, and validated entirely in simulation before
   connecting to actual hardware. This improves safety and enables rapid iteration in early stages, helping streamline
   the overall development cycle.

With Isaac Lab as simulator, users can build scenes in Isaac Lab, command simulated robots using AICA System, validate
performance, switch the hardware interface to a real robot, and hit play with no code changes required.

## Installing Isaac Lab

Begin by cloning AICA's fork of
[Isaac Lab](https://github.com/aica-technology/isaac-lab/tree/f17384a1b487630128b4782ce02166565ef4464f/).

Once the repository is cloned, build and start the Docker container by running:

```shell
python3 docker/container.py start
```

Next, enter the running container using:

```shell
python3 docker/container.py enter
```

This ensures you are inside a development environment where Isaac Lab and all required dependencies are already
installed. Run the following command in the same terminal to verify the installation:

```shell
python3 scripts/custom/aica_bridge/run_bridge.py --scene basic_scene
```

This will spawn a UR5e robot, a ground plane, and lights. If you see the UR5e robot in the scene, then the installation
was successful and you are ready to proceed with the next steps.

If not, carefully go over the instructions again or reach out to AICA for help.

## Connecting AICA System to Isaac Lab

In this section, we’ll demonstrate a simple example of using a point attractor to move a simulated UR5e robot in Isaac
Lab toward a target frame, and then manipulate that frame within the 3D visualization of AICA Studio.

First, let’s review the key steps involved in this setup:

1. **Creating a New Scene in Isaac Lab**: Define a scene configuration class that inherits from `InteractiveSceneCfg`
   and register it in the `scenes` dictionary. This scene will include the robot model and any other objects you want to
   interact with.
2. **Running the Isaac Lab Simulator**: Launch the simulator with parameters that specify the scene, rate, force sensor,
   ports, joint names, command interface, and device.
3. **Configuring the AICA Application**: Set up a hardware interface in AICA Studio that uses the `LightWeightInterface`
   plugin to connect to the simulator.
4. **Running the AICA Application**: Start the AICA application to control the robot in the simulator.

### Creating a New Scene in Isaac Lab

To create a new scene, you should define a scene configuration class that inherits from `InteractiveSceneCfg`. Various
examples of scene config classes can be found in the
[scenes](https://github.com/aica-technology/isaac-lab/blob/f17384a1b487630128b4782ce02166565ef4464f/scripts/custom/aica_bridge/scenes)
directory of the Isaac Lab repository.

Note that the scene lives entirely in Isaac Lab and the definitions of the assets used in the scene should be defined
there. The 3D visualization in AICA Studio will only display the robot and mirror the robot's movements, but it will not
display the scene itself.

Once you've defined your scene configuration class, register it by adding a corresponding key to the `scenes` dictionary
located in
[this file](https://github.com/aica-technology/isaac-lab/blob/f17384a1b487630128b4782ce02166565ef4464f/scripts/custom/aica_bridge/scenes/__init__.py).
After registering the scene, you can launch it by running the following command in the `run_bridge.py` script:

```shell
python3 scripts/custom/aica_bridge/run_bridge.py --scene <your_scene_name>
```

In this example we will run the `basic_scene` scene, which is already registered in the `scenes` dictionary.

### Running the Isaac Lab Simulator

The simulator provides several key parameters that you should understand before configuring it:

- **scene**: Specifies which scene to load in the simulator.  
- **rate**: Sets the simulation update frequency in hertz (Hz). The default is **100 Hz**, but you can adjust this value based on your application’s requirements.  
- **force_sensor**: Defines the name of the force sensor specified in the URDF. If present, the simulator will attach this sensor to the end-effector link.  
- **ip_address**: Indicates the IP address of the machine running AICA Core. If the simulator and AICA Core are on the same network, keep the default `"*"`.  
- **state_port**: The port used to stream state updates from the simulator to the hardware interface in AICA Studio. The default is **1801**, and 
it must match the `state_port` specified in the hardware interface configuration.  
- **command_port**: The port used to stream commands from the AICA Studio hardware interface to the simulator. The default is **1802**, and 
it must match the `command_port` in the hardware interface configuration.  
- **force_port**: The port used to stream force/torque measurements from the simulator to the hardware interface in AICA Studio. The default 
is **1803**, and it must match the `ft_sensor_port` in the hardware interface configuration.  
- **joint_names**: Lists the joint names from the URDF that AICA will command. For example, if you are using a Franka Panda robot with a gripper 
but only want to control the arm, you can specify:  
  `"panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"`.
  The simulator will then only send states and accept commands for those joints.  If you want to control all joints, you can keep the default `"*"`.
- **command_interface**: Defines the command type accepted by the simulator. The default is `"positions"`, but you can set 
it to `"velocities"` or `"torques"` as needed. If a mismatched command type is received, the simulator will stop with a `ValueError`.  
- **headless**: When set to `true`, runs the simulator in headless mode (without a user interface), useful for remote simulations or running the simulation at high frequencies.  
- **device**: Specifies the compute device for the simulation. The default is `"cuda"` for GPU acceleration, but you can switch to `"cpu"` if GPU resources are unavailable.  

Ensure these parameters are correctly configured to enable seamless communication between the simulator and your AICA
application. In case you want to run the simulator with different parameters, you can do so by running the following
command in the `run_bridge.py` script:

```shell
python3 scripts/custom/aica_bridge/run_bridge.py \
  --scene <your_scene_name> \
  --rate <simulation_rate> \
  --force_sensor <force_sensor_name_in_urdf> \
  --state_port <state_port> \
  --command_port <command_port> \
  --force_port <force_port> \
  --joint_names <comma_separated_joint_names_to_control> \
  --command_interface <positions/velocities/torques> \
  --headless <true/false> \
  --device <cuda/cpu>
```

### Configuring the AICA Application

If you haven’t already, please follow the [Point Attractor Example](../core-components/point-attractor) to create an
AICA application that moves a robot’s end-effector using a point attractor. We’ll use that application as the foundation
for this guide. You will also need the latest Universal Robots collection so make sure to include that collection when
launching AICA Studio using AICA Launcher.

Once you have your AICA application ready, the next step is to configure the hardware interface in AICA Studio to
communicate with the Isaac Lab simulator. This involves creating a new hardware interface configuration that uses the
`LightWeightInterface` plugin to connect to the simulator.

1. In AICA Studio, go to the Hardware tab.
2. Click on the Universal Robot 5e using `Mock Interface` to open it and use "Save As" to create a copy with a new name
   and description. For example, you can name it `Universal Robots 5e (LightWeightInterface)` as this is the named used
   with the attached example (below).
3. Inspect the content of the copied robot description to find the `hardware` tag. Replace it with the following:

```xml
<hardware>
   <plugin>aica_core_interfaces/LightWeightInterface</plugin>
   <param name="ip">0.0.0.0</param>
   <param name="state_port">1801</param>
   <param name="command_port">1802</param>
   <param name="ft_sensor_port">1803</param>
   <param name="bind_state_port">False</param>
   <param name="bind_command_port">False</param>
   <param name="bind_ft_sensor_port">False</param>
</hardware>
```

4. Save your changes.

Here is a prepared URDF file for the UR5e robot with the `LightWeightInterface` hardware plugin. You can copy and paste
this into a new URDF file in AICA Studio:

<details>
  <summary>Prepared UR5e URDF using the `LightWeightInterface`</summary>

```xml
<?xml version="1.0"?>
<robot name="ur5e">
  <link name="world" />
  <link name="base_link" />
  <link name="base_link_inertia">
    <visual>
      <origin rpy="0 0 3.141592653589793" xyz="0 0 0" />
      <geometry>
        <mesh filename="package://ur_description/meshes/ur5e/visual/base.dae" />
      </geometry>
    </visual>
    <collision>
      <origin rpy="0 0 3.141592653589793" xyz="0 0 0" />
      <geometry>
        <mesh filename="package://ur_description/meshes/ur5e/collision/base.stl" />
      </geometry>
    </collision>
    <inertial>
      <mass value="4.0" />
      <origin rpy="0 0 0" xyz="0 0 0" />
      <inertia ixx="0.00443333156" ixy="0.0" ixz="0.0" iyy="0.00443333156" iyz="0.0" izz="0.0072" />
    </inertial>
  </link>
  <link name="shoulder_link">
    <visual>
      <origin rpy="0 0 3.141592653589793" xyz="0 0 0" />
      <geometry>
        <mesh filename="package://ur_description/meshes/ur5e/visual/shoulder.dae" />
      </geometry>
    </visual>
    <collision>
      <origin rpy="0 0 3.141592653589793" xyz="0 0 0" />
      <geometry>
        <mesh filename="package://ur_description/meshes/ur5e/collision/shoulder.stl" />
      </geometry>
    </collision>
    <inertial>
      <mass value="3.761" />
      <origin rpy="0 0 0" xyz="0 0 0" />
      <inertia ixx="0.01043677082529" ixy="0.0" ixz="0.0" iyy="0.01043677082529" iyz="0.0"
        izz="0.006769799999999999" />
    </inertial>
  </link>
  <link name="upper_arm_link">
    <visual>
      <origin rpy="1.5707963267948966 0 -1.5707963267948966" xyz="0 0 0.138" />
      <geometry>
        <mesh filename="package://ur_description/meshes/ur5e/visual/upperarm.dae" />
      </geometry>
    </visual>
    <collision>
      <origin rpy="1.5707963267948966 0 -1.5707963267948966" xyz="0 0 0.138" />
      <geometry>
        <mesh filename="package://ur_description/meshes/ur5e/collision/upperarm.stl" />
      </geometry>
    </collision>
    <inertial>
      <mass value="8.058" />
      <origin rpy="0 1.5707963267948966 0" xyz="-0.2125 0.0 0.138" />
      <inertia ixx="0.128541836083245" ixy="0.0" ixz="0.0" iyy="0.128541836083245" iyz="0.0"
        izz="0.014504399999999999" />
    </inertial>
  </link>
  <link name="forearm_link">
    <visual>
      <origin rpy="1.5707963267948966 0 -1.5707963267948966" xyz="0 0 0.007" />
      <geometry>
        <mesh filename="package://ur_description/meshes/ur5e/visual/forearm.dae" />
      </geometry>
    </visual>
    <collision>
      <origin rpy="1.5707963267948966 0 -1.5707963267948966" xyz="0 0 0.007" />
      <geometry>
        <mesh filename="package://ur_description/meshes/ur5e/collision/forearm.stl" />
      </geometry>
    </collision>
    <inertial>
      <mass value="2.846" />
      <origin rpy="0 1.5707963267948966 0" xyz="-0.1961 0.0 0.007" />
      <inertia ixx="0.03904256026963631" ixy="0.0" ixz="0.0" iyy="0.03904256026963631" iyz="0.0"
        izz="0.005122799999999999" />
    </inertial>
  </link>
  <link name="wrist_1_link">
    <visual>
      <origin rpy="1.5707963267948966 0 0" xyz="0 0 -0.127" />
      <geometry>
        <mesh filename="package://ur_description/meshes/ur5e/visual/wrist1.dae" />
      </geometry>
    </visual>
    <collision>
      <origin rpy="1.5707963267948966 0 0" xyz="0 0 -0.127" />
      <geometry>
        <mesh filename="package://ur_description/meshes/ur5e/collision/wrist1.stl" />
      </geometry>
    </collision>
    <inertial>
      <mass value="1.37" />
      <origin rpy="0 0 0" xyz="0 0 0" />
      <inertia ixx="0.0028769988492000002" ixy="0.0" ixz="0.0" iyy="0.0028769988492000002" iyz="0.0"
        izz="0.0024660000000000003" />
    </inertial>
  </link>
  <link name="wrist_2_link">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 -0.0997" />
      <geometry>
        <mesh filename="package://ur_description/meshes/ur5e/visual/wrist2.dae" />
      </geometry>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 -0.0997" />
      <geometry>
        <mesh filename="package://ur_description/meshes/ur5e/collision/wrist2.stl" />
      </geometry>
    </collision>
    <inertial>
      <mass value="1.3" />
      <origin rpy="0 0 0" xyz="0 0 0" />
      <inertia ixx="0.002729998908" ixy="0.0" ixz="0.0" iyy="0.002729998908" iyz="0.0" izz="0.00234" />
    </inertial>
  </link>
  <link name="wrist_3_link">
    <visual>
      <origin rpy="1.5707963267948966 0 0" xyz="0 0 -0.0989" />
      <geometry>
        <mesh filename="package://ur_description/meshes/ur5e/visual/wrist3.dae" />
      </geometry>
    </visual>
    <collision>
      <origin rpy="1.5707963267948966 0 0" xyz="0 0 -0.0989" />
      <geometry>
        <mesh filename="package://ur_description/meshes/ur5e/collision/wrist3.stl" />
      </geometry>
    </collision>
    <inertial>
      <mass value="0.365" />
      <origin rpy="0 0 0" xyz="0.0 0.0 -0.0229" />
      <inertia ixx="0.00019212345231725498" ixy="0.0" ixz="0.0" iyy="0.00019212345231725498"
        iyz="0.0" izz="0.000256640625" />
    </inertial>
  </link>
  <joint name="base_joint" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 0" />
    <parent link="world" />
    <child link="base_link" />
  </joint>
  <joint name="base_link-base_link_inertia" type="fixed">
    <parent link="base_link" />
    <child link="base_link_inertia" />
    <origin rpy="0 0 3.141592653589793" xyz="0 0 0" />
  </joint>
  <joint name="shoulder_pan_joint" type="revolute">
    <parent link="base_link_inertia" />
    <child link="shoulder_link" />
    <origin rpy="0 0 0" xyz="0 0 0.1625" />
    <axis xyz="0 0 1" />
    <limit effort="150.0" lower="-6.283185307179586" upper="6.283185307179586"
      velocity="3.141592653589793" />
    <dynamics damping="0" friction="0" />
  </joint>
  <joint name="shoulder_lift_joint" type="revolute">
    <parent link="shoulder_link" />
    <child link="upper_arm_link" />
    <origin rpy="1.570796327 0 0" xyz="0 0 0" />
    <axis xyz="0 0 1" />
    <limit effort="150.0" lower="-6.283185307179586" upper="6.283185307179586"
      velocity="3.141592653589793" />
    <dynamics damping="0" friction="0" />
  </joint>
  <joint name="elbow_joint" type="revolute">
    <parent link="upper_arm_link" />
    <child link="forearm_link" />
    <origin rpy="0 0 0" xyz="-0.425 0 0" />
    <axis xyz="0 0 1" />
    <limit effort="150.0" lower="-3.141592653589793" upper="3.141592653589793"
      velocity="3.141592653589793" />
    <dynamics damping="0" friction="0" />
  </joint>
  <joint name="wrist_1_joint" type="revolute">
    <parent link="forearm_link" />
    <child link="wrist_1_link" />
    <origin rpy="0 0 0" xyz="-0.3922 0 0.1333" />
    <axis xyz="0 0 1" />
    <limit effort="28.0" lower="-6.283185307179586" upper="6.283185307179586"
      velocity="3.141592653589793" />
    <dynamics damping="0" friction="0" />
  </joint>
  <joint name="wrist_2_joint" type="revolute">
    <parent link="wrist_1_link" />
    <child link="wrist_2_link" />
    <origin rpy="1.570796327 0 0" xyz="0 -0.0997 -2.044881182297852e-11" />
    <axis xyz="0 0 1" />
    <limit effort="28.0" lower="-6.283185307179586" upper="6.283185307179586"
      velocity="3.141592653589793" />
    <dynamics damping="0" friction="0" />
  </joint>
  <joint name="wrist_3_joint" type="revolute">
    <parent link="wrist_2_link" />
    <child link="wrist_3_link" />
    <origin rpy="1.570796326589793 3.141592653589793 3.141592653589793"
      xyz="0 0.0996 -2.042830148012698e-11" />
    <axis xyz="0 0 1" />
    <limit effort="28.0" lower="-6.283185307179586" upper="6.283185307179586"
      velocity="3.141592653589793" />
    <dynamics damping="0" friction="0" />
  </joint>
  <link name="base" />
  <joint name="base_link-base_fixed_joint" type="fixed">
    <origin rpy="0 0 3.141592653589793" xyz="0 0 0" />
    <parent link="base_link" />
    <child link="base" />
  </joint>
  <link name="flange" />
  <joint name="wrist_3-flange" type="fixed">
    <parent link="wrist_3_link" />
    <child link="flange" />
    <origin rpy="0 -1.5707963267948966 -1.5707963267948966" xyz="0 0 0" />
  </joint>
  <link name="tool0" />
  <joint name="flange-tool0" type="fixed">
    <origin rpy="1.5707963267948966 0 1.5707963267948966" xyz="0 0 0" />
    <parent link="flange" />
    <child link="tool0" />
  </joint>
  <ros2_control name="ur5e" type="system">
   <hardware>
      <plugin>aica_core_interfaces/LightWeightInterface</plugin>
      <param name="ip">0.0.0.0</param>
      <param name="state_port">1801</param>
      <param name="command_port">1802</param>
      <param name="ft_sensor_port">1803</param>
      <param name="bind_state_port">False</param>
      <param name="bind_command_port">False</param>
      <param name="bind_ft_sensor_port">False</param>
   </hardware>
    <joint name="shoulder_pan_joint">
      <command_interface name="position" />
      <command_interface name="velocity" />
      <state_interface name="position">
        <!-- initial position for the mock system and simulation -->
        <param name="initial_value">0.0</param>
      </state_interface>
      <state_interface name="velocity">
        <param name="initial_value">0.0</param>
      </state_interface>
      <state_interface name="effort">
        <param name="initial_value">0.0</param>
      </state_interface>
    </joint>
    <joint name="shoulder_lift_joint">
      <command_interface name="position" />
      <command_interface name="velocity" />
      <state_interface name="position">
        <!-- initial position for the mock system and simulation -->
        <param name="initial_value">-1.57</param>
      </state_interface>
      <state_interface name="velocity">
        <param name="initial_value">0.0</param>
      </state_interface>
      <state_interface name="effort">
        <param name="initial_value">0.0</param>
      </state_interface>
    </joint>
    <joint name="elbow_joint">
      <command_interface name="position" />
      <command_interface name="velocity" />
      <state_interface name="position">
        <!-- initial position for the mock system and simulation -->
        <param name="initial_value">0.0</param>
      </state_interface>
      <state_interface name="velocity">
        <param name="initial_value">0.0</param>
      </state_interface>
      <state_interface name="effort">
        <param name="initial_value">0.0</param>
      </state_interface>
    </joint>
    <joint name="wrist_1_joint">
      <command_interface name="position" />
      <command_interface name="velocity" />
      <state_interface name="position">
        <!-- initial position for the mock system and simulation -->
        <param name="initial_value">-1.57</param>
      </state_interface>
      <state_interface name="velocity">
        <param name="initial_value">0.0</param>
      </state_interface>
      <state_interface name="effort">
        <param name="initial_value">0.0</param>
      </state_interface>
    </joint>
    <joint name="wrist_2_joint">
      <command_interface name="position" />
      <command_interface name="velocity" />
      <state_interface name="position">
        <!-- initial position for the mock system and simulation -->
        <param name="initial_value">0.0</param>
      </state_interface>
      <state_interface name="velocity">
        <param name="initial_value">0.0</param>
      </state_interface>
      <state_interface name="effort">
        <param name="initial_value">0.0</param>
      </state_interface>
    </joint>
    <joint name="wrist_3_joint">
      <command_interface name="position" />
      <command_interface name="velocity" />
      <state_interface name="position">
        <!-- initial position for the mock system and simulation -->
        <param name="initial_value">0.0</param>
      </state_interface>
      <state_interface name="velocity">
        <param name="initial_value">0.0</param>
      </state_interface>
      <state_interface name="effort">
        <param name="initial_value">0.0</param>
      </state_interface>
    </joint>
    <sensor name="tcp_fts_sensor">
      <state_interface name="force.x" />
      <state_interface name="force.y" />
      <state_interface name="force.z" />
      <state_interface name="torque.x" />
      <state_interface name="torque.y" />
      <state_interface name="torque.z" />
    </sensor>
  </ros2_control>
</robot>
```

</details>

Additionally, here is the AICA Application that runs the point attractor example. You can copy and paste this into AICA
Studio:

<details>
  <summary>Point Attractor Application</summary>

```yaml
schema: 2-0-5
dependencies:
  core: v4.4.2
frames:
  command:
    reference_frame: world
    position:
      x: 0.505
      y: -0.021148
      z: 0.484999
    orientation:
      w: -0.000004
      x: 1
      y: 0
      z: 0
on_start:
  load:
    - component: frame_to_signal
    - hardware: hardware
components:
  frame_to_signal:
    component: aica_core_components::ros::TfToSignal
    display_name: Frame to Signal
    events:
      transitions:
        on_load:
          lifecycle:
            component: frame_to_signal
            transition: configure
          load:
            component: signal_point_attractor
        on_configure:
          lifecycle:
            component: frame_to_signal
            transition: activate
    parameters:
      frame:
        value: command
        type: string
    outputs:
      pose: /frame_to_signal/pose
  signal_point_attractor:
    component: aica_core_components::motion::SignalPointAttractor
    display_name: Signal Point Attractor
    events:
      transitions:
        on_load:
          lifecycle:
            component: signal_point_attractor
            transition: configure
        on_configure:
          lifecycle:
            component: signal_point_attractor
            transition: activate
    inputs:
      state: /hardware/robot_state_broadcaster/cartesian_state
      attractor: /frame_to_signal/pose
    outputs:
      twist: /signal_point_attractor/twist
hardware:
  hardware:
    display_name: Hardware Interface
    urdf: Universal Robots 5e (LightWeightInterface)
    rate: 100
    events:
      transitions:
        on_load:
          load:
            - controller: robot_state_broadcaster
              hardware: hardware
            - controller: ik_velocity_controller
              hardware: hardware
    controllers:
      robot_state_broadcaster:
        plugin: aica_core_controllers/RobotStateBroadcaster
        outputs:
          cartesian_state: /hardware/robot_state_broadcaster/cartesian_state
        events:
          transitions:
            on_load:
              switch_controllers:
                hardware: hardware
                activate: robot_state_broadcaster
        display_name: Robot State Broadcaster
      ik_velocity_controller:
        plugin: aica_core_controllers/velocity/IKVelocityController
        inputs:
          command: /signal_point_attractor/twist
        events:
          transitions:
            on_load:
              switch_controllers:
                hardware: hardware
                activate: ik_velocity_controller
graph:
  positions:
    components:
      frame_to_signal:
        x: 240
        y: 140
      signal_point_attractor:
        x: 740
        y: 360
    hardware:
      hardware:
        x: 1300
        y: 0
  edges:
    on_start_on_start_frame_to_signal_frame_to_signal:
      path:
        - x: 160
          y: 60
        - x: 160
          y: 200
    hardware_hardware_robot_state_broadcaster_cartesian_state_signal_point_attractor_state:
      path:
        - x: 1240
          y: 500
        - x: 1240
          y: 320
        - x: 720
          y: 320
        - x: 720
          y: 620
    signal_point_attractor_twist_hardware_hardware_ik_velocity_controller_command:
      path:
        - x: 1220
          y: 620
        - x: 1220
          y: 760
    frame_to_signal_on_load_signal_point_attractor_signal_point_attractor:
      path:
        - x: 680
          y: 320
        - x: 680
          y: 420
    frame_to_signal_pose_signal_point_attractor_attractor:
      path:
        - x: 680
          y: 400
        - x: 680
          y: 660
```

</details>

### Running the AICA Application

When the simulator is running, you can execute your AICA application by first choosing the UR5e URDF file that you just
created with the hardware plugin being the `LightWeightInterface`, and then starting the application by clicking the
**Play** button in the **AICA Studio**.

Here is a screenshot of the AICA application running with the Isaac Lab simulator:
<div class="text--center">
  <img src={application} alt="Point Attractor Example" />
</div>

## Beware

When running the AICA System and Isaac Lab simulator, there are several important points to keep in mind to ensure
safe and reliable performance:

1. **Robot joint names**: Ensure that the joint names in your URDF file match those expected by the USD file in Isaac
   Lab. In the current implementation, there are two sources of truth for joint names: the URDF file and USD file. If
   these names do not match, the simulator will not be able to send the states correctly to the AICA application.

2. **Hardware Interface Rate in the AICA Studio**: The hardware interface rate in the **AICA Studio** should be equal to
   the simulation rate set in Isaac Lab. This ensures that the AICA application can send commands to the simulator at a
   rate that matches the simulation updates, preventing command loss or delays.

3. **Simulation Rate**: The simulation rate in Isaac Lab should be set to a value that allows for smooth and realistic
   updates. Commands are updated at the simulation rate, so if the rate is too low, then the robot may not respond as
   expected.

4. **Force Sensor**: If you enable the force sensor in the hardware interface, ensure that the simulator is configured
   to provide force-torque data. This is done by setting the `force_sensor` parameter to the name of the force sensor
   present in the URDF file. If the force sensor is not configured correctly, the simulator will not send force-torque
   data, and the AICA application may not function as expected.

5. **Command Interface**: Ensure that the command interface in the simulator matches the type of commands being sent.

## Conclusion

By following the steps in this guide, you should be able to connect and run your AICA applications (e.g., those
developed in the **AICA Studio**) with a physics simulator such as Isaac Lab.
