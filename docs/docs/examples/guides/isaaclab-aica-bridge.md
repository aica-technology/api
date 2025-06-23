---
sidebar_position: 6
title: A guide for using Isaac Lab as a simulator for AICA System
---

import application from './assets/isaaclab-aica-bridge-application.png'

# A guide for using Isaac Lab as a simulator for AICA System

## Motivation

By interfacing the AICA System with Isaac Lab, we establish a workflow for developing, testing, and deploying robotic
applications. This connection provides several key benefits:

1. **RL Policy Testing**: AICA’s RL Policy Component SDK allows developers to deploy Reinforcement Learning (RL) models
   directly onto real hardware through components. These models can be trained in Isaac Lab, and with the AICA System
   interacting directly with Isaac Lab, users can validate trained policies under the same conditions in which they
   were learned.

2. **Reliable Policy Validation**: Developers can monitor the behavior of the trained policies and test the effect of
   various parameters, enabling confident transitions from simulation to real-world deployment.

3. **Digital Twin Control**: Beyond RL, running the AICA System with Isaac Lab provides users with ways to interact with
   digital twins of their robots. Applications can be authored, tested, and validated entirely in simulation before
   connecting to actual hardware. This improves safety and enables rapid iteration in early stages, helping streamline
   the overall development cycle.

With Isaac Lab as simulator, users can build scenes in Isaac Lab, command simulated robots using AICA
System, validate performance, switch the hardware interface to a real robot, and hit play with no code changes required.

## Installing Isaac Lab

Begin by cloning AICA's fork of [Isaac Lab](https://github.com/aica-technology/isaac-lab/tree/f17384a1b487630128b4782ce02166565ef4464f/).

Once the repository is cloned, build and start the Docker container by running:

```shell
python3 docker/container.py start
```

Next, enter the running container using:

```shell
python3 docker/container.py enter
```

This ensures you are inside a development environment where Isaac Lab and all required dependencies are already
installed.

Run the following command in the same terminal to verify the installation:

```shell
python3 scripts/custom/aica_bridge/run_bridge.py --scene basic_scene
```

This will spawn a UR5e robot, a ground plane, and lights. If you see the UR5e robot in the scene, then the installation
was successful and you are ready to proceed with the next steps.

If not, carefully go over the instructions again or reach out to AICA for help.

## Connecting AICA System to Isaac Lab

In this section, we’ll walk through a simple example of using a point attractor to move a UR5e simulated robot in Isaac
Lab to a target frame and then manipulate that frame in the 3D visualization of the AICA Studio.

### Creating a New Scene in Isaac Lab

To create a new scene, you should define a scene configuration class that inherits from `InteractiveSceneCfg`. Various examples
of scene config classes can be found in the [scenes](https://github.com/aica-technology/isaac-lab/blob/f17384a1b487630128b4782ce02166565ef4464f/scripts/custom/aica_bridge/scenes) directory of the Isaac Lab
repository.

Note that the scene lives entirely in Isaac Lab and the definitions of the assets used in the scene should be defined there. The
3D visualization in the AICA Studio will only display the robot and mirror the robot's movements, but it will not display
the scene itself.

Once you've defined your scene configuration class, register it by adding a corresponding key to the `scenes` dictionary
located in [this file](https://github.com/aica-technology/isaac-lab/blob/f17384a1b487630128b4782ce02166565ef4464f/scripts/custom/aica_bridge/scenes/__init__.py).
After registering the scene, you can launch it by running the following command in the `run_bridge.py` script:

```shell
python3 scripts/custom/aica_bridge/run_bridge.py --scene <your_scene_name>
```

In this example we will run the `basic_scene` scene, which is already registered in the `scenes` dictionary.

### Running the Isaac Lab Simulator

The simulator includes several important parameters that you should be familiar with:

- **scene**: Specifies the scene to load in the simulator.
- **rate**: Sets the simulation update rate in Hz. The default is 100 Hz, but you can adjust this to meet your
  application's needs.
- **end_effector**: Indicates the name of the end-effector link on the robot being controlled.
- **force_sensor**: Enables the robot's force sensor when set to `true`. This is useful for applications that require
  force feedback.
- **state_port**: Defines the port used to stream state updates from the simulator to the hardware interface in the AICA Studio.
The default is `1801`, and it must match the `state_port` parameter specified in the hardware interface configuration (see below).
- **command_port**: Defines the port used to stream commands from hardware interface in the AICA Studio to the simulator.
The default is `1802`, and it must match the `command_port` parameter specified in the hardware interface configuration (see below).
- **force_port**: Defines the port used to stream force/torque measurements from the simulator to the hardware interface in the AICA Studio
The default is `1803`, and it must match the `ft_sensor_port` parameter specified in the hardware interface configuration (see below).
- **command_interface**: Specifies the type of command the simulator accepts. The default is `"position"`, meaning it
  expects position commands. You can change this to `"velocity"` if required. If a mismatched command type is received,
  the simulator will terminate with a `ValueError` indicating the mismatch.
- **headless**: When set to `true`, the simulator runs in headless mode, which is useful for running simulations without
  a graphical interface.
- **device**: Specifies the device to use for simulation. The default is `cuda`, which utilizes GPU acceleration. You
  can change this to `cpu` if you want to run the simulation on the CPU instead.

Ensure these parameters are correctly configured to enable seamless communication between the simulator and your AICA
application. In case you want to run the simulator with different parameters, you can do so by running the following
command in the `run_bridge.py` script:

```shell
python3 scripts/custom/aica_bridge/run_bridge.py --scene <your_scene_name> --rate <simulation_rate> --end_effector <end_effector_link_name> --force_sensor <true/false> --state_port <state_port> --command_port <command_port> --force_port <force_port> --command_interface <position/velocity> --headless <true/false> --device <cuda/cpu>
```

### Configuring the Hardware Interface

In AICA Studio, you need to set up a hardware interface that communicates with the Isaac Lab simulator:

1. Open AICA Launcher and launch a configuration using the latest core image along with the latest Universal Robots
   collection.
2. In AICA Studio, go to the Hardware tab.
3. Click on the Universal Robot 5e entry to open it and use "Save As" to create a copy with a new name and description.
4. Inspect the content of the copied robot description to find the `hardware` tag. Replace it with the following:

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

5. Save your changes.

### Creating an AICA Application

1. Go back to the Editor tab and create a new application.
2. In the default hardware interface, open the URDF drop down and select the hardware created in the previous section.
3. Add the following components to the application:

   - **Signal Point Attractor**: Drives the robot’s movement toward a specified target frame.
   - **Frame to Signal**: Converts the target frame into a signal that feeds into the point attractor.
   - **IK Velocity Controller**: Calculates the inverse kinematics for the UR5e, enabling smooth motion toward the
     target frame.

4. Connect the events and signals as shown in the image. Make sure to auto-configure and auto-activate all components
   and controllers.

<div class="text--center">
  <img src={application} alt="Signal Point Attractor Application" />
</div>

5. Press **Play**, then switch to the **3D Visualizer** tab in the **AICA Studio**. You should see the UR5e robot in the
   scene. Use the **Record Frame** button to capture a target frame at the robot's tool, specify the tool frame name
   (for UR robots from the collection, that is `ur_tool0`), and then move the frame to your desired target position.
6. Once the target frame is set, parametrize the **Frame to Signal** component with the name of the target frame you
   just created.

Now that you have set up the application, let's go over the necessary steps that needs to be done on the Hardware
Interface level to connect **AICA System** to the Isaac Lab simulator.


### Running the AICA Application

When the simulator is running, you can execute your AICA application by first chosing the UR5e URDF file that you just
created with the hardware plugin being the `LightWeightInterface`, and then starting the applicationby clicking the
**Play** button in the **AICA Studio**.

## Beware

When running the **AICA System** and Isaac Lab simulator, there are several important points to keep in mind to ensure
safe and reliable performance:

1. **Robot joint names**: Ensure that the joint names in your URDF file match those expected by the USD file in Isaac
   Lab. In the current implementation, there are two sources of truth for joint names: the URDF file and USD file. If
   these names do not match, the simulator will not be able to send the states correctly to the AICA application.

2. **Hardware Interface Rate in the AICA Studio**: The hardware interface rate in the **AICA Studio** should be greater than the
   simulation rate in Isaac Lab. This ensures that the AICA application can send commands to the simulator at a rate
   that matches or exceeds the simulation updates, preventing command loss or delays.

3. **Simulation Rate**: The simulation rate in Isaac Lab should be set to a value that allows for smooth and realistic
   updates. Commands are updated at the simulation rate, so if the rate is too low, then the robot may skip commands or
   not respond as expected.

4. **Force Sensor**: If you enable the force sensor in the hardware interface, ensure that the simulator is configured
   to provide force-torque data. This is done by setting the `force_sensor` parameter to the name of the force sensor
   present in the URDF file. If the force sensor is not configured correctly, the simulator will not send force-torque
   data, and the AICA application may not function as expected.

5. **Command Interface**: Ensure that the command interface in the simulator matches the type of commands being sent.

## Conclusion

By following the steps in this guide, you should be able to connect and run your AICA applications (e.g., those developed in the **AICA Studio**) with a physics simulator such as Isaac Lab, leveraging it as a digital twin environment.
