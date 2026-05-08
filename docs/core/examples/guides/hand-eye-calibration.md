---
sidebar_position: 15
title: Hand-Eye calibration
---

// Should the name of the document change to "robot calibration" ?

import RobotCalibrationConfiguration from './assets/robot-calibration-configuration.png'
import RobotCameraCalibration from './assets/robot-camera-calibration-component.png'

# Hand-Eye calibration

Robot calibration is a fundamental prerequisite for any robotic system that relies on precise coordination between a vision sensor and a manipulator. It establishes the spatial transformation between the robot’s end-effector and the camera frame, enabling accurate mapping between observed features and actionable robot coordinates. Without proper calibration, even high-quality perception or motion planning algorithms can yield significant positioning errors.

The AICA's `core-vision` package provides a structured workflow for performing hand–eye calibration efficiently and reproducibly.

Accurate hand–eye calibration is critical in tasks such as visual servoing, object manipulation, inspection, and assembly. This tool is designed to minimize discrepancies and provide reliable calibration outputs suitable for industrial environments.

## Robot Camera Calibration component

The `Robot Camera Calibration` component is the component that does the main job of calculating the transformation between the `camera` and the `robot end-effector`.

click on the `Robot Camera Calibration` component block to view and edit the available parameters.

<div class="text--center">
  <img src={RobotCameraCalibration} alt="Robot Camera Calibration parameters" style={{ borderRadius: "8px" }}/>
</div>

The parameters of the `Robot Camera Calibration` component are defined as follows:

- **Rate**: Determines the frequency at which transformations are acquired. This parameter does not affect the component’s behavior.
- **Bundle file:** The filepath to a predefined marker bundle configuration. This additional feature is described in a separate guide (coming soon).
- **Camera frame**: The name of the camera frame used in the application. This can be retrieved from the list of frames in RViz.
- **Marker frame**: The name of the marker detected by the camera. It is indicated in the STag or Aruco marker detector component used in the application.
- **Robot base frame**: The name of the robot's base frame, which can be found in RViz.
- **Robot end-effector frame**: The name of the robot’s end-effector frame. This is also available in the RViz frame list.
- **Number of recorded points**: The number of transformations from `camera` to `robot end-effector` camera to robot end-effector that are recorded. These are computed from multiple pairs of `robot end-effector` -> `robot base` and `camera` -> `marker` transformations.
- **Distance between points**: The minimum spatial difference required between two consecutive transformations for them to be recorded.
- **Epsilon time**:
- **Calibration folder path**: The directory where the final calibration file will be stored.
- **Calibration file**: The name of the output file generated after calibration, containing the computed calibration data.
- **Points dataset file**: A file containing previously recorded transformations of the robot end-effector or marker, which can be reused if available.

```info
Set the `Is camera attached` check to true, if a camera is attached to the robot end-effector frame.
```

## Robot Calibration using AICA Studio and a marker

After completing the camera calibration as described in the [`Camera Calibration` example](./camera-calibration.md), and verifying marker detection as outlined in the [`Marker detection`](./marker-detection.md) section, you can proceed with the hand–eye calibration process.

This example demonstrates the eye-in-hand configuration (camera mounted on the robot arm). The procedure for the eye-to-hand configuration (static camera) follows a similar workflow.

- Ensure that the camera is properly configured and operational.
- Connect all required components and controllers in the AICA application. Refer to the system setup illustration below for guidance.
- Place the marker within the robot workspace, ensuring it is fully visible to the camera. To monitor the live camera feed, enable **Launch RViz** from the Launcher settings, as described in the [`marker detection`](./marker-detection.md) guide.
- Run the program and move the robot TCP (Tool Center Point) to capture images of the marker from multiple perspectives. The application starts capturing images automatically.
  Ensure sufficient variation in position and orientation to improve calibration accuracy.
- Once the number of captured images reaches the `Number of recorded points` indicated in the `Robot Camera Calibration component` parameters, the system automatically generates a calibration file in YAML format in the following directory:

```bash
/tmp/calibration/camera_calibration.yaml
```

The following YAML snippet containing the full application:

<div class="text--center">
  <img src={RobotCalibrationConfiguration} alt="The configuration required for hand-eye calibration" style={{ borderRadius: "8px" }}/>
</div>

<details>
  <summary>Example application, Hand-Eye calibration</summary>

```yaml
schema: 2-0-6
dependencies:
  core: v5.1.0
on_start:
  load:
    - component: orbbec_camera
    - component: robot_camera_calibration
    - component: stag_detector
    - hardware: hardware
components:
  orbbec_camera:
    component: orbbec_camera::OBCameraNodeDriver
    display_name: Orbbec Camera
    outputs:
      color_image: /orbbec_camera/color_image
      color_camera_info: /orbbec_camera/color_camera_info
  robot_camera_calibration:
    component: core_vision_components::calibration::RobotCameraCalibration
    display_name: Robot Camera Calibration
    events:
      transitions:
        on_load:
          lifecycle:
            component: robot_camera_calibration
            transition: configure
        on_configure:
          lifecycle:
            component: robot_camera_calibration
            transition: activate
    parameters:
      camera_frame:
        value: orbbec_camera_link
        type: string
      marker_frame:
        value: stag_0
        type: string
      robot_base_frame:
        value: world
        type: string
      robot_ee_frame:
        value: ur_tool0
        type: string
      is_camera_attached:
        value: true
        type: bool
  stag_detector:
    component: core_vision_components::pose_detection::STagDetector
    display_name: STag Detector
    events:
      transitions:
        on_load:
          lifecycle:
            component: stag_detector
            transition: configure
        on_configure:
          lifecycle:
            component: stag_detector
            transition: activate
    parameters:
      marker_selection:
        value:
          - stag_0
        type: string_array
    inputs:
      image: /orbbec_camera/color_image
      camera_info: /orbbec_camera/color_camera_info
hardware:
  hardware:
    display_name: Hardware Interface
    urdf: Universal Robots 5e
    rate: 500
    events:
      transitions:
        on_load:
          load:
            - controller: robot_state_broadcaster
              hardware: hardware
            - controller: ur_hand_guiding_controller
              hardware: hardware
    parameters:
      robot_ip: 192.168.42.20
    controllers:
      robot_state_broadcaster:
        plugin: aica_core_controllers/RobotStateBroadcaster
        events:
          transitions:
            on_load:
              switch_controllers:
                hardware: hardware
                activate: robot_state_broadcaster
      ur_hand_guiding_controller:
        plugin: aica_ur_controllers/URHandGuidingController
        parameters:
          ft_sensor_name:
            value: ur_tcp_fts_sensor
            type: string
          ft_sensor_reference_frame:
            value: ur_tool0
            type: string
          force_limit:
            value:
              - 20
              - 20
              - 20
              - 2
              - 2
              - 2
            type: vector
        events:
          transitions:
            on_load:
              switch_controllers:
                hardware: hardware
                activate: ur_hand_guiding_controller
graph:
  positions:
    on_start:
      x: 120
      y: 0
    stop:
      x: 120
      y: 100
    components:
      orbbec_camera:
        x: 340
        y: 0
      robot_camera_calibration:
        x: 340
        y: 320
      stag_detector:
        x: 800
        y: -80
    hardware:
      hardware:
        x: 1260
        y: -120
  edges:
    on_start_on_start_robot_camera_calibration_robot_camera_calibration:
      path:
        - x: 260
          y: 60
        - x: 260
          y: 380
    on_start_on_start_stag_detector_stag_detector:
      path:
        - x: 260
          y: 60
        - x: 260
          y: -20
    on_start_on_start_hardware_hardware:
      path:
        - x: 260
          y: 60
        - x: 260
          y: -60
```

</details>

:::tip
Don't forget to modify the `robot_ip` according to the ip of the robot that you are using.
:::

::: info

The robot TCP can be moved by jogging or Freedrive mode using the robot's teach pendant. In the case of using a Universal Robot, AICA Studio offers the option of `Hand Guiding Controller` which facilitates and accelerates the process. This controller is described in the [`Hand Guiding Controller`](./ur-harware-interface.md) page.

:::

::: tip

The Camera and the marker detector components might differ based on the type of hardware being used.

:::

There are several ways to use the transformation information obtained in the calibration file:

1. Adding the camera link to the URDF.
2. Publish the transformation with a FrameBroadcaster manually.
3. Using the calibration file path directly as a `Frame Broadcaster` component parameter to publish the transformations.
