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

the parameters of the Robot Camera Calibration component are consisted of:

- **Rate**: Rate determines the speed of transformation acquisition. This parameter has no effect on the behaviour of the component.
- **Camera frame**: The name of the camera frame being used in the application. It can be looked up from RViz list of frames.
- **Marker frame**: The name of the marker being detected by the camera. It is indicated in the STag or Aruco marker deterctor component used in the application.
- **Robot base frame**: The name of your robots base frame, whichc can be looked up in RViz.
- **Robot end-effector frame**: The name of the robots end-effector frame. It can be found in the RViz list of frames.
- **Number of recorded points**: Number of `camera` to `robot end-effector` transformations calculated based on various sets of `robot end-effector` to `robot base` and `camera to marker` transformations.
- **Distance between points**: This is the minimum difference in terms of distance between two consequetive transformations to be recorded.
- **Epsilon time**:
- **Calibration folder path**: This is the directory in which the final calibration file will be stored.
- **Calibration file**: This is the name of the file produced after the calibration, containing the calibration information.
- **Points dataset file**: The file containing transformations of robot end-effector or marker, which might exist from the past.

## Robot Calibration using AICA Studio and a marker

After completing the camera calibration as described in the [`Camera Calibration` example](./camera-calibration.md), and verifying marker detection as outlined in the [`Marker detection`](./marker-detection.md) section, you can proceed with the hand–eye calibration process.

This example demonstrates the eye-in-hand configuration (camera mounted on the robot arm). The procedure for the eye-to-hand configuration (static camera) follows a similar workflow.

- Ensure that the camera is properly configured and operational.
- Connect all required components and controllers in the AICA application. Refer to the system setup illustration below for guidance.
- Place the marker within the robot workspace, ensuring it is fully visible to the camera. To monitor the live camera feed, enable **Launch RViz** from the Launcher settings, as described in the [`marker detection`](./marker-detection.md) guide.
- Run the program and move the robot TCP (Tool Center Point) to capture images of the marker from multiple perspectives. The application starts capturing images automatically.
  Ensure sufficient variation in position and orientation to improve calibration accuracy.
- Once 100 unique images have been captured, the system automatically generates a calibration file in YAML format in the following directory:

```bash
/tmp/calibration/camera_calibration.yaml
```

<div class="text--center">
  <img src={RobotCalibrationConfiguration} alt="The configuration required for hand-eye calibration" style={{ borderRadius: "8px" }}/>
</div>

::: info

The robot TCP can be moved in different ways. Using teach pendant (jogging or Freedrive mode), or using AICA Studio's Hand Guiding Controller as described in [`Hand Guiding controller`](./ur-harware-interface.md)

:::

::: tip

The Camera and the marker detector components might differ based on the type of hardwares being used.

:::

There are several ways to use the transformation information obtained in the calibration file:

1. Adding the camera link to the URDF.
2. Publish the transformation with a FrameBroadcaster manually.
3. Using the calibration file path directly as a `Frame Broadcaster` component parameter to publish the transformations.
