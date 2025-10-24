---
sidebar_position: 12
title: Camera calibration
---

import cameraCalibration from './assets/camera-calibration.gif'

:::info

This example only works on Linux hosts as it requires graphics forwarding from Docker to your host machine.

:::

# Camera calibration

Accurate camera calibration is essential for ensuring that visual detections and measurements correspond correctly to
real-world geometry. In many perception pipelines, critical output is expressed in image pixel coordinates. However, if
your camera's lens introduces distortion or if the intrinsic parameters are unknown, these pixel coordinates can deviate
significantly from their true position.

A calibration procedure allows you to determine your camera's intrinsic (focal length, optical center) and extrinsic
(position, orientation) parameters, as well as its distortion coefficients. Once known, these parameters can be used to
undistort images in real time, improving the spatial accuracy of detections and any downstream estimation tasks (e.g.,
pose reconstruction, depth reasoning, or robot alignment).

## Calibrate camera using AICA Studio and a checkerboard

If you are using the `CameraStreamer` with a camera with an unknown calibration, we recommend to follow this guide with
the code from the corresponding [`CameraStreamer` example](./camera-streamer.md). Cameras with first-party drivers (such
as [RealSense](./realsense-component.md) and [Orbbec](./orbbec-component.md)) usually provide the intrinsic parameters
and distortion coefficients and don't need to be calibrated.

:::note

Instead of using `CameraStreamer`, advanced users can follow this process with any ROS node publishing
`sensor_msgs::msg::Image` messages from a camera.

:::

Once you have a established an image stream, make sure to generate a **checkerboard** pattern (e.g., from
[here](https://calib.io/pages/camera-calibration-pattern-generator)). The calibrator will use this pattern to determine
how the picture is distorted and ultimately generate the necessary matrices that can be used to undistort images from
your camera. Take note of the checkerboard width, height, and box size as you will need it later. Print the checkerboard
and attach it to a flat surface throughout the calibration process.

:::tip

Notice that the calibrator is detecting the internal corners of the outermost boxes, so a 8x11 checkerboard will have a
7x10 area with which the calibrator will work.

:::

Clone our docker image repository in a directory of your choice:

```shell
git clone https://github.com/aica-technology/docker-images.git && cd docker-images
```

then:

```shell
cd camera_calibration
```

Within that folder you will find a `build-run.sh` script that, as the name suggests, will build a Docker image and run a
container with the camera calibration software as an entrypoint.


Then, run the `build-run.sh` script specifying the necessary parameters similarly to:

```shell
./build-run.sh --calibration-height 7 --calibration-width 10 --calibration-square 0.015
```

where the calibration square sizes are in meters.

:::warning

If you are using AICA's `CameraStreamer` example to produce the image stream, the above command should already work.
If you are using your own node to stream images, you will likely need to specify which topic the calibrator needs to
subscribe to by adding the `--calibration-topic YOUR_ROS_TOPIC` argument to the command above.

:::

After successfully executing the script, a pop-up window displaying your image stream should appear. If the window is
not displaying the image stream, make sure you followed the above steps correctly.

When you can see a live feed of your camera:

- Move the checkerboard in various positions and orientations until the `CALIBRATE` button is no longer grayed out (and
most of the bars are green, indicating good sample size).
- Once `CALIBRATE` becomes available, press on it to start computing the camera matrices.
- After it becomes available, click on the `SAVE` button to save a recording of the process.

Back at your host computer's filesystem, you will notice a `calibration` directory has been created on under
`docker-image/camera_calibration` that contains a compressed file. The file itself contains the images that were sampled
along with a YAML file containing the camera calibration information.

Finally, move the YAML file into the `data` folder of your AICA configuration such that it becomes available from AICA
Studio.

<div class="text--center">
  <img src={cameraCalibration} alt="Camera calibration process" />
</div>
