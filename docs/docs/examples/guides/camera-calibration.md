---
sidebar_position: 9
title: Camera calibration walkthrough
---

import cameraCalibration from './assets/camera-calibration.gif'

# Camera calibration

Accurate camera calibration is essential for ensuring that visual detections and measurements correspond correctly to
real-world geometry. In many perception pipelines, critical output is expressed in image pixel coordinates. However, if
your camera's lens introduces distortion or if the intrinsic parameters are unknown, these pixel coordinates can deviate
significantly from their true positions.

A calibration procedure allows you to determine your camera's intrinsic (focal length, optical center) and extrinsic
(position, orientation) parameters, as well as its distortion coefficients. Once known, these parameters can be used to
undistort images in real time, improving the spatial accuracy of detections and any downstream estimation tasks (e.g.,
pose reconstruction, depth reasoning, or robot alignment).

## AICA helpers for calibrating cameras

In a typical ROS scenario, you may have a camera whose image is published as a `sensor_msgs::msg::Image` signal or a USB
camera that you can use with the [`CameraStreamer` component](./camera-streamer.md) to create said signal. For the
following guide, we assume, and it is required, that you have a source that publishes the image stream as a
`sensor_msgs::msg::Image`.

Once a `sensor_msgs::msg::Image` signal is established for your camera, clone our docker image repository:

```shell
https://github.com/aica-technology/docker-images.git
```

and open a terminal at the root of `docker-images`. Then:

```shell
cd camera_calibration
```

Within that folder you will find a `build-run.sh` script that, as the name suggests, will build a Docker image and run a
container with the camera calibration software as an entrypoint.

Before running the script, make sure to generate a **checkerboard** pattern (e.g., from
[here](https://calib.io/pages/camera-calibration-pattern-generator)). The calibrator will use this pattern to determine
how the picture is distorted and ultimately generate the necessary matrices that can be used to undistort an image
coming from your camera. Take note of the checkerboard width, height, and box size. Notice that the calibrator is
detecting the internal corners of the outermost boxes, so a 8x11 checkerboard will have a 7x10 area with which the
calibrator will work. Print the checkerboard and attach it to a flat surface throughout the calibration process.
Then, run the `build-run.sh` script specifying the necessary parameters similarly to:

```shell
./build-run.sh --calibration-height 7 --calibration-width 11 --calibration-square 0.015
```

Notice that the calibration square side size is in meters. If you are using AICA's `CameraStreamer` component to produce
an image stream, the above command should already work. If you are using your own node to stream images, you will likely
need to specify which topic the calibrator needs to subscribe to by adding the `--calibration-topic YOUR_ROS_TOPIC`
argument.

After successfully executing the script, a pop-up window displaying your image stream should appear. If the window is
not displaying the image stream, make sure that your image streaming component is running and, for non-`CameraStreamer`
nodes, that the correct topic name is set.

Move the checkerboard in various positions and orientations until the `CALIBRATE` button is no longer grayed out (and
most of the bars are green, indicating good sample size). Once it becomes available, press on it to start computing the
camera matrices. After it becomes available, click on the `SAVE` button to save a recording of the process. You
will notice a `calibration` directory has been created on your host machine under `docker-image/camera_calibration` that
contains a compressed file. The file itself contains the images that were sampled along with a yaml file containing the
camera calibration information. Finally, move this file into the `data` folder of your AICA configuration such that it
becomes available within AICA containers. When using `CameraStreamer`, you only need to specify the calibration's path.
For custom components, make sure to read the camera parameters and apply the necessary undistortion technique(s).

<div class="text--center">
  <img src={cameraCalibration} alt="Camera calibration process" />
</div>
