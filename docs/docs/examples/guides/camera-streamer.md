---
sidebar_position: 8
title: CameraStreamer component
---

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

import cameraStreamerExample from './assets/camera-streamer-example.png'

# CameraStreamer camera component

AICA's `core-vision` package includes, among others, the `CameraStreamer` component that is responsible for reading
image streams from USB devices, network streams, or video files. Most of the standard consumer-grade webcams and video
formats are supported out-of-the-box.

This guide describes shows the basics on how to use the component with a camera and a video file, two very common
choices.

## AICA Launcher configuration

Start the AICA Launcher and add the `core-vision` package to your configuration at version v1.1.0. Mind that some
`CameraStreamer` options are missing from v1.0.0 of `core-vision`.

For a 1-to-1 experience use **AICA Core v5.0.0 or higher**, but note that `CameraStreamer` is compatible with Core
starting v4.4.2.

Select **Launch AICA Studio** to proceed.

:::note
It is recommended that you use AICA Launcher v1.4.1 or higher
:::

## Using CameraStreamer

Start by creating a new application.

1. Remove the hardware interface that is included in new applications by default.
2. From the `Scene` menu, use the `Add Component` tab and look for the **Camera Streamer** component either by searching
or by manually going under the `Core Vision Components` menu. Click to add it to the graph.
3. Next, connect the component to the start block.
4. Enable **auto-configure** and **auto-activate**
5. With your new component selected, make sure you are in the `Scene` menu, where you may find all the available
component parameters.

By this point, you should have something like the following:

<div class="text--center">
  <img src={cameraStreamerExample} alt="Default CameraStreamer configuration" />
</div>

Before pressing play, let us go through the parameters first. You should see:

- **Rate**: This is the component's rate, but it has no effect on the operation of `CameraStreamer`
- **Source**: Path to the source device or video file
- **Camera frame**: The reference frame that will be used when publishing image messages, which should correspond the
camera's sensor position
- **Camera configuration**: A YAML-formatted camera configuration file containing the camera intrinsics (optional)
- **Frame width**: The desired image width
- **Frame height**: The desired image height
- **Frame rate**: The desired frame rate for image streaming
- **Crop undistorted image**: If a camera configuration file is available, this option indicates whether black-pixel
regions created at the edges of the image post-undistortion should be cropped. Mind that this also crops part of the
image that contains usable pixels.
- **Stream encoding**: Sets the output encoding of the image stream

:::info

The **image width and height** are not used to resize an existing stream. In the case of camera devices, they will act
as desired dimensions, but the component will fail to configure if they are not supported. For videos, the native
resolution should be set.

Similarly, the **frame rate** is also a request and will cause the component to fail configuration if the value
requested is not supported.

:::

Once you have selected an appropriate **source**:

6. Press **Play** to start the application.
7. To see the live camera feed, click on the gear icon on the bottom right and select **Launch RViz**.
8. In RViz, select _Add > By topic > /camera_streamer/image > Image_. This adds a panel that shows the live image. The undistorted image (if available) can also be found under _/camera_streamer/undistorted_image > Image_.
