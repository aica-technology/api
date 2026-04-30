---
sidebar_position: 14
title: Fiducial Markers
---


import stagDetectorExample from './assets/stag-detector-example.png'
import stagMarkerDetection from './assets/stag-marker-detection.webm'


# Fiducial Markers

Different types of fiducial markers are used in robotics to provide precise 3D pose estimation and identification for cameras, enabling or improving robotic calibration and object manipulation.

AICA's `core-vision` package gives you the choice between using two commonly used markers, the STag and Aruco. 


:::tip

Performing the [intrinsic calibration](./camera-calibration.md) of the camera improves the precision for fiducial marker detection and tracking.

:::


This guide provides an example of STag marker detection. Using the Aruco marker follows a very similar process.


## Using the STag detector

Launch AICA Studio with a configuration that contains the `core-vision` package and create a new application.

1. Remove the hardware interface that is included in new applications by default.
2. From the `Scene` menu, use the `Add Component` tab and look for the **Camera Streamer** and **STag Detector** components, either by searching
or by manually going under the `Core Vision Components` menu. Add both of them to the graph.
3. Next, connect both components to the start block. Moreover, connect the outputs of the Camera Streamer to the relevant inputs of the STag Detector.
4. Enable **auto-configure** and **auto-activate** on both components.
5. By selecting any of the components, you can find all the available component parameters in the right panel under Settings.
6. If an intrinsic camera calibration is performed prior to this, add the file path of the camera configuration file as a parameter to the **Camera Streamer** component. 

By this point, you should have something like the following:


<div class="text--center">
  <img src={stagDetectorExample} alt="CameraStreamer configuration alongside STagDetector component" />
</div>


:::info

The Camera Streamer parameters are explained in the [CameraStreamer component guide](./camera-streamer.md).

:::


Let's go through the parameters of the STag Detector component:

- **Rate**: The rate parameter doesn't affect the behavior of the component as the detection process
    occurs on reception of a new image.
- **Bundle file**: 
- **Marker selection**: The name(s) of the marker(s) that we intend to be recognized. If any of these markers enters the camera frame, the `is_any_selected_marker_detected` predicate is set to **True**. This name should always be prepended with `stage_`.
- **Marker size**: 
- **Library**: This is the ID number of the HD library utilized by STag markers. The allowed numbers are  `[11, 13, 15, 17, 19, 21, 23]`.
- **Error correction**: 
- **Prefix**: This prefix is used for marker names.


:::tip

If a decision needs to be made based on the existance of a specific STag marker in the camera frame, its name should be indicated in the `Marker Selection` variable.

:::


:::info

If any marker is detected in the camera frame at all, the `is_any_marker_detected` predicate is set to **True**. 

:::

After setting up the proper parameters for Camera Streamer and STag Detector:

1. Press **Start** to start the application.
2. To see the live camera feed, select **Launch RViz** from the Launcher settings
3. In RViz, select _Add > By topic > /stag_detector/annotated_image > Image_. This adds a panel that shows the live image. The marker should be detected in the camera.


<div style={{ display: "flex", justifyContent: "center" }}>
  <video autoPlay loop muted playsInline style={{ maxWidth: "100%", borderRadius: "8px" }}>
    <source src={stagMarkerDetection} type="video/webm" />
    STag marker detection video.
  </video>
</div>


:::info

The process for using Aruco markers follows a similar process.

:::