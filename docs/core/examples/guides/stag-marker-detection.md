---
sidebar_position: 13
title: STag marker detection
---


import stagDetectorExample from './assets/stag-detector-example.png'
import stagMarkerDetection from './assets/stag-marker-detection.webm'


# STag marker detection

As a step in the Hand-Eye calibration, we need to be able to detect markers with our camera. The AICA's `core-vision` package gives you the choice between using the STag or Aruco markers. 


:::TIP

It's important to notice that this should be done after the intrinsic calibration of the camera which is explained in the [calibration guide](./camera-calibration.md).

:::

## Using the STag detector

Launch AICA Studio with a configuration that contains the `core-vision` package and create a new application.

1. Remove the hardware interface that is included in new applications by default.
2. From the `Scene` menu, use the `Add Component` tab and look for the **Camera Streamer** and **STag Detector** components, either by searching
or by manually going under the `Core Vision Components` menu. Add both of them to the graph.
3. Next, connect both components to the start block. Morover, connect the outputs of the Camera Streamer to the relevant inputs of the STag Detector.
4. Enable **auto-configure** and **auto-activate** on both components.
5. By chosing any of the components, you can find all the available component parameters.

By this point, you should have something like the following:


<div class="text--center">
  <img src={stagDetectorExample} alt="CameraStreamer configuration with STagDetector component" />
</div>


:::info

The Camera Streamer parameters are explained in the [CameraStreamer component guid](./camera-streamer.md).

:::


Let's go through the parameters of the STag Detector component:

- **Rate**: The frequency of the callbacks. 
- **Bundle file**: 
- **Marker selection**: The name(s) of the marker(s) that we intend to be recognized. If any of these markers enters the camera frame, the `is_any_selected_marker_detected` predicate lights up. This name should always be written with the `stage_` prefix.
- **Marker size**: 
- **Library**: This is the ID number of the HD library utilized by STag markers. The allowed numbers are  `[11, 13, 15, 17, 19, 21, 23]`.
- **Error correction**: 
- **Prefix**: This prefix is used for marker names.


:::TIP

If a decision needs to be made based on the existance of a specific STag marker in the camera frame, it’s name should be indicated in the `Marker Selection` variable.

:::


:::info

If any marker is detected in the camera frame at all, the `is_any_marker_detected` predicate will light up. 

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
