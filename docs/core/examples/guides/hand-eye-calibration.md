---
sidebar_position: 14
title: Hand-Eye calibration
---
// Should the name of the document change to "robot calibration" ?


# Hand-Eye calibration

Robot calibration is a fundamental prerequisite for any robotic system that relies on precise coordination between a vision sensor and a manipulator. It establishes the spatial transformation between the robot’s end-effector and the camera frame, enabling accurate mapping between observed features and actionable robot coordinates. Without proper calibration, even high-quality perception or motion planning algorithms can yield significant positioning errors.

The AICA's `core-vision` package provides a structured workflow for performing hand–eye calibration efficiently and reproducibly.

Accurate hand–eye calibration is critical in tasks such as visual servoing, object manipulation, inspection, and assembly. This tool is designed to minimize discrepancies and provide reliable calibration outputs suitable for industrial environments.
