# Rolling-Shutter Camera-IMU Calibration

## Status

This directory defines the repository integration for rolling-shutter-aware camera-IMU calibration.

The three files in `rs_feature/` are exemplars. They are not safe replacements for the current ROS2 files.

A direct replacement removes current ROS2 and multi-IMU behavior. Use the selective merge contract in this directory.

## Goal

The feature extends `kalibr_calibrate_imu_camera` with row-aware exposure times. It can estimate these camera values:

- The camera-to-IMU transform.
- The camera-to-IMU time shift.
- The rolling-shutter line delay.
- The projection parameters.
- The distortion parameters.
- Optional camera-chain transforms.

The command keeps the current global-shutter result when the camera file has no `line_delay` field.

## Repository position

```text
ROS2 bag and YAML files
        |
        v
kalibr_calibrate_imu_camera
        |
        v
kalibr_common.ConfigReader
        |
        v
IccCameraChain and IccCamera
        |
        v
IccCalibrator
        |
        v
ASLAM rolling-shutter bindings
        |
        v
camchain-imucam YAML, text result, and PDF report
```

The C++ and Boost.Python layers already expose the required optimizer types. The missing connection is in the application package.

## Exemplar inventory

| Exemplar | Purpose | Selective merge target |
|---|---|---|
| `rs_feature/IccSensors.py` | Adds row times, shutter variables, and shutter-aware reprojection errors. | `src/kalibr_imu_camera/python/kalibr_imu_camera_calibration/IccSensors.py` |
| `rs_feature/IccCalibrator.py` | Adds feature flags, covariance association, and result storage. | `src/kalibr_imu_camera/python/kalibr_imu_camera_calibration/IccCalibrator.py` |
| `rs_feature/kalibr_calibrate_imu_camera` | Adds command options and passes the feature flags. | `src/kalibr_imu_camera/python/kalibr_calibrate_imu_camera` |

## Existing support

The repository already contains these lower-level parts:

- Rolling-shutter camera geometry exports in `aslam_cv_python`.
- Rolling-shutter design-variable exports in `aslam_cv_backend_python`.
- Per-keypoint temporal offsets.
- Shutter-aware reprojection errors.
- Adaptive covariance reprojection errors.
- A camera-only reference in `kalibr_rs_camera_calibration/RsCalibrator.py`.

The official Kalibr wiki also lists rolling-shutter camera calibration as a calibration mode. See the [Kalibr Wiki](https://github.com/ethz-asl/kalibr/wiki/).

Content from the external source was rephrased for compliance with licensing restrictions.

## Missing application support

The current application has these gaps:

1. `ConfigReader.py` writes `line_delay`, but it does not read or validate the value.
2. `AslamCamera.fromParameters()` always creates a global-shutter geometry.
3. `IccSensors.py` uses one pose time for all corners in an image.
4. `IccCalibrator.py` does not own camera estimation flags.
5. `IccUtil.py` does not report line delay, intrinsics, or distortion results.
6. The command does not expose the three new estimation flags.
7. The package has no rolling-shutter camera-IMU regression test.

## Documents

- [Architecture](architecture.md) explains the timing model and optimizer flow.
- [Integration contract](integration-contract.md) defines each required repository change.
- [Validation](validation.md) defines build, regression, and acceptance checks.

## Required compatibility

The implementation must preserve these current behaviors:

- ROS2 bag access through `rosbag2_py`.
- The `--bag-freq` option.
- Multiple IMUs and their stable indices.
- The three supported IMU models.
- Constant and spline IMU bias modes.
- Existing global-shutter camera models.
- Existing file names and result artifacts.
- Existing camera-chain transform behavior.

## Recommended scope

Extend the existing `kalibr_calibrate_imu_camera` command. Do not add a second camera-IMU command.

Do not add a dependency. The required bindings already exist in this workspace.

Do not replace complete files with the exemplars. Merge only the rolling-shutter behavior.

## Terms

| Term | Meaning |
|---|---|
| Frame time | The image timestamp after the camera-to-IMU shift. |
| Line delay | The exposure time difference between adjacent image rows. |
| Center row | The temporal reference row for the image timestamp. |
| Keypoint time | The exposure time for one detected target corner. |
| Projection parameters | Focal length, principal point, and model-specific values. |
| Distortion parameters | Coefficients for the selected distortion model. |
