# Integration Contract

## Merge rule

Do not copy the exemplar files over the current files. The exemplars come from a different Kalibr branch.

Preserve the ROS2 port first. Add the rolling-shutter behavior through the current interfaces.

## File map

| File | Required change | Main compatibility risk |
|---|---|---|
| `python/kalibr_common/ConfigReader.py` | Read `line_delay` and construct the correct shutter geometry. | All camera calibration commands use this factory. |
| `python/kalibr_imu_camera_calibration/IccSensors.py` | Add shutter design variables and per-corner times. | The exemplar uses different dataset and IMU signatures. |
| `python/kalibr_imu_camera_calibration/IccCalibrator.py` | Add a configuration object and result storage. | The exemplar changes bias and covariance behavior. |
| `python/kalibr_imu_camera_calibration/IccUtil.py` | Report all selected camera values. | Current report code reads legacy calibrator fields. |
| `python/kalibr_calibrate_imu_camera` | Add three camera estimation options. | The exemplar omits current ROS2 options. |
| `CMakeLists.txt` | Keep the extended command install path valid. | A second command creates unnecessary duplication. |
| `setup.py` | Remove or correct the invalid console entry during package cleanup. | It references a module that does not exist. |

Paths in this table are relative to `src/kalibr_imu_camera/`.

## 1. Camera configuration

Add a `CameraParameters.getLineDelay()` accessor. Validate the value before camera construction.

Use these rules:

- Treat an absent `line_delay` field as a global-shutter camera.
- Require a finite positive float for a rolling-shutter camera.
- Store the value in seconds per row.
- Print the shutter type and line delay in camera details.
- Preserve the value after a YAML read and write cycle.

`setLineDelay()` must use the same validation and unit contract.

### Supported model pairs

Connect only model pairs that the existing bindings expose.

| Camera model | Distortion model | Rolling-shutter binding |
|---|---|---|
| `pinhole` | `radtan` | Available |
| `pinhole` | `equidistant` | Available |
| `omni` | `radtan` | Available |

Keep other model pairs on the global-shutter path. Reject a rolling-shutter request for an unsupported pair.

### Factory result

Extend `AslamCamera` with the camera model factory and design-variable bundle. Keep all current members for existing callers.

Do not change the shape of `CameraParameters.getIntrinsics()` or `getDistortion()`.

## 2. Sensor layer

Merge these exemplar concepts into `IccCamera`:

- `isRollingShutter()`.
- `getLineDelaySeconds()`.
- `getResultLineDelay()`.
- `getResultProjection()`.
- `getResultDistortion()`.
- Camera design-variable activation.
- Per-corner temporal offsets.
- Shutter-aware reprojection error construction.
- Reprojection diagnostics with the same corner times.

Preserve these current ROS2 details:

- `initCameraBagDataset()`.
- `parsed.bag_freq`.
- `initImuBagDataset()`.
- The `imuNr` argument.
- The current multi-IMU classes.

### Per-corner time sequence

For each observation, use this sequence:

1. Compute the corrected frame time.
2. Reject an observation outside the spline range.
3. Compute the center-row temporal offset.
4. Compute each corner temporal offset.
5. Evaluate the body pose at each corner time.
6. Construct the shutter-specific reprojection error.

The spline range check must include the maximum row offset. A frame-center check alone is insufficient.

### Design-variable order

Keep a deterministic calibration-group order. Covariance association depends on this order.

Use this camera order:

1. Transform variables when active in the calibration group.
2. Camera-to-IMU time shift.
3. Line delay.
4. Projection parameters.
5. Distortion parameters.

Record the exact order in the covariance code. Do not infer it from active flags at a different call site.

## 3. Calibrator layer

Add an instance configuration object. Do not use a mutable class dictionary for shared state.

The configuration must contain these flags:

- `timeOffset`.
- `chainExtrinsics`.
- `shutter`.
- `intrinsics`.
- `distortion`.
- `gravityLength`.
- `pose`.
- `landmarks`.

Each calibrator instance must own its own flag dictionary. This prevents state leakage between tests or repeated command calls.

Preserve the current `doBiasMotionError` condition. The exemplar adds bias motion terms without that guard.

Preserve the current optimizer failure exception. The exemplar logs failure and can continue with invalid output.

Store the optimizer return value before report code reads final cost values.

### Covariance

Associate standard deviations by the actual design-variable order. Include optional camera values only when their variables are active.

Check these cases:

- One camera with all optional values fixed.
- One rolling-shutter camera with line delay active.
- Two cameras with fixed chain transforms.
- Two cameras with active chain transforms.
- Intrinsics and distortion with different parameter counts.

## 4. Command layer

Add these options to the existing command:

| Option | Effect |
|---|---|
| `--estimate-line-delay` | Activates the shutter design variable. |
| `--estimate-intrinsics` | Activates projection design variables. |
| `--estimate-distortion` | Activates distortion design variables. |
| `--timeoffset-pattern` | Sets constant spline sparsity padding. |

Keep these current options:

- `--bag-freq`.
- `--imu-delay-by-correlation`.
- `--imu-models`.
- `--recompute-camera-chain-extrinsics`.
- `--no-time-calibration`.
- `--timeoffset-padding`.
- `--recover-covariance`.

Pass one configuration object into `IccCalibrator`. Do not add parallel boolean arguments for each new value.

Reject `--estimate-line-delay` when no camera uses a rolling-shutter model. Report the camera index and configuration file.

## 5. Results

Write selected camera results to all applicable outputs:

- Standard output.
- The text result file.
- The PDF report text section.
- The camera-chain YAML file.

Always write the optimized camera-to-IMU transforms. Keep current file names.

Write `line_delay` only for a rolling-shutter camera. Store seconds per row.

Write projection or distortion values only when their estimation flags are active. Preserve fixed input values otherwise.

Fix current report assumptions about `noTimeCalibration`, `std_times`, and covariance indices through one result interface.

## 6. Package installation

The requested feature extends the installed `kalibr_calibrate_imu_camera` script. No new executable is required.

The camera-only `kalibr_rs_camera_calibration` package is outside this feature boundary. Install it only under a separate camera-calibration task.

`setup.py` declares a console entry for `kalibr_imu_camera.calibrate_imu_camera`, but that module does not exist. The CMake script installs the real command.

Use one executable installation mechanism. The package currently uses `ament_cmake` as its build type.

## 7. Failure behavior

Stop calibration before optimization for these errors:

- A nonfinite line delay.
- A nonpositive rolling-shutter line delay.
- An unsupported rolling-shutter model pair.
- A missing rolling-shutter design-variable binding.
- A spline range that excludes a corner time.
- An invalid covariance index range.

Do not write result files after an optimizer failure.

## 8. Deliberate exclusions

Do not add these items in this feature:

- A new camera-IMU command.
- A new camera model.
- A new optimizer backend.
- A new ROS message type.
- A new external dependency.
- A documentation site.
