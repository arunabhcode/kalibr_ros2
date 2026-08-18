# Validation

## Validation goal

The checks must prove row-aware timing without a global-shutter regression. A successful build alone does not prove this behavior.

## 1. Static interface checks

Verify the exported rolling-shutter types before application tests.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -c "import aslam_cv, aslam_cv_backend; print('ASLAM imports pass')"
```

Verify these Python API groups:

- Rolling-shutter camera geometries.
- Rolling-shutter shutter models.
- Camera design variables.
- Shutter design variables.
- Per-keypoint temporal offsets.
- Rolling-shutter reprojection errors.

If an API name differs, use the installed binding name. Do not add a duplicate wrapper.

## 2. Build checks

Build the affected package and its dependencies.

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to kalibr_imu_camera --symlink-install --event-handlers console_direct+
```

Source the new overlay.

```bash
source install/setup.bash
```

Verify the installed command.

```bash
ros2 pkg executables kalibr_imu_camera
ros2 run kalibr_imu_camera kalibr_calibrate_imu_camera --help
```

The help output must contain these options:

- `--estimate-line-delay`.
- `--estimate-intrinsics`.
- `--estimate-distortion`.
- `--timeoffset-pattern`.

The help output must still contain `--bag-freq`.

## 3. YAML round-trip check

Create one camera file with a positive `line_delay` value. Use seconds per row.

Read the file through `CameraChainParameters`. Write it to a second file.

Verify that both values match exactly. Verify that the factory selects a rolling-shutter geometry.

Repeat the check without `line_delay`. Verify that the factory selects the original global-shutter geometry.

Test these invalid values:

- Zero.
- A negative value.
- `NaN`.
- Infinity.
- A string.

Each invalid value must stop before dataset extraction.

## 4. Global-shutter regression

Run a known global-shutter camera-IMU dataset without new options.

```bash
ros2 run kalibr_imu_camera kalibr_calibrate_imu_camera \
  --bag <global-shutter-bag> \
  --cams <global-shutter-camchain.yaml> \
  --imu <imu.yaml> \
  --target <target.yaml>
```

Verify these results:

- The command uses the global-shutter geometry.
- The command does not write `line_delay`.
- The transform stays within the accepted baseline tolerance.
- The time shift stays within the accepted baseline tolerance.
- The report and result file complete without new fields.

## 5. Fixed rolling-shutter check

Run a rolling-shutter camera file without `--estimate-line-delay`.

The optimizer must use row-aware corner times. The line-delay design variable must remain fixed.

Verify that the output line delay equals the input value. Verify that the output transform is finite.

## 6. Estimated rolling-shutter check

Run the same dataset with line-delay estimation.

```bash
ros2 run kalibr_imu_camera kalibr_calibrate_imu_camera \
  --bag <rolling-shutter-bag> \
  --cams <rolling-shutter-camchain.yaml> \
  --imu <imu.yaml> \
  --target <target.yaml> \
  --estimate-line-delay \
  --timeoffset-pattern 0.08
```

Verify these results:

- The optimizer activates one shutter variable per rolling-shutter camera.
- The result contains a finite positive line delay.
- The YAML value uses seconds per row.
- The text result contains the same value.
- The PDF text section contains the same value.
- The optimizer reports no spline-bound error.

## 7. Synthetic row-time check

Use a synthetic motion sequence with known line delay. Use target corners across the full image height.

Compare these two runs:

1. A forced global-shutter model.
2. The correct rolling-shutter model.

The rolling-shutter run must recover the injected delay within the selected tolerance. It must reduce row-correlated reprojection error.

Plot residual against image row. A correct model removes the dominant row trend.

## 8. Parameter combination matrix

Run the smallest dataset for each row in this matrix.

| Line delay | Intrinsics | Distortion | Time shift | Chain transforms | Required result |
|---|---|---|---|---|---|
| Fixed | Fixed | Fixed | Active | Fixed | Existing transform result plus row-aware timing. |
| Active | Fixed | Fixed | Active | Fixed | Stable line delay and time shift. |
| Active | Active | Fixed | Active | Fixed | Correct covariance ranges. |
| Active | Fixed | Active | Active | Fixed | Correct model-specific distortion size. |
| Active | Active | Active | Active | Active | Complete covariance and YAML output. |
| Fixed | Fixed | Fixed | Fixed | Fixed | No active temporal camera variables. |

Run the matrix for one camera first. Repeat the first and final rows for two cameras.

## 9. Multi-IMU regression

Run the existing multi-IMU command path. Use at least two IMU files and explicit IMU models.

Verify these results:

- IMU indices remain stable.
- Inter-IMU transforms remain present.
- Inter-IMU time offsets remain present when requested.
- Constant-bias mode still disables bias motion terms.
- Camera shutter selection does not change IMU model selection.

## 10. Covariance check

Run covariance recovery for one small dataset. Enable each optional camera variable in sequence.

Verify that every standard deviation maps to the correct result. Verify the expected size before each slice.

Reject an index mismatch. Do not print a shifted covariance as a valid result.

## 11. Acceptance criteria

The feature is complete only when all criteria pass:

1. The existing command accepts all new options.
2. A global-shutter run keeps its previous behavior.
3. A fixed rolling-shutter run uses per-corner times.
4. An estimated rolling-shutter run writes a positive line delay.
5. All outputs use seconds per row.
6. Multi-IMU and constant-bias paths still work.
7. Optimizer failure prevents result file creation.
8. YAML round-trip checks preserve shutter data.
9. Covariance values map to the correct variables.
10. The package build and command smoke check pass.

## 12. Data quality checks

Use corners across the full sensor height. Use motion about at least three rotation axes.

Keep the target sharp and visible. Keep camera and IMU timestamps in one clock domain.

Reject a dataset with poor row coverage. Such data cannot constrain line delay.
