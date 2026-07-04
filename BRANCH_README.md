# Manual Assisted Record/Playback Branch

This branch is used to develop and test the manual-assisted control workflow for
the ASUQTR submarine.

The main goal is to validate a pilot-friendly "carrot on a stick" control mode
where the pilot commands a target motion, while the LQR remains responsible for
thruster commands, stabilization, depth regulation, and disturbance rejection.

## Scope

This branch focuses on:

- `MANUAL_ASSISTED` control mode;
- joystick/gamepad assisted piloting;
- dashboard-based control and diagnostics;
- recording trajectories from manual-assisted operation;
- playback in world-frame and body-frame modes;
- playback progression by timestamp or lookahead/setpoint-style target tracking;
- LQR profile switching for `Standstill`, `Forward`, and `Forward + Turning`;
- Unity simulation tests and pool-test launch configurations.

## Important Note About EKF And Sensor Fusion

The EKF, sensor fusion, DVL, IMU, and no-magnetometer configurations in this
branch are included so that pool tests can be run with the current integration
setup.

They are not always the most up-to-date or authoritative reference for the
navigation stack.

For sensor fusion, EKF tuning, DVL integration, or no-magnetometer yaw work,
check the dedicated navigation or pool-test branches before treating this branch
as the source of truth.

## Typical Launch Modes

Use the Unity/simulation launch when developing the dashboard, gamepad control,
recording, and playback without the physical submarine.

Use the pool/no-mag launch only when testing the real submarine with the current
pool-test sensor fusion configuration.

The intent is to keep one development branch that supports both workflows:

- simulation mode for fast iteration;
- pool mode for integration with the real robot.

