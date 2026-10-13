# pci_general
Interface between gbplanner and controller

## Optional initialization yaw scan

Set `init_motion/yaw_scan_enable: true` and `init_motion/yaw_scan_rate: 0.35`
in the PCI configuration to perform a counterclockwise full turn after the
existing takeoff/drop/forward initialization motion has reached its final
position. The rate is in rad/s (0.35 is about 20 degrees/s); it must be positive
and finite. The option defaults to false and leaves the old initialization
behavior unchanged.

With the scan enabled, PCI waits until the robot is within 0.3 m of the final
initialization position and below 0.15 m/s for 0.5 seconds, then streams single-pose yaw references at 20 Hz while
holding that position. It stays running until odometry confirms at least
`2*pi - 0.1` radians of accumulated rotation and a final heading error below
0.1 rad. This avoids controllers skipping stationary intermediate waypoints.
The configured rate controls the reference; measured tracking depends on the
controller. The existing Stop Planner service cancels the scan and holds the
current pose. A simulated-time timeout (nominal initialization duration plus
nominal scan duration plus 30 seconds) also cancels and reports an error.

For Unipilot, configure these entries under `init_motion` in robot_bringup's
`config/ros1/gbplanner/uav_nmpc_unipilot_sim/planner_control_interface_config.yaml`.
Click Initialization as usual; no agent or controller changes are required.

The benchmark can request only the stationary scan, after its own takeoff, through
`/pci_general/initialization_yaw_scan` (`std_srvs/Trigger`). It requires the scan
option enabled and PCI ready. `/pci_general/initialization_yaw_scan_status`
returns success only after the measured turn completes; its message is `running`,
`completed`, or `failed or cancelled`. Normal Initialization still includes its
configured translational motion.
