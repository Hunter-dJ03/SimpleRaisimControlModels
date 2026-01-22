# quadruped_description

The `quadruped_description` package defines the physical configuration, URDF model, and associated resources for the quadruped robot. It provides mesh files, a robot description, and configurable parameters to be used by simulation or control packages.

## Features

This package contains:
- A URDF model of the robot with articulated joints
- Mesh files for visualization and collision
- Configuration parameters for initial pose and link geometry

## Configs

#### `leg_config.yaml`

| Parameter                  | Unit    | Description                                                        |
| -------------------------- | ------- | ------------------------------------------------------------------ |
| `body_initial_position`    | meters  | Initial position of robot base in world frame `[x, y, z]`          |
| `body_initial_orientation` | N/A     | Initial orientation as quaternion `[x, y, z, w]`                   |
| `joint_initial_positions`  | radians | Initial joint angles: hipAA, hipFE, kneeFE for each leg (12 total) |
| `link_lengths`             | meters  | Length of each link in the leg: coxa, femur, tibia                 |
