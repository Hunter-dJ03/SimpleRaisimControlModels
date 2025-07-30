# quadruped_control

The `quadruped_control` package contains the primary control node responsible generating joint-level control commands for a quadruped robot from foot trajectories.


## Features

- Cartesian-space foot velocity control (anual input atm, to be switched to reading from a topic)
- Jacobian-based inverse kinematics
- Newton-Euler-based torque computation (gravity + Coriolis/centrifugal)
- Endpoint feedback publishing for foot position tracking
- Parameterized setup for link lengths and initial joint positions


## Node: `control_node`

A ros2 node that handles the calculations of desired joint commands for the quadruped 

### Subscribed Topics

| Topic          | Type                            | Description                        |
|----------------|----------------------------------|------------------------------------|
| `joint_states` | `sensor_msgs/msg/JointState`     | Current joint positions and velocities |

### Published Topics

| Topic                  | Type                                      | Description                               |
|------------------------|-------------------------------------------|-------------------------------------------|
| `joint_desired_control`| `sensor_msgs/msg/JointState`              | Target joint positions, velocities, efforts |
| `endpoint`             | `quadruped_interfaces/msg/Endpoint`       | Cartesian foot position feedback (optional) |
