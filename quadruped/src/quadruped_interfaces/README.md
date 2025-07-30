# quadruped_interfaces

The `quadruped_interfaces` package defines custom ROS 2 message types used for communication between various nodes in the quadruped robot system.

## Message Definitions

#### `Endpoint.msg`

This message captures the desired, actual, and error position of a single quadruped foot in 3D space.

```plaintext
std_msgs/Header header          # Timestamp and coordinate frame
geometry_msgs/Point desired     # Desired foot position (X, Y, Z)
geometry_msgs/Point actual      # Actual measured foot position (X, Y, Z)
geometry_msgs/Point error       # Difference: desired - actual (X, Y, Z)
