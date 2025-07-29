# File: your_package/msg/PointPair.msg
std_msgs/Header      header    # time + frame_id
geometry_msgs/Point desired   # desired (X,Y,Z)
geometry_msgs/Point actual    # actual  (X,Y,Z)
geometry_msgs/Point error     # error    (X,Y,Z)