# Virtual wall obstacle;
#  - The yaw provides the orientation of x-axis
#  - Assumed vertically aligned (roll and pitch must be 0)
#  - Z provides the lower border of the wall (normally 0)

string  name
float32 length
float32 width
float32 height
geometry_msgs/PoseWithCovarianceStamped pose
