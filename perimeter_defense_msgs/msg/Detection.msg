# A person-sized lidar cluster in the map frame (not on the static map).
geometry_msgs/Point position
float64 width            # m, extent of the cluster
float64 range            # m, distance from the observing robot
uint32 points            # number of lidar returns in the cluster
string source            # robot namespace that observed it (fused: comma separated)
