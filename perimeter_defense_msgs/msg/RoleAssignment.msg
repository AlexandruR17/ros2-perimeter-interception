uint8 PATROL = 0
uint8 INTERCEPT = 1
uint8 GUARD = 2
uint8 FLANK = 3
uint8 INVESTIGATE = 4
uint8 SEARCH = 5
uint8 BLOCK = 6
uint8 FOLLOW = 7

std_msgs/Header header
string robot
uint8 role
uint32 track_id
geometry_msgs/Point goal
float64 eta              # s, estimated time to reach the goal
