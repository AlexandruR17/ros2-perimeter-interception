# One tracked target, constant velocity Kalman filter state [x, y, vx, vy] in the map frame.

# Track state
uint8 STATE_ALERT = 0         # fence sensor only, no lidar confirmation
uint8 STATE_TRACKED = 1
uint8 STATE_LOST = 2          # no detection for lost_timeout, prediction continues
uint8 STATE_NEUTRALIZED = 3

# Classification (set by the IFF node)
uint8 CLASS_UNKNOWN = 0
uint8 CLASS_FRIEND = 1
uint8 CLASS_HOSTILE = 2

uint32 id
uint8 state
uint8 classification
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
float64[16] covariance        # row-major 4x4 over [x, y, vx, vy]
builtin_interfaces/Time first_seen
builtin_interfaces/Time last_update   # last measurement update
uint32 hits
string transponder_id         # set when validated by a transponder
