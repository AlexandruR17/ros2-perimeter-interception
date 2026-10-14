uint8 TYPE_FENCE_BREACH = 0
uint8 TYPE_UNAUTHORIZED_ZONE = 1   # own personnel outside the authorized zones
uint8 TYPE_HOSTILE_CONFIRMED = 2
uint8 TYPE_OBJECTIVE_THREAT = 3
uint8 TYPE_TRACK_LOST = 4
uint8 TYPE_SPOT_REPORT = 5      # soldier sighting reported by radio (coarse position)

uint8 SEVERITY_INFO = 0
uint8 SEVERITY_WARNING = 1
uint8 SEVERITY_CRITICAL = 2

std_msgs/Header header
uint32 id
uint8 type
uint8 severity
string source            # sensor or node that raised the alert
geometry_msgs/Point position
float64 uncertainty      # m, one sigma radius of the position
uint32 track_id          # 0 when not related to a track
string text
