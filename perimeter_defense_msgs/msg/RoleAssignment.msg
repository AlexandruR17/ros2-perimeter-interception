uint8 PATROL = 0
uint8 INTERCEPT = 1
uint8 GUARD = 2
uint8 FLANK = 3
uint8 INVESTIGATE = 4
uint8 SEARCH = 5
uint8 BLOCK = 6
uint8 FOLLOW = 7
uint8 HOLD = 8           # stays with a captured person (securing him)
uint8 ESCORT = 9         # leads a captured person to the detention building
uint8 DISPATCH = 10      # transport UGV driving to a captured person to take him over
uint8 RETURN = 11        # transport UGV driving back to its parking place
uint8 STANDBY = 12       # transport UGV parked, available

std_msgs/Header header
string robot
uint8 role
uint32 track_id
geometry_msgs/Point goal
float64 eta              # s, estimated time to reach the goal
