# Custody of a captured person, reported by the soldier who takes over from
# the UGV (radio report).
uint8 DISPATCHED = 0     # soldier on the way to the captured person
uint8 ESCORTING = 1      # soldier has the person, walking to the detention building
uint8 DETAINED = 2       # person locked in the detention building

std_msgs/Header header
uint32 track_id
string soldier           # call sign
uint8 state
geometry_msgs/Point position   # soldier position at the time of the report
