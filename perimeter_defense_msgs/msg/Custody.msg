# Custody of a captured person: who escorts him to the detention building.
# Reported by the UGV that captured him (fully robotic base) or, in the
# scenario with soldiers, by the soldier who takes over from the UGV.
uint8 DISPATCHED = 0     # escort on the way to the captured person (soldier only)
uint8 ESCORTING = 1      # escort has the person, on the way to the detention building
uint8 DETAINED = 2       # person locked in the detention building

std_msgs/Header header
uint32 track_id
string escort            # call sign of the escort (LUP-2, SLD-1, ...)
uint8 state
geometry_msgs/Point position   # escort position at the time of the report
