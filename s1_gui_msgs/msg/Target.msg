# One place the rover has to reach, as the operator entered it.
#
# The coordinates are the truth: east_m and north_m are filled in by the
# mission manager when it converts, and are carried along so the GUI, the log
# and anyone reading a bag can see exactly what conversion was used without
# repeating it.

uint8 STATUS_PENDING=0
uint8 STATUS_ACTIVE=1
uint8 STATUS_REACHED=2
uint8 STATUS_SKIPPED=3
uint8 STATUS_UNREACHABLE=4

string name
float64 latitude
float64 longitude
float32 tolerance_m      # how close counts as arrival, metres
uint8 status
float64 east_m           # local ROS frame (REP 103), filled by the manager
float64 north_m
