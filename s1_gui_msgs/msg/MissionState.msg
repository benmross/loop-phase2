# What the mission manager believes, published latched at 5 Hz.
#
# Latched because a console that starts late, or restarts, must not have to
# ask: it gets the whole mission, including the target list, on subscribe.

uint8 STATE_IDLE=0
uint8 STATE_RUNNING=1
uint8 STATE_HOLD=2        # motion suppressed, targets kept, can resume
uint8 STATE_ABORTED=3
uint8 STATE_COMPLETE=4

std_msgs/Header header
uint8 state
int16 active_index        # -1 when nothing is active
float32 elapsed_s
float64 origin_latitude   # the frame everything here is expressed against
float64 origin_longitude
string detail             # last thing that happened, for the status line
Target[] targets
