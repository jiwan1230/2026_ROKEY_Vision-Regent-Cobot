# Per-tube classification result (State 0/1/2)
# Published on /vision/tube_state by tube_state_publisher_node

std_msgs/Header header

int32 STATE_REFILL_NEEDED = 0
int32 STATE_NORMAL = 1
int32 STATE_DISPOSE_NEEDED = 2
int32 STATE_UNKNOWN = -1

int32[] tube_index
int32[] state
