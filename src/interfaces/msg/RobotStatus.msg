# Robot status broadcast to HMI on /robot/status

std_msgs/Header header

string STATUS_IDLE = "IDLE"
string STATUS_MOVING = "MOVING"
string STATUS_REFILLING = "REFILLING"
string STATUS_PICKING = "PICKING"
string STATUS_DISPOSING = "DISPOSING"
string STATUS_TRANSFER_NORMAL = "TRANSFER_NORMAL"
string STATUS_ERROR = "ERROR"
string STATUS_EMERGENCY_STOP = "EMERGENCY_STOP"

string status
string current_task
string detail
