# Robot status broadcast to HMI on /robot/status
# 20260626 JH, GRIPPER OPEN/CLOSE status 추가, log 수신을 위한 string log 추가
std_msgs/Header header

string STATUS_IDLE = "IDLE"
string STATUS_MOVING = "MOVING"
string STATUS_REFILLING = "REFILLING"
string STATUS_PICKING = "PICKING"
string STATUS_DISPOSING = "DISPOSING"
string STATUS_TRANSFER_TRAY = "TRANSFER_TRAY"
string STATUS_ERROR = "ERROR"
string STATUS_EMERGENCY_STOP = "EMERGENCY_STOP"
string STATUS_GRIPPER_OPEN = "GRIPPER OPEN"
string STATUS_GRIPPER_CLOSE = "GRIPPER CLOSE"

string status
string current_task
string detail
string log