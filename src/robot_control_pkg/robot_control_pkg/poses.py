"""Fixed pose-name catalog shared by doosan_robot_control_node (which owns
the actual [x, y, z, rx, ry, rz] values, loaded from config/robot_params.yaml)
and robot_task_manager_node (which only needs to know which names exist to
sequence a task). Mirrors the coordinate map in spec doc section 13.
"""

HOME_POSE = "home_pose"
WASTE_POSE = "waste_pose"
WASTE_DOWN_POSE = "waste_down_pose"
NORMAL_TRAY_APPROACH_POSE = "normal_tray_approach_pose"
NORMAL_TRAY_POSE = "normal_tray_pose"
TRAY_PICK_POSE = "tray_pick_pose"
REFILL_POSE = "refill_pose"

def tube_approach_pose(tube_index: int) -> str:
    return f"tube_{tube_index}_approach_pose"


def tube_pick_pose(tube_index: int) -> str:
    return f"tube_{tube_index}_pick_pose"


def tube_refill_pose(tube_index: int) -> str:
    return f"tube_{tube_index}_refill_pose"


def all_pose_names(num_tubes: int) -> list:
    names = [HOME_POSE, WASTE_POSE, WASTE_DOWN_POSE, NORMAL_TRAY_APPROACH_POSE, NORMAL_TRAY_POSE, TRAY_PICK_POSE, REFILL_POSE]
    for i in range(num_tubes):
        names += [tube_approach_pose(i), tube_pick_pose(i), tube_refill_pose(i)]
    return names
