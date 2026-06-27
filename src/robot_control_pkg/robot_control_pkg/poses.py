"""Fixed pose-name catalog shared by doosan_robot_control_node (which owns
the actual [x, y, z, rx, ry, rz] values, loaded from config/robot_params.yaml)
and robot_task_manager_node (which only needs to know which names exist to
sequence a task).

Physical layout: 3 trays (3x1 each) are stacked in depth in front of the
robot/camera; only the front-most tray is ever the active one. tray_idx
(0, 1, 2) selects which depth slot a dispose/refill/transfer target sits
in. Each pose is taught/calibrated independently in the yaml (not derived
by a tray_gap/tube_gap formula at runtime) since real-world placement can
drift from the ideal spacing - tray_gap/tube_gap stay in the yaml purely
as a reference for filling in new poses.
"""

HOME_POSE = "home_pose"
WASTE_APPROACH_POSE = "waste_approach_pose"
WASTE_RELEASE_POSE = "waste_release_pose"
WASTE_ROTATE_POSE = "waste_rotate_pose"
WASTE_ROTATE_MIDDLE_POSE = "waste_rotate_middle_pose"
WASTE_ROTATE_REAGENT_POSE = "waste_rotate_reagent_pose"
WASTE_ROTATE_TUBE_POSE = "waste_rotate_tube_pose"

TRAY_TOOL_STAND_APPROACH_POSE = "tray_tool_stand_approach_pose"
TRAY_TOOL_STAND_GRIP_POSE = "tray_tool_stand_grip_pose"

REFILL_SOURCE_APPROACH_POSE = "refill_source_approach_pose"
REFILL_SOURCE_GRIP_POSE = "refill_source_grip_pose"


def tray_transfer_tool_approach_pose(tray_idx: int) -> str:
    return f"tray_transfer_{tray_idx}_tool_approach_pose"


def tray_transfer_tool_preinsert_pose(tray_idx: int) -> str:
    return f"tray_transfer_{tray_idx}_tool_preinsert_pose"


def tray_transfer_tool_insert_pose(tray_idx: int) -> str:
    return f"tray_transfer_{tray_idx}_tool_insert_pose"


def tray_transfer_lift_pose(tray_idx: int) -> str:
    return f"tray_transfer_{tray_idx}_lift_pose"


def tray_transfer_success_approach_pose(tray_idx: int) -> str:
    return f"tray_transfer_{tray_idx}_success_approach_pose"


def tray_transfer_success_place_pose(tray_idx: int) -> str:
    return f"tray_transfer_{tray_idx}_success_place_pose"


def tray_transfer_tool_detach_pose(tray_idx: int) -> str:
    return f"tray_transfer_{tray_idx}_tool_detach_pose"


def refill_target_approach_pose(tray_idx: int, tube_idx: int) -> str:
    return f"refill_target_{tray_idx}_{tube_idx}_approach_pose"


def refill_target_work_pose(tray_idx: int, tube_idx: int) -> str:
    return f"refill_target_{tray_idx}_{tube_idx}_work_pose"


def refill_target_pour_pose(tray_idx: int, tube_idx: int) -> str:
    return f"refill_target_{tray_idx}_{tube_idx}_pour_pose"


def dispose_tube_approach_pose(tray_idx: int, tube_idx: int) -> str:
    return f"dispose_tray_{tray_idx}_tube_{tube_idx}_approach_pose"


def dispose_tube_grip_pose(tray_idx: int, tube_idx: int) -> str:
    return f"dispose_tray_{tray_idx}_tube_{tube_idx}_grip_pose"


def all_pose_names(num_tubes: int = 3, num_trays: int = 3) -> list:
    names = [
        HOME_POSE,
        WASTE_APPROACH_POSE,
        WASTE_RELEASE_POSE,
        WASTE_ROTATE_POSE,
        WASTE_ROTATE_MIDDLE_POSE,
        WASTE_ROTATE_REAGENT_POSE,
        WASTE_ROTATE_TUBE_POSE,
        TRAY_TOOL_STAND_APPROACH_POSE,
        TRAY_TOOL_STAND_GRIP_POSE,
        REFILL_SOURCE_APPROACH_POSE,
        REFILL_SOURCE_GRIP_POSE,
    ]
    for tray_idx in range(num_trays):
        names += [
            tray_transfer_tool_approach_pose(tray_idx),
            tray_transfer_tool_preinsert_pose(tray_idx),
            tray_transfer_tool_insert_pose(tray_idx),
            tray_transfer_lift_pose(tray_idx),
            tray_transfer_success_approach_pose(tray_idx),
            tray_transfer_success_place_pose(tray_idx),
            tray_transfer_tool_detach_pose(tray_idx),
        ]
        for tube_idx in range(num_tubes):
            names += [
                refill_target_approach_pose(tray_idx, tube_idx),
                refill_target_work_pose(tray_idx, tube_idx),
                refill_target_pour_pose(tray_idx, tube_idx),
                dispose_tube_approach_pose(tray_idx, tube_idx),
                dispose_tube_grip_pose(tray_idx, tube_idx),
            ]
    return names
