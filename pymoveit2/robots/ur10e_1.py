from typing import List

MOVE_GROUP_ARM: str = "ur_manipulator"
MOVE_GROUP_GRIPPER: str = "ur_gripper"

OPEN_GRIPPER_JOINT_POSITIONS: List[float] = [0.04, 0.04]
CLOSED_GRIPPER_JOINT_POSITIONS: List[float] = [0.0, 0.0]


def joint_names(prefix: str = "") -> List[str]:
    return [
        'shoulder_pan_joint',
        'shoulder_lift_joint',
        'elbow_joint',
        'wrist_1_joint',
        'wrist_2_joint',
        'wrist_3_joint'
    ]


def base_link_name(prefix: str = "") -> str:
    return "base_link"


def end_effector_name(prefix: str = "") -> str:
    return "tool0"


def gripper_joint_names(prefix: str = "") -> List[str]:
    return [
        prefix + "finger_joint1",
        prefix + "finger_joint2",
    ]
