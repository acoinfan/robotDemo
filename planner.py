from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation

from object import Object, Table


CM_TO_M = 0.01
DEFAULT_APPROACH_HEIGHT = 0.10
DEFAULT_RELEASE_HEIGHT = 0.0
DEFAULT_MIN_POSITION_DELTA = 5.0
DEFAULT_GRIPPER_MAX_WIDTH = 0.08
DEFAULT_EE_DOWN_QUAT = Rotation.from_euler("xyz", [np.pi, 0.0, 0.0]).as_quat()


@dataclass
class Pose:
    translation: np.ndarray
    quaternion: np.ndarray


@dataclass
class PickPlacePlan:
    score: float
    position_delta: float
    rotation_delta: float
    object: Object
    target: Object
    grasp_width: float
    poses: list[Pose]


@dataclass
class TableFrame:
    translation: np.ndarray
    yaw: float = 0.0

    @classmethod
    def identity(cls):
        return cls(np.zeros(3), 0.0)

    @classmethod
    def from_values(cls, xyz, yaw: float = 0.0):
        return cls(np.array(xyz, dtype=float), yaw)

    def transform_position(self, position_cm) -> np.ndarray:
        position_m = np.array(position_cm, dtype=float) * CM_TO_M
        rotation = Rotation.from_euler("z", self.yaw)
        return rotation.apply(position_m) + self.translation

    def transform_quaternion(self, quaternion) -> np.ndarray:
        rotation = Rotation.from_euler("z", self.yaw) * Rotation.from_quat(quaternion)
        return rotation.as_quat()


def generate_pick_place_plan(
    origin_table: Table,
    end_table: Table,
    table_frame: TableFrame | None = None,
    approach_height: float = DEFAULT_APPROACH_HEIGHT,
    release_height: float = DEFAULT_RELEASE_HEIGHT,
    min_position_delta: float = DEFAULT_MIN_POSITION_DELTA,
) -> PickPlacePlan:
    table_frame = table_frame or TableFrame.identity()
    moved = find_moved_object(origin_table, end_table, min_position_delta)
    origin_obj, end_obj = moved["origin"], moved["end"]

    pick_pose = grasp_pose(origin_obj, table_frame)
    place_pose = grasp_pose(end_obj, table_frame, z_offset=release_height)
    pick_above = offset_z(pick_pose, approach_height)
    place_above = offset_z(place_pose, approach_height)

    return PickPlacePlan(
        score=moved["score"],
        position_delta=moved["position_delta"],
        rotation_delta=moved["rotation_delta"],
        object=origin_obj,
        target=end_obj,
        grasp_width=grasp_width(origin_obj),
        poses=[pick_above, pick_pose, pick_above, place_above, place_pose, place_above],
    )


def find_moved_object(
    origin_table: Table,
    end_table: Table,
    min_position_delta: float = DEFAULT_MIN_POSITION_DELTA,
) -> dict:
    diffs = origin_table.diff_objects(end_table, lambda p, r: p + r * 10.0)
    if not diffs:
        raise ValueError("No matching objects were found between origin and end scenes.")

    score, origin_obj, end_obj = diffs[0]
    position_delta = origin_obj.position_changed(end_obj)
    rotation_delta = origin_obj.rotation_changed(end_obj)
    if position_delta < min_position_delta:
        raise ValueError(
            f"No object moved more than {min_position_delta} cm; "
            f"largest move was {position_delta:.3f} cm for {origin_obj.name}."
        )

    return {
        "score": score,
        "position_delta": position_delta,
        "rotation_delta": rotation_delta,
        "origin": origin_obj,
        "end": end_obj,
    }


def grasp_pose(obj: Object, table_frame: TableFrame, z_offset: float = 0.0) -> Pose:
    position = table_frame.transform_position(obj.position)
    position[2] += z_offset
    return Pose(position, table_frame.transform_quaternion(top_down_gripper_quat(obj)))


def top_down_gripper_quat(obj: Object) -> np.ndarray:
    yaw = obj.z_rotation if is_long_object(obj) else 0.0
    return (Rotation.from_euler("z", yaw) * Rotation.from_quat(DEFAULT_EE_DOWN_QUAT)).as_quat()


def is_long_object(obj: Object) -> bool:
    horizontal = sorted(obj.size[:2])
    return horizontal[1] > horizontal[0] * 2.0


def offset_z(pose: Pose, dz: float) -> Pose:
    shifted = np.array(pose.translation, dtype=float)
    shifted[2] += dz
    return Pose(shifted, np.array(pose.quaternion, dtype=float))


def grasp_width(obj: Object) -> float:
    width = min(obj.size[0], obj.size[1]) * CM_TO_M * 0.8
    return min(max(width, 0.005), DEFAULT_GRIPPER_MAX_WIDTH)
