from dataclasses import dataclass, field
from typing import Tuple
from franky import CartesianMotion, Affine
from scipy.spatial.transform import Rotation
import numpy as np

def delta_quaternion(q_from, q_to):
    r_from = Rotation.from_quat(q_from)
    r_to = Rotation.from_quat(q_to)
    r_delta = r_to * r_from.inv()
    return r_delta.as_quat()

@dataclass 
class Object:
    name: str
    instance: str
    retrieved_uid: str
    selected_uid: str
    original_size: Tuple[float, float, float]
    scale_factor: Tuple[float, float, float]
    rotation: Tuple[float, float, float, float]
    z_rotation: float
    size: Tuple[float, float, float]
    position: Tuple[float, float, float]
    def __eq__(self, other):
        if not isinstance(other, Object):
            return NotImplemented
        return self.name == other.name

    def __lt__(self, other):
        if not isinstance(other, Object):
            return NotImplemented
        return self.name < other.name
    
    @classmethod
    def from_dict(cls, data: dict):
        return cls(
            name=data["name"],
            instance=data["instance"],
            retrieved_uid=data["retrieved_uid"],
            selected_uid=data["selected_uid"],
            original_size=data["original_size"],
            scale_factor=data["scale_factor"],
            rotation=data["rotation"],
            z_rotation=data["z_rotation"],
            size=data["size"],
            position=data["position"],
        )
    
    def position_changed(self, other):
        assert(self == other)
        return np.linalg.norm(
            np.array(self.position) - np.array(other.position)
        )

    def rotation_changed(self, other):
        assert(self == other)
        r1 = Rotation.from_quat(self.rotation)
        r2 = Rotation.from_quat(other.rotation)

        delta = r1.inv() * r2
        return delta.magnitude()

        
    """
    返回需要的移动
    """
    def get_movement(self) -> CartesianMotion:
        return CartesianMotion(Affine(self.position, self.rotation))
        

@dataclass
class Table:
    scene_units: str
    scene_up_axis: str
    boundary: Tuple[float, float, float, float]
    objects: list[Object]
    
    @classmethod
    def from_dict(cls, data: dict):
        scene = data["scene_settings"]
        return cls(
            scene_units=scene["units"],
            scene_up_axis=scene["up_axis"],
            boundary=data["item_placement_zone"],
            objects=sorted([Object.from_dict(o) for o in data["objects"]])
        )
        
    @classmethod
    def from_json(cls, json_file: str):
        from json import load
        with open(json_file, "r", encoding="utf-8") as f:
            data = load(f)
        return cls.from_dict(data)
    
    def diff_objects(self, other, func=lambda pos, rotate: pos+rotate):
        origin_map = {obj.uid: obj for obj in self.objects}
        end_map    = {obj.uid: obj for obj in other.objects}
        diffs: list[tuple[float, Object, Object]] = []
        
        for uid, origin_obj in origin_map.items():
            end_obj = end_map.get(uid)
            if end_obj is None:
                continue
            score = func(origin_obj.position_changed(end_obj),
                              origin_obj.rotation_changed(end_obj))
            diffs.append((score, origin_obj, end_obj))
        
        return sorted(diffs, key=lambda x: x[0], reverse=True)