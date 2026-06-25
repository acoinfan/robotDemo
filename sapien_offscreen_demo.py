from argparse import ArgumentParser
from pathlib import Path
import os

import numpy as np
from PIL import Image
import sapien

from planner import CM_TO_M, TableFrame, generate_pick_place_plan
from task_io import load_archived_task


def main():
    parser = ArgumentParser(description="Render a minimal MesaTask office_table scene with SAPIEN offscreen.")
    parser.add_argument("--task", default="level1_1181_1")
    parser.add_argument("--data-dir", default="data/tasks")
    parser.add_argument("--mesatask-root", default="MesaTask-10K")
    parser.add_argument("--output", default="demo/sapien_offscreen_demo.png")
    args = parser.parse_args()

    os.environ.setdefault("MESA_SHADER_CACHE_DIR", "/tmp/mesa_shader_cache")
    origin_table, end_table = load_archived_task(args.data_dir, args.task)
    plan = generate_pick_place_plan(origin_table, end_table, table_frame=TableFrame.identity())
    scene = build_scene(origin_table, plan, Path(args.mesatask_root))
    image = render_scene(scene)

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(image).save(output)
    print(f"Wrote {output}")


def build_scene(table, plan, mesatask_root: Path):
    scene = sapien.Scene()
    scene.set_timestep(1 / 60)
    scene.set_ambient_light([0.45, 0.45, 0.45])
    scene.add_directional_light([1.0, -1.0, -2.0], [1.0, 1.0, 1.0])

    add_box(
        scene,
        "table",
        center=[
            (table.boundary[0] + table.boundary[1]) * CM_TO_M / 2,
            (table.boundary[2] + table.boundary[3]) * CM_TO_M / 2,
            -0.01,
        ],
        half_size=[
            (table.boundary[1] - table.boundary[0]) * CM_TO_M / 2,
            (table.boundary[3] - table.boundary[2]) * CM_TO_M / 2,
            0.01,
        ],
        color=[0.82, 0.73, 0.58, 1.0],
    )

    for obj in table.objects:
        color = [0.95, 0.55, 0.12, 1.0] if obj.uid == plan.object.uid else [0.45, 0.55, 0.65, 1.0]
        glb = mesatask_root / "Assets_library" / f"{obj.uid}.glb"
        if glb.exists() and glb.stat().st_size > 1024:
            add_mesh(scene, obj.instance, glb, np.array(obj.position, dtype=float) * CM_TO_M)
        else:
            add_box(
                scene,
                obj.instance,
                center=np.array(obj.position, dtype=float) * CM_TO_M,
                half_size=np.maximum(np.array(obj.size, dtype=float) * CM_TO_M / 2, 0.003),
                color=color,
            )

    add_box(
        scene,
        "target",
        center=np.array(plan.target.position, dtype=float) * CM_TO_M,
        half_size=np.maximum(np.array(plan.target.size, dtype=float) * CM_TO_M / 2, 0.003),
        color=[0.1, 0.35, 1.0, 0.35],
    )
    add_box(scene, "gripper", center=plan.poses[0].translation, half_size=[0.04, 0.01, 0.01], color=[0.05, 0.07, 0.1, 1.0])
    return scene


def add_box(scene, name, center, half_size, color):
    builder = scene.create_actor_builder()
    builder.add_box_visual(half_size=half_size, material=color)
    actor = builder.build_static(name=name)
    actor.set_pose(sapien.Pose(np.array(center, dtype=np.float32)))
    return actor


def add_mesh(scene, name, path: Path, center):
    builder = scene.create_actor_builder()
    builder.add_visual_from_file(str(path))
    actor = builder.build_static(name=name)
    actor.set_pose(sapien.Pose(np.array(center, dtype=np.float32)))
    return actor


def render_scene(scene):
    camera = scene.add_camera("camera", 960, 720, 0.9, 0.01, 10)
    camera.set_pose(look_at(np.array([0.72, -0.92, 0.74]), np.array([0.34, 0.32, 0.05])))
    scene.update_render()
    camera.take_picture()
    picture = camera.get_picture("Color")
    rgb = np.clip(picture[..., :3] * 255, 0, 255).astype(np.uint8)
    return rgb


def look_at(eye, target):
    forward = target - eye
    forward = forward / np.linalg.norm(forward)
    left = np.cross([0, 0, 1], forward)
    left = left / np.linalg.norm(left)
    up = np.cross(forward, left)

    matrix = np.eye(4, dtype=np.float32)
    matrix[:3, 0] = forward
    matrix[:3, 1] = left
    matrix[:3, 2] = up
    matrix[:3, 3] = eye
    return sapien.Pose(matrix)


if __name__ == "__main__":
    main()
