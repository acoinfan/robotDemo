from argparse import ArgumentParser
from pathlib import Path
import json
import numpy as np
import sapien

from object import Object
from planner import CM_TO_M, TableFrame, generate_pick_place_plan
from task_io import load_archived_task, load_raw_task


DEFAULT_STEP_TIME = 2.0
DEFAULT_DT = 1 / 60
DEFAULT_GRIPPER_HALF_XY = 0.04
DEFAULT_GRIPPER_HALF_Z = 0.012
DEFAULT_MAX_SPEED = 0.25


def main():
    parser = ArgumentParser(description="Use SAPIEN/PhysX contacts to audit a MesaTask pick-place trajectory.")
    parser.add_argument("--task", default=None)
    parser.add_argument("--data-dir", default="data/tasks")
    parser.add_argument("--level", type=int, default=None)
    parser.add_argument("--office-table", default=None)
    parser.add_argument("--variant", default=None)
    parser.add_argument("--raw-data-dir", default="data")
    parser.add_argument("--all", action="store_true", help="Audit all archived tasks under --data-dir.")
    parser.add_argument("--quiet", action="store_true", help="Only print summary for --all.")
    parser.add_argument("--output", default=None)
    parser.add_argument("--step-time", type=float, default=DEFAULT_STEP_TIME)
    parser.add_argument("--dt", type=float, default=DEFAULT_DT)
    parser.add_argument("--max-speed", type=float, default=DEFAULT_MAX_SPEED)
    parser.add_argument("--gripper-half-xy", type=float, default=DEFAULT_GRIPPER_HALF_XY)
    parser.add_argument("--gripper-half-z", type=float, default=DEFAULT_GRIPPER_HALF_Z)
    args = parser.parse_args()

    records = audit_many(args)
    print_reports(records, args.quiet)
    if args.output:
        Path(args.output).parent.mkdir(parents=True, exist_ok=True)
        with open(args.output, "w", encoding="utf-8") as f:
            json.dump(records, f, indent=2)
            f.write("\n")


def audit_many(args):
    if args.all:
        records = []
        for task in sorted(path.name for path in Path(args.data_dir).iterdir() if path.is_dir()):
            try:
                origin, end = load_archived_task(args.data_dir, task)
                records.append(audit_task(task, origin, end, args))
            except ValueError as exc:
                records.append(skipped(task, exc))
        return records

    name, origin, end = load_task(args)
    try:
        return [audit_task(name, origin, end, args)]
    except ValueError as exc:
        return [skipped(name, exc)]


def load_task(args):
    if args.level is not None or args.office_table is not None or args.variant is not None:
        if args.level is None or args.office_table is None or args.variant is None:
            raise SystemExit("--level, --office-table, and --variant must be provided together.")
        name = f"level{args.level}_{args.office_table}_{args.variant}"
        origin, end = load_raw_task(args.raw_data_dir, args.level, args.office_table, args.variant)
        return name, origin, end

    task = args.task or first_task(args.data_dir)
    origin, end = load_archived_task(args.data_dir, task)
    return task, origin, end


def first_task(data_dir: str) -> str:
    tasks = sorted(path.name for path in Path(data_dir).iterdir() if path.is_dir())
    if not tasks:
        raise SystemExit(f"No tasks found under {data_dir}")
    return tasks[0]


def audit_task(task_name, origin_table, end_table, args):
    plan = generate_pick_place_plan(origin_table, end_table, table_frame=TableFrame.identity())
    scene = sapien.Scene([sapien.physx.PhysxCpuSystem()])
    scene.set_timestep(args.dt)

    actors = {}
    for obj in origin_table.objects:
        if obj.uid == plan.object.uid:
            continue
        actors[obj.uid] = add_object_actor(scene, obj, "locked_dynamic")

    moved_actor = add_object_actor(scene, plan.object, "kinematic")
    gripper_actor = add_gripper(scene, args.gripper_half_xy, args.gripper_half_z)
    table_actor = add_table(scene, origin_table.boundary)
    ignored = {table_actor.name, "ground"}
    moved_name = plan.object.instance

    samples = sample_plan(plan, args.step_time, args.dt)
    events = []
    for sample in samples:
        set_pose(gripper_actor, sample["position"])
        if sample["carrying"]:
            set_pose(moved_actor, sample["position"])
        elif sample["segment"] >= 4:
            set_pose(moved_actor, np.array(plan.target.position, dtype=float) * CM_TO_M)
        else:
            set_pose(moved_actor, np.array(plan.object.position, dtype=float) * CM_TO_M)

        scene.step()
        events.extend(read_contacts(scene, sample, ignored, moved_name))
        if sample["speed"] > args.max_speed:
            events.append(event(sample, "speed", "warning", f"speed {sample['speed']:.3f} m/s exceeds {args.max_speed:.3f} m/s"))

    events = compress_events(events)
    status = "ok"
    if any(item["severity"] == "error" for item in events):
        status = "error"
    elif events:
        status = "warning"
    return {
        "task": task_name,
        "status": status,
        "object": plan.object.instance,
        "target": plan.target.instance,
        "position_delta_cm": plan.position_delta,
        "max_speed_mps": max((sample["speed"] for sample in samples), default=0),
        "events": events,
    }


def skipped(task_name, exc):
    return {
        "task": task_name,
        "status": "skipped",
        "object": None,
        "target": None,
        "position_delta_cm": 0,
        "max_speed_mps": 0,
        "events": [{
            "time": 0,
            "segment": -1,
            "kind": "planning",
            "severity": "warning",
            "message": str(exc),
            "count": 1,
            "first_time": 0,
            "last_time": 0,
        }],
    }


def add_table(scene, boundary):
    min_x, max_x, min_y, max_y = np.array(boundary, dtype=float) * CM_TO_M
    builder = scene.create_actor_builder()
    builder.add_box_collision(half_size=[(max_x - min_x) / 2, (max_y - min_y) / 2, 0.01])
    actor = builder.build_static(name="table")
    actor.set_pose(sapien.Pose(np.array([(min_x + max_x) / 2, (min_y + max_y) / 2, -0.011], dtype=np.float32)))
    return actor


def add_object_actor(scene, obj: Object, kind: str):
    builder = scene.create_actor_builder()
    half = np.maximum(np.array(obj.size, dtype=float) * CM_TO_M / 2, 0.003)
    builder.add_box_collision(half_size=half)
    actor = build_actor(builder, kind, obj.instance)
    set_pose(actor, np.array(obj.position, dtype=float) * CM_TO_M)
    return actor


def add_gripper(scene, half_xy: float, half_z: float):
    builder = scene.create_actor_builder()
    half = np.array([half_xy, half_xy, half_z], dtype=float)
    builder.add_box_collision(half_size=half)
    actor = builder.build_kinematic(name="gripper")
    return actor


def build_actor(builder, kind: str, name: str):
    if kind == "static":
        return builder.build_static(name=name)
    if kind == "kinematic":
        return builder.build_kinematic(name=name)
    actor = builder.build(name=name)
    if kind == "locked_dynamic":
        component = actor.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent)
        component.set_disable_gravity(True)
        component.set_locked_motion_axes([True, True, True, True, True, True])
    return actor


def set_pose(actor, position):
    pose = sapien.Pose(np.array(position, dtype=np.float32))
    actor.set_pose(pose)
    component = actor.find_component_by_type(sapien.physx.PhysxRigidDynamicComponent)
    if component is not None and component.get_kinematic():
        component.set_kinematic_target(pose)


def sample_plan(plan, step_time: float, dt: float):
    samples = []
    poses = [pose.translation for pose in plan.poses]
    steps = max(2, int(np.ceil(step_time / dt)) + 1)
    for segment in range(len(poses) - 1):
        start = poses[segment]
        end = poses[segment + 1]
        for i in range(steps):
            if segment > 0 and i == 0:
                continue
            alpha = i / (steps - 1)
            position = start + (end - start) * alpha
            samples.append({
                "segment": segment,
                "time": segment * step_time + alpha * step_time,
                "position": position,
                "speed": float(np.linalg.norm(end - start) / step_time),
                "carrying": segment in (2, 3, 4),
            })
    return samples


def read_contacts(scene, sample, ignored: set[str], moved_name: str):
    events = []
    for contact in scene.get_contacts():
        names = [body.entity.name for body in contact.bodies]
        if any(name in ignored for name in names):
            continue
        if set(names) == {"gripper", moved_name}:
            continue
        if "gripper" in names:
            other = names[1] if names[0] == "gripper" else names[0]
            events.append(event(sample, "physx_gripper_contact", "error", f"gripper contacts {other}"))
            continue
        if sample["carrying"] and moved_name in names:
            other = names[1] if names[0] == moved_name else names[0]
            events.append(event(sample, "physx_carried_object_contact", "error", f"{moved_name} contacts {other}"))
    return events


def event(sample, kind: str, severity: str, message: str):
    return {
        "time": round(float(sample["time"]), 3),
        "segment": int(sample["segment"]),
        "kind": kind,
        "severity": severity,
        "message": message,
    }


def compress_events(events):
    grouped = {}
    order = []
    for item in events:
        key = (item["segment"], item["kind"], item["severity"], item["message"])
        if key not in grouped:
            grouped[key] = {**item, "count": 0, "first_time": item["time"], "last_time": item["time"]}
            order.append(key)
        grouped[key]["count"] += 1
        grouped[key]["last_time"] = item["time"]
    return [grouped[key] for key in order]


def print_reports(records, quiet=False):
    total = len(records)
    errors = sum(1 for record in records if record["status"] == "error")
    warnings = sum(1 for record in records if record["status"] == "warning")
    skipped_count = sum(1 for record in records if record["status"] == "skipped")
    ok = total - errors - warnings - skipped_count
    print(f"PhysX audited {total} task(s): {errors} error, {warnings} warning, {ok} ok, {skipped_count} skipped")
    if quiet:
        return
    for record in records:
        print_report(record)


def print_report(record):
    print(f"[{record['status'].upper()}] {record['task']} object={record['object']} max_speed={record['max_speed_mps']:.3f} m/s events={len(record['events'])}")
    for item in record["events"][:20]:
        span = f"t={item['first_time']:.2f}s"
        if item["count"] > 1:
            span = f"t={item['first_time']:.2f}-{item['last_time']:.2f}s x{item['count']}"
        print(f"  {span} seg={item['segment']} {item['severity']} {item['kind']}: {item['message']}")
    if len(record["events"]) > 20:
        print(f"  ... {len(record['events']) - 20} more events")


if __name__ == "__main__":
    main()
