from argparse import ArgumentParser
from dataclasses import dataclass
from pathlib import Path
import json

import numpy as np

from object import Object
from planner import CM_TO_M, TableFrame, generate_pick_place_plan
from task_io import load_archived_task, load_raw_task


DEFAULT_STEP_TIME = 2.0
DEFAULT_SAMPLE_DT = 0.05
DEFAULT_MAX_SPEED = 0.25
DEFAULT_CLEARANCE = 0.005
DEFAULT_GRIPPER_HALF_XY = 0.04
DEFAULT_GRIPPER_HALF_Z = 0.012


@dataclass
class Box:
    uid: str
    name: str
    center: np.ndarray
    half: np.ndarray


def main():
    parser = ArgumentParser(description="Audit pick-place trajectory for collisions, penetration, bounds, and speed.")
    parser.add_argument("--task", default=None, help="Archived task name under --data-dir.")
    parser.add_argument("--data-dir", default="data/tasks")
    parser.add_argument("--level", type=int, default=None)
    parser.add_argument("--office-table", default=None)
    parser.add_argument("--variant", default=None)
    parser.add_argument("--raw-data-dir", default="data")
    parser.add_argument("--all", action="store_true", help="Audit all archived tasks under --data-dir.")
    parser.add_argument("--output", default=None, help="Optional JSON log output path.")
    parser.add_argument("--quiet", action="store_true", help="Only print the summary line.")
    parser.add_argument("--limit", type=int, default=12, help="Maximum events to print per task.")
    parser.add_argument("--step-time", type=float, default=DEFAULT_STEP_TIME, help="Seconds per planner segment.")
    parser.add_argument("--sample-dt", type=float, default=DEFAULT_SAMPLE_DT, help="Sampling interval in seconds.")
    parser.add_argument("--max-speed", type=float, default=DEFAULT_MAX_SPEED, help="Speed threshold in m/s.")
    parser.add_argument("--clearance", type=float, default=DEFAULT_CLEARANCE, help="Required clearance from obstacles/table in meters.")
    parser.add_argument("--gripper-half-xy", type=float, default=DEFAULT_GRIPPER_HALF_XY, help="Gripper horizontal half extent in meters.")
    parser.add_argument("--gripper-half-z", type=float, default=DEFAULT_GRIPPER_HALF_Z, help="Gripper vertical half extent in meters.")
    args = parser.parse_args()

    records = audit_many(args)
    print_report(records, quiet=args.quiet, limit=args.limit)
    if args.output:
        Path(args.output).parent.mkdir(parents=True, exist_ok=True)
        with open(args.output, "w", encoding="utf-8") as f:
            json.dump(records, f, indent=2)
            f.write("\n")


def audit_many(args) -> list[dict]:
    if args.all:
        tasks = sorted(path.name for path in Path(args.data_dir).iterdir() if path.is_dir())
        return [audit_archived_task(task, args) for task in tasks]

    if args.level is not None or args.office_table is not None or args.variant is not None:
        if args.level is None or args.office_table is None or args.variant is None:
            raise SystemExit("--level, --office-table, and --variant must be provided together.")
        origin, end = load_raw_task(args.raw_data_dir, args.level, args.office_table, args.variant)
        name = f"level{args.level}_{args.office_table}_{args.variant}"
        return [audit_task(name, origin, end, args)]

    task = args.task or first_task(args.data_dir)
    return [audit_archived_task(task, args)]


def first_task(data_dir: str) -> str:
    tasks = sorted(path.name for path in Path(data_dir).iterdir() if path.is_dir())
    if not tasks:
        raise SystemExit(f"No tasks found under {data_dir}")
    return tasks[0]


def audit_archived_task(task: str, args) -> dict:
    origin, end = load_archived_task(args.data_dir, task)
    return audit_task(task, origin, end, args)


def audit_task(task_name, origin_table, end_table, args) -> dict:
    try:
        plan = generate_pick_place_plan(origin_table, end_table, table_frame=TableFrame.identity())
    except ValueError as exc:
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
    boxes = [object_box(obj) for obj in origin_table.objects if obj.uid != plan.object.uid]
    carried = object_box(plan.object)
    table_bounds = np.array(origin_table.boundary, dtype=float) * CM_TO_M
    samples = sample_plan(plan, args.step_time, args.sample_dt)
    events = []

    for sample in samples:
        gripper_box = gripper_aabb(sample["position"], args.gripper_half_xy, args.gripper_half_z)
        moving_box = carried_at(carried, sample["position"]) if sample["carrying"] else object_box(plan.object)
        check_table(sample, gripper_box, moving_box, events, args.clearance)
        check_bounds(sample, moving_box, table_bounds, events, args.clearance)
        check_obstacles(sample, gripper_box, moving_box, boxes, events, args.clearance)

    max_speed = max((sample["speed"] for sample in samples), default=0.0)
    for sample in samples:
        if sample["speed"] > args.max_speed:
            events.append(event(sample, "speed", "warning", f"speed {sample['speed']:.3f} m/s exceeds {args.max_speed:.3f} m/s"))

    events = compress_events(events)
    severity = "ok"
    if any(item["severity"] == "error" for item in events):
        severity = "error"
    elif events:
        severity = "warning"

    return {
        "task": task_name,
        "status": severity,
        "object": plan.object.instance,
        "target": plan.target.instance,
        "position_delta_cm": plan.position_delta,
        "max_speed_mps": max_speed,
        "events": events,
    }


def object_box(obj: Object) -> Box:
    center = np.array(obj.position, dtype=float) * CM_TO_M
    half = np.maximum(np.array(obj.size, dtype=float) * CM_TO_M / 2, 0.003)
    return Box(obj.uid, obj.instance, center, half)


def gripper_aabb(center, half_xy: float, half_z: float) -> Box:
    return Box("gripper", "gripper", np.array(center, dtype=float), np.array([half_xy, half_xy, half_z], dtype=float))


def carried_at(box: Box, gripper_position) -> Box:
    return Box(box.uid, box.name, np.array(gripper_position, dtype=float), box.half)


def sample_plan(plan, step_time: float, sample_dt: float) -> list[dict]:
    samples = []
    poses = [pose.translation for pose in plan.poses]
    steps = max(2, int(np.ceil(step_time / sample_dt)) + 1)

    for segment in range(len(poses) - 1):
        start = poses[segment]
        end = poses[segment + 1]
        for i in range(steps):
            if segment > 0 and i == 0:
                continue
            alpha = i / (steps - 1)
            position = start + (end - start) * alpha
            speed = float(np.linalg.norm(end - start) / step_time)
            samples.append({
                "segment": segment,
                "time": segment * step_time + alpha * step_time,
                "position": position,
                "speed": speed,
                "carrying": segment in (2, 3, 4),
            })
    return samples


def check_table(sample, gripper: Box, moving: Box, events: list, clearance: float):
    boxes = [gripper]
    if sample["carrying"]:
        boxes.append(moving)
    for box in boxes:
        bottom = box.center[2] - box.half[2]
        if bottom < -clearance:
            events.append(event(sample, "table_penetration", "error", f"{box.name} bottom {bottom:.3f} m below table"))
        elif bottom < clearance:
            events.append(event(sample, "low_clearance", "warning", f"{box.name} bottom clearance {bottom:.3f} m"))


def check_bounds(sample, moving: Box, bounds, events: list, clearance: float):
    min_x, max_x, min_y, max_y = bounds
    low = moving.center - moving.half
    high = moving.center + moving.half
    if low[0] < min_x - clearance or high[0] > max_x + clearance or low[1] < min_y - clearance or high[1] > max_y + clearance:
        events.append(event(sample, "bounds", "warning", f"{moving.name} outside table placement boundary"))


def check_obstacles(sample, gripper: Box, moving: Box, obstacles: list[Box], events: list, clearance: float):
    for obstacle in obstacles:
        if intersects(gripper, obstacle, clearance):
            events.append(event(sample, "gripper_collision", "error", f"gripper intersects {obstacle.name}"))
        if sample["carrying"] and intersects(moving, obstacle, clearance):
            events.append(event(sample, "carried_object_collision", "error", f"{moving.name} intersects {obstacle.name}"))


def intersects(a: Box, b: Box, padding: float) -> bool:
    return bool(np.all(np.abs(a.center - b.center) <= (a.half + b.half + padding)))


def event(sample, kind: str, severity: str, message: str) -> dict:
    return {
        "time": round(float(sample["time"]), 3),
        "segment": int(sample["segment"]),
        "kind": kind,
        "severity": severity,
        "message": message,
    }


def compress_events(events: list[dict]) -> list[dict]:
    grouped = {}
    order = []
    for item in events:
        key = (item["segment"], item["kind"], item["severity"], item["message"])
        if key not in grouped:
            grouped[key] = {**item, "count": 0, "first_time": item["time"], "last_time": item["time"]}
            order.append(key)
        grouped[key]["count"] += 1
        grouped[key]["last_time"] = item["time"]

    compressed = []
    for key in order:
        item = grouped[key]
        item["time"] = item["first_time"]
        compressed.append(item)
    return compressed


def print_report(records: list[dict], quiet: bool = False, limit: int = 12):
    total = len(records)
    errors = sum(1 for record in records if record["status"] == "error")
    warnings = sum(1 for record in records if record["status"] == "warning")
    skipped = sum(1 for record in records if record["status"] == "skipped")
    ok = total - errors - warnings - skipped
    print(f"Audited {total} task(s): {errors} error, {warnings} warning, {ok} ok, {skipped} skipped")
    if quiet:
        return
    for record in records:
        print(f"[{record['status'].upper()}] {record['task']} object={record['object']} max_speed={record['max_speed_mps']:.3f} m/s events={len(record['events'])}")
        for item in record["events"][:limit]:
            span = f"t={item['first_time']:.2f}s"
            if item["count"] > 1:
                span = f"t={item['first_time']:.2f}-{item['last_time']:.2f}s x{item['count']}"
            print(f"  {span} seg={item['segment']} {item['severity']} {item['kind']}: {item['message']}")
        if len(record["events"]) > limit:
            print(f"  ... {len(record['events']) - limit} more events")


if __name__ == "__main__":
    main()
