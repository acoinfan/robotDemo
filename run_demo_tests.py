from argparse import ArgumentParser
from pathlib import Path
import json
import shutil

from export_demo import export_demo
from maniskill_collision_audit import audit_task, skipped
from mesatask_3d_demo import export_3d_demo
from planner import TableFrame, generate_pick_place_plan
from task_io import load_archived_task


def main():
    parser = ArgumentParser(description="Generate demos and PhysX collision reports for archived tasks.")
    parser.add_argument("--task", action="append", default=[], help="Task name under --data-dir. Can be repeated.")
    parser.add_argument("--all", action="store_true", help="Process every task under --data-dir.")
    parser.add_argument("--data-dir", default="data/tasks")
    parser.add_argument("--output-dir", default="demo/tasks")
    parser.add_argument("--mesatask-root", default="MesaTask-10K")
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--skip-2d", action="store_true")
    parser.add_argument("--skip-3d", action="store_true")
    parser.add_argument("--skip-collision", action="store_true")
    args = parser.parse_args()

    tasks = select_tasks(args)
    output_root = Path(args.output_dir)
    output_root.mkdir(parents=True, exist_ok=True)

    summary = []
    for index, task in enumerate(tasks, start=1):
        print(f"[{index}/{len(tasks)}] {task}")
        summary.append(process_task(task, args))

    write_json(output_root / "summary.json", summary)
    print_summary(summary, output_root)


def select_tasks(args) -> list[str]:
    if args.all:
        return sorted(path.name for path in Path(args.data_dir).iterdir() if path.is_dir())
    if args.task:
        return args.task
    return [first_task(args.data_dir)]


def first_task(data_dir: str) -> str:
    tasks = sorted(path.name for path in Path(data_dir).iterdir() if path.is_dir())
    if not tasks:
        raise SystemExit(f"No tasks found under {data_dir}. Run prepare_tasks.py first.")
    return tasks[0]


def process_task(task: str, args) -> dict:
    task_dir = Path(args.output_dir) / task
    if task_dir.exists() and args.overwrite:
        shutil.rmtree(task_dir)
    task_dir.mkdir(parents=True, exist_ok=True)

    origin, end = load_archived_task(args.data_dir, task)
    record = {
        "task": task,
        "output_dir": str(task_dir),
        "status": "ok",
        "files": {},
        "errors": [],
    }

    try:
        plan = generate_pick_place_plan(origin, end, table_frame=TableFrame.identity())
        write_plan(task_dir / "trajectory.json", task, plan)
        record["files"]["trajectory"] = "trajectory.json"
        record["object"] = plan.object.instance
        record["target"] = plan.target.instance
    except Exception as exc:
        write_json(task_dir / "error.json", {"task": task, "error": str(exc)})
        record["status"] = "skipped"
        record["errors"].append(str(exc))
        return record

    if not args.skip_2d:
        run_step(record, "demo_2d", "demo_2d.html", lambda path: export_demo(task, args.data_dir, str(path)), task_dir)

    if not args.skip_3d:
        run_step(record, "demo_3d", "demo_3d.html", lambda path: export_3d_demo(task, args.data_dir, args.mesatask_root, str(path)), task_dir)

    if not args.skip_collision:
        collision_path = task_dir / "collision_report.json"
        try:
            audit_args = AuditArgs(args.data_dir)
            collision = audit_task(task, origin, end, audit_args)
        except Exception as exc:
            collision = skipped(task, exc)
            record["errors"].append(str(exc))
        write_json(collision_path, collision)
        record["files"]["collision_report"] = "collision_report.json"
        record["collision_status"] = collision["status"]
        if collision["status"] == "error":
            record["status"] = "collision_error"
        elif collision["status"] == "warning" and record["status"] == "ok":
            record["status"] = "collision_warning"

    return record


class AuditArgs:
    def __init__(self, data_dir: str):
        self.data_dir = data_dir
        self.step_time = 2.0
        self.dt = 1 / 60
        self.max_speed = 0.25
        self.gripper_half_xy = 0.04
        self.gripper_half_z = 0.012


def run_step(record: dict, key: str, filename: str, fn, task_dir: Path):
    path = task_dir / filename
    try:
        fn(path)
        record["files"][key] = filename
    except Exception as exc:
        record["errors"].append(f"{key}: {exc}")


def write_plan(path: Path, task: str, plan):
    data = {
        "task": task,
        "object": plan.object.instance,
        "target": plan.target.instance,
        "position_delta_cm": plan.position_delta,
        "rotation_delta_rad": plan.rotation_delta,
        "grasp_width_m": plan.grasp_width,
        "poses": [
            {
                "translation": pose.translation.tolist(),
                "quaternion": pose.quaternion.tolist(),
            }
            for pose in plan.poses
        ],
    }
    write_json(path, data)


def write_json(path: Path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
        f.write("\n")


def print_summary(summary: list[dict], output_root: Path):
    total = len(summary)
    collision_errors = sum(1 for item in summary if item.get("collision_status") == "error")
    collision_warnings = sum(1 for item in summary if item.get("collision_status") == "warning")
    skipped_count = sum(1 for item in summary if item["status"] == "skipped")
    print(
        f"Wrote {total} task report(s) to {output_root}. "
        f"collision_error={collision_errors}, collision_warning={collision_warnings}, skipped={skipped_count}"
    )


if __name__ == "__main__":
    main()
