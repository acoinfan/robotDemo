from argparse import ArgumentParser
from pathlib import Path
import json

from planner import TableFrame, generate_pick_place_plan
from task_io import load_archived_task


def main():
    parser = ArgumentParser(description="Export a self-contained MesaSkill pick-place demo HTML.")
    parser.add_argument("--task", default="level1_1181_1", help="Archived task name under --data-dir.")
    parser.add_argument("--data-dir", default="data/tasks", help="Directory containing archived task folders.")
    parser.add_argument("--output", default="demo/mesa_skill_demo_standalone.html", help="Output HTML file.")
    args = parser.parse_args()

    export_demo(args.task, args.data_dir, args.output)
    print(f"Wrote {args.output}")


def export_demo(task: str, data_dir: str, output: str):
    origin_table, end_table = load_archived_task(data_dir, task)
    plan = generate_pick_place_plan(origin_table, end_table, table_frame=TableFrame.identity())
    payload = {
        "task": task,
        "origin": table_to_dict(origin_table),
        "end": table_to_dict(end_table),
        "plan": {
            "object_uid": plan.object.uid,
            "object_name": plan.object.name,
            "object_instance": plan.object.instance,
            "position_delta": plan.position_delta,
            "rotation_delta": plan.rotation_delta,
            "grasp_width": plan.grasp_width,
            "poses": [
                {
                    "translation": pose.translation.tolist(),
                    "quaternion": pose.quaternion.tolist(),
                }
                for pose in plan.poses
            ],
        },
    }

    html = Path("demo/mesa_skill_demo.html").read_text(encoding="utf-8")
    css = Path("demo/mesa_skill_demo.css").read_text(encoding="utf-8")
    js = Path("demo/mesa_skill_demo.js").read_text(encoding="utf-8")
    html = html.replace('<link rel="stylesheet" href="./mesa_skill_demo.css">', f"<style>\n{css}\n</style>")
    html = html.replace('<script src="./mesa_skill_demo.js"></script>', f"<script>\nwindow.DEMO_PAYLOAD = {json.dumps(payload)};\n{js}\n</script>")
    html = html.replace("<title>MesaSkill Pick Demo</title>", f"<title>MesaSkill Pick Demo - {task}</title>")

    output_path = Path(output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(html, encoding="utf-8")


def table_to_dict(table):
    return {
        "scene_settings": {
            "units": table.scene_units,
            "up_axis": table.scene_up_axis,
        },
        "item_placement_zone": table.boundary,
        "objects": [object_to_dict(obj) for obj in table.objects],
    }


def object_to_dict(obj):
    return {
        "name": obj.name,
        "instance": obj.instance,
        "retrieved_uid": obj.retrieved_uid,
        "selected_uid": obj.selected_uid,
        "original_size": obj.original_size,
        "scale_factor": obj.scale_factor,
        "rotation": obj.rotation,
        "z_rotation": obj.z_rotation,
        "size": obj.size,
        "position": obj.position,
    }


if __name__ == "__main__":
    main()
