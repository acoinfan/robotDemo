from argparse import ArgumentParser
from pathlib import Path
from shutil import copyfile
import json


def main():
    parser = ArgumentParser(description="Archive office_table layout tasks into origin/end task folders.")
    parser.add_argument("--data-dir", default="data", help="Root directory containing level_* folders.")
    parser.add_argument("--output-dir", default="data/tasks", help="Destination directory for archived tasks.")
    parser.add_argument("--overwrite", action="store_true", help="Overwrite existing archived origin/end files.")
    args = parser.parse_args()

    tasks = archive_tasks(Path(args.data_dir), Path(args.output_dir), args.overwrite)
    write_index(Path(args.output_dir), tasks)
    print(f"Archived {len(tasks)} tasks into {args.output_dir}")


def archive_tasks(data_dir: Path, output_dir: Path, overwrite: bool) -> list[dict]:
    output_dir.mkdir(parents=True, exist_ok=True)
    tasks = []

    for level_dir in sorted(data_dir.glob("level_*")):
        if not level_dir.is_dir():
            continue
        level_id = level_dir.name.removeprefix("level_")
        for table_dir in sorted(level_dir.glob("office_table_*")):
            original = table_dir / "original" / "layout.json"
            if not original.exists():
                continue
            table_id = table_dir.name.removeprefix("office_table_")
            variants = sorted(
                child for child in table_dir.iterdir()
                if child.is_dir() and child.name != "original" and (child / "layout.json").exists()
            )
            for group_id, variant_dir in enumerate(variants):
                task_name = f"level{level_id}_{table_id}_{group_id}"
                task_dir = output_dir / task_name
                task_dir.mkdir(parents=True, exist_ok=True)
                copy_layout(original, task_dir / "origin.json", overwrite)
                copy_layout(variant_dir / "layout.json", task_dir / "end.json", overwrite)

                metadata = {
                    "task": task_name,
                    "level": int(level_id),
                    "office_table": table_id,
                    "group": group_id,
                    "variant": variant_dir.name,
                    "origin": str(original),
                    "end": str(variant_dir / "layout.json"),
                }
                write_json(task_dir / "metadata.json", metadata, overwrite)
                tasks.append(metadata)

    return tasks


def copy_layout(src: Path, dst: Path, overwrite: bool):
    if dst.exists() and not overwrite:
        return
    copyfile(src, dst)


def write_json(path: Path, data, overwrite: bool = True):
    if path.exists() and not overwrite:
        return
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
        f.write("\n")


def write_index(output_dir: Path, tasks: list[dict]):
    write_json(output_dir / "index.json", {"tasks": tasks})


if __name__ == "__main__":
    main()
