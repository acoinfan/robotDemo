from pathlib import Path

from object import Table


def load_archived_task(data_dir: str, task: str) -> tuple[Table, Table]:
    task_dir = Path(data_dir) / task
    return (
        Table.from_json(str(task_dir / "origin.json")),
        Table.from_json(str(task_dir / "end.json")),
    )


def load_raw_task(raw_data_dir: str, level: int, office_table: str, variant: str) -> tuple[Table, Table]:
    table_dir = Path(raw_data_dir) / f"level_{level}" / f"office_table_{office_table}"
    return (
        Table.from_json(str(table_dir / "original" / "layout.json")),
        Table.from_json(str(table_dir / variant / "layout.json")),
    )


def first_archived_task(data_dir: str) -> str:
    root = Path(data_dir)
    if not root.exists():
        raise ValueError(f"No archived task directory found at {data_dir}. Run prepare_tasks.py first.")

    tasks = sorted(path.name for path in root.iterdir() if path.is_dir())
    if not tasks:
        raise ValueError(f"No archived tasks found under {data_dir}. Run prepare_tasks.py first.")
    return tasks[0]
