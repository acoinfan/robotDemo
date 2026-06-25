from argparse import ArgumentParser
from pathlib import Path
import json
import subprocess


def main():
    parser = ArgumentParser(description="Download only the MesaTask GLB assets needed by selected layouts.")
    parser.add_argument("--mesatask-root", default="MesaTask-10K", help="Path to the MesaTask-10K git repository.")
    parser.add_argument("--layout", action="append", default=[], help="Specific layout.json path. Can be repeated.")
    parser.add_argument("--office-table", action="append", default=[], help="Office table id, e.g. 1181 or office_table_1181. Can be repeated.")
    parser.add_argument("--task", action="append", default=[], help="Archived task name under --task-dir. Can be repeated.")
    parser.add_argument("--task-dir", default="data/tasks", help="Archived task directory used with --task.")
    parser.add_argument("--all-office-table", action="store_true", help="Use every layout under MesaTask Layout_info/office_table.")
    parser.add_argument("--dry-run", action="store_true", help="Print required GLB paths without downloading.")
    parser.add_argument("--batch-size", type=int, default=100, help="Number of GLB paths per git lfs pull call.")
    args = parser.parse_args()

    root = Path(args.mesatask_root)
    ensure_mesatask_repo(root)
    layout_paths = collect_layout_paths(args, root)
    if not layout_paths:
        raise SystemExit("No layouts selected. Use --layout, --office-table, --task, or --all-office-table.")

    glb_paths = sorted(collect_required_glbs(layout_paths, root))
    if not glb_paths:
        raise SystemExit("No selected_uid/retrieved_uid entries found in selected layouts.")

    print(f"Selected layouts: {len(layout_paths)}")
    print(f"Required GLB assets: {len(glb_paths)}")
    for path in glb_paths:
        print(path)

    if args.dry_run:
        return

    checkout_pointer_files(root, glb_paths)
    pull_lfs_files(root, glb_paths, args.batch_size)


def ensure_mesatask_repo(root: Path):
    if not root.exists():
        raise SystemExit(f"MesaTask root does not exist: {root}")
    if not (root / ".git").exists():
        raise SystemExit(f"MesaTask root is not a git repository: {root}")


def collect_layout_paths(args, root: Path) -> list[Path]:
    paths = []
    for layout in args.layout:
        path = Path(layout)
        if not path.is_absolute():
            path = Path.cwd() / path
        paths.append(path)

    for table_id in args.office_table:
        normalized = table_id if table_id.startswith("office_table_") else f"office_table_{table_id}"
        paths.append(root / "Layout_info" / "office_table" / normalized / "layout.json")

    for task in args.task:
        task_dir = Path(args.task_dir) / task
        paths.extend([task_dir / "origin.json", task_dir / "end.json"])

    if args.all_office_table:
        paths.extend(sorted((root / "Layout_info" / "office_table").glob("**/layout.json")))

    missing = [str(path) for path in paths if not path.exists()]
    if missing:
        raise SystemExit("Missing layout files:\n" + "\n".join(missing))
    return unique_paths(paths)


def unique_paths(paths):
    seen = set()
    result = []
    for path in paths:
        key = path.resolve()
        if key not in seen:
            seen.add(key)
            result.append(path)
    return result


def collect_required_glbs(layout_paths: list[Path], root: Path) -> set[str]:
    known_glbs = load_known_glbs(root)
    annotation_uids = load_annotation_uids(root)
    required = set()

    for layout_path in layout_paths:
        with open(layout_path, "r", encoding="utf-8") as f:
            data = json.load(f)
        for obj in data.get("objects", []):
            uid = obj.get("selected_uid") or obj.get("retrieved_uid")
            if not uid:
                continue
            candidates = glb_candidates(uid, known_glbs, annotation_uids)
            if not candidates:
                print(f"Warning: no GLB path found for uid {uid}")
            required.update(candidates)

    return required


def load_known_glbs(root: Path) -> set[str]:
    attributes = root / ".gitattributes"
    known = set()
    with open(attributes, "r", encoding="utf-8") as f:
        for line in f:
            path = line.split(" ", 1)[0]
            if path.startswith("Assets_library/") and path.endswith(".glb"):
                known.add(path)
    return known


def load_annotation_uids(root: Path) -> set[str]:
    annotation = root / "Asset_annotation.json"
    if not annotation.exists():
        return set()
    with open(annotation, "r", encoding="utf-8") as f:
        return set(json.load(f).keys())


def glb_candidates(uid: str, known_glbs: set[str], annotation_uids: set[str]) -> list[str]:
    exact = f"Assets_library/{uid}.glb"
    if exact in known_glbs:
        return [exact]

    variant_matches = [
        f"Assets_library/{item}.glb"
        for item in sorted(annotation_uids)
        if item.startswith(f"{uid}_") and f"Assets_library/{item}.glb" in known_glbs
    ]
    if variant_matches:
        return variant_matches

    prefix = f"Assets_library/{uid}_"
    return sorted(path for path in known_glbs if path.startswith(prefix))


def checkout_pointer_files(root: Path, glb_paths: list[str]):
    missing = [path for path in glb_paths if not (root / path).exists()]
    if not missing:
        return

    for chunk in chunks(missing, 100):
        run(["git", "checkout", "HEAD", "--", *chunk], root)


def pull_lfs_files(root: Path, glb_paths: list[str], batch_size: int):
    for index, chunk in enumerate(chunks(glb_paths, batch_size), start=1):
        include = ",".join(chunk)
        print(f"Downloading batch {index}: {len(chunk)} assets")
        run(["git", "lfs", "pull", "--include", include], root)


def run(command: list[str], cwd: Path):
    print("+", " ".join(command))
    subprocess.run(command, cwd=cwd, check=True)


def chunks(items: list[str], size: int):
    for index in range(0, len(items), size):
        yield items[index:index + size]


if __name__ == "__main__":
    main()
