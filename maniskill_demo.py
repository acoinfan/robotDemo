from argparse import ArgumentParser

from mesatask_3d_demo import export_3d_demo
from sapien_offscreen_demo import main as sapien_main


def main():
    parser = ArgumentParser(description="MesaTask office_table 3D demo entrypoint.")
    parser.add_argument("--task", default="level1_1181_1", help="Archived task name under --data-dir.")
    parser.add_argument("--data-dir", default="data/tasks", help="Directory containing archived task folders.")
    parser.add_argument("--mesatask-root", default="MesaTask-10K", help="Path to MesaTask-10K.")
    parser.add_argument("--html-output", default="demo/mesatask_3d_demo.html", help="Output interactive 3D HTML.")
    parser.add_argument("--offscreen", action="store_true", help="Also render one SAPIEN offscreen PNG.")
    args = parser.parse_args()

    export_3d_demo(args.task, args.data_dir, args.mesatask_root, args.html_output)
    print(f"Wrote interactive 3D demo to {args.html_output}")

    if args.offscreen:
        import sys

        old_argv = sys.argv
        sys.argv = [
            "sapien_offscreen_demo.py",
            "--task",
            args.task,
            "--data-dir",
            args.data_dir,
        ]
        try:
            sapien_main()
        finally:
            sys.argv = old_argv


if __name__ == "__main__":
    main()
