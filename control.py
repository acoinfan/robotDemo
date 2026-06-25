from argparse import ArgumentParser
import numpy as np
from franky import (
    Affine,
    CartesianMotion,
    CartesianPose,
    Duration,
    Gripper,
    RelativeDynamicsFactor,
    Robot,
    RobotState,
)

from object import Table
from planner import (
    DEFAULT_APPROACH_HEIGHT,
    DEFAULT_RELEASE_HEIGHT,
    PickPlacePlan,
    Pose,
    TableFrame,
    generate_pick_place_plan,
)
from task_io import first_archived_task, load_archived_task, load_raw_task


IP: str = "10.90.90.1"
DEFAULT_GRIPPER_SPEED = 0.03
DEFAULT_GRASP_FORCE = 30.0
DEFAULT_SPEED = 0.05


def main():
    parser = ArgumentParser(description="Pick the object that moved between two table scenes.")
    parser.add_argument("--ip", default=IP, help="Franka FCI hostname/IP.")
    parser.add_argument("--task", default=None, help="Archived task folder name under --data-dir.")
    parser.add_argument("--data-dir", default="data/tasks", help="Directory containing archived task folders.")
    parser.add_argument("--level", type=int, default=None, help="Level id for direct data/level_N loading.")
    parser.add_argument("--office-table", default=None, help="Office table id for direct loading, e.g. 1181.")
    parser.add_argument("--variant", default=None, help="Terminal scene folder for direct loading.")
    parser.add_argument("--raw-data-dir", default="data", help="Root directory for direct level/office_table loading.")
    parser.add_argument("--dry-run", action="store_true", help="Print the planned pick-and-place steps only.")
    parser.add_argument("--approach-height", type=float, default=DEFAULT_APPROACH_HEIGHT, help="Vertical clearance in meters.")
    parser.add_argument("--release-height", type=float, default=DEFAULT_RELEASE_HEIGHT, help="Extra height above the final object center in meters.")
    parser.add_argument("--table-origin", nargs=3, type=float, metavar=("X", "Y", "Z"), default=(0.0, 0.0, 0.0), help="Table frame origin in robot base frame, meters.")
    parser.add_argument("--table-yaw", type=float, default=0.0, help="Table frame yaw in robot base frame, radians.")
    parser.add_argument("--speed", type=float, default=DEFAULT_SPEED, help="Robot relative velocity/acceleration/jerk factor.")
    parser.add_argument("--gripper-speed", type=float, default=DEFAULT_GRIPPER_SPEED, help="Gripper speed in m/s.")
    parser.add_argument("--grasp-force", type=float, default=DEFAULT_GRASP_FORCE, help="Grasp force in N.")
    args = parser.parse_args()

    origin_table, end_table = load_task(args)
    table_frame = TableFrame.from_values(args.table_origin, args.table_yaw)
    plan = generate_pick_place_plan(
        origin_table,
        end_table,
        table_frame=table_frame,
        approach_height=args.approach_height,
        release_height=args.release_height,
    )

    print_plan(plan)
    if args.dry_run:
        return

    robot, gripper = init(args.ip, args.speed)
    execute_plan(robot, gripper, plan, args.gripper_speed, args.grasp_force)


def load_task(args) -> tuple[Table, Table]:
    if args.level is not None or args.office_table is not None or args.variant is not None:
        if args.level is None or args.office_table is None or args.variant is None:
            raise ValueError("--level, --office-table, and --variant must be provided together.")
        return load_raw_task(args.raw_data_dir, args.level, args.office_table, args.variant)

    task = args.task or first_archived_task(args.data_dir)
    return load_archived_task(args.data_dir, task)


def execute_plan(
    robot: Robot,
    gripper: Gripper,
    plan: PickPlacePlan,
    gripper_speed: float,
    grasp_force: float,
):
    motions = [make_motion(pose) for pose in plan.poses]
    robot.move(motions[0])
    robot.move(motions[1])
    if not grab(gripper, plan.grasp_width, gripper_speed, grasp_force):
        raise RuntimeError("The gripper did not report a successful grasp.")
    robot.move(motions[2])
    robot.move(motions[3])
    robot.move(motions[4])
    ungrab(gripper, gripper_speed)
    robot.move(motions[5])


def make_motion(pose: Pose) -> CartesianMotion:
    motion = CartesianMotion(Affine(pose.translation, pose.quaternion))
    motion.register_callback(cb)
    return motion


def grab(gripper: Gripper, width: float, speed: float, force: float) -> bool:
    gripper.move(min(width + 0.03, 0.08), speed)
    return gripper.grasp(width, speed, force, epsilon_outer=0.02)


def ungrab(gripper: Gripper, speed: float) -> bool:
    return gripper.open(speed)


def init(ip: str, speed: float = DEFAULT_SPEED) -> tuple[Robot, Gripper]:
    robot = Robot(ip)
    gripper = Gripper(ip)
    robot.recover_from_errors()
    robot.stop()
    robot.relative_dynamics_factor = RelativeDynamicsFactor(
        velocity=speed, acceleration=speed, jerk=speed
    )
    return robot, gripper


def print_plan(plan: PickPlacePlan):
    obj = plan.object
    target = plan.target
    print(f"Moved object: {obj.name}")
    print(f"Position delta: {plan.position_delta:.3f} cm")
    print(f"Rotation delta: {plan.rotation_delta:.3f} rad")
    print(f"Origin: {obj.position}")
    print(f"Target: {target.position}")
    print(f"Grasp width: {plan.grasp_width:.3f} m")
    for index, pose in enumerate(plan.poses, start=1):
        print(f"Step {index}: t={np.array(pose.translation)}, q={np.array(pose.quaternion)}")


def cb(
    robot_state: RobotState,
    time_step: Duration,
    rel_time: Duration,
    abs_time: Duration,
    control_signal: CartesianPose,
):
    print(f"At time {abs_time}, the target cartesian pose was {control_signal}")


if __name__ == "__main__":
    main()
