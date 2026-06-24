from argparse import ArgumentParser

import time, math
from scipy.spatial.transform import Rotation
from franky import *
from object import Object, Table
IP:str = "10.90.90.1"
def main():
    robot = init(IP)
    origin_table = Table.from_json("./data/0001/origin.json")
    end_table = Table.from_json("./data/0001/end.json")
    routes = generate_route(origin_table, end_table)
    
    grab()
    for route in routes:
        robot.move(route)
    
    
    
# 基于数据得到对应运动路径(不考虑避障),按照先后顺序存放列表
# 返回的第一个值让机器人运动到origin需要移动的位置，此时grab
# 第二个值就是机器人运动到结果的位置，此时放下
def generate_route(origin_table: Table, end_table: Table) -> list[CartesianMotion] :
    origin_map = {obj.uid: obj for obj in origin_table.objects}
    end_map    = {obj.uid: obj for obj in end_table.objects}
    
    moved_uid = []
    
    pass

# 抓取物品
def grab() :
    pass

# 设置机械臂基本数据
def init(IP: str) -> Robot:
    robot = Robot(IP)
    robot.recover_from_errors()
    # 控制速度上限1%
    robot.relative_dynamics_factor = RelativeDynamicsFactor(
        velocity=0.01, acceleration=0.01, jerk=0.01
    )
    return robot

# 回调函数
def cb(
        robot_state: RobotState,
        time_step: Duration,
        rel_time: Duration,
        abs_time: Duration,
        control_signal: JointPositions,
    ):
    print(f"At time {abs_time}, the target joint positions were {control_signal.q}")


if __name__ == "__main__":
    main()
    