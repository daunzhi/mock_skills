#!/usr/bin/env python
# -*- encoding: utf-8 -*-
import math
import time
import numpy as np
from copy import deepcopy
from loguru import logger

from src.client.aico2 import LabbotManagerClient
from src.util.util import safe_get_tf_shift, safe_fast_move_j_to

def pull_out_crack(column_machine_config: dict, crack_config: dict):
    """
    拉出试管架任务流程
    
    功能：从过柱机中安全拉出试管架到身前近处位置
    主要步骤包括视觉定位、抓取试管架、使用力控安全拉出等
    
    Args:
        column_machine_config: 过柱机配置字典，包含基础坐标系参数
        crack_config: 试管架配置字典，包含视觉识别和抓取参数
    
    Returns:
        bool: 任务执行成功返回True，失败返回False
    """
    client = LabbotManagerClient()

    base_shift = [0,0,0]
    # 步骤1：打开夹爪并观察底座位置，计算视觉偏差
    client.gripper(left_position=0.08,right_position=0.08,wait=False)
    base_shift = safe_get_tf_shift(client, column_machine_config, wait_before_start=1.5)
    if base_shift is None:
        print(f"=> Get Frame Shift failed! Abort!!!")
        return False

    ret = client.run_traj("demo2/pull_out_tube_crack/to_bow_ready.json")
    if ret is None or ret["code"] != 200:
        print(f"=> Run Traj failed! Abort!!!")
        return False

    ret = client.run_traj("demo2/pull_out_tube_crack/to_bow_down.json")
    if ret is None or ret["code"] != 200:
        print(f"=> Run Traj failed! Abort!!!")
        return False

    ret = client.run_traj("demo2/pull_out_tube_crack/to_observe_tube_crack_2.json")
    if ret is None or ret["code"] != 200:
        print(f"=> Run Traj failed! Abort!!!")
        return False

    # # observe_crack2(client)
    client.gripper(**crack_config["grasp_gripper_config"], wait=False)

    crack2_shift = safe_get_tf_shift(client, crack_config, wait_before_start=2.0, build_origin_tf=False)
    if crack2_shift is None:
        print(f"=> Get Frame Shift failed! Abort!!!")
        return False

    # 忽略z轴偏差
    crack2_shift[2] = 0
    target_position = np.array(crack_config["base_shift"])+np.array(crack2_shift)
    logger.info(f"target_position: {target_position}")

    # 检查move_j 翻转
    result = safe_fast_move_j_to(
        client=client,
        arm_name="right",
        offset_commands=f"x{target_position[0]:.4f},y{target_position[1]:.4f},z{target_position[2]:.4f}",
        traj_point_limit=40,
    )
    if result is None or result["code"] != 200:
        print(f"=> Execute Traj failed! Abort!!!")
        return False

    grasp_config = deepcopy(crack_config["grasp_gripper_config"])
    if "left_position" in grasp_config: grasp_config["left_position"] = 0.0
    if "right_position" in grasp_config: grasp_config["right_position"] = 0.0
    client.gripper(**grasp_config, wait=False)
    time.sleep(1.7)
    
    result = safe_fast_move_j_to(
        client=client,
        arm_name="right",
        offset_commands=f"z+0.003",
        traj_point_limit=6,
    )
    if result is None or result["code"] != 200:
        print(f"=> Execute Traj failed! Abort!!!")
        return False

    rad = math.radians(20)
    # 计算需要拉出的距离
    # base_shift = [0,0,0]
    # crack2_shift = [-0.002972606639267683, -0.0006917950498951525, 0.0005078125599848882]
    # crack2_shift = [0,0,0]

    pull_distance_shift = - base_shift[0] + crack2_shift[0]
    logger.info(f"pull_distance_shift: {pull_distance_shift}")
    if abs(pull_distance_shift) > 0.1:
        logger.info(f"pull_distance_shift too large: {pull_distance_shift}")
        return False
    pull_distance = pull_distance_shift + 0.31
    logger.info(f"pull_distance: {pull_distance}")
    # 这里往外拉出32cm，经过实际测试，可能是机器人平面和桌面不平齐，导致上移较多，因此这里角度加一些offset，让机器人往下拉一点
    rad_shift = math.atan(0.014/0.32)

    # 拉出试管架
    rad = math.radians(20) + rad_shift
    pull_distance_z = -pull_distance * math.cos(rad)
    pull_distance_x = pull_distance * math.sin(rad)
    client.force_comp(
        arm_name="right",
        ref_frame="tcp",
        position=[pull_distance_x, -0.002, pull_distance_z],
        stiff_scale=[1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
        vel=0.05
    )
    client.gripper(right_position=0.06,right_force=50, wait=False)
    result = safe_fast_move_j_to(
        client=client,
        arm_name="right",
        offset_commands=f"x-0.05",
        traj_point_limit=60,
    )
    if result is None or result["code"] != 200:
        print(f"=> Execute Traj failed! Abort!!!")
        return False
    client.move_j_to_traj_start("demo2/pull_out_tube_crack/to_bow_down_reversed.json")
    ret = client.run_traj("demo2/pull_out_tube_crack/to_bow_down_reversed.json")
    if ret is None or ret["code"] != 200:
        print(f"=> Run Traj failed! Abort!!!")
        return False
    ret = client.run_traj("demo2/pull_out_tube_crack/to_bow_ready_reversed.json")
    if ret is None or ret["code"] != 200:
        print(f"=> Run Traj failed! Abort!!!")
        return False

    return True