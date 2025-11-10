#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
push_front_crack.py - 试管架前端推入任务

功能：将试管架从身前近处位置推入过柱机的任务流程
主要步骤包括：
1. 视觉定位获取试管架位置偏移
2. 移动到抓取准备位姿
3. 精确抓取试管架
4. 使用力控将试管架推入过柱机
5. 释放试管架并返回准备位姿

注意：crack指的是过柱机试管架，front指的是试管架在过柱机上靠近身前的位置
"""

import math
import time
import numpy as np
from copy import deepcopy
from loguru import logger

from src.client.aico2 import LabbotManagerClient
from src.util.util import safe_get_tf_shift, safe_fast_move_j_to

def push_front_crack(crack_config: dict):
    """
    试管架推入任务
    
    功能：将试管架从身前近处位置推入过柱机
    使用视觉定位和力控完成精确推入操作
    
    Args:
        crack_config: 试管架配置字典，包含视觉识别和抓取参数
        
    Returns:
        bool: 任务执行成功返回True，失败返回False
    """
    client = LabbotManagerClient()

    front_crack_shift = safe_get_tf_shift(client, crack_config, wait_before_start=2.0, build_origin_tf=False)
    if front_crack_shift is None:
        print(f"=> Get TF Shift failed! Abort!!!")
        return False

    ret = client.run_traj("demo2/push_front_crack/1_to_observe_front_tube_reversed.json")
    if ret is None or ret["code"] != 200:
        print(f"=> Run Traj failed! Abort!!!")
        return False
    ret = client.run_traj("demo2/push_front_crack/2_to_pick_pose_reversed.json")
    if ret is None or ret["code"] != 200:
        print(f"=> Run Traj failed! Abort!!!")
        return False
    ret = client.run_traj("demo2/push_front_crack/3_to_bow_ready.json")
    if ret is None or ret["code"] != 200:
        print(f"=> Run Traj failed! Abort!!!")
        return False
    client.gripper(right_position=0.08,right_force=25, wait=False)
    ret = client.run_traj("demo2/push_front_crack/4_to_bow_down.json")
    if ret is None or ret["code"] != 200:
        print(f"=> Run Traj failed! Abort!!!")
        return False
    
    
    # 忽略z轴偏差
    front_crack_shift[2] = 0
    target_position = np.array(crack_config["base_shift"])+np.array(front_crack_shift)
    logger.info(f"target_position: {target_position}")
    result = safe_fast_move_j_to(
        client=client,
        arm_name="right",
        offset_commands=f"x{target_position[0]:.4f},y{target_position[1]:.4f},z{target_position[2]:.4f}",
        traj_point_limit=26,
    )
    if result is None or result["code"] != 200:
        print(f"=> Fast Move J To failed! Abort!!!")
        return False


    grasp_config = deepcopy(crack_config["grasp_gripper_config"])
    if "left_position" in grasp_config: grasp_config["left_position"] = 0.0
    if "right_position" in grasp_config: grasp_config["right_position"] = 0.0
    client.gripper(**grasp_config)
    time.sleep(1)

    result = safe_fast_move_j_to(
        client=client,
        arm_name="right",
        offset_commands=f"z+0.003",
        traj_point_limit=6,
    )
    if result is None or result["code"] != 200:
        print(f"=> Fast Move J To failed! Abort!!!")
        return False
    
    # TODO: 这里需要根据架子位置调整
    # push_distance = 0.29
    push_distance = 0.27
    logger.info(f"push_distance: {push_distance}")
    # 这里往外拉出32cm，经过实际测试，可能是机器人平面和桌面不平齐，导致上移较多，因此这里角度加一些offset，让机器人往下拉一点
    
    # 先推入到一定距离试管架
    # rad = math.radians(20) + math.radians(0.25)
    rad = math.radians(20) + math.radians(0.28)
    push_distance_z = push_distance * math.cos(rad)
    push_distance_x = -push_distance * math.sin(rad)
    
    client.force_comp(
        arm_name="right",
        ref_frame="tcp",
        position=[push_distance_x, -0.001, push_distance_z],
        stiff_scale=[0.8, 1.0, 0.8, 1.0, 1.0, 1.0],
        vel=0.03,
        must_reach_target=True
    )
    # 再contact，因为contact速度比较慢
    rad = math.radians(20.3)
    client.contact(
        arm_name="right",
        contact_coord="tcp",
        contact_dir=[-math.sin(rad),0.002,math.cos(rad)],
        speed=0.01,
        max_contact_force=30.0,
        wait=True
    )
    time.sleep(1)
    client.gripper(right_position=0.08,right_force=50, wait=False)
    time.sleep(1.5)

    
    client.move_j_to_traj_start("demo2/push_front_crack/5_to_observe_tube_crack_reversed.json")
    ret = client.run_traj("demo2/push_front_crack/5_to_observe_tube_crack_reversed.json")
    if ret is None or ret["code"] != 200:
        print(f"=> Run Traj failed! Abort!!!")
        return False
    ret = client.run_traj("demo2/push_front_crack/6_to_bow_down_reversed.json")
    if ret is None or ret["code"] != 200:
        print(f"=> Run Traj failed! Abort!!!")
        return False
    ret = client.run_traj("demo2/push_front_crack/7_to_bow_ready_reversed.json")
    if ret is None or ret["code"] != 200:
        print(f"=> Run Traj failed! Abort!!!")
        return False

    return True
