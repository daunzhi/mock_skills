#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
setup_front_crack.py - 试管架前端安装任务

功能：将试管架安装到过柱机前端位置的任务流程
主要步骤包括：
1. 移动到观察位姿进行视觉定位
2. 获取基础坐标系偏移量
3. 规划双臂轨迹到安装位置上方
4. 执行精确放置操作
5. 释放试管架并返回准备位姿

注意：crack指的是过柱机试管架，front指的是试管架在过柱机上安装时靠近身前的位置
"""

import os
import time
import numpy as np
import json
from loguru import logger
import const as Const

from aico2 import LabbotManagerClient
from src.util.util import safe_get_tf_shift, safe_plan_fast_move_j_to_dual_arm

def setup_front_crack(crack_config: dict, column_machine_config: dict):
    """
    试管架前端安装任务
    
    功能：将试管架精确安装到过柱机前端位置
    使用视觉定位和双臂协调控制完成安装任务
    
    Args:
        crack_config: 试管架配置字典，包含视觉识别和抓取参数
        column_machine_config: 过柱机配置字典，包含基础坐标系参数
        
    Returns:
        bool: 任务执行成功返回True，失败返回False
    """
    client = LabbotManagerClient()

    ret = client.run_traj("demo2/setup_front_crack/0_raise_up.json")
    if ret is None or ret["code"] != 200:
        print(f"=> Run Traj failed! Abort!!!")
        return False

    if crack_config.get("origin_crack", False):
        ret = client.run_traj("demo2/setup_front_crack/1_dual_arm_to_observe_base_origin_crack.json")
    else:
        ret = client.run_traj("demo2/setup_front_crack/1_dual_arm_to_observe_base.json")
    if ret is None or ret["code"] != 200:
        print(f"=> Run Traj failed! Abort!!!")
        return False

    base_shift = safe_get_tf_shift(client, column_machine_config, wait_before_start=2.0)
    if base_shift is None:
        print(f"=> Get TF Shift failed! Abort!!!")
        return False
    logger.info(f"base_shift: {base_shift}")

    if crack_config.get("origin_crack", False):
        ret = client.run_traj("demo2/setup_front_crack/3_dual_arm_to_above_base_origin_crack.json")
    else:
        ret = client.run_traj("demo2/setup_front_crack/3_dual_arm_to_above_base.json")
    if ret is None or ret["code"] != 200:
        print(f"=> Run Traj failed! Abort!!!")
        return False
    
    # 根据偏差进行小幅度校准, 到达放置位置的上方
    # base_shift = [0, 0, 0]
    base_shift[2] = 0
    if crack_config.get("origin_crack", False):
        END_POSE = Const.DUAL_ARM_SETUP_FRONT_CRACK_ORIGIN_END_POSE
    else:
        END_POSE = Const.DUAL_ARM_SETUP_FRONT_CRACK_END_POSE
    traj_right_start_position = np.array(END_POSE["right_arm"]["robot_tf"]["tcp"]["position"])
    traj_right_start_quaternion = np.array(END_POSE["right_arm"]["robot_tf"]["tcp"]["orientation"])
    CRACK_BASE_SHIFT = np.array(crack_config["base_shift"]) + np.array(base_shift).tolist()
    # CRACK_BASE_SHIFT = np.array([0.0045, 0.002, 0.0156]) + np.array(base_shift).tolist()
    logger.info(f"dual arm start joint states: {END_POSE['joint_states']}")
    dual_arm_plan_result = safe_plan_fast_move_j_to_dual_arm(
        client=client,
        offset_commands=f"x{CRACK_BASE_SHIFT[0]:.6f},y{CRACK_BASE_SHIFT[1]:.6f},z{CRACK_BASE_SHIFT[2]:.6f}",
        start_joint_states=END_POSE["joint_states"],
        right_start_position=traj_right_start_position.tolist(),
        right_start_quaternion=traj_right_start_quaternion.tolist()
    )
    if isinstance(dual_arm_plan_result, dict) and dual_arm_plan_result["code"] == 200:
        traj_id = dual_arm_plan_result["traj_id"]
        os.system(f"cp {os.path.join(Const.TRAJ_HOME, traj_id)} {os.path.join(Const.TRAJ_HOME, 'tmp_dual_arm_setup_front_crack_traj_1.json')}")
        logger.info(f"reverse_trajectory_file: {traj_id}")
    else:
        print(f"=> Fast Move J To Dual Arm failed! Abort!!!")
        return False
    # 获取轨迹的最后一个joint_states
    with open(os.path.join(Const.TRAJ_HOME, "tmp_dual_arm_setup_front_crack_traj_1.json"), "r") as f:
        data = json.load(f)
        last_joint_states = data["points"][-1]["positions"]
        logger.info(f"last_joint_states: {last_joint_states}")
    # 得到这个点的右手tcp位置
    new_right_start_tcp_position = traj_right_start_position + np.array(CRACK_BASE_SHIFT)
    logger.info(f"new_right_start_tcp_position: {new_right_start_tcp_position}")

    # 规划放下轨迹
    dual_arm_plan_result = safe_plan_fast_move_j_to_dual_arm(
        client=client,
        offset_commands=f"z-0.04",
        start_joint_states=list(last_joint_states),
        right_start_position=new_right_start_tcp_position.tolist(),
        right_start_quaternion=traj_right_start_quaternion.tolist()
    )
    if isinstance(dual_arm_plan_result, dict) and dual_arm_plan_result["code"] == 200:
        traj_id = dual_arm_plan_result["traj_id"]
        os.system(f"cp {os.path.join(Const.TRAJ_HOME, traj_id)} {os.path.join(Const.TRAJ_HOME, 'tmp_dual_arm_setup_front_crack_traj_2.json')}")
        logger.info(f"reverse_trajectory_file: {traj_id}")
    else:
        print(f"=> Fast Move J To Dual Arm failed! Abort!!!")
        return False



    ret = client.run_traj("tmp_dual_arm_setup_front_crack_traj_1.json")
    if ret is None or ret["code"] != 200:
        print(f"=> Run Traj failed! Abort!!!")
        return False
    ret = client.run_traj("tmp_dual_arm_setup_front_crack_traj_2.json")
    if ret is None or ret["code"] != 200:
        print(f"=> Run Traj failed! Abort!!!")
        return False
        
    client.gripper(**crack_config["release_gripper_config"])
    time.sleep(2.0)


    client.move_j_to_traj_start("demo2/setup_front_crack/4_to_pick_front_ready_2_reversed.json")
    client.gripper(left_position=0.03,left_force=40,right_position=0.03,right_force=40, wait=False)
    ret = client.run_traj("demo2/setup_front_crack/4_to_pick_front_ready_2_reversed.json")
    if ret is None or ret["code"] != 200:
        print(f"=> Run Traj failed! Abort!!!")
        return False
    ret = client.run_traj("demo2/setup_front_crack/5_to_pick_front_ready_reversed.json")
    if ret is None or ret["code"] != 200:
        print(f"=> Run Traj failed! Abort!!!")
        return False

    return True