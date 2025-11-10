#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
从另一个桌子拾取试管架子任务

功能：从远处桌子拾取试管架，使用双机械臂协调操作
主要步骤：
1. 移动到观察位姿
2. 获取试管架坐标系偏移量
3. 控制夹爪准备抓取
4. 规划双机械臂拾取轨迹
5. 执行抓取操作
6. 移动试管架到身体附近
"""

import os
import time
import numpy as np
import json
from copy import deepcopy
from loguru import logger
import const as Const

from aico2 import LabbotManagerClient
from src.util.util import reverse_trajectory_file, safe_get_tf_shift, safe_move_j, safe_plan_fast_move_j_to_dual_arm

def pick_crack_from_another_table(crack_config: dict):
    """
    从另一个桌子拾取试管架
    
    功能：使用双机械臂从远处桌子精确拾取试管架，包含完整的视觉定位、
    轨迹规划和执行流程

    双手运动要点:
    1. 双手实时规划耗时较久，目前采取大部分轨迹离线算好静态轨迹, 直接执行的形式, 但runtime需要根据实际偏移,
       实时计算离线轨迹末端到runtime偏移位置的一小部分动态轨迹。
    2. 以双手端起试管架为例，为了端起后能到达一个搬向靠近身体位置的静态轨迹的起点, 双手不仅需要tcp的position和quaterion都
       到达起点位置，而实际上是需要所有关节都到达起点位置，才能开始执行轨迹。
    3. 因此, 实际做法是【反向求解动态轨迹】, 即以静态轨迹起点为动态轨迹的起点(本应是终点), 根据runtime偏移量计算双手同步
       运动到试管架正上方的短程动态轨迹[1], 再计算向下放置的短程动态轨迹[2], 得到此刻的双手末端姿态, 然后只需要在当前姿态下, 双手
       各自move_j到这个姿态, 然后反向播放刚才求解的两端动态轨迹, 先[1]同步向上抬起试管架([2]的反向轨迹), 再同步微调到静态轨迹起
       点([1]的反向轨迹),接着就能衔接静态轨迹了。

    
    Args:
        crack_config: 试管架配置字典，包含坐标系、夹爪、偏移量等参数
        
    Returns:
        bool: 拾取成功返回True，失败返回False
    """
    client = LabbotManagerClient()

    ret = client.run_traj("demo2/pick_crack_remote/1_to_observe_pose.json")
    if ret is None or ret["code"] != 200:
        print(f"=> Run Traj failed! Abort!!!")
        return False

    front_crack_shift = safe_get_tf_shift(client, crack_config, wait_before_start=1.5, build_origin_tf=False)
    if front_crack_shift is None:
        print(f"=> Get Frame Shift failed! Abort!!!")
        return False

    client.gripper(**crack_config["grasp_gripper_config"])
    ret = client.run_traj("demo2/pick_crack_remote/2_to_pick_prepare.json")
    if ret is None or ret["code"] != 200:
        print(f"=> Run Traj failed! Abort!!!")
        return False


    # 忽略z轴偏移
    # front_crack_shift = [0,0,0]
    front_crack_shift[2] = 0

    # 双手端试管架轨迹末端位置与试管架标准拾取位置之间的基准偏移
    # 这是: 试管架拾取标准位置 - 双手轨迹末端位置
    traj_right_start_position = np.array(Const.DUAL_ARM_PICK_CRACK_REMOTE_END_POSE["right_arm"]["robot_tf"]["tcp"]["position"])
    traj_right_start_quaternion = np.array(Const.DUAL_ARM_PICK_CRACK_REMOTE_END_POSE["right_arm"]["robot_tf"]["tcp"]["orientation"])
    CRACK_BASE_SHIFT = (np.array(crack_config["base_shift"]) + np.array(front_crack_shift)).tolist()
    logger.info(f"dual arm start joint states: {Const.DUAL_ARM_PICK_CRACK_REMOTE_END_POSE['joint_states']}")
    dual_arm_plan_result = safe_plan_fast_move_j_to_dual_arm(
        client=client,
        offset_commands=f"x{CRACK_BASE_SHIFT[0]:.6f},y{CRACK_BASE_SHIFT[1]:.6f},z{CRACK_BASE_SHIFT[2]:.6f}",
        start_joint_states=Const.DUAL_ARM_PICK_CRACK_REMOTE_END_POSE["joint_states"],
        right_start_position=traj_right_start_position.tolist(),
        right_start_quaternion=traj_right_start_quaternion.tolist()
    )
    if isinstance(dual_arm_plan_result, dict) and dual_arm_plan_result["code"] == 200:
        traj_id = dual_arm_plan_result["traj_id"]
        os.system(f"cp {os.path.join(Const.TRAJ_HOME, traj_id)} {os.path.join(Const.TRAJ_HOME, 'tmp_dual_arm_traj_1.json')}")
        reverse_trajectory_file(
            os.path.join(Const.TRAJ_HOME, "tmp_dual_arm_traj_1.json"),
            os.path.join(Const.TRAJ_HOME, "tmp_dual_arm_traj_1_reversed.json")
        )
        logger.info(f"reverse_trajectory_file: {traj_id}")
    else:
        print(f"=> Fast Move J To Dual Arm failed! Abort!!!")
        return False
    # 获取反向轨迹的第一个joint_states
    with open(os.path.join(Const.TRAJ_HOME, "tmp_dual_arm_traj_1_reversed.json"), "r") as f:
        data = json.load(f)
        last_joint_states = data["points"][0]["positions"]
        logger.info(f"last_joint_states: {last_joint_states}")
    # 得到这个点的右手tcp位置
    new_right_start_tcp_position = traj_right_start_position + np.array(CRACK_BASE_SHIFT)
    logger.info(f"new_right_start_tcp_position: {new_right_start_tcp_position}")

    # 规划放下轨迹
    dual_arm_plan_result = safe_plan_fast_move_j_to_dual_arm(
        client=client,
        offset_commands=f"z-0.065",
        start_joint_states=list(last_joint_states),
        right_start_position=new_right_start_tcp_position.tolist(),
        right_start_quaternion=traj_right_start_quaternion.tolist()
    )
    if isinstance(dual_arm_plan_result, dict) and dual_arm_plan_result["code"] == 200:
        traj_id = dual_arm_plan_result["traj_id"]
        os.system(f"cp {os.path.join(Const.TRAJ_HOME, traj_id)} {os.path.join(Const.TRAJ_HOME, 'tmp_dual_arm_traj_2.json')}")
        reverse_trajectory_file(
            os.path.join(Const.TRAJ_HOME, "tmp_dual_arm_traj_2.json"),
            os.path.join(Const.TRAJ_HOME, "tmp_dual_arm_traj_2_reversed.json")
        )
        logger.info(f"reverse_trajectory_file: {traj_id}")
    else:
        print(f"=> Fast Move J To Dual Arm failed! Abort!!!")
        return False
    # 获取反向轨迹的第一个joint_states，这就是接下来双手需要move_j到达的joint_states
    with open(os.path.join(Const.TRAJ_HOME, "tmp_dual_arm_traj_2_reversed.json"), "r") as f:
        data = json.load(f)
        target_joint_states = data["points"][0]["positions"]
        logger.info(f"target_joint_states: {target_joint_states}")
    
    # 开始move_j !!!!!!!!!!!
    result = safe_move_j(client, target_joint_states, traj_point_limit=50)
    if isinstance(result, dict) and result["code"] == 200:
        logger.info(f"move_j success: {result}")
        traj_id = result["traj_id"][0]
        os.system(f"cp {os.path.join(Const.TRAJ_HOME, traj_id)} {os.path.join(Const.TRAJ_HOME, 'tmp_dual_arm_pick_traj.json')}")
        reverse_trajectory_file(
            os.path.join(Const.TRAJ_HOME, "tmp_dual_arm_pick_traj.json"),
            os.path.join(Const.TRAJ_HOME, "tmp_dual_arm_pick_traj_reversed.json")
        )
    else:
        logger.error(f"move_j failed: {result}")
        return False

    grasp_config = deepcopy(crack_config["grasp_gripper_config"])
    if "left_position" in grasp_config: grasp_config["left_position"] = 0.0
    if "right_position" in grasp_config: grasp_config["right_position"] = 0.0
    client.gripper(**grasp_config)
    time.sleep(2.0)

    ret = client.run_traj("tmp_dual_arm_traj_2_reversed.json")
    if ret is None or ret["code"] != 200:
        print(f"=> Run Traj failed! Abort!!!")
        return False
    ret = client.run_traj("tmp_dual_arm_traj_1_reversed.json")
    if ret is None or ret["code"] != 200:
        print(f"=> Run Traj failed! Abort!!!")
        return False
    ret = client.run_traj("demo2/pick_crack_remote/3_dual_arm_move_to_body.json")
    if ret is None or ret["code"] != 200:
        print(f"=> Run Traj failed! Abort!!!")
        return False

    return True
