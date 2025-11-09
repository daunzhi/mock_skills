#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
@File    :   util.py
@Time    :   2025/11/10 00:46:22
@Author  :   LUMR
@Version :   1.0
@Desc    :   
'''

import json
import os
import time
import numpy as np

from loguru import logger

# 轨迹文件存储目录（需要根据实际配置设置）
TRAJ_HOME = "/path/to/trajectory/home"  # 轨迹文件存储目录路径

def agv_move(*args, **kwargs):
    """无具体实现, 仅做示意"""
    return True

def reverse_trajectory_file(*args, **kwargs):
    """无具体实现, 仅做示意"""
    return True

def safe_get_tf_shift(client: LabbotManagerClient, config: dict, retry_times: int = 3, wait_before_start: float = 1.5, build_origin_tf: bool = False):
    """
    安全获取坐标系偏移量
    
    功能：通过视觉识别获取运行时坐标系相对于参考坐标系的偏移量，并进行安全检查
    
    Args:
        client: LabbotManagerClient实例
        config: 坐标系配置字典，包含tf_name、marker_id等参数
        retry_times: 重试次数，默认3次
        wait_before_start: 开始前的等待时间，默认1.5秒
        build_origin_tf: 是否先构建原始坐标系，默认False
        
    Returns:
        list: 三维偏移量 [x, y, z]，如果失败返回None
    """
    time.sleep(wait_before_start)
    
    # 如果需要，先构建原始参考坐标系
    if build_origin_tf:
        client.create_frame(
            frame_name=config["tf_name"],
            marker_id=config["marker_id"],
            expected_count=config["expected_count"],
            arms=config["arms"]
        )

    history_shifts = []
    for i in range(retry_times):
        # 创建运行时坐标系（基于当前视觉观测）
        success = client.create_frame(
            frame_name=config["tf_name"]+"_runtime",
            marker_id=config["marker_id"],
            expected_count=config["expected_count"],
            arms=config["arms"]
        )
        if not success:
            print(f"=> Create Frame failed! Abort!!!")
            continue
        
        # 获取运行时坐标系相对于参考坐标系的偏移量
        result = client.get_frame_offset(
            target_frame=config["tf_name"]+"_runtime",
            ref_frame=config["tf_name"],
        )
        runtime_shift = result["position_offset"]
        history_shifts.append(runtime_shift)
        logger.info(f"{config['tf_name']}_shift: {runtime_shift}")
        
        # 检查偏移量是否在允许范围内
        over_limit = False
        for i in range(3):
            if abs(runtime_shift[i]) > config["runtime_shift_limits"][i]:
                print(f"=> {config['tf_name']}_shift[{i}] is out of range! Abort!!!")
                over_limit = True
        if over_limit:
            continue
            
        return runtime_shift
    
    # 如果所有尝试都失败，计算平均偏移量（如果标准差较小）
    history_shifts = np.array(history_shifts)
    mean_shifts = history_shifts.mean(axis=0)
    std = history_shifts.std(axis=0)
    if sum(std) < 0.01:
        return mean_shifts.tolist()
    
    print(f"=> Get Frame Shift failed after {retry_times} times! Abort!!!")
    return None


def safe_fast_move_j_to(client, arm_name, offset_commands, traj_point_limit: int, max_retry_times: int=5):
    """
    安全快速关节空间移动（带轨迹长度检查）
    
    功能：执行快速关节空间移动，但先检查规划轨迹的点数是否超过限制
    避免机器人执行大翻转或异常轨迹，提高运动安全性
    
    Args:
        client: LabbotManagerClient实例
        arm_name: 机械臂名称（"left"或"right"）
        offset_commands: 偏移命令字符串，格式为"x,y,z,rx,ry,rz"
        traj_point_limit: 轨迹点数量限制，超过此值将重新规划
        max_retry_times: 最大重试次数，默认5次
    
    Returns:
        dict: 运动结果信息，包含轨迹ID等信息
        None: 如果运动失败
    """
    # 检查move_j 翻转
    success = False
    for _ in range(max_retry_times):
        # 规划轨迹但先不执行
        result = client.fast_move_j_to(
            arm_name=arm_name,
            offset_commands=offset_commands,
            execute=False
        )
        if result is None or result["code"] != 200:
            print(f"=> Fast Move J To failed! Skip!")
            continue
        traj_id = result["traj_id"]
        logger.info(f"traj_id: {traj_id}")
        # 检查轨迹点数量是否超过预期
        with open(os.path.join(Const.TRAJ_HOME, traj_id), 'r') as f:
            traj_data = json.load(f)
        traj_length = len(traj_data["points"])
        logger.info(f"traj_length: {traj_length}")
        if traj_length > traj_point_limit:
            print(f"=> Traj point limit exceeded! Skip!!!")
            continue

        # 检查通过，运行轨迹
        ret = client.run_traj(traj_id)
        if ret is None or ret["code"] != 200:
            print(f"=> Run Traj failed! Abort!!!")
            return False
        success = True
        break
    if success:
        return result
    return None


def safe_move_j(client, target_joint_states, traj_point_limit: int, max_retry_times: int=5):
    """
    安全关节空间移动（带轨迹长度检查）
    
    功能：执行关节空间移动，但先检查规划轨迹的点数是否超过限制
    避免机器人执行大翻转或异常轨迹，支持双机械臂和身体关节
    
    Args:
        client: LabbotManagerClient实例
        target_joint_states: 目标关节状态列表，格式为[body1, body2, left1-7, right1-7]
        traj_point_limit: 轨迹点数量限制，超过此值将重新规划
        max_retry_times: 最大重试次数，默认5次
    
    Returns:
        dict: 运动结果信息，包含轨迹ID等信息
        None: 如果运动失败
    """
    # 检查move_j 翻转
    success = False
    for _ in range(max_retry_times):
        # 规划轨迹但先不执行
        
        result = client.move_j(
            body_positions=target_joint_states[:2],
            left_positions=target_joint_states[2:9],
            right_positions=target_joint_states[9:],
            execute=False
        )
        if result is None or result["code"] != 200:
            print(f"=> Fast Move J To failed! Skip!")
            continue
        traj_id = result["traj_id"]
        logger.info(f"traj_id: {traj_id}")
        # 检查轨迹点数量是否超过预期
        if isinstance(traj_id, list):
            traj_id = traj_id[0]
        with open(os.path.join(TRAJ_HOME, traj_id), 'r') as f:
            traj_data = json.load(f)
        traj_length = len(traj_data["points"])
        logger.info(f"traj_length: {traj_length}")
        if traj_length > traj_point_limit:
            print(f"=> Traj point limit exceeded! Skip!!!")
            continue

        # 检查通过，运行轨迹
        ret = client.run_traj(traj_id)
        if ret is None or ret["code"] != 200:
            print(f"=> Run Traj failed! Abort!!!")
            return False
        success = True
        break
    if success:
        return result
    return None

def safe_plan_fast_move_j_to_dual_arm(
    client,
    offset_commands: str,
    start_joint_states: list,
    right_start_position: list,
    right_start_quaternion: list,
    max_retry_times: int=5
):
    """
    安全规划双机械臂快速关节空间移动（仅规划不执行）
    
    功能：为双机械臂规划快速关节空间移动轨迹，提供重试机制确保规划成功
    主要用于需要双机械臂协调运动的复杂任务场景
    
    Args:
        client: LabbotManagerClient实例
        offset_commands: 偏移命令字符串，格式为"x,y,z"
        start_joint_states: 起始关节状态列表，包含所有关节的初始位置
        right_start_position: 右机械臂起始位置 [x, y, z]
        right_start_quaternion: 右机械臂起始四元数 [x, y, z, w]
        max_retry_times: 最大重试次数，默认5次
    
    Returns:
        dict: 规划结果信息，包含轨迹ID等信息
        None: 如果规划失败
    """
    result = None
    for _ in range(max_retry_times):
        result = client.fast_move_j_to_dual_arm(
            offset_commands=offset_commands,
            execute=False,
            start_joint_states=start_joint_states,
            right_start_position=right_start_position,
            right_start_quaternion=right_start_quaternion
        )
        if result is None or result["code"] != 200:
            print(f"=> Fast Move J To Dual Arm failed! Tray again!!!")
            continue
        break
    return result