#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
put_crack_on_another_table.py - 试管架放置到其他台面任务

功能：将试管架放置到另一个台面的任务流程
主要步骤包括：
1. 抬起试管架
2. 移动到目标台面位置
3. 放下试管架
4. 释放试管架
5. 收回机械臂

注意：crack指的是过柱机试管架，此任务用于将试管架放置到其他工作台面
"""

import time

from src.client.aico2 import LabbotManagerClient

def put_crack_on_another_table(crack_config: dict):
    """
    试管架放置到其他台面任务
    
    功能：将试管架放置到另一个工作台面
    使用预定义轨迹完成放置操作
    
    Args:
        crack_config: 试管架配置字典，包含释放抓取参数
        
    Returns:
        bool: 任务执行成功返回True，失败返回False
    """
    client = LabbotManagerClient()
    ret = client.run_traj("demo2/put_back_crack/1_raise_up.json")
    if ret is None or ret["code"] != 200:
        print(f"=> Run Traj failed! Abort!!!")
        return False
    ret = client.run_traj("demo2/put_back_crack/2_move_forward.json")
    if ret is None or ret["code"] != 200:
        print(f"=> Run Traj failed! Abort!!!")
        return False
    if crack_config.get("origin_crack", False):
        ret = client.run_traj("demo2/put_back_crack/3_put_down_origin_crack.json")
    else:
        ret = client.run_traj("demo2/put_back_crack/3_put_down.json")
    if ret is None or ret["code"] != 200:
        print(f"=> Run Traj failed! Abort!!!")
        return False
    client.gripper(**crack_config["release_remote_gripper_config"])
    time.sleep(2)
    if crack_config.get("origin_crack", False):
        ret = client.run_traj("demo2/put_back_crack/4_release_hands_origin_crack.json")
    else:
        ret = client.run_traj("demo2/put_back_crack/4_release_hands.json")
    if ret is None or ret["code"] != 200:
        print(f"=> Run Traj failed! Abort!!!")
        return False
    ret = client.run_traj("demo2/put_back_crack/5_widthdraw_hands.json")
    if ret is None or ret["code"] != 200:
        print(f"=> Run Traj failed! Abort!!!")
        return False

    return True
