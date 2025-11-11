#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
@File    :   aico2.py
@Time    :   2025/10/29 11:18:24
@Author  :   Zhangzhongwen
@Version :   1.0
@Desc    :   精简版机器人管理客户端，去除冗余边界检查
'''

import re
import requests
import json
import math
import fire
import os
import glob
from datetime import datetime, timedelta
from pathlib import Path
from labbot_manager_base import LabbotManagerClientBase

class LabbotManagerClient(LabbotManagerClientBase):
    """精简版机器人管理客户端，基于基类添加更多功能"""
    
    def __init__(self):
        super().__init__()
    
    def fast_move_j(self, cmds, speed=0.3, acc=0.3, execute=True, wait=True):
        """快速控制多个关节的角度变化（增量模式）
        
        Args:
            cmds: 关节命令字符串，格式如 "b1:+5,l1:-5,r1:-5"
            speed: 运动速度 (0.0-1.0)
            acc: 运动加速度 (0.0-1.0)
            execute: 是否执行运动
            wait: 是否等待执行完成
        
        Returns:
            bool: 成功返回True，失败返回False
        """
        joint_mapping = {
            'b1': ('body', 0), 'b2': ('body', 1),
            'l1': ('left', 0), 'l2': ('left', 1), 'l3': ('left', 2), 'l4': ('left', 3),
            'l5': ('left', 4), 'l6': ('left', 5), 'l7': ('left', 6),
            'r1': ('right', 0), 'r2': ('right', 1), 'r3': ('right', 2), 'r4': ('right', 3),
            'r5': ('right', 4), 'r6': ('right', 5), 'r7': ('right', 6)
        }
        
        try:
            arm_increments = {}
            for cmd_pair in cmds.split(','):
                joint_name, cmd = cmd_pair.strip().split(':', 1)
                increment_degrees = float(cmd.strip())
                
                arm_name, joint_index = joint_mapping[joint_name.strip()]
                increment_radians = math.radians(increment_degrees)
                
                if arm_name not in arm_increments:
                    arm_increments[arm_name] = [0.0, 0.0] if arm_name == 'body' else [0.0] * 7
                arm_increments[arm_name][joint_index] = increment_radians
            
            arm_requests = []
            for arm_name, increments in arm_increments.items():
                arm_requests.append({"arm_name": arm_name, "increment_joint_positions": increments})
            
            response = requests.post(
                f"{self.server_url}/move_j",
                json={
                    "arm_requests": arm_requests,
                    "speed": speed,
                    "acc": acc,
                    "need_traj": True,
                    "wait": wait,
                    "execute": execute
                },
                headers={"Content-Type": "application/json"},
                timeout=60
            )
            return response.status_code == 200
        except Exception:
            return False

    def fast_move_j_to(self, arm_name, offset_commands, speed=0.1, acc=0.1, execute=True, wait=True):
        """快速末端位置控制
        
        Args:
            arm_name: 手臂名称，"left" 或 "right"
            offset_commands: 偏移命令字符串，格式如 "x+0.1" 或 "x+0.05,y-0.02"
            speed: 运动速度 (0.0-1.0)
            acc: 运动加速度 (0.0-1.0)
            execute: 是否执行运动
            wait: 是否等待执行完成
        
        Returns:
            bool: 成功返回True，失败返回False
        """
        try:
            # 获取当前状态
            response = requests.post(
                f"{self.server_url}/get_robot_status",
                json={"arm": arm_name},
                headers={"Content-Type": "application/json"},
                timeout=30
            )
            
            if response.status_code != 200:
                return False
            
            result = response.json()
            if result.get('code') != 200:
                return False
            
            # 获取末端执行器位姿
            arm_data = result.get('left_arm' if arm_name == 'left' else 'right_arm', {})
            robot_tf = arm_data.get('robot_tf', {})
            tcp_robot = robot_tf.get('tcp', {})
            current_position = tcp_robot.get('position', [])
            current_quaternion = tcp_robot.get('orientation', [])
            
            if len(current_position) != 3 or len(current_quaternion) != 4:
                return False
            
            # 解析偏移命令
            target_position = current_position.copy()
            for offset_part in offset_commands.split(','):
                offset_part = offset_part.strip()
                if len(offset_part) < 2:
                    continue
                
                axis = offset_part[0].lower()
                value_str = offset_part[1:]
                
                if axis in ['x', 'y', 'z']:
                    try:
                        offset_value = float(value_str)
                        axis_index = {'x': 0, 'y': 1, 'z': 2}[axis]
                        target_position[axis_index] += offset_value
                    except ValueError:
                        continue
            
            # 调用基类的move_j_to方法
            position_str = f"{target_position[0]},{target_position[1]},{target_position[2]}"
            quaternion_str = f"{current_quaternion[0]},{current_quaternion[1]},{current_quaternion[2]},{current_quaternion[3]}"
            
            result = self.move_j_to(
                arm_name=arm_name,
                position=position_str,
                quaternion=quaternion_str,
                ref_frame="world",
                speed=speed,
                acc=acc,
                need_traj=True,
                execute=execute,
                wait=wait,
                max_complexity_score=2.0,
                max_retry_attempts=3,
                cartesian=False,
                keep_orientation=False,
                weight=100.0,
                tolerance=None
            )
            
            return result and result.get('code') == 200
        except Exception:
            return False

    def force_comp(self, arm_name, position, orientation=[0,0,0], ref_frame="tcp", vel=0.02,
                   zero_ft_sensor_first=True, stiff_scale=[1.0,1.0,1.0,1.0,1.0,1.0], speed=0.02, acc=0.02,
                   need_traj=False, execute=True, wait=True, must_reach_target=False):
        """力控补偿运动（ForceComp）
        
        Args:
            arm_name: 手臂名称，"left" 或 "right"
            position: 位置增量，逗号分隔的字符串
            orientation: 姿态增量，逗号分隔的字符串
            ref_frame: 参考坐标系
            vel: 运动速度
            zero_ft_sensor_first: 是否先清零力传感器
            stiff_scale: 刚度缩放
            speed: 运动速度
            acc: 运动加速度
            need_traj: 是否需要轨迹数据
            execute: 是否执行运动
            wait: 是否等待执行完成
            must_reach_target: 是否必须到达目标
        
        Returns:
            bool: 成功返回True，失败返回False
        """
        try:
            # 解析位置和姿态
            if isinstance(position, str):
                position_parts = position.split(',')
                position = [float(x.strip()) for x in position_parts if x.strip()]
            
            if isinstance(orientation, str):
                orientation_parts = orientation.split(',')
                orientation = [float(x.strip()) for x in orientation_parts if x.strip()]
            
            # 构造请求
            force_comp_request = {
                "arm_name": arm_name,
                "position": position,
                "orientation": orientation,
                "ref_frame": ref_frame,
                "vel": float(vel),
                "zero_ft_sensor_first": bool(zero_ft_sensor_first),
                "stiff_scale": stiff_scale,
                "speed": float(speed),
                "acc": float(acc),
                "need_traj": bool(need_traj),
                "execute": bool(execute),
                "wait": bool(wait),
                "must_reach_target": bool(must_reach_target)
            }
            
            response = requests.post(
                f"{self.server_url}/force_comp",
                json=force_comp_request,
                headers={"Content-Type": "application/json"},
                timeout=30
            )
            
            return response.status_code == 200
        except Exception:
            return False

def main():
    """主函数，使用Fire创建命令行接口"""
    os.environ['PAGER'] = 'cat'
    fire.Fire(LabbotManagerClient)

if __name__ == "__main__":
    main()