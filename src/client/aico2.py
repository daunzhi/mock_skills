#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
@File    :   aico2.py
@Time    :   2025/10/29 11:18:24
@Author  :   Zhangzhongwen
@Version :   1.0
@Desc    :   
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
    """扩展的机器人管理客户端，基于基类添加更多功能"""
    
    def __init__(self):
        super().__init__()
    
    def _get_arm_joint_positions(self, arm):
        """获取指定手臂的 7 个关节位置（单位：弧度）。

        Args:
            arm (str): 手臂名称，取值为 `"left"` 或 `"right"`。

        Returns:
            list[float] | None: 成功时返回长度为 7 的关节位置列表（弧度），
            失败或数据不完整时返回 `None`。

        说明:
            - 通过向 `self.server_url + "/get_robot_status"` 发送 POST 请求获取全局关节状态。
            - 关节数据约定：前 2 个为躯干，之后 7 个为左臂，最后 7 个为右臂。
            - 根据 `arm` 参数切片提取对应手臂的 7 个关节位置。
        """
        print(f"正在获取{arm}手臂的关节位置...")
        
        # 构造请求参数
        joint_states_request = {
            "arm": "all"  # 获取全部关节状态
        }
        
        try:
            response = requests.post(
                f"{self.server_url}/get_robot_status",
                json=joint_states_request,
                headers={"Content-Type": "application/json"},
                timeout=30
            )
            
            if response.stsyatus_code == 200:
                result = response.json()
                
                if result.get('code') == 200:  # ErrorCode.Success
                    joint_states = result.get('joint_states', [])
                    
                    if len(joint_states) >= 16:  # 确保有足够的关节数据
                        # 根据机器人配置：前2个是躯干，接下来7个是左臂，最后7个是右臂
                        if arm == "left":
                            arm_positions = joint_states[2:9]  # 左臂7个关节
                        elif arm == "right":
                            arm_positions = joint_states[9:16]  # 右臂7个关节
                        else:
                            print(f"❌ 无效的手臂名称: {arm}")
                            return None
                        
                        print(f"✅ 成功获取{arm}手臂关节位置: {[f'{x:.4f}' for x in arm_positions]}")
                        return arm_positions
                    else:
                        print(f"❌ 关节数据不完整，期望16个关节，实际获取{len(joint_states)}个")
                        return None
                else:
                    print(f"❌ 获取关节状态失败: {result.get('msg', '未知错误')}")
                    return None
            else:
                print(f"❌ 请求失败，状态码: {response.status_code}")
                print(f"错误信息: {response.text}")
                return None
                
        except requests.exceptions.RequestException as e:
            print(f"❌ 请求异常: {e}")
            return None
    
    def _move_arm_to_positions(self, arm, positions, speed=0.3, acc=0.3, execute=True, wait=True):
        """将指定手臂移动到指定关节位置
        
        Args:
            arm: 手臂名称，"left" 或 "right"
            positions: 7个关节位置的列表
            speed: 运动速度 (0.0-1.0)
            acc: 运动加速度 (0.0-1.0)
            execute: 是否执行运动
            wait: 是否等待执行完成
            
        Returns:
            bool: 成功返回True，失败返回False
        """
        print(f"正在将{arm}手臂移动到目标位置...")
        print(f"目标关节位置: {[f'{x:.4f}' for x in positions]}")
        
        # 构造请求参数
        arm_requests = [{
            "arm_name": arm,
            "joint_positions": positions
        }]
        
        movej_request = {
            "arm_requests": arm_requests,
            "speed": speed,
            "acc": acc,
            "need_traj": True,
            "wait": wait,
            "execute": execute
        }
        
        print(f"发送MoveJ请求: {json.dumps(movej_request, indent=2, ensure_ascii=False)}")
        
        try:
            response = requests.post(
                f"{self.server_url}/move_j",
                json=movej_request,
                headers={"Content-Type": "application/json"},
                timeout=60  # 运动可能需要较长时间
            )
            
            if response.status_code == 200:
                result = response.json()
                print("✅ MoveJ请求成功!")
                print(f"响应: {json.dumps(result, indent=2, ensure_ascii=False)}")
                
                # 显示最终关节位置
                if result.get('final_joint_positions'):
                    final_pos = result['final_joint_positions']
                    if arm == "left":
                        arm_final_pos = final_pos[2:9]  # 左臂7个关节
                    else:  # right
                        arm_final_pos = final_pos[9:16]  # 右臂7个关节
                    
                    print(f"✅ {arm}手臂最终关节位置: {[f'{x:.4f}' for x in arm_final_pos]}")
                    
                    # 转换为角度显示
                    final_degrees = [math.degrees(x) for x in arm_final_pos]
                    print(f"✅ {arm}手臂最终关节位置(角度): {[f'{x:.2f}°' for x in final_degrees]}")
                
                return True
            else:
                print(f"❌ MoveJ请求失败，状态码: {response.status_code}")
                print(f"错误信息: {response.text}")
                return False
                
        except requests.exceptions.RequestException as e:
            print(f"❌ MoveJ请求异常: {e}")
            return False
    
    def copy_left_to_right(self, speed=1.0, acc=1.0, execute=True, wait=True):
        """将左手的关节位置复制到右手，使右手与左手姿势完全相同
        
        Args:
            speed: 运动速度 (0.0-1.0)，默认0.3
            acc: 运动加速度 (0.0-1.0)，默认0.3
            execute: 是否执行运动，默认True
            wait: 是否等待执行完成，默认True
        
        Examples:
            python3 labbot_manager.py copy_left_to_right
            python3 labbot_manager.py copy_left_to_right --speed=0.2 --execute=False
            python3 labbot_manager.py copy_left_to_right --speed=0.5 --acc=0.5
        """
        print(f"\n{'='*60}")
        print(f"🤖 复制左手姿势到右手")
        print(f"{'='*60}")
        print(f"运动参数: 速度={speed}, 加速度={acc}, 执行={'是' if execute else '否'}, 等待={'是' if wait else '否'}")
        
        # 1. 获取左手当前关节位置
        left_positions = self._get_arm_joint_positions("left")
        if left_positions is None:
            print("❌ 获取左手关节位置失败，操作终止")
            return False
        
        # 显示左手当前位置（角度）
        left_degrees = [math.degrees(x) for x in left_positions]
        print(f"\n📍 左手当前关节位置(角度): {[f'{x:.2f}°' for x in left_degrees]}")
        
        # 2. 将左手位置应用到右手
        print(f"\n🔄 开始将左手姿势复制到右手...")
        success = self._move_arm_to_positions("right", left_positions, speed, acc, execute, wait)
        
        if success:
            print(f"\n🎉 成功将左手姿势复制到右手!")
            if execute:
                print("✅ 右手现在与左手姿势完全相同")
            else:
                print("ℹ️ 仅进行了运动规划，未实际执行")
        else:
            print(f"\n❌ 复制左手姿势到右手失败")
        
        return success
    
    def copy_right_to_left(self, speed=1.0, acc=1.0, execute=True, wait=True):
        """将右手的关节位置复制到左手，使左手与右手姿势完全相同
        
        Args:
            speed: 运动速度 (0.0-1.0)，默认0.3
            acc: 运动加速度 (0.0-1.0)，默认0.3
            execute: 是否执行运动，默认True
            wait: 是否等待执行完成，默认True
        
        Examples:
            python3 labbot_manager.py copy_right_to_left
            python3 labbot_manager.py copy_right_to_left --speed=0.2 --execute=False
            python3 labbot_manager.py copy_right_to_left --speed=0.5 --acc=0.5
        """
        print(f"\n{'='*60}")
        print(f"🤖 复制右手姿势到左手")
        print(f"{'='*60}")
        print(f"运动参数: 速度={speed}, 加速度={acc}, 执行={'是' if execute else '否'}, 等待={'是' if wait else '否'}")
        
        # 1. 获取右手当前关节位置
        right_positions = self._get_arm_joint_positions("right")
        if right_positions is None:
            print("❌ 获取右手关节位置失败，操作终止")
            return False
        
        # 显示右手当前位置（角度）
        right_degrees = [math.degrees(x) for x in right_positions]
        print(f"\n📍 右手当前关节位置(角度): {[f'{x:.2f}°' for x in right_degrees]}")
        
        # 2. 将右手位置应用到左手
        print(f"\n🔄 开始将右手姿势复制到左手...")
        success = self._move_arm_to_positions("left", right_positions, speed, acc, execute, wait)
        
        if success:
            print(f"\n🎉 成功将右手姿势复制到左手!")
            if execute:
                print("✅ 左手现在与右手姿势完全相同")
            else:
                print("ℹ️ 仅进行了运动规划，未实际执行")
        else:
            print(f"\n❌ 复制右手姿势到左手失败")
        
        return success
    
    def mirror_arms(self, source_arm="left", speed=0.3, acc=0.3, execute=True, wait=True):
        """镜像手臂姿势（将一只手的姿势镜像复制到另一只手）
        
        Args:
            source_arm: 源手臂，"left" 或 "right"，默认"left"
            speed: 运动速度 (0.0-1.0)，默认0.3
            acc: 运动加速度 (0.0-1.0)，默认0.3
            execute: 是否执行运动，默认True
            wait: 是否等待执行完成，默认True
        
        Examples:
            python3 labbot_manager.py mirror_arms --source_arm=left
            python3 labbot_manager.py mirror_arms --source_arm=right --speed=0.2
        """
        if source_arm == "left":
            return self.copy_left_to_right(speed, acc, execute, wait)
        elif source_arm == "right":
            return self.copy_right_to_left(speed, acc, execute, wait)
        else:
            print(f"❌ 无效的源手臂名称: {source_arm}，必须是 'left' 或 'right'")
            return False
    
    def sync_arms_to_home(self, speed=0.3, acc=0.3, execute=True, wait=True):
        """将双臂同步移动到初始位置（所有关节角度为0）
        
        Args:
            speed: 运动速度 (0.0-1.0)，默认0.3
            acc: 运动加速度 (0.0-1.0)，默认0.3
            execute: 是否执行运动，默认True
            wait: 是否等待执行完成，默认True
        
        Examples:
            python3 labbot_manager.py sync_arms_to_home
            python3 labbot_manager.py sync_arms_to_home --speed=0.2 --execute=False
        """
        print(f"\n{'='*60}")
        print(f"🏠 将双臂同步移动到初始位置")
        print(f"{'='*60}")
        print(f"运动参数: 速度={speed}, 加速度={acc}, 执行={'是' if execute else '否'}, 等待={'是' if wait else '否'}")
        
        # 初始位置：所有关节角度为0
        home_positions = [0.0] * 7
        
        print(f"📍 目标位置: 所有关节角度为0°")
        
        # 构造双臂请求
        arm_requests = [
            {
                "arm_name": "left",
                "joint_positions": home_positions
            },
            {
                "arm_name": "right", 
                "joint_positions": home_positions
            }
        ]
        
        movej_request = {
            "arm_requests": arm_requests,
            "speed": speed,
            "acc": acc,
            "need_traj": True,
            "wait": wait,
            "execute": execute
        }
        
        print(f"\n发送双臂MoveJ请求: {json.dumps(movej_request, indent=2, ensure_ascii=False)}")
        
        try:
            response = requests.post(
                f"{self.server_url}/move_j",
                json=movej_request,
                headers={"Content-Type": "application/json"},
                timeout=60
            )
            
            if response.status_code == 200:
                result = response.json()
                print("✅ 双臂MoveJ请求成功!")
                print(f"响应: {json.dumps(result, indent=2, ensure_ascii=False)}")
                
                if execute:
                    print(f"\n🎉 双臂已成功移动到初始位置!")
                else:
                    print(f"\nℹ️ 双臂运动规划完成，未实际执行")
                
                return True
            else:
                print(f"❌ 双臂MoveJ请求失败，状态码: {response.status_code}")
                print(f"错误信息: {response.text}")
                return False
                
        except requests.exceptions.RequestException as e:
            print(f"❌ 双臂MoveJ请求异常: {e}")
            return False
    
    def fast_move_j(self, cmds, speed=0.3, acc=0.3, execute=True, wait=True):
        """快速控制多个关节的角度变化（增量模式）
        
        Args:
            cmds: 关节命令字符串，格式为 joint_name1:cmd,joint_name2:cmd,...
                比如: b1:+5,l1:-5,r1:-5
                joint_name: 关节名称，支持：
                        - b1, b2: 躯干关节
                        - l1, l2, l3, l4, l5, l6, l7: 左臂关节
                        - r1, r2, r3, r4, r5, r6, r7: 右臂关节
                cmd: 增量命令，格式为 +数字 或 -数字，如 +15, -10
            speed: 运动速度 (0.0-1.0)，默认0.3
            acc: 运动加速度 (0.0-1.0)，默认0.3
            execute: 是否执行运动，默认True
            wait: 是否等待执行完成，默认True
        
        Examples:
            python3 aico2.py fast_move_j "r3:+15"              # 右臂关节3增加15度
            python3 aico2.py fast_move_j "b1:-5"               # 躯干关节1减少5度
            python3 aico2.py fast_move_j "b1:+5,l1:-5,r1:-5"   # 多关节同时运动
        """
        print(f"\n{'='*60}")
        print(f"🎯 快速控制关节: {cmds}")
        print(f"{'='*60}")
        print(f"运动参数: 速度={speed}, 加速度={acc}, 执行={'是' if execute else '否'}, 等待={'是' if wait else '否'}")
        
        # 定义关节映射
        joint_mapping = {
            # 躯干关节 (body)
            'b1': ('body', 0), 'b2': ('body', 1),
            # 左臂关节 (left arm)
            'l1': ('left', 0), 'l2': ('left', 1), 'l3': ('left', 2), 'l4': ('left', 3),
            'l5': ('left', 4), 'l6': ('left', 5), 'l7': ('left', 6),
            # 右臂关节 (right arm)
            'r1': ('right', 0), 'r2': ('right', 1), 'r3': ('right', 2), 'r4': ('right', 3),
            'r5': ('right', 4), 'r6': ('right', 5), 'r7': ('right', 6)
        }
        
        # 解析命令字符串
        try:
            cmd_pairs = [cmd.strip() for cmd in cmds.split(',')]
            joint_commands = {}
            
            for cmd_pair in cmd_pairs:
                if ':' not in cmd_pair:
                    print(f"❌ 无效的命令格式: {cmd_pair}，应为 joint_name:cmd")
                    return False
                
                joint_name, cmd = cmd_pair.split(':', 1)
                joint_name = joint_name.strip()
                cmd = cmd.strip()
                
                # 验证关节名称
                if joint_name not in joint_mapping:
                    valid_joints = ', '.join(sorted(joint_mapping.keys()))
                    print(f"❌ 无效的关节名称: {joint_name}")
                    print(f"支持的关节名称: {valid_joints}")
                    return False
                
                # 解析增量命令
                if not (cmd.startswith('+') or cmd.startswith('-')):
                    print(f"❌ 无效的命令格式: {cmd}，应为 +数字 或 -数字")
                    return False
                
                try:
                    increment_degrees = float(cmd)
                    joint_commands[joint_name] = increment_degrees
                    print(f"📍 {joint_name}: {increment_degrees:+.1f}°")
                except ValueError:
                    print(f"❌ 无效的数字格式: {cmd}")
                    return False
            
            if not joint_commands:
                print("❌ 没有有效的关节命令")
                return False
            
        except Exception as e:
            print(f"❌ 命令解析失败: {e}")
            return False
        
        # 按手臂分组构造增量位置
        arm_increments = {}
        
        for joint_name, increment_degrees in joint_commands.items():
            arm_name, joint_index = joint_mapping[joint_name]
            increment_radians = math.radians(increment_degrees)
            
            if arm_name not in arm_increments:
                if arm_name == 'body':
                    arm_increments[arm_name] = [0.0, 0.0]
                else:  # left or right arm
                    arm_increments[arm_name] = [0.0] * 7
            
            arm_increments[arm_name][joint_index] = increment_radians
        
        # 构造请求参数
        arm_requests = []
        for arm_name, increments in arm_increments.items():
            arm_requests.append({
                "arm_name": arm_name,
                "increment_joint_positions": increments
            })
            print(f"📊 {arm_name} 增量: {[f'{math.degrees(x):+.1f}°' for x in increments]}")
        
        print(f"🔄 总共控制 {len(joint_commands)} 个关节，涉及 {len(arm_requests)} 个手臂")
        
        # 发送MoveJ请求
        movej_request = {
            "arm_requests": arm_requests,
            "speed": speed,
            "acc": acc,
            "need_traj": True,
            "wait": wait,
            "execute": execute
        }
        
        print(f"\n🚀 发送MoveJ请求...")
        
        try:
            response = requests.post(
                f"{self.server_url}/move_j",
                json=movej_request,
                headers={"Content-Type": "application/json"},
                timeout=60
            )
            
            if response.status_code == 200:
                result = response.json()
                print("✅ MoveJ请求成功!")
                
                if execute:
                    print(f"🎉 多关节运动完成!")
                else:
                    print(f"ℹ️ 多关节运动规划完成，未实际执行")
                
                return True
            else:
                print(f"❌ MoveJ请求失败，状态码: {response.status_code}")
                print(f"错误信息: {response.text}")
                return False
                
        except requests.exceptions.RequestException as e:
            print(f"❌ MoveJ请求异常: {e}")
            return False

    def fast_move_j_to(self, arm_name, offset_commands, speed=0.1, acc=0.1, execute=True, wait=True):
        """快速末端位置控制
        
        Args:
            arm_name: 手臂名称，left 或 right
            offset_commands: 偏移命令字符串，格式如 "x+0.1" 或 "x+0.05,y-0.02"
            speed: 运动速度 (0.0-1.0)
            acc: 运动加速度 (0.0-1.0)
            execute: 是否执行运动
            wait: 是否等待执行完成
            
        Returns:
            bool: 成功返回True，失败返回False
            
        Examples:
            python3 aico2.py fast_move_l right "x+0.1"
            python3 aico2.py fast_move_l left "x+0.05,y-0.02"
            python3 aico2.py fast_move_l right "z-0.03" --execute=False
        """
        print(f"\n=== 快速末端位置控制 ===\n")
        print(f"手臂: {arm_name}")
        print(f"偏移命令: {offset_commands}")
        print(f"速度: {speed}, 加速度: {acc}")
        print(f"执行: {execute}, 等待: {wait}")
        
        # 验证手臂名称
        if arm_name not in ['left', 'right']:
            print(f"❌ 无效的手臂名称: {arm_name}，必须是 'left' 或 'right'")
            return False
        
        # 获取当前状态
        print(f"\n📊 获取{arm_name}手臂当前状态...")
        try:
            joint_states_request = {"arm": arm_name}
            
            response = requests.post(
                f"{self.server_url}/get_robot_status",
                json=joint_states_request,
                headers={"Content-Type": "application/json"},
                timeout=30
            )
            
            if response.status_code != 200:
                print(f"❌ 获取当前状态失败，状态码: {response.status_code}")
                return False
            
            result = response.json()
            if result.get('code') != 200:
                print(f"❌ 获取当前状态失败: {result.get('msg', '未知错误')}")
                return False
            
            # 获取末端执行器位姿
            if arm_name == 'left':
                arm_data = result.get('left_arm', {})
            else:  # right
                arm_data = result.get('right_arm', {})
            
            robot_tf = arm_data.get('robot_tf', {})
            tcp_robot = robot_tf.get('tcp', {})
            current_position = tcp_robot.get('position', [])
            current_quaternion = tcp_robot.get('orientation', [])
            
            if not current_position or not current_quaternion:
                print(f"❌ 未获取到{arm_name}手臂末端执行器位姿数据")
                return False
            
            if len(current_position) != 3 or len(current_quaternion) != 4:
                print(f"❌ 末端执行器位姿数据格式错误")
                return False
            
            print(f"✅ 当前位置: [{current_position[0]:.4f}, {current_position[1]:.4f}, {current_position[2]:.4f}]")
            print(f"✅ 当前姿态: [{current_quaternion[0]:.4f}, {current_quaternion[1]:.4f}, {current_quaternion[2]:.4f}, {current_quaternion[3]:.4f}]")
            
        except requests.exceptions.RequestException as e:
            print(f"❌ 获取当前状态异常: {e}")
            return False
        
        # 解析偏移命令
        try:
            target_position = current_position.copy()
            
            # 解析多个偏移命令（用逗号分隔）
            offset_parts = offset_commands.split(',')
            
            for offset_part in offset_parts:
                offset_part = offset_part.strip()
                
                # 解析单个偏移命令，格式如 "x+0.1" 或 "y-0.02"
                if len(offset_part) < 3:
                    print(f"❌ 偏移命令格式错误: {offset_part}")
                    return False
                
                axis = offset_part[0].lower()
                # operator = offset_part[1]
                # value_str = offset_part[2:]
                value_str = offset_part[1:]
                
                # 验证轴名称
                if axis not in ['x', 'y', 'z']:
                    print(f"❌ 无效的轴名称: {axis}，必须是 'x', 'y' 或 'z'")
                    return False
                
                # # 验证操作符
                # if operator not in ['+', '-']:
                #     print(f"❌ 无效的操作符: {operator}，必须是 '+' 或 '-'")
                #     return False
                
                # 解析数值
                try:
                    offset_value = float(value_str)
                    # if operator == '-':
                    #     offset_value = -offset_value
                except ValueError:
                    print(f"❌ 无效的偏移数值: {value_str}")
                    return False
                
                # 应用偏移
                axis_index = {'x': 0, 'y': 1, 'z': 2}[axis]
                target_position[axis_index] += offset_value
                
                print(f"📐 {axis.upper()}轴偏移: {offset_value:+.4f}m")
            
            print(f"🎯 目标位置: [{target_position[0]:.4f}, {target_position[1]:.4f}, {target_position[2]:.4f}]")
            print(f"🎯 保持姿态: [{current_quaternion[0]:.4f}, {current_quaternion[1]:.4f}, {current_quaternion[2]:.4f}, {current_quaternion[3]:.4f}]")
            
        except Exception as e:
            print(f"❌ 偏移命令解析错误: {e}")
            return False
        
        # 调用move_j_to执行运动
        print(f"\n🚀 执行末端位置运动...")
        
        # 将位置和姿态转换为字符串格式
        position_str = f"{target_position[0]},{target_position[1]},{target_position[2]}"
        quaternion_str = f"{current_quaternion[0]},{current_quaternion[1]},{current_quaternion[2]},{current_quaternion[3]}"
        
        # 调用基类的move_j_to方法
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
        
        if result and result.get('code') == 200:
            print(f"\n🎉 快速末端位置控制成功!")
            if execute:
                print(f"✅ {arm_name}手臂已移动到目标位置")
            else:
                print(f"ℹ️ 仅进行了运动规划，未实际执行")
        else:
            print(f"\n❌ 快速末端位置控制失败")
        return result

    def force_comp(self, arm_name, position, orientation=[0,0,0], ref_frame="tcp", vel=0.02,
                   zero_ft_sensor_first=True, stiff_scale=[1.0,1.0,1.0,1.0,1.0,1.0], speed=0.02, acc=0.02,
                   need_traj=False, execute=True, wait=True, must_reach_target=False):
        """力控补偿运动（ForceComp）
        
        Args:
            arm_name: 手臂名称，left 或 right
            position: 位置增量，逗号分隔的字符串，如 "0.0,0.0,0.04"
            orientation: 姿态增量，逗号分隔的字符串，如 "0,0,0"
            ref_frame: 参考坐标系，格式为 "tcp, world"，默认 "tcp"
            vel: 运动速度 (默认0.02)
            zero_ft_sensor_first: 是否先清零力传感器 (默认True)
            stiff_scale: 刚度缩放，6个浮点数的列表 (默认[1.0,1.0,1.0,1.0,1.0,1.0])
            speed: 运动速度 (默认0.02)
            acc: 运动加速度 (默认0.02)
            need_traj: 是否需要轨迹数据 (默认False)
            execute: 是否执行运动 (默认True)
            wait: 是否等待执行完成 (默认True)
        
        Examples:
            python3 aico2.py force_comp left "0.0,0.0,0.04" "0,0,0"
            python3 aico2.py force_comp right "0.0,0.0,0.04" "0,0,0" --ref_frame="world"
            python3 aico2.py force_comp left "0.0,0.0,0.04" "0,0,0" --stiff_scale="0.5,0.5,0.5,1.0,1.0,1.0"
        """
        print(f"\n=== 力控补偿运动（ForceComp） ===\n")
        print(f"手臂: {arm_name}")
        print(f"位置增量: {position}")
        print(f"姿态增量: {orientation}")
        print(f"参考坐标系: {ref_frame}")
        print(f"速度: {vel}")
        print(f"清零力传感器: {zero_ft_sensor_first}")
        print(f"刚度缩放: {stiff_scale}")
        print(f"运动参数: 速度={speed}, 加速度={acc}")
        print(f"执行: {execute}, 等待: {wait}")
        
        # 验证手臂参数
        if arm_name not in ["left", "right"]:
            print(f"❌ 无效的手臂名称: {arm_name}，必须是 'left' 或 'right'")
            return False
        
        # 解析位置参数
        try:
            if isinstance(position, str):
                position_list = [float(x.strip()) for x in position.split(',')]
            elif isinstance(position, (list, tuple)):
                position_list = [float(x) for x in position]
            else:
                raise ValueError(f"不支持的位置参数类型: {type(position)}")
            
            if len(position_list) != 3:
                print(f"❌ 位置参数应该有3个值，但得到{len(position_list)}个")
                return False
        except (ValueError, TypeError) as e:
            print(f"❌ 位置解析错误: {e}")
            return False
        
        # 解析姿态参数
        try:
            if isinstance(orientation, str):
                orientation_list = [float(x.strip()) for x in orientation.split(',')]
            elif isinstance(orientation, (list, tuple)):
                orientation_list = [float(x) for x in orientation]
            else:
                raise ValueError(f"不支持的姿态参数类型: {type(orientation)}")
            
            if len(orientation_list) != 3:
                print(f"❌ 姿态参数应该有3个值，但得到{len(orientation_list)}个")
                return False
        except (ValueError, TypeError) as e:
            print(f"❌ 姿态解析错误: {e}")
            return False
        
        # 解析刚度缩放参数
        try:
            if isinstance(stiff_scale, str):
                stiff_scale_list = [float(x.strip()) for x in stiff_scale.split(',')]
            elif isinstance(stiff_scale, (list, tuple)):
                stiff_scale_list = [float(x) for x in stiff_scale]
            else:
                raise ValueError(f"不支持的刚度缩放参数类型: {type(stiff_scale)}")
            
            if len(stiff_scale_list) != 6:
                print(f"❌ 刚度缩放参数应该有6个值，但得到{len(stiff_scale_list)}个")
                return False
        except (ValueError, TypeError) as e:
            print(f"❌ 刚度缩放解析错误: {e}")
            return False
        
        # 验证参考坐标系
        valid_ref_frames = ["tcp", "world"]
        if ref_frame not in valid_ref_frames:
            print(f"❌ 无效的参考坐标系: {ref_frame}，必须是 {valid_ref_frames} 中的一个")
            return False
        
        # 构造请求参数
        force_comp_request = {
            "arm_requests": [
                {
                    "arm_name": arm_name,
                    "position": position_list,
                    "orientation": orientation_list,
                    "ref_frame": ref_frame
                }
            ],
            "vel": float(vel),
            "zero_ft_sensor_first": bool(zero_ft_sensor_first),
            "stiff_scale": stiff_scale_list,
            "speed": float(speed),
            "acc": float(acc),
            "need_traj": bool(need_traj),
            "execute": bool(execute),
            "wait": bool(wait),
            "must_reach_target": bool(must_reach_target)
        }
        
        print(f"\n发送请求: {json.dumps(force_comp_request, indent=2, ensure_ascii=False)}")
        
        try:
            response = requests.post(
                f"{self.server_url}/force_comp",
                json=force_comp_request,
                headers={"Content-Type": "application/json"},
                timeout=30
            )
            
            if response.status_code == 200:
                result = response.json()
                print("\n✅ 请求成功!")
                print(f"响应: {json.dumps(result, indent=2, ensure_ascii=False)}")
                
                # 显示执行结果
                if result.get('code') == 200:  # ErrorCode.Success
                    trajectory_path = result.get('trajectory_path', '')
                    execution_result = result.get('execution_result', '')
                    
                    if trajectory_path:
                        print(f"\n💾 轨迹文件: {trajectory_path}")
                    
                    if execution_result:
                        print(f"\n🎯 执行结果: {execution_result}")
                    
                    print(f"\n🎉 ForceComp力控补偿运动完成!")
                else:
                    print(f"\n⚠️ ForceComp力控补偿运动失败: {result.get('msg', '未知错误')}")
                
                return result
            else:
                print(f"\n❌ 请求失败，状态码: {response.status_code}")
                print(f"错误信息: {response.text}")
                return False
                
        except requests.exceptions.RequestException as e:
            print(f"❌ 请求异常: {e}")
            return False
        
    def mate(self, arm_name, contact_dir=[0, 0, 1],max_contact_force = 10,
                zero_ft_sensor_first=True,safe_force=50,mate_times=1,mate_axis=[0, 0, 0, 0, 0, 0],
                max_distance_contact_dir=0.01,slide_range=0.02,slide_vel=0.02,slide_acc=0.1,
                rotate_range=10,rotate_vel=10,rotate_acc=180,
                   need_traj=False, execute=True, wait=True):
        """力控补偿运动（ForceComp）
        
        Args:
            arm_name: 手臂名称，left 或 right
            contact_dir: 接触轴
            max_contact_force: 最大接触力
            zero_ft_sensor_first: 是否先清零力传感器
            safe_force: 安全力
            mate_axis: 6个方向的啮合轴 [x, y, z, rx, ry, rz]
            max_distance_contact_dir: 最大接触方向插入距离，超过最大距离啮合完成
            slide_range: 滑动啮合范围
            slide_vel: 滑动啮合速度
            slide_acc: 滑动啮合加速度
            rotate_range: 旋转啮合范围
            rotate_vel: 旋转啮合速度
            rotate_acc: 旋转啮合加速度
            need_traj: 是否需要轨迹数据 (默认False)
            execute: 是否执行运动 (默认True)
            wait: 是否等待执行完成 (默认True)
        
        Examples:
            python3 aico2.py mate left "0.0,0.0,0.04" "0,0,0"
            
        """
        print(f"\n=== 啮合运动（Mating） ===\n")
        print(f"手臂: {arm_name}")
        print(f"接触轴: {contact_dir}")
        print(f"最大接触力: {max_contact_force}")
        print(f"安全力: {safe_force}")
        print(f"6个方向的啮合轴: {mate_axis}")
        print(f"啮合次数: {mate_times}")
        print(f"清零力传感器: {zero_ft_sensor_first}")
        print(f"最大接触方向插入距离: {max_distance_contact_dir}")
        print(f"滑动啮合参数: 滑动啮合范围={slide_range}, 滑动啮合速度={slide_vel}, 滑动啮合加速度={slide_acc}")
        print(f"旋转啮合参数: 旋转啮合范围={rotate_range}, 旋转啮合速度={rotate_vel}, 旋转啮合加速度={rotate_acc}")
        print(f"执行: {execute}, 等待: {wait}")
        
        # 验证手臂参数
        if arm_name not in ["left", "right"]:
            print(f"❌ 无效的手臂名称: {arm_name}，必须是 'left' 或 'right'")
            return False
        
        # 解析接触方向参数
        try:
            if isinstance(contact_dir, str):
                contact_dir_list = [int(x.strip()) for x in contact_dir.split(',')]
            elif isinstance(contact_dir, (list, tuple)):
                contact_dir_list = [int(x) for x in contact_dir]
            else:
                raise ValueError(f"不支持的接触方向参数类型: {type(contact_dir)}")
            
            if len(contact_dir_list) != 3:
                print(f"❌ 接触方向参数应该有3个值，但得到{len(contact_dir_list)}个")
                return False
        except (ValueError, TypeError) as e:
            print(f"❌ 接触方向解析错误: {e}")
            return False
        
        # 解析6个方向的啮合轴参数
        try:
            if isinstance(mate_axis, str):
                mate_axis_list = [int(x.strip()) for x in mate_axis.split(',')]
            elif isinstance(mate_axis, (list, tuple)):
                mate_axis_list = [int(x) for x in mate_axis]
            else:
                raise ValueError(f"不支持的啮合轴参数类型: {type(mate_axis)}")
            
            if len(mate_axis_list) != 6:
                print(f"❌ 啮合轴参数应该有6个值，但得到{len(mate_axis_list)}个")
                return False
        except (ValueError, TypeError) as e:
            print(f"❌ 啮合轴解析错误: {e}")
            return False
        
        
        
        # 构造请求参数
        mate_request = {
            "arm_name": arm_name,
            "mate_times": int(mate_times),
            "contact_dir": contact_dir_list,
            "zero_ft_sensor_first": bool(zero_ft_sensor_first),
            "max_contact_force": float(max_contact_force),
            "mate_axis": mate_axis_list,
            "safe_force": float(safe_force),
            "max_distance_contact_dir": float(max_distance_contact_dir),
            "slide_range": float(slide_range),
            "slide_vel": float(slide_vel),
            "slide_acc": float(slide_acc),
            "rotate_range": float(rotate_range),
            "rotate_vel": float(rotate_vel),
            "rotate_acc": float(rotate_acc),
            "need_traj": bool(need_traj),
            "execute": bool(execute),
            "wait": bool(wait)
        }
        
        print(f"\n发送请求: {json.dumps(mate_request, indent=2, ensure_ascii=False)}")
        
        try:
            response = requests.post(
                f"{self.server_url}/mate",
                json=mate_request,
                headers={"Content-Type": "application/json"},
                timeout=30
            )
            
            if response.status_code == 200:
                result = response.json()
                print("\n✅ 请求成功!")
                print(f"响应: {json.dumps(result, indent=2, ensure_ascii=False)}")
                
                # 显示执行结果
                if result.get('code') == 200:  # ErrorCode.Success
                    trajectory_path = result.get('trajectory_path', '')
                    execution_result = result.get('execution_result', '')
                    
                    if trajectory_path:
                        print(f"\n💾 轨迹文件: {trajectory_path}")
                    
                    if execution_result:
                        print(f"\n🎯 执行结果: {execution_result}")
                    
                    print(f"\n🎉 ForceComp力控补偿运动完成!")
                else:
                    print(f"\n⚠️ ForceComp力控补偿运动失败: {result.get('msg', '未知错误')}")
                
                return result
            else:
                print(f"\n❌ 请求失败，状态码: {response.status_code}")
                print(f"错误信息: {response.text}")
                return False
                
        except requests.exceptions.RequestException as e:
            print(f"❌ 请求异常: {e}")
            return False

    def sync_real(self):
        """从aico2真机获取status，然后仿真机器move_j过去"""
        result = self.status(remote_host="192.168.12.206")
        print(f"获取aico2状态成功，开始同步到仿真机器人...")
        if result:
            joint_states = result['joint_states']
            self.move_j(
                body_positions=joint_states[:2],
                left_positions=joint_states[2:9],
                right_positions=joint_states[9:],
            )
        print(f"完成！")

    def run_last_traj_in_real(self, speed=0.5, acc=0.3, wait=True):
        """在真机上运行最后一次执行的轨迹
        
        在~/.aico2/executed_traj目录下查找1分钟内的最新轨迹文件并执行
        
        Args:
            speed: 执行速度 (0.0-1.0)，默认0.5
            acc: 执行加速度 (0.0-1.0)，默认0.3
            wait: 是否等待执行完成，默认True
            
        Returns:
            bool: 成功返回True，失败返回False
            
        Examples:
            python3 aico2.py run_last_traj_in_real
            python3 aico2.py run_last_traj_in_real --speed=0.3 --acc=0.2
        """
        print(f"\n{'='*60}")
        print(f"🚀 在真机上运行最后一次执行的轨迹")
        print(f"{'='*60}")
        print(f"执行参数: 速度={speed}, 加速度={acc}, 等待={'是' if wait else '否'}")
        
        # 1. 构造轨迹目录路径
        traj_dir = Path.home() / ".aico2" / "executed_traj"
        print(f"📁 轨迹目录: {traj_dir}")
        
        # 2. 检查目录是否存在
        if not traj_dir.exists():
            print(f"❌ 轨迹目录不存在: {traj_dir}")
            return False
        
        # 3. 扫描轨迹文件
        pattern = str(traj_dir / "trajectory_*.json")
        traj_files = glob.glob(pattern)
        
        if not traj_files:
            print(f"❌ 在目录 {traj_dir} 中未找到任何轨迹文件")
            return False
        
        print(f"📋 找到 {len(traj_files)} 个轨迹文件")
        
        # 4. 解析文件名中的时间戳并筛选1分钟内的文件
        current_time = datetime.now()
        ten_minute_ago = current_time - timedelta(minutes=30)
        
        valid_files = []
        
        for file_path in traj_files:
            filename = Path(file_path).name
            
            # 解析文件名格式: trajectory_20251015_114128.562_39d5c7fe-929d-4bdf-ac23-900c7f9a5c9f.json
            match = re.match(r'trajectory_(\d{8})_(\d{6})\.(\d{3})_[a-f0-9\-]+\.json', filename)
            
            if match:
                date_str = match.group(1)  # 20251015
                time_str = match.group(2)  # 114128
                ms_str = match.group(3)    # 562
                
                try:
                    # 构造完整的时间戳字符串
                    timestamp_str = f"{date_str}_{time_str}.{ms_str}"
                    file_time = datetime.strptime(timestamp_str, "%Y%m%d_%H%M%S.%f")
                    
                    # 检查是否在1分钟内
                    if file_time >= ten_minute_ago:
                        valid_files.append((file_path, file_time, filename))
                        print(f"✅ 有效文件: {filename} (时间: {file_time.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]})")
                    # else:
                    #     print(f"⏰ 过期文件: {filename} (时间: {file_time.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]})")
                        
                except ValueError as e:
                    print(f"⚠️ 时间戳解析失败: {filename} - {e}")
            else:
                print(f"⚠️ 文件名格式不匹配: {filename}")
        
        # 5. 检查是否有有效文件
        if not valid_files:
            print(f"❌ 在过去1分钟内未找到任何轨迹文件")
            print(f"   当前时间: {current_time.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}")
            print(f"   查找范围: {ten_minute_ago.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]} 至 {current_time.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}")
            return False
        
        # 6. 找到最新的文件
        latest_file = max(valid_files, key=lambda x: x[1])
        latest_path, latest_time, latest_filename = latest_file
        
        print(f"\n🎯 找到最新轨迹文件:")
        print(f"   文件名: {latest_filename}")
        print(f"   时间戳: {latest_time.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}")
        print(f"   路径: {latest_path}")
        
        # 7. 加载轨迹文件
        try:
            with open(latest_path, 'r', encoding='utf-8') as f:
                traj_data = json.load(f)
            print(f"✅ 成功加载轨迹文件")
            
            # 显示轨迹基本信息
            if isinstance(traj_data, dict):
                points_count = len(traj_data.get('points', []))
                joint_names = traj_data.get('joint_names', [])
                print(f"   轨迹点数: {points_count}")
                print(f"   关节数量: {len(joint_names)}")
                print(f"   关节名称: {joint_names[:5]}{'...' if len(joint_names) > 5 else ''}")
            
        except Exception as e:
            print(f"❌ 加载轨迹文件失败: {e}")
            return False
        
        # 8. 从文件名提取轨迹ID（去掉.json后缀）
        traj_id = Path(latest_filename).stem
        print(f"🆔 轨迹ID: {traj_id}")
        
        # 9. 执行轨迹
        print(f"\n🚀 开始在真机上执行轨迹...")
        os.system(f"scp {latest_path} aico2:/home/ubuntu/.aico2/executed_traj/")
        try:
            result = self.run_traj(
                traj_id=traj_id,
                remote_host="192.168.1.92"
            )
            
            if result and result.get('code') == 200:
                print(f"\n🎉 轨迹执行成功!")
                return True
            else:
                print(f"\n❌ 轨迹执行失败")
                return False
                
        except Exception as e:
            print(f"❌ 轨迹执行异常: {e}")
            return False

def main():
    """主函数，使用Fire创建命令行接口"""
    # 禁用分页，直接在终端显示帮助信息
    os.environ['PAGER'] = 'cat'
    fire.Fire(LabbotManagerClient)

if __name__ == "__main__":
    main()