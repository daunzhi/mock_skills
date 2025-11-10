#!/usr/bin/env python3

import requests
import json
import math
import fire
import os

# 服务器地址
SERVER_URL = "http://localhost:9999/api/robot/action"

class LabbotManagerClientBase:
    """MoveJ客户端命令行工具"""
    
    def __init__(self):
        self.server_url = SERVER_URL

    def get_remote_server_url(self, host: str):
        return f"http://{host}:9999/api/robot/action"
    
    def _degrees_to_radians(self, positions):
        """将角度转换为弧度"""
        return [math.radians(pos) for pos in positions]
    
    def _parse_positions(self, positions_input):
        """解析位置参数为浮点数列表"""
        try:
            # 处理 fire 传递的元组或字符串参数
            if isinstance(positions_input, tuple):
                positions = list(positions_input)
            elif isinstance(positions_input, str):
                positions = [float(x.strip()) for x in positions_input.split(',')]
            elif isinstance(positions_input, list):
                positions = positions_input
            else:
                raise ValueError(f"不支持的位置参数类型: {type(positions_input)}")
            
            # 转换为浮点数
            positions = [float(pos) for pos in positions]
            
            # if len(positions) != 7:
            #     raise ValueError(f"期望7个关节位置，但得到{len(positions)}个")
            return positions
        except (ValueError, TypeError) as e:
            print(f"位置解析错误: {e}")
            return None
    
    def move_j_inc(self, body_positions, left_positions, right_positions, degree=True, speed=0.8, acc=0.8, 
                   need_traj=True, wait=True, execute=False, use_arms=None):
        """增量关节运动
        
        Args:
            body_positions: 躯干2个关节的增量位置，逗号分隔的字符串
            left_positions: 左臂7个关节的增量位置，逗号分隔的字符串
            right_positions: 右臂7个关节的增量位置，逗号分隔的字符串
            degree: 是否使用角度单位（默认True，使用角度）
            speed: 运动速度 (0.0-1.0)
            acc: 运动加速度 (0.0-1.0)
            need_traj: 是否需要轨迹数据
            wait: 是否等待执行完成
            execute: 是否执行运动
            use_arms: 指定要使用的机械臂列表，如"left_arm,right_arm"或"left_arm"
        
        Examples:
            python3 movej_test_client.py move_j_inc "0,0" "0,0,0,0,0,0,0" "30,0,0,0,0,0,0" --degree
            python3 movej_test_client.py move_j_inc "0,0" "0,0,0,0,0,0,0" "0.5,0,0,0,0,0,0"
            python3 movej_test_client.py move_j_inc "0,0" "0,0,0,0,0,0,0" "30,0,0,0,0,0,0" --use_arms="left_arm"
        """
        print(f"\n=== 增量关节运动 ===\n")
        print(f"躯干增量位置: {body_positions}")
        print(f"左臂增量位置: {left_positions}")
        print(f"右臂增量位置: {right_positions}")
        print(f"单位: {'角度' if degree else '弧度'}")
        print(f"速度: {speed}, 加速度: {acc}")
        print(f"执行: {execute}, 等待: {wait}")
        
        # 解析位置参数
        body_pos = self._parse_positions(body_positions)
        left_pos = self._parse_positions(left_positions)
        right_pos = self._parse_positions(right_positions)
        
        if body_pos is None or left_pos is None or right_pos is None:
            return False
        
        # 验证躯干关节数量（应该是2个自由度）
        if len(body_pos) != 2:
            print(f"错误: 躯干关节应该有2个自由度，但提供了{len(body_pos)}个")
            return False
        
        # 角度转弧度
        if degree:
            body_pos = self._degrees_to_radians(body_pos)
            left_pos = self._degrees_to_radians(left_pos)
            right_pos = self._degrees_to_radians(right_pos)
            print(f"转换后躯干位置(弧度): {[f'{x:.4f}' for x in body_pos]}")
            print(f"转换后左臂位置(弧度): {[f'{x:.4f}' for x in left_pos]}")
            print(f"转换后右臂位置(弧度): {[f'{x:.4f}' for x in right_pos]}")
        
        # 构造请求参数
        arm_requests = []
        
        # 检查躯干是否有非零增量
        if any(abs(pos) > 1e-6 for pos in body_pos):
            arm_requests.append({
                "arm_name": "body",
                "increment_joint_positions": body_pos
            })
        
        # 检查左臂是否有非零增量
        if any(abs(pos) > 1e-6 for pos in left_pos):
            arm_requests.append({
                "arm_name": "left",
                "increment_joint_positions": left_pos
            })
        
        # 检查右臂是否有非零增量
        if any(abs(pos) > 1e-6 for pos in right_pos):
            arm_requests.append({
                "arm_name": "right",
                "increment_joint_positions": right_pos
            })
        
        if not arm_requests:
            print("警告: 所有关节增量都为0，没有运动需求")
            return True
        
        movej_request = {
            "arm_requests": arm_requests,
            "speed": speed,
            "acc": acc,
            "need_traj": need_traj,
            "wait": wait,
            "execute": execute
        }
        
        # 添加use_arms参数
        if use_arms is not None:
            if isinstance(use_arms, str):
                # 如果是字符串，按逗号分割
                use_arms_list = [arm.strip() for arm in use_arms.split(',')]
            else:
                use_arms_list = use_arms
            movej_request["use_arms"] = use_arms_list
        
        print(f"\n发送请求: {json.dumps(movej_request, indent=2, ensure_ascii=False)}")
        
        try:
            response = requests.post(
                f"{self.server_url}/move_j",
                json=movej_request,
                headers={"Content-Type": "application/json"},
                timeout=30
            )
            
            if response.status_code == 200:
                result = response.json()
                print("\n✅ 请求成功!")
                print(f"响应: {json.dumps(result, indent=2, ensure_ascii=False)}")
                
                # 显示最终关节位置
                if result.get('final_joint_positions'):
                    final_pos = result['final_joint_positions']
                    print(f"\n最终关节位置: \n{[round(x, 4) for x in final_pos]}")
                    if degree:
                        final_degrees = [math.degrees(x) for x in final_pos]
                        print(f"最终关节位置(角度): \n{[round(x, 2) for x in final_degrees]}")
                
                return True
            else:
                print(f"\n❌ 请求失败，状态码: {response.status_code}")
                print(f"错误信息: {response.text}")
                return False
                
        except requests.exceptions.RequestException as e:
            print(f"\n❌ 请求异常: {e}")
            return False
    
    def move_j(self, body_positions, left_positions, right_positions, degree=False, speed=1.0, acc=1.0,
                   need_traj=True, wait=False, execute=True, use_arms=""):
        """绝对关节运动
        
        Args:
            body_positions: 躯干2个关节的绝对位置，逗号分隔的字符串
            left_positions: 左臂7个关节的绝对位置，逗号分隔的字符串
            right_positions: 右臂7个关节的绝对位置，逗号分隔的字符串
            degree: 是否使用角度单位（默认False，使用弧度）
            speed: 运动速度 (0.0-1.0)
            acc: 运动加速度 (0.0-1.0)
            need_traj: 是否需要轨迹数据
            wait: 是否等待执行完成
            execute: 是否执行运动
            use_arms: 指定要使用的机械臂列表，如"left_arm,right_arm"或"left_arm"
        
        Examples:
            python3 movej_test_client.py move_j "0,0" "0,0,0,0,0,0,0" "90,0,0,0,0,0,0" --degree
            python3 movej_test_client.py move_j "0,0" "0,0,0,0,0,0,0" "90,0,0,0,0,0,0" --use_arms="left_arm"
        """
        print(f"\n=== 绝对关节运动 ===\n")
        print(f"躯干目标位置: {body_positions}")
        print(f"左臂目标位置: {left_positions}")
        print(f"右臂目标位置: {right_positions}")
        print(f"单位: {'角度' if degree else '弧度'}")
        print(f"速度: {speed}, 加速度: {acc}")
        print(f"执行: {execute}, 等待: {wait}")
        
        # 解析位置参数
        body_pos = self._parse_positions(body_positions)
        left_pos = self._parse_positions(left_positions)
        right_pos = self._parse_positions(right_positions)
        
        if body_pos is None or left_pos is None or right_pos is None:
            return False
        
        # 验证躯干关节数量（应该是2个自由度）
        if len(body_pos) != 2:
            print(f"错误: 躯干关节应该有2个自由度，但提供了{len(body_pos)}个")
            return False
        
        # 角度转弧度
        if degree:
            body_pos = self._degrees_to_radians(body_pos)
            left_pos = self._degrees_to_radians(left_pos)
            right_pos = self._degrees_to_radians(right_pos)
            print(f"转换后躯干位置(弧度): {[f'{x:.4f}' for x in body_pos]}")
            print(f"转换后左臂位置(弧度): {[f'{x:.4f}' for x in left_pos]}")
            print(f"转换后右臂位置(弧度): {[f'{x:.4f}' for x in right_pos]}")
        
        # 构造请求参数
        arm_requests = []
        
        # 检查躯干是否有非零位置（如果不是全零，则添加躯干请求）
        # if any(abs(pos) > 1e-6 for pos in body_pos):
        arm_requests.append({
            "arm_name": "body",
            "joint_positions": body_pos
        })
        
        # 添加左臂请求
        arm_requests.append({
            "arm_name": "left",
            "joint_positions": left_pos
        })
        
        # 添加右臂请求
        arm_requests.append({
            "arm_name": "right",
            "joint_positions": right_pos
        })
        
        movej_request = {
            "arm_requests": arm_requests,
            "speed": speed,
            "acc": acc,
            "need_traj": need_traj,
            "wait": wait,
            "execute": execute,
            "use_arms": use_arms.split(",")
        }
        
        print(f"\n发送请求: {json.dumps(movej_request, indent=2, ensure_ascii=False)}")
        
        try:
            response = requests.post(
                f"{self.server_url}/move_j",
                json=movej_request,
                headers={"Content-Type": "application/json"},
                timeout=120
            )
            
            if response.status_code == 200:
                result = response.json()
                print("\n✅ 请求成功!")
                print(f"响应: {json.dumps(result, indent=2, ensure_ascii=False)}")
                
                # 显示最终关节位置
                if result.get('final_joint_positions'):
                    final_pos = result['final_joint_positions']
                    print(f"\n最终关节位置: {[f'{x:.4f}' for x in final_pos]}")
                    if degree:
                        final_degrees = [math.degrees(x) for x in final_pos]
                        print(f"最终关节位置(角度): {[f'{x:.2f}' for x in final_degrees]}")
                
                return result
            else:
                print(f"\n❌ 请求失败，状态码: {response.status_code}")
                print(f"错误信息: {response.text}")
                return False
                
        except requests.exceptions.RequestException as e:
            print(f"\n❌ 请求异常: {e}")
            return False
    
    def find_apriltag(self, arm="left", marker_id=0, repeat_times=1, repeat_time_interval=0.1):
        """查找AprilTag标记
        
        Args:
            arm: 使用哪个手臂的相机进行检测，left 或 right（默认left）
            marker_id: 要查找的AprilTag标记ID（默认0）
            repeat_times: 重复查找次数（默认4次）
            repeat_time_interval: 每次重复查找的时间间隔（秒）（默认0.1秒）
        
        Examples:
            python3 labbot_manager.py find_apriltag --arm=left --marker_id=5
            python3 labbot_manager.py find_apriltag --arm=right --marker_id=10
        """
        print(f"\n=== 查找AprilTag标记 ===\n")
        print(f"使用手臂: {arm}")
        print(f"标记ID: {marker_id}")
        print(f"重复次数: {repeat_times}")
        print(f"重复时间间隔: {repeat_time_interval}秒")
        
        # 验证手臂参数
        if arm not in ["left", "right"]:
            print(f"❌ 无效的手臂名称: {arm}，必须是 'left' 或 'right'")
            return False
        
        # 构造请求参数
        apriltag_request = {
            "arm": arm,
            "marker_id": int(marker_id),
            "repeat_times": int(repeat_times),
            "repeat_time_interval": float(repeat_time_interval)
        }
        
        print(f"\n发送请求: {json.dumps(apriltag_request, indent=2, ensure_ascii=False)}")
        
        try:
            response = requests.post(
                f"{self.server_url}/find_apriltag",
                json=apriltag_request,
                headers={"Content-Type": "application/json"},
                timeout=30
            )
            
            if response.status_code == 200:
                result = response.json()
                print("\n✅ 请求成功!")
                print(f"响应: {json.dumps(result, indent=2, ensure_ascii=False)}")
                
                # 显示AprilTag信息
                if result.get('code') == 200:  # ErrorCode.Success
                    position = result.get('position', [])
                    quaternion = result.get('quaternion', [])
                    rotation_matrix = result.get('rotation_matrix', [])
                    
                    if position:
                        print(f"\n📍 AprilTag位置 (机器人坐标系):")
                        print(f"   X: {position[0]:.4f} m")
                        print(f"   Y: {position[1]:.4f} m")
                        print(f"   Z: {position[2]:.4f} m")
                    
                    if quaternion:
                        print(f"\n🔄 AprilTag姿态四元数:")
                        print(f"   X: {quaternion[0]:.4f}")
                        print(f"   Y: {quaternion[1]:.4f}")
                        print(f"   Z: {quaternion[2]:.4f}")
                        print(f"   W: {quaternion[3]:.4f}")
                    
                    if rotation_matrix:
                        print(f"\n📐 旋转矩阵:")
                        for i, row in enumerate(rotation_matrix):
                            print(f"   [{row[0]:8.4f}, {row[1]:8.4f}, {row[2]:8.4f}]")
                else:
                    print(f"\n⚠️ 未找到AprilTag: {result.get('msg', '未知错误')}")
                
                return True
            else:
                print(f"\n❌ 请求失败，状态码: {response.status_code}")
                print(f"错误信息: {response.text}")
                return False
                
        except requests.exceptions.RequestException as e:
            print(f"\n❌ 请求异常: {e}")
            return False
    
    def aim_at_apriltag(self, arm="left", marker_id=0, distance=0.3, execute=False):
        """
        瞄准AprilTag标记并移动到指定距离
        
        Args:
            arm: 使用哪个手臂进行瞄准，left 或 right（默认left）
            marker_id: 要瞄准的AprilTag标记ID（默认0）
            distance: 距离AprilTag的目标距离，单位米（默认0.3）
            execute: 是否执行运动，False时只规划不执行（默认True）
        
        Examples:
            python3 labbot_manager.py aim_at_apriltag --arm=left --marker_id=5 --distance=0.2
            python3 labbot_manager.py aim_at_apriltag --arm=right --marker_id=10 --distance=0.5
            python3 labbot_manager.py aim_at_apriltag --arm=left --marker_id=0 --distance=0.3 --execute=False
        """
        print(f"\n=== 瞄准AprilTag标记 ===\n")
        print(f"使用手臂: {arm}")
        print(f"标记ID: {marker_id}")
        print(f"目标距离: {distance} 米")
        print(f"执行模式: {'执行运动' if execute else '仅规划'}")
        
        # 验证手臂参数
        if arm not in ["left", "right"]:
            print(f"❌ 无效的手臂名称: {arm}，必须是 'left' 或 'right'")
            return False
        
        # 验证距离参数
        try:
            distance = float(distance)
            if distance <= 0:
                print(f"❌ 无效的距离值: {distance}，必须大于0")
                return False
        except ValueError:
            print(f"❌ 无效的距离值: {distance}，必须是数字")
            return False
        
        # 构造请求参数
        aim_request = {
            "arm": arm,
            "marker_id": int(marker_id),
            "distance": distance,
            "execute": execute
        }
        
        print(f"\n发送请求: {json.dumps(aim_request, indent=2, ensure_ascii=False)}")
        
        try:
            response = requests.post(
                f"{self.server_url}/aim_at_apriltag",
                json=aim_request,
                headers={"Content-Type": "application/json"},
                timeout=60  # 瞄准操作可能需要更长时间
            )
            
            if response.status_code == 200:
                result = response.json()
                print("\n✅ 请求成功!")
                print(f"响应: {json.dumps(result, indent=2, ensure_ascii=False)}")
                
                # 显示瞄准结果
                if result.get('code') == 200:  # ErrorCode.Success
                    apriltag_position = result.get('apriltag_position', [])
                    aim_position = result.get('aim_position', [])
                    actual_distance = result.get('distance', 0.0)
                    trajectory_path = result.get('trajectory_path', '')
                    
                    if apriltag_position:
                        print(f"\n📍 AprilTag位置 (机器人坐标系):")
                        print(f"   X: {apriltag_position[0]:.4f} m")
                        print(f"   Y: {apriltag_position[1]:.4f} m")
                        print(f"   Z: {apriltag_position[2]:.4f} m")
                    
                    if aim_position:
                        print(f"\n🎯 瞄准位置 (机器人坐标系):")
                        print(f"   X: {aim_position[0]:.4f} m")
                        print(f"   Y: {aim_position[1]:.4f} m")
                        print(f"   Z: {aim_position[2]:.4f} m")
                    
                    print(f"\n📏 实际距离: {actual_distance:.3f} m")
                    
                    if trajectory_path:
                        print(f"\n💾 轨迹文件: {trajectory_path}")
                    
                    print(f"\n🎉 成功瞄准AprilTag标记 {marker_id}!")
                else:
                    print(f"\n⚠️ 瞄准失败: {result.get('msg', '未知错误')}")
                
                return True
            else:
                print(f"\n❌ 请求失败，状态码: {response.status_code}")
                print(f"错误信息: {response.text}")
                return False
                
        except requests.exceptions.RequestException as e:
            print(f"\n❌ 请求异常: {e}")
            return False
    
    def action_back(self):
        """反向执行上一个轨迹
        
        Examples:
            python3 labbot_manager.py action_back
        """
        print(f"\n=== 反向执行上一个轨迹 ===\n")
        
        # 构造请求参数（ActionBackRequest为空）
        action_back_request = {}
        
        print(f"发送请求: {json.dumps(action_back_request, indent=2, ensure_ascii=False)}")
        
        try:
            response = requests.post(
                f"{self.server_url}/action_back",
                json=action_back_request,
                headers={"Content-Type": "application/json"},
                timeout=120  # 反向执行可能需要较长时间
            )
            
            if response.status_code == 200:
                result = response.json()
                print("\n✅ 请求成功!")
                print(f"响应: {json.dumps(result, indent=2, ensure_ascii=False)}")
                
                # 显示执行结果
                if result.get('code') == 200:  # ErrorCode.Success
                    task_found = result.get('task_found', False)
                    task_timestamp = result.get('task_timestamp', '')
                    task_type = result.get('task_type', '')
                    original_trajectory_path = result.get('original_trajectory_path', '')
                    reversed_trajectory_path = result.get('reversed_trajectory_path', '')
                    smoothed_trajectory_path = result.get('smoothed_trajectory_path', '')
                    execution_result = result.get('execution_result', '')
                    
                    if task_found:
                        print(f"\n📋 找到任务:")
                        print(f"   时间戳: {task_timestamp}")
                        print(f"   任务类型: {task_type}")
                        
                        if original_trajectory_path:
                            print(f"\n📁 原始轨迹文件: {original_trajectory_path}")
                        
                        if reversed_trajectory_path:
                            print(f"🔄 反向轨迹文件: {reversed_trajectory_path}")
                        
                        if smoothed_trajectory_path:
                            print(f"✨ 平滑轨迹文件: {smoothed_trajectory_path}")
                        
                        if execution_result:
                            print(f"\n🎯 执行结果: {execution_result}")
                        
                        print(f"\n🎉 成功反向执行轨迹!")
                    else:
                        print(f"\n⚠️ 未找到可反向执行的任务")
                else:
                    print(f"\n⚠️ 反向执行失败: {result.get('msg', '未知错误')}")
                
                return True
            else:
                print(f"\n❌ 请求失败，状态码: {response.status_code}")
                print(f"错误信息: {response.text}")
                return False
                
        except requests.exceptions.RequestException as e:
            print(f"\n❌ 请求异常: {e}")
            return False

    def clear_fault(self, arm_name="left"):
        """清除机器人故障
        
        Args:
            arm_name: 手臂名称，left 或 right（默认left）
        
        Examples:
            python3 labbot_manager.py clear_fault --arm_name=left
            python3 labbot_manager.py clear_fault --arm_name=right
        """
        print(f"\n=== 清除机器人故障 ===\n")
        print(f"手臂名称: {arm_name}")
        
        # 验证手臂参数
        if arm_name not in ["left", "right"]:
            print(f"❌ 无效的手臂名称: {arm_name}，必须是 'left' 或 'right'")
            return False
        
        # 构造请求参数
        clear_fault_request = {
            "arm_name": arm_name
        }
        
        print(f"\n发送请求: {json.dumps(clear_fault_request, indent=2, ensure_ascii=False)}")
        
        try:
            response = requests.post(
                f"{self.server_url}/clear_fault",
                json=clear_fault_request,
                headers={"Content-Type": "application/json"},
                timeout=30
            )
            
            if response.status_code == 200:
                result = response.json()
                print("\n✅ 请求成功!")
                print(f"响应: {json.dumps(result, indent=2, ensure_ascii=False)}")
                
                # 显示清除故障结果
                if result.get('code') == 200:  # ErrorCode.Success
                    print(f"\n🎉 成功清除 {arm_name} 手臂故障!")
                else:
                    print(f"\n⚠️ 清除故障失败: {result.get('msg', '未知错误')}")
                
                return True
            else:
                print(f"\n❌ 请求失败，状态码: {response.status_code}")
                print(f"错误信息: {response.text}")
                return False
                
        except requests.exceptions.RequestException as e:
            print(f"\n❌ 请求异常: {e}")
            return False
    
    def execute_primitive(self, arm="left", primitive_name="", input_params="{}", block_until_started=True):
        """执行机器人原语命令
        
        Args:
            arm: 手臂名称，left 或 right（默认left）
            primitive_name: 原语名称
            input_params: 输入参数的JSON字符串（默认为空对象）
            block_until_started: 是否阻塞直到开始执行（默认True）
        
        Examples:
            python3 labbot_manager.py execute_primitive --arm=left --primitive_name="move_to_pose" --input_params='{"x":0.5,"y":0.0,"z":0.3}'
            python3 labbot_manager.py execute_primitive --arm=right --primitive_name="gripper_open" --input_params='{}'
        """
        print(f"\n=== 执行机器人原语命令 ===\n")
        print(f"手臂名称: {arm}")
        print(f"原语名称: {primitive_name}")
        print(f"输入参数: {input_params}")
        print(f"阻塞直到开始: {block_until_started}")
        
        # 验证手臂参数
        if arm not in ["left", "right"]:
            print(f"❌ 无效的手臂名称: {arm}，必须是 'left' 或 'right'")
            return False
        
        # 验证原语名称
        if not primitive_name.strip():
            print(f"❌ 原语名称不能为空")
            return False
        
        # 解析输入参数
        if isinstance(input_params, str):
            try:
                params_dict = json.loads(input_params)
                if not isinstance(params_dict, dict):
                    print(f"❌ 输入参数必须是有效的JSON对象")
                    return False
            except json.JSONDecodeError as e:
                print(f"❌ 输入参数JSON格式错误: {e}")
                return False
        elif isinstance(input_params, dict):
            params_dict = input_params
        else:
            print(f"❌ 输入参数必须是字符串或字典")
            return False
        
        # 构造请求参数
        execute_primitive_request = {
            "arm": arm,
            "primitive_name": primitive_name,
            "input_params": params_dict,
            "block_until_started": bool(block_until_started)
        }
        
        print(f"\n发送请求: {json.dumps(execute_primitive_request, indent=2, ensure_ascii=False)}")
        
        try:
            response = requests.post(
                f"{self.server_url}/execute_primitive",
                json=execute_primitive_request,
                headers={"Content-Type": "application/json"},
                timeout=60  # 原语执行可能需要较长时间
            )
            
            if response.status_code == 200:
                result = response.json()
                print("\n✅ 请求成功!")
                print(f"响应: {json.dumps(result, indent=2, ensure_ascii=False)}")
                
                # 显示执行结果
                if result.get('code') == 200:  # ErrorCode.Success
                    print(f"\n🎉 成功执行原语命令!")
                    print(f"   手臂: {arm}")
                    print(f"   原语: {primitive_name}")
                    print(f"   参数: {params_dict}")
                else:
                    print(f"\n⚠️ 原语执行失败: {result.get('msg', '未知错误')}")
                
                return True
            else:
                print(f"\n❌ 请求失败，状态码: {response.status_code}")
                print(f"错误信息: {response.text}")
                return False
                
        except requests.exceptions.RequestException as e:
            print(f"\n❌ 请求异常: {e}")
            return False
    
    def execute_plan(self, arm, plan_name):
        """执行轨迹计划
        
        Args:
            arm: 手臂名称，left 或 right
            plan_name: 计划名称
        
        Examples:
            python3 labbot_manager.py execute_plan left "my_plan"
            python3 labbot_manager.py execute_plan right "grasp_plan"
        """
        print(f"\n=== 执行轨迹计划 ===\n")
        print(f"手臂: {arm}")
        print(f"计划名称: {plan_name}")
        
        # 验证手臂参数
        if arm not in ["left", "right"]:
            print(f"❌ 无效的手臂名称: {arm}，必须是 'left' 或 'right'")
            return False
        
        # 验证计划名称
        if not plan_name or not isinstance(plan_name, str):
            print(f"❌ 无效的计划名称: {plan_name}")
            return False
        
        # 构造请求参数
        execute_plan_request = {
            "arm": arm,
            "plan_name": plan_name
        }
        
        print(f"\n发送请求: {json.dumps(execute_plan_request, indent=2, ensure_ascii=False)}")
        
        try:
            response = requests.post(
                f"{self.server_url}/execute_plan",
                json=execute_plan_request,
                headers={"Content-Type": "application/json"},
                timeout=120  # 轨迹执行可能需要较长时间
            )
            
            if response.status_code == 200:
                result = response.json()
                print("\n✅ 请求成功!")
                print(f"响应: {json.dumps(result, indent=2, ensure_ascii=False)}")
                
                # 显示执行结果
                if result.get('code') == 200:  # ErrorCode.Success
                    execution_time = result.get('execution_time', 0.0)
                    trajectory_points = result.get('trajectory_points', 0)
                    
                    print(f"\n🎉 成功执行轨迹计划!")
                    print(f"   手臂: {arm}")
                    print(f"   计划名称: {plan_name}")
                    if trajectory_points > 0:
                        print(f"   轨迹点数: {trajectory_points}")
                    if execution_time > 0:
                        print(f"   执行时间: {execution_time:.2f} 秒")
                else:
                    print(f"\n⚠️ 轨迹执行失败: {result.get('msg', '未知错误')}")
                
                return True
            else:
                print(f"\n❌ 请求失败，状态码: {response.status_code}")
                print(f"错误信息: {response.text}")
                return False
                
        except requests.exceptions.RequestException as e:
            print(f"\n❌ 请求异常: {e}")
            return False
    
    def status(self, arm="all", remote_host: str=None):
        """获取指定手臂的关节位置和末端执行器位姿
        
        Args:
            arm: 要获取状态的手臂名称，left 或 right 或 all（默认all）
        
        Examples:
            python3 labbot_manager.py status --arm=left
            python3 labbot_manager.py status --arm=right
            python3 labbot_manager.py status --arm=all
        """
        print(f"\n=== 获取机器人状态 ===\n")
        print(f"手臂: {arm}")
        
        # 验证手臂参数
        if arm not in ["left", "right", "all"]:
            print(f"❌ 无效的手臂名称: {arm}，必须是 'left' 或 'right' 或 'all'")
            return False
        
        # 构造请求参数
        joint_states_request = {
            "arm": arm
        }
        
        print(f"\n发送请求: {json.dumps(joint_states_request, indent=2, ensure_ascii=False)}")
        
        try:
            if remote_host is not None:
                server_url = self.get_remote_server_url(remote_host)
            else:
                server_url = self.server_url
            response = requests.post(
                f"{server_url}/get_robot_status",
                json=joint_states_request,
                headers={"Content-Type": "application/json"},
                timeout=30
            )
            
            if response.status_code == 200:
                result = response.json()
                print("\n✅ 请求成功!")
                print(f"响应: {json.dumps(result, indent=2, ensure_ascii=False)}")
                
                # 显示关节位置信息
                if result.get('code') == 200:  # ErrorCode.Success
                    joint_states = result.get('joint_states', [])
                    
                    if joint_states:
                        print(f"\n🤖 {arm.upper()}手臂关节位置 (弧度):")
                        for i, pos in enumerate(joint_states):
                            print(f"   关节{i+1}: {pos:.4f} rad")
                        
                        # 转换为角度显示
                        joint_degrees = [math.degrees(pos) for pos in joint_states]
                        print(f"\n🤖 {arm.upper()}手臂关节位置 (角度):")
                        for i, pos in enumerate(joint_degrees):
                            print(f"   关节{i+1}: {pos:.2f}°")

                        # 显示为move_j指令参数
                        joint_states_str = [str(x) for x in joint_states]
                        print(f"\n📋 Move_J参数格式:")
                        print(f"\"{','.join(joint_states_str[:2])}\" \"{','.join(joint_states_str[2:9])}\" \"{','.join(joint_states_str[9:])}\"")
                    else:
                        print(f"\n⚠️ 未获取到关节位置数据")
                    
                    # 显示左手末端执行器位姿
                    if arm in ["left", "all"]:
                        left_arm_data = result.get('left_arm', {})
                        left_robot_tf = left_arm_data.get('robot_tf', {})
                        left_arm_tf = left_arm_data.get('arm_tf', {})
                        
                        # 获取robot_tf坐标系下的TCP位姿
                        left_tcp_robot = left_robot_tf.get('tcp', {})
                        left_position = left_tcp_robot.get('position', [])
                        left_quaternion = left_tcp_robot.get('orientation', [])
                        
                        # 获取arm_tf坐标系下的TCP位姿
                        left_tcp_arm = left_arm_tf.get('tcp', {})
                        left_position_on_arm_tf = left_tcp_arm.get('position', [])
                        left_quaternion_on_arm_tf = left_tcp_arm.get('orientation', [])
                        
                        if left_arm_data:
                            print(f"\n🤖 左手末端执行器位姿:")
                            
                            # 机器人坐标系下的位姿
                            print(f"   📍 相对于机器人坐标系:")
                            # TCP位姿
                            left_tcp_robot = left_robot_tf.get('tcp', {})
                            tcp_pos = left_tcp_robot.get('position', [])
                            tcp_ori = left_tcp_robot.get('orientation', [])
                            if tcp_pos and tcp_ori:
                                print(f"      TCP位置 [x,y,z]: [{tcp_pos[0]:.6f},{tcp_pos[1]:.6f},{tcp_pos[2]:.6f}] m")
                                print(f"      TCP姿态 [x,y,z,w]: [{tcp_ori[0]:.6f},{tcp_ori[1]:.6f},{tcp_ori[2]:.6f},{tcp_ori[3]:.6f}]")
                                # 显示yaw, pitch, roll角度
                                tcp_yaw = left_tcp_robot.get('yaw', 0.0)
                                tcp_pitch = left_tcp_robot.get('pitch', 0.0)
                                tcp_roll = left_tcp_robot.get('roll', 0.0)
                                print(f"      TCP角度 [yaw,pitch,roll]: [{math.degrees(tcp_yaw):.2f}°,{math.degrees(tcp_pitch):.2f}°,{math.degrees(tcp_roll):.2f}°]")
                            # 法兰位姿
                            left_flange_robot = left_robot_tf.get('flange', {})
                            flange_pos = left_flange_robot.get('position', [])
                            flange_ori = left_flange_robot.get('orientation', [])
                            if flange_pos and flange_ori:
                                print(f"      法兰位置 [x,y,z]: [{flange_pos[0]:.6f},{flange_pos[1]:.6f},{flange_pos[2]:.6f}] m")
                                print(f"      法兰姿态 [x,y,z,w]: [{flange_ori[0]:.6f},{flange_ori[1]:.6f},{flange_ori[2]:.6f},{flange_ori[3]:.6f}]")
                                # 显示yaw, pitch, roll角度
                                flange_yaw = left_flange_robot.get('yaw', 0.0)
                                flange_pitch = left_flange_robot.get('pitch', 0.0)
                                flange_roll = left_flange_robot.get('roll', 0.0)
                                print(f"      法兰角度 [yaw,pitch,roll]: [{math.degrees(flange_yaw):.2f}°,{math.degrees(flange_pitch):.2f}°,{math.degrees(flange_roll):.2f}°]")
                            
                            # 手臂坐标系下的位姿
                            print(f"   📍 相对于手臂坐标系:")
                            # TCP位姿
                            left_tcp_arm = left_arm_tf.get('tcp', {})
                            tcp_pos_arm = left_tcp_arm.get('position', [])
                            tcp_ori_arm = left_tcp_arm.get('orientation', [])
                            if tcp_pos_arm and tcp_ori_arm:
                                print(f"      TCP位置 [x,y,z]: [{tcp_pos_arm[0]:.6f},{tcp_pos_arm[1]:.6f},{tcp_pos_arm[2]:.6f}] m")
                                print(f"      TCP姿态 [x,y,z,w]: [{tcp_ori_arm[0]:.6f},{tcp_ori_arm[1]:.6f},{tcp_ori_arm[2]:.6f},{tcp_ori_arm[3]:.6f}]")
                                # 显示yaw, pitch, roll角度
                                tcp_yaw_arm = left_tcp_arm.get('yaw', 0.0)
                                tcp_pitch_arm = left_tcp_arm.get('pitch', 0.0)
                                tcp_roll_arm = left_tcp_arm.get('roll', 0.0)
                                print(f"      TCP角度 [yaw,pitch,roll]: [{math.degrees(tcp_yaw_arm):.2f}°,{math.degrees(tcp_pitch_arm):.2f}°,{math.degrees(tcp_roll_arm):.2f}°]")
                            # 法兰位姿
                            left_flange_arm = left_arm_tf.get('flange', {})
                            flange_pos_arm = left_flange_arm.get('position', [])
                            flange_ori_arm = left_flange_arm.get('orientation', [])
                            if flange_pos_arm and flange_ori_arm:
                                print(f"      法兰位置 [x,y,z]: [{flange_pos_arm[0]:.6f},{flange_pos_arm[1]:.6f},{flange_pos_arm[2]:.6f}] m")
                                print(f"      法兰姿态 [x,y,z,w]: [{flange_ori_arm[0]:.6f},{flange_ori_arm[1]:.6f},{flange_ori_arm[2]:.6f},{flange_ori_arm[3]:.6f}]")
                                # 显示yaw, pitch, roll角度
                                flange_yaw_arm = left_flange_arm.get('yaw', 0.0)
                                flange_pitch_arm = left_flange_arm.get('pitch', 0.0)
                                flange_roll_arm = left_flange_arm.get('roll', 0.0)
                                print(f"      法兰角度 [yaw,pitch,roll]: [{math.degrees(flange_yaw_arm):.2f}°,{math.degrees(flange_pitch_arm):.2f}°,{math.degrees(flange_roll_arm):.2f}°]")
                        elif arm == "left":
                            print(f"\n⚠️ 未获取到左手末端执行器位姿数据")
                            
                    
                    # 显示右手末端执行器位姿
                    if arm in ["right", "all"]:
                        right_arm_data = result.get('right_arm', {})
                        right_robot_tf = right_arm_data.get('robot_tf', {})
                        right_arm_tf = right_arm_data.get('arm_tf', {})
                        
                        # 获取robot_tf坐标系下的TCP位姿
                        right_tcp_robot = right_robot_tf.get('tcp', {})
                        right_position = right_tcp_robot.get('position', [])
                        right_quaternion = right_tcp_robot.get('orientation', [])
                        
                        # 获取arm_tf坐标系下的TCP位姿
                        right_tcp_arm = right_arm_tf.get('tcp', {})
                        right_position_on_arm_tf = right_tcp_arm.get('position', [])
                        right_quaternion_on_arm_tf = right_tcp_arm.get('orientation', [])
                        
                        if right_arm_data:
                            print(f"\n🤖 右手末端执行器位姿:")
                            
                            # 机器人坐标系下的位姿
                            print(f"   📍 相对于机器人坐标系:")
                            # TCP位姿
                            right_tcp_robot = right_robot_tf.get('tcp', {})
                            tcp_pos = right_tcp_robot.get('position', [])
                            tcp_ori = right_tcp_robot.get('orientation', [])
                            if tcp_pos and tcp_ori:
                                print(f"      TCP位置 [x,y,z]: [{tcp_pos[0]:.6f},{tcp_pos[1]:.6f},{tcp_pos[2]:.6f}] m")
                                print(f"      TCP姿态 [x,y,z,w]: [{tcp_ori[0]:.6f},{tcp_ori[1]:.6f},{tcp_ori[2]:.6f},{tcp_ori[3]:.6f}]")
                                # 显示yaw, pitch, roll角度
                                tcp_yaw = right_tcp_robot.get('yaw', 0.0)
                                tcp_pitch = right_tcp_robot.get('pitch', 0.0)
                                tcp_roll = right_tcp_robot.get('roll', 0.0)
                                print(f"      TCP角度 [yaw,pitch,roll]: [{math.degrees(tcp_yaw):.2f}°,{math.degrees(tcp_pitch):.2f}°,{math.degrees(tcp_roll):.2f}°]")
                            # 法兰位姿
                            right_flange_robot = right_robot_tf.get('flange', {})
                            flange_pos = right_flange_robot.get('position', [])
                            flange_ori = right_flange_robot.get('orientation', [])
                            if flange_pos and flange_ori:
                                print(f"      法兰位置 [x,y,z]: [{flange_pos[0]:.6f},{flange_pos[1]:.6f},{flange_pos[2]:.6f}] m")
                                print(f"      法兰姿态 [x,y,z,w]: [{flange_ori[0]:.6f},{flange_ori[1]:.6f},{flange_ori[2]:.6f},{flange_ori[3]:.6f}]")
                                # 显示yaw, pitch, roll角度
                                flange_yaw = right_flange_robot.get('yaw', 0.0)
                                flange_pitch = right_flange_robot.get('pitch', 0.0)
                                flange_roll = right_flange_robot.get('roll', 0.0)
                                print(f"      法兰角度 [yaw,pitch,roll]: [{math.degrees(flange_yaw):.2f}°,{math.degrees(flange_pitch):.2f}°,{math.degrees(flange_roll):.2f}°]")
                            
                            # 手臂坐标系下的位姿
                            print(f"   📍 相对于手臂坐标系:")
                            # TCP位姿
                            right_tcp_arm = right_arm_tf.get('tcp', {})
                            tcp_pos_arm = right_tcp_arm.get('position', [])
                            tcp_ori_arm = right_tcp_arm.get('orientation', [])
                            if tcp_pos_arm and tcp_ori_arm:
                                print(f"      TCP位置 [x,y,z]: [{tcp_pos_arm[0]:.6f},{tcp_pos_arm[1]:.6f},{tcp_pos_arm[2]:.6f}] m")
                                print(f"      TCP姿态 [x,y,z,w]: [{tcp_ori_arm[0]:.6f},{tcp_ori_arm[1]:.6f},{tcp_ori_arm[2]:.6f},{tcp_ori_arm[3]:.6f}]")
                                # 显示yaw, pitch, roll角度
                                tcp_yaw_arm = right_tcp_arm.get('yaw', 0.0)
                                tcp_pitch_arm = right_tcp_arm.get('pitch', 0.0)
                                tcp_roll_arm = right_tcp_arm.get('roll', 0.0)
                                print(f"      TCP角度 [yaw,pitch,roll]: [{math.degrees(tcp_yaw_arm):.2f}°,{math.degrees(tcp_pitch_arm):.2f}°,{math.degrees(tcp_roll_arm):.2f}°]")
                            # 法兰位姿
                            right_flange_arm = right_arm_tf.get('flange', {})
                            flange_pos_arm = right_flange_arm.get('position', [])
                            flange_ori_arm = right_flange_arm.get('orientation', [])
                            if flange_pos_arm and flange_ori_arm:
                                print(f"      法兰位置 [x,y,z]: [{flange_pos_arm[0]:.6f},{flange_pos_arm[1]:.6f},{flange_pos_arm[2]:.6f}] m")
                                print(f"      法兰姿态 [x,y,z,w]: [{flange_ori_arm[0]:.6f},{flange_ori_arm[1]:.6f},{flange_ori_arm[2]:.6f},{flange_ori_arm[3]:.6f}]")
                                # 显示yaw, pitch, roll角度
                                flange_yaw_arm = right_flange_arm.get('yaw', 0.0)
                                flange_pitch_arm = right_flange_arm.get('pitch', 0.0)
                                flange_roll_arm = right_flange_arm.get('roll', 0.0)
                                print(f"      法兰角度 [yaw,pitch,roll]: [{math.degrees(flange_yaw_arm):.2f}°,{math.degrees(flange_pitch_arm):.2f}°,{math.degrees(flange_roll_arm):.2f}°]")
                        elif arm == "right":
                            print(f"\n⚠️ 未获取到右手末端执行器位姿数据")
                    
                    print(f"\n🎉 成功获取机器人状态信息!")
                else:
                    print(f"\n⚠️ 获取机器人状态失败: {result.get('msg', '未知错误')}")
                
                return result
            else:
                print(f"\n❌ 请求失败，状态码: {response.status_code}")
                print(f"错误信息: {response.text}")
                return False
                
        except requests.exceptions.RequestException as e:
            print(f"\n❌ 请求异常: {e}")
            return False
    
    def move_l(self, arm_name, position, orientation=[0,0,0], ref_frame="tcp", speed=0.02, acc=0.02, 
               need_traj=False, execute=True, wait=True):
        """直线运动（MoveL）
        
        Args:
            arm_name: 手臂名称，left 或 right
            position: 位置增量，逗号分隔的字符串，如 "0.0,0.0,0.04"
            orientation: 姿态增量，逗号分隔的字符串，如 "0,0,0"
            ref_frame: 参考坐标系，格式为 "tcp, world"，默认 "tcp"
            speed: 运动速度 (默认0.02)
            acc: 运动加速度 (默认0.02)
            need_traj: 是否需要轨迹数据 (默认False)
            execute: 是否执行运动 (默认True)
            wait: 是否等待执行完成 (默认True)
        
        Examples:
            python3 labbot_manager_base.py move_l left "0.0,0.0,0.04" "0,0,0"
            python3 labbot_manager_base.py move_l right "0.0,0.0,0.04" "0,0,0" --ref_frame="world"
            python3 labbot_manager_base.py move_l left "0.0,0.0,0.04" "0,0,0" --speed=0.05 --execute=False
        """
        print(f"\n=== 直线运动（MoveL） ===\n")
        print(f"手臂: {arm_name}")
        print(f"位置增量: {position}")
        print(f"姿态增量: {orientation}")
        print(f"参考坐标系: {ref_frame}")
        print(f"速度: {speed}, 加速度: {acc}")
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
        
        # 验证参考坐标系
        valid_ref_frames = ["tcp", "world"]
        if ref_frame not in valid_ref_frames:
            print(f"❌ 无效的参考坐标系: {ref_frame}，必须是 {valid_ref_frames} 中的一个")
            return False
        
        # 构造请求参数
        move_l_request = {
            "arm_requests": [
                {
                    "arm_name": arm_name,
                    "position": position_list,
                    "orientation": orientation_list,
                    "ref_frame": ref_frame
                }
            ],
            "speed": float(speed),
            "acc": float(acc),
            "need_traj": bool(need_traj),
            "execute": bool(execute),
            "wait": bool(wait)
        }
        
        print(f"\n发送请求: {json.dumps(move_l_request, indent=2, ensure_ascii=False)}")
        
        try:
            response = requests.post(
                f"{self.server_url}/move_l",
                json=move_l_request,
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
                    
                    print(f"\n🎉 MoveL运动完成!")
                else:
                    print(f"\n⚠️ MoveL运动失败: {result.get('msg', '未知错误')}")
                
                return True
            else:
                print(f"\n❌ 请求失败，状态码: {response.status_code}")
                print(f"错误信息: {response.text}")
                return False
                
        except requests.exceptions.RequestException as e:
            print(f"\n❌ 请求异常: {e}")
            return False
    
    def contact(self, arm_name, contact_coord, contact_dir, speed=0.01, max_contact_force=10.0, wait=True):
        """接触操作（Contact）
        
        Args:
            arm_name: 手臂名称，left 或 right
            contact_coord: 接触坐标系, tcp或world
            contact_dir: 接触方向，逗号分隔的字符串，如 "0,0,-1"
            speed: 运动速度 (默认0.02)
            max_contact_force: 最大接触力 (默认10.0)
            wait: 是否等待执行完成 (默认True)
        
        Examples:
            python3 labbot_manager_base.py contact left "0.0,0.0,0.04" "0,0,-1"
            python3 labbot_manager_base.py contact right "0.0,0.0,0.04" "0,0,-1" --speed=0.05 --max_contact_force=15.0
        """
        print(f"\n=== 接触操作（Contact） ===\n")
        print(f"手臂: {arm_name}")
        print(f"接触坐标: {contact_coord}")
        print(f"接触方向: {contact_dir}")
        print(f"速度: {speed}, 最大接触力: {max_contact_force}")
        print(f"等待: {wait}")
        
        # 验证手臂参数
        if arm_name not in ["left", "right"]:
            print(f"❌ 无效的手臂名称: {arm_name}，必须是 'left' 或 'right'")
            return False
        
        # 解析接触坐标参数
        try:
            if not contact_coord in ["world", "tcp"]:
                print(f"❌ 无效的接触坐标参数: {contact_coord}，必须是 'world' 或 'tcp'")
                return False
        except (ValueError, TypeError) as e:
            print(f"❌ 接触坐标解析错误: {e}")
            return False
        
        # 解析接触方向参数
        try:
            if isinstance(contact_dir, str):
                contact_dir_list = [float(x.strip()) for x in contact_dir.split(',')]
            elif isinstance(contact_dir, (list, tuple)):
                contact_dir_list = [float(x) for x in contact_dir]
            else:
                raise ValueError(f"不支持的接触方向参数类型: {type(contact_dir)}")
            
            if len(contact_dir_list) != 3:
                print(f"❌ 接触方向参数应该有3个值，但得到{len(contact_dir_list)}个")
                return False
        except (ValueError, TypeError) as e:
            print(f"❌ 接触方向解析错误: {e}")
            return False
        
        # 构造请求参数
        contact_request = {
            "arm_name": arm_name,
            "contact_coord": contact_coord,
            "contact_dir": contact_dir_list,
            "speed": float(speed),
            "max_contact_force": float(max_contact_force),
            "wait": bool(wait)
        }
        
        print(f"\n发送请求: {json.dumps(contact_request, indent=2, ensure_ascii=False)}")
        
        try:
            response = requests.post(
                f"{self.server_url}/contact",
                json=contact_request,
                headers={"Content-Type": "application/json"},
                timeout=120
            )
            
            if response.status_code == 200:
                result = response.json()
                print("\n✅ 请求成功!")
                print(f"响应: {json.dumps(result, indent=2, ensure_ascii=False)}")
                
                # 显示执行结果
                if result.get('code') == 200:  # ErrorCode.Success
                    execution_result = result.get('execution_result', '')
                    
                    if execution_result:
                        print(f"\n🎯 执行结果: {execution_result}")
                    
                    print(f"\n🎉 Contact操作完成!")
                else:
                    print(f"\n⚠️ Contact操作失败: {result.get('msg', '未知错误')}")
                
                return True
            else:
                print(f"\n❌ 请求失败，状态码: {response.status_code}")
                print(f"错误信息: {response.text}")
                return False
                
        except requests.exceptions.RequestException as e:
            print(f"\n❌ 请求异常: {e}")
            return False
    
    def create_frame(self, frame_name, marker_id=0, expected_count=5, arms="left", repeat_times=1, repeat_time_interval=0.1):
        """创建坐标系
        
        Args:
            frame_name: 坐标系名称
            marker_id: AprilTag标记ID（默认0）
            expected_count: 期望观察次数（默认5）
            arms: 使用的手臂，可以是 "left", "right", "both"（默认"left"）
            repeat_times: 重复查找次数（默认4次）
            repeat_time_interval: 每次重复查找的时间间隔（秒）（默认0.1秒）
        
        Examples:
            python3 labbot_manager_base.py create_frame "my_frame" --marker_id=5 --expected_count=10 --arms="left"
            python3 labbot_manager_base.py create_frame "dual_frame" --marker_id=0 --expected_count=8 --arms="both"
            python3 labbot_manager_base.py create_frame "right_frame" --marker_id=3 --expected_count=6 --arms="right"
        """
        print(f"\n=== 创建坐标系 ===\n")
        print(f"坐标系名称: {frame_name}")
        print(f"AprilTag标记ID: {marker_id}")
        print(f"期望观察数量: {expected_count}")
        print(f"使用手臂: {arms}")
        print(f"重复次数: {repeat_times}")
        print(f"重复时间间隔: {repeat_time_interval}秒")
        
        # 验证手臂参数
        if isinstance(arms, list):
            arms = sorted(arms)
        elif isinstance(arms, str):
            arms = sorted(arms.split(","))
        arms = sorted(list(arms))
            
        if len(arms) == 0 or len(set(arms)-set(["left", "right"])) > 0:
            print(f"❌ 无效的手臂名称: {arms}，必须是 'left', 'right', 或 'left,right'")
            return False
        
        # 验证期望观察次数
        try:
            expected_count = int(expected_count)
            if expected_count <= 0:
                print(f"❌ 无效的期望观察次数: {expected_count}，必须大于0")
                return False
        except ValueError:
            print(f"❌ 无效的期望观察次数: {expected_count}，必须是整数")
            return False
        
        # 验证坐标系名称
        if not frame_name or not isinstance(frame_name, str):
            print(f"❌ 无效的坐标系名称: {frame_name}，必须是非空字符串")
            return False
        
        # 构造请求参数
        create_frame_request = {
            "frame_name": frame_name,
            "marker_id": int(marker_id),
            "expected_count": expected_count,
            "arms": arms,
            "repeat_times": int(repeat_times),
            "repeat_time_interval": float(repeat_time_interval)
        }
        
        print(f"\n发送请求: {json.dumps(create_frame_request, indent=2, ensure_ascii=False)}")
        
        try:
            response = requests.post(
                f"{self.server_url}/create_frame",
                json=create_frame_request,
                headers={"Content-Type": "application/json"},
                timeout=120  # 创建坐标系可能需要较长时间
            )
            
            if response.status_code == 200:
                result = response.json()
                print("\n✅ 请求成功!")
                print(f"响应: {json.dumps(result, indent=2, ensure_ascii=False)}")
                
                # 显示创建结果
                if result.get('code') == 200:  # ErrorCode.Success
                    frame_name_result = result.get('frame_name', '')
                    file_path = result.get('file_path', '')
                    transformation_matrix = result.get('transformation_matrix', [])
                    found_apriltags_count = result.get('found_apriltags_count', 0)
                    arms_used = result.get('arms_used', [])
                    
                    print(f"\n🎉 成功创建坐标系!")
                    print(f"   坐标系名称: {frame_name_result}")
                    print(f"   保存路径: {file_path}")
                    print(f"   找到AprilTag次数: {found_apriltags_count}")
                    print(f"   使用的手臂: {', '.join(arms_used)}")
                    
                    if transformation_matrix:
                        print(f"\n📐 变换矩阵:")
                        for i, row in enumerate(transformation_matrix):
                            if len(row) == 4:
                                print(f"   [{row[0]:8.4f}, {row[1]:8.4f}, {row[2]:8.4f}, {row[3]:8.4f}]")
                            else:
                                print(f"   {row}")
                    
                    print(f"\n💾 坐标系文件已保存到: {file_path}")
                    return True
                else:
                    print(f"\n⚠️ 创建坐标系失败: {result.get('msg', '未知错误')}")
                
            else:
                print(f"\n❌ 请求失败，状态码: {response.status_code}")
                print(f"错误信息: {response.text}")
                
        except requests.exceptions.RequestException as e:
            print(f"\n❌ 请求异常: {e}")
        return False
    
    def gripper(self, left_position=None, left_speed=0.03, left_force=20.0,
                        right_position=None, right_speed=0.03, right_force=20.0, wait: bool=False):
        """控制左右手夹爪运动
        
        Args:
            left_position: 左手夹爪位置 (0.0-100.0)，None表示不控制左手夹爪
            left_speed: 左手夹爪速度 (0.0-100.0)
            left_force: 左手夹爪力度 (0.0-100.0)
            right_position: 右手夹爪位置 (0.0-100.0)，None表示不控制右手夹爪
            right_speed: 右手夹爪速度 (0.0-100.0)
            right_force: 右手夹爪力度 (0.0-100.0)
        
        Examples:
            python3 labbot_manager_base.py control_grippers --left_position=50.0 --right_position=80.0
            python3 labbot_manager_base.py control_grippers --left_position=0.0 --left_speed=30.0
            python3 labbot_manager_base.py control_grippers --right_position=100.0 --right_force=10.0
        """
        print(f"\n=== 夹爪控制 ===\n")
        
        # 验证参数
        if left_position is None and right_position is None:
            print("❌ 错误: 至少需要指定一个夹爪的位置参数")
            return False
        
        # 构造请求参数
        request_data = {}
        
        if left_position is not None:
            # 验证左手夹爪参数
            if not (0.0 <= left_position <= 100.0):
                print(f"❌ 错误: 左手夹爪位置必须在 0.0-100.0 范围内，当前值: {left_position}")
                return False
            if not (0.0 <= left_speed <= 100.0):
                print(f"❌ 错误: 左手夹爪速度必须在 0.0-100.0 范围内，当前值: {left_speed}")
                return False
            if not (0.0 <= left_force <= 100.0):
                print(f"❌ 错误: 左手夹爪力度必须在 0.0-100.0 范围内，当前值: {left_force}")
                return False
            
            request_data["left_gripper"] = {
                "position": left_position,
                "speed": left_speed,
                "force": left_force
            }
            print(f"🤖 左手夹爪: 位置={left_position}, 速度={left_speed}, 力度={left_force}")
        
        if right_position is not None:
            # 验证右手夹爪参数
            if not (0.0 <= right_position <= 100.0):
                print(f"❌ 错误: 右手夹爪位置必须在 0.0-100.0 范围内，当前值: {right_position}")
                return False
            if not (0.0 <= right_speed <= 100.0):
                print(f"❌ 错误: 右手夹爪速度必须在 0.0-100.0 范围内，当前值: {right_speed}")
                return False
            if not (0.0 <= right_force <= 100.0):
                print(f"❌ 错误: 右手夹爪力度必须在 0.0-100.0 范围内，当前值: {right_force}")
                return False
            
            request_data["right_gripper"] = {
                "position": right_position,
                "speed": right_speed,
                "force": right_force
            }
            print(f"🤖 右手夹爪: 位置={right_position}, 速度={right_speed}, 力度={right_force}")
        request_data["wait"] = wait
        try:
            print(f"\n📡 发送夹爪控制请求到: {self.server_url}/control_grippers")
            print(f"📦 请求数据: {json.dumps(request_data, indent=2, ensure_ascii=False)}")
            
            response = requests.post(
                f"{self.server_url}/control_grippers",
                json=request_data,
                headers={"Content-Type": "application/json"},
                timeout=30
            )
            
            if response.status_code == 200:
                result = response.json()
                print(f"\n✅ 响应成功，状态码: {response.status_code}")
                print(f"📋 响应数据: {json.dumps(result, indent=2, ensure_ascii=False)}")
                
                # 解析响应结果
                if result.get("code") == 0:  # Success
                    print(f"\n🎉 夹爪控制成功!")
                    print(f"   整体状态: {'成功' if result.get('overall_success') else '部分失败'}")
                    print(f"   消息: {result.get('msg', '')}")
                    
                    # 显示左手夹爪结果
                    if result.get("left_gripper_result"):
                        left_result = result["left_gripper_result"]
                        status = "✅ 成功" if left_result.get("success") else "❌ 失败"
                        print(f"\n🤖 左手夹爪结果: {status}")
                        print(f"   消息: {left_result.get('message', '')}")
                        if left_result.get("final_position") is not None:
                            print(f"   最终位置: {left_result['final_position']}")
                    
                    # 显示右手夹爪结果
                    if result.get("right_gripper_result"):
                        right_result = result["right_gripper_result"]
                        status = "✅ 成功" if right_result.get("success") else "❌ 失败"
                        print(f"\n🤖 右手夹爪结果: {status}")
                        print(f"   消息: {right_result.get('message', '')}")
                        if right_result.get("final_position") is not None:
                            print(f"   最终位置: {right_result['final_position']}")
                    
                    return result.get('overall_success', False)
                else:
                    print(f"\n⚠️ 夹爪控制失败: {result.get('msg', '未知错误')}")
                    print(f"   错误码: {result.get('code', 'N/A')}")
                    return False
            else:
                print(f"\n❌ 请求失败，状态码: {response.status_code}")
                print(f"错误信息: {response.text}")
                return False
                
        except requests.exceptions.RequestException as e:
            print(f"\n❌ 请求异常: {e}")
            return False

    def move_j_to(self, arm_name, position, quaternion, ref_frame="world", speed=0.8, acc=0.8, 
                  need_traj=False, execute=True, wait=True, max_complexity_score=2.0, max_retry_attempts=3, cartesian=False,
                  keep_orientation=False, weight=100.0, tolerance=None, simultaneously_reach=False):
        """关节空间运动到指定位姿（MoveJTo）
        
        Args:
            arm_name: 手臂名称，left 或 right
            position: 目标位置，逗号分隔的字符串，如 "0.5,0.2,0.8"
            quaternion: 目标四元数，逗号分隔的字符串，如 "0,0,0,1"
            ref_frame: 参考坐标系，格式为 "world" 或坐标系文件名，默认 "world"
            speed: 运动速度 (默认0.8)
            acc: 运动加速度 (默认0.8)
            need_traj: 是否需要轨迹数据 (默认False)
            execute: 是否执行运动 (默认True)
            wait: 是否等待执行完成 (默认True)
            max_complexity_score: 最大复杂度评分阈值 (默认2.0)
            cartesian: 是否使用笛卡尔路径规划 (默认False)
            max_retry_attempts: 最大重试次数 (默认3)
            keep_orientation: 是否在轨迹过程中保持末端法兰朝向一致 (默认False)
            weight: 朝向约束的权重值，用于控制约束的强度 (默认100.0)
            tolerance: 朝向约束的容差 [x, y, z]，单位为弧度 (默认None)
            simultaneously_reach: 是否同时到达目标位置 (默认False，单手臂时通常为False)
        
        Examples:
            python3 labbot_manager_base.py move_j_to left "0.5,0.2,0.8" "0,0,0,1"
            python3 labbot_manager_base.py move_j_to right "0.4,0.3,0.7" "0,0,0.707,0.707" --ref_frame="my_frame"
            python3 labbot_manager_base.py move_j_to left "0.5,0.2,0.8" "0,0,0,1" --speed=0.5 --execute=False
        """
        print(f"\n=== 关节空间运动到指定位姿（MoveJTo） ===\n")
        print(f"手臂: {arm_name}")
        print(f"目标位置: {position}")
        print(f"目标四元数: {quaternion}")
        print(f"参考坐标系: {ref_frame}")
        print(f"速度: {speed}, 加速度: {acc}")
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
        
        # 解析四元数参数
        try:
            if isinstance(quaternion, str):
                quaternion_list = [float(x.strip()) for x in quaternion.split(',')]
            elif isinstance(quaternion, (list, tuple)):
                quaternion_list = [float(x) for x in quaternion]
            else:
                raise ValueError(f"不支持的四元数参数类型: {type(quaternion)}")
            
            if len(quaternion_list) != 4:
                print(f"❌ 四元数参数应该有4个值，但得到{len(quaternion_list)}个")
                return False
        except (ValueError, TypeError) as e:
            print(f"❌ 四元数解析错误: {e}")
            return False
        
        # 构造请求参数 - 使用新的多手臂格式
        arm_request = {
            "arm": arm_name,
            "position": position_list,
            "quaternion": quaternion_list,
            "ref_frame": ref_frame,
            "cartesian": bool(cartesian),
            "keep_orientation": bool(keep_orientation),
            "weight": float(weight),
            "tolerance": tolerance
        }
        
        move_j_to_request = {
            "arm_requests": [arm_request],
            "execute": bool(execute),
            "max_complexity_score": float(max_complexity_score),
            "max_retry_attempts": int(max_retry_attempts),
            "simultaneously_reach": bool(simultaneously_reach),
            "speed": float(speed),
            "acc": float(acc),
        }
        # move_j_to_request = {
        #     "arm_name": arm_name,
        #     "position": position_list,
        #     "quaternion": quaternion_list,
        #     "ref_frame": ref_frame,
        #     "speed": float(speed),
        #     "acc": float(acc),
        #     "need_traj": bool(need_traj),
        #     "execute": bool(execute),
        #     "wait": bool(wait)
        # }
        
        print(f"\n发送请求: {json.dumps(move_j_to_request, indent=2, ensure_ascii=False)}")
        
        try:
            response = requests.post(
                f"{self.server_url}/move_j_to",
                json=move_j_to_request,
                headers={"Content-Type": "application/json"},
                timeout=30
            )
            
            if response.status_code == 200:
                result = response.json()
                print(f"\n收到响应: {json.dumps(result, indent=2, ensure_ascii=False)}")
                
                if result.get('code') == 200:
                    print(f"\n✅ 关节空间运动到指定位姿成功!")
                    print(f"   消息: {result.get('msg', '')}")
                    print(f"   整体规划成功: {result.get('overall_planned', False)}")
                    print(f"   整体执行成功: {result.get('overall_executed', False)}")
                    
                    # 显示各手臂结果
                    arm_results = result.get('arm_results', [])
                    if arm_results:
                        for arm_result in arm_results:
                            print(f"   手臂 {arm_result.get('arm', 'N/A')}:")
                            print(f"     规划成功: {arm_result.get('planned', False)}")
                            print(f"     执行成功: {arm_result.get('executed', False)}")
                            if arm_result.get('final_position'):
                                print(f"     最终位置: {arm_result.get('final_position', [])}")
                            if arm_result.get('final_quaternion'):
                                print(f"     最终姿态: {arm_result.get('final_quaternion', [])}")
                    
                    # 显示轨迹ID
                    if result.get('traj_id'):
                        print(f"   轨迹ID: {result.get('traj_id')}")
                    
                    return result
                else:
                    print(f"\n⚠️ 关节空间运动到指定位姿失败: {result.get('msg', '未知错误')}")
                    print(f"   错误码: {result.get('code', 'N/A')}")
                    print(f"   整体规划成功: {result.get('overall_planned', False)}")
                    print(f"   整体执行成功: {result.get('overall_executed', False)}")
                    
                    # 显示各手臂结果
                    arm_results = result.get('arm_results', [])
                    if arm_results:
                        for arm_result in arm_results:
                            print(f"   手臂 {arm_result.get('arm', 'N/A')}:")
                            print(f"     规划成功: {arm_result.get('planned', False)}")
                            print(f"     执行成功: {arm_result.get('executed', False)}")
                    
                    return result
            else:
                print(f"\n❌ 请求失败，状态码: {response.status_code}")
                print(f"错误信息: {response.text}")
                return None
                
        except requests.exceptions.RequestException as e:
            print(f"\n❌ 请求异常: {e}")
            return None

    def multi_arm_move_j_to(self, arm_configs, execute=True, simultaneously_reach=False):
        """
        多手臂同时关节空间运动到指定位姿
        
        参数:
        - arm_configs: 手臂配置列表，每个配置包含:
          - arm: 手臂名称 ("left" 或 "right")
          - position: 目标位置 [x, y, z]
          - quaternion: 目标四元数 [x, y, z, w]
          - reference_frame: 参考坐标系 (可选，默认为"base_link")
        - execute: 是否执行轨迹 (默认False)
        - simultaneously_reach: 是否同时到达 (默认True)
        
        示例:
        python3 labbot_manager_base.py multi_arm_move_j_to '[{"arm":"left","position":[0.5,0.3,0.4],"quaternion":[0,0,0,1]},{"arm":"right","position":[0.5,-0.3,0.4],"quaternion":[0,0,0,1]}]' --execute=True
        """
        try:
            print(f"\n🤖 多手臂关节空间运动到指定位姿...")
            
            # 解析arm_configs参数
            if isinstance(arm_configs, str):
                arm_configs = json.loads(arm_configs)
            
            if not isinstance(arm_configs, list) or len(arm_configs) == 0:
                print(f"❌ 错误: arm_configs必须是非空列表")
                return None
            
            # 构造arm_requests
            arm_requests = []
            for config in arm_configs:
                arm = config.get('arm')
                position = config.get('position')
                quaternion = config.get('quaternion')
                ref_frame = config.get('ref_frame', 'world')
                
                # 验证参数
                if arm not in ["left", "right"]:
                    print(f"❌ 错误: 无效的手臂名称 '{arm}'，必须是 'left' 或 'right'")
                    return None
                
                if not position or len(position) != 3:
                    print(f"❌ 错误: 手臂 {arm} 的位置必须是3个数值的列表")
                    return None
                
                if not quaternion or len(quaternion) != 4:
                    print(f"❌ 错误: 手臂 {arm} 的四元数必须是4个数值的列表")
                    return None
                
                arm_request = {
                    "arm": arm,
                    "position": position,
                    "quaternion": quaternion,
                    "ref_frame": ref_frame
                }
                arm_requests.append(arm_request)
                
                print(f"   手臂 {arm}: 位置={position}, 四元数={quaternion}, 参考坐标系={ref_frame}")
            
            # 构造请求数据
            request_data = {
                "arm_requests": arm_requests,
                "execute": execute,
                "simultaneously_reach": simultaneously_reach
            }
            
            print(f"\n📤 发送请求: {json.dumps(request_data, indent=2, ensure_ascii=False)}")
            
            # 发送POST请求
            response = requests.post(f"{SERVER_URL}/move_j_to", json=request_data, timeout=30)
            
            if response.status_code == 200:
                result = response.json()
                print(f"\n收到响应: {json.dumps(result, indent=2, ensure_ascii=False)}")
                
                if result.get('code') == 200:
                    print(f"\n✅ 多手臂关节空间运动到指定位姿成功!")
                    print(f"   消息: {result.get('msg', '')}")
                    print(f"   整体规划成功: {result.get('overall_planned', False)}")
                    print(f"   整体执行成功: {result.get('overall_executed', False)}")
                    
                    # 显示各手臂结果
                    arm_results = result.get('arm_results', [])
                    if arm_results:
                        for arm_result in arm_results:
                            print(f"   手臂 {arm_result.get('arm', 'N/A')}:")
                            print(f"     规划成功: {arm_result.get('planned', False)}")
                            print(f"     执行成功: {arm_result.get('executed', False)}")
                            if arm_result.get('final_position'):
                                print(f"     最终位置: {arm_result.get('final_position', [])}")
                            if arm_result.get('final_quaternion'):
                                print(f"     最终姿态: {arm_result.get('final_quaternion', [])}")
                    
                    # 显示轨迹ID
                    if result.get('traj_id'):
                        print(f"   轨迹ID: {result.get('traj_id')}")
                    
                    return result
                else:
                    print(f"\n⚠️ 多手臂关节空间运动到指定位姿失败: {result.get('msg', '未知错误')}")
                    print(f"   错误码: {result.get('code', 'N/A')}")
                    print(f"   整体规划成功: {result.get('overall_planned', False)}")
                    print(f"   整体执行成功: {result.get('overall_executed', False)}")
                    
                    # 显示各手臂结果
                    arm_results = result.get('arm_results', [])
                    if arm_results:
                        for arm_result in arm_results:
                            print(f"   手臂 {arm_result.get('arm', 'N/A')}:")
                            print(f"     规划成功: {arm_result.get('planned', False)}")
                            print(f"     执行成功: {arm_result.get('executed', False)}")
                    
                    return result
            else:
                print(f"\n❌ HTTP错误: {response.status_code}")
                print(f"响应内容: {response.text}")
                return None
                
        except requests.exceptions.RequestException as e:
            print(f"\n❌ 请求异常: {e}")
            return None

    def get_end_effector_relative_position(self, arm, frame_name):
        """
        获取末端执行器相对于指定物件坐标系的位置和姿态
        
        参数:
        - arm: 手臂名称 ("left" 或 "right")
        - frame_name: 物件坐标系名称（对应/home/ubuntu/.aico2/frames/目录下的json文件名）
        
        示例:
        python3 labbot_manager_base.py get_end_effector_relative_position left tube_crack_tf
        python3 labbot_manager_base.py get_end_effector_relative_position right my_object_frame
        """
        try:
            print(f"\n🤖 获取 {arm} 手臂末端执行器相对于物件坐标系 '{frame_name}' 的位置和姿态...")
            
            # 验证参数
            if arm not in ["left", "right"]:
                print(f"❌ 错误: 无效的手臂名称 '{arm}'，必须是 'left' 或 'right'")
                return False
            
            if not frame_name or not isinstance(frame_name, str):
                print(f"❌ 错误: 物件坐标系名称不能为空")
                return False
            
            # 构造请求数据
            request_data = {
                "arm": arm,
                "frame_name": frame_name
            }
            
            print(f"📤 发送请求: {request_data}")
            
            # 发送POST请求
            response = requests.post(
                f"{SERVER_URL}/get_end_effector_relative_position",
                json=request_data,
                headers={"Content-Type": "application/json"},
                timeout=30
            )
            
            if response.status_code == 200:
                result = response.json()
                print(f"📥 收到响应: {result}")
                
                if result.get('code') == 200:
                    print(f"\n✅ 成功获取末端执行器相对位置!")
                    print(f"   消息: {result.get('msg', '')}")
                    
                    position = result.get('position', [])
                    quaternion = result.get('quaternion', [])
                    frame_file_path = result.get('frame_file_path', '')
                    
                    if position:
                        print(f"   相对位置 [x, y, z]: [{position[0]:.6f},{position[1]:.6f},{position[2]:.6f}]")
                    
                    if quaternion:
                        print(f"   相对姿态 [x, y, z, w]: [{quaternion[0]:.6f},{quaternion[1]:.6f},{quaternion[2]:.6f},{quaternion[3]:.6f}]")
                    
                    if frame_file_path:
                        print(f"   物件坐标系文件: {frame_file_path}")
                    
                    return True
                else:
                    print(f"\n⚠️ 获取末端执行器相对位置失败: {result.get('msg', '未知错误')}")
                    print(f"   错误码: {result.get('code', 'N/A')}")
                    return False
            else:
                print(f"\n❌ 请求失败，状态码: {response.status_code}")
                print(f"错误信息: {response.text}")
                return False
                
        except requests.exceptions.RequestException as e:
            print(f"\n❌ 请求异常: {e}")
            return False

    def run_traj(self, traj_id, speed=0.5, acc=0.3, wait=True, validate_trajectory=True, remote_host: str=None):
        """根据轨迹ID执行相应的轨迹
        
        Args:
            traj_id: 轨迹ID，用于查找对应的轨迹文件
            speed: 执行速度 (0.0-1.0，默认0.5)
            acc: 执行加速度 (0.0-1.0，默认0.3)
            wait: 是否等待执行完成 (默认True)
            validate_trajectory: 是否验证轨迹文件 (默认True)
        
        Examples:
            python3 labbot_manager_base.py run_trajectory "trajectory_123"
            python3 labbot_manager_base.py run_trajectory "my_traj_001" --speed=0.8 --acc=0.5
            python3 labbot_manager_base.py run_trajectory "test_trajectory" --wait=False --validate_trajectory=False
        """
        print(f"\n=== 执行轨迹 ===\n")
        print(f"轨迹ID: {traj_id}")
        print(f"执行速度: {speed}")
        print(f"执行加速度: {acc}")
        print(f"等待完成: {wait}")
        print(f"验证轨迹: {validate_trajectory}")
        
        # 验证参数
        if not traj_id or not isinstance(traj_id, str):
            print(f"❌ 无效的轨迹ID: {traj_id}")
            return False
        
        try:
            speed = float(speed)
            acc = float(acc)
            if not (0.0 <= speed <= 1.0):
                print(f"❌ 无效的速度值: {speed}，必须在0.0-1.0之间")
                return False
            if not (0.0 <= acc <= 1.0):
                print(f"❌ 无效的加速度值: {acc}，必须在0.0-1.0之间")
                return False
        except ValueError:
            print(f"❌ 速度和加速度必须是数字")
            return False
        
        # 构造请求参数
        run_traj_request = {
            "traj_id": str(traj_id),
            "speed": speed,
            "acc": acc,
            "wait": bool(wait),
            "validate_trajectory": bool(validate_trajectory)
        }
        
        print(f"\n发送请求: {json.dumps(run_traj_request, indent=2, ensure_ascii=False)}")
        
        try:
            if remote_host is not None:
                server_url = self.get_remote_server_url(remote_host)
            else:
                server_url = self.server_url
            response = requests.post(
                f"{server_url}/run_trajectory",
                json=run_traj_request,
                headers={"Content-Type": "application/json"},
                timeout=120  # 轨迹执行可能需要较长时间
            )
            
            if response.status_code == 200:
                result = response.json()
                print("\n✅ 请求成功!")
                print(f"响应: {json.dumps(result, indent=2, ensure_ascii=False)}")
                
                # 显示执行结果
                if result.get('code') == 200:  # ErrorCode.Success
                    traj_file_path = result.get('traj_file_path', '')
                    execution_time = result.get('execution_time', 0.0)
                    executed = result.get('executed', False)
                    trajectory_points_count = result.get('trajectory_points_count', 0)
                    
                    print(f"\n🎉 轨迹执行成功!")
                    print(f"   轨迹ID: {result.get('traj_id', traj_id)}")
                    print(f"   轨迹文件: {traj_file_path}")
                    print(f"   执行时间: {execution_time:.2f} 秒")
                    print(f"   执行状态: {'已执行' if executed else '未执行'}")
                    print(f"   轨迹点数: {trajectory_points_count}")
                    print(f"   消息: {result.get('msg', '')}")
                    
                    return result
                else:
                    print(f"\n⚠️ 轨迹执行失败: {result.get('msg', '未知错误')}")
                    print(f"   错误码: {result.get('code', 'N/A')}")
                    print(f"   轨迹ID: {result.get('traj_id', traj_id)}")
                    
                    # 显示可能的额外信息
                    traj_file_path = result.get('traj_file_path', '')
                    execution_time = result.get('execution_time', 0.0)
                    trajectory_points_count = result.get('trajectory_points_count', 0)
                    
                    if traj_file_path:
                        print(f"   轨迹文件: {traj_file_path}")
                    if execution_time > 0:
                        print(f"   执行时间: {execution_time:.2f} 秒")
                    if trajectory_points_count > 0:
                        print(f"   轨迹点数: {trajectory_points_count}")
                    
                    return result
            else:
                print(f"\n❌ 请求失败，状态码: {response.status_code}")
                print(f"错误信息: {response.text}")
                return None
                
        except requests.exceptions.RequestException as e:
            print(f"\n❌ 请求异常: {e}")
            return result

    def get_frame_offset(self, target_frame, ref_frame):
        """获取两个参考系之间的位置偏移量
        
        Args:
            target_frame (str): 目标参考系名称
            ref_frame (str): 参考参考系名称
            
        Returns:
            bool: 操作是否成功
            
        Example:
            python labbot_manager_base.py get_frame_offset --target_frame="table" --ref_frame="camera"
        """
        print(f"\n=== 获取参考系位置偏移量 ===\n")
        print(f"目标参考系: {target_frame}")
        print(f"参考参考系: {ref_frame}")
        
        # 构造请求参数
        frame_offset_request = {
            "target_frame": target_frame,
            "ref_frame": ref_frame
        }
        
        print(f"\n发送请求: {json.dumps(frame_offset_request, indent=2, ensure_ascii=False)}")
        
        try:
            response = requests.post(
                f"{self.server_url}/get_frame_offset",
                json=frame_offset_request,
                headers={"Content-Type": "application/json"},
                timeout=30
            )
            
            if response.status_code == 200:
                result = response.json()
                print("\n✅ 请求成功!")
                print(f"响应: {json.dumps(result, indent=2, ensure_ascii=False)}")
                
                # 显示偏移量信息
                if result.get('code') == 200:  # ErrorCode.Success
                    position_offset = result.get('position_offset', [])
                    target_frame_position = result.get('target_frame_position', [])
                    ref_frame_position = result.get('ref_frame_position', [])
                    
                    print(f"\n📊 参考系位置信息:")
                    
                    if target_frame_position:
                        print(f"\n📍 目标参考系 '{target_frame}' 位置 (世界坐标系):")
                        print(f"   X: {target_frame_position[0]:.4f} m")
                        print(f"   Y: {target_frame_position[1]:.4f} m")
                        print(f"   Z: {target_frame_position[2]:.4f} m")
                    
                    if ref_frame_position:
                        print(f"\n📍 参考参考系 '{ref_frame}' 位置 (世界坐标系):")
                        print(f"   X: {ref_frame_position[0]:.4f} m")
                        print(f"   Y: {ref_frame_position[1]:.4f} m")
                        print(f"   Z: {ref_frame_position[2]:.4f} m")
                    
                    if position_offset:
                        print(f"\n🔄 位置偏移量 (target_frame 相对于 ref_frame):")
                        print(f"   ΔX: {position_offset[0]:.4f} m")
                        print(f"   ΔY: {position_offset[1]:.4f} m")
                        print(f"   ΔZ: {position_offset[2]:.4f} m")
                        
                        # 计算总距离
                        total_distance = math.sqrt(sum(x**2 for x in position_offset))
                        print(f"   总距离: {total_distance:.4f} m")
                    
                    print(f"\n✅ 参考系偏移量计算成功!")
                    print(f"   消息: {result.get('msg', '')}")
                    
                    return result
                else:
                    print(f"\n⚠️ 参考系偏移量计算失败: {result.get('msg', '未知错误')}")
                    print(f"   错误码: {result.get('code', 'N/A')}")
                    
                    return None
            else:
                print(f"\n❌ 请求失败，状态码: {response.status_code}")
                print(f"错误信息: {response.text}")
                return None
                
        except requests.exceptions.RequestException as e:
            print(f"\n❌ 请求异常: {e}")
            return None
    
    def rotate_hand(self, arm_name, yaw=0.0, pitch=0.0, roll=0.0, absolute_mode=False, speed=0.8, acc=0.8, 
                    need_traj=True, wait=True, execute=True, max_complexity_score=0.1, max_retry_attempts=10):
        """旋转手部末端执行器朝向
        
        保持指定手的TCP末端位置不变，仅调整朝向。旋转顺序为绕法兰自身坐标系
        x轴（yaw）、y轴（pitch）、z轴（roll）。
        
        Args:
            arm_name (str): 手臂名称，"left" 或 "right"
            yaw (float): 绕x轴旋转角度（度），默认0.0
            pitch (float): 绕y轴旋转角度（度），默认0.0  
            roll (float): 绕z轴旋转角度（度），默认0.0
            absolute_mode (bool): 是否使用绝对旋转模式，True时以机器人正前方为初始朝向进行旋转，False时在当前朝向基础上增量旋转，默认False
            speed (float): 运动速度 (0.0-1.0)，默认0.8
            acc (float): 运动加速度 (0.0-1.0)，默认0.8
            need_traj (bool): 是否需要轨迹数据，默认True
            wait (bool): 是否等待执行完成，默认True
            execute (bool): 是否执行运动，默认True
            max_complexity_score (float): 轨迹复杂度上限阈值，超过此值的轨迹将被拒绝，默认2.0
            max_retry_attempts (int): 轨迹规划的最大重试次数，默认3
            
        Returns:
            bool: 操作是否成功
            
        Examples:
            # 左手绕x轴旋转30度
            python labbot_manager_base.py rotate_hand --arm_name=left --yaw=30.0
            
            # 右手绕y轴旋转-45度，绕z轴旋转90度
            python labbot_manager_base.py rotate_hand --arm_name=right --pitch=-45.0 --roll=90.0
            
            # 仅规划不执行
            python labbot_manager_base.py rotate_hand --arm_name=left --yaw=15.0 --execute=False
        """
        print(f"\n=== 旋转手部末端执行器 ===\n")
        print(f"手臂: {arm_name}")
        print(f"旋转角度 - Yaw(绕x轴): {yaw}°, Pitch(绕y轴): {pitch}°, Roll(绕z轴): {roll}°")
        print(f"速度: {speed}, 加速度: {acc}")
        print(f"执行: {execute}, 等待: {wait}")
        
        # 验证手臂参数
        if arm_name not in ["left", "right"]:
            print(f"❌ 无效的手臂名称: {arm_name}，必须是 'left' 或 'right'")
            return False
        
        # 验证角度参数
        try:
            yaw = float(yaw)
            pitch = float(pitch)
            roll = float(roll)
        except ValueError as e:
            print(f"❌ 角度参数转换失败: {e}")
            return False
        
        # 验证速度和加速度参数
        try:
            speed = float(speed)
            acc = float(acc)
            if not (0.0 <= speed <= 1.0):
                print(f"❌ 速度参数超出范围: {speed}，必须在 0.0-1.0 之间")
                return False
            if not (0.0 <= acc <= 1.0):
                print(f"❌ 加速度参数超出范围: {acc}，必须在 0.0-1.0 之间")
                return False
        except ValueError as e:
            print(f"❌ 速度/加速度参数转换失败: {e}")
            return False
        
        # 将角度转换为弧度
        yaw_rad = math.radians(yaw)
        pitch_rad = math.radians(pitch)
        roll_rad = math.radians(roll)
        
        # 构造请求参数
        rotate_hand_request = {
            "arm": arm_name,
            "yaw": yaw_rad,
            "pitch": pitch_rad,
            "roll": roll_rad,
            "absolute_mode": absolute_mode,
            "speed": speed,
            "acc": acc,
            "need_traj": need_traj,
            "wait": wait,
            "execute": execute,
            "max_complexity_score": max_complexity_score,
            "max_retry_attempts": max_retry_attempts
        }
        
        print(f"\n发送请求: {json.dumps(rotate_hand_request, indent=2, ensure_ascii=False)}")
        
        try:
            response = requests.post(
                f"{self.server_url}/rotate_hand",
                json=rotate_hand_request,
                headers={"Content-Type": "application/json"},
                timeout=60  # 手部旋转可能需要较长时间
            )
            
            if response.status_code == 200:
                result = response.json()
                print("\n✅ 请求成功!")
                print(f"响应: {json.dumps(result, indent=2, ensure_ascii=False)}")
                
                # 显示旋转结果
                if result.get('code') == 200:  # ErrorCode.Success
                    current_position = result.get('current_position', [])
                    target_quaternion = result.get('target_quaternion', [])
                    trajectory_path = result.get('trajectory_path', '')
                    
                    if current_position:
                        print(f"\n📍 当前TCP位置 (保持不变):")
                        print(f"   X: {current_position[0]:.4f} m")
                        print(f"   Y: {current_position[1]:.4f} m")
                        print(f"   Z: {current_position[2]:.4f} m")
                    
                    if target_quaternion:
                        print(f"\n🔄 目标姿态四元数:")
                        print(f"   X: {target_quaternion[0]:.4f}")
                        print(f"   Y: {target_quaternion[1]:.4f}")
                        print(f"   Z: {target_quaternion[2]:.4f}")
                        print(f"   W: {target_quaternion[3]:.4f}")
                    
                    if trajectory_path:
                        print(f"\n💾 轨迹文件: {trajectory_path}")
                    
                    print(f"\n🎉 手部旋转操作成功!")
                    print(f"   消息: {result.get('msg', '')}")
                    
                    return True
                else:
                    print(f"\n⚠️ 手部旋转失败: {result.get('msg', '未知错误')}")
                    print(f"   错误码: {result.get('code', 'N/A')}")
                    
                    return False
            else:
                print(f"\n❌ 请求失败，状态码: {response.status_code}")
                print(f"错误信息: {response.text}")
                return False
                
        except requests.exceptions.RequestException as e:
            print(f"\n❌ 请求异常: {e}")
            return False

    def multi_arm_move_j_to_new(self, arm_configs, execute=False, simultaneously_reach=True, 
                               max_complexity_score=2.0, max_retry_attempts=3):
        """
        调用新实现的多手臂关节空间运动接口
        
        参数:
        - arm_configs: 手臂配置列表，每个配置包含:
          - arm: 手臂名称 ("left" 或 "right")
          - position: 目标位置 [x, y, z]
          - quaternion: 目标四元数 [x, y, z, w]
          - ref_frame: 参考坐标系 (可选，默认为"world")
          - cartesian: 是否使用笛卡尔路径规划 (可选，默认False)
          - keep_orientation: 是否保持朝向 (可选，默认False)
          - weight: 朝向约束权重 (可选，默认100.0)
          - tolerance: 朝向约束容差 (可选，默认None)
        - execute: 是否执行轨迹 (默认False)
        - simultaneously_reach: 是否同时到达 (默认True)
        - max_complexity_score: 最大复杂度评分阈值 (默认2.0)
        - max_retry_attempts: 最大重试次数 (默认3)
        
        示例:
        python3 labbot_manager_base.py multi_arm_move_j_to_new '[{"arm":"left","position":[0.5,0.3,0.4],"quaternion":[0,0,0,1],"ref_frame":"world"},{"arm":"right","position":[0.5,-0.3,0.4],"quaternion":[0,0,0,1],"ref_frame":"world"}]' --execute=True --simultaneously_reach=True
        """
        try:
            print(f"\n🤖 调用新实现的多手臂关节空间运动接口...")
            
            # 解析arm_configs参数
            if isinstance(arm_configs, str):
                import json
                arm_configs = json.loads(arm_configs)
            
            if not isinstance(arm_configs, list) or len(arm_configs) == 0:
                print(f"❌ 错误: arm_configs必须是非空列表")
                return None
            
            # 构造arm_requests
            arm_requests = []
            for config in arm_configs:
                arm = config.get('arm')
                position = config.get('position')
                quaternion = config.get('quaternion')
                ref_frame = config.get('ref_frame', 'world')
                cartesian = config.get('cartesian', False)
                keep_orientation = config.get('keep_orientation', False)
                weight = config.get('weight', 100.0)
                tolerance = config.get('tolerance', None)
                
                # 验证参数
                if arm not in ["left", "right"]:
                    print(f"❌ 错误: 无效的手臂名称 '{arm}'，必须是 'left' 或 'right'")
                    return None
                
                if not position or len(position) != 3:
                    print(f"❌ 错误: 手臂 {arm} 的位置必须是3个数值的列表")
                    return None
                
                if not quaternion or len(quaternion) != 4:
                    print(f"❌ 错误: 手臂 {arm} 的四元数必须是4个数值的列表")
                    return None
                
                arm_request = {
                    "arm": arm,
                    "position": position,
                    "quaternion": quaternion,
                    "ref_frame": ref_frame,
                    "cartesian": cartesian,
                    "keep_orientation": keep_orientation,
                    "weight": weight,
                    "tolerance": tolerance
                }
                arm_requests.append(arm_request)
                
                print(f"   手臂 {arm}: 位置={position}, 四元数={quaternion}, 参考坐标系={ref_frame}")
                print(f"     笛卡尔={cartesian}, 保持朝向={keep_orientation}, 权重={weight}")
            
            # 构造请求数据
            request_data = {
                "arm_requests": arm_requests,
                "execute": execute,
                "simultaneously_reach": simultaneously_reach,
                "max_complexity_score": max_complexity_score,
                "max_retry_attempts": max_retry_attempts
            }
            
            print(f"\n📤 发送请求到新接口: {json.dumps(request_data, indent=2, ensure_ascii=False)}")
            
            # 发送POST请求到新的接口
            response = requests.post(f"{SERVER_URL}/multi_arm_move_j_to", json=request_data, timeout=30)
            
            if response.status_code == 200:
                result = response.json()
                print(f"\n收到响应: {json.dumps(result, indent=2, ensure_ascii=False)}")
                
                if result.get('code') == 200:
                    print(f"\n✅ 新接口多手臂关节空间运动成功!")
                    print(f"   消息: {result.get('msg', '')}")
                    print(f"   整体规划成功: {result.get('overall_planned', False)}")
                    print(f"   整体执行成功: {result.get('overall_executed', False)}")
                    
                    # 显示各手臂结果
                    arm_results = result.get('arm_results', [])
                    if arm_results:
                        for arm_result in arm_results:
                            print(f"   手臂 {arm_result.get('arm', 'N/A')}:")
                            print(f"     规划成功: {arm_result.get('planned', False)}")
                            print(f"     执行成功: {arm_result.get('executed', False)}")
                            if arm_result.get('final_position'):
                                print(f"     最终位置: {arm_result.get('final_position', [])}")
                            if arm_result.get('final_quaternion'):
                                print(f"     最终姿态: {arm_result.get('final_quaternion', [])}")
                    
                    # 显示轨迹ID
                    if result.get('traj_id'):
                        print(f"   轨迹ID: {result.get('traj_id')}")
                    
                    return result
                else:
                    print(f"\n⚠️ 新接口多手臂关节空间运动失败: {result.get('msg', '未知错误')}")
                    print(f"   错误码: {result.get('code', 'N/A')}")
                    print(f"   整体规划成功: {result.get('overall_planned', False)}")
                    print(f"   整体执行成功: {result.get('overall_executed', False)}")
                    
                    # 显示各手臂结果
                    arm_results = result.get('arm_results', [])
                    if arm_results:
                        for arm_result in arm_results:
                            print(f"   手臂 {arm_result.get('arm', 'N/A')}:")
                            print(f"     规划成功: {arm_result.get('planned', False)}")
                            print(f"     执行成功: {arm_result.get('executed', False)}")
                    
                    return result
            else:
                print(f"\n❌ HTTP错误: {response.status_code}")
                print(f"响应内容: {response.text}")
                return None
                
        except requests.exceptions.RequestException as e:
            print(f"\n❌ 请求异常: {e}")
            return None

    def move_j_to_traj_start(self, traj_id, speed=1.0, acc=1.0, wait=True, execute:bool=True):
        """移动到指定轨迹的起始点
        
        Args:
            traj_id: 轨迹ID
            speed: 运动速度 (0.0-1.0)
            acc: 运动加速度 (0.0-1.0)
            wait: 是否等待执行完成
        
        Examples:
            python3 labbot_manager_base.py move_j_to_traj_start "traj_123"
            python3 labbot_manager_base.py move_j_to_traj_start "traj_456" --speed=0.5 --acc=0.5 --wait=False
        """
        print(f"\n=== 移动到轨迹起始点 ===\n")
        print(f"轨迹ID: {traj_id}")
        print(f"速度: {speed}, 加速度: {acc}")
        print(f"等待完成: {wait}")
        
        # 验证参数
        if not traj_id:
            print("❌ 轨迹ID不能为空")
            return False
        
        try:
            speed = float(speed)
            acc = float(acc)
            if not (0.0 <= speed <= 1.0):
                print(f"❌ 速度值无效: {speed}，必须在0.0-1.0之间")
                return False
            if not (0.0 <= acc <= 1.0):
                print(f"❌ 加速度值无效: {acc}，必须在0.0-1.0之间")
                return False
        except ValueError:
            print("❌ 速度和加速度必须是数字")
            return False
        
        # 构造请求参数
        move_j_to_traj_start_request = {
            "traj_id": str(traj_id),
            "speed": speed,
            "acc": acc,
            "execute": execute,
            "wait": bool(wait)
        }
        
        print(f"\n发送请求: {json.dumps(move_j_to_traj_start_request, indent=2, ensure_ascii=False)}")
        
        try:
            response = requests.post(
                f"{self.server_url}/move_j_to_traj_start",
                json=move_j_to_traj_start_request,
                headers={"Content-Type": "application/json"},
                timeout=60  # 移动操作可能需要较长时间
            )
            
            if response.status_code == 200:
                result = response.json()
                print("\n✅ 请求成功!")
                print(f"响应: {json.dumps(result, indent=2, ensure_ascii=False)}")
                
                # 显示执行结果
                if result.get('code') == 200:  # ErrorCode.Success
                    print(f"\n🎉 成功移动到轨迹起始点!")
                    print(f"   轨迹ID: {result.get('traj_id', 'N/A')}")
                    print(f"   轨迹文件路径: {result.get('traj_file_path', 'N/A')}")
                    print(f"   移动轨迹ID: {result.get('move_j_traj_id', 'N/A')}")
                    print(f"   执行状态: {'已执行' if result.get('executed', False) else '未执行'}")
                    
                    # 显示初始关节位置
                    initial_positions = result.get('initial_joint_positions', [])
                    if initial_positions:
                        print(f"\n📍 轨迹起始关节位置:")
                        for i, pos in enumerate(initial_positions):
                            print(f"   关节{i+1}: {pos:.4f} rad ({math.degrees(pos):.2f}°)")
                else:
                    print(f"\n⚠️ 移动到轨迹起始点失败: {result.get('msg', '未知错误')}")
                    print(f"   错误码: {result.get('code', 'N/A')}")
                    print(f"   轨迹ID: {result.get('traj_id', 'N/A')}")
                    print(f"   轨迹文件路径: {result.get('traj_file_path', 'N/A')}")
                
                return True
            else:
                print(f"\n❌ 请求失败，状态码: {response.status_code}")
                print(f"错误信息: {response.text}")
                return False
                
        except requests.exceptions.RequestException as e:
            print(f"\n❌ 请求异常: {e}")
            return False
    
    def fast_move_j_dual_arm(self, position, quaternion, ref_frame="world", cartesian=False, 
                           keep_orientation=False, weight=100.0, tolerance=None, execute=True, 
                           max_complexity_score=2.0, max_retry_attempts=3, start_joint_positions=None, remote_host: str=None):
        """
        双手同步运动
        
        基于右手目标位姿规划轨迹，然后生成双手同步轨迹，左手保持末端执行器朝向不变。
        
        Args:
            position: 右手末端执行器的目标位置，逗号分隔的字符串 "x,y,z"
            quaternion: 右手末端执行器的目标四元数姿态，逗号分隔的字符串 "x,y,z,w"
            ref_frame: 参考坐标系名称，world表示机器人世界坐标系（默认world）
            cartesian: 是否使用笛卡尔路径规划（默认False）
            keep_orientation: 是否在轨迹过程中保持末端法兰朝向一致（默认False）
            weight: 朝向约束权重值（默认100.0）
            tolerance: 朝向约束的容差，逗号分隔的字符串 "x,y,z"，单位为弧度（可选）
            execute: 是否立即执行运动（默认True）
            max_complexity_score: 轨迹复杂度上限阈值（默认2.0）
            max_retry_attempts: 轨迹规划的最大重试次数（默认3）
            start_joint_positions: 规划轨迹的起始关节位置，若未指定则使用当前关节位置
        
        Examples:
            python3 labbot_manager_base.py fast_move_j_dual_arm "0.5,0.2,0.3" "0,0,0,1"
            python3 labbot_manager_base.py fast_move_j_dual_arm "0.5,0.2,0.3" "0,0,0,1" --execute=False
            python3 labbot_manager_base.py fast_move_j_dual_arm "0.5,0.2,0.3" "0,0,0,1" --cartesian=True
            python3 labbot_manager_base.py fast_move_j_dual_arm "0.5,0.2,0.3" "0,0,0,1" --ref_frame="object_frame"
        """
        print(f"\n=== 双手同步运动 ===\n")
        print(f"目标位置: {position}")
        print(f"目标姿态: {quaternion}")
        print(f"参考坐标系: {ref_frame}")
        print(f"笛卡尔规划: {cartesian}")
        print(f"保持朝向: {keep_orientation}")
        print(f"朝向权重: {weight}")
        print(f"执行模式: {'执行运动' if execute else '仅规划'}")
        
        # 解析位置参数
        try:
            if isinstance(position, str):
                position_list = [float(x.strip()) for x in position.split(',')]
            else:
                position_list = list(position)
            
            if len(position_list) != 3:
                print(f"❌ 位置参数无效，必须是3个数值，但得到{len(position_list)}个")
                return False
        except (ValueError, TypeError) as e:
            print(f"❌ 位置参数解析错误: {e}")
            return False
        
        # 解析四元数参数
        try:
            if isinstance(quaternion, str):
                quaternion_list = [float(x.strip()) for x in quaternion.split(',')]
            else:
                quaternion_list = list(quaternion)
            
            if len(quaternion_list) != 4:
                print(f"❌ 四元数参数无效，必须是4个数值，但得到{len(quaternion_list)}个")
                return False
        except (ValueError, TypeError) as e:
            print(f"❌ 四元数参数解析错误: {e}")
            return False
        
        # 解析容差参数（可选）
        tolerance_list = None
        if tolerance is not None:
            try:
                if isinstance(tolerance, str):
                    tolerance_list = [float(x.strip()) for x in tolerance.split(',')]
                else:
                    tolerance_list = list(tolerance)
                
                if len(tolerance_list) != 3:
                    print(f"❌ 容差参数无效，必须是3个数值，但得到{len(tolerance_list)}个")
                    return False
            except (ValueError, TypeError) as e:
                print(f"❌ 容差参数解析错误: {e}")
                return False
        
        # 构造请求参数
        fast_move_j_dual_arm_request = {
            "position": position_list,
            "quaternion": quaternion_list,
            "ref_frame": ref_frame,
            "cartesian": cartesian,
            "keep_orientation": keep_orientation,
            "weight": weight,
            "execute": execute,
            "max_complexity_score": max_complexity_score,
            "max_retry_attempts": max_retry_attempts,
            "start_joint_positions": start_joint_positions
        }
        
        if tolerance_list is not None:
            fast_move_j_dual_arm_request["tolerance"] = tolerance_list
        
        print(f"\n发送请求: {json.dumps(fast_move_j_dual_arm_request, indent=2, ensure_ascii=False)}")
        
        try:
            if remote_host is not None:
                server_url = self.get_remote_server_url(remote_host)
            else:
                server_url = self.server_url
            response = requests.post(
                f"{server_url}/fast_move_j_dual_arm",
                json=fast_move_j_dual_arm_request,
                headers={"Content-Type": "application/json"},
                timeout=120  # 双手同步运动可能需要较长时间
            )
            
            if response.status_code == 200:
                result = response.json()
                print("\n✅ 请求成功!")
                print(f"响应: {json.dumps(result, indent=2, ensure_ascii=False)}")
                
                # 显示执行结果
                if result.get('code') == 200:  # ErrorCode.Success
                    print(f"\n🎉 双手同步运动成功!")
                    print(f"   规划状态: {'成功' if result.get('planned', False) else '失败'}")
                    print(f"   执行状态: {'已执行' if result.get('executed', False) else '未执行'}")
                    print(f"   轨迹ID: {result.get('traj_id', 'N/A')}")
                    
                    # 显示右手最终位姿
                    right_position = result.get('right_arm_final_position', [])
                    right_quaternion = result.get('right_arm_final_quaternion', [])
                    if right_position:
                        print(f"\n📍 右手最终位置:")
                        print(f"   X: {right_position[0]:.4f} m")
                        print(f"   Y: {right_position[1]:.4f} m")
                        print(f"   Z: {right_position[2]:.4f} m")
                    
                    if right_quaternion:
                        print(f"\n🔄 右手最终姿态四元数:")
                        print(f"   X: {right_quaternion[0]:.4f}")
                        print(f"   Y: {right_quaternion[1]:.4f}")
                        print(f"   Z: {right_quaternion[2]:.4f}")
                        print(f"   W: {right_quaternion[3]:.4f}")
                    
                    # 显示左手最终位姿
                    left_position = result.get('left_arm_final_position', [])
                    left_quaternion = result.get('left_arm_final_quaternion', [])
                    if left_position:
                        print(f"\n📍 左手最终位置:")
                        print(f"   X: {left_position[0]:.4f} m")
                        print(f"   Y: {left_position[1]:.4f} m")
                        print(f"   Z: {left_position[2]:.4f} m")
                    
                    if left_quaternion:
                        print(f"\n🔄 左手最终姿态四元数:")
                        print(f"   X: {left_quaternion[0]:.4f}")
                        print(f"   Y: {left_quaternion[1]:.4f}")
                        print(f"   Z: {left_quaternion[2]:.4f}")
                        print(f"   W: {left_quaternion[3]:.4f}")
                    
                    # 显示参考坐标系信息
                    ref_frame_file = result.get('ref_frame_file_path', '')
                    if ref_frame_file:
                        print(f"\n📄 参考坐标系文件: {ref_frame_file}")
                    
                    # 显示变换矩阵
                    transformation_matrix = result.get('transformation_matrix', [])
                    if transformation_matrix:
                        print(f"\n📐 变换矩阵:")
                        for i, row in enumerate(transformation_matrix):
                            if len(row) >= 4:
                                print(f"   [{row[0]:8.4f}, {row[1]:8.4f}, {row[2]:8.4f}, {row[3]:8.4f}]")
                else:
                    print(f"\n⚠️ 双手同步运动失败: {result.get('msg', '未知错误')}")
                    print(f"   错误码: {result.get('code', 'N/A')}")
                    print(f"   规划状态: {'成功' if result.get('planned', False) else '失败'}")
                    print(f"   执行状态: {'已执行' if result.get('executed', False) else '未执行'}")
                    print(f"   轨迹ID: {result.get('traj_id', 'N/A')}")
                
                return result
            else:
                print(f"\n❌ 请求失败，状态码: {response.status_code}")
                print(f"错误信息: {response.text}")
                return False
                
        except requests.exceptions.RequestException as e:
            print(f"\n❌ 请求异常: {e}")
            return False

    def fast_move_j_to_dual_arm(
        self,
        offset_commands,
        ref_frame="world",
        cartesian=False, 
        keep_orientation=False,
        weight=100.0,
        tolerance=None,
        execute=True, 
        max_complexity_score=2.0,
        max_retry_attempts=3,
        remote_host=None,
        start_joint_states=None,
        right_start_position=None,
        right_start_quaternion=None
    ):
        """
        基于偏移量的双手同步运动
        
        获取当前右手位姿，应用偏移量后调用fast_move_j_dual_arm进行双手同步运动。
        
        Args:
            offset_commands: 偏移命令字符串，格式如 "x+0.1" 或 "x+0.05,y-0.02,z-0.03"
            ref_frame: 参考坐标系名称，world表示机器人世界坐标系（默认world）
            cartesian: 是否使用笛卡尔路径规划（默认False）
            keep_orientation: 是否在轨迹过程中保持末端法兰朝向一致（默认False）
            weight: 朝向约束权重值（默认100.0）
            tolerance: 朝向约束的容差，逗号分隔的字符串 "x,y,z"，单位为弧度（可选）
            execute: 是否立即执行运动（默认True）
            max_complexity_score: 轨迹复杂度上限阈值（默认2.0）
            max_retry_attempts: 轨迹规划的最大重试次数（默认3）
            remote_host: 远程主机地址，若指定则用远程主机进行轨迹计算
        
        Examples:
            python3 labbot_manager_base.py fast_move_j_to_dual_arm "x+0.1"
            python3 labbot_manager_base.py fast_move_j_to_dual_arm "x+0.05,y-0.02"
            python3 labbot_manager_base.py fast_move_j_to_dual_arm "z-0.03" --execute=False
            python3 labbot_manager_base.py fast_move_j_to_dual_arm "x+0.1,y+0.05" --cartesian=True
        """
        print(f"\n=== 基于偏移量的双手同步运动 ===\n")
        print(f"偏移命令: {offset_commands}")
        print(f"参考坐标系: {ref_frame}")
        print(f"笛卡尔规划: {cartesian}")
        print(f"保持朝向: {keep_orientation}")
        print(f"朝向权重: {weight}")
        print(f"执行模式: {'执行运动' if execute else '仅规划'}")
        
        if start_joint_states is not None:
            # 若指定了起始关节状态, 就不需要获取当前状态了
            current_joint_positions = list(start_joint_states)
            current_position = list(right_start_position)
            current_quaternion = list(right_start_quaternion)
        else:
            # 获取当前右手状态
            print(f"\n📊 获取右手当前状态...")
            try:
                joint_states_request = {"arm": "right"}
                
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
                
                # 获取右手末端执行器位姿
                right_arm_data = result.get('right_arm', {})
                right_robot_tf = right_arm_data.get('robot_tf', {})
                right_tcp_robot = right_robot_tf.get('tcp', {})
                current_position = right_tcp_robot.get('position', [])
                current_quaternion = right_tcp_robot.get('orientation', [])
                current_joint_positions = result.get('joint_states', [])
                
                if not current_position or not current_quaternion:
                    print(f"❌ 未获取到右手末端执行器位姿数据")
                    return False
                
                if len(current_position) != 3 or len(current_quaternion) != 4:
                    print(f"❌ 末端执行器位姿数据格式错误")
                    return False

                if len(current_joint_positions) != 16:
                    print(f"❌ 未获取到关节位置数据")
                    return False
                
                print(f"✅ 当前关节位置: {current_joint_positions}")
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
                value_str = offset_part[1:]
                
                # 验证轴名称
                if axis not in ['x', 'y', 'z']:
                    print(f"❌ 无效的轴名称: {axis}，必须是 'x', 'y' 或 'z'")
                    return False
                
                # 解析数值
                try:
                    offset_value = float(value_str)
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
        
        # 调用fast_move_j_dual_arm函数
        print(f"\n🚀 调用双手同步运动接口...")
        
        # 将位置和姿态转换为字符串格式
        position_str = f"{target_position[0]},{target_position[1]},{target_position[2]}"
        quaternion_str = f"{current_quaternion[0]},{current_quaternion[1]},{current_quaternion[2]},{current_quaternion[3]}"
        
        # 调用已有的fast_move_j_dual_arm函数
        if remote_host is None:
            local_result = self.fast_move_j_dual_arm(
                position=position_str,
                quaternion=quaternion_str,
                ref_frame=ref_frame,
                cartesian=cartesian,
                keep_orientation=keep_orientation,
                weight=weight,
                tolerance=tolerance,
                execute=execute,
                max_complexity_score=max_complexity_score,
                max_retry_attempts=max_retry_attempts,
                start_joint_positions=current_joint_positions
            )
            # return local_result["traj_id"]
            return local_result

        # 如果用远程机器计算, 远程不用执行，但要拿到结果轨迹
        remote_result = self.fast_move_j_dual_arm(
            position=position_str,
            quaternion=quaternion_str,
            ref_frame=ref_frame,
            cartesian=cartesian,
            keep_orientation=keep_orientation,
            weight=weight,
            tolerance=tolerance,
            execute=False,
            max_complexity_score=max_complexity_score,
            max_retry_attempts=max_retry_attempts,
            start_joint_positions=current_joint_positions,
            remote_host=remote_host
        )
        if isinstance(remote_result, dict):
            remote_traj_id = remote_result["traj_id"]
            local_traj_file = os.path.join(os.path.realpath(os.path.expanduser("~/.aico2/executed_traj")), remote_traj_id)
            os.system(f"curl http://{remote_host}:7100/executed_traj/{remote_traj_id} -o {local_traj_file}")
            self.run_traj(remote_traj_id)
        # return remote_traj_id
        return remote_result

def main():
    """主函数，使用Fire创建命令行接口"""
    # 禁用分页，直接在终端显示帮助信息
    os.environ['PAGER'] = 'cat'
    fire.Fire(LabbotManagerClientBase)

if __name__ == "__main__":
    main()