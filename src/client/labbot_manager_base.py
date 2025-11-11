#!/usr/bin/env python3

import traceback
import requests
import json
import math
import fire
import os
import traceback

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
        except (ValueError, TypeError):
            print(f"Joints parameter parsing error: {traceback.format_exc()}")
            return None
    
    def _send_request(self, endpoint, data, timeout=120):
        """发送HTTP POST请求并处理响应"""
        try:
            response = requests.post(
                f"{self.server_url}/{endpoint}",
                json=data,
                headers={"Content-Type": "application/json"},
                timeout=timeout
            )
            response.raise_for_status()  # 如果状态码不是200, 则引发HTTPError
            print("\n✅ 请求成功!")
            return response.json()
        except requests.exceptions.RequestException:
            print(f"\nRequest exception: {traceback.format_exc()}")
            return None

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
        """
        print(f"\n=== 绝对关节运动 ===\n")
        
        # 解析位置参数
        body_pos = self._parse_positions(body_positions)
        left_pos = self._parse_positions(left_positions)
        right_pos = self._parse_positions(right_positions)
        
        if body_pos is None or left_pos is None or right_pos is None:
            return False
        
        # 角度转弧度
        if degree:
            body_pos = self._degrees_to_radians(body_pos)
            left_pos = self._degrees_to_radians(left_pos)
            right_pos = self._degrees_to_radians(right_pos)
        
        arm_requests = [
            {"arm_name": "body", "joint_positions": body_pos},
            {"arm_name": "left", "joint_positions": left_pos},
            {"arm_name": "right", "joint_positions": right_pos}
        ]
        
        movej_request = {
            "arm_requests": arm_requests,
            "speed": speed,
            "acc": acc,
            "need_traj": need_traj,
            "wait": wait,
            "execute": execute,
            "use_arms": use_arms.split(",")
        }
        
        return self._send_request("move_j", movej_request)

    def find_apriltag(self, arm="left", marker_id=0, repeat_times=1, repeat_time_interval=0.1):
        """查找AprilTag标记
        
        Args:
            arm: 使用哪个手臂的相机进行检测，left 或 right（默认left）
            marker_id: 要查找的AprilTag标记ID（默认0）
            repeat_times: 重复查找次数（默认4次）
            repeat_time_interval: 每次重复查找的时间间隔（秒）（默认0.1秒）
        """
        print(f"\n=== 查找AprilTag标记 ===\n")
        
        # 构造请求参数
        apriltag_request = {
            "arm": arm,
            "marker_id": int(marker_id),
            "repeat_times": int(repeat_times),
            "repeat_time_interval": float(repeat_time_interval)
        }
        
        return self._send_request("find_apriltag", apriltag_request, timeout=30)

    def action_back(self):
        """反向执行上一个轨迹"""
        print(f"\n=== 反向执行上一个轨迹 ===\n")
        
        return self._send_request("action_back", {})

    def execute_primitive(self, arm="left", primitive_name="", input_params="{}", block_until_started=True):
        """执行机器人原语命令
        
        Args:
            arm: 手臂名称，left 或 right（默认left）
            primitive_name: 原语名称
            input_params: 输入参数的JSON字符串（默认为空对象）
            block_until_started: 是否阻塞直到开始执行（默认True）
        """
        print(f"\n=== 执行机器人原语命令 ===\n")
        
        try:
            params_dict = json.loads(input_params) if isinstance(input_params, str) else (input_params if isinstance(input_params, dict) else {})
        except json.JSONDecodeError:
            params_dict = {}
        
        # 构造请求参数
        execute_primitive_request = {
            "arm": arm,
            "primitive_name": primitive_name,
            "input_params": params_dict,
            "block_until_started": bool(block_until_started)
        }
        
        return self._send_request("execute_primitive", execute_primitive_request, timeout=60)

    def status(self, arm="all", remote_host: str = None):
        """获取指定手臂的关节位置和末端执行器位姿"""
        print(f"\n=== 获取机器人状态 ===\n")
        print(f"手臂: {arm}")

        if arm not in ["left", "right", "all"]:
            print(f"❌ 无效的手臂名称: {arm}，必须是 'left' 或 'right' 或 'all'")
            return False

        if remote_host:
            self.server_url = self.get_remote_server_url(remote_host)

        response = self._send_request("get_robot_status", {"arm": arm})

        if response and response.get('code') == 200:
            print(f"响应: {json.dumps(response, indent=2, ensure_ascii=False)}")
        return response
    
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
        except (ValueError, TypeError):
            print(f"Position parameter parsing error: {traceback.format_exc()}")
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
        except (ValueError, TypeError):
            print(f"Orientation parameter parsing error: {traceback.format_exc()}")
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
                
        except requests.exceptions.RequestException:
            print(f"\nRequest exception: {traceback.format_exc()}")
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
        except (ValueError, TypeError):
            print(f"Direction parameter parsing error: {traceback.format_exc()}")
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
        except (ValueError, TypeError):
            print(f"Direction parameter parsing error: {traceback.format_exc()}")
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
            print(f"\n❌ 请求异常: {traceback.format_exc()}")
            return False
            
    def create_frame(self, frame_name, marker_id=0, expected_count=5, arms="left", repeat_times=1, repeat_time_interval=0.1):
        """创建坐标系"""
        print(f"\n=== 创建坐标系 ===\n")
        print(f"坐标系名称: {frame_name}")
        print(f"AprilTag标记ID: {marker_id}")
        print(f"期望观察数量: {expected_count}")
        print(f"使用手臂: {arms}")
        print(f"重复次数: {repeat_times}")
        print(f"重复时间间隔: {repeat_time_interval}秒")

        if isinstance(arms, list):
            arms = sorted(arms)
        elif isinstance(arms, str):
            arms = sorted(arms.split(","))
        arms = sorted(list(arms))

        if len(arms) == 0 or len(set(arms) - set(["left", "right"])) > 0:
            print(f"❌ 无效的手臂名称: {arms}，必须是 'left', 'right', 或 'left,right'")
            return False

        try:
            expected_count = int(expected_count)
            if expected_count <= 0:
                print(f"❌ 无效的期望观察次数: {expected_count}，必须大于0")
                return False
        except ValueError:
            print(f"❌ 无效的期望观察次数: {expected_count}，必须是整数")
            return False

        if not frame_name or not isinstance(frame_name, str):
            print(f"❌ 无效的坐标系名称: {frame_name}，必须是非空字符串")
            return False

        create_frame_request = {
            "frame_name": frame_name,
            "marker_id": int(marker_id),
            "expected_count": expected_count,
            "arms": arms,
            "repeat_times": int(repeat_times),
            "repeat_time_interval": float(repeat_time_interval)
        }

        response = self._send_request("create_frame", create_frame_request, timeout=120)

        if response and response.get('code') == 200:
            print(f"响应: {json.dumps(response, indent=2, ensure_ascii=False)}")
        return response
    def gripper(self, left_position=None, left_speed=0.03, left_force=20.0,
                        right_position=None, right_speed=0.03, right_force=20.0, wait: bool=False):
        """控制左右手夹爪运动"""
        print(f"\n=== 夹爪控制 ===\n")

        if left_position is None and right_position is None:
            print("❌ 错误: 至少需要指定一个夹爪的位置参数")
            return False

        try:
            request_data = {}

            if left_position is not None:
                left_position = float(left_position)
                left_speed = float(left_speed)
                left_force = float(left_force)

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
                right_position = float(right_position)
                right_speed = float(right_speed)
                right_force = float(right_force)

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

            response = self._send_request("control_grippers", request_data)

            if response and response.get('code') == 200:
                print(f"响应: {json.dumps(response, indent=2, ensure_ascii=False)}")
            return response
        except (ValueError, TypeError):
            print(f"Gripper parameter parsing error: {traceback.format_exc()}")
            return False

    def move_j_to(self, arm_name, position, quaternion, ref_frame="world", speed=0.8, acc=0.8, 
                  need_traj=False, execute=True, wait=True, max_complexity_score=2.0, max_retry_attempts=3, cartesian=False,
                  keep_orientation=False, weight=100.0, tolerance=None, simultaneously_reach=False):
        print(f"\n=== 关节空间运动到指定位姿（MoveJTo） ===\n")
        print(f"手臂: {arm_name}")
        print(f"目标位置: {position}")
        print(f"目标四元数: {quaternion}")
        print(f"参考坐标系: {ref_frame}")
        print(f"速度: {speed}, 加速度: {acc}")
        print(f"执行: {execute}, 等待: {wait}")
        
        if arm_name not in ["left", "right"]:
            print(f"❌ 无效的手臂名称: {arm_name}，必须是 'left' 或 'right'")
            return False
        
        position_list = self._parse_positions(position)
        if position_list is None or len(position_list) != 3:
            print(f"❌ 位置参数应该有3个值，但得到{len(position_list) if position_list is not None else 'None'}个")
            return False
            
        quaternion_list = self._parse_positions(quaternion)
        if quaternion_list is None or len(quaternion_list) != 4:
            print(f"❌ 四元数参数应该有4个值，但得到{len(quaternion_list) if quaternion_list is not None else 'None'}个")
            return False

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
        
        response = self._send_request("move_j_to", move_j_to_request)

        if response and response.get('code') == 200:
            print(f"响应: {json.dumps(response, indent=2, ensure_ascii=False)}")
        return response

    def multi_arm_move_j_to(self, arm_configs, execute=True, simultaneously_reach=False):
        print(f"\n🤖 多手臂关节空间运动到指定位姿...")
        
        try:
            if isinstance(arm_configs, str):
                arm_configs = json.loads(arm_configs)
        except (json.JSONDecodeError, TypeError):
            print(f"arm_configs parameter parsing error: {traceback.format_exc()}")
            return None
        
        if not isinstance(arm_configs, list) or len(arm_configs) == 0:
            print(f"❌ 错误: arm_configs必须是非空列表")
            return None
        
        arm_requests = []
        for config in arm_configs:
            arm = config.get('arm')
            position = config.get('position')
            quaternion = config.get('quaternion')
            ref_frame = config.get('ref_frame', 'world')
            
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
        
        request_data = {
            "arm_requests": arm_requests,
            "execute": execute,
            "simultaneously_reach": simultaneously_reach
        }
        
        response = self._send_request("move_j_to", request_data)

        if response and response.get('code') == 200:
            print(f"响应: {json.dumps(response, indent=2, ensure_ascii=False)}")
        return response

    def get_end_effector_relative_position(self, arm, frame_name):
        print(f"\n🤖 获取 {arm} 手臂末端执行器相对于物件坐标系 '{frame_name}' 的位置和姿态...")
        
        if arm not in ["left", "right"]:
            print(f"❌ 错误: 无效的手臂名称 '{arm}'，必须是 'left' 或 'right'")
            return False
        
        if not frame_name or not isinstance(frame_name, str):
            print(f"❌ 错误: 物件坐标系名称不能为空")
            return False
        
        request_data = {
            "arm": arm,
            "frame_name": frame_name
        }
        
        response = self._send_request("get_end_effector_relative_position", request_data)

        if response and response.get('code') == 200:
            print(f"响应: {json.dumps(response, indent=2, ensure_ascii=False)}")
        return response

    def run_traj(self, traj_id, speed=0.5, acc=0.3, wait=True, validate_trajectory=True, remote_host: str=None):
        print(f"\n=== 执行轨迹 ===\n")
        print(f"轨迹ID: {traj_id}")
        print(f"执行速度: {speed}")
        print(f"执行加速度: {acc}")
        print(f"等待完成: {wait}")
        print(f"验证轨迹: {validate_trajectory}")
        
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
            print(f"Speed and acceleration must be numbers: {traceback.format_exc()}")
            return False

        request_data = {
            "traj_id": traj_id,
            "speed": speed,
            "acc": acc,
            "wait": wait,
            "validate_trajectory": validate_trajectory
        }

        if remote_host:
            request_data["remote_host"] = remote_host
            local_traj_file = os.path.join(os.path.realpath(os.path.expanduser("~/.aico2/executed_traj")), traj_id)
            os.system(f"curl http://{remote_host}:7100/executed_traj/{traj_id} -o {local_traj_file}")

        response = self._send_request("run_traj", request_data)

        if response and response.get('code') == 200:
            print(f"响应: {json.dumps(response, indent=2, ensure_ascii=False)}")
        return response

    def get_frame_offset(self, target_frame, ref_frame):
        print(f"\n=== 获取参考系位置偏移量 ===\n")
        print(f"目标参考系: {target_frame}")
        print(f"参考参考系: {ref_frame}")
        
        request_data = {
            "target_frame": target_frame,
            "ref_frame": ref_frame
        }
        
        response = self._send_request("get_frame_offset", request_data)

        if response and response.get('code') == 200:
            print(f"响应: {json.dumps(response, indent=2, ensure_ascii=False)}")
        return response

    def rotate_hand(self, arm_name, yaw=0.0, pitch=0.0, roll=0.0, absolute_mode=False, speed=0.8, acc=0.8, 
                    need_traj=True, wait=True, execute=True, max_complexity_score=0.1, max_retry_attempts=10):
        print(f"\n=== 旋转手部末端执行器 ===\n")
        print(f"手臂: {arm_name}")
        print(f"旋转角度 - Yaw(绕x轴): {yaw}°, Pitch(绕y轴): {pitch}°, Roll(绕z轴): {roll}°")
        print(f"速度: {speed}, 加速度: {acc}")
        print(f"执行: {execute}, 等待: {wait}")
        
        if arm_name not in ["left", "right"]:
            print(f"❌ 无效的手臂名称: {arm_name}，必须是 'left' 或 'right'")
            return False
        
        try:
            yaw = float(yaw)
            pitch = float(pitch)
            roll = float(roll)
        except (ValueError, TypeError):
            print(f"Angle parameter parsing error: {traceback.format_exc()}")
            return False
        
        try:
            speed = float(speed)
            acc = float(acc)
            if not (0.0 <= speed <= 1.0):
                print(f"❌ 速度参数超出范围: {speed}，必须在 0.0-1.0 之间")
                return False
            if not (0.0 <= acc <= 1.0):
                print(f"❌ 加速度参数超出范围: {acc}，必须在 0.0-1.0 之间")
                return False
        except (ValueError, TypeError):
            print(f"Angle parameter parsing error: {traceback.format_exc()}")
            return False
        
        request_data = {
            "arm": arm_name,
            "yaw": math.radians(yaw),
            "pitch": math.radians(pitch),
            "roll": math.radians(roll),
            "absolute_mode": absolute_mode,
            "speed": speed,
            "acc": acc,
            "need_traj": need_traj,
            "wait": wait,
            "execute": execute,
            "max_complexity_score": max_complexity_score,
            "max_retry_attempts": max_retry_attempts
        }
        
        response = self._send_request("rotate_hand", request_data)

        if response and response.get('code') == 200:
            print(f"响应: {json.dumps(response, indent=2, ensure_ascii=False)}")
        return response

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
                try:
                    arm_configs = json.loads(arm_configs)
                except (json.JSONDecodeError, TypeError):
                    print(f"arm_configs parameter parsing error: {traceback.format_exc()}")
                    return None
            
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
                
        except requests.exceptions.RequestException:
            print(f"\nRequest exception: {traceback.format_exc()}")
            return None

    def move_j_to_traj_start(self, traj_id, speed=1.0, acc=1.0, wait=True, execute:bool=True):
        """Moves to the starting point of a specified trajectory."""
        request_data = {
            "traj_id": traj_id,
            "speed": speed,
            "acc": acc,
            "wait": wait,
            "execute": execute
        }
        return self._send_request("move_j_to_traj_start", request_data)

    def fast_move_j_dual_arm(
        self,
        position,
        quaternion,
        ref_frame="world",
        cartesian=False,
        keep_orientation=False,
        weight=100.0,
        tolerance=None,
        execute=True,
        max_complexity_score=2.0,
        max_retry_attempts=3,
        start_joint_positions=None,
        remote_host=None
    ):
        request_data = {
            "position": position,
            "quaternion": quaternion,
            "ref_frame": ref_frame,
            "cartesian": cartesian,
            "keep_orientation": keep_orientation,
            "weight": weight,
            "tolerance": tolerance,
            "execute": execute,
            "max_complexity_score": max_complexity_score,
            "max_retry_attempts": max_retry_attempts,
            "start_joint_positions": start_joint_positions,
            "remote_host": remote_host
        }

        return self._send_request("fast_move_j_dual_arm", request_data)

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
        if start_joint_states is not None:
            current_joint_positions = list(start_joint_states)
            current_position = list(right_start_position)
            current_quaternion = list(right_start_quaternion)
        else:
            print(f"\nGetting right arm status...")
            joint_states_request = {"arm": "right"}
            result = self._send_request("get_robot_status", joint_states_request, timeout=30)
            if not result or result.get('code') != 200:
                print(f"Failed to get current status: {result.get('msg', 'Unknown error') if result else 'No response'}")
                return False

            right_arm_data = result.get('right_arm', {})
            right_robot_tf = right_arm_data.get('robot_tf', {})
            right_tcp_robot = right_robot_tf.get('tcp', {})
            current_position = right_tcp_robot.get('position', [])
            current_quaternion = right_tcp_robot.get('orientation', [])
            current_joint_positions = result.get('joint_states', [])

            if not current_position or not current_quaternion or not current_joint_positions:
                print(f"Failed to get complete robot status data")
                return False

        try:
            target_position = current_position.copy()
            offset_parts = offset_commands.split(',')
            for offset_part in offset_parts:
                offset_part = offset_part.strip()
                if len(offset_part) < 3:
                    print(f"Offset command format error: {offset_part}")
                    return False
                axis = offset_part[0].lower()
                value_str = offset_part[1:]
                if axis not in ['x', 'y', 'z']:
                    print(f"Invalid axis name: {axis}, must be 'x', 'y' or 'z'")
                    return False
                try:
                    offset_value = float(value_str)
                except ValueError:
                    print(f"Invalid offset value: {traceback.format_exc()}")
                    return False
                axis_index = {'x': 0, 'y': 1, 'z': 2}[axis]
                target_position[axis_index] += offset_value
        except Exception:
            print(f"Offset command parsing error: {traceback.format_exc()}")
            return False

        position_str = f"{target_position[0]},{target_position[1]},{target_position[2]}"
        quaternion_str = f"{current_quaternion[0]},{current_quaternion[1]},{current_quaternion[2]},{current_quaternion[3]}"

        if remote_host is None:
            return self.fast_move_j_dual_arm(
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
            remote_traj_id = remote_result.get("traj_id")
            if remote_traj_id:
                self.run_traj(remote_traj_id, remote_host=remote_host)
        return remote_result

def main():
    os.environ['PAGER'] = 'cat'
    try:
        fire.Fire(LabbotManagerClientBase)
    except Exception:
        print(f"An error occurred: {traceback.format_exc()}")

if __name__ == "__main__":
    main()