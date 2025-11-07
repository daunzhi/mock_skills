import requests
import json
from typing import Dict, Any

# API基础地址（根据实际部署情况修改）
BASE_URL = "http://localhost:8000"


def health_check() -> None:
    """测试健康检查接口"""
    url = f"{BASE_URL}/health"
    response = requests.get(url)
    print(f"健康检查响应: {response.json()}")


def control_gripper(width: float, force: float) -> None:
    """测试夹爪控制接口"""
    url = f"{BASE_URL}/action/gripper"
    data = {
        "width": width,
        "force": force
    }
    response = requests.post(url, json=data)
    print(f"夹爪控制响应: {response.json()}")


def observe(joint_pos: list, force: float) -> Dict[str, Any]:
    """测试观察操作接口，返回观察结果"""
    url = f"{BASE_URL}/action/observe"
    data = {
        "joint_pos": joint_pos,
        "force": force
    }
    response = requests.post(url, json=data)
    result = response.json()
    print(f"观察操作响应: {result}")
    return result


def get_grasp_final_state(object_res: Dict[str, Any]) -> None:
    """测试获取抓取最终状态接口"""
    url = f"{BASE_URL}/calc/get_grasp_final_state"
    # 将观察结果转为JSON字符串作为查询参数
    params = {
        "object_res": json.dumps(object_res)
    }
    response = requests.get(url, params=params)
    print(f"抓取最终状态响应: {response.json()}")


def get_place_final_state(object_res: Dict[str, Any]) -> None:
    """测试获取放置最终状态接口"""
    url = f"{BASE_URL}/calc/get_place_final_state"
    params = {
        "object_res": json.dumps(object_res)
    }
    response = requests.get(url, params=params)
    print(f"放置最终状态响应: {response.json()}")


def go2grasp_pose(grasp_state: list, trajectory: list) -> None:
    """测试移动到抓取位置接口"""
    url = f"{BASE_URL}/action/go2grasp_pose"
    data = {
        "grasp_state": grasp_state,
        "trajectory": trajectory
    }
    response = requests.post(url, json=data)
    print(f"移动到抓取位置响应: {response.json()}")


def place(joint_pos: list, force: float) -> None:
    """测试放置操作接口"""
    url = f"{BASE_URL}/action/place"
    data = {
        "joint_pos": joint_pos,
        "force": force
    }
    response = requests.post(url, json=data)
    print(f"放置操作响应: {response.json()}")



def workflow_easy():
    observe_joint_pos = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]  # 7维关节角度
    observe_result = observe(joint_pos=observe_joint_pos, force=5.0)


def workflow_hard():
    pass



if __name__ == "__main__":

    """
    测试
    """
    # 1. 健康检查
    health_check()

    # 2. 控制夹爪（示例：开合宽度50mm，力度10N）
    control_gripper(width=50.0, force=10.0)

    # 3. 观察操作（示例：7维关节角度，观察力度5N）
    observe_joint_pos = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]  # 7维关节角度
    observe_result = observe(joint_pos=observe_joint_pos, force=5.0)

    # 4. 根据观察结果获取抓取最终状态
    if observe_result.get("code") == 200:
        get_grasp_final_state(object_res=observe_result["observe_res"])

    # 5. 移动到抓取位置（示例：目标关节角度+轨迹）
    grasp_state = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]  # 目标抓取关节角度（7维）
    trajectory = [
        [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7],  # 轨迹点1（7维）
        [0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8],  # 轨迹点2（7维）
        [0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9]   # 轨迹点3（7维）
    ]
    go2grasp_pose(grasp_state=grasp_state, trajectory=trajectory)

    # 6. 根据观察结果获取放置最终状态
    if observe_result.get("code") == 200:
        get_place_final_state(object_res=observe_result["observe_res"])

    # 7. 放置操作（示例：放置前的关节角度，放置力度8N）
    place_joint_pos = [0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1]  # 7维关节角度
    place(joint_pos=place_joint_pos, force=8.0)