# _*_ coding: utf-8 _*_
"""
Time:     2025/11/7 18:52
Author:   Duanzhi Wang
Version:  V 0.1
File:     app.py.py
Describe:
"""

from fastapi import FastAPI, HTTPException, Query
from pydantic import BaseModel
from typing import List, Dict, Any, Optional
from enum import Enum

# 创建FastAPI应用实例
app = FastAPI(title="机械臂控制API", description="用于控制机械臂抓取、放置和观察操作的API")


# 定义枚举类型
class ObjectType(str, Enum):
    APRIL_TAG = "april_tag"
    # 可以根据需要添加更多类型


# 定义请求和响应模型
class Go2GraspPoseRequest(BaseModel):
    grasp_state: List[float]  # 抓取状态: 目标的joint pos角度
    trajectory: List[List[float]]  # 轨迹：为从当前运行到抓取的joint pos的轨迹


class GripperRequest(BaseModel):
    width: float  # 夹爪目标行程
    force: float  # 目标力


class PlaceRequest(BaseModel):
    joint_pos: List[float]  # 放置之前的joint pos角度
    force: float  # 放置的力


class ObserveRequest(BaseModel):
    joint_pos: List[float]  # 观察的joint pos角度
    force: float  # 观察的力


class ObserveResponse(BaseModel):
    code: int  # 状态码，正常是200
    message: str  # 错误信息
    observe_res: Optional[Dict[str, Any]] = None  # 观察结果


class ObjectResponse(BaseModel):
    code: int
    object_tcp: List[float]
    object_type: ObjectType


class ErrorResponse(BaseModel):
    code: int
    message: str


class GraspFinalStateResponse(BaseModel):
    joint_pos: List[float]
    trajectory: List[List[float]]



class PlaceFinalStateResponse(BaseModel):
    joint_pos: List[float]
    trajectory: List[List[float]]


# 定义API端点
@app.post("/action/go2grasp_pose", response_model=ErrorResponse, summary="移动到抓取位置")
async def go2grasp_pose(request: Go2GraspPoseRequest):
    """
    控制机械臂移动到指定的抓取位置

    - **grasp_state**: 目标的joint pos角度
    - **trajectory**: 从当前运行到抓取的joint pos的轨迹，每个joint pos是7维的关节角度
    """
    try:
        # 验证轨迹中的每个joint pos都是7维的
        for joint_pos in request.trajectory:
            if len(joint_pos) != 7:
                raise ValueError("轨迹中的每个joint pos必须是7维的关节角度")

        # 这里应该是实际的机械臂控制逻辑
        # 模拟成功响应
        return ErrorResponse(code=200, message="抓取成功")
    except Exception as e:
        raise HTTPException(status_code=400, detail=ErrorResponse(code=400, message=str(e)).dict())


@app.post("/action/gripper", response_model=ErrorResponse, summary="控制夹爪")
async def control_gripper(request: GripperRequest):
    """
    控制夹爪的开合和力度

    - **width**: 夹爪目标行程
    - **force**: 目标力
    """
    try:
        # 验证参数范围
        if request.width < 0 or request.force < 0:
            raise ValueError("width和force必须是非负数")

        # 这里应该是实际的夹爪控制逻辑
        # 模拟成功响应
        return ErrorResponse(code=200, message="夹爪控制成功")
    except Exception as e:
        raise HTTPException(status_code=400, detail=ErrorResponse(code=400, message=str(e)).dict())


@app.post("/action/place", response_model=ErrorResponse, summary="放置操作")
async def place(request: PlaceRequest):
    """
    控制机械臂放置物体

    - **joint_pos**: 放置之前的joint pos角度
    - **force**: 放置的力
    """
    try:
        # 验证joint_pos是7维的
        if len(request.joint_pos) != 7:
            raise ValueError("joint_pos必须是7维的关节角度")

        if request.force < 0:
            raise ValueError("force必须是非负数")

        # 这里应该是实际的放置控制逻辑
        # 模拟成功响应
        return ErrorResponse(code=200, message="放置成功")
    except Exception as e:
        raise HTTPException(status_code=400, detail=ErrorResponse(code=400, message=str(e)).dict())


@app.post("/action/observe", response_model=ObserveResponse, summary="观察操作")
async def observe(request: ObserveRequest):
    """
    控制机械臂进行观察操作

    - **joint_pos**: 观察的joint pos角度
    - **force**: 观察的力
    """
    try:
        # 验证joint_pos是7维的
        if len(request.joint_pos) != 7:
            raise ValueError("joint_pos必须是7维的关节角度")

        if request.force < 0:
            raise ValueError("force必须是非负数")

        # 这里应该是实际的观察控制逻辑
        # 模拟观察结果
        observe_res = {
            "code": 200,
            "object_tcp": [0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0],  # 示例的TCP位置（x,y,z,qx,qy,qz,qw）
            "object_type": ObjectType.APRIL_TAG
        }

        return ObserveResponse(code=200, message="观察成功", observe_res=observe_res)
    except Exception as e:
        return ObserveResponse(code=400, message=str(e))


@app.get("/calc/get_grasp_final_state", response_model=GraspFinalStateResponse, summary="获取抓取最终状态")
async def get_grasp_final_state(object_res: str = Query(..., description="观察结果对象")):
    """
    根据观察结果计算抓取的最终状态

    - **object_res**: 观察结果对象的JSON字符串
    """
    try:
        # 这里应该是实际的计算逻辑
        # 模拟返回抓取的joint pos
        return GraspFinalStateResponse(joint_pos=[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7],trajectory=[
            [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7],
            [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7],
            [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
        ])
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))


@app.get("/calc/get_place_final_state", response_model=PlaceFinalStateResponse, summary="获取放置最终状态")
async def get_place_final_state(object_res: str = Query(..., description="观察结果对象")):
    """
    根据观察结果计算放置的最终状态

    - **object_res**: 观察结果对象的JSON字符串
    """
    try:
        # 这里应该是实际的计算逻辑
        # 模拟返回放置的joint pos
        return PlaceFinalStateResponse(joint_pos=[0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1],trajectory=[
            [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7],
            [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7],
            [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
        ])
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))


# 添加健康检查端点
@app.get("/health", summary="健康检查")
async def health_check():
    return {"status": "healthy", "service": "机械臂控制API"}


# 启动应用的代码（如果直接运行此文件）
if __name__ == "__main__":
    import uvicorn

    uvicorn.run(app, host="0.0.0.0", port=8000)