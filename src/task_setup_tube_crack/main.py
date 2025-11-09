#!/usr/bin/env python
# -*- encoding: utf-8 -*-
"""
过柱机试管架架设任务主程序

功能：协调执行完整的过柱机试管架架设流程，包括从远程桌子拾取试管架、
安装试管架、推入试管架、拉出试管架、双手抓取试管架、
到另一个桌子放置试管架等完整流程

主要流程：
1. 从远程桌子拾取试管架
2. 移动到LM8位置(过柱机通风橱)
3. 安装试管架
4. 推入试管架
5. 拉出试管架
6. 拾取试管架
7. 移动到LM4位置
8. 放置试管架到LM4的桌子
"""

import json

import fire

from src.task_setup_tube_crack.sub_tasks import (pick_crack_from_another_table,
                                                 pick_front_crack,
                                                 pull_out_crack,
                                                 push_front_crack,
                                                 put_crack_on_another_table,
                                                 setup_front_crack)
from src.util.util import agv_move

# 加载AprilTag标记配置
with open("object_marker_config.json", 'r') as f:
    OBJECT_MARKER_CONFIG = json.load(f)

def complete_roundtrip():
    """
    执行完整的试管架测试往返流程
    
    功能：协调执行完整的试管架处理流程，包括拾取、设置、推入、
    拉出、移动和放置等所有子任务
    
    Returns:
        bool: 所有子任务成功完成返回True，任何子任务失败返回False
    """
    # crack_config = OBJECT_MARKER_CONFIG["formal_crack_20"]
    crack_config = OBJECT_MARKER_CONFIG["origin_crack_23"]
    column_machine_config = OBJECT_MARKER_CONFIG["column_machine_base"]

    if not pick_crack_from_another_table(crack_config=crack_config["pick_crack_remote"]):
        print(f"=> pick_crack_from_another_table failed! Abort!!!")
        return False
    agv_move("LM8", wait=True)

    if not setup_front_crack(
        crack_config=crack_config["setup_crack"],
        column_machine_config=column_machine_config["setup_crack"]
    ):
        print(f"=> Setup Front Crack failed! Abort!!!")
        return False
    if not push_front_crack(crack_config=crack_config["push_crack"]):
        print(f"=> Push Front Crack failed! Abort!!!")
        return False

    if not pull_out_crack(crack_config=crack_config["pull_crack"], column_machine_config=column_machine_config["pull_crack"]):
        print(f"=> Pull Out Crack failed! Abort!!!")
        return False
    if not pick_front_crack(crack_config=crack_config["pick_crack"]):
        print(f"=> Pick Front Crack failed! Abort!!!")
        return False


    agv_move("LM4", wait=True)
    if not put_crack_on_another_table(crack_config=crack_config["pick_crack"]):
        print(f"=> Put Front Crack failed! Abort!!!")
        return False

def main():
    """
    主函数 - 使用Fire库创建命令行接口
    
    功能：提供命令行接口，可以通过命令行参数调用complete_roundtrip等函数
    示例用法：python main.py complete_roundtrip
    """
    fire.Fire()
    
if __name__ == '__main__':
    """
    程序入口点
    
    当直接运行此脚本时，启动Fire命令行接口
    """
    main()
