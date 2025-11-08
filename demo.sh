#!/bin/bash

# 保存当前终端设置
# original_tty_settings=$(stty -g)

# # 确保在脚本退出时恢复终端设置
# trap 'stty "$original_tty_settings"; reset' EXIT

# 启动可视化界面
python3 scripts/ros_pyqt_visualizer.py --local_ip 192.168.31.149
# visualizer_pid=$!

# # 启动评分系统
# # python3 scripts/score.py

# # 清理进程
# kill $visualizer_pid 2>/dev/null

# # 恢复终端设置
# stty "$original_tty_settings"

# # 确保终端完全恢复
# reset 
