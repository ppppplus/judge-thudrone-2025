'''
Author: Mrasamu
Date: 2024-12-08 01:14:54
LastEditors: Mrasamu
LastEditTime: 2024-12-14 11:36:44
description: file content
FilePath: /cursor/ros_pyqt_visualizer.py
'''
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)

import sys, os
import rospy
from PyQt5.QtWidgets import QApplication, QMainWindow, QFileDialog
from PyQt5.QtCore import QTimer
from PyQt5.QtCore import QTime
from PyQt5.QtWidgets import QPushButton
from std_msgs.msg import String
from qt_gui import Ui_MainWindow
import yaml

class RosVisualizer(QMainWindow):
    def __init__(self):
        super(RosVisualizer, self).__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        # self.first_hit = True
        # 初始化ROS节点
        rospy.init_node('qt_visualizer', anonymous=True)
        
        # 创建订阅者
        self.subscriber = rospy.Subscriber('/judge', String, self.callback)
        self.score_subscriber = rospy.Subscriber('/score', String, self.score_callback)
        self.team_subscriber = rospy.Subscriber('/team_name', String, self.team_callback)
        
        # 创建发布者
        self.score_publisher = rospy.Publisher('/score_adjustment', String, queue_size=10)
        
        # 创建定时器用于更新GUI
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_gui)
        self.timer.start(100)  # 每100ms更新一次
        
        # 创建比赛计时器
        self.match_timer = QTime(0, 0)  # 初始化为0分0秒
        self.match_timer_running = False
        self.match_timer_update = QTimer()
        self.match_timer_update.timeout.connect(self.update_match_timer)
        
        # 连接按钮信号
        self.ui.timerStartButton.clicked.connect(self.start_match_timer)
        self.ui.timerStopButton.clicked.connect(self.stop_match_timer)
        self.ui.takeoffButton.clicked.connect(self.handle_takeoff)
        self.ui.landingButton.clicked.connect(self.handle_landing)
        
        # 连接分数调整按钮信号
        for button in self.findChildren(QPushButton):
            if hasattr(button, 'property') and button.property('score_type') == 'manual':
                button.clicked.connect(self.handle_score_adjustment)
        
        # 初始化状态变量
        self.takeoff_done = False
        self.landing_done = False
        self.received_data = ""
        self.current_score = "0"
        self.team_name = "Team 1"
        self.config_file = os.path.join(os.path.dirname(__file__), 'config.yaml')
        
        # 初始加载配置文件
        self.load_config_file()

    def load_config_file(self):
        """加载YAML配置文件"""
        try:
            with open(self.config_file, 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f)
            
            # 读取队伍名称
            self.team_name = config.get('team_name', 'Team 1')
            self.ui.teamDisplay.setText(self.team_name)
            
        except Exception as e:
            rospy.logerr(f"读取配置文件错误：{str(e)}")

    def callback(self, msg):
        """处理识别结果"""
        data = msg.data
        
        # 如果是分数调整消息，忽略
        if data.startswith('score_adjust_'):
            return
            
        self.received_data = data
        # 更新对应位置的标签
        try:
            position = int(data[0]) - 1  # 假设数据格式为"1A"、"2B"等
            result_type = data[1]
            
            # 获取对应的网格位置
            positions = {
                0: (0, 2), 1: (4, 2), 2: (1, 1),
                3: (3, 3), 4: (2, 0), 5: (2, 4),
                6: (3, 1), 7: (1, 3), 8: (2, 2)
            }
            
            if position in positions:
                row, col = positions[position]
                label = self.ui.gridLabels[row][col]
                # 根据类型设置不同的样式
                style = {
                    'A': "background-color: #2ecc71; color: white;",  # 绿色
                    'B': "background-color: #3498db; color: white;",  # 蓝色
                    'C': "background-color: #e74c3c; color: white;"   # 红色
                }
                if result_type in style:
                    label.setStyleSheet(style[result_type])
                    label.setText(f"{position + 1}号\n{result_type}类")
        except Exception as e:
            rospy.logerr(f"处理识别结果错误：{str(e)}")

    def score_callback(self, data):
        """分数话题回调函数"""
        self.current_score = data.data
        self.ui.scoreDisplay.setText(self.current_score)

    def team_callback(self, data):
        """队伍名称话题回调函数"""
        self.team_name = data.data
        self.ui.teamDisplay.setText(self.team_name)

    def update_gui(self):
        """更新GUI显示"""
        formatted_answer = self.format_answer(self.received_data)
        self.ui.playerAnswer.setHtml(formatted_answer)
        self.ui.scoreDisplay.setText(self.current_score)
        self.ui.teamDisplay.setText(self.team_name)  # 更新队伍名称显示
        
    def closeEvent(self, event):
        """程序关闭时的处理"""
        rospy.signal_shutdown('Qt GUI closed')
        event.accept()

    def start_match_timer(self):
        """开始比赛计时"""
        self.match_timer_running = True
        self.match_timer_update.start(1000)  # 每秒更新一次
        self.ui.timerStartButton.setEnabled(False)
        self.ui.timerStopButton.setEnabled(True)

    def stop_match_timer(self):
        """停止比赛计时"""
        self.match_timer_running = False
        self.match_timer_update.stop()
        self.ui.timerStartButton.setEnabled(True)
        self.ui.timerStopButton.setEnabled(False)
        rospy.loginfo(f"比赛结束，用时：{self.match_timer.toString('mm:ss')}")

    def update_match_timer(self):
        """更新比赛计时器显示"""
        if self.match_timer_running:
            self.match_timer = self.match_timer.addSecs(1)
            self.ui.timerDisplay.setText(self.match_timer.toString("mm:ss"))

    def handle_takeoff(self):
        """处理起飞按钮点击"""
        if not self.takeoff_done:
            self.takeoff_done = True
            self.ui.takeoffButton.setEnabled(False)
            self.ui.takeoffButton.setStyleSheet("background-color: #27ae60; color: white;")
            # 发布得分消息
            msg = String()
            msg.data = "takeoff"
            self.score_publisher.publish(msg)

    def handle_landing(self):
        """处理降落按钮点击"""
        if not self.landing_done:
            self.landing_done = True
            self.ui.landingButton.setEnabled(False)
            self.ui.landingButton.setStyleSheet("background-color: #27ae60; color: white;")
            # 发布得分消息
            msg = String()
            msg.data = "landing"
            self.score_publisher.publish(msg)

    def handle_score_adjustment(self):
        """处理分数调整按钮点击"""
        button = self.sender()
        if button and hasattr(button, 'property'):
            score_change = button.property('score')
            score_type = button.property('type')
            
            # 发布分数变更消息
            msg = String()
            msg.data = f"score_adjust_{score_change}"
            self.score_publisher.publish(msg)

def main():
    app = QApplication(sys.argv)
    window = RosVisualizer()
    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass 
