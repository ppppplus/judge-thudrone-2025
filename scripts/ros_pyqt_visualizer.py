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
from PyQt5.QtWidgets import QApplication, QMainWindow, QFileDialog, QPushButton
from PyQt5.QtCore import QTimer, QTime, Qt
from std_msgs.msg import String
from qt_gui import Ui_MainWindow
import yaml

class RosVisualizer(QMainWindow):
    def __init__(self):
        super(RosVisualizer, self).__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        
        # 初始化ROS节点
        rospy.init_node('qt_visualizer', anonymous=True)
        
        # 创建订阅者
        self.subscriber = rospy.Subscriber('/judge', String, self.callback)
        self.score_subscriber = rospy.Subscriber('/score', String, self.score_callback)
        self.team_subscriber = rospy.Subscriber('/team_name', String, self.team_callback)
        self.result_subscriber = rospy.Subscriber('/result', String, self.result_callback)
        
        # 创建发布者
        self.score_publisher = rospy.Publisher('/judge', String, queue_size=10)
        
        # 初始化识别结果
        self.latest_result = ""
        
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
        
        # 设置网格标签的点击事件
        positions = {
            (0, 2): "1", (4, 2): "2", (1, 1): "3",
            (3, 3): "4", (2, 0): "5", (2, 4): "6",
            (3, 1): "7", (1, 3): "8", (2, 2): "9"
        }
        
        for pos, num in positions.items():
            label = self.ui.gridLabels[pos[0]][pos[1]]
            label.mousePressEvent = lambda e, n=num: self.handle_grid_click(e, n)
            label.setStyleSheet("background-color: #f8f9fa; border: 1px solid #e0e0e0;")
        
        # 添加圈1和圈2按钮
        self.circle1_button = QPushButton("圈1")
        self.circle2_button = QPushButton("圈2")
        self.ui.gridLayout.addWidget(self.circle1_button, 1, 2)  # 2行3列
        self.ui.gridLayout.addWidget(self.circle2_button, 3, 2)  # 4行3列
        
        # 连接圈按钮的点击事件
        self.circle1_button.clicked.connect(lambda: self.handle_circle_click(1))
        self.circle2_button.clicked.connect(lambda: self.handle_circle_click(2))
        
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

    def format_answer(self, answer):
        """格式化识别结果显示"""
        if not answer:
            return ""
        try:
            position = int(answer[0])  # 位置编号
            result_type = answer[1]    # 结果类型
            return f"位置 {position}: {result_type}类"
        except:
            return answer

    def update_gui(self):
        """更新GUI显示"""
        # 更新分数和队伍名称显示
        self.ui.scoreDisplay.setText(self.current_score)
        self.ui.teamDisplay.setText(self.team_name)
        
        # 更新计时器显示（如果正在运行）
        if self.match_timer_running:
            self.ui.timerDisplay.setText(self.match_timer.toString("mm:ss"))
        
    def closeEvent(self, event):
        """程序关闭时的处理"""
        rospy.signal_shutdown('Qt GUI closed')
        event.accept()

    def start_match_timer(self):
        """开始比赛计时"""
        self.match_timer_running = True
        self.match_timer_update.start(100)  # 每100毫秒更新一次
        self.ui.timerStartButton.setEnabled(False)
        self.ui.timerStopButton.setEnabled(True)

    def result_callback(self, data):
        """处理选手识别结果"""
        if self.match_timer_running:  # 只在计时器运行时接收结果
            self.latest_result = data.data
            self.ui.resultDisplay.setText(self.latest_result)

    def stop_match_timer(self):
        """停止比赛计时"""
        self.match_timer_running = False
        self.match_timer_update.stop()
        self.ui.timerStartButton.setEnabled(True)
        self.ui.timerStopButton.setEnabled(False)
        
        # 取消订阅result话题
        if hasattr(self, 'result_subscriber'):
            self.result_subscriber.unregister()
            
        rospy.loginfo(f"比赛结束，用时：{self.match_timer.toString('mm:ss.zzz')}")

    def update_match_timer(self):
        """更新比赛计时器显示"""
        if self.match_timer_running:
            self.match_timer = self.match_timer.addMSecs(100)  # 每100毫秒更新一次
            self.ui.timerDisplay.setText(self.match_timer.toString("mm:ss.zzz"))

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

    def handle_grid_click(self, event, number):
        """处理网格标签的点击事件"""
        if event.button() == Qt.LeftButton:  # 左键点击
            score = 1
            color = "#90EE90"  # 浅绿色
        else:  # 右键点击
            score = 3
            color = "#228B22"  # 深绿色
            
        # 9号位置特殊处理
        if number == "9":
            score = 6
            color = "#FF0000"  # 红色
            
        # 更新标签样式
        label = None
        for pos, num in {
            (0, 2): "1", (4, 2): "2", (1, 1): "3",
            (3, 3): "4", (2, 0): "5", (2, 4): "6",
            (3, 1): "7", (1, 3): "8", (2, 2): "9"
        }.items():
            if num == number:
                label = self.ui.gridLabels[pos[0]][pos[1]]
                break
                
        if label:
            label.setStyleSheet(f"background-color: {color}; color: white;")
            
        # 发布得分消息
        msg = String()
        msg.data = f"score_adjust_{score}"
        self.score_publisher.publish(msg)

    def handle_circle_click(self, circle_num):
        """处理圈按钮的点击事件"""
        button = self.circle1_button if circle_num == 1 else self.circle2_button
        if not button.property("clicked"):
            button.setStyleSheet("background-color: #27ae60; color: white;")
            button.setProperty("clicked", True)
            
            # 发布得分消息
            msg = String()
            msg.data = "score_adjust_10"
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
