'''
Author: Mrasamu
Date: 2024-12-08 01:15:00
LastEditors: Mrasamu
LastEditTime: 2024-12-14 11:33:13
description: file content
FilePath: /cursor/qt_gui.py
'''
# -*- coding: utf-8 -*-

from PyQt5 import QtCore, QtGui, QtWidgets
from ros_video_display import ROSVideoDisplay

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        # 设置窗口基本属性
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1920, 1080)  # 设置初始窗口大小为1920x1080
        MainWindow.setMinimumSize(1000, 800)  # 保持最小尺寸限制
        
        # 设置窗口样式
        MainWindow.setStyleSheet("""
            QMainWindow {
                background-color: #f5f6fa;
            }
            QWidget {
                font-family: 'Microsoft YaHei UI', 'Segoe UI', 'PingFang SC', sans-serif;
            }
            QPushButton {
                font-size: 24px;
                padding: 10px;
                border-radius: 5px;
                border: 1px solid #dcdde1;
                background-color: #f5f6fa;
            }
            QPushButton:hover {
                background-color: #dcdde1;
            }
            QPushButton:pressed {
                background-color: #718093;
                color: white;
            }
            QPushButton:disabled {
                background-color: #718093;
                color: white;
            }
        """)
        
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        
        # 创建主布局
        self.mainLayout = QtWidgets.QVBoxLayout(self.centralwidget)
        self.mainLayout.setSpacing(10)
        self.mainLayout.setContentsMargins(20, 20, 20, 20)
        
        # 创建顶部信息卡片
        self.infoCard = QtWidgets.QWidget()
        self.infoCard.setStyleSheet("""
            QWidget {
                background-color: white;
                border-radius: 15px;
                border: 1px solid #e0e0e0;
            }
        """)
        self.infoLayout = QtWidgets.QHBoxLayout(self.infoCard)
        self.infoLayout.setContentsMargins(30, 20, 30, 20)
        
        # 队伍信息部分
        self.teamLabel = QtWidgets.QLabel("队伍名称")
        self.teamLabel.setStyleSheet("""
            color: #7f8c8d;
            font-size: 48px;
            font-weight: normal;
        """)
        self.infoLayout.addWidget(self.teamLabel)
        
        self.teamDisplay = QtWidgets.QLabel("Glgg说得都队")
        self.teamDisplay.setStyleSheet("""
            color: #2c3e50;
            font-size: 48px;
            font-weight: bold;
        """)
        self.infoLayout.addWidget(self.teamDisplay)
        
        # 添加计时器部分
        self.timerLabel = QtWidgets.QLabel("用时")
        self.timerLabel.setStyleSheet("""
            color: #7f8c8d;
            font-size: 48px;
            font-weight: normal;
        """)
        self.infoLayout.addWidget(self.timerLabel)
        
        self.timerDisplay = QtWidgets.QLabel("00:00.000")
        self.timerDisplay.setStyleSheet("""
            color: #2c3e50;
            font-size: 48px;
            font-weight: bold;
        """)
        self.infoLayout.addWidget(self.timerDisplay)
        
        # 添加计时控制按钮
        self.timerStartButton = QtWidgets.QPushButton("开始计时")
        self.timerStartButton.setFixedWidth(150)
        self.infoLayout.addWidget(self.timerStartButton)
        
        self.timerStopButton = QtWidgets.QPushButton("停止计时")
        self.timerStopButton.setFixedWidth(150)
        self.timerStopButton.setEnabled(False)
        self.infoLayout.addWidget(self.timerStopButton)
        
        self.infoLayout.addStretch()
        
        # 分数显示
        self.scoreLabel = QtWidgets.QLabel("当前得分")
        self.scoreLabel.setStyleSheet("""
            color: #7f8c8d;
            font-size: 48px;
            font-weight: normal;
        """)
        self.infoLayout.addWidget(self.scoreLabel)
        
        self.scoreDisplay = QtWidgets.QLabel("00")
        self.scoreDisplay.setStyleSheet("""
            color: #e74c3c;
            font-size: 72px;
            font-weight: bold;
        """)
        self.infoLayout.addWidget(self.scoreDisplay)
        
        # 将信息卡片添加到主布局
        self.mainLayout.addWidget(self.infoCard)
        
        # 创建内容区域
        self.contentArea = QtWidgets.QHBoxLayout()
        self.contentArea.setSpacing(20)
        
        # 左侧结果显示区域
        self.resultsCard = QtWidgets.QWidget()
        self.resultsCard.setStyleSheet("""
            QWidget {
                background-color: white;
                border-radius: 15px;
                border: 1px solid #e0e0e0;
            }
            QLabel {
                color: #2c3e50;
                font-size: 36px;
                font-weight: bold;
                padding: 5px;
                border: 1px solid #e0e0e0;
                background-color: #f8f9fa;
                border-radius: 5px;
            }
        """)
        
        self.resultsLayout = QtWidgets.QVBoxLayout(self.resultsCard)
        self.resultsLayout.setContentsMargins(20, 20, 20, 20)
        
        # 创建5x5网格布局
        self.gridLayout = QtWidgets.QGridLayout()
        self.gridLayout.setSpacing(10)
        
        # 创建网格中的标签
        self.gridLabels = []
        for i in range(5):
            row = []
            for j in range(5):
                label = QtWidgets.QLabel("")
                label.setAlignment(QtCore.Qt.AlignCenter)
                label.setMinimumSize(120, 120)
                self.gridLayout.addWidget(label, i, j)
                row.append(label)
            self.gridLabels.append(row)
        
        # 设置特定位置的标签
        positions = {
            (0, 2): "1号", (4, 2): "2号", (1, 1): "3号",
            (3, 3): "4号", (2, 0): "5号", (2, 4): "6号",
            (3, 1): "7号", (1, 3): "8号", (2, 2): "9号"
        }
        
        for pos, text in positions.items():
            self.gridLabels[pos[0]][pos[1]].setText(text)
        
        # 添加起飞和降落按钮
        self.takeoffButton = QtWidgets.QPushButton("起飞成功")
        self.landingButton = QtWidgets.QPushButton("降落成功")
        self.gridLayout.addWidget(self.takeoffButton, 0, 0)
        self.gridLayout.addWidget(self.landingButton, 4, 4)
        
        self.resultsLayout.addLayout(self.gridLayout)
        
        # 添加识别结果显示区域
        self.resultLabel = QtWidgets.QLabel("选手识别结果：")
        self.resultLabel.setStyleSheet("""
            color: #7f8c8d;
            font-size: 36px;
            font-weight: normal;
            padding: 10px 0;
            border: none;
            background: none;
        """)
        self.resultsLayout.addWidget(self.resultLabel)
        
        self.resultDisplay = QtWidgets.QLabel("")
        self.resultDisplay.setStyleSheet("""
            color: #2c3e50;
            font-size: 36px;
            font-weight: bold;
            padding: 10px;
            border: 1px solid #e0e0e0;
            border-radius: 5px;
            background-color: #f8f9fa;
            min-height: 60px;
        """)
        self.resultDisplay.setAlignment(QtCore.Qt.AlignCenter)
        self.resultsLayout.addWidget(self.resultDisplay)
        
        # 右侧视频区域
        self.videoCard = QtWidgets.QWidget()
        self.videoCard.setStyleSheet("""
            QWidget {
                background-color: white;
                border-radius: 15px;
                border: 1px solid #e0e0e0;
            }
            QLabel {
                color: #2c3e50;
                font-size: 36px;
                font-weight: bold;
                padding: 10px 0;
            }
        """)
        
        self.videoLayout = QtWidgets.QVBoxLayout(self.videoCard)
        self.videoLayout.setContentsMargins(20, 20, 20, 20)
        
        self.videoLabel = QtWidgets.QLabel("实时视频流")
        self.videoLayout.addWidget(self.videoLabel)
        
        self.videoDisplay = ROSVideoDisplay()
        self.videoDisplay.setMinimumSize(800, 600)
        self.videoDisplay.setStyleSheet("""
            QLabel {
                border: 2px solid #e0e0e0;
                border-radius: 10px;
                padding: 0px;
                background-color: #000000;
            }
        """)
        self.videoLayout.addWidget(self.videoDisplay)
        
        # 将左右两部分添加到内容区域
        self.contentArea.addWidget(self.resultsCard, 1)
        self.contentArea.addWidget(self.videoCard, 2)
        
        # 将内容区域添加到主布局
        self.mainLayout.addLayout(self.contentArea)
        
        MainWindow.setCentralWidget(self.centralwidget)
        
        # 创建状态栏
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setStyleSheet("""
            QStatusBar {
                background-color: white;
                color: #7f8c8d;
            }
        """)
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "无人机比赛裁判系统"))