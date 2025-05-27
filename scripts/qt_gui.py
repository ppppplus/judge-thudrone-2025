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
        # 移除最大尺寸限制，允许全屏
        
        # 设置窗口样式
        MainWindow.setStyleSheet("""
            QMainWindow {
                background-color: #f5f6fa;
            }
            QWidget {
                font-family: 'Microsoft YaHei UI', 'Segoe UI', 'PingFang SC', sans-serif;
            }
        """)
        
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        
        # 创建主布局
        self.mainLayout = QtWidgets.QVBoxLayout(self.centralwidget)
        self.mainLayout.setSpacing(10)  # 增加组件间距
        self.mainLayout.setContentsMargins(20, 20, 20, 20)  # 增加边距
        
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
        self.infoLayout.setContentsMargins(30, 40, 30, 40)
        
        # 直接在infoLayout中添加标签
        self.teamLabel = QtWidgets.QLabel("队伍名称：")
        self.teamLabel.setStyleSheet("""
            color: #7f8c8d;
            font-size: 72px;
            font-weight: normal;
        """)
        self.infoLayout.addWidget(self.teamLabel)
        
        self.teamDisplay = QtWidgets.QLabel("Glgg说得都队")
        self.teamDisplay.setStyleSheet("""
            color: #2c3e50;
            font-size: 72px;
            font-weight: bold;
        """)
        self.infoLayout.addWidget(self.teamDisplay)
        
        self.infoLayout.addStretch()
        
        self.scoreLabel = QtWidgets.QLabel("当前得分：")
        self.scoreLabel.setStyleSheet("""
            color: #7f8c8d;
            font-size: 72px;
            font-weight: normal;
        """)
        self.infoLayout.addWidget(self.scoreLabel)
        
        self.scoreDisplay = QtWidgets.QLabel("0")
        self.scoreDisplay.setStyleSheet("""
            color: #e74c3c;
            font-size: 100px;
            font-weight: bold;
        """)
        self.infoLayout.addWidget(self.scoreDisplay)
        
        # 将信息添加到顶部卡片
        self.mainLayout.addWidget(self.infoCard)
        
        # 创建内容区域
        self.contentArea = QtWidgets.QHBoxLayout()
        self.contentArea.setSpacing(5)
        
        # 左侧答案区域
        self.answersCard = QtWidgets.QWidget()
        self.answersCard.setStyleSheet("""
            QWidget {
                background-color: white;
                border-radius: 15px;
                border: 1px solid #e0e0e0;
            }
            QLabel {
                color: #2c3e50;
                font-size: 36px;
                font-weight: bold;
                padding: 5px 0;
            }
            QTextBrowser {
                border: 2px solid #e0e0e0;
                border-radius: 12px;
                padding: 20px;
                font-size: 72px;
                line-height: 1.0;
                background-color: #f8f9fa;
                color: #2c3e50;
                min-height: 120px;
                max-height: 160px;
            }
        """)
        
        self.answersLayout = QtWidgets.QVBoxLayout(self.answersCard)
        self.answersLayout.setContentsMargins(20, 10, 20, 10)
        self.answersLayout.setSpacing(5)
        
        # 正确答案部分
        self.correctLabel = QtWidgets.QLabel("正确答案")
        self.correctLabel.setFixedWidth(400)  # 固定标签宽度
        self.correctLabel.setMinimumHeight(40)  # 设置最小高度
        self.correctLabel.setStyleSheet("""
            color: #2c3e50;
            font-size: 56px;
            font-weight: bold;
            padding: 5px 10px;
            background-color: #f8f9fa;
            border-radius: 10px;
            margin: 5px;
        """)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        self.correctLabel.setSizePolicy(sizePolicy)
        self.answersLayout.addWidget(self.correctLabel)
        
        self.correctAnswer = QtWidgets.QTextBrowser()
        self.correctAnswer.setMinimumHeight(160)
        self.correctAnswer.setMaximumHeight(240)
        self.answersLayout.addWidget(self.correctAnswer)
        
        # 添加分隔线
        self.line = QtWidgets.QFrame()
        self.line.setFrameShape(QtWidgets.QFrame.HLine)
        self.line.setStyleSheet("background-color: #e0e0e0; margin: 5px 0;")
        self.answersLayout.addWidget(self.line)
        
        # 选手答案部分
        self.playerLabel = QtWidgets.QLabel("选手答案")
        self.playerLabel.setFixedWidth(400)  # 固定标签宽度
        self.playerLabel.setMinimumHeight(40)  # 设置最小高度
        self.playerLabel.setStyleSheet("""
            color: #2c3e50;
            font-size: 56px;
            font-weight: bold;
            padding: 5px 10px;
            background-color: #f8f9fa;
            border-radius: 10px;
            margin: 5px;
        """)
        # sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        self.playerLabel.setSizePolicy(sizePolicy)
        self.answersLayout.addWidget(self.playerLabel)
        
        self.playerAnswer = QtWidgets.QTextBrowser()
        self.playerAnswer.setMinimumHeight(160)
        self.playerAnswer.setMaximumHeight(240)
        self.answersLayout.addWidget(self.playerAnswer)
        
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
                font-size: 56px;
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
        self.contentArea.addWidget(self.answersCard, 1)  # 1是拉伸因子
        self.contentArea.addWidget(self.videoCard, 2)    # 2是拉伸因子，使视频区域更宽
        
        # 将内容区域添加到主布局
        self.mainLayout.addLayout(self.contentArea)
        
        MainWindow.setCentralWidget(self.centralwidget)
        
        # 创建菜单栏
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1000, 22))
        self.menubar.setStyleSheet("""
            QMenuBar {
                background-color: white;
                border-bottom: 1px solid #e0e0e0;
            }
            QMenuBar::item {
                padding: 5px 10px;
                color: #2c3e50;
            }
            QMenuBar::item:selected {
                background-color: #f5f6fa;
            }
        """)
        MainWindow.setMenuBar(self.menubar)
        
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