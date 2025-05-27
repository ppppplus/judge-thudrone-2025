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
        MainWindow.resize(1000, 800)  # 增加初始窗口大小
        MainWindow.setMinimumSize(1000, 800)
        MainWindow.setMaximumSize(1920, 1080)
        
        # 设置窗口样式
        MainWindow.setStyleSheet("""
            QMainWindow {
                background-color: #f5f6fa;
            }
            QWidget {
                font-family: 'Microsoft YaHei', 'Segoe UI', sans-serif;
            }
        """)
        
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        
        # 创建主布局
        self.mainLayout = QtWidgets.QVBoxLayout(self.centralwidget)
        self.mainLayout.setSpacing(20)  # 增加组件间距
        self.mainLayout.setContentsMargins(20, 20, 20, 20)  # 增加边距
        
        # 创建顶部信息卡片
        self.infoCard = QtWidgets.QWidget()
        self.infoCard.setStyleSheet("""
            QWidget {
                background-color: white;
                border-radius: 15px;
                box-shadow: 0 2px 10px rgba(0, 0, 0, 0.1);
            }
        """)
        self.infoLayout = QtWidgets.QHBoxLayout(self.infoCard)
        self.infoLayout.setContentsMargins(20, 15, 20, 15)
        
        # 队伍信息部分
        self.teamInfo = QtWidgets.QWidget()
        self.teamLayout = QtWidgets.QVBoxLayout(self.teamInfo)
        
        self.teamLabel = QtWidgets.QLabel("队伍名称")
        self.teamLabel.setStyleSheet("""
            QLabel {
                color: #7f8c8d;
                font-size: 16px;
                font-weight: normal;
            }
        """)
        self.teamLayout.addWidget(self.teamLabel)
        
        self.teamDisplay = QtWidgets.QLabel("Team 1")
        self.teamDisplay.setStyleSheet("""
            QLabel {
                color: #2c3e50;
                font-size: 24px;
                font-weight: bold;
            }
        """)
        self.teamLayout.addWidget(self.teamDisplay)
        
        # 分数信息部分
        self.scoreInfo = QtWidgets.QWidget()
        self.scoreLayout = QtWidgets.QVBoxLayout(self.scoreInfo)
        
        self.scoreLabel = QtWidgets.QLabel("当前得分")
        self.scoreLabel.setStyleSheet("""
            QLabel {
                color: #7f8c8d;
                font-size: 16px;
                font-weight: normal;
            }
        """)
        self.scoreLayout.addWidget(self.scoreLabel)
        
        self.scoreDisplay = QtWidgets.QLabel("0")
        self.scoreDisplay.setStyleSheet("""
            QLabel {
                color: #e74c3c;
                font-size: 32px;
                font-weight: bold;
            }
        """)
        self.scoreLayout.addWidget(self.scoreDisplay)
        
        # 将信息添加到顶部卡片
        self.infoLayout.addWidget(self.teamInfo)
        self.infoLayout.addStretch()
        self.infoLayout.addWidget(self.scoreInfo)
        
        # 添加顶部卡片到主布局
        self.mainLayout.addWidget(self.infoCard)
        
        # 创建内容区域
        self.contentArea = QtWidgets.QHBoxLayout()
        self.contentArea.setSpacing(20)
        
        # 左侧答案区域
        self.answersCard = QtWidgets.QWidget()
        self.answersCard.setStyleSheet("""
            QWidget {
                background-color: white;
                border-radius: 15px;
                box-shadow: 0 2px 10px rgba(0, 0, 0, 0.1);
            }
            QLabel {
                color: #2c3e50;
                font-size: 18px;
                font-weight: bold;
                padding: 10px 0;
            }
            QTextBrowser {
                border: 1px solid #e0e0e0;
                border-radius: 8px;
                padding: 10px;
                font-size: 16px;
                background-color: #f8f9fa;
                color: #2c3e50;
            }
        """)
        
        self.answersLayout = QtWidgets.QVBoxLayout(self.answersCard)
        self.answersLayout.setContentsMargins(20, 20, 20, 20)
        
        # 正确答案部分
        self.correctLabel = QtWidgets.QLabel("正确答案")
        self.answersLayout.addWidget(self.correctLabel)
        
        self.correctAnswer = QtWidgets.QTextBrowser()
        self.correctAnswer.setMaximumHeight(100)
        self.answersLayout.addWidget(self.correctAnswer)
        
        # 添加分隔线
        self.line = QtWidgets.QFrame()
        self.line.setFrameShape(QtWidgets.QFrame.HLine)
        self.line.setStyleSheet("background-color: #e0e0e0;")
        self.answersLayout.addWidget(self.line)
        
        # 选手答案部分
        self.playerLabel = QtWidgets.QLabel("选手答案")
        self.answersLayout.addWidget(self.playerLabel)
        
        self.playerAnswer = QtWidgets.QTextBrowser()
        self.playerAnswer.setMaximumHeight(100)
        self.answersLayout.addWidget(self.playerAnswer)
        
        # 右侧视频区域
        self.videoCard = QtWidgets.QWidget()
        self.videoCard.setStyleSheet("""
            QWidget {
                background-color: white;
                border-radius: 15px;
                box-shadow: 0 2px 10px rgba(0, 0, 0, 0.1);
            }
            QLabel {
                color: #2c3e50;
                font-size: 18px;
                font-weight: bold;
                padding: 10px 0;
            }
        """)
        
        self.videoLayout = QtWidgets.QVBoxLayout(self.videoCard)
        self.videoLayout.setContentsMargins(20, 20, 20, 20)
        
        self.videoLabel = QtWidgets.QLabel("实时视频流")
        self.videoLayout.addWidget(self.videoLabel)
        
        self.videoDisplay = ROSVideoDisplay()
        self.videoDisplay.setMinimumSize(640, 480)
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