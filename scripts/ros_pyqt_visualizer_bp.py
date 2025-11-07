#!/usr/bin/env python
# -*- coding: utf-8 -*-

import warnings
warnings.filterwarnings("ignore", category=DeprecationWarning)

import sys, os
import multiprocessing
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton
from PyQt5.QtCore import QTimer, QTime, Qt
from std_msgs.msg import String
from qt_gui import Ui_MainWindow
import yaml
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from PyQt5.QtGui import QImage, QPixmap


# =================== ROS Worker 子进程 =================== #
def ros_worker(master_uri, queue):
    """子进程：连接指定 ROS_MASTER_URI，转发消息到队列"""
    import rospy
    from std_msgs.msg import String

    os.environ["ROS_MASTER_URI"] = master_uri
    rospy.init_node("qt_visualizer_worker", anonymous=True, disable_signals=True)

    def judge_cb(msg):
        queue.put(("judge", msg.data))

    def score_cb(msg):
        queue.put(("score", msg.data))

    def team_cb(msg):
        queue.put(("team", msg.data))
    
    bridge = CvBridge()

    def image_cb(msg):
        try:
            cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            queue.put(("image", cv_image))
        except Exception as e:
            rospy.logerr(f"图像转换失败: {e}")

    # def result_cb(msg):
    #     queue.put(("result", msg.data))

    rospy.Subscriber("/judge", String, judge_cb)
    rospy.Subscriber("/score", String, score_cb)
    rospy.Subscriber("/team_name", String, team_cb)
    # sub4 = rospy.Subscriber("/result", String, result_cb)
    rospy.Subscriber("/image", Image, image_cb)

    pub = rospy.Publisher("/judge", String, queue_size=10)

    # 循环等待
    rospy.spin()


# =================== 主窗口 GUI =================== #
class RosVisualizer(QMainWindow):
    def __init__(self):
        super(RosVisualizer, self).__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        # 队伍列表
        teams = self.load_team_list()
        for t in teams:
            self.ui.teamComboBox.addItem(t["name"], t["ros_master"])
        self.ui.teamComboBox.currentTextChanged.connect(self.update_team_name)

        # 按钮绑定
        self.ui.timerStartButton.clicked.connect(self.start_match_timer)
        self.ui.timerStopButton.clicked.connect(self.stop_match_timer)
        self.ui.takeoffButton.clicked.connect(self.handle_takeoff)
        self.ui.landingButton.clicked.connect(self.handle_landing)
        self.ui.refreshButton.clicked.connect(self.reset_all)

        # 状态变量
        self.takeoff_done = False
        self.landing_done = False
        self.received_data = ""
        self.current_score = "0"
        self.latest_result = ""

        # 比赛计时器
        self.match_timer = QTime(0, 0)
        self.match_timer_running = False
        # self.match_timer_update = QTimer()
        # self.match_timer_update.timeout.connect(self.update_match_timer)

        # 周期刷新 GUI + 队列监听
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_gui)
        self.timer.start(100)

        # ROS Worker 子进程
        self.ros_proc = None
        self.ros_queue = None

    # ---------------- 队伍管理 ---------------- #
    def load_team_list(self):
        team_file = os.path.join(os.path.dirname(__file__), "teams.yaml")
        try:
            with open(team_file, "r", encoding="utf-8") as f:
                config = yaml.safe_load(f)
            return config.get("teams", [])
        except Exception as e:
            print(f"[Error] 加载队伍列表失败: {e}")
            return []

    def update_team_name(self, name):
        index = self.ui.teamComboBox.currentIndex()
        new_master = self.ui.teamComboBox.itemData(index)
        print(f"[INFO] 切换到队伍 {name}, ROS_MASTER_URI={new_master}")

        # 停掉旧的 worker
        if self.ros_proc:
            self.ros_proc.terminate()
            self.ros_proc = None

        # 启动新的 worker
        q = multiprocessing.Queue()
        p = multiprocessing.Process(target=ros_worker, args=(new_master, q))
        p.start()
        self.ros_proc = p
        self.ros_queue = q

        # 重置界面
        self.reset_all()

    # ---------------- ROS 回调逻辑 ---------------- #
    def handle_ros_msg(self, topic, data):
        if topic == "judge":
            self.callback(String(data))
        elif topic == "score":
            self.score_callback(String(data))
        elif topic == "team":
            self.team_callback(String(data))
        # elif topic == "result":
        #     self.result_callback(String(data))

    # ---------------- GUI 更新 ---------------- #
    def update_gui(self):
        if self.ros_queue and not self.ros_queue.empty():
            topic, data = self.ros_queue.get()
            if topic == "image":
                self.update_video_display(data)
            else:
                self.handle_ros_msg(topic, data)

        if self.match_timer_running:
            self.match_timer = self.match_timer.addMSecs(100)
            self.ui.timerDisplay.setText(self.match_timer.toString("mm:ss.zzz"))

        self.ui.scoreDisplay.setText(self.current_score)
    
    def update_video_display(self, frame):
        """frame 是 numpy 数组 (H, W, 3)"""
        h, w, ch = frame.shape
        bytes_per_line = 3 * w
        q_img = QImage(frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
        self.ui.videoDisplay.setPixmap(QPixmap.fromImage(q_img))

    def closeEvent(self, event):
        if self.ros_proc:
            self.ros_proc.terminate()
        event.accept()

    # ---------------- 回调函数 ---------------- #
    def callback(self, msg):
        self.received_data = msg.data

    def score_callback(self, msg):
        self.current_score = msg.data

    def team_callback(self, msg):
        pass  # 队伍名字直接用下拉框

    # def result_callback(self, msg):
    #     if self.match_timer_running:
    #         self.latest_result = msg.data
    #         self.ui.resultDisplay.setText(self.latest_result)

    # ---------------- 比赛控制 ---------------- #
    def start_match_timer(self):
        self.match_timer_running = True
        self.match_timer = QTime(0, 0)
        self.ui.timerStartButton.setEnabled(False)
        self.ui.timerStopButton.setEnabled(True)

    def stop_match_timer(self):
        self.match_timer_running = False
        self.ui.timerStartButton.setEnabled(True)
        self.ui.timerStopButton.setEnabled(False)

    def handle_takeoff(self):
        if not self.takeoff_done:
            self.takeoff_done = True
            self.ui.takeoffButton.setEnabled(False)
            self.ui.takeoffButton.setStyleSheet("background-color: #27ae60; color: white;")
            if self.ros_queue:
                msg = String(data="takeoff")
                self.ros_queue.put(("publish", msg))


    def handle_landing(self):
        if not self.landing_done:
            self.landing_done = True
            self.ui.landingButton.setEnabled(False)
            self.ui.landingButton.setStyleSheet("background-color: #27ae60; color: white;")

    # ---------------- 重置 ---------------- #
    def reset_all(self):
        self.current_score = "0"
        self.ui.scoreDisplay.setText("0")

        self.match_timer = QTime(0, 0)
        self.match_timer_running = False
        self.ui.timerDisplay.setText("00:00.000")
        self.ui.timerStartButton.setEnabled(True)
        self.ui.timerStopButton.setEnabled(False)

        for row in self.ui.gridLabels:
            for label in row:
                label.setStyleSheet("background-color: #f8f9fa; border: 1px solid #e0e0e0;")
                label.setText("")
                label.setProperty("active", False)
                label.setProperty("score", 0)

        positions = {
            (0, 2): "1号", (4, 2): "2号", (1, 1): "3号",
            (3, 3): "4号", (2, 0): "5号", (2, 4): "6号",
            (3, 1): "7号", (1, 3): "8号", (2, 2): "9号"
        }
        for pos, text in positions.items():
            self.ui.gridLabels[pos[0]][pos[1]].setText(text)

        self.circle1_button = QPushButton("圈1")
        self.circle2_button = QPushButton("圈2")

        self.takeoff_done = False
        self.landing_done = False
        self.ui.takeoffButton.setEnabled(True)
        self.ui.takeoffButton.setStyleSheet("")
        self.ui.landingButton.setEnabled(True)
        self.ui.landingButton.setStyleSheet("")

        # self.latest_result = ""
        # self.ui.resultDisplay.setText("")


# =================== 主入口 =================== #
def main():
    multiprocessing.set_start_method("spawn")  # Mac/Windows 兼容
    app = QApplication(sys.argv)
    window = RosVisualizer()
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
