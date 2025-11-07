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
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    import cv2

    os.environ["ROS_MASTER_URI"] = master_uri
    rospy.init_node("qt_visualizer_worker", anonymous=True, disable_signals=True)

    def judge_cb(msg):
        queue.put(("judge", msg.data))

    bridge = CvBridge()

    def image_cb(msg):
        try:
            cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            queue.put(("image", cv_image))
        except Exception as e:
            rospy.logerr(f"图像转换失败: {e}")

    rospy.Subscriber("/judge", String, judge_cb)
    rospy.Subscriber("/image", Image, image_cb)

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
        self.match_timer = QTime(0, 0)
        self.match_timer_running = False

        # 分数系统变量
        self.score = 0
        self.recognition_results = set()  # 记录已识别的 (位置, 类型)

        # 九宫格点击事件绑定
        self.init_grid_click_events()

        # 圈按钮
        self.circle1_button = QPushButton("圈1")
        self.circle2_button = QPushButton("圈2")
        self.ui.gridLayout.addWidget(self.circle1_button, 1, 2)
        self.ui.gridLayout.addWidget(self.circle2_button, 3, 2)
        self.circle1_button.clicked.connect(lambda: self.handle_circle_click(1))
        self.circle2_button.clicked.connect(lambda: self.handle_circle_click(2))
        self.fixed_positions = {
            (0, 2): "1号", (4, 2): "2号", (1, 1): "3号",
            (3, 3): "4号", (2, 0): "5号", (2, 4): "6号",
            (3, 1): "7号", (1, 3): "8号", (2, 2): "9号"
        }

        # 定时器：统一刷新 GUI 和 ROS 消息
        self.gui_timer = QTimer()
        self.gui_timer.timeout.connect(self.update_gui)
        self.gui_timer.start(100)

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

    # ---------------- ROS 消息处理 ---------------- #
    def handle_ros_msg(self, topic, data):
        """处理 /judge 消息：集成分数逻辑"""
        if topic != "judge":
            return

        msg = data.strip()

        # ---- 1. 手动调整分数 ----
        if msg.startswith('score_adjust_'):
            try:
                score_change = int(msg.split('_')[-1])
                self.score += score_change
                self.ui.scoreDisplay.setText(str(self.score))
                return
            except ValueError:
                print("[Error] 无效的分数调整消息")
                return

        # ---- 2. 起飞 / 降落 ----
        if msg == "takeoff":
            self.score += 5
            self.ui.scoreDisplay.setText(str(self.score))
            return
        elif msg == "landing":
            self.score += 5
            self.ui.scoreDisplay.setText(str(self.score))
            return

        # ---- 3. 识别结果 ----
        try:
            position, result_type = msg[0], msg[1]
            if (position, result_type) not in self.recognition_results:
                self.recognition_results.add((position, result_type))
                if result_type == 'A':
                    self.score += 3
                elif result_type == 'B':
                    self.score += 1
                elif result_type == 'C':
                    self.score += 6
                self.ui.scoreDisplay.setText(str(self.score))
        except Exception as e:
            print(f"[Error] 处理识别结果出错: {e}")

    # ---------------- GUI 更新 ---------------- #
    def update_gui(self):
        # 从队列读取 ROS worker 消息
        if self.ros_queue and not self.ros_queue.empty():
            topic, data = self.ros_queue.get()
            if topic == "image":
                self.update_video_display(data)
            else:
                self.handle_ros_msg(topic, data)

        # 更新计时器
        if self.match_timer_running:
            self.match_timer = self.match_timer.addMSecs(100)
            self.ui.timerDisplay.setText(self.match_timer.toString("mm:ss.zzz"))

        # 更新分数
        self.ui.scoreDisplay.setText(str(self.score))

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

    # ---------------- 操作按钮 ---------------- #
    def handle_takeoff(self):
        if not self.takeoff_done:
            self.takeoff_done = True
            self.ui.takeoffButton.setEnabled(False)
            self.ui.takeoffButton.setStyleSheet("background-color: #27ae60; color: white;")
            self.handle_ros_msg("judge", "takeoff")

    def handle_landing(self):
        if not self.landing_done:
            self.landing_done = True
            self.ui.landingButton.setEnabled(False)
            self.ui.landingButton.setStyleSheet("background-color: #27ae60; color: white;")
            self.handle_ros_msg("judge", "landing")

    # ---------------- 九宫格点击事件 ---------------- #
    def init_grid_click_events(self):
        """绑定每个格子的点击事件"""
        positions = {
            (0, 2): "1", (4, 2): "2", (1, 1): "3",
            (3, 3): "4", (2, 0): "5", (2, 4): "6",
            (3, 1): "7", (1, 3): "8", (2, 2): "9"
        }
        for pos, num in positions.items():
            label = self.ui.gridLabels[pos[0]][pos[1]]
            label.mousePressEvent = lambda e, n=num: self.handle_grid_click(e, n)
            label.setStyleSheet("background-color: #f8f9fa; border: 1px solid #e0e0e0;")

    def handle_grid_click(self, event, number):
        """根据左键或右键点击调整分数"""
        label = None
        for pos, num in {
            (0, 2): "1", (4, 2): "2", (1, 1): "3",
            (3, 3): "4", (2, 0): "5", (2, 4): "6",
            (3, 1): "7", (1, 3): "8", (2, 2): "9"
        }.items():
            if num == number:
                label = self.ui.gridLabels[pos[0]][pos[1]]
                break
        if not label:
            return

        is_active = label.property("active") or False

        if not is_active:
            # 新点击：左键 +1，右键 +3；9号左键 +6
            if number == "9":
                score = 6
                color = "#FF0000"
            elif event.button() == Qt.LeftButton:
                score = 1
                color = "#90EE90"
            else:
                score = 3
                color = "#228B22"
            label.setStyleSheet(f"background-color: {color}; color: white;")
            label.setProperty("active", True)
            label.setProperty("score", score)
            self.score += score
        else:
            # 再次点击取消激活，扣回分
            score = -label.property("score")
            label.setStyleSheet("background-color: #f8f9fa; border: 1px solid #e0e0e0;")
            label.setProperty("active", False)
            self.score += score

        self.ui.scoreDisplay.setText(str(self.score))

    # ---------------- 圈按钮 ---------------- #
    def handle_circle_click(self, circle_num):
        button = self.circle1_button if circle_num == 1 else self.circle2_button
        is_active = button.property("active") or False
        if not is_active:
            button.setStyleSheet("background-color: #27ae60; color: white;")
            button.setProperty("active", True)
            score = 10
        else:
            button.setStyleSheet("")
            button.setProperty("active", False)
            score = -10
        self.score += score
        self.ui.scoreDisplay.setText(str(self.score))

    # ---------------- 重置 ---------------- #
    def reset_all(self):
        # 分数与识别记录
        self.score = 0
        self.ui.scoreDisplay.setText("0")
        self.recognition_results.clear()

        # 计时器
        self.match_timer = QTime(0, 0)
        self.match_timer_running = False
        self.ui.timerDisplay.setText("00:00.000")
        self.ui.timerStartButton.setEnabled(True)
        self.ui.timerStopButton.setEnabled(False)

        # 网格：样式/状态清空，但固定位置的文字要恢复
        for i, row in enumerate(self.ui.gridLabels):
            for j, label in enumerate(row):
                label.setStyleSheet("background-color: #f8f9fa; border: 1px solid #e0e0e0;")
                label.setProperty("active", False)
                label.setProperty("score", 0)
                # 恢复文字：在固定表里用“1号~9号”，否则置空
                text = self.fixed_positions.get((i, j), "")
                label.setText(text)

        # 圈按钮：只重置状态与样式，千万别重新创建按钮
        if hasattr(self, "circle1_button"):
            self.circle1_button.setProperty("active", False)
            self.circle1_button.setStyleSheet("")
        if hasattr(self, "circle2_button"):
            self.circle2_button.setProperty("active", False)
            self.circle2_button.setStyleSheet("")

        # 起飞/降落按钮
        self.takeoff_done = False
        self.landing_done = False
        self.ui.takeoffButton.setEnabled(True)
        self.ui.takeoffButton.setStyleSheet("")
        self.ui.landingButton.setEnabled(True)
        self.ui.landingButton.setStyleSheet("")



# =================== 主入口 =================== #
def main():
    multiprocessing.set_start_method("spawn")
    app = QApplication(sys.argv)
    window = RosVisualizer()
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
