import rospy
import yaml
from std_msgs.msg import String
import sys, os
import select
import termios
import tty
import threading
import signal
import atexit

class JudgeListener:
    def __init__(self, yaml_file):
        # 保存原始终端设置
        self.original_terminal_settings = termios.tcgetattr(sys.stdin)
        
        # 注册终端恢复函数
        atexit.register(self.restore_terminal)
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        # Load the YAML file
        with open(yaml_file, 'r') as file:
            config = yaml.safe_load(file)
        
        self.correct_answer = config['correct_answer']
        self.stage = int(config['stage'])
        self.score = 0
        self.last_score = 0
        self.running = True

        rospy.init_node('judge_listener', anonymous=True)
        rospy.Subscriber('/judge', String, self.judge_callback)

        # Publisher for /score topic
        self.score_pub = rospy.Publisher('/score', String, queue_size=10)

        self.terminal_thread = threading.Thread(target=self.modify_score)
        self.terminal_thread.daemon = True
        self.terminal_thread.start()

        self.score_publisher_thread = threading.Thread(target=self.publish_score)
        self.score_publisher_thread.daemon = True
        self.score_publisher_thread.start()

    def signal_handler(self, signum, frame):
        """处理信号，确保程序正常退出"""
        self.running = False
        self.restore_terminal()
        sys.exit(0)

    def configure_terminal(self):
        """配置终端为非规范模式"""
        tty.setcbreak(sys.stdin.fileno())

    def restore_terminal(self):
        """恢复终端设置"""
        try:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.original_terminal_settings)
        except:
            pass

    def judge_callback(self, msg):
        user_answer = msg.data
        
        # Compare the user answer with the correct answer
        correct_count = sum(1 for a, b in zip(user_answer, self.correct_answer) if a == b)
        
        if correct_count > 0:
            if self.stage == 1:
                self.score -= self.last_score
                self.score += correct_count * 10
                self.last_score = correct_count * 10
            elif self.stage == 2:
                self.score -= self.last_score
                self.score += correct_count * 15
                self.last_score = correct_count * 15
            else:
                rospy.logwarn("Unsupported stage: %d" % self.stage)
                return

            rospy.loginfo(f"Correct count:{correct_count}, score: {self.score}")
        else:
            rospy.loginfo("Incorrect answer.")

    def modify_score(self):
        rospy.loginfo("Press 'a' to add 10, 'd' to subtract 10, 's' to submit score.")
        self.configure_terminal()
        
        try:
            while self.running and not rospy.is_shutdown():
                if is_data_available():
                    key = sys.stdin.read(1)
                    if key == 'a':
                        self.score += 10
                        rospy.loginfo(f"Score increased: {self.score}")
                    elif key == 'd':
                        self.score -= 10
                        rospy.loginfo(f"Score decreased: {self.score}")
                    elif key == 's':
                        rospy.loginfo(f"Final Score submitted: {self.score}")
                        self.running = False
                        break
                rospy.sleep(0.1)
        except Exception as e:
            rospy.logerr(f"Error in modify_score: {e}")
        finally:
            self.restore_terminal()
    
    def publish_score(self):
        rate = rospy.Rate(1) 
        while self.running and not rospy.is_shutdown():
            self.score_pub.publish(str(self.score))
            rate.sleep()

def is_data_available():
    return select.select([sys.stdin], [], [], 0)[0] != []

if __name__ == "__main__":
    config_file = os.path.join(os.path.dirname(__file__), 'config.yaml')

    try:
        judge_listener = JudgeListener(config_file)
        rospy.spin()
    except FileNotFoundError:
        print(f"YAML file '{config_file}' not found.")
    except KeyError as e:
        print(f"Missing key in YAML file: {e}")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'judge_listener' in locals():
            judge_listener.restore_terminal()
