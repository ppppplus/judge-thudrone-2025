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
        
        # 加载配置文件
        with open(yaml_file, 'r') as file:
            config = yaml.safe_load(file)
        
        self.team_name = config['team_name']
        self.score = 0
        self.running = True
        
        # 初始化识别结果记录
        self.recognition_results = set()  # 用于记录已识别的位置和类型
        
        rospy.init_node('judge_listener', anonymous=True)
        rospy.Subscriber('/judge', String, self.judge_callback)
        
        # 发布分数
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
        """处理识别结果"""
        data = msg.data
        
        # 处理起飞和降落
        if data == "takeoff":
            self.score += 5
            rospy.loginfo("起飞成功：+5分")
            return
        elif data == "landing":
            self.score += 5
            rospy.loginfo("降落成功：+5分")
            return
            
        try:
            # 假设数据格式为"1A"、"2B"等，第一个字符是位置，第二个字符是类型
            position = data[0]
            result_type = data[1]
            
            # 检查是否已经识别过这个位置
            if (position, result_type) not in self.recognition_results:
                self.recognition_results.add((position, result_type))
                
                # 根据类型给分
                if result_type == 'A':
                    self.score += 3
                    rospy.loginfo(f"识别到A类图片：+3分")
                elif result_type == 'B':
                    self.score += 1
                    rospy.loginfo(f"识别到B类图片：+1分")
                elif result_type == 'C':
                    self.score += 6
                    rospy.loginfo(f"识别到C类图片：+6分")
                
                rospy.loginfo(f"当前总分：{self.score}")
            else:
                rospy.loginfo(f"位置{position}的{result_type}类图片已经被识别过")
                
        except Exception as e:
            rospy.logerr(f"处理识别结果错误：{str(e)}")

    def modify_score(self):
        """手动修改分数"""
        rospy.loginfo("按键说明：")
        rospy.loginfo("'a': +1分")
        rospy.loginfo("'s': +3分")
        rospy.loginfo("'d': +5分")
        rospy.loginfo("'f': +6分")
        rospy.loginfo("'z': -1分")
        rospy.loginfo("'x': -3分")
        rospy.loginfo("'c': -5分")
        rospy.loginfo("'v': -6分")
        rospy.loginfo("'q': 退出程序")
        
        self.configure_terminal()
        
        try:
            while self.running and not rospy.is_shutdown():
                if is_data_available():
                    key = sys.stdin.read(1)
                    if key == 'a':
                        self.score += 1
                        rospy.loginfo(f"加1分，当前分数：{self.score}")
                    elif key == 's':
                        self.score += 3
                        rospy.loginfo(f"加3分，当前分数：{self.score}")
                    elif key == 'd':
                        self.score += 5
                        rospy.loginfo(f"加5分，当前分数：{self.score}")
                    elif key == 'f':
                        self.score += 6
                        rospy.loginfo(f"加6分，当前分数：{self.score}")
                    elif key == 'z':
                        self.score -= 1
                        rospy.loginfo(f"减1分，当前分数：{self.score}")
                    elif key == 'x':
                        self.score -= 3
                        rospy.loginfo(f"减3分，当前分数：{self.score}")
                    elif key == 'c':
                        self.score -= 5
                        rospy.loginfo(f"减5分，当前分数：{self.score}")
                    elif key == 'v':
                        self.score -= 6
                        rospy.loginfo(f"减6分，当前分数：{self.score}")
                    elif key == 'q':
                        rospy.loginfo(f"最终分数：{self.score}")
                        self.running = False
                        break
                rospy.sleep(0.1)
        except Exception as e:
            rospy.logerr(f"Error in modify_score: {e}")
        finally:
            self.restore_terminal()
    
    def publish_score(self):
        """发布分数"""
        rate = rospy.Rate(1) 
        while self.running and not rospy.is_shutdown():
            self.score_pub.publish(str(self.score))
            rate.sleep()

def is_data_available():
    """检查是否有键盘输入"""
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
