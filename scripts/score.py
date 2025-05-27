import rospy
import yaml
from std_msgs.msg import String
import sys, os
import signal
import atexit
import threading

class JudgeListener:
    def __init__(self, yaml_file):
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
        
        # 启动分数发布线程
        self.score_publisher_thread = threading.Thread(target=self.publish_score)
        self.score_publisher_thread.daemon = True
        self.score_publisher_thread.start()

    def judge_callback(self, msg):
        """处理识别结果和分数调整"""
        data = msg.data
        
        # 处理分数调整消息
        if data.startswith('score_adjust_'):
            try:
                score_change = int(data.split('_')[-1])
                self.score += score_change
                if score_change > 0:
                    rospy.loginfo(f"手动加{score_change}分，当前总分：{self.score}")
                else:
                    rospy.loginfo(f"手动减{-score_change}分，当前总分：{self.score}")
                return
            except ValueError:
                rospy.logerr("无效的分数调整消息")
                return
        
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
    
    def publish_score(self):
        """发布分数"""
        rate = rospy.Rate(1) 
        while not rospy.is_shutdown():
            self.score_pub.publish(str(self.score))
            rate.sleep()

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
