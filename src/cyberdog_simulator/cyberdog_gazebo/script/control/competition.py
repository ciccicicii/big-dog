#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
from threading import Thread
import self_pose
import lcm #lcm通信e核心库
import sys
import os
import time
from threading import Thread, Lock # 多线程+锁，处理并发收发
from robot_control_cmd_lcmt import robot_control_cmd_lcmt # 导入预定义的LCM消息类（自动生成，对应控制指令/响应的结构化数据）
from robot_control_response_lcmt import robot_control_response_lcmt
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.append(parent_dir)
from Ctrl.Robot_ctrl import Robot_Ctrl
class COM(Node):
    def __init__(self):
        super().__init__('competition_Node')
        self.pose = {
            'mode': 11,           # 运动模式
            'gait_id': 26,        # 自变频步态
            'contact': 15,        # 四脚着地
            'life_count': 0,      # 生命计数
            'vel_des': [0.0, 0.0, 0.0],  # 速度 [x, y, yaw]
            'rpy_des': [0.0, 0.0, 0.0],  # 姿态 [roll, pitch, yaw]
            'pos_des': [0.0, 0.0, 0.28], # 位置 [x, y, z]
            'acc_des': [0.0] * 6,        # 加速度
            'ctrl_point': [0.0, 0.0, 0.0],  # 控制点
            'foot_pose': [0.0] * 6,      # 足端姿态
            'step_height': [0.06, 0.06], # 抬腿高度
            'value': 0,                  # 自定义值
            'duration': 0                # 持续时间
        }
        self.ctrl = Robot_Ctrl()
        self.Pose=self_pose.CustomGaits() #需要的动作库
        self.base_move=self_pose.Dog_movements() #需要的基础动作
        self.msg=robot_control_cmd_lcmt() #指令消息对象
        self.stage = 1              # 当前大赛段
        self.sub_stage = 0          # 当前赛段内部步骤
        self.stage_finish = False   # 当前赛段是否完成
        self.all_finish = False     # 整体比赛是否完成
        self.state_name = "Stone"   # 当前状态名字（方便打印/调试）
        #视觉控制
        self.yellow_sub = self.create_subscription( # 订阅A区 视觉功能话题
            Bool,
            '/is_yellow_line',
            self.yellow_callback,
            10
        )
        self.yellow_ratio_sub = self.create_subscription( # 订阅A区 视觉功能话题
            Float32,
            '/yellow_ratio',
            self.yellow_ratio_callback,
            10
        )
        self.yellow_line = False
        self.yellow_ratio = 0.0
        print("=" * 60)
        print("  比赛主程序")
        print("=" * 60)
        print("比赛流程:")
        print("1. 石板路")
        print("2. 平地速度")
        print("3. 弯道超速")
        print("4. 跨越低杆")
        print("5. 上坡路")
        print("6. 下坡路") 
        print("7. 侧斜坡")
        print("8. 跳跃台阶")
        print("9. 踢瓶子")
        print("10. 踢足球")
        print("=" * 60)
        # self.ctrl.run() # 启动控制器（开启LCM通信线程）
    def yellow_callback(self, msg):
        self.yellow_line = msg.data
    def yellow_ratio_callback(self, msg):
        self.yellow_ratio = msg.data
    def Stone(self):
        """第一关：石板路"""
        print("\n=== 第1关：石板路 ===")
        
        # self.msg.mode = 12 # Recovery stand
        # self.msg.gait_id = 0
        # self.ctrl.Send_cmd(self.msg)    # 发送指令


        self.base_move.restand_1(self.msg,self.ctrl) # 预备动作：站立准备
        # 2. 低头
        self.base_move.head_down(self.msg, self.ctrl)
        time.sleep(1)
        self.msg = self.Pose.get_self_gait("stone_path") 
        self.ctrl.Send_cmd(self.msg) # 石板路行走
        print("石板路行走中...")
        # 4. 循环检测黄线
        start_time = time.time()
        hit_count = 0
        # 循环检测黄线
        while time.time() - start_time < 10.0:
            if self.yellow_line:
                hit_count += 1
            else:
                hit_count = 0
            # 连续检测到3次再停，防抖
            if hit_count >= 3:
                print(f"检测到黄线，yellow_ratio={self.yellow_ratio:.3f}")
                break
            time.sleep(0.05)
        # time.sleep(9)  # 等待5秒完成石板路挑战
        self.base_move.restand_1(self.msg,self.ctrl) # 结束动作：站立准备
        print("石板路挑战完成！")
        self.base_move.turn_l(self.msg,self.ctrl,0.4,0) # 转向准备下一关
        self.base_move.restand_1(self.msg,self.ctrl)
        print("转向完成，准备下一关...")
        return True
    def ball(self):
        return 0
    def quxian(self):
        return 0
    def cocola(self):
        return 0
    def orange_ball(self):
        return 0
    def football(self):
        return 0
    def bridge_0(self):
        return 0
    def bridge_1(self):
        return 0
    def bridge_2(self):
        return 0
    def bridge_3(self):
        return 0
    def bridge_4(self):
        return 0
    def jump(self):
        return 0
    def tick_football(self):
        return 0
    def get_pose(self,pose_name,default_pose):
        return use_pose.get_self_gait(pose_name,default_pose)
    


    def start_competition(self):
        while self.ctrl.runing and not self.all_finish:
            print(f"\n当前赛段: {self.stage}")

            if self.stage == 1:
                self.state_name = "Stone"
                self.stage_finish = self.Stone()

            # elif self.stage == 2:
            #     self.state_name = "ball"
            #     self.stage_finish = self.ball()

            # elif self.stage == 3:
            #     self.state_name = "quxian"
            #     self.stage_finish = self.quxian()

            # elif self.stage == 4:
            #     self.state_name = "cocola"
            #     self.stage_finish = self.cocola()

            # elif self.stage == 5:
            #     self.state_name = "bridge"
            #     self.stage_finish = self.bridge()

            # elif self.stage == 6:
            #     self.state_name = "football"
            #     self.stage_finish = self.football()

            else:
                self.all_finish = True
                break

            if self.stage_finish:
                print(f"{self.state_name} 完成，进入下一赛段")
                self.stage += 1
                self.stage_finish = False
                time.sleep(0.5)
            else:
                print(f"{self.state_name} 未完成，继续当前状态")
                time.sleep(0.1)

        print("比赛流程结束")

if __name__ == "__main__":
    # com = COM()
    # com.ctrl.run() 
    # com.start_competition()
    rclpy.init()
    com = COM()
    com.ctrl.run() # 启动控制器（开启LCM通信线程）
    # 启动比赛主流程线程
    competition_thread = Thread(target=com.start_competition)
    competition_thread.start()
    # 保持ROS节点运行，直到比赛结束
    rclpy.spin(com)
    # 比赛结束后清理资源
    com.destroy_node()
    rclspy.shutdown()
        
