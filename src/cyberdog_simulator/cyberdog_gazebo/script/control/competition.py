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
import math
from math import atan2, asin, copysign, pi
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy #设置Qos
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
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
        #雷达控制
        self.lidar_ranges = None
        self.lidar_angle_min = 0.0
        self.lidar_angle_increment = 0.0
        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            scan_qos
        )
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
        #角度控制
        # IMU订阅
        imu_qos = QoSProfile( #设置Qos为Best_efforts
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            imu_qos
        )
        self.current_yaw = None
        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.yaw_zero = None
        self.global_yaw = {
            "front": None,
            "left": None,
            "back": None,
            "right": None,
        }
        self.imu_ready = False
        #自恢复
        self.motion_lock = Lock() # 运动控制锁，确保同一时间只有一个赛段在控制机器人
        self.is_recovering = False
        self.fall_detected = False
        self.pause_competition = False
        self.roll = 0.0  #判断机器人是否侧翻的参数
        self.pitch = 0.0
        self.body_height = 0.28 

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
    #a雷达参数回调
    def lidar_callback(self, msg):
        self.lidar_ranges = list(msg.ranges)
        self.lidar_angle_min = msg.angle_min
        self.lidar_angle_increment = msg.angle_increment
    def get_lidar_distance_at_angle(self, target_angle, window_deg=5):
        if self.lidar_ranges is None:
            return None
        window = math.radians(window_deg)
        min_dist = float('inf')
        for i, r in enumerate(self.lidar_ranges):
            angle = self.lidar_angle_min + i * self.lidar_angle_increment
            if abs(self.angle_diff(target_angle, angle)) < window:
                if math.isfinite(r) and 0.05 < r < min_dist:
                    min_dist = r
        if min_dist == float('inf'):
            return None
        return min_dist

    def yellow_callback(self, msg):
        self.yellow_line = msg.data
    def yellow_ratio_callback(self, msg):
        self.yellow_ratio = msg.data
    def imu_callback(self, msg):
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        w = msg.orientation.w
        # roll
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        self.current_roll = atan2(sinr_cosp, cosr_cosp)
        # pitch
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            self.current_pitch = copysign(pi / 2, sinp)
        else:
            self.current_pitch = asin(sinp)
        # yaw
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        self.current_yaw = atan2(siny_cosp, cosy_cosp)
        # 给跌倒检测用
        self.roll = self.current_roll
        self.pitch = self.current_pitch
        self.imu_ready = True
    def normalize_angle(self, angle): # 将角度规范化到[-pi, pi]范围内
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    def angle_diff(self, target, current):  # 计算当前角度与目标角度之间的最短差值，结果在[-pi, pi]范围内  本质工具函数
        return self.normalize_angle(target - current)
    def init_global_heading(self,timeout=5.0):
        """
        程序启动后记录初始航向作为全局正前方
        """
        self.get_logger().info("等待IMU稳定，记录全局起始方向...")
        start = time.time()
        while rclpy.ok() and not self.imu_ready:
            if time.time() - start > timeout:
                self.get_logger().error("等待IMU超时，请检查 /imu/data 是否存在")
                return False
            time.sleep(0.05)    
        # 连续取几次，减小抖动
        yaw_samples = []
        for _ in range(20):
            if self.current_yaw is not None:
                yaw_samples.append(self.current_yaw)
            time.sleep(0.02)
        if len(yaw_samples) == 0:
            raise RuntimeError("IMU没有可用yaw数据，无法初始化全局方向")
        # 简单取最后一个，或者可换平均
        self.yaw_zero = yaw_samples[-1]
        self.global_yaw["front"] = self.normalize_angle(self.yaw_zero)
        self.global_yaw["left"]  = self.normalize_angle(self.yaw_zero + math.pi / 2)
        self.global_yaw["back"]  = self.normalize_angle(self.yaw_zero + math.pi)
        self.global_yaw["right"] = self.normalize_angle(self.yaw_zero - math.pi / 2)

        self.get_logger().info(
            f"全局方向初始化完成: "
            f"front={self.global_yaw['front']:.3f}, "
            f"left={self.global_yaw['left']:.3f}, "
            f"back={self.global_yaw['back']:.3f}, "
            f"right={self.global_yaw['right']:.3f}"
        )
    def turn_to_global_direction(self, direction_name):
        """
        direction_name: 'front' / 'left' / 'back' / 'right'
        """
        if direction_name not in self.global_yaw:
            self.get_logger().error(f"未知方向: {direction_name}")
            return False
        target_yaw = self.global_yaw[direction_name]
        self.get_logger().info(
            f"开始转向全局方向 {direction_name}, target={target_yaw:.3f}"
        )
        start_time = time.time()
        while rclpy.ok():
            if self.current_yaw is None:
                time.sleep(0.05)
                continue
            err = self.angle_diff(target_yaw, self.current_yaw)
            # 误差足够小，停止
            if abs(err) < 0.02:   # 大约3度
                break
            # 比例控制
            yaw_speed = 1.2 * err
            # 限幅
            if yaw_speed > 0.35:
                yaw_speed = 0.35
            elif yaw_speed < -0.35:
                yaw_speed = -0.35
            # 防止快到目标时速度太小转不动
            if 0 < yaw_speed < 0.10:
                yaw_speed = 0.10
            elif -0.10 < yaw_speed < 0:
                yaw_speed = -0.10
            self.base_move.turn_l(self.msg, self.ctrl, yaw_speed, 0)
            # 超时保护
            if time.time() - start_time > 20.0:
                self.get_logger().warn("转向超时，强制停止")
                break
            time.sleep(0.05)
        # 停止
        self.base_move.restand_1(self.msg, self.ctrl)
        time.sleep(0.2)
        self.get_logger().info(f"已转到全局方向 {direction_name}")
        return True
    def turn_relative(self,delta_yaw,timeout=10.0):
        target_yaw = self.normalize_angle(self.current_yaw + delta_yaw)
        start = time.time()
        while rclpy.ok():
            if time.time() - start > timeout:
                break
            err = self.angle_diff(target_yaw, self.current_yaw)
            self.get_logger().info(f"err={err:.3f}")
            if abs(err) < 0.03:   # 约3度
                break
            # 比例控制s
            yaw_speed = 0.9 * err
            # 限幅，避免太快
            if yaw_speed > 0.6:
                yaw_speed = 0.6
            elif 0<yaw_speed < 0.05:
                yaw_speed = 0.05
            if yaw_speed <-0.6:
                yaw_speed = -0.6
            elif 0>yaw_speed >-0.05:
                yaw_speed = -0.05
            self.get_logger().info(f"yaw={yaw_speed:.3f}")
            self.base_move.turn_l(self.msg, self.ctrl, yaw_speed, 0)
            time.sleep(0.05)
        self.base_move.restand_1(self.msg, self.ctrl)
        time.sleep(0.2)
    def is_fallen(self):
        if abs(self.roll) > 0.9:
            return True
        if abs(self.pitch) > 0.9:
            return True
        if self.body_height < 0.12:
            return True
        return False
    def atuto_recover(self):
        while self.ctrl.runing and not self.all_finish:
            if not self.is_recovering and self.is_fallen():
                self.get_logger().warn("检测到机器人跌倒，开始自动恢复！")
                self.is_recovering = True
                self.pause_competition = True
                with self.motion_lock:
                    try:
                        self.base_move.restand_1(self.msg, self.ctrl) # 预备动作：站立准备
                        time.sleep(5) # 等待动作完成
                        self.get_logger().info("自动恢复完成，继续比赛")
                    except Exception as e:
                        self.get_logger().error(f"自动恢复失败: {e}")
                self.is_recovering = False
                self.pause_competition = False

    def Stone(self):
        """第一关：石板路"""
        print("\n=== 第1关：石板路 ===")
        # self.msg.mode = 12 # Recovery stand
        # self.msg.gait_id = 0
        # self.ctrl.Send_cmd(self.msg)    # 发送指令
        self.base_move.restand_1(self.msg,self.ctrl) # 预备动作：站立准备
        #self.turn_relative(-math.pi/4)
        # 2. 低头
        #self.base_move.head_down(self.msg, self.ctrl)
        time.sleep(1)
        self.msg = self.Pose.get_self_gait("stone_path") 
        self.ctrl.Send_cmd(self.msg) # 石板路行走
        print("石板路行走中...")
        time.sleep(11)  # 等待行走完成（根据实际动作时间调整）
        #转向，头朝前
        self.turn_to_global_direction("back")
        self.base_move.walk(self.msg, self.ctrl, 0.15, 1, 0.15, 0.26)
        start_time = time.time()
        while time.time() - start_time < 8.0:
            left_dist = self.get_lidar_distance_at_angle(math.pi / 2, window_deg=6)
            if left_dist is None:
                time.sleep(0.05)
                continue
            self.get_logger().info(f"left lidar dist={left_dist:.3f}")
            if 0.25 < left_dist < 1.2:
                self.get_logger().info("检测到小球位于机器狗左侧，第一关闭环完成")
                time.sleep(1)
                break
            time.sleep(0.05)
        self.base_move.restand_1(self.msg,self.ctrl) # 结束动作：站立准备
        # 4. 循环检测黄线
        # start_time = time.time()
        # hit_count = 0
        # # 循环检测黄线
        # while time.time() - start_time < 10.0:
        #     if self.yellow_line:
        #         hit_count += 1
        #     else:
        #         hit_count = 0
        #     # 连续检测到3次再停，防抖
        #     if hit_count >= 3:
        #         print(f"检测到黄线，yellow_ratio={self.yellow_ratio:.3f}")
        #         break
        #     time.sleep(0.05)
        # # time.sleep(9)  # 等待5秒完成石板路挑战
        print("石板路挑战完成！")
        self.turn_to_global_direction("right") #左转准备下一关
        #time.sleep(9) # 朝向下一关等待转向完成
        print("转向完成")
        self.base_move.walk(self.msg,self.ctrl,0.3,1,0.15,0.26) # 前进准备下一关
        time.sleep(4.5) # 等待前进完成
        self.base_move.restand_1(self.msg,self.ctrl)
        self.turn_to_global_direction("front") # 转向准备下一关
        #time.sleep(6) # 等待转向完成
        return True
    def ball(self):
        self.base_move.walk(self.msg,self.ctrl,0.3,1,0.15,0.26) # 前进准备下一关
        time.sleep(11.4) # 等待前进完成
        self.base_move.restand_1(self.msg,self.ctrl)
        self.turn_to_global_direction("right") # 转向第一个球
        #撞击球
        self.base_move.walk(self.msg,self.ctrl,0.5,1,0.15,0.26) # 前进准备撞击球
        print("撞击第一个球")
        time.sleep(2.5) # 等待前进完成
        self.base_move.restand_1(self.msg,self.ctrl)
        self.turn_relative(-4*math.pi/12) # 转向第二个球
        #time.sleep(3) # 等待转向完成
        self.base_move.walk(self.msg,self.ctrl,0.5,1,0.15,0.26) # 前进准备撞击球
        time.sleep(9) 
        self.base_move.restand_1(self.msg,self.ctrl)
        self.turn_to_global_direction("right") # 转向准备撞击第三个球
        #time.sleep(3) # 等待转向完成
        self.base_move.walk(self.msg,self.ctrl,0.5,1,0.15,0.26) # 前进准备撞击球
        print("撞击第2个球")
        time.sleep(2.7)
        self.base_move.restand_1(self.msg,self.ctrl)
        self.turn_to_global_direction("front") # 转向准备撞击第四个球
        #time.sleep(6) # 等待转向完成
        self.base_move.walk(self.msg,self.ctrl,0.3,1,0.15,0.26) # 前进准备撞击第四个球
        time.sleep(17) # 等待前进完成
        self.base_move.restand_1(self.msg,self.ctrl)
        self.turn_to_global_direction("right") # 转向准备撞击第四个球
        self.base_move.walk(self.msg,self.ctrl,0.5,1,0.15,0.26) # 撞击第四个球
        print("撞击第四个球")
        time.sleep(3) # 等待前进完成
        print("平地速度挑战完成！到达第三关")
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
        # 先等IMU并初始化全局方向
        self.init_global_heading()
        while self.ctrl.runing and not self.all_finish:
            print(f"\n当前赛段: {self.stage}")

            if self.stage == 1:
                self.state_name = "Stone"
                self.stage_finish = self.Stone()

            elif self.stage == 2:
                self.state_name = "ball"
                self.stage_finish = self.ball()

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
    #自动恢复线程
    recover_thread = Thread(target=com.atuto_recover,daemon=True)
    recover_thread.start()
    # 保持ROS节点运行，直到比赛结束
    # rclpy.spin(com)
    # # 比赛结束后清理资源
    # com.destroy_node()
    # rclspy.shutdown()
    try:
        rclpy.spin(com)
    except KeyboardInterrupt:
        pass
    finally:
        com.destroy_node()
        rclpy.shutdown()
        
