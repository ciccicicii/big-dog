#!/usr/bin/env python3
import self_pose
import lcm #lcm通信e核心库
import sys
import os
import time
from threading import Thread, Lock # 多线程+锁，处理并发收发
from robot_control_cmd_lcmt import robot_control_cmd_lcmt # 导入预定义的LCM消息类（自动生成，对应控制指令/响应的结构化数据）
from robot_control_response_lcmt import robot_control_response_lcmt
class Robot_Ctrl(object):
    def __init__(self):
        # 1. 初始化收发线程（响应接收+指令发送）
        self.rec_thread = Thread(target=self.rec_responce)
        self.send_thread = Thread(target=self.send_publish)
        # 2. 初始化LCM对象（收发不同的UDP组播地址）
        self.lc_r = lcm.LCM("udpm://239.255.76.67:7670?ttl=255")
        self.lc_s = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
        # 3. 初始化消息对象（指令/响应）
        self.cmd_msg = robot_control_cmd_lcmt()
        self.rec_msg = robot_control_response_lcmt()
        # 4. 线程锁（保护共享变量）
        self.send_lock = Lock() 
        self.delay_cnt = 0 # 发送延时计数器（用于心跳）
        self.mode_ok = 0 # 已完成的动作模式ID
        self.gait_ok = 0 # 已完成的步态ID
        self.runing = 1 # 线程运行标志（1=运行，0=停止）
        self.life_count = 0
    def next_life(self): # 生命计数器（0-255循环），每次调用+1，用于指令心跳
        self.life_count = (self.life_count + 1) % 256
        return self.life_count
    def run(self):
        # 1. 订阅控制板响应的LCM通道（"robot_control_response"）
        self.lc_r.subscribe("robot_control_response", self.msg_handler)
        # 2. 启动发送/接收线程
        self.send_thread.start()
        self.rec_thread.start()

    def msg_handler(self, channel, data):
        # LCM响应消息处理回调函数（控制板返回数据时触发）  类似与callbackn函数
        self.rec_msg = robot_control_response_lcmt().decode(data)
        # 动作执行进度≥95% → 标记该动作完成
        if(self.rec_msg.order_process_bar >= 95):
            self.mode_ok = self.rec_msg.mode # 记录完成的模式ID
        else:
            self.mode_ok = 0

    def rec_responce(self):
        # 响应接收线程：持续监听LCM消息
        while self.runing: 
            self.lc_r.handle() # 处理待接收的LCM消息（触发msg_handler）
            time.sleep( 0.002 ) # 500Hz监听频率，降低CPU占用

    def Wait_finish(self, mode, gait_id):# 等待指定动作执行完成（阻塞，最长10s）
        count = 0
        while self.runing and count < 2000: #10s
            # 等待指定动作执行完成（阻塞，最长10s）
            if self.mode_ok == mode and self.gait_ok == gait_id:
                return True
            else:
                time.sleep(0.005)
                count += 1
        
    def send_publish(self): # 指令发送线程：10Hz心跳发送（维持控制连接）
        while self.runing:
            self.send_lock.acquire() # 加锁保护共享变量
            if self.delay_cnt > 20: # Heartbeat signal 10HZ, It is used to maintain the heartbeat when life count is not updated # 每20次循环（20*0.005=0.1s）发送一次心跳（10Hz）
                print("Send cmd: mode={}, gait_id={}, life_count={}".format(self.cmd_msg.mode, self.cmd_msg.gait_id, self.cmd_msg.life_count))
                self.lc_s.publish("robot_control_cmd",self.cmd_msg.encode())# 发布指令到LCM通道"robot_control_cmd"
                self.delay_cnt = 0
            self.delay_cnt += 1
            self.send_lock.release() # 重置计数器
            time.sleep( 0.005 ) 

    def Send_cmd(self, msg):# 下发指令接口：更新指令并触发立即发送（重置delay_cnt）
        msg.life_count = self.next_life() # 更新指令的生命计数（心跳）
        self.send_lock.acquire() #枷锁保护共享a变量
        self.delay_cnt = 50 # 强制触发发送（>20）
        self.cmd_msg = msg   # 更新待发送的指令
        self.send_lock.release() #解锁

    def quit(self):
        # 退出函数：停止所有线程
        self.runing = 0  # 置0后，收发线程会退出循环
        self.rec_thread.join() # 等待接收线程结束
        self.send_thread.join() # 等待发送线程结束


class COM:
    def __init__(self):
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

    def Stone(self):
        """第一关：石板路"""
        print("\n=== 第1关：石板路 ===")
        
        self.msg.mode = 12 # Recovery stand
        self.msg.gait_id = 0
        self.ctrl.Send_cmd(self.msg)    # 发送指令


        #self.base_move.restand_1(self.msg,self.ctrl) # 预备动作：站立准备
        # self.msg = self.Pose.get_self_gait("stone_path") 
        # self.ctrl.Send_cmd(self.msg) # 石板路行走
        # print("石板路行走中...")
        # time.sleep(10)  # 等待5秒完成石板路挑战
        # self.base_move.restand_1(self.msg,self.ctrl) # 结束动作：站立准备
        # print("石板路挑战完成！")
        # self.base_move.turn_l(self.msg,self.ctrl,0.4,0) # 转向准备下一关
        # self.base_move.restand_1(self.msg,self.ctrl)
        # print("转向完成，准备下一关...")
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
        self.Stone()
        # ball()
        # quxian()
        # cocola()
        # orange_ball()
        # football()
        # bridge_0()
        # bridge_1()
        # bridge_2()
        # bridge_3()
        # bridge_4()
        # jump()
        # tick_football()

if __name__ == "__main__":
    com = COM()
    com.ctrl.run() # 启动控制器（开启LCM通信线程）
    com.start_competition()
        