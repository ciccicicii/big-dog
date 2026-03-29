'''
This demo show the communication interface of MR813 motion control board based on Lcm.
Dependency: 
- robot_control_cmd_lcmt.py # 运动控制指令的LCM消息结构定义
- robot_control_response_lcmt.py  # 运动控制响应的LCM消息结构定义
'''
"""
基于 LCM（Lightweight Communications and Marshalling）协议的 MR813 运动控制板通信 demo
，核心功能是向机器狗发送一系列预设的运动指令（如起身、握手、抬头、行走、阻
尼模式等），并通过 LCM 监听运动控制板的响应，确保指令执行完成。
"""
import lcm #lcm通信e核心库
import sys
import os
import time
from threading import Thread, Lock # 多线程+锁，处理并发收发

from robot_control_cmd_lcmt import robot_control_cmd_lcmt # 导入预定义的LCM消息类（自动生成，对应控制指令/响应的结构化数据）
from robot_control_response_lcmt import robot_control_response_lcmt

def main():
    Test_var=Tem_test_var_class()
    Ctrl = Robot_Ctrl() # 初始化机器人控制类
    Ctrl.run() # 启动收发线程
    msg = robot_control_cmd_lcmt() # 创建空的指令消息对象
    # try:
    #     # 1. 恢复站立（mode=12，gait_id=0）
    #     msg.mode = 12 # Recovery stand
    #     msg.gait_id = 0
    #     msg.life_count += 1  # 指令生效条件：life_count更新（防止重复指令）
    #     Ctrl.Send_cmd(msg)    # 发送指令
    #     Ctrl.Wait_finish(12, 0)  # 等待动作执行完成
    #     # 2. 握手（基于位置插值控制，mode=62，gait_id=2）
    #     msg.mode = 62 # Shake hand, based on position interpolation control
    #     msg.gait_id = 2
    #     msg.life_count += 1
    #     Ctrl.Send_cmd(msg)
    #     Ctrl.Wait_finish(62, 2)
    #     # 3. 双腿站立（mode=64，gait_id=0）
    #     msg.mode = 64 # Twoleg Stand
    #     msg.gait_id = 0
    #     msg.life_count += 1
    #     Ctrl.Send_cmd(msg)
    #     Ctrl.Wait_finish(64, 0)
    #     # 4. 抬头（位置插值控制，mode=21，期望姿态rpy=[0,0.3,0]，执行0.5s）
    #     msg.mode = 21 # Position interpolation control
    #     msg.gait_id = 0
    #     msg.rpy_des = [0, 0.3, 0] # Head up 抬头
    #     msg.duration = 500 # Expected execution time, 0.5s 
    #     msg.life_count += 1
    #     Ctrl.Send_cmd(msg)
    #     time.sleep( 0.5 )
    #     # 5. 低头（同理，pitch=-0.3rad，执行0.3s）
    #     msg.mode = 21 # Position interpolation control
    #     msg.gait_id = 0
    #     msg.rpy_des = [0, -0.3, 0] # Head down 低头 
    #     msg.duration = 300 
    #     msg.life_count += 1
    #     Ctrl.Send_cmd(msg)
    #     time.sleep( 0.3 )
    #     # 6. 调整身体高度（位置插值，z轴0.22m，执行0.4s）
    #     msg.mode = 21 # Position interpolation control
    #     msg.gait_id = 5
    #     msg.rpy_des = [0, 0, 0] #头部复位 
    #     msg.pos_des = [0, 0, 0.22] # Set body height # 期望身体位置（x/y/z，单位m）
    #     msg.duration = 400 
    #     msg.life_count += 1
    #     Ctrl.Send_cmd(msg)
    #     time.sleep( 1 )
    #     # 7. 行走（ locomotion模式=11，自变频步态，转向0.5rad/s，持续运动5s）
    #     msg.mode = 11 # Locomotion
    #     msg.gait_id = 26 # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
    #     msg.vel_des = [0, 0, 0.5] #转向， 期望速度（x/y/yaw，m/s & rad/s）→ 仅yaw=0.5，原地转向
    #     msg.duration = 0 # Zero duration means continuous motion until a new command is used. # 0表示持续运动，直到新指令覆盖
    #                      # Continuous motion can interrupt non-zero duration interpolation motion
    #     msg.step_height = [0.06, 0.06] # 摆动腿离地高度（前后腿，单位m）
    #     msg.life_count += 1
    #     Ctrl.Send_cmd(msg)
    #     time.sleep( 5 )
    #     # 8. 纯阻尼模式（放松关节，mode=7）
    #     msg.mode = 7    # PureDamper
    #     msg.gait_id = 0
    #     msg.life_count += 1
    #     Ctrl.Send_cmd(msg)
    #     Ctrl.Wait_finish(7, 0)

    # except KeyboardInterrupt: # 捕获Ctrl+C
    #     pass
    while True:
        print("\n" + "=" * 60)
        print("  交互式运动控制菜单")
        print("=" * 60)
        print("R. 恢复站立 (mode=12)")
        print("G. 握手动作 (mode=62)")
        print("K. 双腿站立 (mode=64)")
        print("H. 抬头 (mode=21, rpy=[0,0.3,0])")
        print("N. 低头 (mode=21, rpy=[0,-0.3,0])")
        print("M. 调整身体高度 (mode=21, pos=[0,0,0.22])")
        print("W. 前进(mode=11, yaw=0.5, 5秒)")
        print("S. 后退 (mode=7)")
        print("A. 左转 (1->2->3->4->5->6->7->8)")
        print("D. 右转 (1->2->3->4->5->6->7->8)")
        print("Z. 阻尼模式 (1->2->3->4->5->6->7->8)")
        print("Q. 退出程序")
        print("=" * 60)
        print("当前速度:",Test_var.tem_vel,"m/s,摆腿高度:",Test_var.tem_step_height,"m","xn重心高度:",Test_var.bodyzhong,"m")
        choice = input("请输入选择(按Q退出): ").strip()
        if choice == 'Q':
            print("退出程序")
            Ctrl.quit()
            sys.exit()
        try:
            if choice == 'R':
                # 1. 恢复站立s
                Dog_movements.restand_1(msg,Ctrl)

            elif choice == 'G':
                # 2. 握手动作
                Dog_movements.shake_hand(msg,Ctrl)

            elif choice == 'K':
                # 3. 双腿站立
                Dog_movements.stand(msg,Ctrl)

            elif choice == 'H':
                # 4. 抬头
                Dog_movements.head_up(msg,Ctrl)

            elif choice == 'N':
                # 5. 低头
                Dog_movements.head_down(msg,Ctrl)
            elif choice == 'I':
                Test_var.tem_step_height=float(input("摆腿高度:"))
            elif choice == 'V':
                Test_var.tem_vel=float(input("运动速度:"))
            elif choice == 'Z':
                Test_var.bodyzhong=float(input("重心高度:"))
            elif choice == 'M':
                # 6. 调整身体高度
                Test_var.bodyheight=float(input("身体高度:"))
                Dog_movements.set_bodyheight(msg,Ctrl,Test_var.bodyheight)
            elif choice == 'W':
                Dog_movements.walk(msg,Ctrl,Test_var.tem_vel,1,Test_var.tem_step_height,Test_var.bodyheight)

            elif choice == 'S':
               Dog_movements.walk(msg,Ctrl,Test_var.tem_vel,0,Test_var.tem_step_height,Test_var.bodyheight)

            elif choice == 'A':
                Dog_movements.turn_l(msg,Ctrl,0.3,0)
                time.sleep(7.86)
                Dog_movements.restand_1(msg,Ctrl)
            elif choice == 'D':
                Dog_movements.turn_l(msg,Ctrl,-0.3,0)
                time.sleep(7.86)
                Dog_movements.restand_1(msg,Ctrl)

            else:
                print("无效选择，请重新输入")

        except KeyboardInterrupt:
            print("\n用户中断")
            Ctrl.quit()
            sys.exit()
        except Exception as e:
            print(f"执行出错: {e}")

class Tem_test_var_class:
    def __init__(self):
        self.tem_step_height=0.06
        self.tem_vel=0.6
        self.bodyheight=0.22
        self.bodyzhong=0.26
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
                #print("Heartbeat: mode={}, gait_id={}, life_count={}".format(self.cmd_msg.mode, self.cmd_msg.gait_id, self.cmd_msg.life_count))
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

class Dog_movements:
    @staticmethod
    def restand(msg,ctrl):
        msg.mode = 12 # Recovery stand
        msg.gait_id = 0
        #msg.life_count += 1  # 指令生效条件：life_count更新（防止重复指令）
        ctrl.Send_cmd(msg)    # 发送指令
        ctrl.Wait_finish(12, 0)  # 等待动作执行完成
    @staticmethod
    def restand_1(msg,ctrl):
        msg.mode = 12 # Recovery stand
        msg.gait_id = 0
        #msg.life_count += 1  # 指令生效条件：life_count更新（防止重复指令）
        ctrl.Send_cmd(msg)    # 发送指令
        #ctrl.Wait_finish(12, 0)  # 等待动作执行完成
    @staticmethod
    def shake_hand(msg,ctrl):
        msg.mode = 62 # Shake hand, based on position interpolation control
        msg.gait_id = 2
        #msg.life_count += 1
        ctrl.Send_cmd(msg)
        ctrl.Wait_finish(62, 2)
    @staticmethod
    def stand(msg,ctrl):
        msg.mode = 64 # Twoleg Stand
        msg.gait_id = 0
        #msg.life_count += 1
        ctrl.Send_cmd(msg)
        ctrl.Wait_finish(64, 0)
    @staticmethod
    def head_up(msg,ctrl):
        msg.mode = 21 # Position interpolation control
        msg.gait_id = 0
        msg.rpy_des = [0, 0.3, 0] # Head up 抬头
        msg.duration = 500 # Expected execution time, 0.5s 
        #msg.life_count += 1
        ctrl.Send_cmd(msg)
        time.sleep( 0.5 )
    @staticmethod
    def head_down(msg,ctrl):
        msg.mode = 21 # Position interpolation control
        msg.gait_id = 0
        msg.rpy_des = [0, -0.3, 0] # Head down 低头 
        msg.duration = 300 
        #msg.life_count += 1
        ctrl.Send_cmd(msg)
        time.sleep( 0.3 )
    @staticmethod
    def walk(msg,ctrl,v,head,step_height,zhong):
        msg.mode = 11 # Locomotion
        msg.gait_id = 26 # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
        if head != 1:v=-v
        msg.vel_des = [v, 0, 0] #转向， 期望速度（x/y/yaw，m/s & rad/s）→ 仅yaw=0.5，原地转向
        msg.duration = 0 # Zero duration means continuous motion until a new command is used. # 0表示持续运动，直到新指令覆盖
                         # Continuous motion can interrupt non-zero duration interpolation motion
        msg.step_height = [step_height,step_height] # 摆动腿离地高度（前后腿，单位m）
        msg.pos_des=[0.0, 0.0, zhong]
        #msg.life_count += 1
        ctrl.Send_cmd(msg)
        #time.sleep( 2 )
    @staticmethod
    def turn_l(msg,ctrl,v,t):
        msg.mode = 11 # Locomotion
        msg.gait_id = 26 # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
        msg.vel_des = [0, 0, v] #转向， 期望速度（x/y/yaw，m/s & rad/s）→ 仅yaw=0.5，原地转向
        msg.duration = t # Zero duration means continuous motion until a new command is used. # 0表示持续运动，直到新指令覆盖
                         # Continuous motion can interrupt non-zero duration interpolation motion
        msg.step_height = [0.06, 0.06] # 摆动腿离地高度（前后腿，单位m）
        #msg.life_count += 1
        ctrl.Send_cmd(msg)
        #time.sleep( 2 )
    @staticmethod
    def turn_r(msg,ctrl,v,t):
        msg.mode = 11 # Locomotion
        msg.gait_id = 26 # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
        msg.vel_des = [0, 0, -v] #转向， 期望速度（x/y/yaw，m/s & rad/s）→ 仅yaw=0.5，原地转向
        msg.duration = t # Zero duration means continuous motion until a new command is used. # 0表示持续运动，直到新指令覆盖
                         # Continuous motion can interrupt non-zero duration interpolation motion
        msg.step_height = [0.06, 0.06] # 摆动腿离地高度（前后腿，单位m）
        #msg.life_count += 1
        ctrl.Send_cmd(msg)
        #time.sleep( 2 )
    @staticmethod
    def zuni(msg,ctrl):
        msg.mode = 7    # PureDamper
        msg.gait_id = 0
        #msg.life_count += 1
        ctrl.Send_cmd(msg)
        ctrl.Wait_finish(7, 0)
    @staticmethod
    def set_bodyheight(msg,ctrl,height):
        msg.mode = 21 # Position interpolation control
        msg.gait_id = 5
        msg.rpy_des = [0, 0, 0] #头部复位 
        msg.pos_des = [0, 0, height] # Set body height # 期望身体位置（x/y/z，单位m）
        msg.duration = 400 
        #msg.life_count += 1
        ctrl.Send_cmd(msg)
        #time.sleep( 1 )

# Main function
if __name__ == '__main__':
    main()
