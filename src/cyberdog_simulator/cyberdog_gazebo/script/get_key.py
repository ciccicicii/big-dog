cat > /home/cyberdog_sim/keyboard_interactive.py << 'EOF'
#!/usr/bin/env python3
"""
键盘交互控制脚本
监听键盘输入，修改控制消息，按P键发布到LCM通道
"""
import lcm
import sys
import time
import threading
from robot_control_cmd_lcmt import robot_control_cmd_lcmt

try:
    import msvcrt  # Windows
except ImportError:
    import select  # Linux/Mac

class KeyboardInteractiveController:
    def __init__(self):
        # LCM初始化
        self.lc = lcm.LCM("udpm://239.255.76.67:7671?ttl=255")
        self.channel = "key_control"  # LCM通道名称

        # 控制消息
        self.msg = robot_control_cmd_lcmt()
        self.msg.mode = 11  # 默认运动模式
        self.msg.gait_id = 26  # 自变频步态
        self.msg.vel_des = [0.0, 0.0, 0.0]
        self.msg.step_height = [0.06, 0.06]
        self.msg.duration = 0  # 持续模式
        self.msg.life_count = 0

        # 控制参数
        self.current_speed = 0.2
        self.current_turn = 0.5

        # 运行标志
        self.running = True

        print("=" * 60)
        print("  键盘交互控制脚本")
        print("=" * 60)
        print("按键说明:")
        print("  W - 前进    S - 后退")
        print("  A - 左转    D - 右转")
        print("  Q - 停止    R - 恢复站立")
        print("  E - 阻尼模式(趴下)")
        print("  + - 增加速度  - - 减少速度")
        print("  P - 发布当前指令到LCM通道")
        print("  ESC - 退出程序")
        print("=" * 60)
        print("当前速度: {:.2f} m/s, 转向速度: {:.2f} rad/s".format(
            self.current_speed, self.current_turn))
        print("按任意键开始控制...")
        input()

    def get_key(self):
        """获取按键（跨平台）"""
        # Linux/Mac
        if select.select([sys.stdin], [], [], 0.1)[0]:
            key = sys.stdin.read(1)
            return key
        return None

    def update_message(self):
        """更新消息内容"""
        self.msg.life_count = (self.msg.life_count + 1) % 256
        return self.msg

    def publish_message(self):
        """发布消息到LCM通道"""
        msg = self.update_message()
        try:
            self.lc.publish(self.channel, msg.encode())
            print("✓ 已发布指令到LCM通道 '{}'".format(self.channel))
            print("  mode={}, gait_id={}, vel_des={}".format(
                msg.mode, msg.gait_id, msg.vel_des))
        except Exception as e:
            print("✗ 发布失败: {}".format(e))

    def handle_key(self, key):
        """处理按键"""
        key_lower = key.lower()

        if key_lower == 'w':  # 前进
            self.msg.mode = 11
            self.msg.gait_id = 26
            self.msg.vel_des = [self.current_speed, 0.0, 0.0]
            print("前进: {:.2f} m/s".format(self.current_speed))

        elif key_lower == 's':  # 后退
            self.msg.mode = 11
            self.msg.gait_id = 26
            self.msg.vel_des = [-self.current_speed, 0.0, 0.0]
            print("后退: {:.2f} m/s".format(self.current_speed))

        elif key_lower == 'a':  # 左转
            self.msg.mode = 11
            self.msg.gait_id = 26
            self.msg.vel_des = [0.0, 0.0, self.current_turn]
            print("左转: {:.2f} rad/s".format(self.current_turn))

        elif key_lower == 'd':  # 右转
            self.msg.mode = 11
            self.msg.gait_id = 26
            self.msg.vel_des = [0.0, 0.0, -self.current_turn]
            print("右转: {:.2f} rad/s".format(self.current_turn))

        elif key_lower == 'q':  # 停止
            self.msg.mode = 11
            self.msg.gait_id = 26
            self.msg.vel_des = [0.0, 0.0, 0.0]
            print("停止")

        elif key_lower == 'r':  # 恢复站立
            self.msg.mode = 12
            self.msg.gait_id = 0
            self.msg.vel_des = [0.0, 0.0, 0.0]
            print("恢复站立 (mode=12)")

        elif key_lower == 'e':  # 阻尼模式
            self.msg.mode = 7
            self.msg.gait_id = 0
            self.msg.vel_des = [0.0, 0.0, 0.0]
            print("阻尼模式 (mode=7)")

        elif key == '+':  # 增加速度
            self.current_speed = min(self.current_speed + 0.1, 1.0)
            self.current_turn = min(self.current_turn + 0.1, 1.0)
            print("速度增加: {:.2f} m/s, 转向: {:.2f} rad/s".format(
                self.current_speed, self.current_turn))

        elif key == '-':  # 减少速度
            self.current_speed = max(self.current_speed - 0.1, 0.1)
            self.current_turn = max(self.current_turn - 0.1, 0.1)
            print("速度减少: {:.2f} m/s, 转向: {:.2f} rad/s".format(
                self.current_speed, self.current_turn))

        elif key_lower == 'p':  # 发布指令
            self.publish_message()

        elif key == '\x1b' or key == '\x03':  # ESC或Ctrl+C
            print("退出程序")
            return False

        else:
            print("未知按键: {}".format(key))

        return True
    ef restand(msg,ctrl):
        msg.mode = 12 # Recovery stand
        msg.gait_id = 0
        msg.life_count += 1  # 指令生效条件：life_count更新（防止重复指令）
        ctrl.Send_cmd(msg)    # 发送指令
        ctrl.Wait_finish(12, 0)  # 等待动作执行完成
    def shake_hand(msg,ctrl):
        msg.mode = 62 # Shake hand, based on position interpolation control
        msg.gait_id = 2
        msg.life_count += 1
        ctrl.Send_cmd(msg)
        ctrl.Wait_finish(62, 2)
    def stand(msg,ctrl):
        msg.mode = 64 # Twoleg Stand
        msg.gait_id = 0
        msg.life_count += 1
        Ctrl.Send_cmd(msg)
        Ctrl.Wait_finish(64, 0)
    def head_up(msg,ctrl):
        msg.mode = 21 # Position interpolation control
        msg.gait_id = 0
        msg.rpy_des = [0, 0.3, 0] # Head up 抬头
        msg.duration = 500 # Expected execution time, 0.5s 
        msg.life_count += 1
        Ctrl.Send_cmd(msg)
        time.sleep( 0.5 )
    def head_down(msg,ctrl):
        msg.mode = 21 # Position interpolation control
        msg.gait_id = 0
        msg.rpy_des = [0, -0.3, 0] # Head down 低头 
        msg.duration = 300 
        msg.life_count += 1
        Ctrl.Send_cmd(msg)
        time.sleep( 0.3 )
    def walk(msg,ctrl,v,head):
        msg.mode = 11 # Locomotion
        msg.gait_id = 26 # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
        if head != 1:v=-v
        msg.vel_des = [v, 0, 0] #转向， 期望速度（x/y/yaw，m/s & rad/s）→ 仅yaw=0.5，原地转向
        msg.duration = 0 # Zero duration means continuous motion until a new command is used. # 0表示持续运动，直到新指令覆盖
                         # Continuous motion can interrupt non-zero duration interpolation motion
        msg.step_height = [0.06, 0.06] # 摆动腿离地高度（前后腿，单位m）
        msg.life_count += 1
        Ctrl.Send_cmd(msg)
        time.sleep( 5 )
    def turn_l(msg,ctrl,v,t):
        msg.mode = 11 # Locomotion
        msg.gait_id = 26 # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
        msg.vel_des = [0, 0, v] #转向， 期望速度（x/y/yaw，m/s & rad/s）→ 仅yaw=0.5，原地转向
        msg.duration = t # Zero duration means continuous motion until a new command is used. # 0表示持续运动，直到新指令覆盖
                         # Continuous motion can interrupt non-zero duration interpolation motion
        msg.step_height = [0.06, 0.06] # 摆动腿离地高度（前后腿，单位m）
        msg.life_count += 1
        Ctrl.Send_cmd(msg)
        time.sleep( 5 )
    def turn_r(msg,ctrl,v,t):
        msg.mode = 11 # Locomotion
        msg.gait_id = 26 # TROT_FAST:10 TROT_MEDIUM:3 TROT_SLOW:27 自变频:26
        msg.vel_des = [0, 0, -v] #转向， 期望速度（x/y/yaw，m/s & rad/s）→ 仅yaw=0.5，原地转向
        msg.duration = t # Zero duration means continuous motion until a new command is used. # 0表示持续运动，直到新指令覆盖
                         # Continuous motion can interrupt non-zero duration interpolation motion
        msg.step_height = [0.06, 0.06] # 摆动腿离地高度（前后腿，单位m）
        msg.life_count += 1
        Ctrl.Send_cmd(msg)
        time.sleep( 5 )

    def zuni(msg,ctrl):
        msg.mode = 7    # PureDamper
        msg.gait_id = 0
        msg.life_count += 1
        Ctrl.Send_cmd(msg)
        Ctrl.Wait_finish(7, 0)

    def run(self):
        """主循环"""
        try:
            while self.running:
                key = self.get_key()
                if key:
                    if not self.handle_key(key):
                        break
                time.sleep(0.01)  # 降低CPU占用
        except KeyboardInterrupt:
            print("\n用户中断")
        finally:
            self.running = False
            print("程序已退出")

def main():
    controller = KeyboardInteractiveController()
    controller.run()

if __name__ == '__main__':
    main()
EOF

chmod +x /home/cyberdog_sim/keyboard_interactive.py
