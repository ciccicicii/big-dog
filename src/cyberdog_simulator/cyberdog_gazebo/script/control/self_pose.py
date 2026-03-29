#!/usr/bin/env python3
"""
自定义步态类 - 针对不同场景设置步态参数
"""
import math
from robot_control_cmd_lcmt import robot_control_cmd_lcmt

class CustomGaits:
    """
    自定义步态类，场景配置机器人参数
    """

    def __init__(self):
        # 基础参数
        self.base_params = {
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

    def get_self_gait(self, gait_name, **overrides):
        """
        创建步态消息

        参数:
            gait_name: 步态名称
            overrides: 覆盖默认参数的字典

        返回:
            robot_control_cmd_lcmt 消息对象
        """
        # 复制基础参数
        params = self.base_params.copy()

        # 应用特定步态的默认参数
        if gait_name in self.gait_presets:
            params.update(self.gait_presets[gait_name])

        # 应用自定义覆盖参数
        params.update(overrides)

        # 创建消息对象
        msg = robot_control_cmd_lcmt()
        msg.mode = params['mode']
        msg.gait_id = params['gait_id']
        msg.contact = params['contact']
        msg.life_count = params['life_count']
        msg.vel_des = params['vel_des'].copy()
        msg.rpy_des = params['rpy_des'].copy()
        msg.pos_des = params['pos_des'].copy()
        msg.acc_des = params['acc_des'].copy()
        msg.ctrl_point = params['ctrl_point'].copy()
        msg.foot_pose = params['foot_pose'].copy()
        msg.step_height = params['step_height'].copy()
        msg.value = params['value']
        msg.duration = params['duration']

        return msg

    # 预定义步态参数
    gait_presets = {
        # 1. 石板路 - 稳定、抬腿高、速度慢
        'stone_path': {
            'mode': 11,
            'gait_id': 26,        # 慢速小跑，更稳定
            'step_height': [0.08, 0.15],  # 抬腿高度8cm
            'pos_des': [0.0, 0.0, 0.26],  # 降低重心
            'vel_des': [0.45, 0.0, 0.0],  # 慢速前进
        },

        # 2. 平地速度 - 快速前进
        'flat_speed': {
            'mode': 11,
            'gait_id': 10,        # 快速小跑
            'step_height': [0.06, 0.06],
            'pos_des': [0.0, 0.0, 0.28],
            'vel_des': [0.5, 0.0, 0.0],  # 快速前进
        },

        # 3. 弯道超速 - 转向优化
        'curve_speed': {
            'mode': 11,
            'gait_id': 26,        # 自变频步态，适合转向
            'step_height': [0.06, 0.06],
            'pos_des': [0.0, 0.0, 0.28],
            'vel_des': [0.3, 0.0, 0.8],  # 前进+快速转向
        },

        # 4. 跨越低杆 - 高抬腿
        'jump_low_barrier': {
            'mode': 11,
            'gait_id': 26,
            'step_height': [0.12, 0.12],  # 高抬腿12cm
            'pos_des': [0.0, 0.0, 0.25],  # 降低重心增加稳定
            'vel_des': [0.2, 0.0, 0.0],
        },

        # 5. 上坡路 - 身体前倾
        'uphill': {
            'mode': 11,
            'gait_id': 26,
            'step_height': [0.07, 0.07],  # 稍高抬腿
            'pos_des': [0.0, 0.0, 0.27],  # 稍降低重心
            'rpy_des': [0.0, 0.1, 0.0],   # 身体前倾（pitch正）
            'vel_des': [0.2, 0.0, 0.0],   # 慢速上坡
        },

        # 6. 下坡路 - 身体后倾
        'downhill': {
            'mode': 11,
            'gait_id': 26,
            'step_height': [0.07, 0.07],
            'pos_des': [0.0, 0.0, 0.27],
            'rpy_des': [0.0, -0.1, 0.0],  # 身体后倾（pitch负）
            'vel_des': [0.2, 0.0, 0.0],   # 慢速下坡
        },

        # 7. 左侧斜坡 - 身体右倾
        'side_slope_left': {
            'mode': 11,
            'gait_id': 26,
            'step_height': [0.07, 0.07],
            'pos_des': [0.0, 0.0, 0.27],
            'rpy_des': [0.1, 0.0, 0.0],   # 身体右倾（roll正）
            'vel_des': [0.15, 0.0, 0.0],  # 慢速前进
        },

        # 8. 右侧斜坡 - 身体左倾
        'side_slope_right': {
            'mode': 11,
            'gait_id': 26,
            'step_height': [0.07, 0.07],
            'pos_des': [0.0, 0.0, 0.27],
            'rpy_des': [-0.1, 0.0, 0.0],  # 身体左倾（roll负）
            'vel_des': [0.15, 0.0, 0.0],  # 慢速前进
        },

        # 9. 前后斜坡 - 身体俯仰
        'side_slope_front_back': {
            'mode': 11,
            'gait_id': 26,
            'step_height': [0.07, 0.07],
            'pos_des': [0.0, 0.0, 0.27],
            'rpy_des': [0.0, 0.15, 0.0],  # 身体前倾
            'vel_des': [0.15, 0.0, 0.0],
        },

        # 10. 跳跃台阶 - 高跳准备
        'jump_step': {
            'mode': 22,        # 强制跳跃模式
            'gait_id': 0,
            'step_height': [0.15, 0.15],  # 高跳15cm
            'pos_des': [0.0, 0.0, 0.25],  # 降低重心
            'acc_des': [0.0, 0.0, 5.0, 0.0, 0.0, 0.0],  # 向上加速度
            'duration': 500,   # 0.5秒跳跃
        },

        # 11. 踢瓶子 - 精确控制
        'kick_bottle': {
            'mode': 21,        # 位置插值控制
            'gait_id': 0,
            'step_height': [0.06, 0.06],
            'pos_des': [0.0, 0.0, 0.28],
            'foot_pose': [0.0, 0.0, 0.1, 0.0, 0.0, 0.0],  # 抬起右前腿
            'duration': 300,   # 0.3秒完成踢动作
        },

        # 12. 踢足球 - 力量控制
        'kick_soccer': {
            'mode': 21,        # 位置插值控制
            'gait_id': 0,
            'step_height': [0.08, 0.08],  # 较高抬腿
            'pos_des': [0.0, 0.0, 0.26],  # 降低重心增加力量
            'foot_pose': [0.0, 0.0, 0.15, 0.0, 0.0, 0.0],  # 抬起右前腿更高
            'duration': 400,   # 0.4秒完成踢球动作
        },

        # 13. 站立等待 - 静止姿态
        'stand_wait': {
            'mode': 11,
            'gait_id': 0,      # 站立步态
            'step_height': [0.0, 0.0],
            'pos_des': [0.0, 0.0, 0.28],
            'vel_des': [0.0, 0.0, 0.0],
        },

        # 14. 紧急停止
        'emergency_stop': {
            'mode': 7,         # 纯阻尼模式
            'gait_id': 0,
            'step_height': [0.0, 0.0],
            'vel_des': [0.0, 0.0, 0.0],
        }
    }

    def get_gait_names(self):
        """获取所有可用的步态名称"""
        return list(self.gait_presets.keys())

    def print_gait_info(self, gait_name=None):
        """打印步态信息"""
        if gait_name:
            if gait_name in self.gait_presets:
                print(f"步态: {gait_name}")
                for key, value in self.gait_presets[gait_name].items():
                    print(f"  {key}: {value}")
            else:
                print(f"未找到步态: {gait_name}")
        else:
            print("可用步态:")
            for name in self.gait_presets.keys():
                print(f"  - {name}")


# 使用示例
# if __name__ == '__main__':
#     # 创建自定义步态实例
#     gaits = CustomGaits()

#     # 打印所有可用步态
#     print("自定义步态列表:")
#     gaits.print_gait_info()

#     # 创建特定步态消息
#     print("\n创建石板路步态消息:")
#     stone_msg = gaits.get_self_gait('stone_path')
#     print(f"模式: {stone_msg.mode}, 步态ID: {stone_msg.gait_id}")
#     print(f"抬腿高度: {stone_msg.step_height}")
#     print(f"前进速度: {stone_msg.vel_des[0]}")

#     # 创建跳跃台阶消息
#     print("\n创建跳跃台阶消息:")
#     jump_msg = gaits.get_self_gait('jump_step')
#     print(f"模式: {jump_msg.mode}, 步态ID: {jump_msg.gait_id}")
#     print(f"抬腿高度: {jump_msg.step_height}")
#     print(f"加速度: {jump_msg.acc_des}")

#     # 自定义参数覆盖
#     print("\n创建自定义石板路步态（更慢速度）:")
#     custom_msg = gaits.get_self_gait('stone_path', vel_des=[0.1, 0.0, 0.0])
#     print(f"自定义前进速度: {custom_msg.vel_des[0]}")
# EOF

# chmod +x /home/cyberdog_sim/custom_gaits.py




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