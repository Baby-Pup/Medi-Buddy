#!/usr/bin/env python3
# encoding: utf-8
import os
import math
import rclpy
from enum import Enum
from rclpy.node import Node
from sdk.common import val_map
from std_srvs.srv import Trigger
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from ros_robot_controller_msgs.msg import BuzzerState
from ros_robot_controller_msgs.msg import BuzzerState, SetPWMServoState, PWMServoState


AXES_MAP = 'lx', 'ly', 'rx', 'ry', 'r2', 'l2', 'hat_x', 'hat_y'
BUTTON_MAP = 'cross', 'circle', '', 'square', 'triangle', '', 'l1', 'r1', 'l2', 'r2', 'select', 'start', '', 'l3', 'r3', '', 'hat_xl', 'hat_xr', 'hat_yu', 'hat_yd', ''

class ButtonState(Enum):
    Normal = 0
    Pressed = 1
    Holding = 2
    Released = 3

class JoystickController(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)

        self.min_value = 0.1
        self.declare_parameter('max_linear', 0.01)
        self.declare_parameter('max_angular', 0.5)
        self.declare_parameter('disable_servo_control', True)

        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value
        self.disable_servo_control = self.get_parameter('disable_servo_control').value
        self.machine = os.environ['MACHINE_TYPE']
        self.get_logger().info('\033[1;32m%s\033[0m' % self.max_linear)
        self.servo_state_pub = self.create_publisher(SetPWMServoState, 'ros_robot_controller/pwm_servo/set_state', 1)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 1)
        self.buzzer_pub = self.create_publisher(BuzzerState, 'ros_robot_controller/set_buzzer', 1)
        self.mecanum_pub = self.create_publisher(Twist, 'controller/cmd_vel', 1)

        self.last_axes = dict(zip(AXES_MAP, [0.0, ] * len(AXES_MAP)))
        self.last_buttons = dict(zip(BUTTON_MAP, [0.0, ] * len(BUTTON_MAP)))
        self.mode = 0
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def get_node_state(self, request, response):
        response.success = True
        return response

    def axes_callback(self, axes):
            twist = Twist()
            # lx, ly, rx의 Dead Zone 처리 (기존 로직 유지)
            if abs(axes['lx']) < self.min_value:
                axes['lx'] = 0
            if abs(axes['ly']) < self.min_value:
                axes['ly'] = 0
            if abs(axes['rx']) < self.min_value:
                axes['rx'] = 0
            if abs(axes['ry']) < self.min_value:
                axes['ry'] = 0
                
            ly = axes['ly']
            rx = axes['rx']
            hat_y = axes['hat_y'] # 십자키 상하 입력

            # 🚀 선속도 결정 로직 (조작 시 0.1, 정지 시 0.0)
            # ly (스틱) 또는 hat_y (십자키) 중 하나라도 움직임이 감지되면 속도를 0.1로 고정
            if abs(ly) > 0 or abs(hat_y) > 0:
                # ➡️ 입력이 감지됨: 선속도를 0.1로 고정
                
                # 전진/후진 방향을 결정합니다.
                # 스틱(ly)이 -1(후진) 또는 십자키(hat_y)가 -1(아래)일 경우 후진(-0.1)
                # 스틱 입력이 우선하고, 둘 다 0일 때만 십자키를 확인합니다.
                
                # 최종 입력 방향 확인 (ly가 Dead Zone 밖이면 ly 사용, 아니면 hat_y 사용)
                direction_input = ly if abs(ly) > self.min_value else hat_y
                
                # direction_input이 양수(전진)면 +0.1, 음수(후진)면 -0.1
                if direction_input > 0:
                    twist.linear.x = 0.1 # 전진
                elif direction_input < 0:
                    twist.linear.x = -0.1 # 후진
                else:
                    # Dead Zone 밖이지만 방향이 0인 경우는 거의 없으나, 안전을 위해 0.0
                    twist.linear.x = 0.0
                    
            else:
                # 🛑 입력이 감지되지 않음: 선속도를 0.0으로 설정
                twist.linear.x = 0.0


            # --- MentorPi_Mecanum 유형 처리 ---
            if self.machine == 'MentorPi_Mecanum':
                # Y축 선속도는 0으로 유지 (이전 요청 반영)
                twist.linear.y = 0.0
                
                # Z축 각속도 (회전)는 스틱 입력(rx)에 따라 비례 제어 유지
                twist.angular.z = val_map(rx, -1, 1, -self.max_angular, self.max_angular)
            
            # --- JetRover_Tank 유형 처리 ---
            elif self.machine == 'JetRover_Tank':
                # X축 선속도는 위에서 이미 twist.linear.x로 설정됨
                
                # Z축 각속도 (회전)는 스틱 입력(rx)에 따라 비례 제어 유지
                twist.angular.z = val_map(rx, -1, 1, -self.max_angular, self.max_angular)
            
            # --- MentorPi_Acker 유형 처리 ---
            elif self.machine == 'MentorPi_Acker':
                # X축 선속도는 이미 twist.linear.x로 설정됨 (0.1 또는 -0.1 또는 0.0)
                
                # 조향 로직은 유지하되, twist.linear.x가 0.1로 고정될 수 있음을 인지
                steering_angle = val_map(rx, -1, 1, -math.radians(322 / 2000 * 180), math.radians(322 / 2000 * 180))
                
                if steering_angle == 0:  
                    twist.angular.z = 0.0
                    # ... (servo_state publish 로직 유지) ...
                else:
                    R = 0.145 / math.tan(steering_angle)
                    # twist.angular.z 계산 시 선속도(twist.linear.x)를 사용
                    # 선속도가 0일 경우 angular.z도 0이 됨 (정상적인 동작)
                    twist.angular.z = float(twist.linear.x / R)  

                    # ... (servo_state publish 로직 유지) ...
                
            self.mecanum_pub.publish(twist)



    def select_callback(self, new_state):
        pass

    def l1_callback(self, new_state):
        pass

    def l2_callback(self, new_state):
        pass

    def r1_callback(self, new_state):
        pass

    def r2_callback(self, new_state):
        pass

    def square_callback(self, new_state):
        pass

    def cross_callback(self, new_state):
        pass

    def circle_callback(self, new_state):
        pass

    def triangle_callback(self, new_state):
        pass

    def start_callback(self, new_state):
        if new_state == ButtonState.Pressed:
            msg = BuzzerState()
            msg.freq = 2500
            msg.on_time = 0.05
            msg.off_time = 0.01
            msg.repeat = 1
            self.buzzer_pub.publish(msg)

    def hat_xl_callback(self, new_state):
        pass

    def hat_xr_callback(self, new_state):
        pass

    def hat_yd_callback(self, new_state):
        pass

    def hat_yu_callback(self, new_state):
        pass

    def joy_callback(self, joy_msg):
        axes = dict(zip(AXES_MAP, joy_msg.axes))
        axes_changed = False
        hat_x, hat_y = axes['hat_x'], axes['hat_y']
        hat_xl, hat_xr = 1 if hat_x > 0.5 else 0, 1 if hat_x < -0.5 else 0
        hat_yu, hat_yd = 1 if hat_y > 0.5 else 0, 1 if hat_y < -0.5 else 0
        buttons = list(joy_msg.buttons)
        buttons.extend([hat_xl, hat_xr, hat_yu, hat_yd, 0])
        buttons = dict(zip(BUTTON_MAP, buttons))
        for key, value in axes.items(): 
            if self.last_axes[key] != value:
                axes_changed = True
        if axes_changed:
            try:
                self.axes_callback(axes)
            except Exception as e:
                self.get_logger().error(str(e))
        for key, value in buttons.items():
            if value != self.last_buttons[key]:
                new_state = ButtonState.Pressed if value > 0 else ButtonState.Released
            else:
                new_state = ButtonState.Holding if value > 0 else ButtonState.Normal
            callback = "".join([key, '_callback'])
            if new_state != ButtonState.Normal:
                self.get_logger().info(str(new_state))
                if  hasattr(self, callback):
                    try:
                        getattr(self, callback)(new_state)
                    except Exception as e:
                        self.get_logger().error(str(e))
        self.last_buttons = buttons
        self.last_axes = axes

def main():
    node = JoystickController('joystick_control')
    rclpy.spin(node)  

if __name__ == "__main__":
    main()


