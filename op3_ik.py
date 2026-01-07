import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time

class RvizWalkNode(Node):
    def __init__(self):
        super().__init__('rviz_walk_node')
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.l = 0.110

        self.initialZPos = 0.02
        self.z = 0.025    # 抬腳高度
        self.step_x = 0.030     # 跨步長度
        self.swing_y = 0.015   
        self.interp_steps = 10
        
        # l_el left hand
        self.joint_names = [
            'l_el', 'r_el', 'l_sho_pitch', 'r_sho_pitch',
            'l_sho_roll', 'r_sho_roll',
            'l_hip_yaw', 'l_hip_roll', 'l_hip_pitch', 'l_knee', 'l_ank_pitch', 'l_ank_roll',
            'r_hip_yaw', 'r_hip_roll', 'r_hip_pitch', 'r_knee', 'r_ank_pitch', 'r_ank_roll'
        ]
        
        self.l_el = -0.96
        self.r_el = 0.96
        self.l_sho_roll = 0.87
        self.r_sho_roll = -0.87

        self.current_angles = {name: 0.0 for name in self.joint_names}
        
        self.arm_swing_gain = 10
        self.create_timer(0.5, self.walking_sequence)

    def calculate_ik(self, x, y, z):
        l = self.l
        
        theta_hr = math.atan2(y, (2 * l - z))
        dist_xz = math.sqrt((2 * l - z)**2 + x**2)
        
        ratio = dist_xz / (2 * l)
        
        theta_hp2 = math.acos(ratio)
        theta_hp1 = math.atan2(x, (2 * l - z))
        
        theta_hp = theta_hp1 + theta_hp2  
        theta_kp = 2 * theta_hp2          
        theta_ap = theta_hp2 - (theta_kp - theta_hp1)
        theta_ar = theta_hr           
        
        return theta_hr, theta_hp, theta_kp, theta_ap, theta_ar

    def calculate_arm_swing(self):
        l_arm_pitch = 0.020 * self.arm_swing_gain
        r_arm_pitch = 0.020 * self.arm_swing_gain
        
        return l_arm_pitch, -r_arm_pitch

    def move_to_pos_step(self, xl, yl, zl, xr, yr, zr):
        rl_l, hp_l, kn_l, ap_l, ar_l = self.calculate_ik(xl, yl, zl)
        rl_r, hp_r, kn_r, ap_r, ar_r = self.calculate_ik(xr, yr, zr)
        
        rl_init, hp_init, kn_init, ap_init, ar_init = self.calculate_ik(0, 0, self.initialZPos)

        l_arm, r_arm = self.calculate_arm_swing()

        target = {}
        stance = 0.0 

        if yr > 0: 
            if zr == self.initialZPos:
                # 兩腳在地且重心偏移，左腳參考反向位移
                rl_inv, hp_inv, kn_inv, ap_inv, ar_inv = self.calculate_ik(-xr, 0, self.initialZPos)
                target['l_hip_roll']  = ar_inv
                target['l_hip_pitch'] = -hp_inv
                target['l_knee']      = kn_inv
                target['l_ank_pitch'] = -ap_inv
                target['l_ank_roll']  = ar_inv
                
                target['r_hip_roll']  = ar_r
                target['r_hip_pitch'] = hp_r
                target['r_knee']      = -kn_r
                target['r_ank_pitch'] = ap_r
                target['r_ank_roll']  = ar_r

                target['l_sho_pitch'] = 0
                target['r_sho_pitch'] = 0
            else:
                # 右腳移動中 (抬腳)
                target['l_hip_roll']  = ar_l
                target['l_hip_pitch'] = -hp_init
                target['l_knee']      = kn_init
                target['l_ank_pitch'] = -ap_init
                target['l_ank_roll']  = ar_l + stance
                
                target['r_hip_roll']  = ar_r
                target['r_hip_pitch'] = hp_r
                target['r_knee']      = -kn_r
                target['r_ank_pitch'] = ap_r
                target['r_ank_roll']  = ar_r - stance

                target['l_sho_pitch'] = -l_arm
                target['r_sho_pitch'] = r_arm
        elif yr < 0:
            if zl == self.initialZPos:
                # 兩腳在地且重心偏移，右腳參考反向位移
                rl_inv, hp_inv, kn_inv, ap_inv, ar_inv = self.calculate_ik(-xl, 0, self.initialZPos)
                target['l_hip_roll']  = ar_l
                target['l_hip_pitch'] = -hp_l
                target['l_knee']      = kn_l
                target['l_ank_pitch'] = -ap_l
                target['l_ank_roll']  = ar_l
                
                target['r_hip_roll']  = ar_inv
                target['r_hip_pitch'] = hp_inv
                target['r_knee']      = -kn_inv
                target['r_ank_pitch'] = ap_inv
                target['r_ank_roll']  = ar_inv

                target['l_sho_pitch'] = 0
                target['r_sho_pitch'] = 0
            else:
                # 左腳移動中 (抬腳)
                target['l_hip_roll']  = ar_l
                target['l_hip_pitch'] = -hp_l
                target['l_knee']      = kn_l
                target['l_ank_pitch'] = -ap_l
                target['l_ank_roll']  = ar_l + stance
                
                target['r_hip_roll']  = ar_r
                target['r_hip_pitch'] = hp_init
                target['r_knee']      = -kn_init
                target['r_ank_pitch'] = ap_init
                target['r_ank_roll']  = ar_r - stance

                target['l_sho_pitch'] = l_arm
                target['r_sho_pitch'] = -r_arm
        else: # yr == 0
            # 回到初始姿態
            target['l_hip_roll']  = ar_l
            target['l_hip_pitch'] = -hp_l
            target['l_knee']      = kn_l
            target['l_ank_pitch'] = -ap_l
            target['l_ank_roll']  = ar_l
            
            target['r_hip_roll']  = ar_r
            target['r_hip_pitch'] = hp_r
            target['r_knee']      = -kn_r
            target['r_ank_pitch'] = ap_r
            target['r_ank_roll']  = ar_r

            #target['l_sho_pitch'] = -l_arm
            #target['r_sho_pitch'] = 0.0

        target['l_hip_yaw'] = 0.0
        target['r_hip_yaw'] = 0.0

        target['l_el'] = self.l_el
        target['r_el'] = self.r_el

        target['l_sho_roll'] = self.l_sho_roll
        target['r_sho_roll'] = self.r_sho_roll

        #target['l_sho_pitch'] = l_arm
        #target['r_sho_pitch'] = r_arm
        start_angles = self.current_angles.copy()
        
        for i in range(self.interp_steps):
            t = 0.5 * (1 - math.cos(math.pi * i / (self.interp_steps - 1)))
            
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = self.joint_names
            positions = []
            
            for name in self.joint_names:
                start = start_angles.get(name, 0.0)
                end = target.get(name, 0.0)
                current = start + (end - start) * t
                self.current_angles[name] = current
                positions.append(current)
            
            msg.position = positions
            self.joint_pub.publish(msg)
            time.sleep(0.01)

    def walking_sequence(self):
        y = self.swing_y
        z = self.initialZPos
        lift_z = z + self.z 
        step_x = self.step_x
        
        self.get_logger().info("Moving Right Leg...")
        self.move_to_pos_step(0, y, z, 0, y, lift_z)
        self.move_to_pos_step(0, y, z, step_x, y, z)
        
        self.move_to_pos_step(0, 0, z, step_x, 0, z)
        
        self.get_logger().info("Moving Left Leg...")
        self.move_to_pos_step(0, -y, lift_z, step_x, -y, z) 
        self.move_to_pos_step(step_x, -y, z, step_x, -y, z)
        
        self.move_to_pos_step(step_x, 0, z, step_x, 0, z)

def main():
    rclpy.init()
    node = RvizWalkNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()