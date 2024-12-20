# this file is legacy, need to fix.
from unitree_dds_wrapper.idl import unitree_go
from unitree_dds_wrapper.publisher import Publisher
from unitree_dds_wrapper.subscription import Subscription
# from avp_teleoperate.teleop.open_television.constants import inspire_tip_indices
from teleop.robot_control.hand_retargeting import HandRetargeting, HandType
import numpy as np
import threading
import time

inspire_tip_indices = [4, 9, 14, 19, 24]

class InspireController:
    def __init__(self):
        self.cmd = unitree_go.msg.dds_.MotorCmds_()
        self.state = unitree_go.msg.dds_.MotorStates_()
        self.lock = threading.Lock()
        self.handcmd = Publisher(unitree_go.msg.dds_.MotorCmds_, "rt/inspire/cmd")
        self.handstate = Subscription(unitree_go.msg.dds_.MotorStates_, "rt/inspire/state")
        self.cmd.cmds = [unitree_go.msg.dds_.MotorCmd_() for _ in range(12)]
        self.state.states = [unitree_go.msg.dds_.MotorState_() for _ in range(12)]

        self.subscribe_state_thread = threading.Thread(target=self.subscribe_state)
        self.subscribe_state_thread.daemon = True
        self.subscribe_state_thread.start()

        self.hand_retargeting = HandRetargeting(HandType.INSPIRE_HAND)

    def subscribe_state(self):
        while True:
            if self.handstate.msg:
                self.state = self.handstate.msg
                # right_angles = tuple(self.state.states[i].q for i in range(6))
                # print(f"right_angles: {right_angles}")
            time.sleep(0.01)

    def ctrl(self, left_angles, right_angles):
        for i in range(6):
            self.cmd.cmds[i].q = right_angles[i]
            self.cmd.cmds[i+6].q = left_angles[i]
        self.handcmd.msg.cmds = self.cmd.cmds
        self.handcmd.write()

    def get_current_dual_hand_q(self):
        with self.lock:
            q = np.array([self.state.states[i].q for i in range(12)])
            return q
        
    def get_right_q(self):
        with self.lock:
            q = np.array([self.state.states[i].q for i in range(6)])
            return q

    def get_left_q(self):
        with self.lock:
            q = np.array([self.state.states[i+6].q for i in range(6)])
            return q

    def process_hand_data(self, left_hand_array, right_hand_array):
        '''
        left_hand_array: [input] Left hand skeleton data (required from XR device) to control_thread

        right_hand_array: [input] Right hand skeleton data (required from XR device) to control_thread
        '''

        # print(f"right_hand_mat: {right_hand_array}")

        ref_left_value = left_hand_array[inspire_tip_indices]
        ref_right_value = right_hand_array[inspire_tip_indices]

        print(f"ref_right_value: {ref_right_value}")

        left_q_target  = self.hand_retargeting.left_retargeting.retarget(ref_left_value)
        right_q_target = self.hand_retargeting.right_retargeting.retarget(ref_right_value)

        print(f"right_q_target: {right_q_target}")

        left_angles = self.get_hand_angles(left_q_target)
        right_angles = self.get_hand_angles(right_q_target)

        print(f"right_angles: {right_angles}")

        self.ctrl(left_angles, right_angles)

    def get_hand_angles(self, hand_pos):
        '''
        hand_pos: [input] Hand position retargeted from XR device, order is         
        [thumb] (0-3)
        [index] (4-5) 
        [middle] (6-7)
        [ring] (8-9)
        [pinky] (10-11)

        return: [output] Hand angles to control the robot, order is 
        [pinky, ring, middle, index, thumb_bend, thumb_rotation]
        '''
        finger_angles = np.zeros(6)
        
        # 普通手指（食指、中指、无名指、小指）4 - 11
        # 它们都有相同的限制：0 到 1.7
        for i in range(4):
            joint_pos = hand_pos[10 - i*2]  # 从小指开始,每次减2获取proximal关节位置
            finger_angles[i] = 1 - (joint_pos / 1.7)  # 归一化到0-1
    
        # 拇指比较特殊，有yaw和pitch两个主动关节
        # yaw: -0.1 到 1.3
        # pitch: 0.0 到 0.5
        thumb_yaw = 1 - ((hand_pos[0] + 0.1) / 1.4) # 归一化yaw
        thumb_pitch = 1 - (hand_pos[1] / 0.5)  # 归一化pitch
        # 使用yaw和pitch的组合来表示拇指的弯曲程度
        finger_angles[4] = thumb_pitch  # thumb bend
        finger_angles[5] = thumb_yaw    # thumb rotation
        
        # 确保所有值都在0-1范围内
        finger_angles = np.clip(finger_angles, 0, 1)
        
        return finger_angles