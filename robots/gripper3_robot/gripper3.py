import os
import sys
import time
import numpy as np
from .scservo_sdk import *

class GRIPPER3(object):
    """
    GRIPPER3 controls HL3606-based gripper servos using model inference data (exo angles).
    Interfaces mirror common_robot.AssembledRobot without ROS dependencies.
    """
    def __init__(self, ctrl_freq=100.0, device='/dev/ttyUSB0', baudrate=1000000):
        # Servo configuration
        self.servo_num = 8
        self.servo_ids = list(range(self.servo_num))
        self.ctrl_freq = ctrl_freq
        self.zero_pos = np.array([2048] * self.servo_num)
        self.rot_direction = np.array([-1, -1, 1, -1, 1, -1, -1, -1])

        # Internal state
        self.exo_angles = np.zeros(self.servo_num, dtype=np.int32)
        self.target_pos = self.zero_pos.copy()
        self.target_vel = 32766
        self.target_torque = 1000
        self.target_acc = 0

        # Init HL3606 SDK
        self.portHandler = PortHandler(device)
        self.HLCtrl = HLS(self.portHandler)
        if not self.portHandler.openPort():
            raise IOError(f"Failed to open port {device}")
        if not self.portHandler.setBaudRate(baudrate):
            raise IOError(f"Failed to set baudrate {baudrate}")

    def update_action_angles(self, action_angles):
        print(f"action_angles: {action_angles}")
        print(self.rot_direction * action_angles * 2048 / 3.14)
        
        self.target_pos = action_angles * 2048 / 3.14
        return self.target_pos.astype(np.int32)

    def servo_ctrl_callback(self, action):
        
        # action \in (0, 6.28)

        pos = self.update_action_angles(action)

        print(f"action_2_servopos: {pos}")

        # Group sync write parameters
        for sid in self.servo_ids:
            ok = self.HLCtrl.SyncWritePST(sid, pos[sid], self.target_vel, self.target_torque)
            if ok is not True:
                print(f"[ID:{sid:03d}] SyncWritePST failed")
        # Transmit packet
        result = self.HLCtrl.GrSyncWr_APST.txPacket()
        if result != COMM_SUCCESS:
            print(self.HLCtrl.packetHandler.getTxRxResult(result))
        self.HLCtrl.GrSyncWr_APST.clearParam()

    # Interface methods mirroring common_robot
    def get_current_joint_positions(self):
         # add all id to the group sync read
        for id in self.servo_ids:
            success_added= self.HLCtrl.GrSyncRd_Pos.addParam(id)
            if success_added != True:
                print("[ID:%03d] GrSyncReadPos addparam failed" % id)

        # Send and Receive GroupSyncRead Package
        comm_result = self.HLCtrl.GrSyncRd_Pos.txRxPacket()
        if comm_result != COMM_SUCCESS:
            print("%s" % self.HLCtrl.getTxRxResult(comm_result))

        # Parse received package to get pos
        pos=[]
        for id in self.servo_ids:
            data, error = self.HLCtrl.GrSyncRd_Pos.isAvailable(id, HLS_PRESENT_POSITION_L, 2)
            if data:
                position = self.HLCtrl.GrSyncRd_Pos.getData(id, HLS_PRESENT_POSITION_L, 2)
                pos.append(position)
            else:
                print("[ID:%03d] GrSyncReadPos getdata failed" % id)
                continue
            if error != 0:
                print("%s" % self.HLCtrl.getRxPacketError(error))
        
        print(f"Current 8 Servo pos: {pos}")
        return pos



    def set_joint_position_target(self, qpos, qvel=None, blocking=False):
        """
        Accept target positions list (length servo_num) and send commands.
        Ignores qvel/blocking, uses internal velocity/torque.
        """
        self.servo_ctrl_callback(action=np.array(qpos, dtype=np.int32))


    def go_zero(self):

        for id in range(self.servo_num):
            self.HLCtrl.SyncWritePST(id, position = self.zero_pos[id], speed=32766, torque=1000)
        
        # send the group sync write
        _ = self.HLCtrl.GrSyncWr_APST.txPacket()
        self.HLCtrl.GrSyncWr_APST.clearParam()