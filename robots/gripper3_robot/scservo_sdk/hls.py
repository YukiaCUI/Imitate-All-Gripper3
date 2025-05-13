#!/usr/bin/env python

from .scservo_def import *
from .protocol_packet_handler import *
from .group_sync_read import *
from .group_sync_write import *

#波特率定义
HLS_1M = 0
HLS_0_5M = 1
HLS_250K = 2
HLS_128K = 3
HLS_115200 = 4
HLS_76800 = 5
HLS_57600 = 6
HLS_38400 = 7

#内存表定义
#-------EPROM(只读)--------
HLS_MODEL_L = 3
HLS_MODEL_H = 4

#-------EPROM(读写)--------
HLS_ID = 5
HLS_BAUD_RATE = 6
HLS_MIN_ANGLE_LIMIT_L = 9
HLS_MIN_ANGLE_LIMIT_H = 10
HLS_MAX_ANGLE_LIMIT_L = 11
HLS_MAX_ANGLE_LIMIT_H = 12
HLS_CW_DEAD = 26
HLS_CCW_DEAD = 27
HLS_OFS_L = 31
HLS_OFS_H = 32
HLS_MODE = 33

#-------SRAM(读写)--------
HLS_TORQUE_ENABLE = 40
HLS_ACC = 41
HLS_GOAL_POSITION_L = 42
HLS_GOAL_POSITION_H = 43
HLS_GOAL_TORQUE_L = 44     #Done 
HLS_GOAL_TORQUE_H = 45     #Done 
HLS_GOAL_SPEED_L = 46
HLS_GOAL_SPEED_H = 47
HLS_LOCK = 55

#-------SRAM(只读)--------
HLS_PRESENT_POSITION_L = 56
HLS_PRESENT_POSITION_H = 57
HLS_PRESENT_SPEED_L = 58
HLS_PRESENT_SPEED_H = 59
HLS_PRESENT_LOAD_L = 60
HLS_PRESENT_LOAD_H = 61
HLS_PRESENT_VOLTAGE = 62
HLS_PRESENT_TEMPERATURE = 63
HLS_MOVING = 66
HLS_PRESENT_CURRENT_L = 69
HLS_PRESENT_CURRENT_H = 70

class HLS(protocol_packet_handler):
    def __init__(self, portHandler):
        protocol_packet_handler.__init__(self, portHandler, 0)
        self.GrSyncWr_APST = GroupSyncWrite(self, HLS_ACC, 7) 
        self.GrSyncWr_Set2048 = GroupSyncWrite(self, HLS_TORQUE_ENABLE, 1) 
        self.GrSyncRd_Pos = GroupSyncRead(self, HLS_PRESENT_POSITION_L, 2)



    def WritePosEx(self, scs_id, position, speed, acc):
        txpacket = [acc, self.scs_lobyte(position), self.scs_hibyte(position), 0, 0, self.scs_lobyte(speed), self.scs_hibyte(speed)]
        return self.writeTxRx(scs_id, HLS_ACC, len(txpacket), txpacket)

    def ReadPos(self, scs_id):
        scs_present_position, scs_comm_result, scs_error = self.read2ByteTxRx(scs_id, HLS_PRESENT_POSITION_L)
        return self.scs_tohost(scs_present_position, 15), scs_comm_result, scs_error

    def ReadSpeed(self, scs_id):
        scs_present_speed, scs_comm_result, scs_error = self.read2ByteTxRx(scs_id, HLS_PRESENT_SPEED_L)
        return self.scs_tohost(scs_present_speed, 15), scs_comm_result, scs_error

    def ReadPosSpeed(self, scs_id):
        scs_present_position_speed, scs_comm_result, scs_error = self.read4ByteTxRx(scs_id, HLS_PRESENT_POSITION_L)
        scs_present_position = self.scs_loword(scs_present_position_speed)
        scs_present_speed = self.scs_hiword(scs_present_position_speed)
        return self.scs_tohost(scs_present_position, 15), self.scs_tohost(scs_present_speed, 15), scs_comm_result, scs_error

    def ReadMoving(self, scs_id):
        moving, scs_comm_result, scs_error = self.read1ByteTxRx(scs_id, HLS_MOVING)
        return moving, scs_comm_result, scs_error

    def SyncWritePosEx(self, scs_id, position, speed, acc):
        txpacket = [acc, self.scs_lobyte(position), self.scs_hibyte(position), 0, 0, self.scs_lobyte(speed), self.scs_hibyte(speed)]
        return self.GrSyncWr_APST.addParam(scs_id, txpacket)

    def RegWritePosEx(self, scs_id, position, speed, acc):
        txpacket = [acc, self.scs_lobyte(position), self.scs_hibyte(position), 0, 0, self.scs_lobyte(speed), self.scs_hibyte(speed)]
        return self.regWriteTxRx(scs_id, HLS_ACC, len(txpacket), txpacket)

    def SyncReadPos(self, scs_id):
        return self.GrSyncRd.addParam(scs_id, HLS_PRESENT_POSITION_L, 2)
    
    def RegAction(self):
        return self.action(BROADCAST_ID)

    def WheelMode(self, scs_id):
        return self.write1ByteTxRx(scs_id, HLS_MODE, 1)

    def WriteSpec(self, scs_id, speed, acc):
        speed = self.scs_toscs(speed, 15)
        txpacket = [acc, 0, 0, 0, 0, self.scs_lobyte(speed), self.scs_hibyte(speed)]
        return self.writeTxRx(scs_id, HLS_ACC, len(txpacket), txpacket)

    def LockEprom(self, scs_id):
        return self.write1ByteTxRx(scs_id, HLS_LOCK, 1)

    def unLockEprom(self, scs_id):
        return self.write1ByteTxRx(scs_id, HLS_LOCK, 0)
    

    # --------------------Customized Function---------------------------
    
    def SyncWritePST(self, scs_id, position, speed, torque, acc=0):
        txpacket = [acc, self.scs_lobyte(position), self.scs_hibyte(position), self.scs_lobyte(torque), self.scs_hibyte(torque), self.scs_lobyte(speed), self.scs_hibyte(speed)]
        return self.GrSyncWr_APST.addParam(scs_id, txpacket)
    
    def SyncWriteAPST(self, scs_id, position, speed, torque, acc):
        txpacket = [acc, self.scs_lobyte(position), self.scs_hibyte(position), self.scs_lobyte(torque), self.scs_hibyte(torque), self.scs_lobyte(speed), self.scs_hibyte(speed)]
        return self.GrSyncWr_APST.addParam(scs_id, txpacket)

    def SyncSet2048(self, id_list=[1]):
        # Add all id to the group sync write
        for id in id_list:
            if self.GrSyncWr_Set2048.addParam(id, [128]):
                print(f"Successfully add ID:{id} to 2048")
            else:
                print(f"Failed to add ID:{id} to 2048")
        
        # Send the group sync write
        scs_comm_result = self.GrSyncWr_Set2048.txPacket()
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % self.getTxRxResult(scs_comm_result))

        # Clear syncwrite parameter storage
        self.GrSyncWr_Set2048.clearParam()
        print(f"Successfully SyncSet2048 to Servo ID {id_list}")

    #-----------------应该挪到上层G3-----------------
    def Go_zero(self, id_list=list(range(8)), zero_pos=[2048]*8):
        # add all id to the group sync write
        for id in id_list:
            self.SyncWritePST(id, position=zero_pos[id], speed=32766, torque=1000)
        
        # send the group sync write
        _ = self.GrSyncWr_APST.txPacket()
        self.GrSyncWr_APST.clearParam()

    def Go_home(self, id_list=list(range(8)), home_pos=[2048]*8):
        # add all id to the group sync write
        for id in id_list:
            self.SyncWritePST(id, position=home_pos[id], speed=32766, torque=1000)

        # send the group sync write
        _ = self.GrSyncWr_APST.txPacket()
        self.GrSyncWr_APST.clearParam()

    # def SyncCal2048(self, id_list=list(range(8))):
    #     # add all id to the group sync write



    



    

