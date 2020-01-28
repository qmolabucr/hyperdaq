'''
thordevices.py

Controlls devices from Thorlabs using the Thorlabs.MotionControl.C_API DLLs that go with
the Thorlabs kinesis software.

Last updated: January 2020 by Trevor Arp

Gabor Lab
University of California, Riverside
All Rights Reserved
'''
import parameters as pm

import ctypes
from ctypes import byref
from os.path import join
import numpy as np
import time

from traceback import format_exc

int32 = ctypes.c_long
uInt32 = ctypes.c_ulong
uInt64 = ctypes.c_ulonglong
float64 = ctypes.c_double

motor_stop_bits = -2147470336
rotation_stop_bits = -2147482624
Nanorotator_stop_bits = -2146429952

DLL_device = "Thorlabs.MotionControl.DeviceManager.dll"
DLL_delay = "Thorlabs.MotionControl.Benchtop.BrushlessMotor.dll"
DLL_rotation = "Thorlabs.MotionControl.IntegratedStepperMotors.dll"
DLL_BSC = "Thorlabs.MotionControl.Benchtop.StepperMotor.dll"

class DelayController():
    '''
    A class for controlling a Thorlabs Delay Stage such as a DDS220, but more genercally a
    "Benchtop Burshless Motor"
    '''
    def __init__(self, serialNum, autohome=True):
        s = str(serialNum)
        self.serial = byref(ctypes.create_string_buffer(s.encode('utf-8')))
        self.channel = ctypes.c_short(1)
        self.pollingrate = ctypes.c_int(100)
        self.device_to_mm = pm.DELAY_step_to_mm
        self.velocity_to_mm = pm.DELAY_vel_to_mm
        self.position = 0.0

        # Load the DLLs
        self.device_manager = ctypes.CDLL(join(pm.DLL_directory, DLL_device)) #Here to make subsequent modules load
        self.dll = ctypes.CDLL(join(pm.DLL_directory, DLL_delay))

        # Initialize the device
        self.dll.BMC_Open(self.serial)
        self.dll.BMC_EnableChannel(self.serial, self.channel)
        self.dll.BMC_StartPolling(self.serial, self.channel, self.pollingrate)
        time.sleep(0.1)

        self.minPos = 1.0*self.GetStageAxisInfo_MinPos()/self.device_to_mm
        self.maxPos = 1.0*self.GetStageAxisInfo_MaxPos()/self.device_to_mm

        # Home the device
        if bool(self.dll.BMC_NeedsHoming(self.serial, self.channel)) and autohome:
            self.dll.BMC_Home(self.serial, self.channel)
            time.sleep(0.25)
            self.wait_till_stopped()

        #Get Velocity Parameters
        self.accel = ctypes.c_int(int(pm.DELAY_default_accel))
        self.velocity = ctypes.c_int(int(100.0*self.velocity_to_mm))
        self.dll.BMC_SetVelParams(self.serial, self.channel, self.velocity, self.accel)

        self.GetPosition()
        self.MoveMM(100.0)
    #

    def wait_till_stopped(self):
        loopcnt = 0
        time.sleep(0.1)
        while self.dll.BMC_GetStatusBits(self.serial, self.channel) != motor_stop_bits:
            if loopcnt > 600:
                print("Error throdevices.DelayController.wait_till_stopped: Timed out after 60 seconds")
                break
            time.sleep(0.1)
            loopcnt += 1
        #
    #

    def GetPosition(self):
        time.sleep(0.05)
        pos = self.dll.BMC_GetPosition(self.serial, self.channel)
        self.position = float(1.0*pos/self.device_to_mm)
        return self.position
    #

    def MoveMM(self, position, wait_forever=False):
        if position >= self.minPos and position <= self.maxPos:
            loopcnt = 0
            pos = int(position*self.device_to_mm)
            self.dll.BMC_SetVelParams(self.serial, self.channel, self.velocity, self.accel)
            self.dll.BMC_MoveToPosition(self.serial, self.channel, pos)
            while self.dll.BMC_GetPosition(self.serial, self.channel) != pos:
                if loopcnt > 600 and not wait_forever:
                    print("Error throdevices.DelayController.wait_till_stopped: Timed out after 30 seconds")
                    break
                time.sleep(0.1)
                loopcnt += 1
            self.position = float(1.0*pos/self.device_to_mm)
        else:
            print("Error thordevices.MoveMM: Cannot Move to given position")
    #

    def MoveToIn(self, position, time):
        if position >= self.minPos and position <= self.maxPos:
            vel = np.abs(position - self.position)/time
            vel_c = ctypes.c_int(int(vel*self.velocity_to_mm))
            pos = int(position*self.device_to_mm)
            self.dll.BMC_SetVelParams(self.serial, self.channel, vel_c, self.accel)
            self.dll.BMC_MoveToPosition(self.serial, self.channel, pos)
            self.position = float(1.0*pos/self.device_to_mm)
        else:
            print("Error thordevices.MoveToIn: Cannot move to given position")
    #

    def GetStageAxisInfo_MaxPos(self):
        return self.dll.BMC_GetStageAxisMaxPos(self.serial, self.channel)
    #

    def GetStageAxisInfo_MinPos(self):
        return self.dll.BMC_GetStageAxisMinPos(self.serial, self.channel)
    #

    def __del__(self):
        self.dll.BMC_StopPolling(self.serial, self.channel)
        self.dll.BMC_Close(self.serial)
    #
#

class RotationController():
    '''
    A class for controlling a Thorlabs Rotation Stage such as a K10CR1, but more genercally a
    "Integrated Stepper Motor"
    '''
    def __init__(self, serialNum, autohome=True):
        s = str(serialNum)
        self.serialNum = serialNum
        self.serial = byref(ctypes.create_string_buffer(s.encode('utf-8')))
        self.channel = ctypes.c_short(1)
        self.pollingrate = ctypes.c_int(100)

        self.angle = 0.0
        self.device_to_deg = pm.ROTATION_step_to_deg

        # Load the DLLs
        self.device_manager = ctypes.CDLL(join(pm.DLL_directory, DLL_device)) #Here to make subsequent modules load
        self.dll = ctypes.CDLL(join(pm.DLL_directory, DLL_rotation))

        # Initialize the device

        self.dll.TLI_BuildDeviceList() # Somehow allows it to find the device

        self.dll.ISC_Open(self.serial)
        self.dll.ISC_EnableChannel(self.serial)
        self.dll.ISC_StartPolling(self.serial, self.pollingrate)
        time.sleep(0.1)

        self.minPos = 1.0*self.GetStageAxisInfo_MinPos()
        self.maxPos = 1.0*self.GetStageAxisInfo_MaxPos()

        try:
            velocity = ctypes.c_int(180000000)
            current_accel = ctypes.c_int(0)
            current_velocity = ctypes.c_int(0)
            self.dll.ISC_GetVelParams(self.serial, byref(current_velocity), byref(current_accel))
            self.dll.ISC_SetVelParams(self.serial, velocity, current_accel)
        except Exception as e:
            print(format_exc())

        # Home the device
        if autohome:
            self.dll.ISC_Home(self.serial)
            time.sleep(0.25)
            self.wait_till_stopped()

        self.GetPosition()
    #

    def wait_till_stopped(self):
        loopcnt = 0
        time.sleep(0.1)
        while self.dll.ISC_GetStatusBits(self.serial) != rotation_stop_bits:
            if loopcnt > 300:
                print("Error throdevices.RotationController.wait_till_stopped: Timed out after 30 seconds")
                break
            time.sleep(0.1)
            loopcnt += 1
    #

    def GetPosition(self):
        time.sleep(0.05)
        pos = self.dll.ISC_GetPosition(self.serial)
        pos = float(1.0*pos/self.device_to_deg)
        self.angle = pos
        return pos
    #

    def MoveDeg(self, position, wait_forever=False):
        if position >= 0.0 and position <= 360.0:
            loopcnt = 0
            pos = int(position*self.device_to_deg)
            self.dll.ISC_MoveToPosition(self.serial, pos)
            while self.dll.ISC_GetPosition(self.serial) != pos:
                if loopcnt > 600 and not wait_forever:
                    print("Error throdevices.RotationController.MoveDeg: Timed out after 60 seconds")
                    break
                time.sleep(0.1)
                loopcnt += 1
            self.angle = float(1.0*pos/self.device_to_deg)
        else:
            print("Error thordevices.MoveMM: Cannot Move to given position")
    #

    def GetStageAxisInfo_MaxPos(self):
        return self.dll.ISC_GetStageAxisMaxPos(self.serial)
    #

    def GetStageAxisInfo_MinPos(self):
        return self.dll.ISC_GetStageAxisMinPos(self.serial)
    #

    def __del__(self):
        self.dll.ISC_StopPolling(self.serial)
        self.dll.ISC_Close(self.serial)
    #
#

class DualRotationController():
    '''
    A class for controlling two Thorlabs Rotation Stages, such as a K10CR1, in concert to control
    two laser beam-lines. Highly specialized to dual beam control applications.
    '''
    def __init__(self, serialDEL, serialREF, autohome=True):
        self.serialNumDel = serialDEL
        sd = str(serialDEL)
        self.serialD = byref(ctypes.create_string_buffer(sd.encode('utf-8')))
        self.serialNumRef = serialREF
        sr = str(serialREF)
        self.serialR = byref(ctypes.create_string_buffer(sr.encode('utf-8')))

        self.channel = ctypes.c_short(1)
        self.pollingrate = ctypes.c_int(100)
        self.device_to_deg = pm.ROTATION_step_to_deg

        self.angleD = 0.0
        self.angleR = 0.0

        # Load the DLLs
        self.device_manager = ctypes.CDLL(join(pm.DLL_directory, DLL_device)) #Here to make subsequent modules load
        self.dll = ctypes.CDLL(join(pm.DLL_directory, DLL_rotation))

        # Initialize the device
        self.dll.TLI_BuildDeviceList() # Somehow allows it to find the device

        self.dll.ISC_Open(self.serialD)
        self.dll.ISC_Open(self.serialR)
        self.dll.ISC_EnableChannel(self.serialD)
        self.dll.ISC_EnableChannel(self.serialR)
        self.dll.ISC_StartPolling(self.serialD, self.pollingrate)
        self.dll.ISC_StartPolling(self.serialR, self.pollingrate)
        time.sleep(0.1)

        self.minPosD = 1.0*self.GetStageAxisInfo_MinPos(self.serialD)
        self.maxPosD = 1.0*self.GetStageAxisInfo_MaxPos(self.serialD)
        self.minPosR = 1.0*self.GetStageAxisInfo_MinPos(self.serialR)
        self.maxPosR = 1.0*self.GetStageAxisInfo_MaxPos(self.serialR)

        try:
            velocity = ctypes.c_int(180000000)
            current_accel = ctypes.c_int(0)
            current_velocity = ctypes.c_int(0)
            self.dll.ISC_GetVelParams(self.serialD, byref(current_velocity), byref(current_accel))
            self.dll.ISC_SetVelParams(self.serialD, velocity, current_accel)
            self.dll.ISC_GetVelParams(self.serialR, byref(current_velocity), byref(current_accel))
            self.dll.ISC_SetVelParams(self.serialR, velocity, current_accel)
        except Exception as e:
            print(format_exc())

        if autohome:
            self.dll.ISC_Home(self.serialD)
            self.dll.ISC_Home(self.serialR)
            time.sleep(0.25)
            self.wait_till_stopped()

        self.GetPosition()
    #

    def wait_till_stopped(self):
        loopcnt = 0
        time.sleep(0.1)
        while self.dll.ISC_GetStatusBits(self.serialD) != rotation_stop_bits or self.dll.ISC_GetStatusBits(self.serialR) != rotation_stop_bits:
            if loopcnt > 600:
                print("Error throdevices.DualRotationController.wait_till_stopped: Timed out after 60 seconds")
                break
            time.sleep(0.1)
            loopcnt += 1
    #

    def GetPosition(self):
        time.sleep(0.05)
        pos1 = self.dll.ISC_GetPosition(self.serialD)
        pos1 = float(1.0*pos1/self.device_to_deg)
        self.angleD = pos1
        pos2 = self.dll.ISC_GetPosition(self.serialR)
        pos2 = float(1.0*pos2/self.device_to_deg)
        self.angleR = pos2
        return pos1, pos2
    #

    def MoveDeg(self, positionDelay, positionRef, wait_forever=False):
        if positionDelay >= 0.0 and positionDelay <= 360.0 and positionRef >= 0.0 and positionRef <= 360.0:
            loopcnt = 0
            posd = int(positionDelay*self.device_to_deg)
            self.dll.ISC_MoveToPosition(self.serialD, posd)
            posr = int(positionRef*self.device_to_deg)
            self.dll.ISC_MoveToPosition(self.serialR, posr)
            while self.dll.ISC_GetPosition(self.serialD) != posd or self.dll.ISC_GetPosition(self.serialR) != posr:
                if loopcnt > 600 and not wait_forever:
                    print("Error throdevices.RotationController.MoveDeg: Timed out after 30 seconds")
                    break
                time.sleep(0.1)
                loopcnt += 1
            self.angleD = float(1.0*posd/self.device_to_deg)
            self.angleR = float(1.0*posr/self.device_to_deg)
        else:
            print("Error thordevices.MoveMM: Cannot Move to given position")
    #

    def GetStageAxisInfo_MaxPos(self, serial):
        return self.dll.ISC_GetStageAxisMaxPos(serial)
    #

    def GetStageAxisInfo_MinPos(self, serial):
        return self.dll.ISC_GetStageAxisMinPos(serial)
    #

    def __del__(self):
        self.dll.ISC_StopPolling(self.serialD)
        self.dll.ISC_StopPolling(self.serialR)
        self.dll.ISC_Close(self.serialD)
        self.dll.ISC_Close(self.serialR)
    #
#

class BSCRotationController():
    '''
    A class for controlling a Thorlabs larger size Rotation Stage, but more
    genercally a "Benchtop Stepper Motor"
    '''
    def __init__(self, serialNum, autohome=True):
        s = str(serialNum)
        self.serialNum = serialNum
        self.serial = byref(ctypes.create_string_buffer(s.encode('utf-8')))
        self.channel = ctypes.c_short(1)
        self.pollingrate = ctypes.c_int(200)

        self.angle = 0.0
        self.device_to_deg = pm.ROTATION_NanoRotator_step_to_deg

        # Load the DLLs
        self.device_manager = ctypes.CDLL(join(pm.DLL_directory, DLL_device)) #Here to make subsequent modules load
        self.dll = ctypes.CDLL(join(pm.DLL_directory, DLL_BSC))

        # Initialize the device

        self.dll.TLI_BuildDeviceList() # Somehow allows it to find the device
        self.dll.SBC_Open(self.serial)
        self.dll.SBC_EnableChannel(self.serial, self.channel)
        self.dll.SBC_StartPolling(self.serial, self.channel, self.pollingrate)

        time.sleep(0.1)

        self.minPos = 1.0*self.GetStageAxisInfo_MinPos()
        self.maxPos = 1.0*self.GetStageAxisInfo_MaxPos()

        try:
            velocity = ctypes.c_int(pm.ROTATION_NanoRotator_default_vel)
            acceleration = ctypes.c_int(pm.ROTATION_NanoRotator_default_acc)
            current_accel = ctypes.c_int(0)
            current_velocity = ctypes.c_int(0)
            self.dll.SBC_SetVelParams(self.serial, self.channel, velocity, acceleration)
            self.dll.SBC_GetVelParams(self.serial, self.channel, byref(current_velocity), byref(current_accel))
        except Exception as e:
            print(format_exc())

        if autohome:
            self.dll.SBC_Home(self.serial, self.channel)
            time.sleep(0.25)
            self.wait_till_stopped()

        self.GetPosition()
    #

    def wait_till_stopped(self):
        loopcnt = 0
        time.sleep(0.1)
        while self.dll.SBC_GetStatusBits(self.serial, self.channel) != Nanorotator_stop_bits:
            if loopcnt > 300:
                print("Error throdevices.RotationController.wait_till_stopped: Timed out after 30 seconds")
                break
            time.sleep(0.1)
            loopcnt += 1
    #

    def GetPosition(self):
        time.sleep(0.05)
        pos = self.dll.SBC_GetPosition(self.serial, self.channel)
        pos = float(1.0*pos/self.device_to_deg)
        self.angle = pos
        return pos
    #

    def MoveDeg(self, position, wait_forever=False):
        if position >= 0.0 and position <= 360.0:
            loopcnt = 0
            pos = int(position*self.device_to_deg)
            self.dll.SBC_MoveToPosition(self.serial, self.channel, pos)
            while self.dll.SBC_GetPosition(self.serial, self.channel) != pos:
                if loopcnt > 600 and not wait_forever:
                    print("Error throdevices.RotationController.MoveDeg: Timed out after 60 seconds")
                    break
                self.angle = float(self.dll.SBC_GetPosition(self.serial, self.channel)/self.device_to_deg)
                time.sleep(0.1)
                loopcnt += 1
            self.angle = float(self.dll.SBC_GetPosition(self.serial, self.channel)/self.device_to_deg)
        else:
            print("Error thordevices.MoveMM: Cannot Move to given position")
    #

    def GetStageAxisInfo_MaxPos(self):
        return self.dll.SBC_GetStageAxisMaxPos(self.serial, self.channel)
    #

    def GetStageAxisInfo_MinPos(self):
        return self.dll.SBC_GetStageAxisMinPos(self.serial, self.channel)
    #

    def __del__(self):
        self.dll.SBC_StopPolling(self.serial, self.channel)
        self.dll.SBC_Close(self.serial)
    #
#
