'''
hyperDAQ : Hypercubic Data Aquisition Program

devicegui.py

A module for user interfaces of hardware devices controlled through hyperDAQ

Last Updated: January 2020

|  Trevor Arp
|  Gabor Lab
|  University of California, Riverside

All Rights Reserved
'''
import tkinter as tk
from traceback import format_exc
import numpy as np
from os.path import join
from os import listdir
from datetime import date
from scipy.interpolate import interp1d

import parameters as pm

class generic_device():
    '''
    Generic Interface for a hardware device

    $master is the frame in which to pack it

    $gui is a reference to the broader user interface, used for writing out information

    $controller is the device controller

    $calibration file is the calibration of the device if needed

    '''

    def __init__(self, master, gui, controller, data_out, calibration_file=None):
        '''
        Generic initilizer
        '''
        self.frame = tk.Frame(master, relief=tk.RIDGE, bd=5)
        self.gui = gui
        self.controller = controller
        self.data_out = data_out
        if calibration_file is not None:
            self.calibrated = True
            self.calibration_file = calibration_file
            self.load_calibration()
        else:
            self.calibrated = False

        self.init_interface()
    # end __init__

    def init_interface(self):
        '''
        Initlizes all the tk elements, override for functionality
        '''
        pass
    # end init_interface

    def update(self):
        '''
        Updates all visible parameters
        '''
        pass
    # end update

    def log_status(self):
        '''
        Logs all relevant parameters for a scan
        '''
        pass
    # end log_status

    def load_calibration(self):
        '''
        Loads the calibration data if applicable, override to add functionality
        '''
        pass
    #
# end generic_device


class thor_delay_stage(generic_device):
    '''
    Controller for a Thor Labs delay stage
    '''
    def init_interface(self):
        self.absDelayPositionTEXT = tk.StringVar()
        self.absDelayPositionTEXT.set("000.0")
        self.relDelaypsTEXT = tk.StringVar()
        self.relDelaypsTEXT.set("0000.0")
        tk.Label(self.frame, text="Delay Stage Controller").grid(row=0, column=1)
        tk.Label(self.frame, text="Absolute Position:   ").grid(row=1, column=0, sticky=tk.W)
        tk.Label(self.frame, textvariable=self.absDelayPositionTEXT, width=7).grid(row=1, column=1)
        tk.Button(self.frame, text="Set Position", command=self.setdelaypos_callback, width=12).grid(row=1, column=2, sticky=tk.E, padx=5)
        tk.Label(self.frame, text="Relative Delay (ps): ").grid(row=2, column=0, sticky=tk.W)
        tk.Label(self.frame, textvariable=self.relDelaypsTEXT, width=7).grid(row=2, column=1)
        tk.Button(self.frame, text="Set Delay", command=self.setdelaytime_callback, width=12).grid(row=2, column=2, sticky=tk.E, padx=5)
    # end init_interface

    def update(self):
        try:
            p = float(self.controller.position)
            rel = float((pm.DELAY_center_position - p)*pm.DELAY_mm_to_ps)
            self.absDelayPositionTEXT.set(str(round(p, 2)))
            self.relDelaypsTEXT.set(str(round(rel, 3)))
        except Exception as e:
            self.gui.display_Error("could not update delay stage position")
    # update

    def log_status(self):
        try:
            p = float(self.controller.position)
            rel = float((pm.DELAY_center_position - p)*pm.DELAY_mm_to_ps)
            self.data_out.log_param("Delay Stage Position", p)
            self.data_out.log_param("Optical Delay(ps)", rel)
        except Exception as e:
            self.gui.display_Error("could not log delay stage position")
    # end log_status

    def set_delay(self, v):
        p = pm.DELAY_center_position - v/pm.DELAY_mm_to_ps
        self.controller.MoveMM(p)
        self.update()
    # end set_delay

    def scan_delay(self, v1, v2):
        p1 = pm.DELAY_center_position - v1/pm.DELAY_mm_to_ps
        p2 = pm.DELAY_center_position - v2/pm.DELAY_mm_to_ps
        t = (1.0 - self.gui.scanner.shift - self.gui.scanner.stay)/self.gui.scanner.linerate
        self.controller.MoveMM(p1)
        self.update()
        self.controller.MoveToIn(p2, t)
    # end scan_delay

    def setdelaypos_callback(self):
        val = self.gui.popup_entry('Set Delay Stage', [['Enter Absolute Position', '', self.controller.position]])
        try:
            v = float(val[0])
        except ValueError:
            s = "Given value is not a float"
            self.gui.display_Error(s)
            return
        if v >= pm.DELAY_min_mm and v <= pm.DELAY_max_mm:
            self.gui.control_queue.put([self.controller.MoveMM, [v]])
            self.gui.display_Text('Moving Delay Stage to ' + str(round(v,2)) + ' mm')
        else:
            s = "position must be between " + str(pm.DELAY_min_mm) + " and " + str(pm.DELAY_max_mm) + " mm"
            self.gui.display_Error(s)
    # setdelaypos_callback

    def setdelaytime_callback(self):
        val = self.gui.popup_entry('Set Delay Stage', [['Enter Delay (ps)', '', 0.0]])
        try:
            v = float(val[0])
        except ValueError:
            s = "Given value is not a float"
            self.gui.display_Error(s)
            return
        if v >= pm.DELAY_min_ps and v <= pm.DELAY_max_ps:
            p = pm.DELAY_center_position - v/pm.DELAY_mm_to_ps
            self.gui.control_queue.put([self.controller.MoveMM, [p]])
            self.gui.display_Text('Moving Delay Stage to ' + str(round(v,2)) + ' ps')
        else:
            s = "delay must be between " + str(round(pm.DELAY_min_ps)) + " and " + str(round(pm.DELAY_max_ps)) + " ps"
            self.gui.display_Error(s)
    # setdelaytime_callback
# end thor_delay_stage

class thor_rotation_stage(generic_device):
    '''
    Simple Controller for Thor Labs rotation stage
    '''
    def init_interface(self, anglename='Angle', angletitle="Rotation Stage Angle: ", anglelog="Rotation Stage Position" ):
        self.rotationangleminiFRAME = tk.Frame(self.frame)
        self.rotationTEXT = tk.StringVar()
        self.rotationTEXT.set(str(round(self.controller.angle, 1)))
        tk.Label(self.rotationangleminiFRAME, text=angletitle).grid(row=0, column=0)
        tk.Label(self.rotationangleminiFRAME, textvariable=self.rotationTEXT, width=7).grid(row=0, column=1)
        self.angleBUTTON = tk.Button(self.rotationangleminiFRAME, text="Set Angle", command=self.set_angle_callback, width=12)
        self.angleBUTTON.grid(row=0, column=2, sticky=tk.W)
        self.rotationangleminiFRAME.grid(row=0, column=0, stick='w')
        self.anglename = str(anglename)
        self.anglelog = str(anglelog)
    # end init_interface

    def update(self):
        try:
            p = float(self.controller.angle)
            self.rotationTEXT.set(str(round(p, 3)))
        except Exception as e:
            self.gui.display_Error("could not update rotation stage position")
    # update

    def set_angle(self, v):
        self.controller.MoveDeg(v)
        self.update()
    # end set_delay

    def log_status(self):
        try:
            p = float(self.controller.angle)
            self.data_out.log_param(self.anglelog, p)
        except Exception as e:
            self.gui.display_Error("could not log rotation stage position")
    # end log_status

    def set_angle_callback(self):
        spec = [[self.anglename, 'Degrees', str(round(self.controller.angle, 1))]]
        v = self.gui.popup_entry("Enter New Angle", spec)
        try:
            v = float(v[0])
        except:
            self.gui.display_Error('Given Value is Not a Float')
            return
        if v >= pm.ROTATION_min_angle and v <= pm.ROTATION_max_angle:
            self.gui.control_queue.put([self.controller.MoveDeg, [v]])
            self.gui.display_Text('Setting rotation angle to ' + str(round(v,1)))
        else:
            s = 'position must be between '+str(pm.ROTATION_min_angle)+" and "+str(pm.ROTATION_max_angle)+" degrees"
            self.gui.display_Error(s)
    # end set_angle_callback
# end thor_rotation_stage

'''
Special classes for different angle contollers
'''

class optic_angle_stage(thor_rotation_stage):
    def init_interface(self):
        thor_rotation_stage.init_interface(self, anglename='Optic Angle', angletitle="Optic Angle: ", anglelog="Optic Angle Position")
    #
#

class inplane_angle_stage(thor_rotation_stage):
    def init_interface(self):
        thor_rotation_stage.init_interface(self, anglename='In-Plane Angle', angletitle="In-Plane Angle: ", anglelog="In-Plane Angle Position")
    #
#

class polarization_angle(thor_rotation_stage):
    '''
    A rotation stage specialized for polarization
    '''
    def init_interface(self, anglename='Angle', angletitle="Rotation Stage Angle: ", anglelog="Rotation Stage Position"):
        self.rotationTEXT = tk.StringVar()
        self.rotationTEXT.set(str(round(self.controller.angle, 1)))
        self.anglename = str(anglename)
        self.anglelog = str(anglelog)

        tk.Label(self.frame, text="").grid(row=0, column=0)
        tk.Label(self.frame, text="Polarization Stage").place(relx=0.5, y=13, anchor='center')
        self.rotationangleminiFRAME = tk.Frame(self.frame)
        tk.Label(self.rotationangleminiFRAME, text="Polarization Angle: ").grid(row=0, column=0)
        tk.Label(self.rotationangleminiFRAME, textvariable=self.rotationTEXT, anchor=tk.E, width=6).grid(row=0, column=1)
        tk.Label(self.rotationangleminiFRAME, text=u"\u00b0" +"  ", anchor=tk.W).grid(row=0, column=2, sticky=tk.W)
        self.angleBUTTON = tk.Button(self.rotationangleminiFRAME, text="Set Angle", command=self.set_angle_callback, width=12)
        self.angleBUTTON.grid(row=0, column=3, sticky=tk.W)
        self.rotationangleminiFRAME.grid(row=1, column=0, stick='w')
    # end init_interface

    def log_status(self):
        try:
            p = float(self.controller.angle)
            self.data_out.log_param("Polarization Rotator Position", p)
        except Exception as e:
            self.gui.display_Error("could not log polarization angle")
    # end log_status
# end polarization_angle

class calibrate_power_angle(thor_rotation_stage):
    '''
    A rotation stage specialized to calibrate the rotation stages
    '''

    def init_interface(self, anglename='Angle', angletitle="Rotation Stage Angle: ", anglelog="Rotation Stage Position"):
        self.rawPower = 0.0
        self.rawPowerTEXT = tk.StringVar()
        self.rawPowerTEXT.set(str(self.rawPower))
        if self.controller.serialNum == pm.SERIAL_rotation_delay:
            self.title = "Delay Beam"
        elif self.controller.serialNum == pm.SERIAL_rotation_refer:
            self.title = "Reference Beam"
        else:
            self.title = "Rotation Stage"
        self.rotationangleminiFRAME = tk.Frame(self.frame)
        self.rotationTEXT = tk.StringVar()
        self.rotationTEXT.set(str(round(self.controller.angle, 1)))
        tk.Label(self.rotationangleminiFRAME, text=self.title + " Angle: ").grid(row=0, column=0)
        tk.Label(self.rotationangleminiFRAME, textvariable=self.rotationTEXT, width=5).grid(row=0, column=1)
        self.angleBUTTON = tk.Button(self.rotationangleminiFRAME, text="Set Angle", command=self.set_angle_callback, width=12)
        self.angleBUTTON.grid(row=0, column=2, sticky=tk.W)
        self.rotationangleminiFRAME.grid(row=0, column=0, stick='w')

        self.powerdisplayFRAME = tk.Frame(self.frame)
        tk.Label(self.powerdisplayFRAME, text="InGaAs Value: ", anchor=tk.W).grid(row=0, column=0, sticky=tk.W)
        tk.Label(self.powerdisplayFRAME, textvariable=self.rawPowerTEXT, width=4).grid(row=0, column=1, sticky=tk.W)
        self.powerdisplayFRAME.grid(row=1, column=0, sticky=tk.W)

        self.anglename = str(anglename)
        self.anglelog = str(anglelog)
    # end init_interface

    def update(self):
        try:
            self.rawPower = self.gui.auximages['powerimg'].current_val
            self.rawPowerTEXT.set(str(round(self.rawPower, ndigits=3)))
            self.rotationTEXT.set(str(round(self.controller.angle,1)))
        except Exception as e:
            self.gui.display_Error("could not update power control")
            print(e)
            format_exc()
    # end update
# end calibrate_power_angle


class dual_stage_power_control(generic_device):
    '''
    An interface for two rotation stages, with keys 'rot_stage' and 'rot_stage_2', for dual control in
    pump probe setups
    '''

    def __init__(self, master, gui, controller, data_out, second_stage='rot_stage_2'):
        self.ratio = pm.BEAM_default_ratio
        self.power = 0.0
        self.rawPower = 0.0
        self.percent = pm.DEFAULT_Power_Percent
        generic_device.__init__(self, master, gui, controller, data_out, calibration_file=join('calibration','power'))
        self.set_power(self.percent)
    # end init

    def init_interface(self):
        tk.Label(self.frame, text="").grid(row=0, column=0)
        tk.Label(self.frame, text="Dual Beam Power Controller").place(relx=0.5, y=13, anchor='center')
        self.powerdisplayFRAME = tk.Frame(self.frame)
        self.powerTEXT = tk.StringVar()
        self.powerTEXT.set(str(self.power))
        self.rawPowerTEXT = tk.StringVar()
        self.rawPowerTEXT.set(str(self.rawPower))
        self.percentTEXT = tk.StringVar()
        self.percentTEXT.set(str(self.percent))
        tk.Label(self.powerdisplayFRAME, text="Power:").grid(row=0, column=0, sticky=tk.W)
        tk.Label(self.powerdisplayFRAME, textvariable=self.powerTEXT, anchor=tk.E, width=4).grid(row=0, column=1, sticky=tk.W)
        tk.Label(self.powerdisplayFRAME, text="mW   Raw:", anchor=tk.W).grid(row=0, column=2, sticky=tk.W)
        tk.Label(self.powerdisplayFRAME, textvariable=self.rawPowerTEXT, width=4).grid(row=0, column=3, sticky=tk.W)
        tk.Label(self.powerdisplayFRAME, text="  ").grid(row=0, column=4, sticky=tk.W)
        tk.Label(self.powerdisplayFRAME, textvariable=self.percentTEXT, width=5, anchor=tk.E).grid(row=0, column=5, sticky=tk.W)
        tk.Label(self.powerdisplayFRAME, text="%  ").grid(row=0, column=6, sticky=tk.W)
        self.powerBUTTON = tk.Button(self.powerdisplayFRAME, text="Set Power", command=self.set_power_callback, width=12)
        self.powerBUTTON.grid(row=0, column=7, sticky=tk.W)
        self.powerdisplayFRAME.grid(row=1, column=0, sticky=tk.W)

        self.angledisplayFRAME = tk.Frame(self.frame)
        self.angledelayTEXT = tk.StringVar()
        self.angledelayTEXT.set(str(round(self.controller.angleD,1)))
        self.anglerefTEXT = tk.StringVar()
        self.anglerefTEXT.set(str(round(self.controller.angleR,1)))
        tk.Label(self.angledisplayFRAME, text="Delay: ").grid(row=0, column=0, sticky=tk.W)
        tk.Label(self.angledisplayFRAME, textvariable=self.angledelayTEXT, anchor=tk.E, width=5).grid(row=0, column=1, sticky=tk.W)
        tk.Label(self.angledisplayFRAME, text=u"\u00b0"+"     Reference:", anchor=tk.W).grid(row=0, column=2, sticky=tk.W)
        tk.Label(self.angledisplayFRAME, textvariable=self.anglerefTEXT, anchor=tk.E, width=5).grid(row=0, column=3, sticky=tk.W)
        tk.Label(self.angledisplayFRAME, text=u"\u00b0" +"      ", anchor=tk.W).grid(row=0, column=4, sticky=tk.W)
        # self.angleBUTTON = tk.Button(self.angledisplayFRAME, text="Set Angles", command=self.set_angle_callback, width=12)
        # self.angleBUTTON.grid(row=0, column=5, sticky=tk.W)
        self.angledisplayFRAME.grid(row=2, column=0, sticky=tk.W)

        self.balanceRatioFRAME = tk.Frame(self.frame)
        self.balanceTEXT = tk.StringVar()
        self.balanceTEXT.set(str(round(self.ratio, 2)))
        tk.Label(self.balanceRatioFRAME, text="Beam Ratio (R/D): ").grid(row=0, column=0, sticky=tk.W)
        tk.Label(self.balanceRatioFRAME, textvariable=self.balanceTEXT, width=6).grid(row=0, column=3, sticky=tk.W)
        self.balanceBUTTON = tk.Button(self.balanceRatioFRAME, text="Set Ratio", command=self.set_ratio_callback, width=12)
        self.balanceBUTTON.grid(row=0, column=5, sticky=tk.W)
        self.balanceRatioFRAME.grid(row=3, column=0, sticky=tk.W)
    # end init_interface

    def update(self):
        try:
            self.rawPower = self.gui.auximages['powerimg'].current_val
            self.power = self.calibrate_power(self.rawPower)
            self.powerTEXT.set(str(round(self.power, ndigits=2)))
            self.rawPowerTEXT.set(str(round(self.rawPower, ndigits=3)))
            self.angledelayTEXT.set(str(round(self.controller.angleD,1)))
            self.anglerefTEXT.set(str(round(self.controller.angleR,1)))
            self.percentTEXT.set(str(round(self.percent,2)))
        except Exception as e:
            self.gui.display_Error("could not update power control")
            print(e)
            format_exc()
    # end update

    def log_status(self):
        try:
            self.data_out.log_param("Delay Beam Angle", self.controller.angleD)
            self.data_out.log_param("Reference Beam Angle", self.controller.angleR)
            self.data_out.log_param("Beam Ratio", self.ratio)
            self.data_out.log_param("IR Power", self.power)
            self.data_out.log_param("Raw Power", self.rawPower)
            self.data_out.log_param("Percent Power", self.percent)
            self.data_out.log_param("Power Calibration File", self.power_calibration_file)
        except Exception as e:
            self.gui.display_Error("could not log rotation stage position")
    # end log_status

    def set_power(self, p):
        self.percent = p
        if p > 0.0 and p <= 100.0:
            thetaD = self.percent_del(p/100.0)
            thetaR = self.percent_ref(self.ratio*self.r0*p/100.0)
            if thetaR > self.ref_theta_max:
                thetaR = self.ref_theta_max
                self.gui.display_Warning('reference beam maxed out')
            if thetaD > self.del_theta_max:
                thetaD = self.del_theta_max
                self.gui.display_Warning('delay beam maxed out')
        else:
            thetaD = 0.0
            thetaR = 0.0
        if thetaD >= pm.ROTATION_min_angle and thetaD <= pm.ROTATION_max_angle and thetaR >= pm.ROTATION_min_angle and thetaR <= pm.ROTATION_max_angle:
            self.controller.MoveDeg(thetaD, thetaR)
        else:
            self.gui.display_Error("Couldn't set rotation stage angle")
    # end set_power

    def set_angle_callback(self):
        spec = [['Delay Angle', 'Degrees', str(round(self.controller.angleD, 1))],
                ['Reference Angle', 'Degrees', str(round(self.controller.angleR, 1))]]
        v = self.gui.popup_entry("Enter New Angles", spec)
        try:
            v_del = float(v[0])
            v_ref = float(v[1])
        except:
            self.gui.display_Error('Given Value(s) is not a Float')
            return
        if v_del >= pm.ROTATION_min_angle and v_del <= pm.ROTATION_max_angle and v_ref >= pm.ROTATION_min_angle and v_ref <= pm.ROTATION_max_angle:
            self.gui.control_queue.put([self.controller.MoveDeg, [v_del, v_ref]])
            self.gui.display_Text('Setting Delay angle to ' + str(round(v_del,1)))
            self.gui.display_Text('Setting Reference angle to ' + str(round(v_ref,1)))
        else:
            s = 'position must be between '+str(pm.ROTATION_min_angle)+" and "+str(pm.ROTATION_max_angle)+" degrees"
            self.gui.display_Error(s)
    # end set_angle_callback

    def set_power_callback(self):
        spec = [['Power', '%', str(round(self.percent, 2))]]
        v = self.gui.popup_entry("Enter Power", spec)
        try:
            v = float(v[0])
        except:
            self.gui.display_Error('Given Value(s) is not a Float')
            return
        if v <= 100.0 and v >= 0.0:
            self.gui.control_queue.put([self.set_power, [v]])
        else:
            s = 'Power percentage must be between 0 and 100'
            self.gui.display_Error(s)
    # end set_angle_callback

    def set_ratio_callback(self):
        spec = [['Beam Ratio', '', str(round(self.ratio, 2))]]
        v = self.gui.popup_entry("Enter New Ratio", spec)
        try:
            v = float(v[0])
        except:
            self.gui.display_Error('Given Value(s) is not a Float')
            return
        if v <= pm.BEAM_max_ratio and v >= pm.BEAM_min_ratio:
            self.ratio = v
            self.balanceTEXT.set(str(round(self.ratio, 2)))
            if v > 1.0/self.r0:
                self.gui.display_Warning('for ratio >' + str(round(1.0/self.r0,2)) + ' reference beam may max out')
            self.gui.control_queue.put([self.set_power, [self.percent]])
        else:
            s = 'beam ratio must be between '+str(pm.BEAM_min_ratio)+" and "+str(pm.BEAM_max_ratio)
            self.gui.display_Error(s)
    # end dual_stage_power_control

    def load_calibration(self):
        # InGaAs calibration
        self.calibration_file = pm.CAL_power_dir
        self.InGaAs_calibration_file = join(pm.CAL_ingaas_dir,'InGaAs_responsivity.txt')
        files = listdir(self.calibration_file)
        rundate = date.today()
        lastdate = date(2014,1,1)
        for f in files:
            if f.split('.')[1] == 'txt':
                s = f.split('_')
                fdate = date(int(s[0]), int(s[1]), int(s[2]))
                if rundate >= fdate and fdate >= lastdate:
                    calib_file = f
                    lastdate = fdate
        self.power_calibration_file = join(self.calibration_file, calib_file)
        d = np.loadtxt(self.power_calibration_file)
        self.pfit = np.polyfit(d[:,1], d[:,0], 1)
        self.InGaAs_response = np.loadtxt(self.InGaAs_calibration_file)
        self.resp1250 = self.calibrate_responsivity(1250.0)

        # Beam Power Curves
        delay = np.loadtxt(join(pm.CAL_beam_dir, pm.DELAY_power_curve))
        ref = np.loadtxt(join(pm.CAL_beam_dir, pm.REF_power_curve))
        self.r0 = pm.DELAY_beam_max/pm.REF_beam_max #DEFAULT_beam_ratio
        self.percent_del = interp1d(delay[:,0]/np.max(delay[:,0]), delay[:,1], bounds_error=False, fill_value='extrapolate')
        self.percent_ref = interp1d(ref[:,0]/np.max(ref[:,0]), ref[:,1], bounds_error=False, fill_value='extrapolate')
        self.del_theta_max = np.max(delay[:,1])
        self.ref_theta_max = np.max(ref[:,1])
    # end load_calibration

    def calibrate_responsivity(self, wav):
        c = self.InGaAs_response
        rows, cols = np.shape(c)
        if wav not in c[:,0]:
            ix = np.searchsorted(c[:,0], wav)
            if ix < 2:
                resp = np.interp(wav, c[0:4,0], c[0:4,1])
            elif ix > rows-2:
                resp = np.interp(wav, c[rows-4:rows,0], c[rows-4:rows,1])
            else:
                resp = np.interp(wav, c[ix-2:ix+2,0], c[ix-2:ix+2,1])
        else:
            i = int(np.argwhere(c[:,0]==wav))
            resp = c[i,1]
        return float(resp)
    # end calibrate_responsivity

    def calibrate_power(self, data):
        try:
            wav = float(self.gui.device_dict['opo_control'].wavelength)
        except Exception as e:
            wav = 1200.0
        if wav < 800.0:
            wav = 1200.0
        resp = self.calibrate_responsivity(wav)
        rawP = data*(self.resp1250/resp)
        return self.pfit[0]*rawP + self.pfit[1]
    # end calibrate_power
# dual_stage_power_control

class oop_angle_stage(thor_rotation_stage):
    '''
    Out Of Plane (OOP) angle controller
    '''
    def init_interface(self):
        thor_rotation_stage.init_interface(self, anglename='OOP Angle', angletitle="OOP Angle: ", anglelog="OOP Angle Position")
    #
#

'''
Serial-Controlled Hardware
'''

class lakeshore_336_temperature(generic_device):
    '''
    Controller for CPS temperature sensors and Lakeshore 336 hardware
    '''

    def init_interface(self):
        self.temp_A_TEXT = tk.StringVar()
        self.temp_A_TEXT.set("300.0")
        self.temp_B_TEXT = tk.StringVar()
        self.temp_B_TEXT.set("301.0")
        self.temp_C_TEXT = tk.StringVar()
        self.temp_C_TEXT.set("302.0")
        self.temp_D_TEXT = tk.StringVar()
        self.temp_D_TEXT.set("303.0")
        tk.Label(self.frame, text="").grid(row=0, column=0)
        tk.Label(self.frame, text="Lakeshore 336 Temperature Controller").place(relx=0.5, y=13, anchor='center')
        tk.Label(self.frame, text="Temp A: ").grid(row=1, column=0)
        tk.Label(self.frame, textvariable=self.temp_A_TEXT, width=5).grid(row=1, column=1)
        tk.Label(self.frame, text=" K").grid(row=1, column=2, sticky=tk.W)

        tk.Label(self.frame, text="Temp B: ").grid(row=1, column=3)
        tk.Label(self.frame, textvariable=self.temp_B_TEXT, width=5).grid(row=1, column=4)
        tk.Label(self.frame, text=" K").grid(row=1, column=5, sticky=tk.W)

        tk.Label(self.frame, text="Temp C: ").grid(row=2, column=0)
        tk.Label(self.frame, textvariable=self.temp_C_TEXT, width=5).grid(row=2, column=1)
        tk.Label(self.frame, text="K").grid(row=2, column=2, sticky=tk.W)

        tk.Label(self.frame, text="Temp D: ").grid(row=2, column=3)
        tk.Label(self.frame, textvariable=self.temp_D_TEXT, width=5).grid(row=2, column=4)
        tk.Label(self.frame, text="K").grid(row=2, column=5, sticky=tk.W)

        tk.Button(self.frame, text="   Setpoint    ", width=15, command=self.setpoint_callback).grid(row=1, column=6, padx=5)
        self.heater_onoff_TEXT = tk.StringVar()
        self.heater_onoff_TEXT.set("Turn Heater On ")
        tk.Button(self.frame, textvariable=self.heater_onoff_TEXT, width=15, command=self.heateronoff_callback).grid(row=2, column=6, padx=5)
    # end init_interface

    def update(self):
        self.temp_A_TEXT.set(str(round(self.controller.A, ndigits=1)))
        self.temp_B_TEXT.set(str(round(self.controller.B, ndigits=1)))
        self.temp_C_TEXT.set(str(round(self.controller.C, ndigits=1)))
        self.temp_D_TEXT.set(str(round(self.controller.D, ndigits=1)))
    # end update

    def log_status(self):
        self.data_out.log_param("Temperature A", round(self.controller.A, ndigits=3))
        self.data_out.log_param("Temperature B", round(self.controller.B, ndigits=3))
        self.data_out.log_param("Temperature C", round(self.controller.C, ndigits=3))
        self.data_out.log_param("Temperature D", round(self.controller.D, ndigits=3))
        self.data_out.log_param("Temperature Setpoint", round(self.controller.get_setpoint(), ndigits=3))
    # end log_status

    def setpoint_callback(self):
        val = self.gui.popup_entry('Setpoint', [['Setpoint', 'K', 0.0]])
        try:
            v = float(val[0])
        except ValueError:
            s = "Given value is not a float"
            self.gui.display_Error(s)
            return
        if v <= pm.MAX_heater_temp:
            self.gui.control_queue.put([self.controller.set_setpoint, [v]])
            self.gui.display_Text('temperature Setpoint changed to ' + str(round(v,1)))
        else:
            s = "Cannot set temperature setpoint higher than parameter MAX_heater_temp=" + str(int(pm.MAX_heater_temp)) + 'K'
            self.gui.display_Error(s)
    # setpoint_callback

    def heateronoff_callback(self):
        status = int(self.controller.get_output_setting())
        if status == 0: # heater is off
            self.controller.set_output_setting(2)
            self.heater_onoff_TEXT.set("Turn Heater Off")
        else:
            self.controller.turn_heater_off()
            self.heater_onoff_TEXT.set("Turn Heater On ")
		#
    # heateronoff_callback
# end lakeshore_temperature

class lakeshore_625_magnet(generic_device):
    '''
    Controller for Superconducting Magnet through Lakeshore 625 hardware
    '''
    def init_interface(self):
        self.magnetFRAME = tk.Frame(self.frame)
        self.voltageTEXT = tk.StringVar()
        self.voltageTEXT.set("0.000")
        self.currentTEXT = tk.StringVar()
        self.currentTEXT.set("0.000")
        self.fieldTEXT = tk.StringVar()
        self.fieldTEXT.set("0.000")
        tk.Label(self.magnetFRAME, text="Voltage: ").grid(row=0, column=0)
        tk.Label(self.magnetFRAME, textvariable=self.voltageTEXT, width=6).grid(row=0, column=1)
        tk.Label(self.magnetFRAME, text=" V").grid(row=0, column=2)
        tk.Label(self.magnetFRAME, text="Current: ").grid(row=1, column=0)
        tk.Label(self.magnetFRAME, textvariable=self.currentTEXT, width=6).grid(row=1, column=1)
        tk.Label(self.magnetFRAME, text=" A").grid(row=1, column=2)
        tk.Label(self.magnetFRAME, text="Field:   ").grid(row=2, column=0)
        tk.Label(self.magnetFRAME, textvariable=self.fieldTEXT, width=6).grid(row=2, column=1)
        tk.Label(self.magnetFRAME, text=" T").grid(row=2, column=2)

        tk.Label(self.frame, text="").grid(row=0, column=1)
        tk.Label(self.frame, text="Lakeshore 625 Magnet Controller").place(relx=0.5, y=13, anchor='center')
        self.magnetFRAME.grid(row=1, column=0, sticky=tk.W)
    # end init_interface

    def log_status(self):
        self.data_out.log_param("Magnet Voltage", self.controller.voltage)
        self.data_out.log_param("Magnet Current", self.controller.current)
        self.data_out.log_param("Magnet Field", self.controller.field)
    # end log_status

    def update(self):
        self.voltageTEXT.set(str(round(self.controller.voltage, ndigits=3)))
        self.currentTEXT.set(str(round(self.controller.current, ndigits=3)))
        self.fieldTEXT.set(str(round(self.controller.field, ndigits=3)))
    # end update
# end lakeshore_625_magnet

class MIRA_900_OPO(generic_device):
    '''
    Controller for the OPO Laser
    '''
    def init_interface(self):
        self.wavelengthTEXT = tk.StringVar()
        self.wavelengthTEXT.set("0000.00")
        self.piezoTEXT = tk.StringVar()
        self.piezoTEXT.set("000.0")
        tk.Label(self.frame, text="").grid(row=0, column=0)
        tk.Label(self.frame, text="MIRA 900 OPO").place(relx=0.5, y=13, anchor='center')
        tk.Label(self.frame, text="Wavelength (nm):").grid(row=1, column=0, sticky=tk.W)
        tk.Label(self.frame, textvariable=self.wavelengthTEXT, width=6).grid(row=1, column=1, sticky=tk.W)
        tk.Button(self.frame, text="Set Wavelength", command=self.set_wavelength_callback).grid(row=1, column=3, padx=5)
        tk.Label(self.frame, text="Piezo: ", width=8).grid(row=2, column=0)
        tk.Label(self.frame, textvariable=self.piezoTEXT).grid(row=2, column=1)
    # end init_interface

    def set_wavelength_callback(self):
        v = self.gui.popup_entry('Set OPO Wavelength', [['Wavelength', 'nm', round(self.controller.wavelength, ndigits=2)]])
        try:
            v = float(v[0])
        except ValueError:
            self.gui.display_Error("Error gui.CPS_interface: Given value is not a float")
        if v <= pm.OPO_max_wavelength and  v >= pm.OPO_min_wavelength:
            self.gui.control_queue.put([self.controller.set_wavelength, [v]])
            self.gui.display_Text('Wavelength Setpoint changed to ' + str(round(v,1)))
        else:
            s = "Error: Input Wavelength must be between " + str(pm.OPO_min_wavelength) + " and " + str(pm.OPO_max_wavelength)
            self.gui.display_Error(s)
    # end set_wavelength_callback

    def set_wavelength(self, v):
        if v <= pm.OPO_max_wavelength and  v >= pm.OPO_min_wavelength:
            self.controller.set_wavelength(v)
        else:
            s = "Error: Input Wavelength must be between " + str(pm.OPO_min_wavelength) + " and " + str(pm.OPO_max_wavelength)
            self.gui.display_Error(s)
    # end set_wavelength_callback

    def update(self):
        try:
            self.wavelengthTEXT.set(round(self.controller.wavelength, ndigits=2))
            self.piezoTEXT.set(round(self.controller.piezo, ndigits=2))
        except Exception as e:
            self.gui.display_Error("Cannot Update OPO Controller")
    # update

    def log_status(self):
        self.data_out.log_param("Wavelength", round(self.controller.wavelength, ndigits=2))
        self.data_out.log_param("FWHM", round(self.controller.fwhm, ndigits=2))
        self.data_out.log_param("Piezo Voltage", round(self.controller.piezo, ndigits=2))
    # end log_status
# end MIRA_900_OPO

class MIRA_900_OPO_Stable(MIRA_900_OPO):
    '''
    Controller for the OPO Laser, attempts to correct for variations in the OPO output based on a calibration.

    Calibration should be performed each time the laser is adjusted, or has drifted significatly.
    '''

    def __init__(self, master, gui, controller, data_out):
        self.pwcontroller = None
        self.reference = [0.0, 0.0, 0.0] # %, wavelength
        generic_device.__init__(self, master, gui, controller, data_out, calibration_file=join(pm.CAL_OPO_dir, pm.OPO_curve))
    # end init

    def init_interface(self):
        MIRA_900_OPO.init_interface(self)
        tk.Button(self.frame, text="Set Reference", command=self.set_reference).grid(row=3, column=0, sticky=tk.W)
    # end init_interface


    def set_reference(self):
        if self.pwcontroller is None:
            try:
                self.pwcontroller = self.gui.dev_interfaces['rot_stage']
            except:
                s = "Error: Cannot set power controller"
                self.gui.display_Error(s)
                print(format_exc())
                return

        try:
            self.reference[0] = self.pwcontroller.percent
            self.reference[1] = self.pwcontroller.power
            self.reference[2] = self.controller.wavelength
            self.correction = interp1d(self.w_cal, self.funcP(self.reference[2])/self.p_cal, bounds_error=False, fill_value='extrapolate')
        except:
            s = "Error: Cannot set reference"
            self.gui.display_Error(s)
            print(format_exc())
        self.gui.display_Text('OPO power reference set as ' + str(round(self.reference[2],2)) + ' nm')
    # end set_reference

    def set_wavelength(self, v):
        if self.reference is None:
            s = "Error: no OPO reference"
            self.gui.display_Error(s)
            raise ValueError("No OPO reference")

        if v < self.calMIN or v > self.calMAX:
            s = "Error: requested wavelength not calibrated."
            self.gui.display_Error(s)
            raise ValueError("requested wavelength not calibrated")

        if v <= pm.OPO_max_wavelength and  v >= pm.OPO_min_wavelength:
            self.controller.set_wavelength(v)
        else:
            s = "Error: Input Wavelength must be between " + str(pm.OPO_min_wavelength) + " and " + str(pm.OPO_max_wavelength)
            self.gui.display_Error(s)
            return
        #

        # Set the power to try and keep the laser power the same as it was in the reference
        pcnt = self.reference[0]*self.correction(v)
        if pcnt > 100.0:
            pcnt = 100.0
        elif pcnt < 0.0:
            pcnt = 0.0
        #
        self.pwcontroller.set_power(pcnt)
    # end set_wavelength

    def load_calibration(self):
        caldata = np.loadtxt(self.calibration_file)
        self.w_cal = caldata[:,0]
        self.p_cal = caldata[:,1]
        self.calMIN = np.min(self.w_cal)
        self.calMAX = np.max(self.w_cal)
        self.funcP = interp1d(self.w_cal, self.p_cal, bounds_error=False, fill_value='extrapolate')
    # end load_calibration
# end MIRA_900_OPO_Stable

class spectrapro_monochrometer(generic_device):
    '''
    Controller for the OPO Laser
    '''
    def init_interface(self):
        tk.Label(self.frame, text="Monochromator Controls").grid(row=0, column=0)
        self.wavelength_miniFRAME = tk.Frame(self.frame)
        self.wavelengthTEXT = tk.StringVar()
        self.wavelengthTEXT.set("0000.00")
        tk.Label(self.wavelength_miniFRAME, text="Wavelength (nm):").grid(row=0, column=0, sticky=tk.W)
        tk.Label(self.wavelength_miniFRAME, textvariable=self.wavelengthTEXT, width=6).grid(row=0, column=1, sticky=tk.W)
        tk.Button(self.wavelength_miniFRAME, text="Set Wavelength", command=self.set_wavelength_callback).grid(row=0, column=3, padx=5)
        self.wavelength_miniFRAME.grid(row=1, column=0, sticky=tk.W)
    # end init_interface

    def set_wavelength(self, v):
        self.controller.set_wavelength(v)
        self.update()
    # end set_delay

    def set_wavelength_callback(self):
        v = self.gui.popup_entry('Enter Wavelength', [['Wavelength', 'nm', round(self.controller.wavelength, ndigits=2)]])
        try:
            v = float(v[0])
        except ValueError:
            self.gui.display_Error("Error devices_gui: Given value is not a float")
        if v <= pm.MAX_mono_wavelength and  v >= pm.MIN_mono_wavelength:
            self.gui.control_queue.put([self.controller.set_wavelength, [v]])
            self.gui.display_Text('Wavelength changed to ' + str(round(v,1)))
        else:
            s = "Input Wavelength must be between " + str(pm.MIN_mono_wavelength) + " and " + str(pm.MAX_mono_wavelength)
            self.gui.display_Error(s)
    # end set_wavelength_callback

    def update(self):
        try:
            self.controller.read_data()
            self.wavelengthTEXT.set(round(self.controller.wavelength, ndigits=3))
        except Exception as e:
            self.gui.display_Error("Cannot Update Monochrometer Controller")
            print(format_exc())
    # update

    def log_status(self):
        self.data_out.log_param("Monochromator Wavelength", round(self.controller.wavelength, ndigits=3))
    # end log_status
# end MIRA_900_OPO


'''
CODE FROM DAVE 11/2018

CURRENTLY BREAKS OTHER SETUPS, NEEDS TO BE MOVED TO A SYSTEM SPECIFIC FILE
OR RE-WRITTEN
'''

# class NDF_rotation_stage(thor_rotation_stage):
#     '''
#     	Was least evil way of accomplishing everything I wanted without
#     	editing main.py or any other main library files. I also had to
#     	implement the diamond in order to save copy and paste time and
#     	in order to not edit any previous classes.
#
#     	NDF_rotation_angle contains only functions and variables shared by the
#     	interfaces for power and angle. rotationTEXT and powerTEXT are the user
#     	import text boxes for power and angle. It is shared so that a call to update
#     	from either interface will update both interfaces, helpfull if you want to keep
#     	track of both as you scan through one.
#     '''
#     rotationTEXT =  None
#     powerTEXT = None
#     ODperDegree = pm.NDFILTER_optical_density / pm.NDFILTER_angular_range
#
#
#     def degToPwr(self, degrees):
#         '''
#         take angle degrees and use the formula OD = degrees * OD/degree
#         to get optical density. Then use the formula I/I_0 = 10^(-OD).
#         I is assumed proportional to power, assumed proportional to
#         the reflection image's amplitude. We also use if statements
#         to catch angles that fall outside range and return early
#         '''
#         if degrees <= pm.NDFILTER_range_start or degrees >= pm.NDFILTER_range_end:
#             return 1
#         degrees -= pm.NDFILTER_range_start
#         if degrees >= 360.0:
#             degrees -= 360.0
#         if degrees < 0.0:
#             degrees += 360.0
#         OD = degrees * self.ODperDegree
#         return 0.1**OD
#     # end degToPwr
#
#
#     def update(self):
#         '''
#         Is overloaded method that will be used by both rotation and angle
#         interfaces so that when either is called to update, they both update.
#         '''
#         try:
#             angle = float(self.controller.angle)
#             pwr = self.degToPwr(angle)
#             NDF_rotation_stage.powerTEXT.set(str(round(pwr, 3)))
#             NDF_rotation_stage.rotationTEXT.set(str(round(angle, 3)))
#         except Exception as e:
#             self.gui.display_Error("could not update rotation stage position")
#     # end update
# # end NDF_rotation_stage
#
# class NDF_rotation_power(NDF_rotation_stage):
#     '''
# 	Is an interface that allows scanning over the fraction I/I_0 in equal
# 	steps or to just adjust this fraction as necessary
#
# 	inherits degToPwr(), update(), and calibrate() from NDF_rotation_stage
# 	inherits set_angle() and set_angle_callback() from thor_rotation_stage
# 	inherits __init__() from generic_device
#     '''
#
#     def init_interface(self, anglename='ND Filter Power'):
#         self.rotationangleminiFRAME = tk.Frame(self.frame)
#
#         NDF_rotation_stage.powerTEXT = tk.StringVar()
#         NDF_rotation_stage.powerTEXT.set(str(round(self.degToPwr(self.controller.angle), 3)))
#
#         tk.Label(self.rotationangleminiFRAME, text="Percent Intensity: ").grid(row=0, column=0)
#         tk.Label(self.rotationangleminiFRAME, textvariable=self.powerTEXT, width=7).grid(row=0, column=1)
#
#         self.powerBUTTON = tk.Button(self.rotationangleminiFRAME, text="Set Power", command=self.set_power_callback, width=12)
#         self.powerBUTTON.grid(row=0, column=2, sticky=tk.W)
#
#         self.rotationangleminiFRAME.grid(row=0, column=0, stick='w')
#         self.anglename = str(anglename)
#     # end init_interface
#
#     def log_status(self):
#         try:
#             r = self.degToPwr(self.controller.angle)
#             self.data_out.log_param("NDF Rotation Stage Power Ratio", r)
#         except Exception as e:
#             self.gui.display_Error("could not log ndf rotation stage power ratio")
#     # end log_status
#
#     def pwrToDeg(self, pctPwr):
#         if pctPwr >= pm.NDFILTER_max_power:
#             return 0
#         elif pctPwr < pm.NDFILTER_min_power:
#             self.gui.display_Error("Power percentage is too low, " + str(round(pctPwr,3)) +
#             ". Must be at least " + str(round(pm.NDFILTER_min_power,3)) + ". Rotating to that position now.")
#             pctPwr = pm.NDFILTER_min_power
#         else:
#             OD = -1.0*np.log10(pctPwr)
#             degrees = pm.NDFILTER_range_start + ( OD / self.ODperDegree )
#             return degrees
#     # end pwrToDeg
#
#     def set_power(self, v):
#         self.set_angle(self.pwrToDeg(v))
#     # end set_delay
#
#     def set_power_callback(self):
#         spec = [[self.anglename, 'Percent', str(round(100.0*self.degToPwr(self.controller.angle), 2))]]
#         p = self.gui.popup_entry("Enter New Percent Intensity", spec)
#         degrees = 0
#         try:
#             pwr = float(p[0]) / 100.0
#             degrees = self.pwrToDeg(pwr)
#         except:
#             self.gui.display_Error('Given Value is Not a Float')
#             return
#         if degrees >= pm.ROTATION_min_angle and degrees <= pm.ROTATION_max_angle:
#             self.gui.control_queue.put([self.controller.MoveDeg, [degrees]])
#             self.gui.display_Text('Setting ndf power to ' + str(round(pwr,2)) + '(' + str(round(degrees,2)) + 'degrees)')
#             self.gui.display_Text('Setting rotation')
#         else:
#             s = 'position must be between '+str(pm.ROTATION_min_angle)+ ' and ' +str(pm.ROTATION_max_angle)+ ' degrees'
#             self.gui.display_Error(s)
#     # end set_power_callback
# #end NDF_rotation_power
#
# class NDF_rotation_angle(NDF_rotation_stage):
#     '''
# 	is an interface class that allows working with the filter's angle directly
# 	instead of using power.
#     '''
#     def init_interface(self):
#         thor_rotation_stage.init_interface(self, anglename='ND Filter Angle', angletitle="ND Filter Angle: ", anglelog="ND Filter Angle ")
#         NDF_rotation_stage.rotationTEXT = self.rotationTEXT
#     # end init_interface
# # end NDF_rotation_angle
