'''
main.py

A module for defining the main User Inferface and main thread of the program

Last Updated: January 2020

|  Trevor Arp
|  Gabor Lab
|  University of California, Riverside

All Rights Reserved
'''
import tkinter as tk
import tkinter.messagebox as mb
import time
from timeit import default_timer as timer
import os
import numpy as np
from traceback import format_exc
from queue import Queue
from scipy.ndimage.measurements import center_of_mass
import threading

import matplotlib
matplotlib.use('TkAgg')
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt

import parameters as pm
from hyperdaq.utilities import changeEntry, checkEntryStar, isPosInt, brute_diff_min
from hyperdaq.card import CAL

class Scan_Spec():
    '''
    A specification for a given scan, passed to the control thread for processing.

    Meant to be extensible to allow for increased functionality in the future.

    The critical parameter is Scan_Spec.dimensions, a list containing the specifications for each
    dimension, the index determines the order of the variables.

    Each specification is a list with the following entries:
    [variable_identifier, start_value, end_value, number_of_points, sampling_function]
    where the sampling function is numpy.linspace by default i.e. numpy.linspace(start_value,
    end_value, number_of_points)
    '''

    def __init__(self):
        self.type = pm.SCAN_default_type
        self.scanvars = []
        self.constants = dict()
        self.cube = False
        self.drift_correct = False
        self.drift_type = None
        self.dimensions = []
    # end __init__

    def add_constant(self, parameter, value):
        '''
        Adds a scanning parameter that will be held constant during the scan.

        Args:
            parameter (str): the parameter to hold constant
            value (float): the value to hold it at
        '''
        if parameter in self.constants.keys() or parameter in self.scanvars:
            print("Warning parameter: " + str(parameter) + ' already in scan spec, nothing updated')
        else:
            self.constants[parameter] = value
    # end add_constant

    def add_axis(self, parameter, start, end, N):
        '''
        Adds a scanning parameter that will be scanned linearly over the range [start, end] taking
        N samples.

        If the paramter was previously defined as a constant this will overwrite it.
        '''
        if parameter in self.scanvars:
            print("Warning parameter: " + str(parameter) + ' already in scan spec, nothing updated')
        else:
            self.dimensions.append([parameter, start, end, N, 'linspace'])
            self.scanvars.append(parameter)
            if parameter in self.constants.keys():
                del self.constants[parameter]
    # end add_axis

    def add_cube_axis(self, parameter, start, end, N):
        '''
        Adds a scanning parameter as decribed in add_axis.

        If this function is called the scan will save as 3D cubes instead of 2D scans.
        '''
        self.add_axis(parameter, start, end, N)
        if self.cube:
            print("Warning scan is already a cube, the third axis is always the cube axis")
        self.cube = True
    # end add_cube_axis

    def set_drift_correction(self, type):
        '''
        Enable dirft correction and set which image to use
        '''
        self.drift_correct = True
        self.drift_type = type
    # end drift_correct

    def set_scan_type(self, type):
        '''
        Change the scan type from default
        '''
        self.type = type
    # end set_scan_type

    def sampleaxis(self, dim):
        '''
        Returns the samples to an axis based on the specified sampling function, for an axis with
        index dim. If dim is 0 or 1, sampleaxis may not be called as that is usually set by the
        scanning function, be sure to check.
        '''
        if self.dimensions[dim][4] == 'linspace':
            return np.linspace(self.dimensions[dim][1], self.dimensions[dim][2], self.dimensions[dim][3])
        elif self.dimensions[dim][4] == 'geomspace':
            return np.geomspace(self.dimensions[dim][1], self.dimensions[dim][2], self.dimensions[dim][3])
        else:
            raise ValueError("Could not sample with function " + str(self.dimensions[dim][4]))
    # end sampleaxis

    def verify(self):
        '''
        Returns true if all required scan parameters are present and there are at least two axes, false otherwise.

        Required parameters are defined by SCAN_spatial_parameters and SCAN_voltage_parameters.
        '''
        for p in pm.SCAN_spatial_parameters:
            if not p in self.constants.keys() and not p in self.scanvars:
                return False
        for p in pm.SCAN_voltage_parameters:
            if not p in self.constants.keys() and not p in self.scanvars:
                return False
        return True
    # end verify
# end Scan_Spec

class Control_Thread(threading.Thread):
    '''
    The thread that controls scanning and other active functions, while allowing
    the GUI to continue updating

    Accepts commands from a queue, while it is performing the task self.active is true, this
    can be used to block input from the GUI while the task is working

    Will process two kinds of commands:
    (1) It will run a hardware command, for example moving the delay stage into position, called with
    command [function, args], where function is a hardware function that takes args (list of arguments)

    (2) It will run a scan based on a valid Scan_Spec object passed to it
    '''

    '''
    Threading Functions
    '''
    def __init__(self, command_q, main_UI, device_dict, scanner, data_out):
        self.com_q = command_q
        self.gui = main_UI
        self.device_dict = device_dict
        self.scanner = scanner
        self.data_out = data_out
        self.drift_type = 'rfi'
        threading.Thread.__init__(self)
        self.running = True
        self.active = False
        self.start()
    # end init

    def run(self):
        '''
        The main loop of the program, called in the new thread.
        '''
        while self.running:
            time.sleep(pm.MAIN_loop_delay/4.0)
            if not self.com_q.empty():
                self.active = True
                c = self.com_q.get()
                try:
                    if len(c) == 1:
                        self.process_scan(c[0])
                    elif len(c) == 2:
                        func = c[0]
                        func(*c[1])
                    else:
                        self.gui.display_Error("Invalid Command")
                except Exception as e:
                    self.gui.display_Error("DAQ control thread cannot complete task")
                    print(format_exc())
                self.active = False
    # end run

    def stop(self):
        '''
        Stops the active process
        '''
        self.running = False
        if self.active:
            self.gui.abort_scan()
        self.active = False
    # end stop

    def process_scan(self, specification):
        '''
        Executes a scan based on the given specification
        '''

        if len(specification.dimensions) < 2:
            self.gui.display_Error("Invalid specification, requires at least two axes")
            return
        #

        outputs = self.starting_card_output_values(specification)

        if specification.drift_correct:
            self.init_drift_buffer(outputs)
            self.drift_type = specification.drift_type
        #

        # Perform the scan
        self.perform_scan(specification, outputs, len(specification.dimensions)-1)
    # end process_scan

    def perform_scan(self, specification, outputs, dim, scanNum=''):
        '''
        Perform a scan, either an individual scan or a data cube.
        parameter dim is the scan dimension to consider, if the dimension is higher than the
        output (scan or cube) repeat the scan recursively along that axis.
        '''

        if dim > 2 or (dim == 2 and not specification.cube): # recursive case
            repeat_spec = specification.dimensions[dim]
            Nrep = repeat_spec[3]
            rngRep = specification.sampleaxis(dim)

            if scanNum is '':
                self.gui.display_Text('Starting ' + str(Nrep) + ' Scans')
            else:
                self.gui.display_Text('Starting ' + str(Nrep) + ' Scans ' + scanNum)

            st = '/'+str(Nrep)+' '

            for i in range(Nrep):
                if specification.drift_correct:
                    outputs['xaxis'], outputs['yaxis'] = self.drift_correction(outputs)
                if repeat_spec[0] in outputs.keys():
                    outputs[repeat_spec[0]] = (rngRep[i], rngRep[i])
                else:
                    rep_axis = self.gui.scan_params[repeat_spec[0]]
                    interface = self.gui.dev_interfaces[repeat_spec[0]]
                    rep_func = getattr(interface, rep_axis[3])
                    rep_func(rngRep[i])
                    time.sleep(pm.CNTL_repeat_wait)
                r = self.perform_scan(specification, outputs, dim-1, scanNum=str(i+1)+st)
                if r != 1:
                    self.gui.display_Text('Ending Repeating Scan Early')
                    break
                if specification.drift_correct:
                    self.drift_buffer_images()
            return r
        elif dim == 2 and specification.cube: # base case, data cube
            self.log_params(specification, outputs)

            fastspec = specification.dimensions[0]
            slowspec = specification.dimensions[1]
            cubespec = specification.dimensions[2]

            Ncube = cubespec[3]
            rngCube = specification.sampleaxis(dim)

            # Figure out any functions that need to be passed to the scan function as keywords
            kwargs = dict()
            if not fastspec[0] in outputs.keys():
                fast_axis = self.gui.scan_params[fastspec[0]]
                fastinterface = self.gui.dev_interfaces[fastspec[0]]
                kwargs['fast_func'] = getattr(fastinterface, fast_axis[2])
                kwargs['fast_func_range'] = (fastspec[1], fastspec[2])
            #

            if not slowspec[0] in outputs.keys():
                slow_axis = self.gui.scan_params[slowspec[0]]
                slowinterface = self.gui.dev_interfaces[slowspec[0]]
                kwargs['func'] = getattr(slowinterface, slow_axis[3])
                kwargs['func_range'] = (slowspec[1], slowspec[2])
            #

            # Scan
            self.data_out.init_data_cube(Ncube)
            self.gui.display_Text("Data Cube Started")
            t0 = timer()
            for i in range(Ncube):
                t1 = timer()
                if specification.drift_correct:
                    outputs['xaxis'], outputs['yaxis'] = self.drift_correction(outputs)
                if cubespec[0] in outputs.keys():
                    outputs[cubespec[0]] = (rngCube[i], rngCube[i])
                else:
                    cube_axis = self.gui.scan_params[cubespec[0]]
                    interface = self.gui.dev_interfaces[cubespec[0]]
                    func = getattr(interface, cube_axis[3])
                    func(rngCube[i])
                    time.sleep(pm.CNTL_function_wait)
                #

                r = self.scanner.scan(specification.type, outputs, fastspec[0], **kwargs)

                if specification.drift_correct:
                    self.drift_buffer_images()
                t2 = timer()
                if r == 1:
                    self.gui.display_Text("Scan " + str(i+1) + '/' + str(Ncube) + " Completed in " + str(round(t2-t1,1)) + ' seconds')
                    self.data_out.buffer_cube()
                else:
                    self.gui.display_Text("Data Cube Aborted")
                    break
            tf = timer()

            # Cleanup
            if r == 1:
                self.gui.display_Text("Data Cube " + scanNum + "Completed in " + str(round((tf-t0)/60.0,1)) + ' minutes')
            m = self.data_out.write_images()
            if m is not None:
                self.gui.display_Text(m)
            return r
        elif dim == 1: # Base case, 2D scan
            self.log_params(specification, outputs)

            fastspec = specification.dimensions[0]
            slowspec = specification.dimensions[1]

            # Figure out any functions that need to be passed to the scan function as keywords
            kwargs = dict()
            if not fastspec[0] in outputs.keys():
                fast_axis = self.gui.scan_params[fastspec[0]]
                fastinterface = self.gui.dev_interfaces[fastspec[0]]
                kwargs['fast_func'] = getattr(fastinterface, fast_axis[2])
                kwargs['fast_func_range'] = (fastspec[1], fastspec[2])
            #

            if not slowspec[0] in outputs.keys():
                slow_axis = self.gui.scan_params[slowspec[0]]
                slowinterface = self.gui.dev_interfaces[slowspec[0]]
                kwargs['func'] = getattr(slowinterface, slow_axis[3])
                kwargs['func_range'] = (slowspec[1], slowspec[2])
            #

            # Scan
            self.gui.display_Text("Scan Started")
            t0 = timer()
            r = self.scanner.scan(specification.type, outputs, fastspec[0], **kwargs)
            tf = timer()

            # Cleanup
            if r == 1:
                self.gui.display_Text("Scan " + scanNum + "Completed in " + str(round(tf-t0,1)) + ' seconds')
            else:
                self.gui.display_Text("Scan Aborted")
            m = self.data_out.write_images()
            if m is not None:
                self.gui.display_Text(m)
            return r
        else: # Something has gone horribly wrong
            raise ValueError("Invalid scan dimension for recursive case")
        #
    # end perform_scan

    def starting_card_output_values(self, specification):
        '''
        Compute the starting values for the voltage output channels
        '''
        outputs = dict()
        for p in pm.SCAN_spatial_parameters:
            if p in specification.scanvars:
                for param in specification.dimensions:
                    if param[0] == p:
                        outputs[p] = (param[1], param[2])
            elif p in specification.constants.keys():
                outputs[p] = (specification.constants[p], specification.constants[p])
            else:
                raise ValueError("Spatial output channel " + str(p) +" not specified")
            #

        for p in pm.SCAN_voltage_parameters:
            if p in specification.scanvars:
                for param in specification.dimensions:
                    if param[0] == p:
                        outputs[p] = (param[1], param[2])
            elif p in specification.constants.keys():
                outputs[p] = (specification.constants[p], specification.constants[p])
            else:
                raise ValueError("Voltage output channel " + str(p) +" not specified")
            #
        return outputs
    # end starting_card_output_values

    def drift_correction(self, outputs):
        '''
        Calculate and apply the applicable drift correction

        WARNING: RFI CORRECTION UNTESTED AS OF 2020/1/14
        '''
        x = outputs['xaxis']
        y = outputs['yaxis']
        if self.num_buffered >= 3:

            # Calculate the applicable correction
            if self.drift_type == 'rfi': # Find the reflection image correction
                sft = brute_diff_min(self.rfi_buffer[2], self.rfi_buffer[0])
                dx = self.pix_to_x*sft[1]
                dy = self.pix_to_y*sft[0]
            else: # Photocurrent image correction
                dx = self.pix_to_x*(self.current_center[1] - self.scan_center[1])
                dy = self.pix_to_y*(self.current_center[0] - self.scan_center[0])
                # print('x:' + str(self.pix_to_x*self.current_center[1]), 'dx: ' +str(dx), 'y:' + str(self.pix_to_y*self.current_center[0]), 'dy:'+str(dy))

            # Determine if the draft is large enough to correct for
            if np.abs(dx) > pm.DRIFT_x_max:
                dx = 1.0*pm.DRIFT_x_max*np.sign(dx)
                newx = (x[0]+dx, x[1]+dx)
            elif np.abs(dx) < pm.DRIFT_x_min:
                dx = 0.0
                newx = x
            else:
                dx = 1.0*round(dx, 2)
                newx = (x[0]+dx, x[1]+dx)
            if np.abs(dy) >pm.DRIFT_y_max:
                dy = 1.0*pm.DRIFT_y_max*np.sign(dy)
                newy = (y[0]+dy, y[1]+dy)
            elif np.abs(dy) < pm.DRIFT_y_min:
                dy = 0.0
                newy = y
            else:
                dy = 1.0*round(dy, 2)
                newy = (y[0]+dy, y[1]+dy)
            #

            # Correct if applicable
            if np.abs(dx) > 0.0 or np.abs(dy) > 0.0:
                if self.drift_type == 'rfi':
                    self.num_buffered = 0 # if you correct with reflection, restart the process
                self.gui.drift_update_center_values(dx, dy)
                self.gui.display_Text("Images corrected by " + str(dx) +',' + str(dy))
                print(str(x)+' to '+str(newx), str(y)+' to '+str(newy))
                return newx, newy
            else:
                return x, y
        else:
            return x, y
    # end drift_correction

    def drift_buffer_images(self):
        '''
        Buffers the current images for use in future drift correction
        '''
        if self.drift_type == 'rfi': # Reflection image correction, based on translating the reflection image
            d = np.abs(self.gui.images[0].data)
            d = d - np.min(d)
            self.rfi_buffer[:,:,2] = self.rfi_buffer[:,:,1]
            self.rfi_buffer[:,:,1] = self.rfi_buffer[:,:,0]
            self.rfi_buffer[:,:,0] = d/np.max(d)
        else: # Photocurrent image correction, based on finding the centroid of the photocurrent
            d = np.abs(self.gui.images[1].data)
            rows, cols = np.shape(d)
            thres = np.max(d)/4
            for i in range(rows):
                for j in range(cols):
                    if d[i,j] > thres:
                        d[i,j] = 1.0
                    else:
                        d[i,j] = 0.0
            center = center_of_mass(d)
            self.current_center[0] = center[0]
            self.current_center[1] = center[1]
            if self.num_buffered < 3:
                self.scan_center[0] = self.scan_center[0] + center[0]/3.0
                self.scan_center[1] = self.scan_center[1] + center[1]/3.0
        self.num_buffered += 1
    # end drift_buffer_images

    def init_drift_buffer(self, outputs):
        '''
        Resets the drift correction buffer
        '''
        x = outputs['xaxis']
        y = outputs['yaxis']
        self.current_center = [0,0]
        self.scan_center = [0,0]
        self.rfi_buffer = np.zeros((self.scanner.ny, self.scanner.nx, 3))
        self.num_buffered = 0
        self.pix_to_x = np.abs(x[0]-x[1])/float(self.scanner.nx)
        self.pix_to_y = np.abs(y[0]-y[1])/float(self.scanner.ny)
    # end init_drift_buffer

    def log_params(self, specification, outputs):
        '''
        Log the basic scan parameters and the required scanning parameters
        '''
        # Type of output file
        if specification.cube:
            self.data_out.log_param("Scan Dimension", "3")
        else:
            self.data_out.log_param("Scan Dimension", "2")
        #

        # Go through the specification and write out the main data axes
        fastvar = specification.dimensions[0][0]
        self.data_out.log_param("Fast Axis Variable", str(self.gui.scan_params[fastvar][0]))
        self.data_out.log_param("Fast Axis Units", str(self.gui.scan_params[fastvar][1]))
        if fastvar == 'xaxis': # if so it may be updated for drift correction, use outputs
            self.data_out.log_param("Fast Axis Start", outputs['xaxis'][0])
            self.data_out.log_param("Fast Axis End", outputs['xaxis'][1])
        else:
            self.data_out.log_param("Fast Axis Start", specification.dimensions[0][1])
            self.data_out.log_param("Fast Axis End", specification.dimensions[0][2])
        self.data_out.log_param("Fast Axis Sampling", specification.dimensions[0][4])

        slowvar = specification.dimensions[1][0]
        self.data_out.log_param("Slow Axis Variable", str(self.gui.scan_params[slowvar][0]))
        self.data_out.log_param("Slow Axis Units", str(self.gui.scan_params[slowvar][1]))
        if slowvar == 'yaxis': # if so it may be updated for drift correction, use outputs
            self.data_out.log_param("Slow Axis Start", outputs['yaxis'][0])
            self.data_out.log_param("Slow Axis End", outputs['yaxis'][1])
        else:
            self.data_out.log_param("Slow Axis Start", specification.dimensions[1][1])
            self.data_out.log_param("Slow Axis End", specification.dimensions[1][2])
        self.data_out.log_param("Slow Axis Sampling", specification.dimensions[1][4])

        if specification.cube:
            cubevar = specification.dimensions[2][0]
            self.data_out.log_param("Cube Axis", str(self.gui.scan_params[cubevar][0]))
            self.data_out.log_param("Cube Axis Units", str(self.gui.scan_params[cubevar][1]))
            self.data_out.log_param("Cube Axis Start", specification.dimensions[2][1])
            self.data_out.log_param("Cube Axis End", specification.dimensions[2][2])
            self.data_out.log_param("Cube Scan Number", specification.dimensions[2][3])
            self.data_out.log_param("Cube Axis Sampling", specification.dimensions[2][4])
        else:
            cubevar = ""


        # Make sure that all the spatial parameters are logged, if they are not the fast, slow or cube axes
        for i in range(len(pm.SCAN_spatial_parameters)):
            param = pm.SCAN_spatial_parameters[i]
            if not param in [fastvar, slowvar, cubevar]:
                if outputs[param][0] == outputs[param][1]:
                    self.data_out.log_param(pm.SCAN_spatial_lognames[i], outputs[param][0])
                else:
                    raise ValueError("Attempting to log a non-constant value as a constant")
        #

        # Make sure that all the voltage parameters are logged, if they are not the fast, slow or cube axes
        for i in range(len(pm.SCAN_voltage_parameters)):
            param = pm.SCAN_voltage_parameters[i]
            if not param in [fastvar, slowvar, cubevar]:
                self.data_out.log_param(pm.SCAN_voltage_lognames[i]+" Channel", str(self.gui.scan_params[param][0]))
                self.data_out.log_param(pm.SCAN_voltage_lognames[i]+" Units", str(self.gui.scan_params[param][1]))
                self.data_out.log_param(pm.SCAN_spatial_lognames[i]+" Start", outputs[param][0])
                self.data_out.log_param(pm.SCAN_spatial_lognames[i]+" End", outputs[param][1])
        #

        #Scanning parameters that are always logged
        self.data_out.log_param("Line Rate", self.scanner.linerate)
        self.data_out.log_param("nx", self.scanner.nx)
        self.data_out.log_param("ny", self.scanner.ny)
        for k, v in self.gui.exp_params.items():
            self.data_out.log_param(k, v)
        self.data_out.log_param("Start Time", time.strftime("%Y/%m/%d %H:%M:%S"))
        for key, dev in self.gui.dev_interfaces.items():
            dev.log_status()
        self.data_out.write_params_file()
    # end log_params
# end class Control_Thread

class axis_selector():
    def __init__(self, gui, master, label, options, optional=False, opt_num_label=None, default=None):
        self.frame = tk.Frame(master, relief=tk.RIDGE, bd=5)
        self.gui = gui
        self.optional = optional
        self.label = label

        tk.Label(self.frame, text=str(self.label)+": ", width=12).grid(row=0, column=0, sticky=tk.W)
        self.param_value = tk.StringVar()
        self.params_OPTION = tk.OptionMenu(self.frame, self.param_value, *options, command=self.param_select)
        self.params_OPTION.config(width=20)
        self.params_OPTION.grid(row=0, column=1, sticky=tk.W, columnspan=5)
        if self.optional:
            self.enable = tk.IntVar()
            self.check = tk.Checkbutton(self.frame, text="Enable", variable=self.enable)
            self.check.grid(row=0, column=6, sticky=tk.W)
        self.startENTRY = tk.Entry(self.frame, width=6)
        self.endENTRY = tk.Entry(self.frame, width=6)
        tk.Label(self.frame, text="Range: ", width=12).grid(row=1, column=0, sticky=tk.W)
        self.startENTRY.grid(row=1, column=1, sticky=tk.W)
        tk.Label(self.frame, text="to").grid(row=1, column=2, sticky=tk.W)
        self.endENTRY.grid(row=1, column=3, sticky=tk.W)
        self.unitTEXT = tk.StringVar()
        tk.Label(self.frame, textvariable=self.unitTEXT, width=8).grid(row=1, column=4, sticky=tk.W)

        if self.optional:
            tk.Label(self.frame, text=str(opt_num_label)+": ").grid(row=2, column=0, sticky=tk.W)
            self.numberENTRY = tk.Entry(self.frame, width=4)
            self.numberENTRY.insert(0, str(pm.DEFAULT_number))
            self.numberENTRY.grid(row=2, column=1, sticky=tk.W)

        if default is not None:
            self.param_select(self.gui.scan_params[default][0])
    # end

    def param_select(self, event):
        '''
        Change selection of the parameter
        '''
        lbl = str(event)
        k = self.gui.params_options[lbl]
        rng = self.gui.scan_params[k][4]
        self.param_value.set(self.gui.scan_params[k][0])
        self.startENTRY.delete(0, 'end')
        self.startENTRY.insert(0, str(rng[0]))
        self.endENTRY.delete(0, 'end')
        self.endENTRY.insert(0, str(rng[1]))
        self.unitTEXT.set(str(self.gui.scan_params[k][1]))
        if self.optional:
            self.numberENTRY.delete(0, 'end')
            self.numberENTRY.insert(0, str(pm.DEFAULT_number))
    # end param_select

    def get_entries(self):
        '''
        Return the start and end entries (and number if it is an optional axis)
        '''
        try:
            s = float(self.startENTRY.get())
            e = float(self.endENTRY.get())
            if self.optional:
                N = int(float(self.numberENTRY.get()))
                return s, e, N
            else:
                return s, e
        except ValueError:
            self.gui.display_Error(str(self.label)+" entries must be floats or integers")
            return -1
    # end get_entries
# end axis_selector

class hyperDAQ():
    '''
    The main user inferface and control for the DAQ. Meant to be generic, extended
    using inheritance.

    Parameters:
    $card_in is the data input object reading from the NI DAQ card

    $images is a list of images to display

    $auximages is a dictionary of auxillary images, that are not displayed but will still be
    updated. The keys allow the images to be specifically accessed by various interfaces. Set
    to None if there are no auxillary images

    $card_control is the controller for the DAQ card

    $scan_params is a dictionary containing the parameters that it is possible to scan over, dictionary
    is formatted as "device_key":["Label", "unit", fast_function, slow_function, (default_start, default_end)]
    where the device_key can be used to find the device in the device dict or identifier for a voltage output,
    label, unit and defaults are displayed and fast_function/slow_function are the functions that can be used
    to scan the device over the fast and slow axies respectivly (or None for voltage output).

    To add new axes to this generic interface use inheritance and override the function
    init_extra_axes, which sets any additional axes beyond the fast, slow and cube
    '''

    '''
    ##############################
    Core Functions
    ##############################
    '''
    def __init__(self, images, auximages, card_control, data_writer, device_dict, interface_dict, scan_params, exp_params, exp_params_units, showdrift=True):
        '''
        Main interface initilization
        '''
        self.scanner = card_control
        self.data_out = data_writer
        self.data_out.set_gui_ref(self)
        self.exp_params = exp_params
        self.exp_params_units = exp_params_units
        self.auximages = auximages
        self.device_dict = device_dict
        self.showdrift = showdrift

        self.init_logger()

        self.num_plots = len(images)
        self.images = images

        # self.params_options is a dictionary where the labels are the keys and the values
        # are the corresponding key in self.scan_params
        self.scan_params = scan_params
        self.params_options = {}
        for k, v in self.scan_params.items():
            self.params_options[v[0]] = k

        # Initilize the UI
        self.screen = tk.Tk()
        self.screen.resizable(0,0)
        self.screen.protocol("WM_DELETE_WINDOW", self.stop)
        self.screen.wm_title(str(pm.DAQ_name))
        self.screen.geometry('+0+0')

        self.init_scanning_frame(self.screen)
        self.init_device_wrapper_frame(self.screen, interface_dict)
        self.init_plotting_frame(self.screen)

        self.scanningFRAME.grid(row=0, column=0, sticky=tk.N)
        self.deviceWrapperFRAME.grid(row=0, column=1, sticky=tk.N)
        self.plottingFRAME.grid(row=0, column=2, sticky=tk.N)

        self.init_menu()
        self.screen.config(menu=self.mainMENU)

        s = "Startup " + time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        self.display_Text(s)

        self.running = True

        # Start the control thread
        self.control_queue = Queue()
        self.control = Control_Thread(self.control_queue, self, self.device_dict, self.scanner, self.data_out)

        # Start the Updating
        self.job = self.screen.after(pm.PLT_update_delay, self.update_callback)
        self.screen.mainloop()
    # end init

    def stop(self):
        '''
        Stop the treads of all devices and close the program
        '''
        if self.data_out.recording:
            if not mb.askokcancel("Confirm Close?", str(pm.DAQ_name) + " is recording, are you sure you want to close?"):
                return
            #
        #

        self.running = False
        if self.job is not None:
            self.screen.after_cancel(self.job)
        self.control.stop()

        self.screen.quit()

        self.logger.flush()
        self.logger.close()

        # Write out experimental parameters
        f = open(pm.PARAMS_file, 'w')
        for k, v in self.exp_params.items():
        	f.write(str(k) + ':' + str(v) + '\n')
        f.flush()
        f.close()

        # Shut down input output functions
        self.data_out.close_files()
        for img in self.images:
            img.stop()
        if self.auximages is not None:
            for key, im in self.auximages.items():
                im.stop()

        # Shut down all devices
        for key, dev in self.device_dict.items():
            if hasattr(dev, 'stop'):
                dev.stop()

        # Shut down the DAQ card
        self.scanner.stop()
    # end stop

    def start_scan(self):
        '''
        Finite Scan, gathers the scan parameters from the interface and passes the scan
        specification off to the control thread.
        '''
        if self.control.active:
            self.display_Error('DAQ is currently active')
            return
        #

        # Check the main axes
        fast = self.params_options[self.fast_axis.param_value.get()]
        slow = self.params_options[self.slow_axis.param_value.get()]
        if self.cube_axis.enable.get():
            cube = self.params_options[self.cube_axis.param_value.get()]
        else:
            cube = "cube_disabled"
        axes = [fast, slow, cube]

        # Check the additional axes
        for i in range(len(self.extra_axes)):
            axis = self.extra_axes[i]
            if axis.enable.get():
                axval = self.params_options[axis.param_value.get()]
            else:
                axval = "repeat_disabled"
            axes.append(axval)
        #

        # Check that all enabled axes are scanning different parameters
        if len(axes) != len(set(axes)):
            self.display_Error('one or more axis is a duplicate')
            return
        #

        # Initilize the scan specification
        spec = Scan_Spec()

        # Check that all the offset entries are valid and add them to the spec
        for param in pm.SCAN_spatial_parameters:
            try:
                val = float(self.spatial_output_entries[param].get())
            except ValueError:
                self.display_Error("Could not read spatial values")
                return
            if not self.check_value(param, val):
                return
            spec.add_constant(param, val)
        #

        for param in pm.SCAN_voltage_parameters:
            try:
                v0 = float(self.voltage_output_entries[param].get())
            except ValueError:
                self.display_Error("Could not read voltage values")
                return
            if not self.check_value(param, v0):
                return
            spec.add_constant(param, v0)
        #

        # Check the scan varaible entries and add them to the spec
        try:
            fs, fe = self.fast_axis.get_entries()
            if fast in pm.SCAN_voltage_parameters:
                x0 = float(self.spatial_output_entries[fast].get())
                fs = fs + x0
                fe = fe + x0
        except Exception as e:
            self.display_Error('Could not read Fast Axis value')
            return
        if not self.check_value(fast, fs) or not self.check_value(fast, fe):
            return
        spec.add_axis(axes[0], fs, fe, self.scanner.nx)

        try:
            ss, se = self.slow_axis.get_entries()
            if slow in pm.SCAN_voltage_parameters:
                y0 = float(self.spatial_output_entries[slow].get())
                ss = ss + y0
                se = se + y0
        except Exception as e:
            self.display_Error('Could not read Slow Axis value')
            return
        if not self.check_value(slow, ss) or  not self.check_value(slow, se):
            return
        spec.add_axis(axes[1], ss, se, self.scanner.ny)

        if axes[2] != 'cube_disabled':
            try:
                cs, ce, Ncube = self.cube_axis.get_entries()
            except Exception as e:
                print(e)
                self.display_Error('Could not read Cube Axis value')
                return
            if not self.check_value(cube, cs) or not self.check_value(cube, ce):
                return
            spec.add_cube_axis(axes[2], cs, ce, Ncube)
        #

        # Add any additional axes
        for i in range(len(self.extra_axes)):
            axis = self.extra_axes[i]
            if axis.enable.get():
                axval = self.params_options[axis.param_value.get()]
                try:
                    rs, re, Nrep = axis.get_entries()
                except Exception as e:
                    self.display_Error('Could not read ' + str(axis.label) +' value')
                    return
                if not self.check_value(axval, rs) or not self.check_value(axval, re):
                    return
                spec.add_axis(axval, rs, re, Nrep)
            #

        # Check for spatial drift correction
        if (self.drift_im0.get() == 1 or self.drift_im1.get() == 1) and slow == 'yaxis' and fast == 'xaxis':
            if self.drift_im0.get() == 1:
                spec.set_drift_correction('rfi')
            elif self.drift_im1.get() == 1:
                spec.set_drift_correction('pci')
            else:
                self.display_Error('invalid drift correction type')
                return
        #

        if spec.verify():
            # Set the axis labels
            xtks = np.linspace(fs, fe, 5)
            ytks = np.linspace(ss, se, 5)
            for i in range(self.num_plots):
                xlbl = self.axes[i].get_xticklabels()
                for j in range(len(xtks)):
                    xlbl[j] = str(round(xtks[j],3))
                self.axes[i].set_xticklabels(xlbl)

                ylbl = self.axes[i].get_yticklabels()
                for j in range(len(ytks)):
                    ylbl[j] = str(round(ytks[j],3))
                self.axes[i].set_yticklabels(ylbl)
                self.axes[i].set_ylabel(self.scan_params[slow][1])
                self.axes[i].set_xlabel(self.scan_params[fast][1])
                self.canvases[i].draw()
            #

            # Hand the scan off the the other thread
            self.control_queue.put([spec])
        else:
            self.display_Error('Invalid scan specification')
    # end start_scan

    def check_value(self, key, value):
        '''
        Checks if a value is valid for a given key to self.scan_params
        '''
        try:
            prms = self.scan_params[key]
        except KeyError: # If it's not a scanning parameter, there is no need to check it
            return True

        lims = prms[5]
        if key in pm.SCAN_spatial_parameters:
            value = value*pm.SCAN_units_to_volts
        value = CAL(value, key)

        if value < lims[0]:
            self.display_Error(prms[0]+' real values must be greater than ' + str(lims[0]))
            return False
        elif value > lims[1]:
            self.display_Error(prms[0]+' real values must be less than ' + str(lims[1]))
            return False
        else:
            return True
    # end check_value

    def abort_scan(self):
        '''
        Abort the scan
        '''
        if self.scanner.scanning:
            self.scanner.abortscan = True
    # end abort_scan

    '''
    ##############################
    TK initilizations
    ##############################
    '''

    def init_menu(self):
        '''
        Initilizes the main file menu
        '''
        self.mainMENU = tk.Menu(self.screen)

        self.fileMENU = tk.Menu(self.mainMENU, tearoff=0)
        self.mainMENU.add_cascade(label="File", menu=self.fileMENU)
        self.fileMENU.add_command(label="Save Current Images", command=self.data_out.write_current_images)
        self.fileMENU.add_separator()
        self.fileMENU.add_command(label="Exit", command=self.stop)

        # Scanning Menu
        self.scanningMENU = tk.Menu(self.mainMENU, tearoff=0)
        self.mainMENU.add_cascade(label="Scanning", menu=self.scanningMENU)
        self.scanningMENU.add_command(label="Clear Processing Queues", command=self.scanner.clear_queues)

        self.savescanMENU = tk.Menu(self.scanningMENU, tearoff=0)
        def makeFunc(x): # to get around scope issues
            return lambda: self.save_scan(x)
        for i in range(pm.NUM_Save_Slots):
            self.savescanMENU.add_command(label=str(i+1)+":                          ", command=makeFunc(i))
        self.scanningMENU.add_cascade(label="Save Current Scan", menu=self.savescanMENU)

        self.loadscanMENU = tk.Menu(self.scanningMENU, tearoff=0)
        def makeFunc(x): # to get around scope issues
            return lambda: self.load_scan(x)
        for i in range(pm.NUM_Save_Slots):
            self.loadscanMENU.add_command(label=str(i+1)+":                          ", command=makeFunc(i))
        self.scanningMENU.add_cascade(label="Load Scan Configuration", menu=self.loadscanMENU)

        # Load in saved scan titles
        for i in range(pm.NUM_Save_Slots):
            flname = os.path.join(pm.SCANS_Save_dir,'save_'+str(i)+'.txt')
            if os.path.exists(flname):
                with open(flname, 'r') as f:
                    l = f.readline()
                    ttl = l.split(':')[1]
                    self.savescanMENU.entryconfigure(i, label=str(i+1)+': '+ttl)
                    self.loadscanMENU.entryconfigure(i, label=str(i+1)+': '+ttl)

        # Plotting Menu
        self.plottingMENU = tk.Menu(self.mainMENU, tearoff=0)
        self.mainMENU.add_cascade(label="Plotting", menu=self.plottingMENU)
        self.plottingMENU.add_command(label="Autoscale Images", command=self.autoscale_images_callback)

        im0_callback = lambda : self.image_lims_callback(0, pm.PLT_image0_title)
        im1_callback = lambda : self.image_lims_callback(1, pm.PLT_image1_title)
        self.plottingMENU.add_command(label=pm.PLT_image0_title+" Max", command=im0_callback)
        self.plottingMENU.add_command(label=pm.PLT_image1_title+" Max", command=im1_callback)
    # end init_menu

    def init_scanning_frame(self, master):
        '''
        Initilizes the Scanning Frame, Initilize the frame and all the sub-frames
        '''
        self.scanningFRAME = tk.Frame(master, relief=tk.RIDGE, bd=5)

        # Top Frame
        scan_params_miniFRAME = tk.Frame(self.scanningFRAME, relief=tk.RIDGE, bd=5)

        # Recording Frame
        self.recordingFRAME = tk.Frame(scan_params_miniFRAME)
        self.recordingTEXT = tk.StringVar()
        self.recordingTEXT.set("Not Recording")
        self.recordingBUTTON = tk.Button(self.recordingFRAME, text='Recording On/Off', command=self.toggle_record)
        self.recordingBUTTON.grid(row=0, column=0)
        self.recordingLABLE = tk.Label(self.recordingFRAME, textvariable=self.recordingTEXT, width=13)
        self.recordingLABLE.grid(row=0, column=1, columnspan=3)
        self.recordingFRAME.grid(row=0, column=0, columnspan=2, pady=2, sticky=tk.N+tk.W)

        # Spatial Center Values
        self.spatial_output_entries = dict()
        if len(pm.SCAN_spatial_parameters) > 0:
            center_values_miniFRAME = tk.Frame(scan_params_miniFRAME)
            r = 0
            c = 0
            for i in range(len(pm.SCAN_spatial_parameters)):
                lbl = pm.SCAN_spatial_lognames[i].replace("Value", "").strip()
                tk.Label(center_values_miniFRAME, text=" "+lbl+":").grid(row=r, column=c)
                entry = tk.Entry(center_values_miniFRAME, width=6)
                entry.insert(0, pm.DEFAULT_spatial_output)
                entry.grid(row=r, column=c+1)
                self.spatial_output_entries[pm.SCAN_spatial_parameters[i]] = entry
                c = c + 2
                if c >= 6:
                    r = r + 1
                    c = 0
            center_values_miniFRAME.grid(row=1, columnspan=2, pady=2)

        # Voltage Controls
        self.voltage_output_entries = dict()
        if len(pm.SCAN_voltage_parameters) > 0:
            voltage_miniFRAME = tk.Frame(scan_params_miniFRAME)
            r = 0
            c = 0
            for i in range(len(pm.SCAN_voltage_parameters)):
                tk.Label(voltage_miniFRAME, text=pm.SCAN_voltage_lognames[i] + ":", width=13).grid(row=r, column=c)
                entry = tk.Entry(voltage_miniFRAME, width=6)
                entry.insert(0, pm.DEFAULT_voltage_output)
                entry.grid(row=r, column=c+1)
                self.voltage_output_entries[pm.SCAN_voltage_parameters[i]] = entry
                c = c + 2
                if c >= 4:
                    r = r + 1
                    c = 0
            # end for
            self.voltage_lock = tk.IntVar()
            self.voltage_lock_check = tk.Checkbutton(voltage_miniFRAME, text="Lock", variable=self.voltage_lock, command=self.voltage_lock_callback)
            self.voltage_lock_check.grid(row=0, column=4, sticky=tk.E)
            voltage_miniFRAME.grid(row=2, column=0, columnspan=2, pady=2, sticky=tk.W)
        else:
            self.voltage_lock = tk.IntVar()
            self.voltage_lock.set(0)
        #

        # Image Size indicator
        img_size_miniFRAME = tk.Frame(scan_params_miniFRAME)
        self.nxTEXT = tk.StringVar()
        self.nxTEXT.set(str(self.scanner.nx))
        self.nyTEXT = tk.StringVar()
        self.nyTEXT.set(str(self.scanner.ny))
        tk.Label(img_size_miniFRAME, text="Scan Size x:").grid(row=0, column=0)
        tk.Label(img_size_miniFRAME, textvariable=self.nxTEXT, width=4, anchor=tk.W).grid(row=0, column=1)
        tk.Label(img_size_miniFRAME, text=" y:").grid(row=0, column=2)
        tk.Label(img_size_miniFRAME, textvariable=self.nyTEXT, width=4, anchor=tk.W).grid(row=0, column=3)
        self.setSizeBUTTON = tk.Button(img_size_miniFRAME, text='edit', command=self.resize_callback).grid(row=0, column=4)
        tk.Label(img_size_miniFRAME, text='  ').grid(row=0, column=5)
        img_size_miniFRAME.grid(row=3, column=0, pady=2)

        # Line rate controls
        line_rate_miniFRAME = tk.Frame(scan_params_miniFRAME)
        self.linerateTEXT = tk.StringVar()
        self.linerateTEXT.set(str(self.scanner.linerate))
        tk.Label(line_rate_miniFRAME, text="Line Rate: ").grid(row=0, column=0)
        tk.Label(line_rate_miniFRAME, textvariable=self.linerateTEXT, width=4, anchor=tk.W).grid(row=0, column=1)
        tk.Label(line_rate_miniFRAME, text="Lines/s").grid(row=0, column=2)
        self.setLineRateBUTTON = tk.Button(line_rate_miniFRAME, text='edit', command=self.set_linerate_callback).grid(row=0, column=3)
        line_rate_miniFRAME.grid(row=3, column=1, pady=2)

        fastOptions = []
        for k,v in self.params_options.items():
            if v not in pm.EXCLUDE_fast:
                fastOptions.append(k)
        fastOptions = sorted(fastOptions)

        slowOptions = []
        for k,v in self.params_options.items():
            if v not in pm.EXCLUDE_slow:
                slowOptions.append(k)
        slowOptions = sorted(slowOptions)

        altOptions = []
        for k,v in self.params_options.items():
            if v not in pm.EXCLUDE_third_axis:
                altOptions.append(k)
        altOptions = sorted(altOptions)

        # Establish Main Axes
        self.fast_axis = axis_selector(self, self.scanningFRAME, "Fast Axis", fastOptions, default='xaxis')
        self.slow_axis = axis_selector(self, self.scanningFRAME, "Slow Axis", slowOptions, default='yaxis')
        self.cube_axis = axis_selector(self, self.scanningFRAME, "Cube Axis", altOptions, optional=True, opt_num_label="Scan Number", default='vbg')

        # Establish additional Axes beyond the three main axes
        self.extra_axes = self.init_extra_axes(altOptions)

        # Drift Correction Frame
        self.init_drift_frame(self.scanningFRAME)

        # Button Frame
        self.scanbutton_miniFRAME = tk.Frame(self.scanningFRAME, relief=tk.RIDGE, bd=5)
        self.finitescanBUTTON = tk.Button(self.scanbutton_miniFRAME, text="Start Scan", command=self.start_scan, width=19, bg='gray84')
        self.abortscanBUTTON = tk.Button(self.scanbutton_miniFRAME, text="Abort Scan", command=self.abort_scan, width=19, bg='gray84')
        tk.Label(self.scanbutton_miniFRAME, text="", width=1).grid(row=0, column=0, sticky=tk.W)
        self.finitescanBUTTON.grid(row=0, column=1, sticky=tk.W, pady=7, padx=3)
        tk.Label(self.scanbutton_miniFRAME, text="", width=7).grid(row=0, column=2, sticky=tk.W)
        self.abortscanBUTTON.grid(row=0, column=3, sticky=tk.E, pady=7, padx=7)

        # Experimental Parameters
        self.init_parameters_frame(self.scanningFRAME)

        # Grid the widgets
        scan_params_miniFRAME.grid(row=0, sticky=tk.N+tk.W)

        self.fast_axis.frame.grid(row=1, sticky=tk.N+tk.W+tk.E)
        self.slow_axis.frame.grid(row=2, sticky=tk.N+tk.W+tk.E)
        self.cube_axis.frame.grid(row=3, sticky=tk.N+tk.W+tk.E)

        if len(self.extra_axes) > 0:
            for i in range(len(self.extra_axes)):
                self.extra_axes[i].frame.grid(row=4+i, sticky=tk.N+tk.W+tk.E)
        else:
            i = 0
        #

        if self.showdrift:
            self.driftFRAME.grid(row=5+i, sticky=tk.N+tk.W+tk.E)
        else:
            i = i - 1
        self.scanbutton_miniFRAME.grid(row=6+i, sticky=tk.N+tk.W+tk.E)
        self.parametersFRAME.grid(row=7+i, sticky=tk.S+tk.W+tk.E)
    # init_scanning_frame

    def init_extra_axes(self, options):
        '''
        Initialize any additional axes beyond the main three (fast, slow, cube). By default creates
         one repeat axis. Override to add additional axes or extend functionality

        The parameter is the options to give the axis (same as cube axis options)

        Should return a list (in order of increasing higher dimensions), that will get added to the
        interface in order, and be accessed by various functions in order.
        '''
        repeat_axis = axis_selector(self, self.scanningFRAME, "Repeat", options, optional=True, opt_num_label="Rep. Number", default='vsd')
        return [repeat_axis]
    # end init_extra_axes

    def init_drift_frame(self, master):
        '''
        Initilizes the frame to correct for drift
        '''
        self.driftFRAME = tk.Frame(master, relief=tk.RIDGE, bd=5)

        # Title and enable
        tk.Label(self.driftFRAME, text="Spatial Drift Correction", width=53).grid(row=0, column=0, sticky=tk.W+tk.E)

        self.drift_im0 = tk.IntVar()
        self.drift_im1 = tk.IntVar()

        def im0driftcallback():
            if self.drift_im0.get() == 1:
                self.drift_im1.set(0)
        # end im0driftcallback

        def im1driftcallback():
            if self.drift_im1.get() == 1:
                self.drift_im0.set(0)
        # end im1driftcallback

        drift_miniFRAME = tk.Frame(self.driftFRAME)

        self.im0_drift_check = tk.Checkbutton(drift_miniFRAME, text=pm.PLT_image0_title, variable=self.drift_im0, command=im0driftcallback)
        self.im0_drift_check.grid(row=0, column=0)
        tk.Label(drift_miniFRAME, text="  ", width=3).grid(row=0, column=1)

        self.im1_drift_check = tk.Checkbutton(drift_miniFRAME, text=pm.PLT_image1_title, variable=self.drift_im1, command=im1driftcallback)
        self.im1_drift_check.grid(row=0, column=2)
        drift_miniFRAME.grid(row=1, column=0, sticky=tk.W+tk.E)
    # end init_drift_frame

    def init_parameters_frame(self, master):
        '''
        Initialize the frame to display experimental parameters
        '''
        self.parametersFRAME = tk.Frame(master, relief=tk.RIDGE, bd=5)
        tk.Label(self.parametersFRAME, text="Experimental Parameters").grid(row=0, column=0, columnspan=3)
        self.params_values = dict()
        n = 1
        for k, v in self.exp_params.items():
            tk.Label(self.parametersFRAME, text=str(k), width=26).grid(row=n, column=0)
            self.params_values[k] = tk.StringVar()
            self.params_values[k].set(str(self.exp_params[k]))
            tk.Label(self.parametersFRAME, textvariable=self.params_values[k], width=8).grid(row=n, column=1)
            tk.Label(self.parametersFRAME, text=str(self.exp_params_units[k]), width=8).grid(row=n, column=2)
            n += 1
        if n < 8:
            for i in range(n,8):
            	tk.Label(self.parametersFRAME, text=" ", width=20).grid(row=i, column=0, columnspan=3)
        self.parametersFRAME.grid(row=1, column=0, columnspan=2, sticky=tk.W)

        # Options for editing the parameters
        self.exp_params_options = list(self.exp_params.keys())
        self.edit_value = tk.StringVar()
        self.edit_value.set(self.exp_params_options[0])
        self.params_OPTION = tk.OptionMenu(self.parametersFRAME, self.edit_value, *self.exp_params_options)
        self.params_OPTION.config(width=26)
        self.editBUTTON = tk.Button(self.parametersFRAME, text="Edit", command=self.edit_params_callback, width=10)
        if n < 10:
            self.params_OPTION.grid(row=9, column=0, columnspan=2, sticky=tk.W)
            self.editBUTTON.grid(row=9, column=2)
        else:
            self.params_OPTION.grid(row=n+1, column=0, columnspan=2, sticky=tk.W)
            self.editBUTTON.grid(row=n+1, column=2)
    # end init_parameters_frame

    def init_device_wrapper_frame(self, master, interface_dict):
        '''
        Initilizes the frame containing the Device UIs and the Experimental parameters
        '''
        self.deviceWrapperFRAME = tk.Frame(master, relief=tk.RIDGE, bd=5)
        tk.Label(self.deviceWrapperFRAME, text='Device frame').grid()

        # Message Frame
        self.init_message_frame(self.deviceWrapperFRAME)
        self.messageFRAME.grid(row=0, sticky=tk.N+tk.E+tk.W)

        self.dev_interfaces = dict()

        # Initilize the device UIs
        rowix = 1
        devkeys = sorted(self.device_dict.keys())
        for key in devkeys:
            if key in interface_dict:
                interface = interface_dict[key]
                dev = self.device_dict[key]
                self.dev_interfaces[key] = interface(self.deviceWrapperFRAME, self, dev, self.data_out)
                self.dev_interfaces[key].frame.grid(row=rowix, column=0, sticky=tk.N+tk.E+tk.W)
                rowix += 1
    # init_device_wrapper_frame

    def init_message_frame(self, master):
        '''
        Initilizes the frame containing message area
        '''
        self.messageFRAME = tk.Frame(master, relief=tk.RIDGE, bd=5)
        self.textOutput = tk.Text(self.messageFRAME, state='disabled', bg='grey94', width=41, height=10)
        self.textOutput.grid()
    # init_message_frame

    def init_plotting_frame(self, master):
        '''
        Initilizes the frame containing the plots
        '''
        self.plottingFRAME = tk.Frame(master, relief=tk.RIDGE, bd=5)

        self.titles = [pm.PLT_image0_title, pm.PLT_image1_title]
        self.xlabels = ['Pixels', 'Pixels']
        self.ylabels = ['Pixels', 'Pixels']
        self.aspect_ratio = 1.0*pm.NUM_y_points/pm.NUM_x_points
        self.autoscaleIm = True
        self.figs = []
        self.canvases = []
        self.widgets = []
        self.im = []
        self.axes = []
        self.cmaps = [pm.PLT_image0_cmap, pm.PLT_image1_cmap]

        #Initialize the images
        initial = self.images[0].data
        rows, cols = initial.shape

        xinches = 6.85
        yinches = 5.53 #5.7
        xmargin = 0.85
        ymargin = 0.5
        width = 4.75
        cbwidth = 0.25
        cbint = 0.15
        for i in range(self.num_plots):
            # Create the figure
            self.figs.append(plt.figure(i+1, figsize=(xinches, yinches), facecolor='#f0f0f0'))
            ax = self.figs[i].add_axes([xmargin/xinches, ymargin/yinches, width/xinches, width/yinches])
            self.axes.append(ax)
            plt.sca(ax)

            # Setup the axis
            plt.suptitle(self.titles[i], fontsize=16, y=0.99)
            ax.set_xlabel(self.xlabels[i])
            ax.set_ylabel(self.ylabels[i])
            ax.tick_params(direction='out', top=False, right=False)

            xtl = np.linspace(0, cols, 5)
            ytl = np.linspace(0, rows, 5)
            ax.set_xticks(np.arange(0,101,25))
            ax.set_xticklabels(xtl.astype(int))
            ax.set_yticks(np.arange(0,101,25))
            ax.set_yticklabels(ytl.astype(int))

            # Create the image
            self.im.append(plt.imshow(initial, cmap=self.cmaps[i], extent=[0,100,100,0], aspect='auto'))

            # Create the colorbars
            cax = self.figs[i].add_axes([(xmargin+width+cbint)/xinches, ymargin/yinches, cbwidth/xinches, width/yinches])
            cax.tick_params(direction='out')
            plt.colorbar(cax=cax, orientation='vertical')

            # Add widgets
            self.canvases.append(FigureCanvasTkAgg(self.figs[i], master=self.plottingFRAME))
            self.widgets.append(self.canvases[i].get_tk_widget())
            self.widgets[i].grid(row=i, column=0)
    # init_plotting_frame

    '''
    ##############################
    Plotting Functions
    ##############################
    '''

    def update_callback(self):
        '''
        Calls the update function after PLT_update_delay (assuming the system isn't busy)
        '''
        try:
            self.update()
        except Exception as e:
            self.display_Error("Could not update GUI")

            # Debug
            print(format_exc())
            self.running = False
            self.stop()
        if self.running:
            self.job = self.screen.after(pm.PLT_update_delay, self.update_callback)
    # end update_callback

    def update(self):
        '''
        Updates the plots
        '''

        # Update the Devices
        for key, dev in self.dev_interfaces.items():
            dev.update()

        # Update the Plots
        if self.control.active:
            for i in range(self.num_plots):
                self.im[i].set_data(self.images[i].data)
                if self.autoscaleIm:
                    self.im[i].autoscale()
                self.canvases[i].draw()
        else:
            self.scanner.pollcard()
    # end update

    def set_aspect(self, nx, ny):
        '''
    	Changes the aspect ratio of the images
    	'''
        ar = 1.0*ny/nx
        if ar > 5.0:
            ar = 5.0
        elif ar < 0.25:
            ar = 0.25
        #
        for i in range(self.num_plots):
            ax = self.axes[i]
            ax.set_aspect(ar)

            # Re-draw
            self.im[i].set_data(self.images[i].data)
            if self.autoscaleIm:
                self.im[i].autoscale()
            self.canvases[i].draw()
        self.aspect_ratio = ar
	# end set_aspect

    '''
    ##############################
    Text Functions
    ##############################
    '''

    def init_logger(self):
        '''
        Initilizes the logger
        '''
        file_dir = pm.DATA_dir_path + time.strftime("%Y\\") + time.strftime("%Y_%m\\") + time.strftime("%Y_%m_%d\\")
        if not os.path.exists(file_dir):
            os.makedirs(file_dir)
        logger_filename = file_dir + time.strftime("%Y_%m_%d.log")
        self.logger = open(logger_filename, "a")
        self.logger.write("######################" + time.strftime("[%Y/%m/%d %H:%M:%S]") +"######################\n")
    # end init_logger

    def display_Text(self, s):
        '''
        display_Texts the given string
        '''
        txt = s + '\n'
        self.textOutput.configure(state='normal')
        self.textOutput.insert('end', txt)
        self.textOutput.see(self.textOutput.index('end -1 line'))
        self.textOutput.configure(state='disabled')
        self.logger.write(txt)
        self.logger.flush()
    # end display_Text

    def display_Error(self, s):
        '''
        Prints the given string in red as an error
        '''
        txt = 'Error: ' + s + '\n'
        self.textOutput.configure(state='normal')
        self.textOutput.tag_config("error_text", foreground='red')
        self.textOutput.insert('end', txt, "error_text")
        self.textOutput.see(self.textOutput.index('end -1 line'))
        self.textOutput.configure(state='disabled')
        self.logger.write(txt)
        self.logger.flush()
    # end display_Error

    def display_Warning(self, s):
        '''
        Prints the given string in red as a warning
        '''
        txt = 'Warning: ' + s + '\n'
        self.textOutput.configure(state='normal')
        self.textOutput.tag_config("error_text", foreground='red')
        self.textOutput.insert('end', txt, "error_text")
        self.textOutput.see(self.textOutput.index('end -1 line'))
        self.textOutput.configure(state='disabled')
        self.logger.write(txt)
        self.logger.flush()
    # end display_Error

    '''
    ##############################
    Callback Functions
    ##############################
    '''

    def popup_entry(self, title, spec, entry_width=None):
        '''
        Function that can be used to get input from a user, for use in callback functions
        $title is the title of the popup window
        $spec is an array that defines the behavior of the popup, for each variable to reterive spec contains a list
        specifying what to say before and after and the starting value to be displayed. For example for a popup asking for two quatifies
        spec = [
            ['Quatity A', 'Units A', Starting_Value],
            ['Quatity B', 'Units B', Starting_Value]
        ]
        when done a list is returned containing user input [Quantity_A, Quantity_B]
        '''
        class popupWindow(object):
            def __init__(self, master, title, spec, entry_Width=5):
                self.top=tk.Toplevel(master)
                self.width = len(title)
                if self.width < 15:
                    self.width = 15
                self.title=tk.Label(self.top, text=str(title), width=self.width)
                self.title.grid(row=0, column=0)
                self.rows = len(spec)
                self.entries = []
                self.values = []
                for i in range(self.rows):
                    subFrame = tk.Frame(self.top)
                    tk.Label(subFrame, text=str(spec[i][0])+' ').grid(row=0, column=0)
                    e1 = tk.Entry(subFrame, width=entry_Width)
                    e1.insert(0, str(spec[i][2]))
                    e1.grid(row=0, column=1)
                    tk.Label(subFrame, text=' '+str(spec[i][1])).grid(row=0, column=2)
                    self.entries.append(e1)
                    self.values.append(str(spec[i][2]))
                    subFrame.grid(row=i+1, column=0)
                self.b = tk.Button(self.top, text='Ok', width=6, command=self.cleanup)
                self.b.grid(row=self.rows+1)
            # end __init__

            def cleanup(self):
                for i in range(self.rows):
                    self.values[i] = self.entries[i].get()
                self.top.destroy()
            # end cleanup
        # end popupWindow
        if entry_width is None:
            self.w = popupWindow(self.screen, title, spec)
        else:
            self.w = popupWindow(self.screen, title, spec, entry_Width=entry_width)
        self.screen.wait_window(self.w.top)
        return self.w.values
    # end popup_entry

    def toggle_record(self):
        '''
        Toggles the recording on the front panel
        '''
        if self.data_out.recording:
            self.data_out.toggle_record()
            self.recordingTEXT.set("Not Recording")
        else:
            self.data_out.toggle_record()
            self.recordingTEXT.set("  Recording  ")
    # toggle_record

    def resize_callback(self):
        '''
        Resizes the images
        '''
        can_resize = True
        for img in self.images:
            if img.processing:
                can_resize = False
        if not self.scanner.scanning and can_resize:
            spec = [
                ['nx:', 'Pixels', self.scanner.nx],
                ['ny:', 'Pixels', self.scanner.ny]
            ]
            v = self.popup_entry("Resize Images", spec)
            nx = v[0]
            ny = v[1]
            if not isPosInt(nx) and not isPosInt(ny):
                s = "nx and ny must be integers greater than 0"
                self.display_Error(s)
                return

            if int(float(nx)) <= pm.NUM_max_points and  int(float(ny)) <= pm.NUM_max_points:
                self.change_img_size(int(float(nx)), int(float(ny)))
            else:
                s = "nx and ny must be less than " + str(pm.NUM_max_points)
                self.display_Error(s)
                return
        else:
            s = "Can't resize images while scanning or processing"
            self.display_Error(s)
            return
	# end resize_callback

    def change_img_size(self, nx, ny):
        '''
        Change the size of the images, called internally
        '''
        self.nxTEXT.set(str(nx))
        self.nyTEXT.set(str(ny))

        # Resize the Images
        self.scanner.set_nxy(nx, ny)
        for i in self.images:
            i.resize()
        if self.auximages is not None:
            for key, im in self.auximages.items():
                im.resize()

        time.sleep(2*pm.MAIN_loop_delay)
        xtl = np.linspace(0, nx, 5)
        ytl = np.linspace(0, ny, 5)
        for i in range(self.num_plots):
            self.axes[i].set_xticks(np.arange(0,101,25))
            self.axes[i].set_xticklabels(xtl.astype(int))
            self.axes[i].set_yticks(np.arange(0,101,25))
            self.axes[i].set_yticklabels(ytl.astype(int))
            self.axes[i].set_ylabel('Pixels')
            self.axes[i].set_xlabel('Pixels')
        self.set_aspect(nx, ny)
        s = "Images set to " + str(nx) + 'x' + str(ny)
        self.display_Text(s)
    # end change_img_size

    def set_linerate_callback(self):
        '''
        Sets the line scan variable in the scanner
        '''
        val = self.popup_entry("Enter Line Rate", [['Line Rate', 'Hz', self.scanner.linerate]])
        try:
            v = float(val[0])
        except ValueError:
            self.display_Error("Given value is not a float")
            return
        if float(v) <= pm.RATE_line_scan:
            self.scanner.set_line_rate(v)
            self.linerateTEXT.set(str(self.scanner.linerate))
        else:
            s = "Cannot set line rate faster than parameter RATE_line_scan=" + str(pm.RATE_line_scan)
            self.display_Error(s)
    # end set_linerate_callback

    def voltage_lock_callback(self):
        '''
        Handels the locking of voltage for all parameters defined in SCAN_voltage_parameters
        '''
        if self.voltage_lock.get() == 1:
            # Check the values before doing anything else
            valid = True
            for chan in pm.SCAN_voltage_parameters:
                element = self.voltage_output_entries[chan]
                val = element.get()
                try:
                    val = float(val)
                except ValueError:
                    self.display_Error("Given value is not a float")
                    valid = False
                #

                if valid and not self.check_value(chan, val):
                    valid = False
                #

                if not valid:
                    self.voltage_lock.set(0)
                    for k in pm.SCAN_voltage_parameters:
                        entry = self.voltage_output_entries[k]
                        entry.config(state=tk.NORMAL)
                    return
            # end for

            for i in range(len(pm.SCAN_voltage_parameters)):
                key = pm.SCAN_voltage_parameters[i]
                element = self.voltage_output_entries[key]
                val = float(element.get())
                self.control_queue.put([self.scanner.set_default_voltage, [key, val]])
                self.display_Text(str(pm.SCAN_voltage_lognames[i]) + " default set to " + str(val))
                element.config(state=tk.DISABLED)
        else:
            for i in range(len(pm.SCAN_voltage_parameters)):
                key = pm.SCAN_voltage_parameters[i]
                element = self.voltage_output_entries[key]
                self.control_queue.put([self.scanner.set_default_voltage, [key, 0.0]])
                self.display_Text(str(pm.SCAN_voltage_lognames[i]) + " default set to 0.0")
                element.config(state=tk.NORMAL)
    # end voltage_lock_callback

    def edit_params_callback(self):
        '''
    	Sets the value of an experimental parameter
    	'''
        k = self.edit_value.get()
        ttl = "Edit Parameter"
        spec = [[str(k),self.exp_params_units[k], self.exp_params[k]]]
        val = self.popup_entry(ttl, spec)
        try:
            v = float(val[0])
        except ValueError:
            self.display_Error("Given value is not a float")
            return
        self.exp_params[k] = v
        self.params_values[k].set(str(self.exp_params[k]))
    # edit_params_callback

    def save_scan(self, n):
        '''
        Saves the current scanning configuration to file
        '''
        flname = os.path.join(pm.SCANS_Save_dir,'save_'+str(n)+'.txt')
        if os.path.exists(flname):
            with open(flname, 'r') as f:
                l = f.readline()
                ttl = l.split(':')[1]
            spec = [['Enter Title:', '', str(ttl)]]
        else:
            spec = [['Enter Title:', '', '']]
        inpt = self.popup_entry('Configuration Title', spec, entry_width=26)
        ttl = inpt[0].strip()
        if ttl == '':
            self.display_Error("Scan Configuration Needs a Title")
            return
        with open(flname, 'w') as f:
            f.write('Title:'+str(ttl)+'\n')

            # Write out constant values
            for i in range(len(pm.SCAN_spatial_parameters)):
                f.write(pm.SCAN_spatial_lognames[i] + ':'+str(checkEntryStar(self.spatial_output_entries[pm.SCAN_spatial_parameters[i]]))+'\n')
            for i in range(len(pm.SCAN_voltage_parameters)):
                f.write(pm.SCAN_voltage_lognames[i] + ':'+str(checkEntryStar(self.voltage_output_entries[pm.SCAN_voltage_parameters[i]]))+'\n')

            f.write('Voltage Lock Enable:'+str(self.voltage_lock.get())+'\n')
            f.write('nx:'+str(self.scanner.nx)+'\n')
            f.write('ny:'+str(self.scanner.ny)+'\n')
            f.write('Line Rate:'+str(self.scanner.linerate)+'\n')

            f.write('Drift Correct Im0:'+str(self.drift_im0.get())+'\n')
            f.write('Drift Correct Im1:'+str(self.drift_im1.get())+'\n')

            f.write('Fast Axis:'+str(self.params_options[self.fast_axis.param_value.get()])+'\n')
            try:
                fs, fe = self.fast_axis.get_entries()
            except:
                self.display_Error("Scan not saved")
                return
            f.write('Fast Axis Start:'+str(fs)+'\n')
            f.write('Fast Axis End:'+str(fe)+'\n')

            f.write('Slow Axis:'+str(self.params_options[self.slow_axis.param_value.get()])+'\n')
            try:
                ss, se = self.slow_axis.get_entries()
            except:
                self.display_Error("Scan not saved")
                return
            f.write('Slow Axis Start:'+str(ss)+'\n')
            f.write('Slow Axis End:'+str(se)+'\n')

            f.write('Cube Axis:'+str(self.params_options[self.cube_axis.param_value.get()])+'\n')
            try:
                cs, ce, Nc = self.cube_axis.get_entries()
            except:
                self.display_Error("Scan not saved")
                return
            f.write('Cube Axis Enable:'+str(self.cube_axis.enable.get())+'\n')
            f.write('Cube Axis Start:'+str(cs)+'\n')
            f.write('Cube Axis End:'+str(ce)+'\n')
            f.write('Cube Axis Number:'+str(Nc)+'\n')

            for i in range(len(self.extra_axes)):
                label = "Axis" + str(4+i)
                axis = self.extra_axes[i]
                f.write(label+':'+str(self.params_options[axis.param_value.get()])+'\n')
                try:
                    rs, re, Nr = axis.get_entries()
                except:
                    self.display_Error("Scan not saved")
                    return
                f.write(label+' Enable:'+str(axis.enable.get())+'\n')
                f.write(label+' Start:'+str(rs)+'\n')
                f.write(label+' End:'+str(re)+'\n')
                f.write(label+' Number:'+str(Nr)+'\n')

            f.flush()
            f.close()
        self.savescanMENU.entryconfigure(n, label=str(n+1)+': '+ttl)
        self.loadscanMENU.entryconfigure(n, label=str(n+1)+': '+ttl)
    # end save_scan

    def load_scan(self, n):
        '''
        Loads a scanning configuration from file
        '''
        flname = os.path.join(pm.SCANS_Save_dir,'save_'+str(n)+'.txt')
        if os.path.exists(flname):
            config = {}
            with open(flname, 'r') as f:
                try:
                    for l in list(f):
                        s = l.split(':')
                        config[s[0]] = s[1].rstrip()
                    f.close()
                except IndexError as e:
                    print(format_exc())
                    f.close()
                    return
            #

            for i in range(len(pm.SCAN_spatial_parameters)):
                lbl = pm.SCAN_spatial_lognames[i]
                if config[lbl] is not '*':
                    changeEntry(self.spatial_output_entries[pm.SCAN_spatial_parameters[i]], config[lbl])
            for i in range(len(pm.SCAN_voltage_parameters)):
                lbl = pm.SCAN_voltage_lognames[i]
                if config[lbl] is not '*':
                    changeEntry(self.voltage_output_entries[pm.SCAN_voltage_parameters[i]], config[lbl])
            #

            nx = int(float(config['nx']))
            ny = int(float(config['ny']))
            if nx != self.scanner.nx or ny != self.scanner.ny:
                self.change_img_size(nx, ny)
            lnrate = float(config['Line Rate'])
            self.scanner.set_line_rate(lnrate)
            self.linerateTEXT.set(str(self.scanner.linerate))

            if int(float(config['Voltage Lock Enable'])) == 1:
                self.voltage_lock.set(1)
                self.voltage_lock_callback()
            if int(float(config['Drift Correct Im0'])) == 1:
                self.drift_im0.set(1)
            if int(float(config['Drift Correct Im1'])) == 1:
                self.drift_im1.set(1)
            #

            self.fast_axis.param_select(self.scan_params[config['Fast Axis']][0])
            changeEntry(self.fast_axis.startENTRY, config['Fast Axis Start'])
            changeEntry(self.fast_axis.endENTRY, config['Fast Axis End'])

            self.slow_axis.param_select(self.scan_params[config['Slow Axis']][0])
            changeEntry(self.slow_axis.startENTRY, config['Slow Axis Start'])
            changeEntry(self.slow_axis.endENTRY, config['Slow Axis End'])

            if int(float(config['Cube Axis Enable'])) == 1:
                self.cube_axis.enable.set(1)
                self.cube_axis.param_select(self.scan_params[config['Cube Axis']][0])
                changeEntry(self.cube_axis.startENTRY, config['Cube Axis Start'])
                changeEntry(self.cube_axis.endENTRY, config['Cube Axis End'])
                changeEntry(self.cube_axis.numberENTRY, config['Cube Axis Number'])
            else:
                self.cube_axis.enable.set(0)

            for i in range(len(self.extra_axes)):
                label = "Axis" + str(4+i)
                axis = self.extra_axes[i]
                if int(float(config[label+' Enable'])) == 1:
                    axis.enable.set(1)
                    axis.param_select(self.scan_params[config[label]][0])
                    changeEntry(axis.startENTRY, config[label+' Start'])
                    changeEntry(axis.endENTRY, config[label+' End'])
                    changeEntry(axis.numberENTRY, config[label+' Number'])
                else:
                    axis.enable.set(0)
            #
        else:
            self.display_Error('Scan Configuration File Not Found')
    # end load_scan

    def autoscale_images_callback(self):
        '''
        Turns on auto-scaling of images
        '''
        self.autoscaleIm = True
        for i in range(self.num_plots):
            self.im[i].autoscale()
            self.canvases[i].draw()
    # end autoscale_images_callback

    def image_lims_callback(self, ix, label):
        '''
        Sets a maximum and minimum value for the photocurrent image
        '''
        spec = [
            ['Minimum', '', 0.0],
            ['Maximum', '', 1.0]
        ]
        l = self.popup_entry('Set ' + str(label) + ' Limits', spec)
        try:
            minV = float(l[0])
            maxV = float(l[1])
        except ValueError:
            self.display_Error("Given value is not a float")
            return
        self.autoscaleIm = False
        self.im[ix].set_clim(vmin=minV, vmax=maxV)
        self.canvases[ix].draw()
    # image_lims_callback

    def drift_update_center_values(self, dx, dy):
        '''
        Updates the center values of X and Y with drift correction, only works with the 'xaxis'
        and 'yaxis' parameters, will not work with voltage or hardware parameter
        '''
        try:
            X_ENTRY = self.spatial_output_entries['xaxis']
            Y_ENTRY = self.spatial_output_entries['yaxis']
            fo = float(self.X_ENTRY.get())
            so = float(self.Y_ENTRY.get())
            if dx != 0.0:
                X_ENTRY.delete(0,tk.END)
                X_ENTRY.insert(0, fo+dx)
            if dy != 0.0:
                Y_ENTRY.delete(0,tk.END)
                Y_ENTRY.insert(0, so+dy)
        except Exception as e:
            self.display_Error("drift correction, could not update center values")
            print(format_exc())
    # end drift_update_center_values
# end hyperDAQ
