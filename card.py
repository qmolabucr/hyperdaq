'''
card.py

Optimized hyperDAQ modules for talking to National Instruments Data Acquisition Card

Replacing nidaq.py and controls.py

Last Updated: January 2020

|  Trevor Arp
|  Gabor Lab
|  University of California, Riverside

All Rights Reserved
'''

import ctypes
import time
import queue
import numpy as np
import parameters as pm
from hyperdaq.utilities import Stopwatch
import warnings

import PyDAQmx as nidaq
from PyDAQmx.DAQmxTypes import TaskHandle

# NI_DAQmx typedefs and constants, correspond with values in
# C:\Program Files(x86)\National Instruments\NI-DAQ\DAQmx ANSI C Dev\include\NIDAQmx.h
int32 = ctypes.c_long
uInt32 = ctypes.c_ulong
uInt64 = ctypes.c_ulonglong
float64 = ctypes.c_double
pointsRead = int32()
DAQmx_device_name = "Dev1"
DAQmx_Val_Cfg_Default = int32(-1)
DAQmx_Val_Volts = 10348
DAQmx_Val_Rising = 10280
DAQmx_Val_FiniteSamps = 10178
DAQmx_Val_GroupByChannel = 0
DAQmx_Val_ChanForAllLines = 1
DAQmx_Val_RSE = 10083
DAQmx_Val_NRSE = 10078
DAQmx_Val_Diff =  10106
DAQmx_Val_ContSamps = 10123
DAQmx_Val_GroupByScanNumber = 1

def CAL(val, channel):
    '''
    Calibrate Voltage Channels
    '''
    mult, off = pm.CHANNEL_calibration[channel]
    return mult*float(val) + off
# end CAL

class card_control_acquire():
    '''
    card_control_acquire
    controls the NI DAQ card for the purposes of scanning measurements, with data acquisition

    num_queues is the number of Queues that the data is being written into
    multiple queues facilitates sending data into multiple places, for example a data
    writing queue, a data analysis queue and a display queue
    The function get_queues returns a list containing all the queues

    Data goes into queues as an array with the 1st column being time and the other
    columns containing voltage values of the channels

    $x_chan is the channel name for the x-axis (e.g. "ao0")
    $y_chan is the channel name for the y-axis (e.g. "ao1")
    $V_s_chan is the channel name for the source (e.g. "ao2")
    $V_bg_chan is the channel name for the backgate (e.g. "ao3")
    at the beginning of a scan it synchronizes with the Scanner's own stopwatch

    Will scan at the rate defined by RATE_line_scan unless otherwise specified using the
    set_line_rate function

    Callable Attributes:
    $nx is the number of x points in the image, by default NUM_x_points
    $ny is the number of y points in the image, by default NUM_y_points
    $linerate is the number of lines per second, when scanning
    '''

    def __init__(self, num_queues):
        self.numChannels = pm.NUM_input_channels
        self.AOtask = TaskHandle(0)
        self.AItask = TaskHandle(1)
        self.chanX = pm.CHANNEL_x
        self.chanY = pm.CHANNEL_y
        self.chanS = pm.CHANNEL_vsd
        self.chanB = pm.CHANNEL_vbg
        self.chanINPUT = pm.CHANNEL_input
        self.triggerName = "ai/StartTrigger"
        self.clockSource = b'OnboardClock'
        self.AISamplesPerChan = uInt64(2000)
        self.sampleFreq = pm.MASTER_sample_freq
        self.sampleRate = float64(self.sampleFreq)

        self.AImin = float64(pm.CARD_AI_Min)
        self.AImax = float64(pm.CARD_AI_Max)

        self.nx = pm.NUM_x_points
        self.ny = pm.NUM_y_points
        self.scanning = False
        self.abortscan = False
        self.timer = Stopwatch()

        # Set the percentage of a line to go to the next point
        self.shift = 0.15

        # The amount of time to stay on the next point to allow the system to stabalize
        self.stay = 0.1

        # The tap of times corresponding to particular data points
        self.time_map = []
        self.num_queues = num_queues
        for i in range(num_queues):
            self.time_map.append(queue.Queue(maxsize=200000))

        self.dataQueue =[]
        for i in range(num_queues):
            self.dataQueue.append(queue.Queue(maxsize=200000))

        # Voltage Channels slow by default
        self.sd_slow = True
        self.bg_slow = True

        # Set the default values for the voltage outputs
        self.vS_default = CAL(0.0, 'vsd')
        self.vB_default = CAL(0.0, 'vbg')

        # Current Values of the channels
        self.vS = 0.0
        self.vB = 0.0
        self.vX = 0.0
        self.vY = 0.0

        # Create Input task and define input channels
        nidaq.DAQmxCreateTask("", ctypes.byref(self.AItask))
        nidaq.DAQmxCreateAIVoltageChan(self.AItask, self.chanINPUT, "", DAQmx_Val_RSE, self.AImin, self.AImax, DAQmx_Val_Volts, None)

        # Create Output task and define output channels. Warning: the order matters, do not change order
        nidaq.DAQmxCreateTask("", ctypes.byref(self.AOtask))
        nidaq.DAQmxCreateAOVoltageChan(self.AOtask, self.chanX, "", pm.CARD_AO_Min, pm.CARD_AO_Max, DAQmx_Val_Volts, None)
        nidaq.DAQmxCreateAOVoltageChan(self.AOtask, self.chanY, "", pm.CARD_AO_Min, pm.CARD_AO_Max, DAQmx_Val_Volts, None)
        nidaq.DAQmxCreateAOVoltageChan(self.AOtask, self.chanS, "", pm.CARD_AO_Min, pm.CARD_AO_Max, DAQmx_Val_Volts, None)
        nidaq.DAQmxCreateAOVoltageChan(self.AOtask, self.chanB, "", pm.CARD_AO_Min, pm.CARD_AO_Max, DAQmx_Val_Volts, None)

        self.set_line_rate(pm.RATE_line_scan)

        # Zero channels Initially
        self.zero_all()
    # end __init__

    def get_queues(self):
        '''
        Returns point Queues for processing
        '''
        return self.time_map
    #

    def get_output_queues(self):
        '''
        Returns a list containing all the Queues that the data is being written into
        '''
        return self.dataQueue
	# end get_queues

    def clear_queues(self):
        '''
        Empties the data Queues
        '''
        for q in self.time_map:
            while not q.empty():
                q.get()
    # end clear_queues

    def set_nxy(self, nx, ny):
        '''
        Set the size of the data image
        '''
        if not self.scanning:
            self.nx = int(nx)
            self.ny = int(ny)
        else:
            print("Error card.card_control: nx and ny cannot be changed while scanning")
    # end set_nxy

    def set_line_rate(self, rate):
        '''
        Sets the line rate
        '''
        if float(rate) <= pm.RATE_line_scan:
            self.linerate = float(rate)

            # Fix Memory error for slow scans
            if self.linerate <= 0.02:
                self.sampleFreq = pm.MASTER_sample_freq/100.0
                self.sampleRate = float64(self.sampleFreq)
            elif self.linerate > 0.02 and self.sampleFreq != pm.MASTER_sample_freq:
                self.sampleFreq = pm.MASTER_sample_freq
                self.sampleRate = float64(self.sampleFreq)

            self.points = int(self.sampleFreq/self.linerate)
            self.samplesPerChan = uInt64(self.points)
            self.xix1 = 0
            self.xix2 = self.xix1 + self.points
            self.yix1 = self.xix2
            self.yix2 = self.yix1 + self.points
            self.six1 = self.yix2
            self.six2 = self.six1 + self.points
            self.bix1 = self.six2
            self.bix2 = self.bix1 + self.points
            nidaq.DAQmxCfgSampClkTiming(self.AOtask, self.clockSource, float64(self.sampleFreq), DAQmx_Val_Rising, DAQmx_Val_FiniteSamps, self.samplesPerChan)
        else:
            print("Error card.card_control : Cannot set line rate faster than parameter RATE_line_scan=" + str(pm.RATE_line_scan))
	# end set_line_rate

    def ramp_up(self, X, Y, Vsd, Vbg, t=-1):
        '''
        Ramps from the current value to the desired value for each channel, in the time defined by
        the parameter $t which if set to -1 is the default CNTL_default_ramp_time
        '''
        if t == -1:
            t = pm.CNTL_default_ramp_time
        #
        N = int(self.sampleFreq*t) #self.points
        data = np.zeros(N*4)
        xix1 = 0
        xix2 = xix1 + N
        yix1 = xix2
        yix2 = yix1 + N
        six1 = yix2
        six2 = six1 + N
        bix1 = six2
        bix2 = bix1 + N
        data[xix1:xix2] = np.linspace(self.vX, X, N, dtype=np.float64)
        data[yix1:yix2] = np.linspace(self.vY, Y, N, dtype=np.float64)
        data[six1:six2] = np.linspace(self.vS, Vsd, N, dtype=np.float64)
        data[bix1:bix2] = np.linspace(self.vB, Vbg, N, dtype=np.float64)
        nidaq.DAQmxCfgSampClkTiming(self.AOtask, self.clockSource, float64(self.sampleFreq), DAQmx_Val_Rising, DAQmx_Val_FiniteSamps, uInt64(N))
        nidaq.DAQmxWriteAnalogF64(self.AOtask, N, False, -1, DAQmx_Val_GroupByChannel, data, None, None)
        nidaq.DAQmxDisableStartTrig(self.AOtask)
        nidaq.DAQmxStartTask(self.AOtask)
        nidaq.DAQmxWaitUntilTaskDone(self.AOtask, float64(-1))
        nidaq.DAQmxStopTask(self.AOtask)
        self.vX = X
        self.vY = Y
        self.vS = Vsd
        self.vB = Vbg
    # end ramp_up

    def set_default_voltage(self, chan, v):
        '''
        Sets and ramps to the default value of source/drain or backgate
        '''
        if chan == 'vsd':
            self.vS_default = CAL(v, 'vsd')
            self.ramp_up(self.vX, self.vY, self.vS_default, self.vB)
        elif chan == 'vbg':
            self.vB_default = CAL(v, 'vbg')
            self.ramp_up(self.vX, self.vY, self.vS, self.vB_default)
        else:
            raise ValueError("Invalid Voltage Channel")
    #

    def reset_input(self):
        '''
        Re-sets the input channels, clearing out any registered functions, allocated resources,
        etc. Should be called after scanning functions are finished or aborted.
        '''
        nidaq.DAQmxClearTask(self.AItask)
        nidaq.DAQmxCreateTask("", ctypes.byref(self.AItask))
        nidaq.DAQmxCreateAIVoltageChan(self.AItask, self.chanINPUT, "", DAQmx_Val_RSE, self.AImin, self.AImax, DAQmx_Val_Volts, None)
    # end reset_input

    def pollcard(self, N=2000):
        '''
        Grabs the current values of the inputs and sends it off for processing
        '''

        nidaq.DAQmxCfgSampClkTiming(self.AItask, self.clockSource, float64(pm.MASTER_sample_freq), DAQmx_Val_Rising, DAQmx_Val_FiniteSamps, self.AISamplesPerChan)
        data = np.zeros((N, self.numChannels), dtype=np.float64)
        nidaq.DAQmxReadAnalogF64(self.AItask, N, 1.0, DAQmx_Val_GroupByScanNumber, data, data.size, ctypes.byref(pointsRead),None)

        # Explicit loop becuase DAQmxWaitUntilTaskDone likes to throw hissy fits
        isdone = uInt32()
        nidaq.DAQmxIsTaskDone(self.AItask, ctypes.byref(isdone))
        loopcnt = 0
        while not isdone:
            if loopcnt > 10:
                print("Error pollcard: Input Timeout on line ")
                try:
                    nidaq.DAQmxStopTask(self.AItask)
                except:
                    print("Error stopping task")
                break
            time.sleep(0.25*N/pm.MASTER_sample_freq)
            loopcnt += 1
            nidaq.DAQmxIsTaskDone(self.AItask, ctypes.byref(isdone))
        nidaq.DAQmxStopTask(self.AItask)
        for q in self.dataQueue:
            q.put([-1, -1, data])
    # end pollcard

    def scan(self, type, outputs, fastchannel, **kwargs):
        '''
        Wrapper function for all the types of scans implemented, based on type will pass
        arguments to the appropriate scanning function.

        Implemented scan types:
        - "rectilinear-4output": A basic rectilinear scan based on a card with 4 voltage outputs designated (xaxis, yaxis, vsd, vbg)

        fastchannel sets which output channel is the fast channel, if it doesn't match specific cases it will be the x-axis

        Included for extensibility, additional types of scans may become availible in the future.
        '''
        if type == "rectilinear-4output":
            if 'fast_func' in kwargs: # Implement fast function scanning in future
                return self.fixed_output_line_scan(outputs['xaxis'], outputs['yaxis'], outputs['vsd'], outputs['vbg'], outputs['scanangle'][0], **kwargs)
            else:
                if fastchannel == 'vsd':
                    self.sd_slow = False
                elif fastchannel == 'vbg':
                    self.bg_slow = False
                r = self.card_output_waveform_scan(outputs['xaxis'], outputs['yaxis'], outputs['vsd'], outputs['vbg'], outputs['scanangle'][0], **kwargs)
                self.sd_slow = True
                self.bg_slow = True
                return r
        else:
            raise ValueError("Unknown Scan Type")
    # end scan

    def card_output_waveform_scan(self, x_range, y_range, vsd_range, vbg_range, theta, func_range=(0,0), func=None, after_vbg=False, after_vsd=False):
        '''
        Scans continuously along a fast axis and in discrete steps along an orthogonal slow axis,
        outputting a scanning waveform along the fast axis.

        $xrange and $yrange are tuples containing (vstart, vend) where the scan is from vstart to
        vend, $theta the scan angle along the fast and slow axes respectively, if vstart = vend
        there will be no scan along that axis, instead that axis will stay at a constant value
        given by vstart=vend. Outputs will be zeroed at the end of the scan, unless default value
        is set

        All input parameter voltages will be calibrated such that they give the correct output
        value at the physical output, calibration paramters are defined for each channel in the
        parameters file.

        Also scans the input function $func along the slow axis in range $funcrange.
        If $funcrange is a tuple of length 2 or a numpy array then the input function should be
        callable with a float value that changes some parameter, i.e. func(x).

        $after_vbg if True the backgate will stay constant at the ending value after the scan is
        sucessfully Completed, if False it will go to the default value

        $after_vsd if True the backgate will stay constant at the ending value after the scan is
        sucessfully Completed, if False it will go to the default value
        '''
        if isinstance(x_range, tuple) and isinstance(y_range, tuple) and isinstance(vsd_range, tuple) and isinstance(vbg_range, tuple):
            # Calibrate the voltages
            x_range = (CAL(x_range[0]*pm.SCAN_units_to_volts, 'xaxis') , CAL(x_range[1]*pm.SCAN_units_to_volts, 'xaxis'))
            y_range = (CAL(y_range[0]*pm.SCAN_units_to_volts, 'yaxis') , CAL(y_range[1]*pm.SCAN_units_to_volts, 'yaxis'))
            vsd_range = (CAL(vsd_range[0], 'vsd') , CAL(vsd_range[1], 'vsd'))
            vbg_range = (CAL(vbg_range[0], 'vbg') , CAL(vbg_range[1], 'vbg'))

            x0 = np.mean(x_range)
            y0 = np.mean(y_range)
        else:
            print("Error card.card_control: parameters of card_output_waveform_scan() must be tuples")
            return -1
        #

        # Scan the function, if there is one
        if func is None:
            slow_func = False
        elif isinstance(func_range, tuple):
            if len(func_range) == 2:
                slow_range = np.linspace(func_range[0], func_range[1], self.ny)
                slow_func = True
            else:
                print("Error card.card_control: Given function range is not a valid length")
                return -1
        elif isinstance(func_range, np.ndarray):
            if np.size(func_range) == self.ny:
                slow_range = func_range
                slow_func = True
            else:
                print("Error card.card_control: Given function range is not equal to size of y-axis")
                return -1
        else:
            print("Error card.card_control: Given function range is not a valid type")
            return -1

        # Establish the setpoints for the four channels
        x_pts = np.zeros((self.ny, 2))
        y_pts = np.zeros((self.ny, 2))
        sd_pts = np.zeros((self.ny, 2))
        bg_pts = np.zeros((self.ny, 2))
        yrange = np.linspace(y_range[0]-y0, y_range[1]-y0, self.ny)

        if self.sd_slow:
            sd_range = np.linspace(vsd_range[0], vsd_range[1], self.ny)
        else:
            sd_range = vsd_range

        if self.bg_slow:
            bg_range = np.linspace(vbg_range[0], vbg_range[1], self.ny)
        else:
            bg_range = vbg_range

        for i in range(self.ny):
            x_pts[i,0] = (x_range[0]-x0)*np.cos(theta*np.pi/180) + (yrange[i])*np.sin(theta*np.pi/180) + x0
            x_pts[i,1] = (x_range[1]-x0)*np.cos(theta*np.pi/180) + (yrange[i])*np.sin(theta*np.pi/180) + x0
            y_pts[i,0] = (yrange[i])*np.cos(theta*np.pi/180) - (x_range[0]-x0)*np.sin(theta*np.pi/180) + y0
            y_pts[i,1] = (yrange[i])*np.cos(theta*np.pi/180) - (x_range[1]-x0)*np.sin(theta*np.pi/180) + y0
            if self.sd_slow:
                sd_pts[i,0] = sd_range[i]
                sd_pts[i,1] = sd_range[i]
            else:
                sd_pts[i,0] = vsd_range[0]
                sd_pts[i,1] = vsd_range[1]
            if self.bg_slow:
                bg_pts[i,0] = bg_range[i]
                bg_pts[i,1] = bg_range[i]
            else:
                bg_pts[i,0] = vbg_range[0]
                bg_pts[i,1] = vbg_range[1]

        # Set times
        time_per_line = 1.0/self.linerate
        time_per_point = ((1.0-self.shift-self.stay)*time_per_line)/float(self.nx)
        samp_per_pt = int(time_per_point*self.sampleFreq)

        # Sync the data aquisition, read the processing
        time.sleep(0.05)
        self.scanning = True
        self.clear_queues()

        time.sleep(pm.IMG_delay_time) # Give the images a chance to catch up

        # Set image parameters to 0
        self.line_over = False
        self.pix_count = 0
        self.line_N = 0

        # Define callcack function to acquire data for a data point and Queue it for processing
        def data_callback(taskHandle, eventType, N, altargs):
            self.line_N += 1
            if self.line_N > self.nx:
                return
            data = np.zeros((N, self.numChannels), dtype=np.float64)
            nidaq.DAQmxReadAnalogF64(taskHandle, N, float64(1.0), DAQmx_Val_GroupByScanNumber,data, data.size,ctypes.byref(pointsRead),None)
            ixi = self.pix_count // self.nx
            ixj = self.pix_count % self.nx
            self.pix_count += 1
            for q in self.dataQueue:
                q.put([ixj, ixi, data])
            if self.line_N == self.nx:
                nidaq.DAQmxStopTask(self.AItask)
            #
            return 0
        # end
        callback_reference = nidaq.DAQmxEveryNSamplesEventCallbackPtr(data_callback)

        # Ramp up to the starting values
        self.ramp_up(x_pts[0,0], y_pts[0,0], sd_pts[0,0], bg_pts[0,0])

        time.sleep(pm.CNTL_scan_wait)

        N = int(self.points)
        data = np.zeros(4*N)
        xix1 = self.xix1
        xix2 = self.xix2
        yix1 = self.yix1
        yix2 = self.yix2
        six1 = self.six1
        six2 = self.six2
        bix1 = self.bix1
        bix2 = self.bix2

        # For efficiency, pre-compute the waveforms
        waveform = np.zeros((self.ny, 4*N))
        for i in range(self.ny):
            if i < self.ny -1:
                waveform[i, xix1:xix2] = self.compute_waveform(x_pts[i,0], x_pts[i,1], x_pts[i+1,0], N)
                waveform[i, yix1:yix2] = self.compute_waveform(y_pts[i,0], y_pts[i,1], y_pts[i+1,0], N)
                waveform[i, six1:six2] = self.compute_waveform(sd_pts[i,0], sd_pts[i,1], sd_pts[i+1,0], N)
                waveform[i, bix1:bix2] = self.compute_waveform(bg_pts[i,0], bg_pts[i,1], bg_pts[i+1,0], N)
            else:
                waveform[i, xix1:xix2] = self.compute_waveform(x_pts[i,0], x_pts[i,1], x_pts[i,1], N)
                waveform[i, yix1:yix2] = self.compute_waveform(y_pts[i,0], y_pts[i,1], y_pts[i,1], N)
                waveform[i, six1:six2] = self.compute_waveform(sd_pts[i,0], sd_pts[i,1], sd_pts[i,1], N)
                waveform[i, bix1:bix2] = self.compute_waveform(bg_pts[i,0], bg_pts[i,1], bg_pts[i,1], N)
        #

        # Configure Timing
        nidaq.DAQmxCfgSampClkTiming(self.AOtask, self.clockSource, float64(self.sampleFreq), DAQmx_Val_Rising, DAQmx_Val_FiniteSamps, self.samplesPerChan)
        nidaq.DAQmxCfgSampClkTiming(self.AItask, self.clockSource, self.sampleRate, DAQmx_Val_Rising, DAQmx_Val_ContSamps, self.AISamplesPerChan)

        nidaq.DAQmxRegisterEveryNSamplesEvent(self.AItask, 1, samp_per_pt, 0, callback_reference, None)

        t_scan_start = self.timer.time()
        while self.scanning:
            for i in range(self.ny):
                if self.abortscan:
                    self.scanning = False
                    self.abortscan = False
                    self.reset_input()
                    self.ramp_up(CAL(0.0, 'xaxis'), CAL(0.0, 'yaxis'), self.vS_default, self.vB_default)
                    return -1
                self.line_over = False

                # Load waveforms into task
                data[xix1:xix2] = waveform[i, xix1:xix2]
                data[yix1:yix2] = waveform[i, yix1:yix2]
                data[six1:six2] = waveform[i, six1:six2]
                data[bix1:bix2] = waveform[i, bix1:bix2]
                self.vX = data[xix2-1]
                self.vY = data[yix2-1]
                self.vS = data[six2-1]
                self.vB = data[bix2-1]
                #

                if slow_func:
                    func(slow_range[i])
                    time.sleep(pm.CNTL_function_wait)

                self.line_N = 0

                # Configure Analog Output
                nidaq.DAQmxWriteAnalogF64(self.AOtask, N, False, -1, DAQmx_Val_GroupByChannel, data, None, None)
                nidaq.DAQmxCfgDigEdgeStartTrig(self.AOtask, self.triggerName, DAQmx_Val_Rising)

                # Start the task
                nidaq.DAQmxStartTask(self.AOtask)
                nidaq.DAQmxStartTask(self.AItask)

                # Wait till input processing is finished
                # Use an explicit loop because DAQmxWaitUntilTaskDone has problems with AItask
                isdone = uInt32()
                nidaq.DAQmxIsTaskDone(self.AItask, ctypes.byref(isdone))
                loopcnt = 0
                while not isdone:
                    if self.abortscan:
                        try:
                            with warnings.catch_warnings(): # to catch a StoppedBeforeDoneWarning on abort
                                warnings.simplefilter("ignore")
                                nidaq.DAQmxStopTask(self.AOtask)
                                nidaq.DAQmxStopTask(self.AItask)
                        except Exception as e:
                            print(e)
                            print("Error stopping tasks")
                        break
                    if loopcnt > self.nx:
                        print("Error finite_scan: Input Timeout on line " + str(i))
                        try:
                            nidaq.DAQmxStopTask(self.AItask)
                        except Exception as e:
                            print(e)
                            print("Error stopping task")
                        break
                    time.sleep(time_per_point)
                    loopcnt += 1
                    nidaq.DAQmxIsTaskDone(self.AItask, ctypes.byref(isdone))
                # end while
                with warnings.catch_warnings(): # to catch a StoppedBeforeDoneWarning on abort
                    warnings.simplefilter("ignore")
                    nidaq.DAQmxWaitUntilTaskDone(self.AOtask, float64(2*time_per_line))
                    nidaq.DAQmxStopTask(self.AOtask)
            # end for
            nidaq.DAQmxDisableStartTrig(self.AOtask)
            self.reset_input()
            if after_vbg:
                end_Vb = vbg_range[1]
            else:
                end_Vb =  self.vB_default

            if after_vsd:
                end_Vs = vsd_range[1]
            else:
                end_Vs =  self.vS_default

            self.ramp_up(CAL(0.0, 'xaxis'), CAL(0.0, 'yaxis'), end_Vs, end_Vb)
            self.scanning = False
            #
        # end while
        t_scan_end = self.timer.time()
        dt = t_scan_end - t_scan_start
        print("Scan Finished in: " + str(round(dt,3)) + " seconds")
        return 1
    # end card_output_waveform_scan

    def fixed_output_line_scan(self, x_range, y_range, vsd_range, vbg_range, theta, fast_func=None, fast_func_range=(0,0), func=None, func_range=(0,0), after_vbg=False, after_vsd=False):
        '''
        Scans in discrete steps along the fast and slow axes, with all the card outputs (space,
        voltage) constant on each line (i.e. only being slow variables). fast_func will be called
        as the fast axis, if None it will throw an error.

        $xrange and $yrange are tuples containing (vstart, vend) where the scan is from vstart to
        vend, $theta the scan angle along the fast and slow axes respectively, if vstart = vend
        there will be no scan along that axis, instead that axis will stay at a constant value
        given by vstart=vend. Outputs will be zeroed at the end of the scan, unless default value
        is set

        All input parameter voltages will be calibrated such that they give the correct output
        value at the physical output, calibration paramters are defined for each channel in the
        parameters file.

        Also scans the input function $func along the slow axis in range $funcrange.
        If $funcrange is a tuple of length 2 or a numpy array then the input function should be
        callable with a float value that changes some parameter, i.e. func(x).
        If $funcrange is a tuple of length 4 then the input function should be callable with three
        float inputs, two of which are constant and the third that changes some parameter, i.e. func(funcrange[2], funcrange[3], x).

        $after_vbg if True the backgate will stay constant at the ending value after the scan is
        sucessfully Completed, if False it will go to the default value

        $after_vsd if True the backgate will stay constant at the ending value after the scan is
        sucessfully Completed, if False it will go to the default value
        '''
        if isinstance(x_range, tuple) and isinstance(y_range, tuple) and isinstance(vsd_range, tuple) and isinstance(vbg_range, tuple):
            # Calibrate the voltages
            x_range = (CAL(x_range[0]*pm.SCAN_units_to_volts, 'xaxis') , CAL(x_range[1]*pm.SCAN_units_to_volts, 'xaxis'))
            y_range = (CAL(y_range[0]*pm.SCAN_units_to_volts, 'yaxis') , CAL(y_range[1]*pm.SCAN_units_to_volts, 'yaxis'))
            vsd_range = (CAL(vsd_range[0], 'vsd') , CAL(vsd_range[1], 'vsd'))
            vbg_range = (CAL(vbg_range[0], 'vbg') , CAL(vbg_range[1], 'vbg'))

            x0 = np.mean(x_range)
            y0 = np.mean(y_range)
        else:
            print("Error card.card_control: parameters of fixed_ouput_line_scan() must be tuples")
            return -1
        #

        if fast_func is None:
            raise ValueError("Error fixed_output_line_scan: A fast scanning function must be specified")
        else:
            fast_range = np.linspace(fast_func_range[0], fast_func_range[1], self.nx)

        # Scan the function, if there is one
        if func is None:
            slow_func = False
        elif isinstance(func_range, tuple):
            if len(func_range) == 2:
                slow_range = np.linspace(func_range[0], func_range[1], self.ny)
                slow_func = True
            else:
                print("Error card.card_control: Given function range is not a valid length")
                return -1
        elif isinstance(func_range, np.ndarray):
            if np.size(func_range) == self.ny:
                slow_range = func_range
                slow_func = True
            else:
                print("Error card.card_control: Given function range is not equal to size of y-axis")
                return -1
        else:
            print("Error card.card_control: Given function range is not a valid type")
            return -1

        # Establish the setpoints for the four channels
        x_pts = np.zeros((self.ny, 2))
        y_pts = np.zeros((self.ny, 2))
        sd_pts = np.zeros((self.ny, 2))
        bg_pts = np.zeros((self.ny, 2))
        yrange = np.linspace(y_range[0]-y0, y_range[1]-y0, self.ny)

        if self.sd_slow:
            sd_range = np.linspace(vsd_range[0], vsd_range[1], self.ny)
        else:
            sd_range = vsd_range

        if self.bg_slow:
            bg_range = np.linspace(vbg_range[0], vbg_range[1], self.ny)
        else:
            bg_range = vbg_range

        for i in range(self.ny):
            x_pts[i,0] = (x_range[0]-x0)*np.cos(theta*np.pi/180) + (yrange[i])*np.sin(theta*np.pi/180) + x0
            x_pts[i,1] = (x_range[1]-x0)*np.cos(theta*np.pi/180) + (yrange[i])*np.sin(theta*np.pi/180) + x0
            y_pts[i,0] = (yrange[i])*np.cos(theta*np.pi/180) - (x_range[0]-x0)*np.sin(theta*np.pi/180) + y0
            y_pts[i,1] = (yrange[i])*np.cos(theta*np.pi/180) - (x_range[1]-x0)*np.sin(theta*np.pi/180) + y0
            if self.sd_slow:
                sd_pts[i,0] = sd_range[i]
                sd_pts[i,1] = sd_range[i]
            else:
                sd_pts[i,0] = vsd_range[0]
                sd_pts[i,1] = vsd_range[1]
            if self.bg_slow:
                bg_pts[i,0] = bg_range[i]
                bg_pts[i,1] = bg_range[i]
            else:
                bg_pts[i,0] = vbg_range[0]
                bg_pts[i,1] = vbg_range[1]

        # Set times
        time_per_line = 1.0/self.linerate
        time_per_point = ((1.0-self.shift-self.stay)*time_per_line)/float(self.nx)
        samp_per_pt = int(time_per_point*self.sampleFreq)
        ramp_time = min([time_per_point, pm.CNTL_function_wait]) # Ramp to switch values, but not for longer than you would wait normally
        if time_per_line < self.nx*pm.CNTL_fast_function_wait:
            print("Warning: low time per line may result in timeouts or unexpected behavior")

        # Sync the data aquisition, read the processing
        time.sleep(0.05)
        self.scanning = True
        self.clear_queues()

        time.sleep(pm.IMG_delay_time) # Give the images a chance to catch up

        # Set image parameters to 0
        self.pix_count = 0

        # Define callcack function to acquire data for a data point and Queue it for processing
        def data_callback(taskHandle, eventType, N, altargs):
            data = np.zeros((N, self.numChannels), dtype=np.float64)
            nidaq.DAQmxReadAnalogF64(taskHandle, N, float64(1.0), DAQmx_Val_GroupByScanNumber,data, data.size,ctypes.byref(pointsRead),None)
            ixi = self.pix_count // self.nx
            ixj = self.pix_count % self.nx
            self.pix_count += 1
            for q in self.dataQueue:
                q.put([ixj, ixi, data])
            #
            return 0
        # end

        # Ramp up to the starting values
        self.ramp_up(x_pts[0,0], y_pts[0,0], sd_pts[0,0], bg_pts[0,0])

        time.sleep(pm.CNTL_scan_wait)

        # Configure Timing
        nidaq.DAQmxCfgSampClkTiming(self.AItask, self.clockSource, self.sampleRate, DAQmx_Val_Rising, DAQmx_Val_ContSamps, self.AISamplesPerChan)

        t_scan_start = self.timer.time()
        while self.scanning:
            for i in range(self.ny):
                if self.abortscan:
                    self.scanning = False
                    self.abortscan = False
                    self.reset_input()
                    self.ramp_up(CAL(0.0, 'xaxis'), CAL(0.0, 'yaxis'), self.vS_default, self.vB_default)
                    return -1
                #

                # Ramp up to the values for the line
                self.ramp_up(x_pts[i,0], y_pts[i,0], sd_pts[i,0], bg_pts[i,0], t=ramp_time)

                if slow_func:
                    func(slow_range[i])
                    time.sleep(pm.CNTL_function_wait)

                # Go to the first point in the fast function range
                fast_func(fast_range[0])
                time.sleep(pm.CNTL_fast_function_wait)

                # Loop through fast axis
                for j in range(self.nx):
                    if self.abortscan:
                        try:
                            with warnings.catch_warnings(): # to catch a StoppedBeforeDoneWarning on abort
                                warnings.simplefilter("ignore")
                                nidaq.DAQmxStopTask(self.AItask)
                        except Exception as e:
                            print(e)
                            print("Error stopping tasks")
                        break
                    # end if

                    fast_func(fast_range[j])
                    time.sleep(pm.CNTL_fast_function_wait)

                    nidaq.DAQmxStartTask(self.AItask)
                    data_callback(self.AItask, 1, samp_per_pt, None)
                    isdone = uInt32()
                    nidaq.DAQmxIsTaskDone(self.AItask, ctypes.byref(isdone))
                    nidaq.DAQmxStopTask(self.AItask)
                # end for
            # end for
            self.reset_input()
            if after_vbg:
                end_Vb = vbg_range[1]
            else:
                end_Vb =  self.vB_default

            if after_vsd:
                end_Vs = vsd_range[1]
            else:
                end_Vs =  self.vS_default

            self.ramp_up(CAL(0.0, 'xaxis'), CAL(0.0, 'yaxis'), end_Vs, end_Vb)
            self.scanning = False
            #
        # end while
        t_scan_end = self.timer.time()
        dt = t_scan_end - t_scan_start
        print("Scan Finished in: " + str(round(dt,3)) + " seconds")
        return 1
    # end fixed_ouput_line_scan

    def compute_waveform(self, startV, stopV, nextV, N):
        '''
        Computes the waveform needed to scan over 80 percent of the waveform, ramping back
        to the next point over the remaining 20 percent of the waveform

        where $startV and $stopV are the points to ramp between and $nextV is the point to ramp
        down to after reaching $stopV, all must be floats

        $N is the number of points to do it for, must be an integer
        '''
        d = np.zeros(N)
        turn = int((1.0-self.shift-self.stay)*N)
        stay = turn + int(self.shift*N)
        d[0:turn+1] = np.linspace(startV, stopV, turn+1, dtype=np.float64)
        d[turn:stay+1] = np.linspace(stopV, nextV, stay-turn+1, dtype=np.float64)
        d[stay:N] = np.linspace(nextV, nextV, N-stay, dtype=np.float64)
        return d
    # end compute_waveform

    def est_scan_time(self):
        '''
        Estimates the number of seconds needed to scan
        '''
        return 1.0 * self.ny / self.linerate
    # end est_scan_time

    def zero_all(self):
        '''
        Zeros all voltage channels
        '''
        self.ramp_up(CAL(0, 'xaxis'), CAL(0, 'yaxis'), CAL(0, 'vsd'), CAL(0, 'vbg'), t=pm.CNTL_default_ramp_time/4)
    # end zero_all

    def stop(self):
        '''
        Closes the voltage ouput channels, sets them to zero
        '''
        self.zero_all()
        nidaq.DAQmxStopTask(self.AOtask)
        nidaq.DAQmxClearTask(self.AOtask)
        nidaq.DAQmxStopTask(self.AItask)
        nidaq.DAQmxClearTask(self.AItask)
    # end stop
# end card_control_acquire
