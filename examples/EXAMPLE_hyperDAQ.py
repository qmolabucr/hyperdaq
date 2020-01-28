'''
calibration_CPS_hyperDAQ

Data Acquisition for the Cryomagnetic Probe Station

Last updated: January 2020 by Trevor Arp

Trevor Arp
University of California, Riverside
All Rights Reserved
'''

#### General Imports ####
import queue
import traceback
from os.path import exists

#### Import hyperDAQ  Modules and parameters ####
import parameters as pm
from hyperdaq.main import hyperDAQ
from hyperdaq.processing import data_image_2
from hyperdaq.utilities import Stopwatch
from hyperdaq.writing import data_scan_writer
from hyperdaq.card import card_control_acquire
import hyperdaq.thordevices as thordevices
import hyperdaq.devices_gui as devices_gui

'''
#############################
Core DAQ Card Initializations
#############################
'''
# Initialize peripheral timer, linked to system time
sys_time = Stopwatch()

# Initialize the Queues
message_q = queue.Queue(maxsize=200000)
# com_q = queue.Queue(maxsize=200000)

# Initialize DAQ card
print("Initializing NI DAQ Card")
scanner = card_control_acquire(pm.NUM_card_queues)
scanner.set_line_rate(2.0)
card_q = scanner.get_output_queues()

# Initialize the images
ref_image   = data_image_2(card_q[0], pm.REF_IMG_index, scanner, message_queue=message_q)
photo_image = data_image_2(card_q[1], pm.PCI_IMG_index, scanner, message_queue=message_q)
power_image = data_image_2(card_q[2], pm.POW_IMG_index, scanner, message_queue=message_q)
aux_images = {'powerimg':power_image}

# Initialize Data Writer
data_out = data_scan_writer([ref_image, photo_image, power_image], ['rfi','pci','pow'])

'''
#############################
Device Initilizations
#############################
'''
# Contains a dictionary of hardware contorllers, initilized and ready, to pass to the main interface
device_dict = dict()

# Contains a dictionary of interfaces for the hardware controllers, passed by reference so they can
# be initilized in the main interface, shares keys with device_dict
interface_dict = dict()

# Initialize the Delay Stage
print("Initializing Delay Stage")
try:
	device_dict['delay_stage'] = thordevices.DelayController(pm.SERIAL_delay, autohome=False) # FOR TESTING
	#device_dict['delay_stage'] = thordevices.DelayController(SERIAL_delay)
	interface_dict['delay_stage'] = devices_gui.thor_delay_stage
except Exception as e:
	print("Error could not load the Optical Delay Stage")
	print(str(e))
	print(traceback.format_exc())

'''
##########################################
Setup Scanning and Experimental Parameters
##########################################
'''
# Parameter Values for the axis
# "device_key":["Label", "unit", fast_function, slow_function, (default_start, default_end), (minimum, maximum)]
# Where fast_function and slow_function are the functions to call for scanning on the fast and slow axes, usually the same
scan_params = {
	'xaxis': ["X Axis ("+pm.CLABEL_x+")", pm.SCAN_units, None, None, (pm.DEFAULT_fastaxis_start, pm.DEFAULT_fastaxis_end), (pm.CARD_AO_Min, pm.CARD_AO_Max)],
	'yaxis': ["Y Axis ("+pm.CLABEL_y+")", pm.SCAN_units, None, None, (pm.DEFAULT_slowaxis_start, pm.DEFAULT_slowaxis_end), (pm.CARD_AO_Min, pm.CARD_AO_Max)],
	'vsd': ["Source/Drain ("+pm.CLABEL_vsd+")", "Volts", None, None, (0.0, 0.0), (pm.CARD_AO_Min, pm.CARD_AO_Max)],
	'vbg': ["Backgate ("+pm.CLABEL_vbg+")", "Volts", None, None, (0.0, 0.0), (pm.CARD_AO_Min, pm.CARD_AO_Max)],
	'delay_stage': ["Delay (ps)", "ps", "set_delay", "set_delay", (-25, 25), (pm.DELAY_min_ps, pm.DELAY_max_ps)]
}

# Experimental Parameters
exp_params = {
	"Chopper Setpoint":500,
	"Pre-Amp Timeconstant":0.3,
	"Pre-Amp Gain":1e-7,
	"Lock-In Timeconstant":1,
	"Lock-In Gain":1000
	}

exp_params_units = {
	"Chopper Setpoint":"Hz",
	"Pre-Amp Timeconstant":"ms",
	"Pre-Amp Gain":"A/V",
	"Lock-In Timeconstant":"ms",
	"Lock-In Gain":"mV",
	}

if exists(pm.PARAMS_file):
	ep = dict()
	for l in open(pm.PARAMS_file,'r'):
		l = l.split(":")
		ep[l[0]] = float(l[1])
	if len(ep) == len(exp_params):
		exp_params = ep

# Initilize the GUI
hyperDAQ(
	[ref_image, photo_image],
	aux_images,
	scanner,
	data_out,
	device_dict,
	interface_dict,
	scan_params,
	exp_params,
	exp_params_units
)
