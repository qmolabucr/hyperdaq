'''
EXAMPLE_hyperDAQ

Data Acquisition for the Cryomagnetic Probe Station

Last updated: January 2020 by Trevor Arp

Trevor Arp
University of California, Riverside
All Rights Reserved
'''

#### General Imports ####
import traceback
from os.path import exists

#### Import hyperDAQ parameters ####
import parameters as pm

#### Import hyperDAQ objects ####
from hyperdaq.main import hyperDAQ
from hyperdaq.processing import data_image_2
from hyperdaq.utilities import Stopwatch
from hyperdaq.writing import data_scan_writer
from hyperdaq.card import card_control_acquire

#### Import hyperDAQ Hardware Interfaces and Controllers ####
# Warning: Import specific objects from modules, importing all may causes errors if you
# do not have all necessary parameters in your parameters file
from hyperdaq.devices_gui import thor_delay_stage, lakeshore_336_temperature

from hyperdaq.thordevices import DelayController
from hyperdaq.serialcom import lakeshore_336

'''
#############################
Core DAQ Card Initializations
#############################
'''
sys_time = Stopwatch() # Initialize peripheral timer, keeps time between the various components

# Initialize DAQ card
print("Initializing DAQ Card")
scanner = card_control_acquire(pm.NUM_card_queues)
card_q = scanner.get_output_queues() # Queues of data to be passed out of the card control for processing

# Initialize the main images
ref_image   = data_image_2(card_q[0], pm.RFI_IMG_index, scanner)
photo_image = data_image_2(card_q[1], pm.PCI_IMG_index, scanner)

## Example of how to add an auxiliary image, for example an image of lser power. Note that
## pm.NUM_card_queues needs to reflect the total number of images, if it is larger than the
## actual number of data image objects it will cause a memory leak.
power_image = data_image_2(card_q[2], pm.POW_IMG_index, scanner)
aux_images = {'powerimg':power_image}

# If there are no auxiliary images, initialize an empty
# aux_images = dict()

# Initialize Data Writer, give it the images and the extensions for all the images
data_out = data_scan_writer([ref_image, photo_image, power_image], ['rfi','pci','pow']) # Include auxillary images, if they exist
# data_out = data_scan_writer([ref_image, photo_image], ['rfi','pci']) # If there are no auxillary images

'''
#############################
Device Initilizations
#############################
'''
# Contains a dictionary of hardware controllers, to pass to the main interface
device_dict = dict()

# Contains a dictionary of interfaces for the hardware controllers, passed by reference so they can
# be initilized in the main interface, shares keys with device_dict
interface_dict = dict()

print("Initilizing External Devices")

## An example of how to add a Thor Labs device, in this case a delay stage controller
try:
	device_dict['delay_stage'] = DelayController(pm.SERIAL_delay)
	interface_dict['delay_stage'] = thor_delay_stage
except Exception as e:
	print("Error could not load the Optical Delay Stage")
	print(traceback.format_exc())

## An example of how to add a serial device, in this case a Lakeshore temeprature controller
try:
	device_dict['temp_control'] = lakeshore_336(pm.COM_tempcontrol, sys_time)
	interface_dict['temp_control'] = lakeshore_336_temperature
except Exception as e:
	print("Error could not load the Temperature Controller")
	print(traceback.format_exc())

'''
##########################################
Setup Scanning and Experimental Parameters
##########################################
'''
# Parameters that can be used as axes, the keys of this dictionary are the channel identifiers
# "device_key":["Label", "unit", fast_function, slow_function, (default_start, default_end), (minimum, maximum)]
# Where fast_function and slow_function are the functions to call for scanning on the fast and slow axes, usually the same
scan_params = {
	'xaxis': ["X Axis ("+pm.CLABEL_x+")", pm.SCAN_units, None, None, (pm.DEFAULT_fastaxis_start, pm.DEFAULT_fastaxis_end), (pm.CARD_AO_Min, pm.CARD_AO_Max)],
	'yaxis': ["Y Axis ("+pm.CLABEL_y+")", pm.SCAN_units, None, None, (pm.DEFAULT_slowaxis_start, pm.DEFAULT_slowaxis_end), (pm.CARD_AO_Min, pm.CARD_AO_Max)],
	'vsd': ["Source/Drain ("+pm.CLABEL_vsd+")", "Volts", None, None, (0.0, 0.0), (pm.CARD_AO_Min, pm.CARD_AO_Max)],
	'vbg': ["Backgate ("+pm.CLABEL_vbg+")", "Volts", None, None, (0.0, 0.0), (pm.CARD_AO_Min, pm.CARD_AO_Max)]
	# 'delay_stage': ["Delay (ps)", "ps", "set_delay", "set_delay", (-25, 25), (pm.DELAY_min_ps, pm.DELAY_max_ps)]
	# Note 'temp_control' is not listed here, as that controller is not meant for rapid scanning
}

# Experimental Parameters, these should document all the parameters that cannot be automatically
# included in the device. They will be saved in the log file.
exp_params = {
	"Pre-Amp Timeconstant":0.3,
	"Pre-Amp Gain":1e-7,
	}

# Units for the experimental parameters
exp_params_units = {
	"Pre-Amp Timeconstant":"ms",
	"Pre-Amp Gain":"A/V",
	}

# Load a previously saved experimental parameters file
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
