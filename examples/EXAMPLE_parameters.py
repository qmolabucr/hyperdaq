'''
####################################
############ Calibration ###########
####################################
Change based on Calibration Data
'''

# Delay Stage Calibration
DELAY_mm_to_ps = 6.673 # ps/mm
DELAY_center_position = 107.506 #107.473 #108.125 # mm +/- 0.01

# Galvos Calibration
# From fitting 40um Hall Bars: 84.0 +/- 2.3 micron/Volt
SCAN_units_to_volts = 1.0/84.0 # V/um Conversion factor
SCAN_units = "Micron" #"Volts"

# Calibration for the voltage channels
# Key is the channel identifier
# Values are (multipiler, offset)
# multiplier : V(card)/V(Output of electionics)
CHANNEL_calibration = {
    'xaxis':(1.0,0.0),
    'yaxis':(1.0,0.0),
    'vsd':(11.088,0.0), # With voltage divider, check sign
    'vbg':(1.0, 0.0) # No amplifier
    }

'''
####################################
### External Hardware Parameters ###
####################################
'''

COM_magnet = "COM8"
COM_tempcontrol = "COM5"
COM_OPO = "COM4"

# OPO Parameters
OPO_min_wavelength = 1120 #nm
OPO_max_wavelength = 1525 #nm

#Delay Stage Parameters
SERIAL_delay = 73852417
DELAY_min_mm = 0.0
DELAY_max_mm = 220.0
DELAY_min_ps = (DELAY_center_position - DELAY_max_mm)*DELAY_mm_to_ps
DELAY_max_ps = (DELAY_center_position - DELAY_min_mm)*DELAY_mm_to_ps
DELAY_step_to_mm = 2.0/0.0001 # step/mm
DELAY_vel_to_mm = 1237/100.0 # Device Units per mm/s
DELAY_default_accel = 13421772 # Device units

# Rotation Stage Parameters
SERIAL_rotation_delay = 55000079
SERIAL_rotation_refer = 55000642
SERIAL_rotation_polar = 55000677

ROTATION_step_to_deg = 136533.33
ROTATION_min_angle = 0.0
ROTATION_max_angle = 360.0

BEAM_default_ratio = 1.0
BEAM_max_ratio = 5.0
BEAM_min_ratio = 0.1

POLARIZATION_default_angle = 121.5

# Heater Parameters
MAX_heater_temp = 420.0 # Kelvin

'''
####################################
#### System Specific Parameters ####
####################################
Change based on system, should only be changed infrequently
'''
SYSTEM_prefix = "CPS" # Identifier for the all the files taken by this system

# Excluded Axes
EXCLUDE_fast = ['yaxis', 'pol_stage', 'opo_control']
EXCLUDE_slow = ['xaxis']
EXCLUDE_third_axis = ['yaxis', 'xaxis']

DATA_dir_path = "E:\\Data\\Raw\\" # Path to data directory
PARAMS_file = "experimental_parameters.txt"

NUM_card_queues = 3
REF_IMG_index = 0
POW_IMG_index = 1
PCI_IMG_index = 2

DRIFT_y_min = 0.25 # minimum drift correction
DRIFT_y_max = 5.0 # maximum drift correction

# Removing the x drift correction for noew
DRIFT_x_min = 0.5 # minimum drift correction
DRIFT_x_max = 5.0 # maximum drift correction

# DAQ Card Limits
CARD_AO_Max = 10.0
CARD_AO_Min = -10.0
CARD_AI_Max = 10.0
CARD_AI_Min = -10.0

# Scan save configurations
NUM_Save_Slots = 12
SCANS_Save_dir = 'saved'

# How often Serial Devices are polled
SERIAL_device_poll = 2.0 # 0.25

# Plots Labels
PLT_image0_title = "Reflection Image"
PLT_image1_title = "Photocurrent Image"

# Plots colorscales
PLT_image0_cmap = 'plasma'
PLT_image1_cmap = 'viridis'

# GUI defaults
DEFAULT_fastaxis_start = -1.0/SCAN_units_to_volts
DEFAULT_fastaxis_end = 1.0/SCAN_units_to_volts
DEFAULT_slowaxis_start = -1.0/SCAN_units_to_volts
DEFAULT_slowaxis_end = 1.0/SCAN_units_to_volts

# Channel Parameters
NUM_input_channels = 10
DEVICE_output = 'Dev1/'
CHANNEL_input = "Dev1/ai0:7, Dev1/ai16:17"
CHANNEL_x = 'Dev1/ao0'
CHANNEL_y = 'Dev1/ao1'
CHANNEL_vsd = 'Dev1/ao2'
CHANNEL_vbg = 'Dev1/ao3'

# Channel Labels
CLABEL_x = 'ao0'
CLABEL_y = 'ao1'
CLABEL_vsd = 'ao2'
CLABEL_vbg = 'ao3'

CNTL_function_wait = 0.1 # Wait after calling a hardware function while scanning
CNTL_fast_function_wait = 0.1 # Wait after calling a hardware function, with the function on the fast axis
CNTL_repeat_wait = 0.5 # Wait before repeating (seconds)

'''
####################################
######### DAQ Parameters ###########
####################################
DO NOT CHANGE UNLESS NECESSARY
'''

DAQ_name = "hyperDAQ - v2.0"

# Data Writing Parameters
DATA_file_ext = "dat" # Data file extension, i.e. filename.ext
WRITE_delay = 0.25

# Required parameters to run a scan, and what to call them in a log file, if they are not the fast, slow or cube axis
# Minimum of xaxis, yaxis are required for 2D spatial scanning. Note that outputs in this will be calibrated with
# the SCAN_units_to_volts parameter, and entries that are not axes will not be calibrated.
SCAN_spatial_parameters = ['xaxis', 'yaxis', 'scanangle']
# Lognames are how the parameter is recorded in the log file, these names are also displayed minus the word "Value"
SCAN_spatial_lognames = ['Center X Value', 'Center Y Value', 'Scan Angle']

# These are voltage card outputs that are required, they will be automatically saved, controled and set constant as appropriate
SCAN_voltage_parameters = ['vsd', 'vbg']
# Lognames are how the parameter is recorded in the log file, these names are also displayed on the interface
SCAN_voltage_lognames = ['Source/Drain', 'Backgate']

# Default scanning mode
SCAN_default_type = "rectilinear-4output"

# Default outputs for spatial variables and voltages
DEFAULT_voltage_output = 0.0
DEFAULT_spatial_output = 0.0

# Required parameters for the log file, keys are the identifiers, values are the log file names
# Will automatically write out the fast and slow axes
SCAN_log_parameters = {}

#Hypercubic parameters
#DEFAULT_scan_number = 25
#DEFAULT_repeat_number = 10
DEFAULT_number = 10

# Data Acquisition Parameters
MASTER_sample_freq = 20000.0 # DAQ card sample frequency, type = float

# Image parameters
NUM_x_points = 200 # Number of x points in an image, default
NUM_y_points = 200 # Number of y points in an image, default
NUM_max_points = 5000 # Maximum Number of x or y points in an image

# Main loop and GUI timing
MAIN_loop_delay = 0.25
PLT_update_delay = 50 # in milliseconds

# Card, Controller and image threading and timing
IMG_delay_time = 0.25 # seconds
CARD_poll_delay = 0.001
CNTL_default_ramp_time = 0.5
CNTL_scan_wait = 0.1
CNTL_zero_time = CNTL_default_ramp_time

RATE_line_scan = 10 #5 #The prime line rate in Hertz
NUM_to_dequeue = int(IMG_delay_time / CARD_poll_delay)

'''
####################################
######## Dummy Parameters ##########
####################################
So that this parameters file will work on multiple system scripts
'''
DLL_directory = "C:\Program Files\Thorlabs\Kinesis"
NDFILTER_optical_density = 1
NDFILTER_angular_range = 1
PCY_IMG_index = 4
NANOROTATION_min_angle = 0.0
NANOROTATION_max_angle = 360.0
