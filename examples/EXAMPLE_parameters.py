'''
####################################
############ Calibration ###########
####################################
Change based on Calibration Data
'''
# Scanning calibration to convert voltage to physical space
# Usually calibrating scanning galvo mirrors.
SCAN_units_to_volts = 1.0/1.0 # V/um Conversion factor
SCAN_units = "Volts" # "Micron"

# Calibration for the voltage channels accounting for any electronics between the card
# and the sample/experiment. These exist so that the values displayed on the GUI are
# the real values applied to the sample
# A dictionary where Key is the channel identifier
# Values are (multiplier, offset)
# multiplier : V(card)/V(Output of electronics)
CHANNEL_calibration = {
    'xaxis':(1.0,0.0),
    'yaxis':(1.0,0.0),
    'vsd':(1.0, 0.0),
    'vbg':(1.0, 0.0)
    }

## Example of calibration parameters for the Thor Labs device defined in EXAMPLE_hyperdaq
## Delay Stage Calibration
# DELAY_mm_to_ps = 6.673 # ps/mm
# DELAY_center_position = 107.506 # mm +/- 0.01

'''
####################################
### External Hardware Parameters ###
####################################
'''

##
## Example of hardware parameters for the example devices defined in EXAMPLE_hyperdaq
##

## Example Hardware: ThorLabs Delay Stage
# DLL_Thorlabs = "C:\Program Files\Thorlabs\Kinesis" # Thor Labs code DLL
# SERIAL_delay = 73852417 # The serial number of the delay stage
# DELAY_min_mm = 0.0
# DELAY_max_mm = 220.0
# DELAY_min_ps = (DELAY_center_position - DELAY_max_mm)*DELAY_mm_to_ps
# DELAY_max_ps = (DELAY_center_position - DELAY_min_mm)*DELAY_mm_to_ps
# DELAY_step_to_mm = 2.0/0.0001 # step/mm
# DELAY_vel_to_mm = 1237/100.0 # Device Units per mm/s
# DELAY_default_accel = 13421772 # Device units

## Example Hardware: Lakeshore Temperature controller
# COM_tempcontrol = "COM5" # The COM address of the serial port
# MAX_heater_temp = 420.0 # Kelvin

'''
####################################
#### System Specific Parameters ####
####################################
Change based on system, should only be changed infrequently
'''
# Identifier for the all the files taken by this system, should be uniform across everything
# Related to this experimental system, often three or four letters
SYSTEM_prefix = "EXMP"

# Path to the directory where the raw data is saved, it is often useful to have a file for the raw
# and processes data in the same directory, with the same directory structures.
DATA_dir_path = "E:\\Data\\Raw\\"

# The number of Queues the card controller should fill, needs to correspond to the number of
# data_images that are processing the data.
NUM_card_queues = 2

# The NI DAQ card controller returns data for all defined card inputs channels, and passes an array
# where the columns correspond to different input channels. These indices tell the data_images which
# column index corresponds to input they are tracking. This may change based on the specific NI card.
RFI_IMG_index = 0
PCI_IMG_index = 2
# POW_IMG_index = 1 # Index for example auxiliary image defined in EXAMPLE_hyperdaq

# DAQ Card Analog Output Limits, changing them will change the output resolution
CARD_AO_Max = 10.0
CARD_AO_Min = -10.0
CARD_AI_Max = 10.0
CARD_AI_Min = -10.0

# DAQ Input Channel Parameters
NUM_input_channels = 10
DEVICE_output = 'Dev1/'
CHANNEL_input = "Dev1/ai0:7, Dev1/ai16:17"

# DAQ Output Channel Parameters, additional voltage channels can be added so long as they are 
# implemented in the card controller
CHANNEL_x = 'Dev1/ao0'
CHANNEL_y = 'Dev1/ao1'
CHANNEL_v1 = 'Dev1/ao2'
CHANNEL_v2 = 'Dev1/ao3'

# DAQ card channel Labels, to be displayed on the User Interface
CLABEL_x = 'ao0'
CLABEL_y = 'ao1'
CLABEL_vsd = 'ao2'
CLABEL_vbg = 'ao3'

# Axes to exclude as options from the fast, slow and additional axes
EXCLUDE_fast = ['yaxis']
EXCLUDE_slow = ['xaxis']
EXCLUDE_third_axis = ['yaxis', 'xaxis']

# Scan save configurations
NUM_Save_Slots = 12
SCANS_Save_dir = 'saved'

# How often Serial Devices are polled
SERIAL_device_poll = 1.0

# Drift correction parameters, for if drift correction is enabled
DRIFT_y_min = 0.25 # minimum drift correction
DRIFT_y_max = 5.0 # maximum drift correction
DRIFT_x_min = 0.5 # minimum drift correction
DRIFT_x_max = 5.0 # maximum drift correction

# Plots Labels
PLT_image0_title = "Reflection Image"
PLT_image1_title = "Photocurrent Image"

# Plots colorscales
PLT_image0_cmap = 'plasma'
PLT_image1_cmap = 'viridis'

# Plot sizes, the approximate size of the main plots
PLT_xsize_inches = 5.25
PLT_ysize_inches = 4.0

# GUI defaults
DEFAULT_fastaxis_start = -1.0/SCAN_units_to_volts
DEFAULT_fastaxis_end = 1.0/SCAN_units_to_volts
DEFAULT_slowaxis_start = -1.0/SCAN_units_to_volts
DEFAULT_slowaxis_end = 1.0/SCAN_units_to_volts
NUM_x_points = 200 # Number of x points in an image, default
NUM_y_points = 200 # Number of y points in an image, default

# Function wait times, cause a delay after a command is sent to a piece of hardware, to give it time to respond
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

PARAMS_file = "experimental_parameters.txt" # the experimental parameters file

# Required parameters to run a scan, and what to call them in a log file, if they are not the fast, slow or cube axis
# Minimum of xaxis, yaxis are required for 2D spatial scanning. Note that outputs in this will be calibrated with
# the SCAN_units_to_volts parameter, and entries that are not axes will not be calibrated.
SCAN_spatial_parameters = ['xaxis', 'yaxis', 'scanangle']
# Lognames are how the parameter is recorded in the log file, these names are also displayed minus the word "Value"
SCAN_spatial_lognames = ['Center X Value', 'Center Y Value', 'Scan Angle']

# These are voltage card outputs that are required, they will be automatically saved, controlled and set constant as appropriate
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

#Default number of scans for a cubic or hypercubic axis
DEFAULT_number = 10

# Data Acquisition Parameters
MASTER_sample_freq = 20000.0 # DAQ card sample frequency, type = float

# Image parameters
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

RATE_max_line_scan = 10.0 # The maximum line rate in Hertz
RATE_default_line_scan = 2.0 # The default line rate in Hertz
NUM_to_dequeue = int(IMG_delay_time / CARD_poll_delay)
