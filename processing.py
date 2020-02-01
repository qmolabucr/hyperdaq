'''
processing.py

hyperDAQ module for processing incoming data

Last Updated: January 2020

|  Trevor Arp
|  Gabor Lab
|  University of California, Riverside

All Rights Reserved
'''

import numpy as np

import threading
import parameters as pm
import time
from hyperdaq.utilities import Stopwatch


DTYPE = np.float64
def extract_all(q):
	'''
	Removes all elements from the given data queue, returns data in a single array
	- q is the input queue, containing data in numpy format
	'''
	#cdef int rows, cols, m, n, start
	#cdef np.ndarray d, data
	start = 0
	try:
		d = q.get()
		rows = d.shape[0]
		cols = d.shape[1]
		n = q.qsize()
		data = np.zeros(((5+n)*rows, cols), dtype=DTYPE)
		m = data.shape[0]
		data[0:rows,:] = d
		start += rows
		while(not q.empty()):
			d = q.get()
			rows = d.shape[0]
			if start+rows-1 >= m:
				n = q.qsize()
				data = np.append(data, np.zeros(((10+n)*rows, cols), dtype=DTYPE), axis=0)
				m = data.shape[0]
			data[start:start+rows,:] = d
			start += rows
		return data[0:start,:]
	except Exception as e:
		print("processing.extract_all : Could not read Queue")
		print(str(e))
		return None
# end extract_all

class Ringbuffer2D():
	'''
	A simple and efficient ring buffer for buffering 2D data from a larger numpy array
	The first index of the array is assumed to be a time coordinate
	Buffer_time_data.num is the number of elements in the array
	'''
	def __init__(self, bufflength=100000):
		self.data = np.zeros((bufflength,2), dtype='f')
		self.length = bufflength
		self.start = 0
		self.end = 0
		self.num = 0
	#


	def put(self, x, ix):
		'''
		Takes a multidimensional numpy array x and inserts x[:,0] and x[:,ix] into the buffer
		'''
		n = np.shape(x)[0]
		if n > 0:
			x_index = (self.end  + np.arange(n)) % self.length
			self.data[x_index,0] = x[:,0]
			self.data[x_index,1] = x[:,ix]
			self.end = x_index[-1] + 1
			self.num += n
		#
	#


	def get(self):
		'''
		Returns the first element in the Queue
		'''
		if self.num > 0:
			out = self.data[self.start]
			self.start = (self.start + 1) % self.length
			self.num -= 1
			return out
		else:
			return np.zeros((0,2))
	# end get


	def getn(self, number):
		'''
		Returns the first number of elements from the buffer
		'''
		if self.num < number:
			return np.zeros((0,2))
		else:
			x_index = (self.start + np.arange(number)) % self.length
			out = self.data[x_index]
			self.start = x_index[-1] + 1
			self.num -= number
			return out
	# end getn
# end Ringbuffer2D

class data_image_2(threading.Thread):
	'''
	Data Image class that processes Queued data from the DAQ card into a 2D image, taking the data in
	chuncks not in a time series.

	To change how data points in the image are inherit this class and override the self.process(x,y,data)
	function which processes a numpy array of data and assigns the value to (x,y) in the image. To change
	the way that the data from the line is processed override self.update_line(data) where data is the raw_input
	data array from the card.

	Args:
		data_queue : the Queue containing data from the card
		channel_ix : the column index of the data arrays (from the card) that is being tracked
		scan : the Scan object that is controlling the measurement

	Attributes:
		data : The current data image. READ ONLY
		current_y : The current line of data. READ ONLY
		current_x : The x axis points for the current line of data. READ ONLY

	'''
	def __init__(self, data_queue, channel_ix, scan, initdata=None, message_queue=None):
		self.running = True
		self.processing	= False
		self.dataQ = data_queue
		self.scanner = scan
		self.message_queue = message_queue
		self.nx = scan.nx
		self.ny = scan.ny
		self.ix = channel_ix
		self.timer = Stopwatch()
		if initdata is None:
			self.data = np.zeros((self.ny, self.nx))
		else:
			self.data = initdata
		self.current_x = np.linspace(0, self.nx, self.nx)
		self.current_y = self.data[0,:]
		self.current_val = 0.0
		self.alpha = 0.25

		# start the thread
		threading.Thread.__init__(self)
		self.start()
	# end init

	def resize(self):
		'''
		Resizes the data image
		'''
		if not self.processing:
			self.nx = self.scanner.nx
			self.ny = self.scanner.ny
			self.data = np.zeros((self.ny, self.nx))
		else:
			print("Error processing.resize: cannot resize image while processing")
	# end resize

	def set_data_queue(self, data_queue):
		'''
		changes the data Queue
		'''
		self.dataQ = data_queue
	# end set_data_queue

	def process(self, x, y, data):
		'''
		Processes the data for a particular point and updates the image. Simply averages, but can be
		overridden for more complex processing

		Args:
			x is the x coordinate of the point to be updated in the image
			y is the y coordinate of the point to be updated in the image
			data is the data array, [time, data] to be processed
		'''
		try:
			self.data[y,x] = np.mean(data[:,self.ix])
			self.current_val = self.alpha*np.mean(data[:,self.ix]) + (1.0-self.alpha)*self.current_val
		except Exception as e:
			print("Error data_image.process")
			print(str(e))
	# end process

	def update_line(self, data):
		'''
		Updates the current values while not writing into the data array
		matches in function with update_line from previous iterations
		'''
		self.current_val = self.alpha*np.mean(data[:,self.ix]) + (1.0-self.alpha)*self.current_val
	# end update_line

	def scan_update_line(self, y):
		'''
		Updates the line plot with current data during a scan
		being scanned,

		Args:
			y : The y-coordinate of the current data point i.e. the line currently being scanned
		'''
		self.current_x = 1.0 * np.arange(self.nx)
		self.current_y = self.data[y,:]
	# end scan_update_line

	def run(self):
		'''
		Main loop of thread, updates image
		'''
		while self.running:
			if not self.dataQ.empty():# If there is data in the data Queue
				# Dequeue data into buffer
				pt = self.dataQ.get()
				if len(pt) == 3:
					if pt[0] < 0:
						self.update_line(pt[2])
					else:
						self.process(pt[0], pt[1], pt[2])
				else:
					print("Error processing: Invalid data in queue")
			else: # Wait for more data
				time.sleep(pm.IMG_delay_time)
			#
		# end main loop
	# end run

	def stop(self):
		'''
		Stops the process
		'''
		self.running = False
	# end stop

	def set_scanner(self, scan):
		'''
		Changes the scan object
		'''
		if not self.processing:
			self.scanner = scan
		else:
			print("Error: cannot change scanning object while scanning")
# end data_image
