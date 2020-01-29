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
		Takes a multidimensional numpy array x and inserts x[:,0] and x[:,$ix]
		into the buffer
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
		Returns the first $number of elements from the buffer
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

class data_image(threading.Thread):
	'''
	Data Image class that processes Queued data from the DAQ card into a 2D image

	$data_queue is the Queue containing data from the card
	$channel_ix is the column index of the data arrays (from the card) that is being tracked
	$scan is the Scan object that is controlling the measurement
	$scan_queue is a Queue from the scan object feeding points into the image

	Accessible attributes, read only:
	self.data is the current data image
	self.current_y is the current line of data
	self.current_x is the x axis points for the current line of data

	To change how data points in the image are inherit this class and override the self.process(x,y,data)
	function which processes a numpy array of data and assigns the value to (x,y) in the image. To change
	the way that the data from the line is processed override self.update_line(data) where data is the raw_input
	data array from the card
	'''
	def __init__(self, data_queue, channel_ix, scan, scan_queue, initdata=None, message_queue=None):
		self.running = True
		self.processing	= False
		self.dataQ = data_queue
		self.scanner = scan
		self.pointQueue = scan_queue
		self.message_queue = message_queue
		self.nx = scan.nx
		self.ny = scan.ny
		self.ix = channel_ix
		self.timer = Stopwatch()
		if initdata is None:
			self.data = np.zeros((self.ny, self.nx))
		else:
			self.data = initdata
		self.line_buffer = Ringbuffer2D(bufflength=200000)
		self.current_x = np.linspace(0, self.nx, self.nx)
		self.current_y = self.data[0,:]
		self.current_val = 0.0
		self.alpha = 0.05

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


	def set_data_queue(self, data_queue):
		'''
		changes the data Queue
		'''
		self.dataQ = data_queue
	# end set_data_queue

	def process(self, x, y, data):
		'''
		Processes the data for a particular point and updates the image. Simply averages, but
		can be overridden for more complex processing

		$x and $y are the x and y coordinates of the point to be updated in the image and $data
		is the data array, [time, data] to be processed
		'''
		try:
			self.data[y,x] = np.mean(data[:,1])
			self.current_val = self.alpha*self.data[y,x] + (1.0-self.alpha)*self.current_val
		except Exception as e:
			print("Error data_image.process")
			print(str(e))
	# end process

	def update_line(self, data):
		'''
		Updates the line plot with current data, while not scanning
		'''
		self.line_buffer.put(data, self.ix)
		temp = self.line_buffer.getn(self.nx)
		while temp.shape[0] != 0:
			self.current_x = temp[:,0]
			self.current_y = temp[:,1]
			self.current_val = self.alpha*np.mean(temp[:,1]) + (1.0-self.alpha)*self.current_val
			temp = self.line_buffer.getn(self.nx)
	# end update_line

	def scan_update_line(self, y):
		'''
		Updates the line plot with current data during a scan
		being scanned, $y is the y-coordinate of the current data point
		i.e. the line currently being scanned
		'''
		self.current_x = 1.0 * np.arange(self.nx)
		self.current_y = self.data[y,:]
	# end scan_update_line

	def did_not_process(self, x, y, line2=""):
		'''
		Displays a did not process error
		'''
		s = "Error cprocessing.data_image : Did not process point (" + str(x) + "," + str(y) + "),"
		print(s)
		print(line2)
		if self.message_queue != None:
			self.message_queue.put("Did not process point (" + str(x) + "," + str(y) + ")")
	# end did_not_process


	def run(self):
		'''
		Main loop of thread, updates image
		'''
		while self.running:
			if self.scanner.scanning:
				time.sleep(pm.IMG_delay_time)
				self.processing	= True
				max_loop_counter = 10
				wait_time = 10.0 /(self.scanner.linerate * pm.NUM_x_points)
				N = int(2.0*pm.MASTER_sample_freq*(1.0/(float(self.nx)*self.scanner.linerate))*self.nx*self.ny)
				N = min(N, int(5e6))
				dbuffer = np.zeros((N,2))#, dtype=DTYPE)
				next = 0 # the next available row in the buffer
				ix1 = 0 # the index corresponding to the first time in the buffer for a scan point
				ix2 = 0 # the index corresponding to the last time in the buffer for a scan point
				last = 0 # The last index in the last point
				while self.scanner.scanning or (not self.pointQueue.empty()):
					if self.pointQueue.empty(): # Wait for more points
						time.sleep(pm.IMG_delay_time)
					else: # Process data points
						loopcnt = 0
						pt = self.pointQueue.get()
						# for reference d = (time1, time2, x-coordinate, y-coordinate)
						while loopcnt < max_loop_counter: # prevent infinite loops
							loopcnt += 1
							if not self.dataQ.empty():# If there is data in the data Queue
								# Dequeue data into buffer
								temp = extract_all(self.dataQ)
								if temp is not None:
									l = temp.shape[0]
									if next+l > N:
										p = dbuffer[last:next,:]
										dbuffer = np.zeros((N,2))
										next = p.shape[0]
										dbuffer[0:next,:] = p
										last = 0
									dbuffer[next:next+l, 0] = temp[:,0]
									dbuffer[next:next+l, 1] = temp[:, self.ix]
									next += l
							# If the last point in the buffer is after the pt[1] (the end of the scan point is in the buffer)
							# Otherwise keep collecting more data
							if dbuffer[next-1,0] > pt[1]:
								# Search the buffer for the data (Binary Search for ix1 and ix2 from the old ix2 to next)
								ix1 = last + np.searchsorted(dbuffer[last:next,0], pt[0])
								ix2 = last + np.searchsorted(dbuffer[last:next,0], pt[1])
								last = ix2
								if dbuffer[ix1-1,0] <= pt[0] and dbuffer[ix2,0] > pt[0]: # If you find the endpoints, process
									self.process(pt[2], pt[3], dbuffer[ix1:ix2])
									self.scan_update_line(int(pt[3]))
									break
								else: # Else, you didn't find both endpoints
									ln2 = "Could not find data range between " + str(pt[0]) + " and " + str(pt[1])
									self.did_not_process(pt[2], pt[3], line2=ln2)
									break
							else:
								time.sleep(wait_time)
						else: # else on the while loop, executed when loop ends unless break was called
							ln2 = "no data received in " + str(max_loop_counter*wait_time) + " seconds, "
							self.did_not_process(pt[2], pt[3], line2=ln2)
					# end if/else
				else:
					self.processing	= False
			else:
				temp = extract_all(self.dataQ)
				self.update_line(temp)
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

class data_image_2(threading.Thread):
	'''
	Data Image class that processes Queued data from the DAQ card into a 2D image, taking the data in
	chuncks not in a time series.

	$data_queue is the Queue containing data from the card
	$channel_ix is the column index of the data arrays (from the card) that is being tracked
	$scan is the Scan object that is controlling the measurement

	Accessible attributes, read only:
	self.data is the current data image
	self.current_y is the current line of data
	self.current_x is the x axis points for the current line of data

	To change how data points in the image are inherit this class and override the self.process(x,y,data)
	function which processes a numpy array of data and assigns the value to (x,y) in the image. To change
	the way that the data from the line is processed override self.update_line(data) where data is the raw_input
	data array from the card
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

		$x and $y are the x and y coordinates of the point to be updated in the image and $data is the data array, [time, data] to be processed
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
		being scanned, $y is the y-coordinate of the current data point
		i.e. the line currently being scanned
		'''
		self.current_x = 1.0 * np.arange(self.nx)
		self.current_y = self.data[y,:]
	# end scan_update_line

	def did_not_process(self, x, y, line2=""):
		'''
		Displays a did not process error

		DEPRICIATED
		'''
		s = "Error cprocessing.data_image : Did not process point (" + str(x) + "," + str(y) + "),"
		print(s)
		print(line2)
		if self.message_queue != None:
			self.message_queue.put("Did not process point (" + str(x) + "," + str(y) + ")")
	# end did_not_process

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
