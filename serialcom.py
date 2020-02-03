'''
serialcom.py

GaborDAQ functions for communicating with equipment via serial

Last Updated: January 2020

|  Trevor Arp
|  Gabor Lab
|  University of California, Riverside

All Rights Reserved
'''

import time
import threading
import serial
import queue
import array

import parameters as pm
from hyperdaq.utilities import hl2int

from traceback import format_exc

class serial_device(threading.Thread):
	'''
	A base class for communicating with serial devices

	Args:
		com_port : string that sets the COM port used by the serial device e.g. "COM2"

	'''
	def __init__(self, com_port, Baudrate=57600, Parity=serial.PARITY_ODD, Stopbits=serial.STOPBITS_ONE, Bytesize=serial.SEVENBITS, Terminator='\r\n', Timeout=0.05):
		self.running = False
		self.term = Terminator
		self.port = com_port
		self.baudrate = Baudrate #9600,
		self.parity = Parity
		self.stopbits = Stopbits
		self.bytesize= Bytesize
		self.timeout=Timeout

		try:
			self.ser = serial.Serial(
				port=str(self.port),
				baudrate=self.baudrate, #9600,
				parity=self.parity,
				stopbits=self.stopbits,
				bytesize=self.bytesize,
				timeout=self.timeout
			)
			self.settings = self.ser.getSettingsDict()
			self.serial_open = True
		except Exception as e:
			self.serial_open = False
			print("Error opening Serial Port" + str(com_port))
			print(str(e))
		#

		# initialize data queue
		self.data_queue = queue.Queue(maxsize=200000)

		threading.Thread.__init__(self)
		self.name = "SERIAL DEVICE"
	# end init

	def reset(self, stoptime=0.5):
		'''
		Resets the serial object
		'''
		self.serial_open = False
		self.ser.close()
		print(str(self.name) + ": Resetting Serial Port")
		time.sleep(stoptime)
		try:
			self.ser = serial.Serial(
				port=str(self.port),
				baudrate=self.baudrate, #9600,
				parity=self.parity,
				stopbits=self.stopbits,
				bytesize=self.bytesize,
				timeout=self.timeout
			)
			time.sleep(0.05)
			self.serial_open = True
		except Exception as e:
			print(str(self.name) + " Error resetting Serial Port" + str(self.port))
			print(str(e))
			self.err_cnt += 1
			self.serial_open = False


	def get_data_queue(self):
		'''
		returns the data Queue
		'''
		return self.data_queue
	# end get_data_queue


	def get_data(self, command):
		'''
		Sends a request for data to the serial device and listens for a response
		Returns none and prints to terminal if no response was received
		'''
		if self.ser.isOpen():
			out = ""
			try:
				b = str(command) + self.term
				self.ser.reset_input_buffer()
				self.ser.write(b.encode('utf-8'))
				self.ser.flush()
				out = self.ser.readline()
				out = out.strip()
			except Exception as e:
				print(str(self.name) + " serialcom.get_data Error: Could not get data")
				print(format_exc())
			if out == "":
				return None
			else:
				return out
	# end get_data


	def get_raw_data(self, command):
		'''
		Similar to get_data, but works in a different way, reads until it times out and
		returns raw serial output as a byte array
		'''
		if self.ser.isOpen():
			out = []
			try:
				b = str(command) + self.term
				self.ser.write(b.encode('utf-8'))
				out = self.ser.readlines()
				#out = self.ser.readline()
				#print out #DEBUG
			except Exception as e:
				print(str(self.name) + " serialcom.get_data Error: Could not get data")
				print(str(e))
			if out == []:
				return None
			else:
				#print array.array('B',out) #DEBUG
				#return array.array('B',out) # if readline
				return array.array('B',out[0]) # if readlines
	# end get_data


	def set_data(self, command):
		'''
		Sends a command to the serial device
		'''
		if self.ser.isOpen():
			try:
				b = str(command) + self.term
				self.ser.write(b.encode('utf-8'))
			except Exception as e:
				print(str(self.name) + " serialcom.set_data Error: Could not set data")
				print(format_exc())
		#
	# end set_data


	def send_command(self, command):
		'''
		Sends a command and does not wait for a response
		'''
		if self.ser.isOpen():
			try:
				b = str(command) + self.term
				self.ser.write(b.encode('utf-8'))
			except Exception as e:
				print(str(self.name) + " serialcom.send_command Error: Could not send command")
				print(str(e))
	# end send_command


	def stop(self):
		'''
		Closes the serial connection and stops the thread
		'''
		self.running = False
		self.ser.close()
	# end stop
# end serial_device


class serial_device_listener(serial_device):
	'''
	Defines a controller for a serial device that is continuously taking data

	Continuously calls the read_data() function, override to define read data from
	the device

	Args:
		com_port : string that sets the COM port used by the serial device e.g. "COM2"
		sys_time : a synchronized Stopwatch object for timing

	'''
	def __init__(self, com_port, sys_time, Baudrate=57600, Parity=serial.PARITY_ODD, Stopbits=serial.STOPBITS_ONE, Bytesize=serial.SEVENBITS, Terminator='\r\n',Timeout=0.05):
		serial_device.__init__(self, com_port, Baudrate=Baudrate, Parity=Parity, Stopbits=Stopbits, Bytesize=Bytesize, Terminator=Terminator, Timeout=Timeout)
		self.timer = sys_time
		self.queueing = False
		self.reading = False
		self.name = "SERIAL DEVICE LISTENER"
		self.err_cnt = 0
		if self.serial_open:
			self.start()
	# end init


	def toggle_queueing(self):
		'''
		Toggles the Queueing of data, not Queueing by default
		'''
		self.queueing = self.queueing != True
	# end toggle_queueing


	def run(self):
		'''
		Runs the thread
		'''
		self.running = True
		self.task = threading.Timer(pm.SERIAL_device_poll, self.read_data_call)
		self.task.start()
	# end run

	def read_data_call(self):
		'''
		Calls the read_data function
		'''
		self.reading = True
		if self.serial_open:
			self.read_data()
		self.reading = False
		if self.err_cnt > 10:
			print("Error "+str(self.name)+": Exceeded 10 errors, stopping polling")
		if self.running and self.err_cnt <= 10:
			self.task = threading.Timer(pm.SERIAL_device_poll, self.read_data_call)
			self.task.start()
	#

	def set_data(self, command):
		'''
		Sends a command to the serial device
		'''
		self.task.cancel()
		serial_device.set_data(self,command)
		self.task = threading.Timer(pm.SERIAL_device_poll, self.read_data_call)
		self.task.start()
	#

	def read_data(self):
		'''
		Reads data from the serial device, override to add functionality
		'''
		pass
	# end read_data

	def get_current_data(self, command):
		'''
		Pauses main task to send command to the device and waits for response
		Returns None if the device isn't running
		'''
		if self.running:
			self.task.cancel()
			time.sleep(0.1)
			d = self.get_data(str(command))
			self.task = threading.Timer(pm.SERIAL_device_poll, self.read_data_call)
			self.task.start()
			return d
		else:
			return None
	# end get_current

	def stop(self):
		'''
		Closes the serial connection and stops the thread
		'''
		if self.running:
			self.running = False
			while self.reading:
				time.sleep(0.1)
			self.task.cancel()
			time.sleep(0.1)
			self.ser.close()
	#end stop
# end serial_device_listener

class lakeshore_336(serial_device_listener):
	'''
	Defines a controller for a LakeShore Model 336 Temperature Controller

	Temperatures are accessible as read-only attributes as self.A, self.B, etc.
	for temperature sensors A,B,... in Kelvin
	'''
	def __init__(self, com_port, sys_time):
		self.device_name = "LakeShore 336"
		self.A = 0.0
		self.B = 0.0
		self.C = 0.0
		self.D = 0.0
		self.setpoint = 0.0
		serial_device_listener.__init__(self, com_port, sys_time)
		self.name = "LakeShore 336"
	# end init


	def get_output_setting(self):
		'''
		Gets the heater output setting
		'''
		return self.get_current_data("RANGE?1")
	# end get_output_setting


	def set_output_setting(self, setting):
		'''
		Sets the heater output setting,
		0 is off, 1 is low, 2 is medium and 3 is high
		'''
		cmd = "RANGE1," + str(int(setting))
		self.set_data(cmd)
	# end set_output_setting


	def turn_heater_off(self):
		'''
		Turns off the heater
		'''
		self.set_data("RANGE1,0")
	# end turn_heater_off

	def get_output_percent(self):
		'''
		Gets the heater output percentage
		'''
		return float(self.get_current_data("HTR?1"))
	# end get_output_percent

	def get_setpoint(self):
		'''
		Gets the temperature setpoint
		'''
		try:
			if self.serial_open:
				self.setpoint = float(self.get_current_data("SETP?1"))
			return self.setpoint
		except:
			return self.setpoint
	# end get_setpoint

	def set_setpoint(self, value):
		'''
		Sets the temperature setpoint
		'''
		if self.serial_open:
			cmd = "SETP1" + "," + str(float(value))
			self.set_data(cmd)
	# set_setpoint


	def get_PID(self):
		'''
		Gets the current gain values of the PID loop
		'''
		d = self.get_current_data("PID?1")
		return [float(i) for i in d.split(",")]
	# end get_PID


	def set_PID(self, P, I, D):
		'''
		Sets the gain values of the PID loop
		'''
		if self.serial_open:
			cmd = "PID1," + str(float(P)) + "," + str(float(I)) + "," + str(float(D))
			self.set_data(cmd)
	# end set_PID


	def read_data(self):
		'''
		Reads data from the controller, currently only reads A due to serial port issues
		'''
		try:
			if self.serial_open:
				a = self.get_data("KRDG?A")
				if a != None:
					self.A = float(a)
				# b = self.get_data("KRDG?B")
				# c = self.get_data("KRDG?C")
				# d = self.get_data("KRDG?D")
				# if all(i!= None for i in [a,b,c,d]):
				# 	self.A = float(a)
				# 	self.B = float(b)
				# 	self.C = float(c)
				# 	self.D = float(d)
				# 	if self.queueing:
				# 		s = str(self.timer.time()) + "\t" + str(self.A) + "\t" + str(self.B) + "\t" + str(self.C) + "\t" + str(self.D)
				# 		self.data_queue.put(s)
		except ValueError:
			self.reset()
			print("Finished Resetting")
			self.err_cnt += 1
			print("Error Count: " + str(self.err_cnt))
		except:
			print("Error in "+str(self.name)+": could not read serial data")
			print(format_exc())
			self.err_cnt += 1
	# end read_data
# end lakeshore_336

class lakeshore_335(lakeshore_336):
	'''
	Defines a controller for a LakeShore Model 336 Temperature Controller

	Temperatures are accessible as read-only attributes as self.A, self.B, etc.
	for temperature sensors A,B,... in Kelvin
	'''
	def __init__(self, com_port, sys_time):
		self.device_name = "LakeShore 335"
		self.A = 0.0
		self.B = 0.0
		lakeshore_336.__init__(self, com_port, sys_time)
		self.name = "LakeShore 335"
	# end init

	def read_data(self):
		'''
		Reads data from the controller
		'''
		try:
			if self.serial_open:
				a = self.get_data("KRDG?A")
				b = 0.0
				#b = self.get_data("KRDG?B")
				if all(i!= None for i in [a,b]):
					self.A = float(a)
					self.B = float(b)
					if self.queueing:
						s = str(self.timer.time()) + "\t" + str(self.A) + "\t" + str(self.B)
						self.data_queue.put(s)
		except:
			print("Error in serialcom.lakeshore_335.read_data, could not read serial data")
			self.err_cnt += 1
	# end read_data
# end lakeshore_335

class lakeshore_625(serial_device_listener):
	'''
	Defines a controller for a LakeShore Model 625 Superconducting Magnet Power supply

	Args:
		com_port : string that sets the COM port used by the serial device e.g. "COM6"

	Attributes:
		voltage : The output voltage, Units in Volts
		current : The output current, Units in Amps
		field   : The central field, computed from the current based on calibration, Units in Tesla

	'''
	def __init__(self, com_port, sys_time):
		self.device_name = "LakeShore 625"
		self.current = 0.0
		self.voltage = 0.0
		self.field = 0.0
		serial_device_listener.__init__(self, com_port, sys_time)
		self.name = "LakeShore 625"
	#


	def read_data(self):
		'''
		Reads the current and voltage and computes the field
		'''
		try:
			if self.serial_open:
				c = self.get_data("RDGI?")
				v = self.get_data("RDGV?")
				if all(i!= None for i in [c,v]):
					self.current = float(c)
					self.voltage = float(v)
					self.field = self.current * pm.CURRENT_to_field * 1.0e-4
					if self.queueing:
						s = str(self.timer.time()) + "\t" + str(self.current) + "\t" + str(self.voltage) + "\t" + str(self.field)
						self.data_queue.put(s)
					#
				#
			#
		except:
			print("Error in serialcom.lakeshore_625, could not read serial data")
			self.err_cnt += 1
	# end read_data
# end lakeshore_625


class spectrapro_2300(serial_device):
	'''
	Defines a controller for the SpectraPro-2300 Monochrometer

	Args:
		com_port : string that sets the COM port used by the serial device e.g. "COM6"

	Attributes:
		wavelength : The Wavelength, in nm

	'''
	def __init__(self, com_port, sys_time):
		self.device_name = "SpectraPro-2300"
		self.wavelength = 0.0
		#self.width = 0.0
		serial_device.__init__(self, com_port,
		Baudrate=9600, Parity=serial.PARITY_NONE, Bytesize=serial.EIGHTBITS, Terminator='\r', Timeout=None)
		self.name = "SpectraPro-2300"
	# end init


	def set_wavelength(self, w):
		'''
		Sets the wavelength
		'''
		cmd = str(round(w, ndigits=3)) + " GOTO"
		try:
			self.set_data(cmd)
			return 1
		except:
			print("Error in serialcom.set_wavelength, could not read serial data")
			self.err_cnt += 1
			return -1
	# end set_output_setting


	def read_data(self):
		'''
		Reads data from the controller
		'''
		try:
			if self.serial_open:
				l = self.get_data("?NM")
				#w = self.get_data("?MICRONS")
				if l == 'ok':
					l = self.get_data("?NM")
				if l is not None:
					try:
						l = l.split()[1]
						self.wavelength = float(l)
					except Exception as e:
						pass
						#print "Error serialcom.spectrapro_2300: Invalid Wavelength Returned"
						#print e
		except:
			print("Error in serialcom.read_data, could not read serial data")
			self.err_cnt += 1
	# end read_data
# end spectrapro_2300

class mira_900_OPO(serial_device_listener):
	'''
	Defines a controller for the OPO controller

	Args:
		com_port : string that sets the COM port used by the serial device e.g. "COM6"

	Attributes:
		wavelength : The Wavelength, in nm
		power : The IR power in arbitrary units
		piezo : the piezo voltage
		RH : The relative Humidity in the cavity

	'''
	def __init__(self, com_port, sys_time):
		self.device_name = "Mira 900 OPO"
		self.wavelength = 0.0
		self.power = 0.0
		self.piezo = 0.0
		self.fwhm = 0.0
		self.RH = 0.0
		serial_device_listener.__init__(self, com_port, sys_time,
			Baudrate=38400,
			Parity=serial.PARITY_NONE,
			Bytesize=serial.EIGHTBITS,
			Timeout=0.1,
			Terminator='\n'
			)
		self.name = "MIRA 900 OPO"
		self.manual_mode = False
	# end init

	def toggle_manual_mode(self):
		'''
		Toggles manual mode, when in manual data is not collected continuously, instead only when
		external function calls are made.
		'''
		if self.manual_mode:
			self.running = True
			self.manual_mode = False
			self.task = threading.Timer(pm.SERIAL_device_poll, self.read_data_call)
			self.task.start()
		else:
			loopcount = 0
			while(self.reading):
				#print "Here"
				loopcount += 1
				if loopcount > 50:
					print("Break out")
					print("Error serialcom.toggle_manual_mode: Reading thread blocking port")
					#self.ser.close()
					self.reset()
					break
				time.sleep(0.05)
			self.running = False
			self.manual_mode = True
			self.task.cancel()
			time.sleep(pm.SERIAL_device_poll)
		#
	# end toggle_manual_mode

	def set_piezo(self, p):
		if not self.manual_mode:
			print("Error mira_900_OPO: OPO must be in manual mode to set piezo voltage")
			return
		if p < 0.0 or p > 4095.0:
			print("Error mira_900_OPO: Piezo voltage must be between 0 and 4095")
			return
		if self.ser.isOpen():
			try:
				command = "P" + str(int(p)) + self.term
				self.ser.write(command.encode('utf-8'))
				time.sleep(0.25) # Give the system some time to respond
			except Exception as e:
				print(str(self.name) + " serialcom.set_piezo Error: Could not set")
				print(str(e))
		#
	# set_piezo



	'''
	Sets the wavelength (in nm)

	BREAKS THE SERIAL PORT IF THE WAVELENGTH FAILS TO SET
	'''
	def set_wavelength(self, w):
		if self.running:
			loopcount = 0
			while(self.reading):
				#print "Here"
				loopcount += 1
				if loopcount > 50:
					print("Break out")
					print("Error serialcom.set_wavelength: Reading thread blocking port")
					#self.ser.close()
					self.reset()
					break
				time.sleep(0.05)
			self.task.cancel()
			time.sleep(0.1)
			cmd = "SL" + str(round(w*10))
			if self.ser.isOpen():
				loopcount = 0
				try:
					b = str(cmd) + self.term
					self.ser.write(b.encode('utf-8'))
					time.sleep(1)
					while self.ser.inWaiting() != 2:
						if loopcount > 300:
							print("Error serialcom.set_wavelength: No response, wavelength not locked to " + str(w))
							#self.ser.close()
							self.reset()
							break
						time.sleep(0.1)
						loopcount += 1
					self.ser.flushInput()
				except Exception as e:
					print("Error serialcom.set_wavelength: Could not set Wavelength")
					print(str(e))
			self.task = threading.Timer(pm.SERIAL_device_poll, self.read_data_call)
			self.task.start()
	# end set_output_setting

	'''
	Sends a request for data to the serial device and listens for a response
	Returns none and prints to terminal if no response was received
	'''
	def get(self, command, nbytes):
		if self.ser.isOpen():
			out = ""
			loopcount = 0
			try:
				b = str(command) + self.term
				self.ser.write(b.encode('utf-8'))
				l = self.ser.inWaiting()
				while l != nbytes:
					if loopcount > 300:
						print("Error serialcom.get: No response")
						self.err_cnt += 1
						self.reset()
						break
					time.sleep(0.1)
					loopcount += 1
					l = self.ser.inWaiting()
				else:
					temp = bytearray(nbytes)
					self.ser.readinto(temp)
					out = array.array('B',temp)
			except Exception as e:
				print(str(self.name) + " serialcom.get Error: Could not get data")
				print(str(format_exc()))
				self.err_cnt += 1
			if out == "":
				return None
			else:
				return out
	# end get_data

	'''
	Reads data from the controller
	'''
	def read_data(self):
		try:
			if self.serial_open:
				pz = self.get("GZ", 2)
				l = self.get("GSV", 8)
				if all(i!= None for i in [pz,l]):
					self.piezo = float(hl2int(pz))
					self.wavelength = 0.1*float(hl2int(l[0:2]))
					self.fwhm  = 0.1*float(hl2int(l[2:4]))
		except Exception as e:
			print("Error in mira_900_OPO.serialcom.read_data, could not read serial data")
			print(e)
			self.err_cnt += 1
		# end read_data
	# end read_data
# end mira_900_OPO
