'''

writing.py

Data writing for GaborDAQ programs

Last updated: January 2020 by Trevor Arp

Gabor Lab
University of California, Riverside
All Rights Reserved
'''

import parameters as pm

import numpy as np
import os
import time
from hyperdaq.utilities import indexof

class data_scan_writer:
	'''
	Handles data writing of scans based on data images

	$images is the data image objects, (or derived classes) corresponding to the scan data to be
	written out

	$images_ext are the file extensions for the image objects in $images
	'''
	def __init__(self, images, images_ext, buffer=True):
		self.recording = False
		self.record_after = False
		self.databuffer = None
		self.buffering_datacube = False
		self.data_files = None
		self.scan_log = None
		self.run_number = "             "
		self.scan_buffer = []
		self.last_buffer = []
		self.buffer = buffer

		# set the data queues
		self.images = images
		self.images_ext = images_ext
		if len(self.images_ext) != len(self.images):
			print("Warning data_image_writer.__init__ : Number of images given does not match number of file extensions given")
		self.file_types = ['log']
		self.file_types.extend(self.images_ext)
		self.mainimage = self.images[indexof(self.images_ext, 'pci')]
	# end init

	def set_gui_ref(self, ref):
		'''
		Sets a reference of the gui
		'''
		self.gui_ref = ref
	# end set_gui_ref

	def init_data_cube(self, n):
		'''
		Starts buffering a data cube
		'''
		self.buffering_datacube = True
		r,c = np.shape(self.mainimage.data)
		self.databuffer = []
		for i in range(len(self.images)):
			self.databuffer.append( np.zeros((r,c,n)) )
		self.databufferix = 0
	#

	def buffer_cube(self):
		'''
		Buffer images into data cube
		'''
		for i in range(len(self.images)):
			self.databuffer[i][:,:,self.databufferix] = self.images[i].data
		self.databufferix += 1
	#

	def write_images(self, aux=''):
		'''
		Records the images
		'''
		if self.recording:
			if self.buffering_datacube:
				self.data_files = self.init_output_files(openfiles=False, aux=aux)
			else:
				self.data_files = self.init_output_files(aux=aux)

			# Write out the Scan info
			self.scan_log = self.data_files[indexof(self.file_types, 'log')]
			self.scan_log.writelines(self.scan_buffer)
			self.last_buffer = self.scan_buffer
			self.scan_buffer = []

			# Record the images
			if self.buffering_datacube:
				for img in range(len(self.images)):
					np.save(self.file_path_date + "_" + str(self.images_ext[img]), self.databuffer[img])
				self.databuffer = None
				self.buffering_datacube = False
			else:
				for img in range(len(self.images)):
					out = self.data_files[indexof(self.file_types, self.images_ext[img])]
					d = self.images[img].data
					for a in d:
						out.write("\t".join(str(x) for x in a) + "\n")
					out.flush()
			self.close_files()
			self.gui_ref.display_Text("Images Saved as " + str(self.run_number))
		else: # Else just clear the buffer
			self.last_buffer = self.scan_buffer
			self.scan_buffer = []
		#
	# write_images

	def write_images_aux_data(self, aux_data, aux_type):
		'''
		Saves images using write_images and saves extra data aux_data to a file with the same run number
		and a file type given by aux_type
		'''
		message = self.write_images(aux=aux_type)
		if message is not "":
			# write out the aux file
			out = open(pm.DATA_dir_path + self.run_number +  '_' + str(aux_type) + '.' + str(pm.DATA_file_ext),'a+')
			for a in aux_data:
				out.write("\t".join(str(x) for x in a) + "\n")
			out.flush()
			out.close()
		return message
	#

	def write_current_images(self):
		'''
		Records the current images and scan data
		'''
		if self.buffering_datacube:
			self.data_files = self.init_output_files(openfiles=False)
		else:
			self.data_files = self.init_output_files()

		# Write out the Scan info
		self.scan_log = self.data_files[indexof(self.file_types, 'log')]
		self.scan_log.writelines(self.last_buffer)
		self.scan_log.writelines(self.scan_buffer)
		for l in self.scan_buffer:
			self.last_buffer.append(l)
		self.scan_buffer = []

		# Record the images
		if self.buffering_datacube:
			for img in range(len(self.images)):
				np.save(self.file_path_date + "_" + str(self.images_ext[img]), self.databuffer[img])
			self.databuffer = None
			self.buffering_datacube = False
		else:
			for img in range(len(self.images)):
				out = self.data_files[indexof(self.file_types, self.images_ext[img])]
				d = self.images[img].data
				for a in d:
					out.write("\t".join(str(x) for x in a) + "\n")
				out.flush()
		self.close_files()
		self.gui_ref.display_Text("Images Saved as " + str(self.run_number))
	# write_current_images

	def close_files(self):
		'''
		Closes all files
		'''
		files=self.data_files
		if files != None:
			for f in files:
				f.flush()
				f.close()
			self.data_files = None
	# end close_files

	def write_params_file(self):
		'''
		Writes out parameters file variables
		'''
		import parameters as module
		book = {key: value for key, value in module.__dict__.items() if not (key.startswith('__') or key.startswith('_'))}
		for k, v in book.items():
			self.scan_buffer.append(str(k) + ": " + str(v) + "\n")
	# end write_params_file

	def init_log_file(self, f, run="", aux=''):
		'''
		Initializes the log file, writing the run number to log
		Takes a file object as a parameter
		'''
		f.write("Run Number: " + str(run) + '\n')
		ds = []
		for i in range(len(self.file_types)):
			if self.file_types[i] != 'log':
				ds.append(str(self.file_types[i]))
		if aux != '':
			ds.append(str(aux))
		f.write("Data Files: " + ','.join(ds) + '\n')
		f.flush()
		return f
	# end init_log_file

	def init_output_files(self, openfiles=True, aux=''):
		'''
		Initialize output files for writing
		Returns a list of files, with types given by self.file_types,
		'''
		run = 0
		out_files = []
		out_file_dir = os.path.join(pm.DATA_dir_path, time.strftime("%Y"), time.strftime("%Y_%m"), time.strftime("%Y_%m_%d"))
		if not os.path.exists(out_file_dir):
			os.makedirs(out_file_dir)
		out_file_dir = os.path.join(out_file_dir, pm.SYSTEM_prefix + '_' + time.strftime("%Y_%m_%d_"))
		while os.path.isfile(out_file_dir + str(run) + '_' + str(self.file_types[0]) + '.log'):
			run += 1
		self.file_run = run
		self.file_path_date = out_file_dir + str(run)
		self.run_number = pm.SYSTEM_prefix + '_' + time.strftime("%Y_%m_%d_") + str(run)
		for type in self.file_types:
			if str(type) == "log":
				out_files.append(open(out_file_dir + str(run) + '_' + str(type) + '.log','a+'))
				out_files[len(out_files)-1] = self.init_log_file(out_files[len(out_files)-1], run=self.run_number, aux=aux)
			else:
				if openfiles:
					out_files.append(open(out_file_dir + str(run) +  '_' + str(type) + '.' + str(pm.DATA_file_ext),'a+'))
		return out_files
	# end init_output_files

	def toggle_record(self):
		'''
		Turns recording on and off
		'''
		if self.recording:
			self.close_files()
			self.data_files = None
			self.scan_log = None
			self.run_number = "             "
			self.recording = False
		else:
			self.recording = True
	# end toggle_record

	def log_param(self, k, v):
		'''
		Records the value of a parameter as "key:value"
		'''
		self.scan_buffer.append(str(k) + ": " + str(v) + "\n")
	# end log_param

	def log_str(self, txt):
		'''
		Records a message to the log file, buffers the change message if not recording
		$txt is the text to be written to the log file
		'''
		message = str(txt) + "\n"
		self.scan_buffer.append(message)
	# end log_str
# end data_scan_writer
