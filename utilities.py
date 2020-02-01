'''
utilities.py

Various useful functions for hyperDAQ

Last Updated: January 2020

|  Trevor Arp
|  Gabor Lab
|  University of California, Riverside

All Rights Reserved
'''
import parameters as pm

from scipy.ndimage.interpolation import shift
import numpy as np
import time
from tkinter import END

def indexof(list, item):
	'''
	Searches list and returns the index of item. Returns -1 if it can't find it.
	Compares as strings
	'''
	for i in range(len(list)):
		if list[i] == item :
			return i
	print("utilities.indexof() Error: Could not locate given item: " + str(item))
	return -1
# end indexof

def type_index(type):
	'''
	Searches the global parameter DATA_file_types and returns the index corresponding to
	the input type. If it can't find the type returns -1
	'''
	for i in range(len(pm.DATA_file_types)):
		if pm.DATA_file_types[i] == str(type) :
			return i
	print("utilities.type_index() Error: Could not locate given type")
	return -1
# end type_index

def tryfloat(value):
	'''
	Attempts to convert input to a float, returns None if it cannot
	'''
	try:
		return float(value)
	except:
		return None
# end try float

def isPosInt(n):
	'''
	Returns true is the number is a positive integer, false otherwise
	'''
	try:
		x = int(n)
		return x > 0
	except Exception as e:
		return False
# end isPosInt

def hl2int(d):
	'''
	Converts a high and low byte into an integer
	Input is [high_Byte, low_Byte]
	'''
	return 256*d[0] + d[1]
#

def dequeue_all(q):
	'''
	Removes all elements from the given data queue, returns data in a single array

	Args:
		q (Queue) : the input queue, containing data as numpy arrays
	'''
	try:
		d = q.get()
		rows, cols = d.shape
		n = q.qsize()
		data = np.zeros(((5+n)*rows, cols))
		m, mcols = data.shape
		start = 0
		data[0:rows,:] = d
		start += rows
		while(not q.empty()):
			d = q.get()
			rows, cols = d.shape
			if start+rows-1 >= m:
				n = q.qsize()
				data = np.append(data, np.zeros(((10+n)*rows, cols)), axis=0)
				m, mcols = data.shape
			data[start:start+rows,:] = d
			start += rows
		return data[0:start,:]
	except Exception as e:
		print("Utilities.dequeue_all : Could not read Queue")
		print(str(e))
# end dequeue_all

def dequeue_n(q, n):
	'''
	Removes n elements from the given data queue, and concatenates them, will
	return if the queue is empty even if n elements have not been collected

	Args:
		q (Queue) : the input queue, containing data as numpy arrays
	'''
	try:
		d = q.get()
		cnt = 1
		while(cnt < n and (not q.empty())):
			d = np.append(d,q.get(),axis=0)
			cnt += 1
		return d
	except Exception as e:
		print("Utilities.dequeue_all : Could not read Queue")
		print(str(e))
# end dequeue_all

def dequeue_str(q):
	'''
	Removes all elements from the given data queue

	Args:
		q (Queue) : the input queue, containing data as strings
	'''
	try:
		d = []
		while (not q.empty()):
			s = str(q.get())
			d.append(s)
		return d
	except Exception as e:
		print("Utilities.dequeue_str : Could not read Queue")
		print(str(e))
# end dequeue_str

def remove_values_from_list(the_list, val):
	'''
	Removes all of a value from a list
	'''
	return [value for value in the_list if value != val]
# remove_values_from_list

def changeEntry(entry, v):
	'''
	Takes a Tkinter Entry object, deletes the old entry and inserts the given value
	'''
	entry.delete(0, END)
	entry.insert(0, float(v))
# end changeEntry

def checkEntryStar(entry):
	'''
	Checks if the value of a tkinter entry contains '*', returns '*' is true, returns the
	string if false
	'''
	s = entry.get()
	if '*' in s:
		return '*'
	else:
		return s
# end checkEntryStar

def brute_diff_min(d1, d2, maxdrift=15):
	'''
	Brute Force approach to drift correction of reflection images, find the minimum difference between
	the (normalized) two images by brute force.

	Args:
		d1 (numpy array) : the prime image
		d2 (numpy array) : the image to correct
		maxdrift=15, the maximum number of pixels to consider

	Returns the shift that maps d2 onto d1 with the minimum difference
	'''
	rows, cols = d1.shape
	if d2.shape != (rows, cols):
	    raise ValueError("Images must be the same size")

	# Normalize the images
	d1 = d1 - np.min(d1)
	d1 = d1/np.max(d1)
	d2 = d2 - np.min(d2)
	d2 = d2/np.max(d2)

	mxdiff = 1e98
	sft = (1,0)
	delta = np.arange(-1.0*maxdrift, maxdrift+1, 1)
	N = len(delta)
	for i in range(N):
	    for j in range(N):
	        ds = shift(d2,(delta[i], delta[j]))

	        if delta[j] < 0:
	            x1 = 0
	            x2 = int(cols + delta[j])
	        else:
	            x1 = int(delta[j])
	            x2 = cols

	        if delta[i] < 0:
	            y1 = 0
	            y2 = int(rows + delta[i])
	        else:
	            y1 = int(delta[i])
	            y2 = rows

	        diff = np.mean(np.abs(d1[y1:y2, x1:x2]-ds[y1:y2, x1:x2]))
	        if diff < mxdiff:
	            mxdiff = diff
	            sft = (delta[i], delta[j])
	return sft
# end brute_diff_min

class Stopwatch():
	'''
	A stopwatch for timing peripheral functions (i.e. those not from the DAQ card)
	Based off of system time but can be synchronized by calling the zero() method

	11/8/2017 time.clock replaced with time.perf_counter, due to depriciation of time.clock in python 3
	'''
	def __init__(self):
		self.start_time = time.perf_counter()
	#
	def time(self):
		return time.perf_counter() - self.start_time
	#
	def zero(self):
		self.start_time = time.perf_counter()
	#
# end Stopwatch
