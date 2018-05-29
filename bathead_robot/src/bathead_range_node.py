#!/usr/bin/env python
# license removed for brevity
import rospy
import RPi.GPIO as gpio
import time
import subprocess
import numpy as np
import scipy.signal
from std_msgs.msg import Float64

# Pin selection
chirp_trigger_pin = 11
gnd_pin = 36

# Smoothed z-score peak detection parameters
lag = 20
threshold = 2
influence = 0

# Audio signal parameters
fs = 625000 # Sampling rate in Hz
#fs = 625 # Sampling rate in kHz(samples/ms)
downsample = 100

# GPIO setup
gpio.setmode(gpio.BCM)
gpio.setup(chirp_trigger_pin, gpio.OUT)
gpio.setup(gnd_pin, gpio.OUT)
gpio.output(gnd_pin, gpio.LOW)

# Functions
def movMean(a, n) :
    ret = np.cumsum(a, dtype=float)
    ret[n:] = ret[n:] - ret[:-n]
    return ret[n - 1:] / n
    
def findSignals(x):
	ch = np.zeros((x.size/4,4,), dtype=np.dtype(np.int16))
	y = np.zeros((x.size/4/downsample + 1,4,), dtype=np.dtype(np.int16))
	
	variance = np.zeros(4)
	i = 0
	while (i < 4):
		ch[:,i] = x[i::4]
		y[:,i] = ch[:,i][::downsample]
		variance[i] = np.var(y[:,i])
		i = i + 1

	# Find sorting index of variance matrix, reverse sort order and select first two elements
	sort = np.argsort(variance)[::-1][[0,1]]
	if ( np.abs(sort[0] - sort[1]) == 1 ): # Channels 0 & 1, 1 & 2, 2 & 3
		ind_left = np.min(sort)
		ind_right = np.max(sort)
	else: # Channels 3 & 0
		ind_left = np.max(sort)
		ind_right = np.min(sort)
		
	#print 'Signals found on channels', ind_left+1, 'and', ind_right+1
	
	y_left = y[:, ind_left]
	y_right = y[:, ind_right]
	
	return y_left, y_right

def smoothedZScore(y, lag, threshold, influence = 0):
	t_smooth = time.time()
	y = y.astype(float)
	y = np.divide(y,32768)
	y = np.add(y,1)
	y[y>1] = y[y>1] - 2
	b, a = scipy.signal.butter(1,.32, btype='highpass')
	y = scipy.signal.filtfilt(b,a,y)
	y = y - np.median(y)
	y = np.abs(y)
	y = y / np.max(y)
	y = movMean(y, 2)
	#print 'Preprocessing time:', time.time()-t_smooth
	#pylab.subplot(211)
	#pylab.plot(np.arange(1, len(y)+1), y)
	
	signals = np.zeros(len(y))
	filteredY = np.array(y)
	avgFilter = [0]*len(y)
	stdFilter = [0]*len(y)
	avgFilter[lag - 1] = np.mean(y[0:lag])
	stdFilter[lag - 1] = np.std(y[0:lag])
	i = lag
	while ( i < len(y) ):
		if (abs(y[i] - avgFilter[i-1]) > (threshold * stdFilter [i-1] + .1)):
			signals[i] = 1

			filteredY[i] = influence * y[i] + (1 - influence) * filteredY[i-1]
			avgFilter[i] = np.mean(filteredY[(i-lag):i])
			stdFilter[i] = np.std(filteredY[(i-lag):i])
			i = i + lag
		else:
			signals[i] = 0
			filteredY[i] = y[i]
			avgFilter[i] = np.mean(filteredY[(i-lag):i])
			stdFilter[i] = np.std(filteredY[(i-lag):i])
		i = i + 1;

	#print 'Smoothed Z-score time:', time.time() - t_smooth

	return dict(signals = np.asarray(signals),
				avgFilter = np.asarray(avgFilter),
				stdFilter = np.asarray(stdFilter))
				
def peakDistance(y, fs):
	# Time per sample: 1/(625000 s^-1/100) = 0.16 ms
	# Distance between samples: .343 m/ms * .16 ms = 0.055 m (5.5 cm)
	# Time for 20 samples: 0.16 ms/sample * 20 samples = 3.2 ms
	# Obstacle distance at 3.2 ms: 3.2 ms * .343 m/ms / 2 = .55 m (55 cm)
	chirp = False
	i = 0
	t0 = 0
	t1 = len(y)/float(fs)
	cooldown = 100
	while i < len(y):
		if ( ((y[i] == 1.0) and (y[i-1] == 0.0)) and (cooldown > lag) ): # Positive edge and 3.2ms after last positive edge
			if (not chirp): # Chirp has not been found until now (this is the chirp)
				t0 = i/float(fs) # Save index of chirp
				#print 't0:',t0 * 1000
				cooldown = 0; #i = i + fs * 3 # Jump 3ms over chirp
				chirp = True
			elif (chirp): # Chirp has already been found (this is the echo)
				t1 = i/float(fs)
				#print 't1:',t1 * 1000
				break
		i = i+1
		cooldown = cooldown + 1
	return (t1-t0)*343/2.0

def batheadRange():
	pub_left = rospy.Publisher('bathead/range/left', Float64, queue_size=1)
	pub_right = rospy.Publisher('bathead/range/right', Float64, queue_size=1)
	rospy.init_node('bathead_range')
	rate = rospy.Rate(3) # Hz
	
	while not rospy.is_shutdown():
		
		# Trigger chirp
		gpio.output(chirp_trigger_pin, gpio.HIGH)
		time.sleep(.0001) # .1 ms
		gpio.output(chirp_trigger_pin, gpio.LOW)
		
		# Trigger snapshot
		subprocess.call('trig -s tcp://localhost:7777 --pre=100 --post=0 snap', shell=True)
	
		# Wait for snapshot to finish writing: repeatedly check snapchat ztatus for 'FIN' string
		ztatus = subprocess.check_output('snapchat z', shell=True)
		while (not ztatus.find('FIN')):
			time.sleep(.001) # Wait 1 ms and try again
			ztatus = subprocess.check_output('snapchat z', shell=True)

		# Get name of latest file in snaps folder
		snap_filename = subprocess.check_output('ls -t /tmp/snap | head -n1', shell=True)
		snap_filename = snap_filename[0:len(snap_filename)-1] # Remove newline character

		# Read audio
		y = np.fromfile('/tmp/snap/' + snap_filename, dtype=np.dtype(np.int16))
		[y_left, y_right] = findSignals(y);
		#y_left = y[0::4] # TODO Check input channels
		#y_right = y[1::4] # TODO Check input channels
		#y_3 = y[2::4]
		#y_4 = y[3::4]		
		
		# Downsample audio to speed up peak detection
		#y_left = y_left[::downsample]
		#y_right = y_right[::downsample]

		# Detect peaks
		peaks_left = smoothedZScore(y_left, lag=lag, threshold=threshold, influence=influence)
		peaks_right = smoothedZScore(y_right, lag=lag, threshold=threshold, influence=influence)

		# Estimate distance to target
		dist_left = peakDistance(peaks_left['signals'], fs / downsample)
		dist_right = peakDistance(peaks_right['signals'], fs / downsample)
		
		dist_max = 1.5;
		
		# Clip range at dist_max meters
		if (dist_left > dist_max):
			dist_left = dist_max;
		if (dist_right > dist_max):
			dist_right = dist_max;
		
		# Publish range estimations to ROS topic
		pub_left.publish( dist_left / dist_max )
		pub_right.publish( dist_right / dist_max )
		
		#print '[bathead_range] Publishing left:', dist_left, '\tright', dist_right
		rospy.loginfo('%f %f', dist_left / dist_max, dist_right / dist_max)
		#print '[bathead_range] Difference:', dist_left - dist_right
		
		rate.sleep()

if __name__ == '__main__':
	try:
		batheadRange()
	except rospy.ROSInterruptException:
		pass
