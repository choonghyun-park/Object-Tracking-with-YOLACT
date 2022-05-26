#!/usr/bin/env python
import numpy as np 
import cv2
from tracker import Tracker
import time
import rospy
from yolact_ROS.msg import Segment_centers
from yolact_ROS.msg import obj_tracking
# from mot.msg import obj_tracking
images = []

''' Segment_centers.msg
Header header
int16[] id
int16[] pixel_x
int16[] pixel_y
float64[] x : Moving right in respect to camera
float64[] y : Moving up in respect to camera
float64[] z : Moving away from camera
float64[] secs
'''

''' obj_tracking.msg
Header header
int16 object_id
float64 x
float64 y
float64 z 
float64 vx
float64 vy
float64 vz
'''

tracker = Tracker(150, 10, 5) # dist_threshold : for 150 pixel distance can be same obj, max_frame_skipped : frame time-to-live, max_trace_length : trace until num

def createimage(w,h):
	# shape = (h, w, 1)
	img = np.ones((h,w,3),np.uint8)*255
	return img

def vel(current,before):
	duration = current[3]-before[3]
	vx = (current[0]-before[0])/duration
	vy = (current[1]-before[1])/duration
	vz = (current[2]-before[2])/duration
	v = (rd(vx),rd(vy),rd(vz))
	return v

def rd(f):
	mul = 10000
	if f<0:
		return 0
	ret = f*mul
	ret = round(ret)
	ret /= mul
	return ret

def mapping(x,z,window):
	# 1 pixel = 0.01m = 10mm
	bias_x = window[0]/2
	bias_z = window[1]-40
	cv_x = int(x*100) + bias_x
	cv_z = -int(z*100) + bias_z
	return cv_x, cv_z

def print_position(str,position):
	print(str)
	print("x :",position[0])
	print("y :",position[1])
	print("z :",position[2])
	print("sec :",position[3])

def bird_eyed_view_tracking(msg):
	# Segment_centers must be a center points of each frame
	centers_list = []
	states_list = []
	msg_id = msg.id
	msg_pixel_x = msg.pixel_x
	msg_pixel_y = msg.pixel_y
	msg_x = msg.x
	msg_y = msg.y
	msg_z = msg.z
	msg_secs = msg.secs
	window = (960,720)

	if len(msg_id)==0: # published none value msg
		return

	for i in range(len(msg_x)):
		if msg_id[i] != 0: # Only detect for person
			continue
		if msg_x[i]==0 and msg_y[i]==0 and msg_z[i]==0: # Detecting errors in depth camera
			continue
		cv_x, cv_z = mapping(msg_x[i],msg_z[i],window)
		centers_list.append([cv_x, cv_z])
		states_list.append([msg_x[i], msg_y[i], msg_z[i], msg_secs[i]])
	centers = np.array(centers_list) 
	states = np.array(states_list)

	global tracker

	skip_frame_count = 0
	track_colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0),
					(127, 127, 255), (255, 0, 255), (255, 127, 255),
					(127, 0, 255), (127, 0, 127),(127, 10, 255), (0,255, 127)]

	frame = createimage(window[0],window[1]) # height, width
	
	tracker.update(centers, states)

	# draw axis
	cv2.line(frame, (window[0]/2,0),(window[0]/2,window[1]), (0,0,0),1) 
	cv2.line(frame, (window[0]/2,0),(window[0]/2-10,10), (0,0,0),1)
	cv2.putText(frame,"z(m)", (window[0]/2+20,20),0, 0.5, (0,0,0),2)
	cv2.line(frame, (0,window[1]-40),(window[0],window[1]-40), (0,0,0),1)
	cv2.line(frame, (window[0]-10,window[1]-30),(window[0],window[1]-40), (0,0,0),1)
	cv2.putText(frame,"x(m)", (window[0]-40,window[1]-20),0, 0.5, (0,0,0),2)

	# draw units on axis in 1 meter (1pixel = 10mm, 100 pixel = 1m)
	for i in range(int(window[0]/100)):
		cv2.line(frame, (window[0]/2+i*100,window[1]-50),(window[0]/2+i*100,window[1]-30), (0,0,0),1)
		cv2.line(frame, (window[0]/2-i*100,window[1]-50),(window[0]/2-i*100,window[1]-30), (0,0,0),1)
	for i in range(int(window[1]/100)):
		cv2.line(frame, (window[0]/2-10,window[1]-40-i*100),(window[0]/2+10,window[1]-40-i*100),(0,0,0),1)

	for j in range(len(tracker.tracks)):
		if (len(tracker.tracks[j].trace) > 1):
			# bird-eyed-view visualization
			cv_x = int(tracker.tracks[j].trace[-1][0,0])
			cv_z = int(tracker.tracks[j].trace[-1][0,1])
			cv_tl = (cv_x-10,cv_z-10)
			cv_br = (cv_x+10,cv_z+10)
			for k in range(len(tracker.tracks[j].trace)):
				x_k = tracker.tracks[j].trace_state[k][0,0]
				z_k = tracker.tracks[j].trace_state[k][0,2]
				cv_x_k, cv_z_k = mapping(x_k, z_k,window)	

				cv2.circle(frame,(cv_x_k,cv_z_k), 3, track_colors[j],-1)
			
			state_current = tracker.tracks[j].trace_state[-1]
			position_current = (state_current[0,0],state_current[0,1],state_current[0,2],state_current[0,3])

			state_before = tracker.tracks[j].trace_state[-2]
			position_before = (state_before[0,0],state_before[0,1],state_before[0,2],state_before[0,3])

			# print_position("position_current",position_current)
			# print_position("position_before",position_before)
			velocity = vel(position_current,position_before)

			# publish infos 
			# print("=============================")
			# print(tracker.tracks[j].trackId)
			# print(position_current)
			# print(velocity)

			msg = obj_tracking()
			msg.object_id = tracker.tracks[j].trackId
			msg.x = position_current[0]
			msg.y = position_current[1]
			msg.z = position_current[2]
			msg.vx = velocity[0]
			msg.vy = velocity[1]
			msg.vz = velocity[2]
			pub.publish(msg)

			cv2.rectangle(frame,cv_tl,cv_br,track_colors[j],1)
			cv2.putText(frame,str(tracker.tracks[j].trackId), (cv_x-10,cv_z-20),0, 0.5, track_colors[j],2)
			cv2.putText(frame,"vel = ("+str(velocity[0])+", "+str(velocity[2])+")", (cv_x+10,cv_z-20),0, 0.5, track_colors[j],2)
			cv2.circle(frame,(cv_x,cv_z), 3, (0,0,0),-1)

	cv2.imshow('image',frame)
		
	if cv2.waitKey(1) & 0xFF == ord('q'):
		cv2.destroyAllWindows()

def forward_view_tracking(msg):
	# Segment_centers must be a center points of each frame
	centers_list = []
	states_list = []
	msg_id = msg.id
	msg_pixel_x = msg.pixel_x
	msg_pixel_y = msg.pixel_y
	msg_x = msg.x
	msg_y = msg.y
	msg_z = msg.z
	msg_secs = msg.secs

	if len(msg_id)==0: # published none value msg
		return

	for i in range(len(msg_x)):
		if msg_id[i] != 0: # Only detect for person
			continue
		if msg_x[i]==0 and msg_y[i]==0 and msg_z[i]==0: # Detecting errors in depth camera
			continue
		centers_list.append([msg_pixel_x[i],msg_pixel_y[i]])
		states_list.append([msg_x[i], msg_y[i], msg_z[i], msg_secs[i]])
	centers = np.array(centers_list) 
	states = np.array(states_list)

	global tracker

	skip_frame_count = 0
	track_colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0),
					(127, 127, 255), (255, 0, 255), (255, 127, 255),
					(127, 0, 255), (127, 0, 127),(127, 10, 255), (0,255, 127)]
	window = (640,480)
	frame = createimage(window[0],window[1])
	
	tracker.update(centers, states)
	
	for j in range(len(tracker.tracks)):
		if (len(tracker.tracks[j].trace) > 1):
			vel_flag=False
			pixel_x = int(tracker.tracks[j].trace[-1][0,0])
			pixel_y = int(tracker.tracks[j].trace[-1][0,1])

			x = tracker.tracks[j].trace_state[-1][0,0]
			y = tracker.tracks[j].trace_state[-1][0,1]
			z = tracker.tracks[j].trace_state[-1][0,2]
			sec = tracker.tracks[j].trace_state[-1][0,3]
			current = (x,y,z,sec)
			current_rd = (rd(x),rd(y),rd(z),rd(sec%100))

			pixel_x_0 = int(tracker.tracks[j].trace[-2][0,0])
			pixel_y_0 = int(tracker.tracks[j].trace[-2][0,1])

			x_b = tracker.tracks[j].trace_state[-2][0,0]
			y_b = tracker.tracks[j].trace_state[-2][0,1]
			z_b = tracker.tracks[j].trace_state[-2][0,2]
			sec_b = tracker.tracks[j].trace_state[-2][0,3]
			before = (x_b, y_b,z_b,sec_b)

			velocity = vel(current, before)

			tl = (pixel_x-10,pixel_y-10)
			br = (pixel_x+10,pixel_y+10)

			# frame based visualization
			cv2.rectangle(frame,tl,br,track_colors[j],1)
			cv2.putText(frame,str(tracker.tracks[j].trackId), (pixel_x-10,pixel_y-20),0, 0.5, track_colors[j],2)
			for k in range(len(tracker.tracks[j].trace)):
				pixel_x_k = int(tracker.tracks[j].trace[k][0,0])
				pixel_y_k = int(tracker.tracks[j].trace[k][0,1])	

				cv2.circle(frame,(pixel_x_k,pixel_y_k), 3, track_colors[j],-1)

			cv2.line(frame, (pixel_x,pixel_y),(pixel_x_0,pixel_y_0), (0,100,0),3)
			cv2.putText(frame,"vel :"+str(velocity), (pixel_x+10,pixel_y+20),0, 0.5, (0,200,0),2)
			cv2.putText(frame,"pos :"+str(current_rd),(pixel_x+10,pixel_y+40),0,0.5, (0,200,0),2)

			cv2.circle(frame,(pixel_x,pixel_y), 6, track_colors[j],-1)
			cv2.circle(frame,(pixel_x,pixel_y), 6, (0,0,0),-1)

			# If you want to see the result in bird-eyed-view, uncomment below and comment #frame based visualization
			'''
			# draw axis
			cv2.line(frame, (window[0]/2,0),(window[0]/2,window[1]), (0,0,0),1) 
			cv2.line(frame, (window[0]/2,0),(window[0]/2-10,10), (0,0,0),1)
			cv2.putText(frame,"z(m)", (window[0]/2+20,20),0, 0.5, (0,0,0),2)
			cv2.line(frame, (0,window[1]-40),(window[0],window[1]-40), (0,0,0),1)
			cv2.line(frame, (window[0]-10,window[1]-30),(window[0],window[1]-40), (0,0,0),1)
			cv2.putText(frame,"x(m)", (window[0]-40,window[1]-20),0, 0.5, (0,0,0),2)

			# draw units on axis in 1 meter (1pixel = 10mm, 100 pixel = 1m)
			for i in range(int(window[0]/100)):
				cv2.line(frame, (window[0]/2+i*100,window[1]-50),(window[0]/2+i*100,window[1]-30), (0,0,0),1)
				cv2.line(frame, (window[0]/2-i*100,window[1]-50),(window[0]/2-i*100,window[1]-30), (0,0,0),1)
			for i in range(int(window[1]/100)):
				cv2.line(frame, (window[0]/2-10,window[1]-40-i*100),(window[0]/2+10,window[1]-40-i*100),(0,0,0),1)

			cv_x, cv_z = mapping(x,z,window)
			cv_tl = (cv_x-10,cv_z-10)
			cv_br = (cv_x+10,cv_z+10)
			for k in range(len(tracker.tracks[j].trace)):
				x_k = tracker.tracks[j].trace_state[k][0,0]
				z_k = tracker.tracks[j].trace_state[k][0,2]
				cv_x_k, cv_z_k = mapping(x_k, z_k, window)	

				cv2.circle(frame,(cv_x_k,cv_z_k), 3, track_colors[j],-1)
			cv2.rectangle(frame,cv_tl,cv_br,track_colors[j],1)
			cv2.putText(frame,str(tracker.tracks[j].trackId), (cv_x-10,cv_z-20),0, 0.5, track_colors[j],2)
			cv2.circle(frame,(cv_x,cv_z), 3, (0,0,0),-1)
			'''

			msg = obj_tracking()
			msg.object_id = tracker.tracks[j].trackId
			msg.x = current[0]
			msg.y = current[1]
			msg.z = current[2]
			msg.vx = velocity[0]
			msg.vy = velocity[1]
			msg.vz = velocity[2]
			pub.publish(msg)

	cv2.imshow('image',frame)
		
	if cv2.waitKey(1) & 0xFF == ord('q'):
		cv2.destroyAllWindows()
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                

if __name__ == '__main__':
	rospy.init_node('object_tracking', anonymous=True)
	pub = rospy.Publisher('object_tracking', obj_tracking, queue_size=10) 
	# rospy.Subscriber("yolact_ros", Segment_centers, forward_view_tracking)
	rospy.Subscriber("yolact_ros", Segment_centers, bird_eyed_view_tracking)
	rospy.spin()
	