#!/usr/bin/env python
import roslib
roslib.load_manifest('lab2')
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
import numpy as np
import random
from numpy import ones,vstack
from numpy.linalg import lstsq
import math


def distance(m,c,p,q):
	d=abs((m*p)+(-1*q)+c)
	d=d/math.sqrt(m*m+1)
	return d 

def regression(x,y):
	n=len(x)
	max=0
	max1=0
	max2=0
	for i in range(0,n-1):
		for j in range(0,n-1):	
			dist = math.hypot(x[i]-x[j], y[i]- y[j])
			if dist>max:
				max=dist
				max1=i
				max2=j
	return max1,max2
					
			

def callback(data):
	
	pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
	
	pub2 = rospy.Publisher('/visualization_marker',Marker, queue_size = 100)
	#uint32_t shape = visualization_msg.Marker.CUBE
	
	marker = Marker()
	marker.header.frame_id = "/base_link"
	marker.header.stamp = rospy.Time.now()
	marker.ns="lines"
	
	marker.type=marker.LINE_LIST
	marker.action = marker.ADD
	
	marker.pose.orientation.x = 0.0
	marker.pose.orientation.y = 0.0
	marker.pose.orientation.z = 0.0
	marker.pose.orientation.w = 1.0
	marker.scale.x = 0.1
	#marker.scale.y = 0.1
	#marker.scale.z = 1.0
	marker.color.g = 1.0
        marker.color.a = 1.0	
	
		

	rangelist=[]
	for i in range(121,240):
		rangelist.append(data.ranges[i])
				
	if min(rangelist)<=1.4:
		pub.publish(Twist(Vector3(0,0,0),Vector3(0,0,400)))
	else:
		pub.publish(Twist(Vector3(2,0,0),Vector3(0,0,0)))
	
	r=[]
	theta=[]
	x=[]
	y=[]	
	x3=[]
	y3=[]
	angle=-1.57079
	for i in range(0,359):
		r.append(data.ranges[i])
				
		theta.append(angle)
		angle=angle+0.0087266
		
		x2,y2=pol2cart(r[i],theta[i])
		x.append(x2)
		y.append(y2)
		
		if r[i]<2.5:
			x3.append(x2)   #filtered x
			y3.append(y2)	#filtered y	
			
			
		
	n=len(x3)
	if n!=0:
		max=0
		for j in range(0,15):  #for n iterations		
			ran1=random.randint(0,n-1)
			ran2=random.randint(0,n-1)
			if ran1!=ran2:
				points=[(x3[ran1],y3[ran1]),(x3[ran2],y3[ran2])]
				x_coords, y_coords = zip(*points)
				A = vstack([x_coords,ones(len(x_coords))]).T
				m, c = lstsq(A, y_coords)[0]
				#print "Line Solution is y = {m}x + {c}".format(m=m,c=c)
				inliers=0
				x4=[]
				y4=[]
				for i in range(0,n-1):
					dist=distance(m,c,x3[i],y3[i])				
					if dist<0.5:
						inliers=inliers+1
						x4.append(x3[i])
						y4.append(y3[i])
			
				if inliers>max:
					max=inliers
					x5=x4[:]
					y5=y4[:]
					
		if ran1!=ran2:
			jmax1,jmax2=regression(x5,y5)
			marker.id=1

			sixPoints=[]	
			p=Point()
			p.x=x5[jmax1]
			p.y=y5[jmax1]
			p.z=0
			sixPoints.append(p)	
			p2=Point()
			p2.x=x5[jmax2]
			p2.y=y5[jmax2]
			p2.z=0
			sixPoints.append(p2)
			marker.points=sixPoints
			pub2.publish(marker)
				
					





	
		
		
		

	
def pol2cart(rt, tt):
    x = rt * np.cos(tt)	
    y = rt * np.sin(tt)
    return(x, y)	
	
def velocitor():
	pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
	rospy.init_node('evader',anonymous=True)
	rate = rospy.Rate(10) 
	sub = rospy.Subscriber('/base_scan',LaserScan,callback)
		
	


	
		
if __name__ =="__main__":
	try:
		velocitor()
	except rospy.ROSInterruptException:
        	pass
	
	


	
	rospy.spin()
	
