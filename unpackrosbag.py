#!/usr/bin/env python3
import rosbag
import numpy as np
import pickle
import os
import tf


class Curve:
	def __init__(self):
		self.data = []
		self.time = []

	def append(self, msg, t):
		self.data.append(msg)
		self.time.append(t.to_sec())

	def plot(self, plot, m='b'):
		plot.plot(np.asarray(self.time), np.asarray(self.data),  m)


# Extract concerned data from a ROS bag
class Unpacker:
	def __init__(self, num):
		self.pathuav = "/home/dongwei/workspace/estimationwithoneuwb/hjk_feiji_"+str(num)
		self.pathugv = "/home/dongwei/workspace/estimationwithoneuwb/UGV_0_"+str(num)
		self.baguav = rosbag.Bag(self.pathuav+".bag")
		self.bagugv = rosbag.Bag(self.pathugv+".bag")
		self.data = {}
		self.data["UGV"] = {}
		self.data["UAV"] = {}
		self.minTimeUGV = 1e10
		self.minTimeUAV = 1e10

	# Load data from a pickle, or extract from a ROS bag
	def unpack(self):
		if os.path.exists(self.pathuav+".pkl"):
			f = open(self.pathuav+".pkl", 'rb')
			self.data["UAV"] = pickle.load(f)
			f.close()
			f = open(self.pathugv+".pkl", 'rb')
			self.data["UGV"] = pickle.load(f)
			f.close()
			return
		for topic, msg, t in self.baguav.read_messages():
			if topic not in self.data["UAV"]:
				self.data["UAV"][topic] = Curve()
				if t.to_sec() < self.minTimeUAV:
					self.minTimeUAV = t.to_sec()
			self.data["UAV"][topic].append(msg, t)

		for topic, msg, t in self.bagugv.read_messages():
			if topic not in self.data["UGV"]:
				self.data["UGV"][topic] = Curve()
				if t.to_sec() < self.minTimeUGV:
					self.minTimeUGV = t.to_sec()
			self.data["UGV"][topic].append(msg, t)


	# Shift the starting time to zero
	def sortTimeUAV(self, t):
		t = np.asarray(t)
		return t-self.minTimeUAV

	# Shift the starting time to zero
	def sortTimeUGV(self, t):
		t = np.asarray(t)
		return t-self.minTimeUGV

	# Fetch the ground truth data from Vicon
	# The Vicon is setup on UGV 1
	def fetchGroundTruth(self):
		curve = self.data["UGV"]["/mocap/pose"]
		t = self.sortTimeUGV(curve.time)
		x = [d.pose.position.x for d in curve.data]
		y = [d.pose.position.y for d in curve.data]
		z = [d.pose.position.z for d in curve.data]
		self.gtPosition = np.vstack((t, x, y, z))
		return self.gtPosition

	# Fetch the distance of the quadrotor, the id of UWB on the quadrotor is 3
	def fetchUGV_UWB(self, i=3):
		curve = self.data["UGV"]["/nlink_linktrack_nodeframe3"]
		t = self.sortTimeUGV(curve.time)
		dis = [d.nodes[-1].dis for d in curve.data]
		lt = [d.local_time for d in curve.data]
		st = [d.system_time for d in curve.data]
		st = np.asarray(st)/1000.0
		self.uwb_disugv1 = np.vstack((t, dis, lt, st))
		return self.uwb_disugv1

	# Fetch the distance of the quadrotor, the id of UWB on the quadrotor is 3
	def fetchUGV_Vision(self, i=3):
		curve = self.data["UGV"]["/mavros/vision_pose/pose"]
		t = self.sortTimeUGV(curve.time)
		x = [d.pose.position.x for d in curve.data]
		y = [d.pose.position.y for d in curve.data]
		z = [d.pose.position.z for d in curve.data]
		self.visualPosition = np.vstack((t, x, y, z))
		return self.visualPosition

	# Fetch the distance of the quadrotor, the id of UWB on the quadrotor is 3
	def fetchUAV_UWB(self, i=0):
		curvea = self.data["UAV"]["/nlink_linktrack_nodeframe3"]
		t = self.sortTimeUGV(curvea.time)
		disa = [d.nodes[0].dis for d in curvea.data]
		lta = [d.local_time for d in curvea.data]
		sta = [d.system_time for d in curvea.data]
		sta = np.asarray(sta)/1000.0
		self.uwb_disuav1 = np.vstack((t, disa, lta, sta))
		return self.uwb_disuav1

	def fetchIMU(self):
		curve = self.data["UAV"]["/mavros/imu/data"]
		t = self.sortTimeUAV(curve.time)
		x = [d.linear_acceleration.x for d in curve.data]
		y = [d.linear_acceleration.y for d in curve.data]
		z = [d.linear_acceleration.z for d in curve.data]
		self.gtPosition = np.vstack((t, x, y, z))
		return self.gtPosition

	def fetchAttref(self):
		curve = self.data["UAV"]["/mavros/setpoint_raw/target_attitude"]
		t = self.sortTimeUAV(curve.time)
		x = []
		y = []
		z = []

		for d in curve.data:
			eular = tf.transformations.euler_from_quaternion((d.orientation.x, d.orientation.y, d.orientation.z, d.orientation.w))
			x.append(eular[0])
			y.append(eular[1])
			z.append(eular[2])
		x = np.asarray(x)
		y = np.asarray(y)
		z = np.asarray(z)
		self.attRef = np.vstack((t, x, y, z))
		return self.attRef

	def fetchPose(self):
		curve = self.data["UAV"]["/mavros/local_position/pose"]
		t = self.sortTimeUAV(curve.time)
		x = []
		y = []
		z = []

		for d in curve.data:
			q = d.pose.orientation
			eular = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))
			x.append(eular[0])
			y.append(eular[1])
			z.append(eular[2])
		x = np.asarray(x)
		y = np.asarray(y)
		z = np.asarray(z)
		self.attitude = np.vstack((t, x, y, z))
		return self.attitude
