""" 
Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
Limited. All rights reserved.
Yan Guohang <yanguohang@pjlab.org.cn>
"""

import numpy as np
from datatypes import *

class rangeConfig:
	def __init__(self):
		# camera
		self.min_grid_pixel = 15
		self.x_frame_percent = 0.2
		# lidar
		self.max_pt_dist = 0.2 # 1/5 of circle diameter
		self.max_angle_diff = 45.0 # +- 45 degree of board angle



def deg2rad(deg):
	return deg * np.pi / 180.0


# min distance from sensor point to board point in xy plane
# camera an lidar are the same way to compute
def getMinBoardDistXY(sensor, board, thresh = 0.9, config = rangeConfig()):
	# assume that board is the same height with sensor
	# min distance computed by horizonal view
	yaw_diff = abs(sensor.yaw - board.yaw) - 180
	half_xfov = sensor.horizonal_fov / 2.0
	dmin_l = board.width / (np.tan(deg2rad(half_xfov + yaw_diff)) + 
		np.tan(deg2rad(half_xfov - yaw_diff))) / np.cos(yaw_diff)

	# min distance computed by vertical view
	pitch_diff = sensor.pitch
	half_yfov = sensor.vertical_fov / 2.0
	dmin_v = board.height / (np.tan(deg2rad(half_yfov + pitch_diff)) + 
		np.tan(deg2rad(half_yfov - pitch_diff))) / np.cos(pitch_diff)

	dmin = max(dmin_l, dmin_v) / thresh
	return dmin	


# max distance from sensor point to board point in xy plane
def getMaxBoardDistXY(sensor, board, thresh = 0.9, config = rangeConfig()):
	dmax = -1
	if (sensor.type == "camera"):
		yaw_diff = abs(sensor.yaw - board.yaw) - 180
		dmax_l = board.grid_size * np.cos(deg2rad(yaw_diff)) * sensor.img_width / \
				2.0 / np.tan(deg2rad(sensor.horizonal_fov/2)) / float(config.min_grid_pixel)
		pitch_diff = sensor.pitch
		dmax_v = board.grid_size * np.cos(deg2rad(pitch_diff)) * sensor.img_height / \
				2.0 / np.tan(deg2rad(sensor.vertical_fov/2)) / float(config.min_grid_pixel)
		dmax = min(dmax_l, dmax_v) * thresh
	elif (sensor.type == "lidar"):
		dmax = config.max_pt_dist * board.radius * 2.0 / \
			np.tan(deg2rad(sensor.angular_resolution)) * thresh
	return dmax

# aniclockwise rotation Area [0~360, 0~360]
def getBoardAngleRegionXY(sensor, board, thresh = 1.0, config = rangeConfig()):
	region = [-1, -1]
	if (sensor.type == "camera"):
		region[0] = (sensor.yaw - sensor.horizonal_fov / 2.0 * thresh) % 360
		region[1] = (sensor.yaw + sensor.horizonal_fov / 2.0 * thresh) % 360
	elif (sensor.type == "lidar"):
		board_angle = (board.yaw + 180) % 360
		board_gap = [(board_angle - config.max_angle_diff+ 360) % 360,
			(board_angle + config.max_angle_diff) % 360]
		gap = [(sensor.yaw - sensor.horizonal_fov / 2.0 + 360) % 360,
			(sensor.yaw + sensor.horizonal_fov / 2.0) % 360]
		region = gap
		if (gap[0] - gap[1] < 0):
			if (board_gap[0] < gap[1] and board_gap[0] > gap[0]):
				region[0] = board_gap[0]
			if (board_gap[1] < gap[1] and board_gap[1] > region[0]):
				region[1] = board_gap[1]
		elif (gap[0] - gap[1] > 0):
			if ((board_gap[0] - gap[1]) * (board_gap[0] - gap[0]) > 0):
				region[0] = board_gap[0]
			if ((board_gap[1] - gap[1]) * (board_gap[1] - region[0]) > 0):
				region[1] = board_gap[1]
		# angle_diff = min(sensor.horizonal_fov / 2.0, config.max_angle_diff) * thresh
		# region[0] = (sensor.yaw - angle_diff + 360) % 360
		# region[1] = (sensor.yaw + angle_diff + 360) % 360
	return region


# aniclockwise rotation Area [0~360, 0~360]
def getBoardAngleRegionSide(sensor, board, thresh = 0.9, config = rangeConfig()):
	region = [-1, -1]
	region[0] = sensor.pitch + sensor.vertical_fov / 2.0
	region[1] = sensor.pitch - sensor.vertical_fov / 2.0
	region[0] *= thresh
	region[1] *= thresh
	return region


def getBoardBestPos(sensor, board, board_region):
	yaw_diff = abs(sensor.yaw - board.yaw) - 180
	bestd = board_region.dmin + 0.3 * (board_region.dmax - board_region.dmin) * np.cos(deg2rad(yaw_diff/2.0))
	board_region.bestdist = bestd
	if (sensor.type == "camera"):
		board_region.bestpos[0] = board_region.origin_pt[0] + np.cos(deg2rad(sensor.yaw)) * bestd
		board_region.bestpos[1] = board_region.origin_pt[1] + np.sin(deg2rad(sensor.yaw)) * bestd
	elif (sensor.type == "lidar"):
		yaw = ((board_region.angle_range[1] - board_region.angle_range[0]) % 360 / 2.0 + board_region.angle_range[0]) % 360
		board_region.bestpos[0] = board_region.origin_pt[0] + np.cos(deg2rad(yaw)) * bestd
		board_region.bestpos[1] = board_region.origin_pt[1] + np.sin(deg2rad(yaw)) * bestd
	z = sensor.posz + np.sin(deg2rad(sensor.pitch)) * bestd
	if (z < board.height / 2.0):
		z = board.height / 2.0
	board_region.bestpos[2] = z


def getSensorAngleRegionXY(sensor, board, sensor_region, thesh = 1.0, config = rangeConfig()):
	hfov = sensor.horizonal_fov
	if (hfov >= 180):
		hfov = 180
	angle_p1 = (sensor.yaw + sensor.horizonal_fov / 2.0 + 180) % 360 
	angle_p2 = (sensor.yaw - sensor.horizonal_fov / 2.0 + 180) % 360
	diff1 = (angle_p1 - board.yaw) % 360
	diff2 = (angle_p2 - board.yaw) % 360
	if (diff1 > 90 and diff1 < 270):
		angle_p1 = (board.yaw + 90) % 360
	if (diff2 > 90 and diff2 < 270):
		angle_p2 = (board.yaw - 90) % 360
	sin1 = np.sin(deg2rad(angle_p1))
	cos1 = np.cos(deg2rad(angle_p1))
	sin2 = np.sin(deg2rad(angle_p2))
	cos2 = np.cos(deg2rad(angle_p2))
	if (sensor.type == "camera"):
		sensor_region.p1_line_angle = angle_p1
		sensor_region.p2_line_angle = angle_p2
		# compute joint point
		if (hfov == 180):
			sensor_region.joint_pt = [board.posx, board.posy]
		else:
			pt1 = sensor_region.p1
			pt2 = sensor_region.p2
			A = np.mat([[sin1, -cos1], [sin2, -cos2]])
			b = np.mat([[sin1*pt1[0] - cos1*pt1[1]], [sin2*pt2[0] - cos2*pt2[1]]])
			r = np.linalg.solve(A, b)
			sensor_region.joint_pt[0] = r[0, 0]
			sensor_region.joint_pt[1] = r[1, 0]
		sensor_region.p1_line_vector = [cos1, sin1]
		sensor_region.p2_line_vector = [cos2, sin2]
	elif (sensor.type == "lidar"):
		lidar_angle1 = (sensor.yaw + config.max_angle_diff + 180) % 360
		lidar_angle2 = (sensor.yaw - config.max_angle_diff + 180) % 360
		diff1 = (lidar_angle1 - board.yaw) % 360
		diff2 = (lidar_angle2 - board.yaw) % 360
		if (diff1 > 90 and diff1 < 270):
			lidar_angle1 = (board.yaw + 90) % 360
		if (diff2 > 90 and diff2 < 270):
			lidar_angle2 = (board.yaw - 90) % 360
		lidar_line_vec1 = [np.cos(deg2rad(lidar_angle1)), 
			np.sin(deg2rad(lidar_angle1))]
		lidar_line_vec2 = [np.cos(deg2rad(lidar_angle2)), 
			np.sin(deg2rad(lidar_angle2))]
		board_line_vec1 = [cos1, sin1]
		board_line_vec2 = [cos2, sin2]
		# print ("\nboard_line_vec1:", board_line_vec1)
		# print ("lidar_line_vec1:", lidar_line_vec1)
		# print ("\nboard_line_vec2:", board_line_vec2)
		# print ("lidar_line_vec2:", lidar_line_vec2)
		if (np.cross(lidar_line_vec1, board_line_vec1) <= 0):
			sensor_region.p1_line_angle = angle_p1
			sensor_region.p1_line_vector = board_line_vec1
		else:
			sensor_region.p1_line_angle = lidar_angle1
			sensor_region.p1_line_vector = lidar_line_vec1
		if (np.cross(lidar_line_vec2, board_line_vec2) >= 0):
			sensor_region.p2_line_angle = angle_p2
			sensor_region.p2_line_vector = board_line_vec2
		else:
			sensor_region.p2_line_angle = lidar_angle2
			sensor_region.p2_line_vector = lidar_line_vec2
		pt1 = sensor_region.p1
		pt2 = sensor_region.p2
		sin1 = np.sin(deg2rad(sensor_region.p1_line_angle))
		cos1 = np.cos(deg2rad(sensor_region.p1_line_angle))
		sin2 = np.sin(deg2rad(sensor_region.p2_line_angle))
		cos2 = np.cos(deg2rad(sensor_region.p2_line_angle))
		A = np.mat([[sin1, -cos1], [sin2, -cos2]])
		b = np.mat([[sin1*pt1[0] - cos1*pt1[1]], [sin2*pt2[0] - cos2*pt2[1]]])
		r = np.linalg.solve(A, b)
		sensor_region.joint_pt[0] = r[0, 0]
		sensor_region.joint_pt[1] = r[1, 0]

	# print ("\np1_line_angle:", sensor_region.p1_line_angle)
	# print ("p2_line_angle:", sensor_region.p2_line_angle)

def getSensorAngleRegionSide(sensor, board, sensor_region, thesh = 1.0, config = rangeConfig()):
	# p1_side_angle = sensor.pitch + sensor.vertical_fov / 2.0 + 
	sensor_region.side_p1_line_angle = - (sensor.pitch + sensor.vertical_fov / 2.0)
	sensor_region.side_p2_line_angle = - (sensor.pitch - sensor.vertical_fov / 2.0)
	if (sensor_region.side_p1_line_angle < -90):
		sensor_region.side_p1_line_angle = -90
	if (sensor_region.side_p2_line_angle > 90):
		sensor_region.side_p1_line_angle = 90
	sin1 = np.sin(deg2rad(sensor_region.side_p1_line_angle))
	cos1 = np.cos(deg2rad(sensor_region.side_p1_line_angle))
	sin2 = np.sin(deg2rad(sensor_region.side_p2_line_angle))
	cos2 = np.cos(deg2rad(sensor_region.side_p2_line_angle))
	pt1 = sensor_region.side_p1
	pt2 = sensor_region.side_p2
	A = np.mat([[sin1, -cos1], [sin2, -cos2]])
	b = np.mat([[sin1*pt1[0] - cos1*pt1[1]], [sin2*pt2[0] - cos2*pt2[1]]])
	r = np.linalg.solve(A, b)
	sensor_region.side_joint_pt[0] = r[0, 0]
	sensor_region.side_joint_pt[1] = r[1, 0]
	sensor_region.side_p1_line_vector = [cos1, sin1]
	sensor_region.side_p2_line_vector = [cos2, sin2]
	# sensor_region.
	# region = [-1, -1]
	# region[0] = sensor.pitch + sensor.vertical_fov / 2.0
	# region[1] = sensor.pitch - sensor.vertical_fov / 2.0
	# region[0] *= thresh
	# region[1] *= thresh
	# return region

def getSensorPosScore(sensor, board, region):
	score = 1
	# in angle range
	posxy_vec = [sensor.posx - region.joint_pt[0], sensor.posy - region.joint_pt[1]]
	if (np.cross(posxy_vec, region.p1_line_vector) <= 0): return 0 
	if (np.cross(posxy_vec, region.p2_line_vector) >= 0): return 0
	# distance check
	p1_pt = [sensor.posx - region.p1[0], sensor.posy - region.p1[1]]
	p2_pt = [sensor.posx - region.p2[0], sensor.posy - region.p2[1]]
	if (sensor.type == "camera"):
		yaw_vec = [np.cos(deg2rad(sensor.yaw)), np.sin(deg2rad(sensor.yaw))]
		p1_dist = abs(np.dot(p1_pt, yaw_vec))
		p2_dist = abs(np.dot(p2_pt, yaw_vec))
	elif (sensor.type == "lidar"):
		p1_dist = np.linalg.norm(p1_pt)
		p2_dist = np.linalg.norm(p2_pt)
	if (p1_dist <= region.dmin or p2_dist <= region.dmin): return 0
	if (p1_dist >= region.dmax or p2_dist >= region.dmax): return 0
	# check side angle
	poszd_vec = [p1_dist - region.side_joint_pt[0], 
		sensor.posz - board.posz - region.side_joint_pt[1]]
	if (np.cross(poszd_vec, region.side_p1_line_vector) >= 0): return 0
	if (np.cross(poszd_vec, region.side_p2_line_vector) <= 0): return 0
	poszd_vec = [p2_dist - region.side_joint_pt[0], 
		sensor.posz - board.posz - region.side_joint_pt[1]]
	if (np.cross(poszd_vec, region.side_p1_line_vector) >= 0): return 0
	if (np.cross(poszd_vec, region.side_p2_line_vector) <= 0): return 0

	return score
	# pos valid, value score



def getSensorRegion(item, thresh = 1.0, config = rangeConfig()):
	region = SensorRegion()
	sensor = item.sensor
	board = item.board
	dx = board.width / 2.0 * np.sin(deg2rad(board.yaw))
	dy= board.width / 2.0 * np.cos(deg2rad(board.yaw))
	region.p1 = [board.posx + dx, board.posy - dy] # right point
	region.p2 = [board.posx - dx, board.posy + dy] # left point
	region.dmax = getMaxBoardDistXY(sensor, board, thresh, config)
	region.dmin = getMinBoardDistXY(sensor, board, thresh, config)
	getSensorAngleRegionXY(sensor, board, region, thresh, config)
	region.side_p1 = [0, board.height / 2.0]
	region.side_p2 = [0, -board.height / 2.0]
	getSensorAngleRegionSide(sensor, board, region, thresh, config)
	region.score = getSensorPosScore(sensor, board, region)
	# lidar and camera region is two different shape
	return region

# get board region and check
def getBoardRegion(item, thresh = 0.9, config = rangeConfig()):
	sensor = item.sensor
	board = item.board
	region = BoardRegion()
	# birdview
	region.origin_pt[0] = sensor.posx 
	region.origin_pt[1] = sensor.posy
	region.dmin = getMinBoardDistXY(sensor, board, thresh, config) 
	region.dmax = getMaxBoardDistXY(sensor, board, thresh, config) 
	region.angle_range = getBoardAngleRegionXY(sensor, board, 1.0, config) 
	# check whether region valid
	if (region.dmin == -1 or region.dmax == -1 or 
		region.angle_range[0] == -1 or region.angle_range[1] == -1):
		return False, region
	# sideview
	region.side_dmin = region.dmin
	region.side_dmax = region.dmax
	region.side_angle_range = getBoardAngleRegionSide(sensor, board, thresh, config) 
	# best board position 
	getBoardBestPos(sensor, board, region)
	return region