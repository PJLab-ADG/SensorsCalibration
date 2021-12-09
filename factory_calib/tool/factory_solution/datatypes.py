""" 
Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
Limited. All rights reserved.
Yan Guohang <yanguohang@pjlab.org.cn>
"""

import numpy as np
import cv2

def deg2rad(deg):
	return deg * np.pi / 180.0


class Sensor:
	# only support camera or lidar type
	def __init__(self):
		self.name = ""
		self.type = "camera"

		self.posx = 0
		self.posy = 0
		self.posz = 0
		self.yaw = 0 # 0~360 degrees
		self.pitch = 0
		self.roll = 0

		self.horizonal_fov = 100
		self.vertical_fov = 50

		self.img_width = 1920
		self.img_height = 1080

		self.angular_resolution = 0.2


class Board:
	# only support chessboard, apriltag, lidar circle type
	def __init__(self):
		self.name = ""
		# type: camera, lidar&camera and lidar
		self.type = "camera"

		self.posx = 0
		self.posy = 0
		self.posz = 0
		self.yaw = 0
		# only support 0 pitch and 0 roll

		self.width = 1.2
		self.height = 0.9

		self.grid_size = 0.2
		self.radius = 0.1


class SensorBoardItem:
	def __init__(self):
		self.name = ""
		self.sensor = Sensor()
		self.board = Board()


# car center coordinate
#       ^ x
#       |
#  y    |
# <-----| car center
class BoardRegion:
	def __init__(self):
		# birdview
		self.origin_pt = [0, 0] #[x,y]
		self.dmin = 0.1
		self.dmax = 4.0
		self.angle_range = [20, 190] # in degree

		# side view (related to vertical fov and z pos)
		self.side_pt = [0, 0] # side origin pt is fixed
		self.side_dmin = 0.1
		self.side_dmax = 4.0
		self.side_angle_range = [15, -15]

		self.bestpos = [0, 0, 0]
		self.bestdist = 0

class SensorRegion:
	def __init__(self):
		self.p1 = [0, 0]
		self.p2 = [2, 0]
		self.dmin= 0.1
		self.dmax = 4.0
		self.p1_line_angle = 0
		self.p2_line_angle = 0
		self.joint_pt = [0, 0]
		self.p1_line_vector = [1, 1]
		self.p2_line_vector = [-1, 1]

		# side view (a sector area)
		self.side_p1 = [0, 0]
		self.side_p2 = [0, 0]
		self.side_dmin = 0.5
		self.side_dmax = 4.5
		self.side_p1_line_angle = 0
		self.side_p2_line_angle = 0
		self.side_joint_pt = [0, 0]
		self.side_p1_line_vector = [1, 1]
		self.side_p2_line_vector = [-1, 1]

		# self.bestpos = [0, 0, 0]
		self.score = 0
