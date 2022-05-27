# This program is used for vanishing point and homography calibration
import numpy as np
import cv2
import os
import random
import json
import copy
import tkinter as tk
import tkinter.messagebox as msg

# Coordinate
# ground  ^ x
#		  |
#		  |
# y <---- car 
#       center

class CalibrationGui:
	def __init__(self, img_path):
		# scale
		# 0: origin
		# 1: 1024*576
		# 2: 1920*1080
		# 3: 704*448
		# 4: 896*512
		self.calibration_scale = (1024, 576)
		# intrinsic scale 
		# self.intrinsic_scale = 1 # setup by user
		self.image = cv2.imread(img_path)
		self.img_scale = self.image.shape
		# self.image = cv2.resize(self.image, self.scale, interpolation=cv2.INTER_CUBIC)
		self.const_image = copy.deepcopy(self.image)

		# input params (under 1024 scale)
		self.cam_fx = 666
		self.cam_fy = 666
		self.cam_height = 1.35
		# val is positive if camera is to the front of car center
		self.cam_vertical_offset = 0 
		# val is negative if camera is to the right of car center
		self.cam_lateral_offset = 0 

		# camera position (user only input vertical pos)
		# front=1, rear=-1
		# left=1, center=0, right=-1
		self.cam_pos_vertical = 1
		self.cam_pos_lateral = 0
		self.camera_name = ""

		# label lane line points
		# 2 left, 2 right
		self.total_point_num = 4
		self.picked_point_num = 0
		self.label_points = []
		self.label_points_origin = []
		# self.img_points = [] set virtual points?
		self.world_points = []

		# output param
		self.vanishing_pt_x = 1024 / 2.0
		self.vanishing_pt_y = 576 / 2.0
		self.vanishing_pt_x_origin = 1024 / 2.0
		self.vanishing_pt_y_origin = 576 / 2.0
		self.homography = []
		self.homography_inv = []
		self.output_dir = "./output_json/"
		self.file_calibration = self.output_dir
		self.file_intrinsic = self.output_dir
		self.file_extrinsic = self.output_dir
		# make dir
		folder = os.path.exists(self.output_dir)
		if not folder:
			os.makedirs(self.output_dir)

		# image and display window
		self.init_window_name = "CalibrationGui"
		self.opencv_window = "image"
		self.window_width = 400
		self.window_height = 400
		self.input_position_seq_y =list(range(20,2000,100))
		self.input_position_seq_x =[20]*len(self.input_position_seq_y)
		self.input_pos_idx = 0


	def start(self):
		self.gui = tk.Tk()
		self.gui.title(self.init_window_name)
		self.gui.geometry(str(self.window_width) + 'x' + str(self.window_height))
		self.init_window()
		self.SelectPoints()
		self.gui.mainloop()

	def init_window(self):
		# input param window
		space = tk.Label(self.gui, text=" ")
		space.grid(row=0)

		self.message_cam_pos = tk.Label(self.gui, text="Camera Position:")
		self.message_cam_pos.grid(row=1, column=0,sticky='E')

		self.pos_vertical_v = tk.IntVar()
		self.cam_pos_botton_front = tk.Radiobutton(self.gui, text='front', value=1, variable=self.pos_vertical_v)
		self.cam_pos_botton_front.grid(row=1, column=1,sticky='W')
		self.cam_pos_botton_back = tk.Radiobutton(self.gui, text='rear', value=-1, variable=self.pos_vertical_v)
		self.cam_pos_botton_back.grid(row=1, column=2,sticky='W')
		self.pos_vertical_v.set(1)

		self.pos_lateral_v = tk.IntVar()
		self.cam_pos_botton_left = tk.Radiobutton(self.gui, text='left', value=-1, variable=self.pos_lateral_v)
		self.cam_pos_botton_left.grid(row=2, column=1,sticky='W')
		self.cam_pos_botton_center = tk.Radiobutton(self.gui, text='center', value=0, variable=self.pos_lateral_v)
		self.cam_pos_botton_center.grid(row=2, column=2,sticky='W')
		self.cam_pos_botton_right = tk.Radiobutton(self.gui, text='right', value=1, variable=self.pos_lateral_v)		
		self.cam_pos_botton_right.grid(row=2, column=3,sticky='W')

		space = tk.Label(self.gui, text=" ")
		space.grid(row=3)

		# camera intrinsic
		self.message_cam_scale = tk.Label(self.gui, text="Calibration Scale:")
		self.message_cam_scale.grid(row=4,column=0,sticky='E')

		self.intrinsic_scale_v = tk.IntVar()
		self.intrinsic_origin = tk.Radiobutton(self.gui, text='origin', value=0, variable=self.intrinsic_scale_v)
		self.intrinsic_origin.grid(row=4,column=1,sticky='W')
		self.intrinsic_1024 = tk.Radiobutton(self.gui, text='1024x576', value=1, variable=self.intrinsic_scale_v)
		self.intrinsic_1024.grid(row=4,column=2,sticky='W')
		self.intrinsic_1920 = tk.Radiobutton(self.gui, text='1920x1080', value=2, variable=self.intrinsic_scale_v)
		self.intrinsic_1920.grid(row=5,column=1,sticky='W')
		self.intrinsic_704 = tk.Radiobutton(self.gui, text='704x448', value=3, variable=self.intrinsic_scale_v)
		self.intrinsic_704.grid(row=5,column=2,sticky='W')
		self.intrinsic_896 = tk.Radiobutton(self.gui, text='896x512', value=4, variable=self.intrinsic_scale_v)
		self.intrinsic_896.grid(row=6,column=1,sticky='W')
		self.intrinsic_scale_v.set(0)

		space = tk.Label(self.gui, text=" ")
		space.grid(row=6)

		self.fx_label = tk.Label(self.gui, text="camera fx:")
		self.fx_entry = tk.Entry(self.gui, bd=1,width=30)
		self.fx_label.grid(row=7,column=0,sticky='E')
		self.fx_entry.grid(row=7,column=1,columnspan=3,sticky='W',padx=5)
		self.fy_label = tk.Label(self.gui, text="camera fy:")
		self.fy_entry = tk.Entry(self.gui, bd=1,width=30)
		self.fy_label.grid(row=8,column=0,sticky='E')
		self.fy_entry.grid(row=8,column=1,columnspan=3,sticky='W',padx=5,pady=5)

		# car param
		self.cam_height_label = tk.Label(self.gui, text="Camera height(m):")
		self.cam_height_entry = tk.Entry(self.gui, bd=1,width=30)
		self.cam_height_label.grid(row=9,column=0,sticky='E')
		self.cam_height_entry.grid(row=9,column=1,columnspan=3,sticky='W',padx=5)
		self.cam_vertical_offset_label = tk.Label(self.gui, text="Camera vertical offset:\nto front is positive")
		self.cam_vertical_offset_entry = tk.Entry(self.gui, bd=1,width=30)
		self.cam_vertical_offset_label.grid(row=10,column=0,sticky='E',pady=5)
		self.cam_vertical_offset_entry.grid(row=10,column=1,columnspan=3,sticky='W',padx=5,pady=5)
		self.cam_lateral_offset_label = tk.Label(self.gui, text="Camera lateral offset:\nto left is positive")
		self.cam_lateral_offset_entry = tk.Entry(self.gui, bd=1,width=30)
		self.cam_lateral_offset_label.grid(row=12,column=0,sticky='E')
		self.cam_lateral_offset_entry.grid(row=12,column=1,columnspan=3,sticky='W',padx=5,pady=5)

		space = tk.Label(self.gui, text=" ")
		space.grid(row=13)

		# button
		# calibration_button = tk.Button(self.gui, text="Calibrate", commond=self.get_resultCallBack)
		self.calibration_button = tk.Button(self.gui, text="Calibrate",bg="#FFFAFA", command = self.startCalibration)
		self.calibration_button.grid(row=14,column=1,columnspan=2)


	def getParam(self):
		# get camera position
		self.cam_pos_vertical = int(self.pos_vertical_v.get())
		self.cam_pos_lateral = int(self.pos_lateral_v.get())
		v_name = ["front", "rear"]
		l_name = ["left", "center", "right"]
		self.camera_name = v_name[int(-(self.cam_pos_vertical - 1) / 2)] + "_" + l_name[int(self.cam_pos_lateral + 1)] + "_camera"
		self.camera_dir = self.output_dir + self.camera_name + "/"
		folder = os.path.exists(self.camera_dir)
		if not folder:
			os.makedirs(self.camera_dir)

		self.file_calibration = self.camera_dir + self.camera_name + "-calibration.json"
		self.file_intrinsic = self.camera_dir + self.camera_name + "-intrinsic.json"
		self.file_extrinsic = self.camera_dir + self.camera_name + "-to-car_center-extrinsic.json"
		# get scale
		scale_box = [(self.image.shape[1], self.image.shape[0]), (1024, 576), (1920, 1080), (704, 448), (896, 512)]
		self.calibration_scale = scale_box[int(self.intrinsic_scale_v.get())]
		# get calibration param
		self.cam_fx = float(self.fx_entry.get())
		self.cam_fy = float(self.fy_entry.get())
		self.cam_height = float(self.cam_height_entry.get())
		self.cam_vertical_offset = float(self.cam_vertical_offset_entry.get())
		self.cam_lateral_offset = float(self.cam_lateral_offset_entry.get())
		# check for parameters
		self.checkParam()


	def checkParam(self):
		if (self.cam_pos_lateral == 0):
			if (abs(self.cam_lateral_offset) > abs(self.cam_vertical_offset)):
				msg.showinfo("Param Warning", "WARNING!\n[Center Camera]\nLateral offset is larger than vertical offset.\nPlease check!")

	# get user input parameters and calc homography
	def startCalibration(self): 
		self.getParam()
		# apply scale
		self.applyScale()
		self.getHomography()
		print("\nVanishing Point is: ", self.vanishing_pt_x, self.vanishing_pt_y)
		print("Homography is:\n", self.homography)
		self.saveResultJson()
		msg.showinfo(self.camera_name, "Calibration is successful!\nCalibration Files were saved.")
		# verification
		self.saveResultImg()
		self.Verification()
		return 0

	def saveResultImg(self):
		display_img = copy.deepcopy(self.image)
		for i in range(len(self.label_points_origin)):
			img_x = int(self.label_points_origin[i][0])
			img_y = int(self.label_points_origin[i][1])
			w_x = self.world_points[i][0]
			w_y = self.world_points[i][1]
			pt_str = "(%.1f,%.1f)" % (w_x, w_y)
			gap_x = 190
			if (i >= len(self.label_points_origin) / 2):
				gap_x = -15
			cv2.circle(display_img, (img_x, img_y), 1, (50, 20, 255), thickness=7)
			cv2.putText(display_img, pt_str, (img_x-gap_x, img_y), cv2.FONT_HERSHEY_COMPLEX, 0.8, (200, 200, 255), 2)
		pt_str = "(%.1f,%.1f)" % (self.vanishing_pt_x, self.vanishing_pt_y)
		scale_str = "Calibration Scale: %d x %d" % (self.calibration_scale[0], self.calibration_scale[1])
		cv2.putText(display_img, pt_str, (int(self.vanishing_pt_x_origin)+15, int(self.vanishing_pt_y_origin)), cv2.FONT_HERSHEY_COMPLEX, 0.2, (235, 200, 50), 1)
		cv2.putText(display_img, scale_str, (20, self.image.shape[0] - 50), cv2.FONT_HERSHEY_COMPLEX, 1.0, (30, 30, 30), 3)
		img_path = self.camera_dir + self.camera_name + "_result.png"
		cv2.imwrite(img_path, display_img)
		print ("Save result image ", img_path)


	def applyScale(self):
		scale_x = self.calibration_scale[0] / float(self.image.shape[1])
		scale_y = self.calibration_scale[1] / float(self.image.shape[0])
		self.vanishing_pt_x = self.vanishing_pt_x_origin * scale_x
		self.vanishing_pt_y = self.vanishing_pt_y_origin * scale_y
		# apply scale to labeled image points
		self.label_points = copy.deepcopy(self.label_points_origin)
		for i in range(len(self.label_points)):
			self.label_points[i][0] *= scale_x
			self.label_points[i][1] *= scale_y


	def Verification(self):
		# (image, img_points, world_points, pos, img_path)
		pos = self.cam_pos_vertical
		if (self.cam_pos_lateral != 0):
			pos *= -1
		birdview_file = self.camera_dir + self.camera_name + "_birdview.png"
		resized_image = cv2.resize(self.const_image, self.calibration_scale, interpolation=cv2.INTER_CUBIC)
		VerifyBirdView(resized_image, np.array(self.label_points), self.world_points, pos, birdview_file)


	def getHomography(self):
		img_pts = []
		world_pts = []
		vp_x = self.vanishing_pt_x
		vp_y = self.vanishing_pt_y
		cam_pos_symbol = self.cam_pos_vertical
		# not center
		if (self.cam_pos_lateral != 0):
			cam_pos_symbol *= -1

		print ("\nworld points:")

		for i in range(len(self.label_points)):
			img_x = self.label_points[i][0]
			img_y = self.label_points[i][1]
			world_x = self.cam_height * self.cam_fy / (img_y - vp_y)
			world_y = world_x * (vp_x - img_x) / self.cam_fx
			# add bias based on camera position
			world_x = cam_pos_symbol * world_x + self.cam_vertical_offset
			world_y = cam_pos_symbol * world_y + self.cam_lateral_offset
			img_pts.append([img_x, img_y])
			world_pts.append([world_x, world_y])
			print (world_x, world_y)
		img_pts = np.array(img_pts)
		world_pts = np.array(world_pts)
		self.world_points = world_pts
		self.homography, mask = cv2.findHomography(img_pts, world_pts)


	# select lane line points and calc vanishing point
	def SelectPoints(self):
		cv2.namedWindow(self.opencv_window, 0)
		cv2.setMouseCallback(self.opencv_window, self.on_EVENT_LBUTTONDOWN)
		cv2.imshow(self.opencv_window, self.image)

		msg.showinfo(self.opencv_window, "Please select points inside the lane line, two points left and two points right")
		while (True) :
			try:
				cv2.waitKey(10)
			except Exception:
				cv2.destroyWindow(self.opencv_window)
				break
			if (self.picked_point_num == self.total_point_num):
				# note: this is vanishing point under origin image size, not final result
				self.vanishing_pt_x_origin, self.vanishing_pt_y_origin = getVanishingPoint(self.image, self.label_points_origin)
				# print("Vanishing Point is: ", self.vanishing_pt_x, self.vanishing_pt_y)
				break

		# show vanishing point
		cv2.circle(self.image, (int(self.vanishing_pt_x_origin), int(self.vanishing_pt_y_origin)), 1, (235, 200, 50), thickness=10)
		cv2.imshow(self.opencv_window, self.image)
		cv2.setMouseCallback(self.opencv_window, self.close_EVENT_LBUTTONDOWN)
		msg.showinfo(self.opencv_window, "Please input calibration parameters.")

		# cv2.destroyWindow(self.opencv_window)


	def saveResultJson(self):
		self.save_calibration_json()
		self.save_intrinsic_json()
		self.save_extrinsic_json()

	def save_calibration_json(self):
		root_name = self.camera_name + "_calibration"
		data = {
			root_name: {
				"device_type": "camera",
				"param": {
					"blinding_area_param": {
						"blinding_area_distance": 0,
						"blinding_area_pixel_height": self.calibration_scale[1]
					},
					"h_matrix": {
						"cols": 3,
						"continuous": "true",
						"data": [list(self.homography[0]), list(self.homography[1]), list(self.homography[2])],
						"rows": 3,
						"type": 6
					},
					"vanishing_point": {
						"vanishing_pt_x": self.vanishing_pt_x,
						"vanishing_pt_y": self.vanishing_pt_y
					}
				},
				"param_type": "calibration",
				"sensor_name": root_name,
				"target_sensor_name": root_name
			}
		}
		saveJsonFile(self.file_calibration, data)

	def save_intrinsic_json(self):
		root_name = self.camera_name + "_calib"
		data = {
			root_name: {
				"device_type": "camera",
				"param": {
					"cam_K": {
						"continuous": "true",
						"data": [
							[self.cam_fx, 0, self.calibration_scale[0] / 2.0 + random.randint(-10, 10)],
							[0, self.cam_fy, self.calibration_scale[1] / 2.0 + random.randint(-10, 10)],
							[0, 0, 1]],
						"rows": 3,
						"cols": 3,
						"type": 6
					},
					"cam_dist": {
						"continuous": "true",
						"data": [[0,0,0,0]],
						"rows": 1,
						"cols": 4,
						"type": 6
					},
					"img_dist_h": self.calibration_scale[1],
					"img_dist_w": self.calibration_scale[0]
				},
				"param_type": "intrinsic",
				"sensor_name": self.camera_name,
				"target_sensor_name": self.camera_name
			}
		}
		saveJsonFile(self.file_intrinsic, data)


	def save_extrinsic_json(self):
		root_name = self.camera_name + "_car_center_calib"
		data = {
			root_name: {
				"device_type": "relational",
				"param": {
					"sensor_calib": {
						"continuous": "true",
						"data": [
							[1, 0, 0, self.cam_vertical_offset],
							[0, 1, 0, self.cam_lateral_offset],
							[0, 0, 0, self.cam_height],
							[0, 0, 0, 1]],
						"rows": 4,
						"cols": 4,
						"type": 6
					},
					"time_lag": 0
				},
				"param_type": "extrinsic",
				"sensor_name": self.camera_name,
				"target_sensor_name": "car_center"
				}
		}
		saveJsonFile(self.file_extrinsic, data)


	def on_EVENT_LBUTTONDOWN(self,event, x, y, flags, param):
		if event == cv2.EVENT_LBUTTONDOWN:
			self.picked_point_num += 1

			x_r = float(x)
			y_r = float(y)
			self.label_points_origin.append([x_r, y_r])

			cv2.circle(self.image, (x, y), 1, (50, 20, 255), thickness=5)
			cv2.imshow(self.opencv_window, self.image)

			xy = "%d,%d" % (x, y)
			print(xy)

	def close_EVENT_LBUTTONDOWN(self,event, x, y, flags, param):
		if event == cv2.EVENT_LBUTTONDOWN:
			print ("\nLane line points already labeled!")
			print ("Please fill in calibration blanks.")


# calibration method
def getLineJointPoint(k1, b1, k2, b2):
	x = (b2- b1) / (k1- k2)
	y = k1 * x + b1
	return x, y

def linear_regression(x, y):
	N = len(x)
	sumx = sum(x)
	sumy = sum(y)
	sumx2 = sum(x ** 2)
	sumxy = sum(x * y)
	A = np.mat([[N, sumx], [sumx, sumx2]])
	b = np.array([sumy, sumxy])
	return np.linalg.solve(A, b)

def getVanishingPoint(img, img_points):
	pt_num = int(len(img_points) / 2)
	x_list = []
	y_list = []
	for i in range(len(img_points)):
		x_list.append(img_points[i][0])
		y_list.append(img_points[i][1])
	x_list = np.array(x_list)
	y_list = np.array(y_list)
	b_left, k_left = linear_regression(x_list[0:pt_num], y_list[0:pt_num])
	b_right, k_right = linear_regression(x_list[pt_num:], y_list[pt_num:])

	vp_x, vp_y = getLineJointPoint(k_left, b_left, k_right, b_right)
	# draw lane line
	start_x_l = (img.shape[0] - b_left) / k_left
	start_x_r = (img.shape[0] - b_right) / k_right
	cv2.line(img, (int(start_x_l), img.shape[0]), (int(vp_x), int(vp_y)), (255, 255, 255), thickness = 3)
	cv2.line(img, (int(start_x_r), img.shape[0]), (int(vp_x), int(vp_y)), (255, 255, 255), thickness = 3)
	return vp_x, vp_y

def saveJsonFile(file_path, data):
	with open(file_path, 'w') as f:
		json.dump(data, f, sort_keys=True, indent=4, separators=(',', ': '))

# verification
# pos: to front(1) camera or to back(-1) camera
def VerifyBirdView(image, img_points, world_points, pos, img_path):
	width = 800
	height = 800
	scale = 60
	x_offset = 400
	y_offset = 950
	direct = int((pos + 1) / 2)
	bv_worldpoints = copy.deepcopy(world_points)
	bv_worldpoints[:, 0] = - scale * world_points[:, 1] + x_offset
	bv_worldpoints[:, 1] = - scale * world_points[:, 0] + y_offset * direct

	birdview_hmat, mask = cv2.findHomography(img_points, bv_worldpoints)

	birdview_img = cv2.warpPerspective(image, birdview_hmat, (width, height))
	cv2.imwrite(img_path, birdview_img)
	print("Save birdview image ", img_path)


if __name__ == "__main__":
	calib_gui = CalibrationGui("calib.jpg")
	calib_gui.start()
