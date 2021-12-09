
""" 
Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
Limited. All rights reserved.
Yan Guohang <yanguohang@pjlab.org.cn>
"""
import numpy as np
import cv2
import os
import re
import math
import random
import tkinter as tk
from tkinter import ttk
import tkinter.messagebox as msg
from datatypes import *

from PIL import Image, ImageTk, ImageDraw
from PIL import ImageGrab

from win32 import win32api, win32gui, win32print
from win32.lib import win32con
from win32.win32api import GetSystemMetrics


def randomcolor():
    colorArr = ['1','2','3','4','5','6','7','8','9','A','B','C','D','E','F']
    color = ""
    for i in range(6):
        color += colorArr[random.randint(0,14)]
    return "#"+color


class BirdviewCanvas:
	def __init__(self):
		self.win = []
		self.item_pattern_box = []
		self.side_item_box = []
		self.item_color = []
		self.canvas_height = 500
		self.side_canvas_height = 160
		self.canvas_width = 400
		# one pixel = 0.06m
		self.scale = 0.06
		self.side_origin_x = 40
		self.side_origin_y = int(self.side_canvas_height / 2 - 1)

		self.car_width = 2.0
		self.car_height = 4.5

		self.bg_pattern = []

		self.bg_color = "White"
		self.images = []

		# save region for side view
		self.result_region = []

	def meterlen2pixellen(self, length):
		return int(length / self.scale)

	def coord2pixel(self, meter2d):
		coord = [self.canvas_width/2.0, self.canvas_height/2.0]
		coord[0] -= meter2d[1] / self.scale
		coord[1] -= meter2d[0] / self.scale
		coord[0] = int(coord[0]) - 1
		coord[1] = int(coord[1]) - 1
		return coord

	def start(self, win):
		self.win = win
		self.canvas = tk.Canvas(win, height=self.canvas_height, width=self.canvas_width, bg=self.bg_color)
		self.canvas.grid(row=0, rowspan=20, column=4, padx=10)
		# side view
		self.side_canvas = tk.Canvas(win, height=self.side_canvas_height, width=self.canvas_width, bg=self.bg_color)
		self.side_canvas.grid(row=20, rowspan=8, column=4, padx=10)
		self.drawBackGround()

	def drawBackGround(self):
		# draw grid line
		w = self.canvas_width
		h = self.canvas_height
		hhalf_m = math.ceil(h * self.scale / 2.0)
		whalf_m = math.ceil(w * self.scale / 2.0)
		hmid = int(h / 2 - 1)
		wmid = int(w / 2 - 1)
		lcolor = "#B0C4DE"
		self.bg_pattern.append(self.canvas.create_line(0, hmid, w, hmid, fill=lcolor, width=1))
		self.bg_pattern.append(self.canvas.create_line(wmid, 0, wmid, h, fill=lcolor, width=1))
		for i in range(1, hhalf_m):
			gap = int(i*1.0/self.scale)
			self.bg_pattern.append(self.canvas.create_line(0, hmid-gap, w, hmid-gap, fill=lcolor, width=1))
			self.bg_pattern.append(self.canvas.create_line(0, hmid+gap, w, hmid+gap, fill=lcolor, width=1))
		for i in range(1, whalf_m):
			gap = int(i*1.0/self.scale)
			self.bg_pattern.append(self.canvas.create_line(wmid-gap, 0, wmid-gap, h, fill=lcolor, width=1))
			self.bg_pattern.append(self.canvas.create_line(wmid+gap, 0, wmid+gap, h, fill=lcolor, width=1))
		
		# draw center car
		car_hhalf = self.meterlen2pixellen(self.car_height/2.0)
		car_whalf = self.meterlen2pixellen(self.car_width/2.0)
		self.bg_pattern.append(self.create_rectangle(wmid-car_whalf, hmid-car_hhalf, wmid+car_whalf, hmid+car_hhalf, 
			fill="Black", outline="", alpha = 0.7))

		# draw side view background
		h = self.side_canvas_height
		ox = self.side_origin_x
		hhalf_m = math.ceil(h * self.scale / 2.0)
		hmid = int(h / 2 - 1)
		whalf_m_min = - math.ceil(self.side_origin_x * self.scale)
		whalf_m_max = math.ceil((w - self.side_origin_x) * self.scale)
		self.bg_pattern.append(self.side_canvas.create_line(0, hmid, w, hmid, fill=lcolor, width=1))
		self.bg_pattern.append(self.side_canvas.create_line(ox, 0, ox, h, fill=lcolor, width=1))
		for i in range(1, hhalf_m):
			gap = int(i*1.0/self.scale)
			self.bg_pattern.append(self.side_canvas.create_line(0, hmid-gap, w, hmid-gap, fill=lcolor, width=1))
			self.bg_pattern.append(self.side_canvas.create_line(0, hmid+gap, w, hmid+gap, fill=lcolor, width=1))
		for i in range(whalf_m_min, whalf_m_max + 1):
			gap = int(i*1.0/self.scale)
			self.bg_pattern.append(self.side_canvas.create_line(ox+gap, 0, ox+gap, h, fill=lcolor, width=1))
		# draw side sensor
		self.bg_pattern.append(self.side_canvas.create_oval(ox-4, hmid-4, ox+4, hmid+4, 
			fill="#696969", outline=""))

	# Define a function to make the transparent rectangle
	def create_rectangle(self, x, y, a, b, **options):
		side = False
		if 'side' in options:
			options.pop('side')
			side = True
		if 'alpha' in options:
			# Calculate the alpha transparency for every color(RGB)
			alpha = int(options.pop('alpha') * 255)
			# Use the fill variable to fill the shape with transparent color
			fill = options.pop('fill')
			fill = self.win.winfo_rgb(fill) + (alpha,)
			image = Image.new('RGBA', (a-x, b-y), fill)
			self.images.append(ImageTk.PhotoImage(image))
			if (not side):
				return self.canvas.create_image(x, y, image=self.images[-1], anchor='nw')
			else:
				return self.side_canvas.create_image(x, y, image=self.images[-1], anchor='nw')
			# self.canvas.create_rectangle(x, y, a, b, **options)
		if (not side):
			return self.canvas.create_rectangle(x, y, a, b, **options)
		else:
			return self.side_canvas.create_rectangle(x, y, a, b, **options)


	def create_arc(self, x, y, a, b, **options):
		side = False
		if 'side' in options:
			options.pop('side')
			side = True
		s = options.pop('start')
		e = options.pop('end')
		tags = ""
		if 'tag' in options:
			tags = options.pop('tag')
		if 'alpha' in options:
			# Calculate the alpha transparency for every color(RGB)
			alpha = int(options.pop('alpha') * 255)
			# Use the fill variable to fill the shape with transparent color
			fill = options.pop('fill')
			fill = self.win.winfo_rgb(fill)
			fill = (fill[0]%256, fill[1]%256, fill[2]%256)
			fill = fill + (alpha,)
			image = Image.new('RGBA', (a-x, b-y))
			drawer = ImageDraw.Draw(image, 'RGBA')
			drawer.pieslice((0, 0, a-x-1, b-y-1), start=s, end=e, fill=fill)
			drawer.arc((0, 0, a-x-1, b-y-1), start=s, end=e, fill="#DCDCDC", width=2)
			self.images.append(ImageTk.PhotoImage(image))
			if (not side):
				return self.canvas.create_image(x, y, image=self.images[-1], anchor='nw', tag=tags)
			else:
				return self.side_canvas.create_image(x, y, image=self.images[-1], anchor='nw', tag=tags)
			# self.canvas.create_rectangle(x, y, a, b, **options)
		if (not side):
			return self.canvas.create_arc(x, y, a, b, tag=tags, **options)
		else:
			return self.side_canvas.create_arc(x, y, a, b, tag=tags, **options)


	def drawBoardRegion(self, item_id, item, board_region):
		# new subject
		if (item_id >= len(self.item_pattern_box)):
			color = randomcolor()
			self.item_color.append(color)
			name = item.name
			pattern = []
			side_pattern = []
			# draw side view ground
			self.drawGround(0, item.sensor, item.board, side_pattern, color, name)
			# draw region
			self.drawRegion(board_region, pattern, side_pattern, color, name)
			# draw sensor
			self.drawSensor(0, item.sensor, item.board, board_region, pattern, side_pattern, color, name)
			# draw best board position
			self.drawBoard(0, item.sensor, item.board, board_region, pattern, side_pattern, color, name)
			self.item_pattern_box.append(pattern)
			self.side_item_box.append(side_pattern)
			self.result_region.append(board_region)
			self.canvas.lift("sensor")
			self.canvas.lift("board")
		else:
			# hidden subject or change subject
			for pattern in self.item_pattern_box[item_id]:
				self.canvas.delete(pattern)
			for pattern in self.side_item_box[item_id]:
				self.side_canvas.delete(pattern)
			self.item_pattern_box[item_id] = []
			self.side_item_box[item_id] = []
			color = self.item_color[item_id]
			name = item.name
			# draw side view ground
			self.drawGround(0, item.sensor, item.board, self.side_item_box[item_id], color, name)
			# draw region
			self.drawRegion(board_region, self.item_pattern_box[item_id], self.side_item_box[item_id], color, name)
			# draw sensor
			self.drawSensor(0, item.sensor, item.board, board_region, self.item_pattern_box[item_id],
				self.side_item_box[item_id], color, name)
			self.drawBoard(0, item.sensor, item.board, board_region, self.item_pattern_box[item_id], 
				self.side_item_box[item_id], color, name)
			self.result_region[item_id] = board_region
			self.canvas.lift("sensor")
			self.canvas.lift("board")


	def drawSensorRegion(self, item_id, item, sensor_region):
		# new subject
		if (item_id >= len(self.item_pattern_box)):
			color = randomcolor()
			self.item_color.append(color)
			name = item.name
			pattern = []
			side_pattern = []
			# draw side view ground
			self.drawGround(1, item.sensor, item.board, side_pattern, color, name)
			# draw region
			self.drawRoughRegion(item.board, sensor_region, pattern, side_pattern, color, name)
			# draw sensor
			self.drawSensor(1, item.sensor, item.board, sensor_region, pattern, side_pattern, color, name)
			# draw best board position
			self.drawBoard(1, item.sensor, item.board, sensor_region, pattern, side_pattern, color, name)
			self.item_pattern_box.append(pattern)
			self.side_item_box.append(side_pattern)
			self.result_region.append(sensor_region)
			self.canvas.lift("board")
			self.canvas.lift("sensor")	
		else:
			# hidden subject or change subject
			for pattern in self.item_pattern_box[item_id]:
				self.canvas.delete(pattern)
			for pattern in self.side_item_box[item_id]:
				self.side_canvas.delete(pattern)
			self.item_pattern_box[item_id] = []
			self.side_item_box[item_id] = []
			color = self.item_color[item_id]
			name = item.name
			# draw side view ground
			self.drawGround(1, item.sensor, item.board, self.side_item_box[item_id], color, name)
			# draw region
			self.drawRoughRegion(item.board, sensor_region, self.item_pattern_box[item_id], 
				self.side_item_box[item_id], color, name)
			# draw sensor
			self.drawSensor(1, item.sensor, item.board, sensor_region, self.item_pattern_box[item_id],
				self.side_item_box[item_id], color, name)
			self.drawBoard(1, item.sensor, item.board, sensor_region, self.item_pattern_box[item_id], 
				self.side_item_box[item_id], color, name)
			self.result_region[item_id] = sensor_region
			self.canvas.lift("board")
			self.canvas.lift("sensor")	


	def hideItem(self, item_id):
		for pattern in self.item_pattern_box[item_id]:
			self.canvas.itemconfigure(pattern, state="hidden")
		for pattern in self.side_item_box[item_id]:
			self.side_canvas.itemconfigure(pattern, state="hidden")

	def showItem(self, item_id):
		for pattern in self.item_pattern_box[item_id]:
			self.canvas.itemconfigure(pattern, state="normal")

	def deleteItem(self, item_id, item):
		for pattern in self.item_pattern_box[item_id]:
			self.canvas.delete(pattern)
		for pattern in self.side_item_box[item_id]:
			self.side_canvas.delete(pattern)
		self.item_pattern_box.pop(item_id)
		self.side_item_box.pop(item_id)
		self.item_color.pop(item_id)
		self.result_region.pop(item_id)


	def clear(self):
		for item in self.item_pattern_box:
			for pattern in item:
				self.canvas.delete(pattern)
		for item in self.side_item_box:
			for pattern in item:
				self.side_canvas.delete(pattern)
		self.item_pattern_box = []
		self.side_item_box = []
		self.item_color = []
		self.images = self.images[:1]
		

	def drawSensor(self, mode, sensor, board, region, pattern, side_pattern, color, sensortag):
		r = 3
		d = 10
		l = int(self.meterlen2pixellen(region.dmax) * 0.8)
		if (mode == 1):
			l /= 2
		pt_pixel = self.coord2pixel([sensor.posx, sensor.posy])
		x = pt_pixel[0]
		y = pt_pixel[1]
		x1 = x-l*np.sin(deg2rad(sensor.yaw))
		y1 = y-l*np.cos(deg2rad(sensor.yaw))
		# draw angle
		pattern.append(self.canvas.create_line(x, y, x1, y1, fill=color, width=3, tag=(sensortag, "sensor")))
		pattern.append(self.canvas.create_oval(x-r, y-r, x+r, y+r, fill=color, outline="", tag=(sensortag, "sensor")))
		self.canvas.tag_bind(pattern[-1],'<Button-1>', self.showSideRegion)
		self.canvas.tag_bind(pattern[-2],'<Button-1>', self.showSideRegion)
		if (mode == 1):
			text_size = 11
			if (region.score == 0):
				color = "#DC143C"
				text_size = 13
			pattern.append(self.canvas.create_text(x-d, y-d, fill=color, text=str(region.score), 
				font=("Purisa", text_size, "bold"), tag=(sensortag, "sensor")))
			self.canvas.tag_bind(pattern[-1],'<Button-1>', self.showSideRegion)
			# draw side sensor
			dist = self.side_origin_x + np.linalg.norm([sensor.posx - board.posx, sensor.posy - board.posy]) / self.scale
			y_pixel =  self.side_origin_y + self.meterlen2pixellen(sensor.posz-board.posz)
			side_pattern.append(self.side_canvas.create_oval(dist-r, y_pixel-r, dist+r, y_pixel+r, 
				fill=color, outline="", tag=("side", sensortag, "sensor")))
			self.side_canvas.itemconfigure(side_pattern[-1], state="hidden")


	def drawGround(self, mode, sensor, board, side_pattern, color, sensortag):
		if (mode == 0):
			h_pixel = self.side_origin_y + self.meterlen2pixellen(sensor.posz)
		elif (mode == 1):
			h_pixel = self.side_origin_y + self.meterlen2pixellen(board.posz) 
		side_pattern.append(self.side_canvas.create_line(0, h_pixel, self.canvas_width, h_pixel, 
			fill="#D3D3D3", width=4, tag=("side", sensortag, "ground")))
		self.side_canvas.itemconfigure(side_pattern[-1], state="hidden")


	def drawBoard(self, mode, sensor, board, board_region, pattern, side_pattern, color, sensortag):
		hwidth_pixel = self.meterlen2pixellen(board.width / 2.0)
		if (mode == 0):
			best_xy = [board_region.bestpos[0], board_region.bestpos[1]]
		elif (mode == 1):
			best_xy = [board.posx, board.posy]
		pt_pixel = self.coord2pixel(best_xy)
		x = pt_pixel[0]
		y = pt_pixel[1]
		x1 = x-hwidth_pixel*np.sin(deg2rad(board.yaw - 90))
		y1 = y-hwidth_pixel*np.cos(deg2rad(board.yaw - 90))		
		x2 = x+hwidth_pixel*np.sin(deg2rad(board.yaw - 90))
		y2 = y+hwidth_pixel*np.cos(deg2rad(board.yaw - 90))
		pattern.append(self.canvas.create_line(x2, y2, x1, y1, fill="#D2B487", width=4, tag=(sensortag, "board")))
		# add text
		d = 10
		number_str =  re.search("\d+", board.name).group()
		pattern.append(self.canvas.create_text(x-d, y-d, fill="#D2B487", text=number_str, tag=(sensortag, "board")))
		self.canvas.tag_bind(pattern[-1],'<Button-1>', self.showSideRegion)
		self.canvas.tag_bind(pattern[-2],'<Button-1>', self.showSideRegion)

		# side board
		hheight_pixel = self.meterlen2pixellen(board.height / 2.0)
		if (mode == 0):
			d_pixel = self.meterlen2pixellen(board_region.bestdist)
			y_pixel = self.meterlen2pixellen(sensor.posz-board_region.bestpos[2])
		elif (mode == 1):
			d_pixel = 0
			y_pixel = 0
		x = self.side_origin_x + d_pixel
		y = self.side_origin_y + y_pixel
		side_pattern.append(self.side_canvas.create_line(x, y-hheight_pixel, x, y+hheight_pixel, 
			fill="#D2B487", width=4, tag=("side", sensortag, "board")))
		self.side_canvas.itemconfigure(side_pattern[-1], state="hidden")



	def transArcAngleXY(self, angle_range):
		a0 = (-angle_range[1] + 270) % 360
		a1 = (-angle_range[0] + 270) % 360
		return a0, a1


	def drawRegion(self, board_region, pattern, side_pattern, color, sensortag):
		a0, a1 = self.transArcAngleXY(board_region.angle_range)
		pt_pixel = self.coord2pixel(board_region.origin_pt)
		x = pt_pixel[0]
		y = pt_pixel[1]
		dmin_pixel = self.meterlen2pixellen(board_region.dmin)
		dmax_pixel = self.meterlen2pixellen(board_region.dmax)
		# print (board_region.angle_range)
		# print (a0, a1)
		# print (x, y)
		# print (dmin_pixel, dmax_pixel)
		# print (board_region.dmax)
		pattern.append(self.create_arc(x-dmax_pixel, y-dmax_pixel, x+dmax_pixel, y+dmax_pixel, start=a0, end=a1, fill=color, 
			outline="", alpha=0.6, tag=(sensortag, "region")))
		pattern.append(self.create_arc(x-dmin_pixel, y-dmin_pixel, x+dmin_pixel, y+dmin_pixel, start=a0, end=a1, fill="White", 
			outline="", alpha=0.4, tag=(sensortag, "region")))
		
		# side view(hidden state)

		a0 = -board_region.side_angle_range[0]
		a1 = -board_region.side_angle_range[1]
		x = self.side_origin_x
		y = self.side_origin_y
		side_pattern.append(self.create_arc(x-dmax_pixel, y-dmax_pixel, x+dmax_pixel, y+dmax_pixel, start=a0, end=a1, fill=color, 
			outline="", alpha=0.6, tag=("side", sensortag, "region"), side=True))
		side_pattern.append(self.create_arc(x-dmin_pixel, y-dmin_pixel, x+dmin_pixel, y+dmin_pixel, start=a0, end=a1, fill="White", 
			outline="", alpha=0.4, tag=("side", sensortag, "region"), side=True))
		self.side_canvas.itemconfigure(side_pattern[-1], state="hidden")
		self.side_canvas.itemconfigure(side_pattern[-2], state="hidden")


	def drawRoughRegion(self, board, sensor_region, pattern, side_pattern, color, sensortag):
		# draw line
		pt_pixel = self.coord2pixel(sensor_region.joint_pt)
		p1_pixel = self.coord2pixel(sensor_region.p1)
		p2_pixel = self.coord2pixel(sensor_region.p2)
		dmin_pixel = self.meterlen2pixellen(sensor_region.dmin)
		dmax_pixel = self.meterlen2pixellen(sensor_region.dmax)
		x = pt_pixel[0]		
		y = pt_pixel[1]
		x1 = x - sensor_region.p1_line_vector[1] * dmax_pixel * 0.5
		y1 = y - sensor_region.p1_line_vector[0] * dmax_pixel * 0.5
		x2 = x - sensor_region.p2_line_vector[1] * dmax_pixel * 0.5
		y2 = y - sensor_region.p2_line_vector[0] * dmax_pixel * 0.5
		pattern.append(self.canvas.create_line(p1_pixel[0], p1_pixel[1], x, y, fill=color, width=2,
			dash=(4,4), tag=(sensortag, "region")))
		pattern.append(self.canvas.create_line(p2_pixel[0], p2_pixel[1], x, y, fill=color, width=2,
			dash=(4,4), tag=(sensortag, "region")))
		pattern.append(self.canvas.create_line(x, y, x1, y1, fill=color, width=2, tag=(sensortag, "region")))
		pattern.append(self.canvas.create_line(x, y, x2, y2, fill=color, width=2, tag=(sensortag, "region")))

		# side region
		hbheight = self.meterlen2pixellen(board.height/2.0)
		pt_pixel = self.coord2pixel(sensor_region.side_joint_pt)
		p1_pixel = [self.side_origin_x, self.side_origin_y-hbheight]
		p2_pixel = [self.side_origin_x, self.side_origin_y+hbheight]
		x = self.side_origin_x + sensor_region.side_joint_pt[0] / self.scale	
		y = self.side_origin_y - sensor_region.side_joint_pt[1] / self.scale
		x1 = x + dmax_pixel * 0.5 * sensor_region.side_p1_line_vector[0]		
		y1 = y - dmax_pixel * 0.5 * sensor_region.side_p1_line_vector[1]
		x2 = x + dmax_pixel * 0.5 * sensor_region.side_p2_line_vector[0]		
		y2 = y - dmax_pixel * 0.5 * sensor_region.side_p2_line_vector[1]			
		side_pattern.append(self.side_canvas.create_line(p1_pixel[0], p1_pixel[1], x, y, fill=color, width=3,
			dash=(4,4), tag=("side", sensortag, "region")))
		side_pattern.append(self.side_canvas.create_line(p2_pixel[0], p2_pixel[1], x, y, fill=color, width=3,
			dash=(4,4), tag=("side", sensortag, "region")))
		side_pattern.append(self.side_canvas.create_line(x, y, x1, y1, fill=color, width=3, tag=("side", sensortag, "region")))
		side_pattern.append(self.side_canvas.create_line(x, y, x2, y2, fill=color, width=3, tag=("side", sensortag, "region")))
		self.side_canvas.itemconfigure(side_pattern[-1], state="hidden")
		self.side_canvas.itemconfigure(side_pattern[-2], state="hidden")
		self.side_canvas.itemconfigure(side_pattern[-3], state="hidden")
		self.side_canvas.itemconfigure(side_pattern[-4], state="hidden")


	def showSideRegion(self, event):
		items = self.canvas.find_closest(event.x, event.y, halo=10)
		tags = self.canvas.gettags(items[0])
		if ("region" in tags):
			return
		item_name = tags[0]
		side_items = self.side_canvas.find_withtag(("side",))
		for item in side_items:
			self.side_canvas.itemconfigure(item, state="hidden")
		# show current item
		show_items = self.side_canvas.find_withtag(item_name)
		for pattern in show_items:
			self.side_canvas.itemconfigure(pattern, state="normal")


	def showSide(self, item_id):
		side_items = self.side_canvas.find_withtag(("side",))
		for item in side_items:
			self.side_canvas.itemconfigure(item, state="hidden")
		for pattern in self.side_item_box[item_id]:
			self.side_canvas.itemconfigure(pattern, state="normal")


	def saveImages(self, output_dir, item_box):
		# save birdwiew
		eps_file = output_dir + "/birdwiew.eps"
		file = output_dir + "/birdwiew.png"
		# self.canvas.postscript(file=eps_file, colormode="color")
		# img = Image.open(eps_file)
		# img.save(file, "png")
		# # image = ImageMagick.Popen("mogrify -format png " + file, shell=True)
		# # image.wait()

		# img.save(file, 'png')
		hDC = win32gui.GetDC(0)
		scale = win32print.GetDeviceCaps(hDC, win32con.DESKTOPHORZRES) /\
			GetSystemMetrics (0)

		x = self.canvas.winfo_rootx() * scale
		y = self.canvas.winfo_rooty() * scale
		x1 = x + self.canvas.winfo_width() * scale
		y1 = y + self.canvas.winfo_height() * scale
		# print (x,y,x1,y1)
		# image = pyautogui.screenshot()
		# image = cv2.cvtColor(np.array(image), cv2.COLOR_RGB2BGR)
		# print (image.shape)
		# img = image[y:y1+1, x:x1+1]
		# cv2.imwrite(file, img)

		ImageGrab.grab((x,y,x1,y1)).save(file)
		# # save each side view
		x = self.side_canvas.winfo_rootx() * scale
		y = self.side_canvas.winfo_rooty() * scale
		x1 = x + self.side_canvas.winfo_width() * scale
		y1 = y + self.side_canvas.winfo_height() * scale
		# print (x,y,x1,y1)

		# HWND = self.canvas.winfo_id()  # get the handle of the canvas
		# rect = win32gui.GetWindowRect(HWND)  # get the coordinate of the canvas
		# im = ImageGrab.grab(rect)
		# im.save(file, 'png')
		# # ImageGrab.grab(rect).save(file)
		# HWND = self.side_canvas.winfo_id()  # get the handle of the canvas
		# rect = win32gui.GetWindowRect(HWND)  # get the coordinate of the canvas

		for i in range(len(item_box)):
			self.showSide(i)
			self.side_canvas.update()
			eps_file = output_dir + "/" + item_box[i].name + ".ps"
			file = output_dir + "/" + item_box[i].name + ".png"
			ImageGrab.grab((x,y,x1,y1)).save(file)



