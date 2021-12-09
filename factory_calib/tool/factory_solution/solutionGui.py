
""" 
Copyright (C) 2021 by Autonomous Driving Group, Shanghai AI Laboratory
Limited. All rights reserved.
Yan Guohang <yanguohang@pjlab.org.cn>
"""

import numpy as np
import cv2
import os
import re
import json
import random
import tkinter as tk
from tkinter import ttk
import tkinter.messagebox as msg
from datatypes import *
from birdviewCanvas import *
from rangeCalculator import *

import matplotlib
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure


# class ItemWindow()
class FactorySolutionGui:
	def __init__(self):
		self.window_width = 840
		self.window_height = 710

		# mode=0, fix sensor position and compute board region
		# mode=1, fix board position and compute sensor region
		self.mode_box = ["Fix Sensors", "Fix Boards"]
		self.mode = 0

		self.sensor_box = ["camera", "lidar"]
		self.board_box = ["camera", "lidar&camera", "lidar"]

		self.item_box = []
		self.item_display_state = []
		self.itemPage = 0

		self.result_box = []

	def start(self):
		self.mainPage = tk.Tk()
		self.mainPage.title("Factory Solution")
		self.mainPage.geometry(str(self.window_width)+'x'+str(self.window_height))
		self.init_main()
		self.canv = BirdviewCanvas()
		self.canv.start(self.mainPage)
		self.mainPage.mainloop()


	def init_main(self):
		rowidx = 0
		space = tk.Label(self.mainPage, text=" ")
		space.grid(row=rowidx)
		rowidx += 1

		mode_button = tk.Button(self.mainPage, text="switch", command=self.switch_mode, width=10, bg="#E6E6FA")
		mode_button.grid(row=rowidx, column=2, sticky="W")
		mode_notice_str = self.mode_box[self.mode] + " Mode"
		self.mode_notice = tk.Label(self.mainPage, text=mode_notice_str, font=("Arial Black", 12), fg="#6A5ACD")
		self.mode_notice.grid(row=rowidx, columnspan=2, sticky="W", padx=10, pady=5)
		rowidx += 1

		space = tk.Label(self.mainPage, text=" ")
		space.grid(row=rowidx)
		rowidx += 1

		subject_label = tk.Label(self.mainPage, text="SUBJECT", font=("Arial Black", 11), fg="#483D88")
		subject_label.grid(row=rowidx, column=0, padx=5)
		self.item_id = tk.StringVar()
		self.item_combo = ttk.Combobox(self.mainPage, values=["new subject"], textvariable=self.item_id, width=30)
		self.item_combo.current(0)
		self.item_combo.grid(row=rowidx, column=1, columnspan=3, padx=5, sticky="W")
		self.item_combo.bind("<<ComboboxSelected>>", self.select_item_combo)
		rowidx += 1

		# parameters
		# sensor config
		sensor_label = tk.Label(self.mainPage, text="Sensor Config", font=("Arial Black", 10), fg="#696969")
		sensor_label.grid(row=rowidx, padx=5, pady=5)
		rowidx += 1

		name = "camera" + str(len(self.item_box))
		name_label = tk.Label(self.mainPage, text="Name:")
		name_label.grid(row=rowidx, column=0, padx=5)
		self.sensor_name_label = tk.Label(self.mainPage, text=name, font=("Arial Black", 10), fg="#778899")
		self.sensor_name_label.grid(row=rowidx, column=1, columnspan=2, padx=5, sticky="W")
		rowidx += 1

		sensor_type_label = tk.Label(self.mainPage, text="Type:")
		sensor_type_label.grid(row=rowidx, column=0, padx=5, pady=5)
		self.sensor_type_combo = ttk.Combobox(self.mainPage, values=self.sensor_box)
		self.sensor_type_combo.current(0)
		self.sensor_type_combo.grid(row=rowidx, column=1, columnspan=2, padx=5, pady=5, sticky="W")
		self.sensor_type_combo.bind("<<ComboboxSelected>>", self.select_sensor_type_combo)
		rowidx += 1

		poslabel = tk.Label(self.mainPage, text="x,y,z (in m):")
		poslabel.grid(row=rowidx, column=0, padx=5)
		self.sensor_posx_entry = tk.Entry(self.mainPage, bd=1, width=10)
		self.sensor_posx_entry.grid(row=rowidx, column=1, sticky="W", padx=5)
		self.sensor_posy_entry = tk.Entry(self.mainPage, bd=1, width=10)
		self.sensor_posy_entry.grid(row=rowidx, column=2, sticky="W", padx=5)
		self.sensor_posz_entry = tk.Entry(self.mainPage, bd=1, width=10)
		self.sensor_posz_entry.grid(row=rowidx, column=3, sticky="W", padx=5)
		rowidx += 1

		anglelabel = tk.Label(self.mainPage, text="yaw,pitch,roll:")
		anglelabel.grid(row=rowidx, column=0, padx=5)
		self.sensor_yaw_entry = tk.Entry(self.mainPage, bd=1, width=10)
		self.sensor_yaw_entry.grid(row=rowidx, column=1, sticky="W", padx=5)
		self.sensor_pitch_entry = tk.Entry(self.mainPage, bd=1, width=10)
		self.sensor_pitch_entry.grid(row=rowidx, column=2, sticky="W", padx=5)
		self.sensor_pitch_entry["textvariable"] = tk.StringVar(self.mainPage, value="0")
		self.sensor_roll_entry = tk.Entry(self.mainPage, bd=1, width=10)
		self.sensor_roll_entry.grid(row=rowidx, column=3, sticky="W", padx=5)
		self.sensor_roll_entry["textvariable"] = tk.StringVar(self.mainPage, value="0")
		rowidx += 1

		space = tk.Label(self.mainPage, text=" ")
		space.grid(row=rowidx)
		rowidx += 1

		fovx_label = tk.Label(self.mainPage, text="horizonal fov:")
		fovx_label.grid(row=rowidx, column=0, padx=5)
		self.fovx_entry = tk.Entry(self.mainPage, bd=1, width=25)
		self.fovx_entry.grid(row=rowidx, column=1, columnspan=2, sticky="W", padx=5)
		rowidx += 1
		fovy_label = tk.Label(self.mainPage, text="vertical fov:")
		fovy_label.grid(row=rowidx, column=0, padx=5)
		self.fovy_entry = tk.Entry(self.mainPage, bd=1, width=25)
		self.fovy_entry.grid(row=rowidx, column=1, columnspan=2, sticky="W", padx=5)
		rowidx += 1
		img_resolution_label = tk.Label(self.mainPage, text="Image Resolution:")
		img_resolution_label.grid(row=rowidx, column=0, padx=5)
		self.img_width_entry = tk.Entry(self.mainPage, bd=1, width=10)
		self.img_width_entry.grid(row=rowidx, column=1, sticky="W", padx=5)
		self.img_height_entry = tk.Entry(self.mainPage, bd=1, width=10)
		self.img_height_entry.grid(row=rowidx, column=2, sticky="W", padx=5)
		self.img_width_entry["textvariable"] = tk.StringVar(self.mainPage, value="1920")
		self.img_height_entry["textvariable"] = tk.StringVar(self.mainPage, value="1080")	

		rowidx += 1
		angular_resolution_label = tk.Label(self.mainPage, text="Angular Resolution:")
		angular_resolution_label.grid(row=rowidx, column=0, padx=5)
		self.angular_resolution_entry = tk.Entry(self.mainPage, bd=1, width=25)
		self.angular_resolution_entry.grid(row=rowidx, column=1, columnspan=2, sticky="W", padx=5)
		self.angular_resolution_entry["state"] = "disabled"
		rowidx += 1

		# board config
		board_label = tk.Label(self.mainPage, text="Board Config", font=("Arial Black", 10), fg="#696969")
		board_label.grid(row=rowidx, padx=5, pady=5)
		rowidx += 1

		name = "board" + str(len(self.item_box))
		name_label = tk.Label(self.mainPage, text="Name:")
		name_label.grid(row=rowidx, column=0, padx=5)
		self.board_name_label = tk.Label(self.mainPage, text=name, font=("Arial Black", 10), fg="#778899")
		self.board_name_label.grid(row=rowidx, column=1, columnspan=2, padx=5, sticky="W")
		rowidx += 1

		board_type_label = tk.Label(self.mainPage, text="Type:")
		board_type_label.grid(row=rowidx, column=0, padx=5, pady=5)
		self.board_type_combo = ttk.Combobox(self.mainPage, values=self.board_box)
		self.board_type_combo.current(0)
		self.board_type_combo.grid(row=rowidx, column=1, columnspan=2, padx=5, pady=5, sticky="W")
		self.board_type_combo.bind("<<ComboboxSelected>>", self.select_board_type_combo)
		rowidx += 1

		poslabel = tk.Label(self.mainPage, text="x,y,z (in m):")
		poslabel.grid(row=rowidx, column=0, padx=5)
		self.board_posx_entry = tk.Entry(self.mainPage, bd=1, width=10)
		self.board_posx_entry.grid(row=rowidx, column=1, sticky="W", padx=5)
		self.board_posy_entry = tk.Entry(self.mainPage, bd=1, width=10)
		self.board_posy_entry.grid(row=rowidx, column=2, sticky="W", padx=5)
		self.board_posz_entry = tk.Entry(self.mainPage, bd=1, width=10)
		self.board_posz_entry.grid(row=rowidx, column=3, sticky="W", padx=5)
		self.board_posx_entry["state"] = "disabled"
		self.board_posy_entry["state"] = "disabled"
		self.board_posz_entry["state"] = "disabled"
		rowidx += 1

		anglelabel = tk.Label(self.mainPage, text="yaw:")
		anglelabel.grid(row=rowidx, column=0, padx=5)
		self.board_yaw_entry = tk.Entry(self.mainPage, bd=1, width=10)
		self.board_yaw_entry.grid(row=rowidx, column=1, sticky="W", padx=5)
		rowidx += 1

		space = tk.Label(self.mainPage, text=" ")
		space.grid(row=rowidx)
		rowidx += 1

		board_size_label = tk.Label(self.mainPage, text="Board width*height:")
		board_size_label.grid(row=rowidx, column=0, padx=5)
		self.board_width_entry = tk.Entry(self.mainPage, bd=1, width=10)
		self.board_width_entry.grid(row=rowidx, column=1, sticky="W", padx=5)
		self.board_height_entry = tk.Entry(self.mainPage, bd=1, width=10)
		self.board_height_entry.grid(row=rowidx, column=2, sticky="W", padx=5)
		self.board_width_entry["textvariable"] = tk.StringVar(self.mainPage, value="2.1")
		self.board_height_entry["textvariable"] = tk.StringVar(self.mainPage, value="0.9")

		rowidx += 1

		grid_label = tk.Label(self.mainPage, text="Grid Size:")
		grid_label.grid(row=rowidx, column=0, padx=5)
		self.board_grid_entry = tk.Entry(self.mainPage, bd=1)
		self.board_grid_entry.grid(row=rowidx, column=1, columnspan=2, sticky="W", padx=5)
		rowidx += 1

		circle_label = tk.Label(self.mainPage, text="Radius:")
		circle_label.grid(row=rowidx, column=0, padx=5)
		self.board_radius_entry = tk.Entry(self.mainPage, bd=1)
		self.board_radius_entry.grid(row=rowidx, column=1, columnspan=2, sticky="W", padx=5)
		self.board_radius_entry["state"] = "disabled"
		rowidx += 1

		space = tk.Label(self.mainPage, text=" ")
		space.grid(row=rowidx)
		rowidx += 1

		self.additem_button = tk.Button(self.mainPage, text="Add New Subject", command=self.addItem)
		self.additem_button.grid(row=rowidx, column=0)

		self.item_change_b = tk.Button(self.mainPage, text="change", command=self.change_item, state="disabled", width=10)
		self.item_change_b.grid(row=rowidx, column=1, padx=5)
		self.item_hide_b = tk.Button(self.mainPage, text="hide", command=self.hide_item, state="disabled", width=10)
		self.item_hide_b.grid(row=rowidx, column=2, padx=5)
		self.item_delete_b = tk.Button(self.mainPage, text="delete", command=self.delete_item, state="disabled", width=10)
		self.item_delete_b.grid(row=rowidx, column=3, padx=5)
		rowidx += 1

		self.save_result_button = tk.Button(self.mainPage, text="Save Result", command=self.saveResult, width=30)
		self.save_result_button.grid(row=rowidx, columnspan=4, pady=10)

		self.sensor_config_entrys = []
		self.sensor_config_entrys.append(self.sensor_posx_entry)
		self.sensor_config_entrys.append(self.sensor_posy_entry)
		self.sensor_config_entrys.append(self.sensor_posz_entry)
		self.sensor_config_entrys.append(self.sensor_yaw_entry)
		self.sensor_config_entrys.append(self.sensor_pitch_entry)
		self.sensor_config_entrys.append(self.sensor_roll_entry)
		self.sensor_config_entrys.append(self.fovx_entry)
		self.sensor_config_entrys.append(self.fovy_entry)
		self.sensor_config_entrys.append(self.img_width_entry)
		self.sensor_config_entrys.append(self.img_height_entry)
		self.sensor_config_entrys.append(self.angular_resolution_entry)

		self.board_config_entrys = []
		self.board_config_entrys.append(self.board_posx_entry)
		self.board_config_entrys.append(self.board_posy_entry)
		self.board_config_entrys.append(self.board_posz_entry)
		self.board_config_entrys.append(self.board_yaw_entry)
		self.board_config_entrys.append(self.board_width_entry)
		self.board_config_entrys.append(self.board_height_entry)
		self.board_config_entrys.append(self.board_grid_entry)
		self.board_config_entrys.append(self.board_radius_entry)


	def switch_mode(self):
		self.mode = (self.mode+1) % len(self.mode_box)
		mode_notice_str = self.mode_box[self.mode] + " Mode"
		self.mode_notice["text"] = mode_notice_str
		msg.showinfo("Switch Mode", "Switch Mode!\nAll data refreshed.\n")
		self.item_combo["values"] = self.item_combo["values"][:1]
		self.item_box = []
		self.item_display_state = []
		self.canv.clear()
		self.item_combo.current(0)
		self.refresh_config_page()
		self.result_box = []
		if (self.mode == 0):
			self.sensor_posx_entry["state"] = "normal"
			self.sensor_posy_entry["state"] = "normal"
			self.sensor_posz_entry["state"] = "normal"
			self.board_posx_entry["state"] = "disabled"
			self.board_posy_entry["state"] = "disabled"
			self.board_posz_entry["state"] = "disabled"
		elif (self.mode == 1):
			self.sensor_posx_entry["state"] = "normal"
			self.sensor_posy_entry["state"] = "normal"
			self.sensor_posz_entry["state"] = "normal"
			self.board_posx_entry["state"] = "normal"
			self.board_posy_entry["state"] = "normal"
			self.board_posz_entry["state"] = "normal"			
		

	def refresh_config_page(self):
		number_str = "-1"
		if (len(self.item_combo["values"]) > 1):
			number_str =  re.search("\d+", self.item_combo["values"][-1]).group()
		# self.sensor_name_label["text"] = self.sensor_type_combo.get() + number_str
		# self.sensor_name_label["text"] = "camera" + str(len(self.item_box))
		self.sensor_name_label["text"] = "camera" + str(int(number_str) + 1)
		self.sensor_type_combo.current(0)

		self.sensor_posx_entry["textvariable"] = tk.StringVar(self.mainPage, value="")
		self.sensor_posy_entry["textvariable"] = tk.StringVar(self.mainPage, value="")
		self.sensor_posz_entry["textvariable"] = tk.StringVar(self.mainPage, value="")

		self.sensor_yaw_entry["textvariable"] = tk.StringVar(self.mainPage, value="")
		self.sensor_pitch_entry["textvariable"] = tk.StringVar(self.mainPage, value="0")
		self.sensor_roll_entry["textvariable"] = tk.StringVar(self.mainPage, value="0")

		self.fovx_entry["textvariable"] = tk.StringVar(self.mainPage, value="")
		self.fovy_entry["textvariable"] = tk.StringVar(self.mainPage, value="")
		self.img_width_entry["state"] = "normal"
		self.img_height_entry["state"] = "normal"
		self.img_width_entry["textvariable"] = tk.StringVar(self.mainPage, value="1920")
		self.img_height_entry["textvariable"] = tk.StringVar(self.mainPage, value="1080")			
		
		self.angular_resolution_entry["textvariable"] = tk.StringVar(self.mainPage, value="")
		self.angular_resolution_entry["state"] = "disabled"

		# board config
		# self.board_name_label["text"] = "board" + str(len(self.item_box))
		self.board_name_label["text"] = "board" + str(int(number_str) + 1)
		self.board_type_combo.current(0)

		self.board_posx_entry["textvariable"] = tk.StringVar(self.mainPage, value="")
		self.board_posy_entry["textvariable"] = tk.StringVar(self.mainPage, value="")
		self.board_posz_entry["textvariable"] = tk.StringVar(self.mainPage, value="")
		self.board_yaw_entry["textvariable"] = tk.StringVar(self.mainPage, value="")

		self.board_width_entry["textvariable"] = tk.StringVar(self.mainPage, value="2.1")
		self.board_height_entry["textvariable"] = tk.StringVar(self.mainPage, value="0.9")

		self.board_grid_entry["state"] = "normal"
		self.board_grid_entry["textvariable"] = tk.StringVar(self.mainPage, value="")
		self.board_radius_entry["state"] = "disabled"
		self.board_radius_entry["textvariable"] = tk.StringVar(self.mainPage, value="")

		self.item_change_b["state"] = "disabled"
		self.item_hide_b["state"] = "disabled"
		self.item_delete_b["state"] = "disabled"
		self.additem_button["state"] = "normal"


	def review_existed_object(self, item_id):
		item = self.item_box[item_id]
		self.sensor_name_label["text"] = item.sensor.name
		self.sensor_type_combo.current(self.sensor_box.index(item.sensor.type))
		if (self.mode == 0):
			self.sensor_posx_entry["textvariable"] = tk.StringVar(self.mainPage, value=str(item.sensor.posx))
			self.sensor_posy_entry["textvariable"] = tk.StringVar(self.mainPage, value=str(item.sensor.posy))
			self.sensor_posz_entry["textvariable"] = tk.StringVar(self.mainPage, value=str(item.sensor.posz))
		elif (self.mode == 1):
			self.sensor_posx_entry["textvariable"] = tk.StringVar(self.mainPage, value=str(item.sensor.posx))
			self.sensor_posy_entry["textvariable"] = tk.StringVar(self.mainPage, value=str(item.sensor.posy))
			self.sensor_posz_entry["textvariable"] = tk.StringVar(self.mainPage, value=str(item.sensor.posz))
			self.board_posx_entry["textvariable"] = tk.StringVar(self.mainPage, value=str(item.board.posx))
			self.board_posy_entry["textvariable"] = tk.StringVar(self.mainPage, value=str(item.board.posy))
			self.board_posz_entry["textvariable"] = tk.StringVar(self.mainPage, value=str(item.board.posz))

		self.sensor_yaw_entry["textvariable"] = tk.StringVar(self.mainPage, value=str(item.sensor.yaw))
		self.sensor_pitch_entry["textvariable"] = tk.StringVar(self.mainPage, value=str(item.sensor.pitch))
		self.sensor_roll_entry["textvariable"] = tk.StringVar(self.mainPage, value=str(item.sensor.roll))

		self.fovx_entry["textvariable"] = tk.StringVar(self.mainPage, value=str(item.sensor.horizonal_fov))
		self.fovy_entry["textvariable"] = tk.StringVar(self.mainPage, value=str(item.sensor.vertical_fov))
		if (item.sensor.type == "camera"):
			self.img_width_entry["state"] = "normal"
			self.img_height_entry["state"] = "normal"
			self.img_width_entry["textvariable"] = tk.StringVar(self.mainPage, value=str(item.sensor.img_width))
			self.img_height_entry["textvariable"] = tk.StringVar(self.mainPage, value=str(item.sensor.img_height))			
			self.angular_resolution_entry["textvariable"] = tk.StringVar(self.mainPage, value="")
			self.angular_resolution_entry["state"] = "disabled"
		elif (item.sensor.type == "lidar"):
			self.img_width_entry["state"] = "disabled"
			self.img_height_entry["state"] = "disabled"
			self.img_width_entry["textvariable"] = tk.StringVar(self.mainPage, value="")
			self.img_height_entry["textvariable"] = tk.StringVar(self.mainPage, value="")			
			self.angular_resolution_entry["textvariable"] = tk.StringVar(self.mainPage, value=str(item.sensor.angular_resolution))
			self.angular_resolution_entry["state"] = "normal"

		# board config
		self.board_name_label["text"] = item.board.name
		self.board_type_combo.current(self.board_box.index(item.board.type))

		self.board_yaw_entry["textvariable"] = tk.StringVar(self.mainPage, value=str(item.board.yaw))

		self.board_width_entry["textvariable"] = tk.StringVar(self.mainPage, value=str(item.board.width))
		self.board_height_entry["textvariable"] = tk.StringVar(self.mainPage, value=str(item.board.height))

		if ("camera" in item.board.type):
			self.board_grid_entry["state"] = "normal"
			self.board_grid_entry["textvariable"] = tk.StringVar(self.mainPage, value=str(item.board.grid_size))
		else:
			self.board_grid_entry["state"] = "disabled"
			self.board_grid_entry["textvariable"] = tk.StringVar(self.mainPage, value="")
		if ("lidar" in item.board.type):
			self.board_radius_entry["state"] = "normal"
			self.board_radius_entry["textvariable"] = tk.StringVar(self.mainPage, value=str(item.board.radius))
		else:
			self.board_radius_entry["state"] = "disabled"
			self.board_radius_entry["textvariable"] = tk.StringVar(self.mainPage, value="")


	def select_sensor_type_combo(self, event):
		number_str =  re.search("\d+", self.sensor_name_label["text"]).group()
		self.sensor_name_label["text"] = self.sensor_type_combo.get() + number_str
		if (self.sensor_type_combo.get() == "lidar"):
			self.angular_resolution_entry["state"] = "normal"
			self.img_width_entry["state"] = "disabled"
			self.img_height_entry["state"] = "disabled"
			self.img_width_entry["textvariable"] = tk.StringVar(self.mainPage, value="")
			self.img_height_entry["textvariable"] = tk.StringVar(self.mainPage, value="")
		elif(self.sensor_type_combo.get() == "camera"):
			self.angular_resolution_entry["state"] = "disabled"
			self.angular_resolution_entry["textvariable"] = tk.StringVar(self.mainPage, value="")
			self.img_width_entry["state"] = "normal"
			self.img_height_entry["state"] = "normal"
			self.img_width_entry["textvariable"] = tk.StringVar(self.mainPage, value="1920")
			self.img_height_entry["textvariable"] = tk.StringVar(self.mainPage, value="1080")


	def select_board_type_combo(self, event):
		if (self.board_type_combo.get() == "camera"):
			self.board_radius_entry["state"] = "disabled"
			self.board_radius_entry["textvariable"] = tk.StringVar(self.mainPage, value="")
			self.board_grid_entry["state"] = "normal"
		elif (self.board_type_combo.get() == "lidar&camera"):
			self.board_radius_entry["state"] = "normal"
			self.board_grid_entry["state"] = "normal"
		elif (self.board_type_combo.get() == "lidar"):
			self.board_radius_entry["state"] = "normal"
			self.board_grid_entry["state"] = "disabled"
			self.board_grid_entry["textvariable"] = tk.StringVar(self.mainPage, value="")


	def select_item_combo(self, event):
		val = self.item_combo["values"].index(self.item_combo.get())
		if (val == 0):
			self.item_change_b["state"] = "disabled"
			self.item_hide_b["state"] = "disabled"
			self.item_delete_b["state"] = "disabled"
			self.additem_button["state"] = "normal"
			self.refresh_config_page()
		else:
			self.item_change_b["state"] = "normal"
			if (self.item_display_state[val-1] == 0):
				self.item_hide_b["text"] = "show"
				self.item_hide_b["bg"] = "#808080"
			else:
				self.item_hide_b["text"] = "hide"
				self.item_hide_b["bg"] = "#F5F5F5"				
			self.item_hide_b["state"] = "normal"
			self.item_delete_b["state"] = "normal"
			self.additem_button["state"] = "disabled"
			self.review_existed_object(val-1)


	def change_item(self):
		item_id = self.item_combo["values"].index(self.item_combo.get()) - 1
		whether_success, item = self.getParamCheck()
		if (whether_success):
			self.item_combo["values"] = self.item_combo["values"][:(item_id+1)] + (item.name,) + self.item_combo["values"][(item_id+2):]
			self.item_box[item_id] = item
			self.item_combo.current(item_id+1)
			if (self.mode == 0):
				board_region = getBoardRegion(item)
				self.result_box[item_id] = board_region
				self.canv.drawBoardRegion(item_id, item, board_region)
			elif (self.mode == 1):
				sensor_region = getSensorRegion(item)
				self.result_box[item_id] = sensor_region
				self.canv.drawSensorRegion(item_id, item, sensor_region)
			self.item_display_state[item_id] = 1
			self.item_hide_b["text"] = "hide"
			self.item_hide_b["bg"] = "#F5F5F5"
		# self.shwo_change_page(item_id)


	def hide_item(self):
		item_id = self.item_combo["values"].index(self.item_combo.get()) - 1
		self.item_display_state[item_id] = abs(self.item_display_state[item_id]-1)
		if (self.item_display_state[item_id] == 0):
			self.item_hide_b["text"] = "show"
			self.item_hide_b["bg"] = "#808080"
			self.canv.hideItem(item_id)
		else:
			self.item_hide_b["text"] = "hide"
			self.item_hide_b["bg"] = "#F5F5F5"
			self.canv.showItem(item_id)


	def delete_item(self):
		item_id = self.item_combo["values"].index(self.item_combo.get()) - 1
		self.canv.deleteItem(item_id, self.item_box[item_id])
		self.item_box.pop(item_id)
		self.result_box.pop(item_id)
		self.item_display_state.pop(item_id)
		self.item_combo.current(0)
		self.select_item_combo(0)
		self.item_change_b["state"] = "disabled"
		self.item_hide_b["state"] = "disabled"
		self.item_delete_b["state"] = "disabled"
		self.item_combo["values"] = self.item_combo["values"][:(item_id+1)] + self.item_combo["values"][(item_id+2):]


	def getParamCheck(self):
		sensor_data = Sensor()
		board_data = Board()
		item = SensorBoardItem()
		if (self.sensor_type_combo.get() == "camera"):
			if (self.board_type_combo.get() == "lidar"):
				msg.showinfo("Param Error", "ERROR!\nFailed to add or change.\nSensor and Board type is not corresponding.\n")
				return False, item
		if (self.sensor_type_combo.get() == "lidar"):
			if (self.board_type_combo.get() == "camera"):
				msg.showinfo("Param Error", "ERROR!\nFailed to add or change.\nSensor and Board type is not corresponding.\n")
				return False, item
		# check whether all blanks are filled
		for entry in self.sensor_config_entrys:
			if (entry["state"] == "normal" and entry.get() == ""):
				msg.showinfo("Param Error", "ERROR!\nFailed to add or change.\nPlease fill all empty blanks\n")
				return False, item
		for entry in self.board_config_entrys:
			if (entry["state"] == "normal" and entry.get() == ""):
				msg.showinfo("Param Error", "ERROR!\nFailed to add or change.\nPlease fill all empty blanks\n")
				return False, item
		# read configs
		sensor_data.name = self.sensor_name_label["text"]
		sensor_data.type = self.sensor_type_combo.get()
		if (self.sensor_posx_entry["state"] == "normal"):
			sensor_data.posx = float(self.sensor_posx_entry.get())
			sensor_data.posy = float(self.sensor_posy_entry.get())
			sensor_data.posz = float(self.sensor_posz_entry.get())
		sensor_data.yaw = float(self.sensor_yaw_entry.get())
		sensor_data.pitch = float(self.sensor_pitch_entry.get())
		sensor_data.roll = float(self.sensor_roll_entry.get())
		sensor_data.horizonal_fov = float(self.fovx_entry.get())
		sensor_data.vertical_fov = float(self.fovy_entry.get())
		if (self.img_width_entry["state"] == "normal"):
			sensor_data.img_width = int(self.img_width_entry.get())
			sensor_data.img_height = int(self.img_height_entry.get())
		if (self.angular_resolution_entry["state"] == "normal"):
			sensor_data.angular_resolution = float(self.angular_resolution_entry.get())

		board_data.name = self.board_name_label["text"]
		board_data.type = self.board_type_combo.get()
		if (self.board_posx_entry["state"] == "normal"):
			board_data.posx = float(self.board_posx_entry.get())
			board_data.posy = float(self.board_posy_entry.get())
			board_data.posz = float(self.board_posz_entry.get())
		board_data.yaw = float(self.board_yaw_entry.get())
		board_data.width = float(self.board_width_entry.get())
		board_data.height = float(self.board_height_entry.get())
		if (self.board_grid_entry["state"] == "normal"):
			board_data.grid_size = float(self.board_grid_entry.get())
		if (self.board_radius_entry["state"] == "normal"):
			board_data.radius = float(self.board_radius_entry.get())

		# check sensor and board yaw
		if (sensor_data.yaw < 0 or sensor_data.yaw > 360):
			msg.showinfo("Param Error", "ERROR!\nFailed to add or change.\nYaw should be [0, 360]\n")
			return False, item
		if (sensor_data.pitch < -90 or sensor_data.pitch > 90):
			msg.showinfo("Param Error", "ERROR!\nFailed to add or change.\nPitch should be [-90, 90]\n")
			return False, item
		if (sensor_data.roll < -90 or sensor_data.roll > 90):
			msg.showinfo("Param Error", "ERROR!\nFailed to add or change.\nRoll should be [-90, 90]\n")
			return False, item
		if (board_data.yaw < 0 or board_data.yaw > 360):
			msg.showinfo("Param Error", "ERROR!\nFailed to add or change.\nYaw should be [0, 360]\n")
			return False, item		
		yaw_diff = board_data.yaw - sensor_data.yaw
		if (yaw_diff > 180): yaw_diff -= 360
		if (yaw_diff < -180): yaw_diff += 360
		if (yaw_diff >= -90 and yaw_diff <= 90):
			msg.showinfo("Param Error", "ERROR!\nFailed to add or change.\nBoard is not in the view of sensor\n")
			return False, item	

		# check parameter value validity
		if (self.board_posz_entry["state"] == "normal"):
			if (board_data.posz * 2 < board_data.height):
				msg.showinfo("Param Error", "Error!\nBoard z position is smaller than half of the board height.\nPlease check!\n")
				return True, item

		item.sensor = sensor_data
		item.board = board_data
		item.name = sensor_data.name + "-" + board_data.name
		return True, item


	def addItem(self):
		whether_success, item = self.getParamCheck()
		if (not whether_success):
			return
		self.item_combo["values"] += (item.name,)
		self.item_box.append(item)
		self.item_display_state.append(1)
		self.refresh_config_page()
		if (self.mode == 0):
			# compute board region
			board_region = getBoardRegion(item)
			self.result_box.append(board_region)
			self.canv.drawBoardRegion(len(self.item_box)-1, item, board_region)
		elif (self.mode == 1):
			sensor_region = getSensorRegion(item)
			self.result_box.append(sensor_region)
			self.canv.drawSensorRegion(len(self.item_box)-1, item, sensor_region)


	def saveRegionJson(self, path):
		file_name = "result.json"
		if (self.mode == 0):
			file_name = path + "/" + "board_region.json"
		elif (self.mode == 1):
			file_name = path + "/" + "sensor_region.json"
		data = {}
		data["mode"] = self.mode_box[self.mode]
		for i in range(len(self.item_box)):
			name = self.item_box[i].name
			data[name] = {}
			if (self.mode == 0):
				data[name]["origin_pt"] = self.result_box[i].origin_pt
				data[name]["dmin"] = self.result_box[i].dmin
				data[name]["dmax"] = self.result_box[i].dmax
				data[name]["angle_range"] = self.result_box[i].angle_range
				data[name]["side_pt"] = self.result_box[i].side_pt
				data[name]["side_dmin"] = self.result_box[i].side_dmin
				data[name]["side_dmax"] = self.result_box[i].side_dmax
				data[name]["side_angle_range"] = self.result_box[i].side_angle_range
				data[name]["bestpos"] = self.result_box[i].bestpos
				data[name]["bestdist"] = self.result_box[i].bestdist
			elif (self.mode == 1):
				data[name]["p1"] = self.result_box[i].p1
				data[name]["p2"] = self.result_box[i].p2
				data[name]["dmin"] = self.result_box[i].dmin
				data[name]["p1_line_angle"] = self.result_box[i].p1_line_angle
				data[name]["p2_line_angle"] = self.result_box[i].p2_line_angle
				data[name]["joint_pt"] = self.result_box[i].joint_pt
				data[name]["p1_line_vector"] = self.result_box[i].p1_line_vector
				data[name]["p2_line_vector"] = self.result_box[i].p2_line_vector
				
				data[name]["side_p1"] = self.result_box[i].side_p1
				data[name]["side_p2"] = self.result_box[i].side_p2
				data[name]["side_dmin"] = self.result_box[i].side_dmin
				data[name]["side_dmax"] = self.result_box[i].side_dmax
				data[name]["side_p1_line_angle"] = self.result_box[i].side_p1_line_angle
				data[name]["side_p2_line_angle"] = self.result_box[i].side_p2_line_angle
				data[name]["side_joint_pt"] = self.result_box[i].side_joint_pt
				data[name]["side_p1_line_vector"] = self.result_box[i].side_p1_line_vector
				data[name]["side_p2_line_vector"] = self.result_box[i].side_p2_line_vector
		saveJsonFile(file_name, data)


	def saveItemConfig(self, path):
		file_name = path + "/" + "sensor_board_config.json"
		data = {}
		data["mode"] = self.mode_box[self.mode]
		for i in range(len(self.item_box)):
			name = self.item_box[i].name
			data[name] = {}
			data[name]["sensor"] = {}
			data[name]["board"] = {}
			sensor = self.item_box[i].sensor
			board = self.item_box[i].board

			data[name]["sensor"]["name"] = sensor.name
			data[name]["sensor"]["type"] = sensor.type
			if (self.mode == 0 or self.mode == 1):
				data[name]["sensor"]["posx"] = sensor.posx
				data[name]["sensor"]["posy"] = sensor.posy
				data[name]["sensor"]["posz"] = sensor.posz
			data[name]["sensor"]["yaw"] = sensor.yaw
			data[name]["sensor"]["pitch"] = sensor.pitch
			data[name]["sensor"]["roll"] = sensor.roll
			data[name]["sensor"]["horizonal_fov"] = sensor.horizonal_fov
			data[name]["sensor"]["vertical_fov"] = sensor.vertical_fov
			if (sensor.type == "camera"):
				data[name]["sensor"]["img_width"] = sensor.img_width
				data[name]["sensor"]["img_height"] = sensor.img_height
			elif (sensor.type == "lidar"):
				data[name]["sensor"]["angular_resolution"] = sensor.angular_resolution

			data[name]["board"]["name"] = board.name
			data[name]["board"]["type"] = board.type
			if (self.mode == 1):
				data[name]["board"]["posx"] = board.posx
				data[name]["board"]["posy"] = board.posy
				data[name]["board"]["posz"] = board.posz
			data[name]["board"]["yaw"] = board.yaw
			data[name]["board"]["width"] = board.width
			data[name]["board"]["height"] = board.height
			if ("camera" in board.type):
				data[name]["board"]["grid_size"] = board.grid_size
			if ("lidar" in board.type):
				data[name]["board"]["radius"] = board.radius
		saveJsonFile(file_name, data)


	def saveResult(self):
		output_dir = "./output"
		self.saveItemConfig("./output")
		self.saveRegionJson("./output")
		self.canv.saveImages("./output", self.item_box)

def saveJsonFile(file_path, data):
	with open(file_path, 'w') as f:
		json.dump(data, f, sort_keys=True, indent=4, separators=(',', ': '))


gui = FactorySolutionGui()
gui.start()