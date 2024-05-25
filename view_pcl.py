# ---------------------------------------------------------------------
# Project "Track 3D-Objects Over Time"
# Copyright (C) 2020, Dr. Antje Muntzinger / Dr. Andreas Haja.  
#
# Purpose of this file : Loop over all frames in a Waymo Open Dataset file,
#                        detect and track objects and visualize results
#
# You should have received a copy of the Udacity license together with this program.
#
# https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013
# ----------------------------------------------------------------------
#

##################
## Imports

## general package imports
import os
import sys
import numpy as np
import math
import cv2
import matplotlib.pyplot as plt
import copy
import open3d as o3d

## Add current working directory to path
sys.path.append(os.getcwd())

## Waymo open dataset reader
from tools.waymo_reader.simple_waymo_open_dataset_reader import utils as waymo_utils
from tools.waymo_reader.simple_waymo_open_dataset_reader import WaymoDataFileReader, dataset_pb2, label_pb2

## 3d object detection
import student.objdet_pcl as pcl

import misc.objdet_tools as tools 
from misc.helpers import save_object_to_file, load_object_from_file, make_exec_list

## Tracking
from student.filter import Filter
from student.trackmanagement import Trackmanagement
from student.association import Association
from student.measurements import Sensor, Measurement
from misc.evaluation import plot_tracks, plot_rmse, make_movie
import misc.params as params 
 
##################
## Set parameters and perform initializations

## Select Waymo Open Dataset file and frame numbers
data_filename = 'training_segment-1005081002024129653_5313_150_5333_150_with_camera_labels.tfrecord' # Sequence 1
# data_filename = 'training_segment-10072231702153043603_5725_000_5745_000_with_camera_labels.tfrecord' # Sequence 2
# data_filename = 'training_segment-10963653239323173269_1924_000_1944_000_with_camera_labels.tfrecord' # Sequence 3

## Prepare Waymo Open Dataset file for loading
data_fullpath = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'dataset', data_filename) # adjustable path in case this script is called from another working directory
results_fullpath = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'results')
datafile = WaymoDataFileReader(data_fullpath)
datafile_iter = iter(datafile)  # initialize dataset iterator

vis_pause_time = 0 # set pause time between frames in ms (0 = stop between frames until key is pressed)

def view_pcl(pcl, frame_no=None):
    
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pcl[:, 0:3])
    o3d.visualization.draw_geometries_with_editing([pcd])


text = ""
while text != "exit":
    text = input("Select frame (type 'exit' to quit): ")
    frame_no = int(text)
    lidar_pcl = load_object_from_file(results_fullpath, data_filename, 'lidar_pcl', frame_no)
    view_pcl(lidar_pcl, frame_no=frame_no)
