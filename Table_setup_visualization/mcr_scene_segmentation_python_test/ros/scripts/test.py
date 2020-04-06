#!/usr/bin/env python

import time


from mcr_scene_segmentation_python.BoundingBoxVisualizer import BoundingBoxVisualizer
from mcr_perception_msgs.msg import BoundingBox
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

import random as rand

import rospy

def GenerateListOfBoundingBoxes(numberOfBoxes):
    list_of_boxes = []
    list_of_colors = []
    for i in range(numberOfBoxes):
        list_of_boxes.append(GetRandomBouningBox())
        list_of_colors.append(GetRandomColor())

    return list_of_boxes, list_of_colors

def GetRandomBouningBox():
    bb = BoundingBox()

    x = rand.randint(1,4)*1.
    y = rand.randint(1,4)*1.
    z = rand.randint(3,5)*1.

    bb.vertices.append(Point(x,y,z))
    bb.vertices.append(Point(-x,y,z))
    bb.vertices.append(Point(x,-y,z))
    bb.vertices.append(Point(-x,-y,z))
    bb.vertices.append(Point(x,y,-z))
    bb.vertices.append(Point(-x,y,-z))
    bb.vertices.append(Point(x,-y,-z))
    bb.vertices.append(Point(-x,-y,-z))

    bb.center = Point(x/2., y/2., z/2.)

    return bb

def GetRandomColor():
    c = ColorRGBA()
    c.r = rand.randint(0,255)
    c.g = rand.randint(0,255)
    c.b = rand.randint(0,255)
    return c

rospy.init_node("test_wrapper_visualization")

list_of_boxes, list_of_colors = GenerateListOfBoundingBoxes(5)
visualizer = BoundingBoxVisualizer('test_wrapper_visualization_topic')
time.sleep(1)

visualizer.publish_list(list_of_boxes, list_of_colors, '/visualization_frame', 'namespace_id_')
print('Publication Completed')
