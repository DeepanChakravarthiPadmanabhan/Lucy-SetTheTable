#! /usr/bin/env python

CONST_STRING_NODE_NAME = 'table_setup_node'
CONST_SERVIVENAME_CONFIGURAIONGETTER = 'service_item_configuration_getter'

# Always Initialize logger first
import Logger_
logger = Logger_.InitLogger(CONST_STRING_NODE_NAME)

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int16
import setup_selector_factory

from mcr_perception_msgs.msg import BoundingBox, BoundingBoxList
import geometry_msgs
from std_msgs.msg import String
from std_msgs.msg import Int16
from geometry_msgs.msg import Point

from set_the_table.srv import ItemConfiguration, ItemConfigurationRequest, ItemConfigurationResponse

from mcr_scene_segmentation_python.BoundingBoxVisualizer import BoundingBoxVisualizer
from std_msgs.msg import ColorRGBA

# Remove log once bounding box setup is removed from node to internal mechanism. 
import table_setter_basic

import random as rand


class TableSetupNode:
    def __init__(self):

        self.number_of_sets = 2
        self.table_items = None 
        self.offset = 0.01 
        self.item_order =  ['Fork' , 'Plate', 'Knife', 'Spoon'] # change
        self.bounding_box_surface_input = BoundingBox() 
        self.ItemsInformation = None 
        self.ItemsInformation = self.get_Items_Information()
        self.visualizer = BoundingBoxVisualizer('table_visualizer')
        rospy.sleep(1.0)

    
    def main(self):
        rospy.init_node(CONST_STRING_NODE_NAME)
        logger.LogInfo(CONST_STRING_NODE_NAME + ' initiating')
        self.run_server()    

    def run_server(self):
       
        rospy.Subscriber('/convex_hull_node/table_bounding_box', BoundingBox, self.set_bounding_box_input_cb)
        rospy.Subscriber('~setup_count', Int16, self.set_number_of_sets_cb)
        rospy.Subscriber('~set_trigger', String, self.factory_creation_cb) 
        logger.LogInfo('Node is up and running ....')
        print('Node is up and running ....')
        rospy.spin()   
    
    def factory_creation_cb(self, msg):
        logger.LogInfo("[Started Factory creation]")
        print("[Started Factory creation]")
        object_ = setup_selector_factory.setup_selector_factory()
        nh = object_.setup_selector_factory_method(msg)
        nh.set_number_of_sets(self.number_of_sets)
        nh.set_item_properties(self.get_Item_Properties())
        nh.set_table_items(self.table_items)
        nh.set_offset(self.offset)  
        nh.set_item_order(self.item_order)
        
        # Temporary. This assignment should be removed from here. Instead the algorithm should calculate the bounding box using pcl.
        # Also remove the helper functions and attributes of this node class that are for this supporting purpose only
        # if nh is table_setter_basic.TableSetter:
        if isinstance(nh, table_setter_basic.TableSetter):
            print(self.bounding_box_surface_input)
            if self.bounding_box_surface_input is None:
                logger.LogInfo('Invalid bounding box. Cannot proceed. Returning without performing setup')

            nh.set_bounding_box_surface_input(self.bounding_box_surface_input)
        logger.LogInfo('Factory completed.')
        logger.LogInfo('Determining poses.')
        ret_value = nh.PerformSetup(msg)
        self.PerformVisualization(ret_value)
        logger.LogInfo('Visualization completed.')   

    def PerformVisualization(self, objectList):
        self.visualize_object_list(objectList)
        self.VisualizeTable()


    def set_number_of_sets_cb(self, msg):
       self.number_of_sets = msg.data
        
    def set_bounding_box_input_cb(self, msg):
        self.bounding_box_surface_input = msg       

    def get_Items_Information(self):
        if self.ItemsInformation is None:
            self.ItemsInformation = self.service_get_Items_Informaion()
        return self.ItemsInformation

    def service_get_Items_Informaion(self):
        rospy.wait_for_service(CONST_SERVIVENAME_CONFIGURAIONGETTER)
        try:
            service_get_items_information = rospy.ServiceProxy(CONST_SERVIVENAME_CONFIGURAIONGETTER, ItemConfiguration)
            resp1 = service_get_items_information()
            return resp1
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return None

    def get_Item_Properties(self):
        if(self.ItemsInformation is None):
            self.ItemsInformation = self.get_Items_Information()
        item_properties = {}
        print(self.ItemsInformation)
        for table_item in self.ItemsInformation.tableItems.tableItem:
            item_properties[table_item.item_id] =  {'height': table_item.height , 'breadth': table_item.breadth , 'width': table_item.width}
        self.table_items = len(item_properties)
        return item_properties

    def visualize_object_list(self, object_list_items_):
        i = 0
        for list_item in object_list_items_:
            ns = 'visualization_object_' # + str(i)
            i+=1
            c = ColorRGBA()
            c.r = rand.randint(0,255)
            c.g = rand.randint(0,255)
            c.b = rand.randint(0,255)
            self.visualizer.publish(list_item.bounding_box, c, '/table', ns)
    
    def VisualizeTable(self):
        ns = 'visualization_table'  
        c = ColorRGBA()
        c.r = rand.randint(0,255)
        c.g = rand.randint(0,255)
        c.b = rand.randint(0,255)

        # Since soometimes you do not get proper center from pcl to bounding box
        self.bounding_box_surface_input.center = Point()
        self.bounding_box_surface_input.center.x = 0
        self.bounding_box_surface_input.center.y = 0
        self.bounding_box_surface_input.center.z = 0
        self.visualizer.publish(self.bounding_box_surface_input, c, '/base', ns)            





    
