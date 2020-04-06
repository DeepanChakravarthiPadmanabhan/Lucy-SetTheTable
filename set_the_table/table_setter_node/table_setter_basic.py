#! /usr/bin/env python
import tf
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose, Point, Quaternion, PolygonStamped
from mcr_perception_msgs.msg import BoundingBox, BoundingBoxList, Object, ObjectList
from tf.transformations import quaternion_from_euler
from ItemSetterInterface import ItemSetterInterface

from Logger_ import logger

NODE = 'table_setter' 

class TableSetter(ItemSetterInterface):
    def __init__(self):
        # class variables
        self.table_height = 3.0
        self.table_breadth = 3.0
        self.table_width = 2.0
        self.corner_offset = 0.1
        self.ItemInformation = None

    def PerformSetup(self, poses):
        return self.event_in_cb(poses)


    def table_display_cb(self, polygon_convex_hull):
        logger.LogInfo("Displaying table task")
        logger.LogInfo(polygon_convex_hull)

    def set_bounding_box_surface_input(self,msg):
       self.table_bounding_box = BoundingBox()
       self.table_bounding_box = msg

    def event_in_cb(self,msg):
        self.table_properties = self.getTableProperties(self.table_bounding_box) 
        #print(self.table_properties)  
        #print(self.item_properties)    
        self.item_order = self.getItemOrder()
        self.x_offset = 0.05 
        self.y_offset = 0.05
        self.centre_offset = 0.1
        self.item_offset = 0.03 
        self.corner_offset = 0.001
        self.table_orign = Pose(Point(0.0,0.0,0.0), Quaternion(0.0,0.0,0.0,0.0))
        self.item_list = self.getItemList()
        self.set_list = [None]*self.number_of_sets
        self.uniform_set = True
        self.set_x = 0.0
        self.set_y = 0.0
        self.single_config = True
        self.tolerance = 0.02
        self.n_item = len(self.item_order)
        self.set_table()
        bouning_box_list = self.get_bounding_box_list()

        return self.get_pose()

    def getTableProperties(self, bouningBox):
        #return {'height': 1 , 'breadth': 4 , 'width': 2}

        print(bouningBox)

        min_value = [bouningBox.vertices[0].x, bouningBox.vertices[1].x, bouningBox.vertices[2].x]
        max_value = [bouningBox.vertices[0].x, bouningBox.vertices[1].x, bouningBox.vertices[2].x]

        for val in bouningBox.vertices:
            if val.x < min_value[0]:
                min_value[0] = val.x
            if val.y < min_value[1]:
                min_value[1] = val.y
            if val.z < min_value[2]:
                min_value[2] = val.z
                
            if val.x > max_value[0]:
                max_value[0] = val.x
            if val.y > max_value[1]:
                max_value[1] = val.y
            if val.z > max_value[2]:
                max_value[2] = val.z

        print('Printing table parameters:')
        dictionar = {'height': max_value[0] - min_value[0] , 'width': max_value[1] - min_value[1] , 'breadth': max_value[2] - min_value[2] }
        print(dictionar)
        return dictionar 
    

    def getItemOrder(self):
        return self.item_order

    def getItemList(self):
        a = {}
        for item in self.item_order:
            a[item] = [BoundingBox() for _ in range(self.number_of_sets)]
        return a

    def set_table(self):
        if self.check_feasibility():
            self.create_itemset()
            self.calculate_set_center()
        else:
            print('Cannot set the table') # to be published
            self.number_of_sets = self.check_possibile_numberofset()
            print( self.number_of_sets)
            self.item_list = self.getItemList()
            self.set_list = [None]*self.number_of_sets
            self.create_itemset()
            self.calculate_set_center()

    def check_possibile_numberofset(self):
        sets = 1
        while(True):
            if (self.check_feasibile_numberofset(sets)):
                sets += 1
            else:
                break
        return sets - 1  

    def check_feasibile_numberofset(self,number_of_set):
              
        if (self.single_config):            
            return False if (self.set_x* number_of_set + 2* self.corner_offset) - self.table_properties.get('breadth') \
                        > self.tolerance else True
        else:
            return False if (self.set_x* -(-number_of_set // 2) + 2*self.corner_offset )- self.table_properties.get('breadth') \
                       > self.tolerance else True       
       

    def check_feasibility(self):        
        table_area = self.table_properties.get('breadth') * self.table_properties.get('width')
        print("table area")
        print(self.calculate_setarea())
        if table_area // (self.calculate_setarea()* self.number_of_sets):
                        
            if (self.check_feasibile_numberofset(self.number_of_sets)):
                  return True
            else:
                self.single_config =  False
                return self.check_feasibile_numberofset(self.number_of_sets)
        else:
            return False   

    def calculate_setarea(self):        
        self.set_x = 0.0
        self.set_y = 0.0
        width = []
        for item in self.item_order:
            self.set_x += self.item_properties.get(item)['breadth']
            width.append(self.item_properties.get(item)['width'])        
        self.set_y += max(width)
        self.set_x += self.item_offset * (len(self.item_order)-1) + self.x_offset*2
        self.set_y += self.y_offset*2 
        return self.set_x * self.set_y   

    def create_itemset(self):  
        for num_set in range(self.number_of_sets): 
            self.set_list[num_set] =[self.item_list[item][num_set] for item in self.item_order]   

    def calculate_set_center(self):
        if self.single_config:            
            self.assign_center(self.set_list,self.corner_offset,self.set_y/2 ) 
        else:
            sets_1 = self.set_list[0:len(self.set_list)//2] 
            sets_2 = self.set_list[len(self.set_list)//2:len(self.set_list)] 
            self.assign_center(sets_1,self.corner_offset, self.set_y/2)
            self.assign_center(sets_2,self.corner_offset, self.table_properties['width'] - self.set_y/2 )

    def assign_center(self, sets, x,y):
        sets_x = x
        sets_y = y 
        for s in sets:
            sets_x += self.x_offset
            for i,item in enumerate(self.item_order):
                sets_x += self.item_properties.get(item)['breadth'] / 2
                s[i].center = Point(sets_x,sets_y,self.table_properties['height'])   
                self.calculate_bounding_box_points(s[i], item)
                sets_x += self.item_properties.get(item)['breadth'] / 2
                sets_x += self.item_offset
            sets_x += self.x_offset - self.item_offset  
        print(sets_x,sets_y)   

    def calculate_bounding_box_points(self,item, o_name):
        
        point_1 = Point(item.center.x - self.item_properties.get(o_name)['breadth'] / 2 ,
                        item.center.y - self.item_properties.get(o_name)['width'] / 2,
                        0)
        
        point_2 = Point(item.center.x - self.item_properties.get(o_name)['breadth'] / 2 ,
                        item.center.y + self.item_properties.get(o_name)['width'] / 2,
                        0)
        
        point_3 = Point(item.center.x + self.item_properties.get(o_name)['breadth'] / 2 ,
                        item.center.y + self.item_properties.get(o_name)['width'] / 2,
                        0)
        
        point_4 = Point(item.center.x + self.item_properties.get(o_name)['breadth'] / 2 ,
                        item.center.y - self.item_properties.get(o_name)['width'] / 2,
                        0)
        
        point_5 = Point(item.center.x - self.item_properties.get(o_name)['breadth'] / 2 ,
                        item.center.y - self.item_properties.get(o_name)['width'] / 2,
                         self.item_properties.get(o_name)['height'])
        
        point_6 = Point(item.center.x - self.item_properties.get(o_name)['breadth'] / 2 ,
                        item.center.y + self.item_properties.get(o_name)['width'] / 2,
                         self.item_properties.get(o_name)['height'])
        
        point_7 = Point(item.center.x + self.item_properties.get(o_name)['breadth'] / 2 ,
                        item.center.y + self.item_properties.get(o_name)['width'] / 2,
                         self.item_properties.get(o_name)['height'] )
        
        point_8 = Point(item.center.x + self.item_properties.get(o_name)['breadth'] / 2 ,
                        item.center.y - self.item_properties.get(o_name)['width'] / 2,
                         self.item_properties.get(o_name)['height'])
        
        item.vertices = [point_1, point_2, point_3, point_4, 
                         point_5, point_6, point_7, point_8]


    def get_bounding_box_list(self):
        bb_list = BoundingBoxList()
        l = []
        for item in  self.item_order:
           for num in range(self.number_of_sets):
               l.append(self.item_list[item][num])
        bb_list = l
        return bb_list
    
    def get_pose(self):
        object_list = ObjectList()
        p = []
        seq = 0
        for item in  self.item_order:
           for num in range(self.number_of_sets):
               ps = Object()
               ps.name = item + str(num)
               ps.pose.pose.position = self.item_list[item][num].center
               ps.pose.pose.orientation = self.table_orign.orientation
               ps.pose.header.frame_id = 'table'
               ps.pose.header.seq = seq
               ps.pose.header.stamp = rospy.Time.now()
               ps.category = item
               ps.database_id = seq
               ps.bounding_box = self.item_list[item][num]            
               p.append(ps)    
               seq += 1    
        object_list = p 
        return object_list

