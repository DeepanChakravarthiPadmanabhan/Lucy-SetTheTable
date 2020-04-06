import rospy
from StringIO import StringIO

from mcr_scene_segmentation_python.BoundingBoxVisualizerWrapper_CPP import BoundingBoxVisualizerWrapper
# Initializes roscpp from rospy
from moveit_ros_planning_interface._moveit_roscpp_initializer import roscpp_init


from std_msgs.msg import Bool
from std_msgs.msg import String
from std_msgs.msg import ColorRGBA

from mcr_perception_msgs.msg import BoundingBox


class BoundingBoxVisualizer:
    def __init__(self, topic_name, nodeName='visualization_node'):
        roscpp_init(nodeName, [])
        self.visualizer = BoundingBoxVisualizerWrapper(self._to_cpp(String(topic_name)))

    def _to_cpp(self, msg):
        """Return a serialized string from a ROS message

        Parameters
        ----------
        - msg: a ROS message instance.
        """
        buf = StringIO()
        msg.serialize(buf)
        return buf.getvalue()

    def _from_cpp(self, str_msg, cls):
        """Return a ROS message from a serialized string

        Parameters
        ----------
        - str_msg: str, serialized message
        - cls: ROS message class, e.g. sensor_msgs.msg.LaserScan.
        """
        msg = cls()
        return msg.deserialize(str_msg)

    def publish(self, bounding_box, color_, frame_id, namespace_name):
        self.visualizer.publish(self._to_cpp(bounding_box), self._to_cpp(color_), self._to_cpp(String(frame_id)), self._to_cpp(String(namespace_name)))

    def publish_list(self, bounding_boxes, colors_, frame_id, namespace_name):
        for i in range(len(bounding_boxes)):
            self.publish(bounding_boxes[i], colors_[i], frame_id, namespace_name)

