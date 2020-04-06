import rospy
from StringIO import StringIO
from set_the_table.LoggerWrapper_CPP import LoggerWrapper


from std_msgs.msg import Bool
from std_msgs.msg import String


class Logger:
    def __init__(self):
        self.logger = LoggerWrapper()

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

    def SetLocalLogger(self, allowLocalLogging, localFileName):
        self.logger.SetLocalLogger(self._to_cpp(Bool(allowLocalLogging)), self._to_cpp(String(localFileName)))

    def SetPublishLogger(self, allowPublishLogging):
        self.logger.SetPublishLogger(self._to_cpp(Bool(allowPublishLogging)))

    def LogMessage(self, messageType, message):
        self.logger.LogMessage(self._to_cpp(String(messageType)), self._to_cpp(String(message)))

    def LogInfo(self, message):
        self.logger.LogInfo(self._to_cpp(String(message)))

    def LogError(self, message):
        self.logger.LogError(self._to_cpp(String(message)))



def InitLogger(nodeName):


    # in order to init roscpp from python. This allows to get an a ros node handle else an error shall code. 
    # This code was copied from the tutorials of the official ros documentation for interoperablity of cpp classes in python 
    from moveit_ros_planning_interface._moveit_roscpp_initializer import roscpp_init
    roscpp_init(nodeName, [])

    # creating single instance of the logger
    logger = Logger()
    return logger
