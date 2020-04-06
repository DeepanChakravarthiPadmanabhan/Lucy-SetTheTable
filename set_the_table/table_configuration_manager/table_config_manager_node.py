#!/usr/bin/env python

# Always Initialize logger first
import Logger_

CONST_NODENAME_CONFIGURATIONMANAGER = 'node_config_manager'
CONST_SERVIVENAME_CONFIGURAIONGETTER = 'service_item_configuration_getter'
CONST_SUBSCRIBERNAME_REFRESH = 'refresh_configuration'

logger = Logger_.InitLogger(CONST_NODENAME_CONFIGURATIONMANAGER)


from set_the_table.srv import ItemConfiguration, ItemConfigurationResponse
import configuration_loader_wrapper
import rospy
from std_msgs.msg import String

class TableSpecificationManager:
    def main(self):
        self.run_server() 

    def get_item_configuration(self, itemConfigurationReqauest):
        logger.LogInfo("Received item configuration service message.")
        itemConfigurationResponse = ItemConfigurationResponse()
        itemConfigurationResponse.tableItems = configuration_loader_wrapper.GetTableItems()
        logger.LogInfo('Responding with message '  + str(String(itemConfigurationResponse)))
        return itemConfigurationResponse

    def load_item_configuration(self, message):
        logger.LogInfo("Received message to refresh items configuration.")
        configuration_loader_wrapper.Refresh()
        logger.LogInfo("Items load completed.")

    def run_server(self):
        rospy.init_node(CONST_NODENAME_CONFIGURATIONMANAGER)
        logger.LogInfo(CONST_NODENAME_CONFIGURATIONMANAGER + " initialization completed.")
        self.service = rospy.Service(CONST_SERVIVENAME_CONFIGURAIONGETTER, ItemConfiguration, self.get_item_configuration)
        logger.LogInfo(CONST_SERVIVENAME_CONFIGURAIONGETTER + " service started successfully.")
        self.subscriber = rospy.Subscriber(CONST_SUBSCRIBERNAME_REFRESH, String, self.load_item_configuration)
        logger.LogInfo(CONST_SUBSCRIBERNAME_REFRESH + " topic subscribed successfully.")
        logger.LogInfo (CONST_NODENAME_CONFIGURATIONMANAGER + " is up and running.")
        rospy.spin()

# if __name__ == '__main__':  
#     TableConfigurationManager().main()