#!/usr/bin/env python

# Refrence: https://stackoverflow.com/a/42054860

from set_the_table.msg import TableItem, TableItems
import yaml
import io

from Logger_ import logger

CONST_STRING_NUMBEROFITEMS = 'Number of Items'
CONST_STRING_TABLEITEM = 'Table Item'
CONST_STRING_HEIGHT = 'Height'
CONST_STRING_BREADTH = 'Breadth'
CONST_STRING_WIDTH = 'Width'


class TableConfiguration:

    def LoadFile(self, filename):
        with open(filename, 'r') as stream:
            self.data_loaded = yaml.load(stream)
        logger.LogInfo('File Opened successfully.')
        self.__set_table_items__()


    def SaveFile(self, filename):
        if(not self.IsValidDataAvailable()):
            logger.LogError('Invalid data provided')
            raise Exception('Invalid data provided')

        with io.open(filename, 'w') as outfile:
            yaml.dump(self.data_loaded, outfile, default_flow_style=False, allow_unicode=True)

    def IsValidDataAvailable(self):
        return self.data_loaded != None

    def SetTableItems(self, table_items):
        self.table_items = table_items

        self.__set_data_loaded__()

    def GetTableItems(self):
        if(not self.IsValidDataAvailable()):
            logger.LogError('Data not loaded')
            raise Exception('Data not loaded')
            
        return self.table_items



    def __set_table_items__(self):
        self.table_items = TableItems()
        self.table_items.numberOfItems = self.data_loaded[CONST_STRING_NUMBEROFITEMS]

        for item_id_dimension in self.data_loaded[CONST_STRING_TABLEITEM]:
            
            item_id, dimension = item_id_dimension.items()[0]

            tableItem = TableItem()
            tableItem.item_id = item_id
            tableItem.height = dimension[CONST_STRING_HEIGHT]
            tableItem.width = dimension[CONST_STRING_WIDTH]
            tableItem.breadth = dimension[CONST_STRING_BREADTH]

            self.table_items.tableItem.append(tableItem)

    def __set_data_loaded__(self):
        self.data_loaded = {}
        self.data_loaded[CONST_STRING_NUMBEROFITEMS] = self.table_items.numberOfItems

        list_of_tableItems = []

        for tableItem in self.table_items.tableItem:
            dimension_dict = {}
            dimension_dict[CONST_STRING_HEIGHT] = tableItem.height
            dimension_dict[CONST_STRING_WIDTH] = tableItem.width
            dimension_dict[CONST_STRING_BREADTH] = tableItem.breadth

            tableItem_pair = {tableItem.item_id : dimension_dict}
            list_of_tableItems.append(tableItem_pair)


        self.data_loaded[CONST_STRING_TABLEITEM] = list_of_tableItems


## Save a file

# tabConfig = TableConfiguration()
# tabItems = TableItems()
# tabItems.numberOfItems = 2

# tabItem1 = TableItem()
# tabItem1.item_id = 'Plate'
# tabItem1.height = 2
# tabItem1.width = 3
# tabItem1.breadth = 4

# tabItems.tableItem.append(tabItem1)

# tabItem1 = TableItem()
# tabItem1.item_id = 'Spoon'
# tabItem1.height = 5
# tabItem1.width = 5
# tabItem1.breadth = 6

# tabItems.tableItem.append(tabItem1)

# tabConfig.SetTableItems(tabItems)
# tabConfig.SaveFile('./sample.yaml')

## Load a file

# tabConfig = TableConfiguration()
# tabConfig.LoadFile('./sample.yaml')
# tb = tabConfig.GetTableItems()
# print(tb)

