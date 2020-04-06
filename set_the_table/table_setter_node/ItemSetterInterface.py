import abc


class ItemSetterInterface(object):
    @abc.abstractmethod
    def __init(self):
        pass

    def set_number_of_sets(self, number_of_sets):
        self.number_of_sets = number_of_sets
    
    def set_table_items(self, table_items):
        self.table_items = table_items

    def set_offset(self, offset):
        self.offset = offset

    def set_item_order(self,item_order):
        self.item_order = item_order

    def set_item_properties(self, item_properties):
        self.item_properties = item_properties

    @abc.abstractmethod
    def PerformSetup(self, poses):
        pass