from config_file_manager import TableConfiguration

# Should be updated to load file from within the node of the package
FileName = './src/set_the_table/table_configuration_manager/common/data/TableConfiguration.yaml'

def GetTableItems():
    return tableConfiguration.GetTableItems()

def Refresh():
    tableConfiguration.LoadFile(FileName)


tableConfiguration = TableConfiguration()
Refresh()