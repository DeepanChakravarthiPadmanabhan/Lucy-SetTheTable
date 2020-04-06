import table_logger.ros.scripts.Logger

logger = None

def InitLogger(nodeName):
    global logger
    if not (logger is None):
        return logger
    logger = table_logger.ros.scripts.Logger.InitLogger(nodeName)
    logger.SetLocalLogger(True, nodeName + "Log.txt")
    logger.SetPublishLogger(True)
    return logger