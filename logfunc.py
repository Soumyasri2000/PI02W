import logging
import os
from logging.handlers import TimedRotatingFileHandler

#import logs from  the programme
def LoggingFile():
    logging.basicConfig(
        filename= '/Development/systemlog.log',
        level=logging.INFO,
        format='%(asctime)s %(levelname)s : %(message)s'
    )



#logs will be stored for the 7days as backup and automatically it will clear
def LogFile():
    log_dir = '/Development'
    os.makedirs(log_dir, exist_ok=True) #Mentioned directory should be exsits
    log_file = os.path.join(log_dir, 'systemlog.log')
    logger = logging.getlogger("systemLogger")
    if not logger.handlers:
       handler = TimedRotatingFileHandler(
           filename = log_file,
           when ='midnight',
           interval = 1,
           backupCount= 7,
           encoding = 'utf-8',
           utc = True
       )
       formatter=logging.Formatter('%(asctime)s %(levelname)s : %(message)s')
       handler.setFormatter(formatter)
       logger.addHandler(formatter)
    return logger

