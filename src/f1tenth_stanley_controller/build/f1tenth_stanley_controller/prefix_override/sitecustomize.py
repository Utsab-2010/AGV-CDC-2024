import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/autodrive_devkit/src/f1tenth_stanley_controller/install/f1tenth_stanley_controller'
