import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/autodrive_devkit/src/install/f1tenth_stanley_controller'
