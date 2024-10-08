import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jimmy/work/docker/DynamixelSDK/control_motor/install/control_motor'
