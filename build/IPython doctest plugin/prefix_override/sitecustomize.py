import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/maya/Documents/projects/agri-grasp-ros2/install/IPython doctest plugin'
