import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/alex/workspace/ros2/totodile/install/tutlebot3_slamNav'
