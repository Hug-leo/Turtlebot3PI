import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/dinhsieu/turtlebot3_pi_ws/install/agv_controller'
