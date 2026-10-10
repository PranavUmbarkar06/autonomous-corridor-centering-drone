import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/pranav-umbarkar/corridor_ws/install/corridor_drone_pkg'
