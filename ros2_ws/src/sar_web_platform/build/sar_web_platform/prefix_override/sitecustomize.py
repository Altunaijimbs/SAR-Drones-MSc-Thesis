import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/src/sar_web_platform/install/sar_web_platform'
