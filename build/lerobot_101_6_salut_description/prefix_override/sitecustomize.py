import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/bogda/Documents/Projects/arm_wizard/install/lerobot_101_6_salut_description'
