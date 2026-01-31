import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/mnt/hgfs/sharedprj/test_export/lerobot_101_6_salut/install/lerobot_101_6_salut_description'
