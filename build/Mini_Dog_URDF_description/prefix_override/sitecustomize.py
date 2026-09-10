import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/sayak/Desktop/Mini Robot Dog/DAZZY/install/Mini_Dog_URDF_description'
