import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/erthirarat/Desktop/Senior_Project-WhitePost-Main_Arm/install/encoder_unit'
