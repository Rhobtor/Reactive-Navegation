import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/mnt/c/Users/luism/Documents/Ubuntu/reactive_navegation/workspace_test/install/car'
