import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/mnt/c/Users/luism/Documents/Ubuntu/Reactive-Navegation/Reactive-Navegation/workspace/install/car'
