import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rhobtor/reactive/Reactive-Navegation/workspace_full_conected/install/car'
