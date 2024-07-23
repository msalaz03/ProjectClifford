import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/mcc/Desktop/project_clifford_ws/ProjectClifford/install/servo_driver'
