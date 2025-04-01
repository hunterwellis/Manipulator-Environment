import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/hunter/dev/thesis/env/src/arm_rl/gym_arm/install/gym_arm'
