import numpy as np
# state_dim = 3

# P = np.eye(state_dim)

# print(P)


Q = np.diag([5.0, 5.0, np.deg2rad(10.0)])**2
print(Q)