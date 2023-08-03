import numpy as np
from scipy.optimize import linprog

def in_hull(points, x):
    n_points = len(points)
    n_dim = len(x)
    c = np.zeros(n_points)
    A = np.r_[points.T,np.ones((1,n_points))]
    b = np.r_[x, np.ones(1)]
    lp = linprog(c, A_eq=A, b_eq=b)
    return lp.success

n_points = 10
n_dim = 3
Z = np.array([[1, 1, 0], [-1, 1, 0], [-1, -1, 0], [1, -1, 0]])
x = np.array([1, 0.5, 0.0])
print(in_hull(Z, x))