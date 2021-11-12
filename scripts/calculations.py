import numpy as np

translational_vector_desired = np.array([[ 0.01737492], [-0.00360343], [ 0.1355011 ]])
homogenious_matrix_desired = np.array([
            [ 0.01952927,  0.93951869, -0.34194034,  0.01737492],
            [ 0.99972844, -0.01400038,  0.01862998, -0.00360343],
            [ 0.01271592, -0.34221131, -0.93953699,  0.1355011 ],
            [ 0.,          0.,          0.,          1.        ]])

translational_vector_current = np.array( [[-0.00725483], [-0.00439238], [ 0.13544616]])
homogenious_matrix_current = np.array( [
            [-0.02282412,  0.99750239, -0.06684382, -0.00725483],
            [ 0.99367386,  0.01528088, -0.11125974, -0.00439238],
            [-0.10996041, -0.06896035, -0.99154079,  0.13544616],
            [ 0.        ,  0.        ,  0.        ,  1.        ]])


out = np.matmul(homogenious_matrix_current, np.linalg.inv(homogenious_matrix_desired))

print(out)