import transforms3d as tfs
import numpy as np

if __name__ == '__main__':
    q1=np.array((0.35, 0.2, 0.3, 0.1))
    q2=np.array((-0.5, 0.4, -0.1, 0.2))
    t1=np.array((0.3, 0.1, 0.1))
    t2=np.array((-0.1, 0.5, 0.3))
    p1=np.array((0.5, 0, 0.2))

    mat = tfs.affines.compose(
        t1, tfs.quaternions.quat2mat(q1),
        [1, 1, 1])

