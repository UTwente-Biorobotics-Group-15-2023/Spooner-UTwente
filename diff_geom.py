#diff_geom is a module that provides differential geometry functions
# i.e. functions to work with H-matrices, twists, and wrenches


# We need to do some tricks to get the right numpy
# option A: we're on a PC, so get the standard numpy
# option B: we're on old micropython, so get ulab
# option C: we have the new ulab, which includes numpy _and_ scipy

from ulab import numpy as np

def tilde(T):
    # return Tilde form of a vector-twist
    Tt = np.zeros((3,3))
    Tt[0,1] = -T[0]
    Tt[1,0] = T[0]
    Tt[:2,2] = T[1:]
    
    return Tt

def Adjoint(H):
    AdH = np.zeros((3,3))
    AdH[0,0] = 1
    AdH[1:,1:] = H[:2,:2]
    AdH[1,0] = H[1,2]
    AdH[2,0] = -H[0,2]
    
    return AdH

def inverseH(H):
    Hinv = np.eye(3)
    Hinv[:2,:2] = H[:2,:2].transpose()
    Hinv[:2,2:] = -Hinv[:2,:2].dot(H[:2,2])
    return Hinv

def expT(Tq):
    # calculate twist exponential for a given twist T * dq
    # (twist should be in tilde form, so 3x3 matrix)
    if not isinstance(Tq, np.ndarray):
        Tq = np.array(Tq)
    if Tq.shape != (3,3):
        raise Exception("Shape of twist should be 3x3 (tilde form of T)")
    
    H = np.eye(3)
    
    w = Tq[1,0]
    vx,vy = Tq[:2,2]
    if abs(w) < (abs(vx)+abs(vy)) * 0.01 or abs(w) < 1e-5:
        # w is smaller than 1% of v: pure translation
        H[0,2] = vx
        H[1,2] = vy
    else:
        # closed-form solution of twist with rotation in it
        rx = -vy / w
        ry = vx / w
        c = np.cos(w)
        s = np.sin(w)
        H[0,0] = c
        H[0,1] = -s
        H[1,0] = s
        H[1,1] = c
        H[0,2] = rx - rx*c + ry*s
        H[1,2] = ry - rx*s - ry*c
    
    return H
    
if __name__ == "__main__":
    # try (verify) expT -- only on PC (full numpy with isclose, and scipy.linalg for ground truth/expm)
    try:
        import numpy as np
        from scipy.linalg import expm
        for T in ([1,1,-1.5],[1,0,0],[0,1,2]):
            for theta in [-np.pi/2, 0, np.pi, 1]:
                Tq = tilde(T) * theta
                assert np.allclose(expT(Tq), expm(Tq)), "Wrong result for" +str(T)+" * "+str(theta)[:8]
    except ImportError:
        print("Verifications only done on PC.")