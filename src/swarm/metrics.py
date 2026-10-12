import numpy as np

def rms(e):
    return np.sqrt(np.mean(np.sum(e**2, axis=1)))

def relative_rms(p_est, p_true):
    """RMS etter beste rigide tilpasning (Kabsch). Måler kun relativ geometri."""
    A, B = p_est - p_est.mean(0), p_true - p_true.mean(0)
    U, _, Vt = np.linalg.svd(A.T @ B)
    D = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))])   # unngå speiling
    return rms(A @ (U @ D @ Vt) - B)

def nees_position(ekf, p_true):
    """Normalisert NEES for posisjonstilstandene (≈1 hvis konsistent)."""
    idx = np.ravel([[6*i, 6*i+1, 6*i+2] for i in range(ekf.n)])
    e = (ekf.pos - p_true).ravel()
    return e @ np.linalg.solve(ekf.P[np.ix_(idx, idx)], e) / len(e)
    