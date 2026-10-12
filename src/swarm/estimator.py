import numpy as np

class SwarmEKF:
    """Sentralisert EKF. Tilstand per drone: [p(3), v(3)] i ENU."""
    def __init__(self, p0, sigma_p0=2.0, sigma_v0=1.0, q=0.3):
        self.n, self.q = len(p0), q
        X = np.zeros((self.n, 6)); X[:, :3] = p0
        self.x = X.ravel()
        self.P = np.diag(np.tile([sigma_p0**2]*3 + [sigma_v0**2]*3, self.n))

    @property
    def pos(self):
        return self.x.reshape(self.n, 6)[:, :3]

    def predict(self, dt):
        I, Z = np.eye(3), np.zeros((3, 3))
        F1 = np.block([[I, dt*I], [Z, I]])
        Q1 = self.q**2 * np.block([[dt**3/3*I, dt**2/2*I], [dt**2/2*I, dt*I]])
        F = np.kron(np.eye(self.n), F1)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + np.kron(np.eye(self.n), Q1)

    def range_model(self, pairs):
        """Predikert avstand og Jacobian H for nabopar."""
        i, j = np.array(pairs).T
        d = self.pos[j] - self.pos[i]
        rho = np.linalg.norm(d, axis=1)
        u = d / rho[:, None]                        # enhetsvektor = ∂ρ/∂p_j
        H = np.zeros((len(pairs), 6 * self.n)); rows = np.arange(len(pairs))
        for a in range(3):
            H[rows, 6*i + a], H[rows, 6*j + a] = -u[:, a], u[:, a]
        return rho, H

    def update_ranges(self, pairs, r, sigma_r):
        rho, H = self.range_model(pairs)
        return self._update(r, rho, H, sigma_r**2 * np.eye(len(r)))

    def update_anchors(self, ids, p_meas, sigma_a):
        H = np.zeros((3 * len(ids), 6 * self.n))
        for k, a in enumerate(ids):
            H[3*k:3*k+3, 6*a:6*a+3] = np.eye(3)
        return self._update(p_meas.ravel(), H @ self.x, H, sigma_a**2 * np.eye(3 * len(ids)))

    def _update(self, z, h, H, R):
        S = H @ self.P @ H.T + R                    # innovasjonskovarians
        K = np.linalg.solve(S, H @ self.P).T        # Kalman-gain
        nu = z - h                                  # innovasjon
        self.x = self.x + K @ nu
        IKH = np.eye(len(self.x)) - K @ H
        self.P = IKH @ self.P @ IKH.T + K @ R @ K.T # Joseph-form
        return nu, S