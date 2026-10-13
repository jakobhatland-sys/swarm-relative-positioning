import sys; sys.path.insert(0, ".")
from experiments.anchor_sweep import run_trial

print(f"{'init σ':>7} {'kjent v0':>9} {'abs RMS':>9} {'rel RMS':>9} {'ANEES':>9}")
for init_sigma in (0.0, 0.3, 1.0, 2.0):
    for known_v0 in (True, False):
        a, r, nees = run_trial(16, 8, 0, init_sigma=init_sigma, known_v0=known_v0)
        print(f"{init_sigma:>7} {str(known_v0):>9} {a:>9.3f} {r:>9.3f} {nees:>9.2f}", flush=True)