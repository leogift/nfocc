import json
import numpy as np
from nfocc_ph1.product import run_product

cases = [
    ("直行 120mm", np.array([120.0, 0, 0])),
    ("侧移 120mm", np.array([0.0, 120.0, 0])),
    ("45deg 斜向", np.array([84.85, 84.85, 0])),
    ("微小 8mm", np.array([8.0, 0, 0])),
    ("过大 300mm", np.array([300.0, 0, 0])),
]
for name, t in cases:
    r = run_product("S_MULTI", "FV", t_veh=t)
    m = r.metrics
    pts = "n/a" if r.points is None else str(r.points.n_total)
    print(f"{name:>12}: mode={r.mode.value if r.mode else '-':<12} "
          f"B={m.get('baseline_mm',0):>6.1f} sigma={m.get('sigma_lateral',0):.2f} "
          f"pts={pts:>7} dist_med={m.get('dist_median_mm',float('nan')):>7.0f} "
          f"sig_med={m.get('sigma_median_mm',float('nan')):>6.0f}")
