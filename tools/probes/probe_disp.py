import json
import time
import sys

from nfocc_ph1.pipeline import run

cases = [(960, 128), (960, 192), (960, 256), (1920, 128)]
for w, nd in cases:
    t0 = time.time()
    r = run("S_BOX", "FV", view_w=w, max_disp=nd, downscale=1)
    m = r.metrics
    print(f"view={m['view_size']} max_disp={nd}: "
          f"mederr={m['disp_median_abs_err_px']:.2f}px "
          f"p90={m['disp_p90_abs_err_px']:.1f} "
          f"valid={m['valid_ratio']:.2f} npts={m['n_points']} "
          f"[{time.time()-t0:.1f}s]", flush=True)
