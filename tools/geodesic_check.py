"""Compare skybag's geodesics with GeographicLib (Karney's algorithm).

GeographicLib is vendored as a wheel in third_party/python, so this runs
offline. It is used only as a checker, never by skybag itself.

    python3 tools/geodesic_check.py [--n 20000] [--seed 1] [--binary build/skybag]
"""
import argparse
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WHEELS = os.path.join(ROOT, "third_party", "python")
for name in sorted(os.listdir(WHEELS)):
    if name.endswith(".whl"):
        sys.path.insert(0, os.path.join(WHEELS, name))

from geographiclib.geodesic import Geodesic  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=20000)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--binary", default=os.path.join(ROOT, "build", "skybag"))
    args = ap.parse_args()

    out = subprocess.run([args.binary, "geodesic-dump", "--n", str(args.n), "--seed", str(args.seed)],
                         check=True, capture_output=True, text=True).stdout
    worst_d = worst_az = 0.0
    worst_line = None
    failed = 0
    rows = 0
    for line in out.splitlines():
        lat1, lon1, lat2, lon2, dist, az1, ok = line.split()
        rows += 1
        if ok != "1":
            failed += 1
            continue
        g = Geodesic.WGS84.Inverse(float(lat1), float(lon1), float(lat2), float(lon2))
        dd = abs(float(dist) - g["s12"])
        da = abs((float(az1) - g["azi1"] + 180) % 360 - 180)
        if dd > worst_d:
            worst_d, worst_line = dd, line
        worst_az = max(worst_az, da)
    print(f"{rows} random pairs, {failed} where skybag's inverse did not converge")
    print(f"max distance error vs GeographicLib: {worst_d * 1000:.4f} mm")
    print(f"max initial-azimuth error:          {worst_az * 3600:.6f} arcsec")
    if worst_line:
        print(f"worst pair: {worst_line}")
    return 1 if worst_d > 1e-3 else 0


if __name__ == "__main__":
    sys.exit(main())
