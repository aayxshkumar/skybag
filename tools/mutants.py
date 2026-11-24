"""Plant known bugs in skybag one at a time and report what catches each.

Copies the repo to a scratch directory, builds it once, then for every mutant
rewrites one file, rebuilds incrementally, and runs the unit tests and
`skybag --check`. A mutant nothing catches is SURVIVED: a hole in the tests.

    python3 tools/mutants.py [--only NAME] [--keep]
"""
import argparse
import os
import shutil
import subprocess
import sys
import tempfile
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# name, file, original text, replacement, what the bug means
MUTANTS = [
    ("wind-crab-sign", "src/skybag/wind.cpp",
     "std::asin(-cross / tas_kt)", "std::asin(cross / tas_kt)",
     "wind correction angle crabs downwind instead of into the wind"),
    ("qnh-sign", "src/skybag/units.cpp",
     "return elevation_m + h_qnh;", "return elevation_m - h_qnh;",
     "low QNH lowers the pressure altitude instead of raising it"),
    ("isa-lapse", "include/skybag/units.hpp",
     "kIsaLapse = 0.0065;", "kIsaLapse = 0.0060;",
     "wrong temperature lapse rate in the standard atmosphere"),
    ("direct-sphere-lat", "src/skybag/geo.cpp",
     "const double tan_u1 = (1 - f) * std::tan(rad(p.lat));", "const double tan_u1 = std::tan(rad(p.lat));",
     "direct geodesic uses geodetic instead of reduced latitude"),
    ("index-no-wrap", "src/skybag/navdb.cpp",
     "const int j = (((j_lo + dj) + 180) % 360 + 360) % 360;", "const int j = std::clamp(j_lo + dj + 180, 0, 359);",
     "grid index doesn't wrap at the antimeridian"),
    ("asd-no-reaction", "src/skybag/perf.cpp",
     "r.for_time(oei, ac.reaction_s);", "(void)oei;",
     "accelerate-stop skips the 2 s recognition and reaction time"),
    ("asd-wet-as-dry", "src/skybag/perf.cpp",
     "const Config stop{ac.engines - 1, true, 0.0, ac.cd0_landing, in.rw.wet ? ac.mu_brake_wet : ac.mu_brake_dry};",
     "const Config stop{ac.engines - 1, true, 0.0, ac.cd0_landing, ac.mu_brake_dry};",
     "a wet runway stops like a dry one in the rejected take-off"),
    ("v1-bisect-backwards", "src/skybag/perf.cpp",
     "(gap(mid) > 0 ? a : b) = mid;", "(gap(mid) > 0 ? b : a) = mid;",
     "V1 bisection moves the wrong bound"),
    ("reserve-20-min", "include/skybag/planner.hpp",
     "double final_reserve_min = 30;", "double final_reserve_min = 20;",
     "final reserve is 20 minutes instead of 30"),
    ("no-contingency", "src/skybag/planner.cpp",
     "const double next = trip_kg * (1 + in.contingency_pct / 100.0)", "const double next = trip_kg",
     "block fuel leaves out contingency"),
    ("notam-end-inclusive", "src/skybag/notam.cpp",
     "return n.from_min <= t && t < n.to_min;", "return n.from_min <= t && t <= n.to_min;",
     "a NOTAM is still active at its C) time"),
    ("envelope-edge-outside", "src/skybag/wb.cpp",
     "      return true;\n    // Crossing test", "      {}\n    // Crossing test",
     "a CG exactly on a certified limit line counts as outside (or inside) by luck"),
    ("terrain-no-blend", "src/skybag/terrain.cpp",
     "if (d <= kFlatInner) return field;", "if (d <= 0) return field;",
     "terrain is flattened only at the airport reference point, not within 4 km"),
    ("track-seed-ignored", "src/skybag/track.cpp",
     "Rng rng(seed);", "Rng rng(seed * 0 + 1);",
     "replay noise ignores the seed"),
    ("tailwind-unfactored", "src/skybag/api.cpp",
     "rw->headwind_kt = head >= 0 ? 0.5 * head : 1.5 * head;", "rw->headwind_kt = head >= 0 ? 0.5 * head : head;",
     "tailwind is not factored by 150% for take-off and landing"),
]


def run(cmd, cwd, timeout=900):
    try:
        p = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True, timeout=timeout)
        return p.returncode, p.stdout + p.stderr
    except subprocess.TimeoutExpired:
        return 124, "timeout"


def build(src, bld):
    return run(["cmake", "--build", bld, "-j", str(os.cpu_count() or 4), "--target", "skybag", "skybag_tests"], src)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", help="run one mutant by name")
    ap.add_argument("--keep", action="store_true", help="keep the scratch directory")
    args = ap.parse_args()

    todo = [m for m in MUTANTS if not args.only or m[0] == args.only]
    tmp = tempfile.mkdtemp(prefix="skybag-mutants-")
    src = os.path.join(tmp, "src")
    bld = os.path.join(tmp, "build")
    shutil.copytree(ROOT, src, ignore=shutil.ignore_patterns("build*", ".git", "__pycache__"))
    code, out = run(["cmake", "-S", src, "-B", bld, "-DCMAKE_BUILD_TYPE=Release"], src)
    if code:
        print(out)
        return 2
    code, out = build(src, bld)
    if code:
        print("baseline build failed\n" + out[-3000:])
        return 2
    data = os.path.join(src, "data")
    print(f"{'mutant':<24}{'tests':>8}{'checks':>10}   meaning")
    survived = []
    for name, rel, old, new, meaning in todo:
        path = os.path.join(src, rel)
        with open(path) as f:
            original = f.read()
        if original.count(old) != 1:
            print(f"{name:<24}   pattern not found exactly once in {rel}")
            survived.append(name)
            continue
        with open(path, "w") as f:
            f.write(original.replace(old, new))
        os.utime(path, (time.time() + 2, time.time() + 2))  # never reuse a stale object file
        code, out = build(src, bld)
        if code:
            tests, checks = "nobuild", "-"
            caught_by = ["compile error"]
        else:
            tcode, tout = run([os.path.join(bld, "skybag_tests"), "--gtest_brief=1"], src)
            ccode, cout = run([os.path.join(bld, "skybag"), "--check", "--data", data], src)
            failed_tests = [l.split("]", 1)[1].strip().split(" ")[0] for l in tout.splitlines() if l.startswith("[  FAILED  ]") and "," not in l and "listed" not in l]
            failed_checks = [l.split()[0] for l in cout.splitlines() if " FAIL " in l]
            tests = f"{len(failed_tests)} fail" if tcode else "pass"
            checks = f"{len(failed_checks)} fail" if ccode else "pass"
            caught_by = [f"tests: {', '.join(sorted(set(failed_tests)))}"] if failed_tests else []
            if failed_checks:
                caught_by.append(f"checks: {', '.join(failed_checks)}")
        status = "" if caught_by else "SURVIVED  "
        print(f"{name:<24}{tests:>8}{checks:>10}   {status}{meaning}")
        for c in caught_by:
            print(f"{'':<29}{c}")
        if not caught_by:
            survived.append(name)
        with open(path, "w") as f:
            f.write(original)
        os.utime(path, (time.time() + 4, time.time() + 4))
    if not args.keep:
        shutil.rmtree(tmp, ignore_errors=True)
    print(f"\n{len(todo) - len(survived)}/{len(todo)} caught" + (f"; survived: {', '.join(survived)}" if survived else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
