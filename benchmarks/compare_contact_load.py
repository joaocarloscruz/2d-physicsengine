"""Archive deterministic paired observations; never turn physical errors into passes."""
import argparse
import json
from pathlib import Path


def read(path):
    if path.stat().st_size > 16 * 1024 * 1024:
        raise ValueError(f"Report exceeds archive byte budget: {path}")
    def nonfinite(value):
        raise ValueError(f"Nonfinite JSON token: {value}")
    return json.loads(path.read_text(encoding="utf-8-sig"), parse_constant=nonfinite)


def archive(directory, output):
    result = {
        "schemaVersion": 1,
        "classification": "observation",
        "baselineRevision": "e0fd45d",
        "experimentalOperatorRevision": "6e576f6",
        "platform": "Windows x86_64 / LLVM-MinGW / Clang 23.1.1 / Release",
        "timingExcluded": True,
        "productionDefaultsChanged": False,
        "thresholdsChanged": False,
        "matrices": {},
    }
    keys = ["fixture", "dt", "iterations", "warmStart"]
    for profile, count in (("quick", 120), ("full", 432)):
        baseline = read(directory / f"contact-{profile}-baseline.json")
        experiment = read(directory / f"contact-{profile}-experiment.json")
        if any(x["executionFailures"] != 0 or len(x["rows"]) != count
               for x in (baseline, experiment)):
            raise ValueError(f"Incomplete {profile} observation matrix")
        columns = keys + [k for k in baseline["rows"][0]
                          if k not in keys and k != "elapsedMilliseconds"]
        columns += [k for k in experiment["rows"][0] if k not in columns
                    and k != "elapsedMilliseconds"]
        for left, right in zip(baseline["rows"], experiment["rows"]):
            if any(left[k] != right[k] for k in keys):
                raise ValueError(f"Mismatched {profile} controls")
            if any(x["classification"] != "observation" for x in (left, right)):
                raise ValueError("Execution failure is not a physical observation")
        result["matrices"][profile] = {
            "controls": {k: v for k, v in baseline.items() if k != "rows"},
            "columns": columns,
            "baseline": [[r.get(k) for k in columns] for r in baseline["rows"]],
            "experiment": [[r.get(k) for k in columns] for r in experiment["rows"]],
        }
    diagnostic = read(directory / "load-observations.json")
    if len(diagnostic["rows"]) != 32 or diagnostic["productionDefaultsChanged"]:
        raise ValueError("Unexpected bounded contact-load diagnostic schema")
    result["integrationControls"] = {
        "solverIterations": 64, "velocityTolerance": 0, "warmStartFactor": 0,
        "positionCorrectionFactor": 0, "sleeping": False, "velocityCaps": False,
        "ccd": False, "supportDuration": 2, "supportGravity": [0, -9.81],
        "supportFriction": 1, "slideFriction": 0.03, "slideInitialSpeed": 1,
        "releaseLoad": "outward normal * 9.81 after one second",
        "stoppingFixture": "g=8, mu=.25, v=.2, centerY=.499, duration=.5",
    }
    result["integrationRows"] = diagnostic["rows"]
    output.write_text(json.dumps(result, separators=(",", ":"), allow_nan=False) + "\n",
                      encoding="utf-8")
    # Parse the final archive as well as all five source artifacts.
    read(output)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--directory", type=Path, default=Path("build"))
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    archive(args.directory, args.output)
