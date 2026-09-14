"""Read-only comparison of the checked-in M08 baseline and M09 BNO paths."""
import hashlib
import json
from pathlib import Path
import re
import subprocess

root = Path(__file__).resolve().parents[1]
m08 = root.parent / "M08_go_to_pose"


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def function(source, name):
    match = re.search(r"^(?:bool|void) " + name + r"\([^\n]*\) \{.*?^\}", source, re.M | re.S)
    if not match:
        raise ValueError(name)
    return match.group(0)


def main():
    sources = [(p / "src/main.cpp").read_text() for p in (m08, root)]
    result = {
        "hardware_test_performed_by_this_script": False,
        "git_head": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=root, text=True).strip(),
        "m08_vs_git": subprocess.check_output(["git", "diff", "HEAD", "--", "diagnostics/M08_go_to_pose"], cwd=root.parents[1], text=True),
        "identical_files": {}, "identical_functions": {}, "dependency_comparison": {}, "images": {},
    }
    for relative in ("platformio.ini", "src/sensor_support.h", "src/control_math.h", "src/motion_watchdog.h", "src/pose_math.h"):
        a, b = sha(m08 / relative), sha(root / relative)
        result["identical_files"][relative] = {"same": a == b, "m08_sha256": a, "m09_sha256": b}
    for name in ("enableReport", "handleReset", "serviceBno", "stableBaseline", "stablePitchBaseline"):
        result["identical_functions"][name] = function(sources[0], name) == function(sources[1], name)
    for package in ("Adafruit BNO08x", "Adafruit BusIO", "Adafruit Unified Sensor", "FastAccelStepper"):
        directories = [p / ".pio/libdeps/esp32dev" / package for p in (m08, root)]
        manifests = [{str(p.relative_to(directory)): sha(p) for p in directory.rglob("*") if p.is_file()} for directory in directories]
        differing = [key for key in sorted(set(manifests[0]) | set(manifests[1])) if manifests[0].get(key) != manifests[1].get(key)]
        result["dependency_comparison"][package] = {"m08_files": len(manifests[0]), "m09_files": len(manifests[1]), "differing_files": differing}
    for project in (m08, root):
        result["images"][project.name] = {name: {"sha256": sha(project / ".pio/build/esp32dev" / name),
            "bytes": (project / ".pio/build/esp32dev" / name).stat().st_size}
            for name in ("firmware.bin", "bootloader.bin", "partitions.bin")}
    commands = ("sh2_setCalConfig", "sh2_getCalConfig", "sh2_saveDcdNow", "sh2_setDcdAutoSave", "sh2_clearDcdAndReset")
    result["explicit_calibration_dcd_calls"] = {project.name: {name: [str(file.relative_to(project)) for file in (project / "src").rglob("*")
        if file.is_file() and re.search(r"\b" + name + r"\s*\(", file.read_text())] for name in commands} for project in (m08, root)}
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
