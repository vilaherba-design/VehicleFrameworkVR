#!/usr/bin/env python3
"""Package a release zip: VehicleFrameworkVR-<version>-fase1.zip next to the repository.

Runs tools/check_package.py first and stops if it fails. The zip holds the files of the previous release (same paths), taken from data/mod
and the built DLL, plus any --add paths (relative to data/mod). Every file must come from this repository: a file the previous zip had but
data/mod has not stops the packaging instead of being copied over unchanged.

Usage: package_release.py <version> <previous zip> [--add <path in data/mod>]...
"""
import os
import subprocess
import sys
import zipfile

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA = os.path.join(REPO, "data", "mod")
INI = os.path.join(REPO, "data", "config", "VehicleFrameworkVR.ini")
DLL = os.path.join(REPO, "build", "Release", "VehicleFrameworkVR.dll")
DLL_IN_ZIP = "F4SE/Plugins/VehicleFrameworkVR.dll"


def main(argv):
    if len(argv) < 2:
        sys.exit(__doc__)
    version, previous = argv[0], argv[1]
    extra = [argv[i + 1].replace("\\", "/") for i, a in enumerate(argv) if a == "--add" and i + 1 < len(argv)]
    out = os.path.join(os.path.dirname(REPO), f"VehicleFrameworkVR-{version}-fase1.zip")
    if os.path.exists(out):
        sys.exit(f"{out} already exists")

    check = subprocess.run([sys.executable, os.path.join(REPO, "tools", "check_package.py"), DATA, INI, previous])
    if check.returncode != 0:
        sys.exit("check_package.py failed: nothing packaged")

    with zipfile.ZipFile(previous) as z:
        names = [i.filename for i in z.infolist() if not i.is_dir()]
    names += [n for n in extra if n not in names]
    sources = {}
    for name in names:
        src = DLL if name == DLL_IN_ZIP else os.path.join(DATA, *name.split("/"))
        if not os.path.isfile(src):
            sys.exit(f"{name}: {src} does not exist, nothing packaged")
        sources[name] = src

    with zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED) as z:
        for name, src in sources.items():
            z.write(src, name)
    print(f"wrote {out}: {len(sources)} files")


if __name__ == "__main__":
    main(sys.argv[1:])
