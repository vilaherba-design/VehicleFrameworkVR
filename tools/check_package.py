#!/usr/bin/env python3
"""Check the mod data before packaging a release.

Fails (exit code 1) when a vehicle profile the INI ships (sVehicleProfiles):
  - has a non-empty "carrierForm" (the vanilla model on a carrier: no wheel nodes, so no rolling or steering wheels and no visible suspension),
  - takes its "baseForm" from Fallout4.esm although VehicleFrameworkVR_Vehicles.esp has the vehicle's own model,
  - names a base form of VehicleFrameworkVR_Vehicles.esp that the plugin does not contain.

Then it lists how data/mod differs from the last published zip (text files compared without line endings), so every change in a release is
one somebody meant.

Usage: check_package.py <data/mod folder> <data/config/VehicleFrameworkVR.ini> <last published zip>
"""
import hashlib
import os
import re
import struct
import sys
import zipfile

PLUGIN = "VehicleFrameworkVR_Vehicles.esp"
TEXT_EXTENSIONS = (".json", ".psc", ".ini", ".txt", ".bgsm", ".bgem")


def strip_comments(text):
    """JSON with // and /* */ comments (what the plugin reads) -> plain JSON, leaving strings alone."""
    out, i, n, in_string = [], 0, len(text), False
    while i < n:
        c = text[i]
        if in_string:
            out.append(c)
            if c == "\\" and i + 1 < n:
                out.append(text[i + 1])
                i += 1
            elif c == '"':
                in_string = False
        elif c == '"':
            in_string = True
            out.append(c)
        elif text.startswith("//", i):
            while i < n and text[i] != "\n":
                i += 1
            continue
        elif text.startswith("/*", i):
            end = text.find("*/", i + 2)
            i = n if end < 0 else end + 2
            continue
        else:
            out.append(c)
        i += 1
    return "".join(out)


def plugin_form_ids(path):
    """Local FormIDs (without the load order byte) of the records in a plugin."""
    ids = set()
    data = open(path, "rb").read()
    i = 0
    while i < len(data):
        tag, size = data[i:i + 4], struct.unpack("<I", data[i + 4:i + 8])[0]
        if tag == b"GRUP":
            j, end = i + 24, i + size
            while j < end:
                rsize, _, form_id = struct.unpack("<III", data[j + 4:j + 16])
                ids.add(form_id & 0x00FFFFFF)
                j += 24 + rsize
            i = end
        else:
            i += 24 + size
    return ids


def shipped_profiles(ini_path):
    for line in open(ini_path, encoding="utf-8", errors="replace"):
        m = re.match(r"\s*sVehicleProfiles\s*=\s*(.*)", line)
        if m:
            return [p.strip() for p in m.group(1).split(",") if p.strip()]
    return []


def check_profiles(data_dir, ini_path):
    import json

    errors = []
    esp = os.path.join(data_dir, PLUGIN)
    esp_ids = plugin_form_ids(esp) if os.path.isfile(esp) else set()
    for profile in shipped_profiles(ini_path):
        path = os.path.join(data_dir, "F4SE", "Plugins", "VehicleFrameworkVR", "Vehicles", profile + ".json")
        if not os.path.isfile(path):
            errors.append(f"{profile}: {path} is missing")
            continue
        try:
            spec = json.loads(strip_comments(open(path, encoding="utf-8").read()))
        except ValueError as e:
            errors.append(f"{profile}: not valid JSON ({e})")
            continue
        base = spec.get("baseForm", "")
        carrier = spec.get("carrierForm", "")
        if carrier:
            errors.append(f'{profile}: "carrierForm" is "{carrier}" (must be empty: the model on a carrier has no wheel nodes)')
        if base.lower().startswith("fallout4.esm") and esp_ids:
            errors.append(f'{profile}: "baseForm" is "{base}" from Fallout4.esm; use the vehicle\'s own model in {PLUGIN}')
        if base.lower().startswith(PLUGIN.lower()):
            form = int(base.split("|", 1)[1], 16) & 0x00FFFFFF
            if form not in esp_ids:
                errors.append(f'{profile}: "baseForm" {base} is not in {PLUGIN}')
        print(f"  {profile:14s} baseForm {base or '-':44s} carrierForm {carrier or '-'}")
    return errors


def digest(data, name):
    if name.lower().endswith(TEXT_EXTENSIONS):
        data = data.replace(b"\r\n", b"\n")
    return hashlib.sha1(data).hexdigest()


def diff_against_zip(data_dir, zip_path):
    changed, missing = [], []
    with zipfile.ZipFile(zip_path) as z:
        for info in z.infolist():
            name = info.filename
            if info.is_dir() or name.lower().endswith(".dll"):
                continue
            path = os.path.join(data_dir, *name.split("/"))
            if not os.path.isfile(path):
                missing.append(name)
            elif digest(open(path, "rb").read(), name) != digest(z.read(name), name):
                changed.append(name)
    return changed, missing


def main(data_dir, ini_path, zip_path):
    print("Vehicle profiles shipped (sVehicleProfiles):")
    errors = check_profiles(data_dir, ini_path)
    changed, missing = diff_against_zip(data_dir, zip_path)
    print(f"\nAgainst {os.path.basename(zip_path)}:")
    for name in changed:
        print("  changed:", name)
    for name in missing:
        print("  NOT in data/mod (would be lost):", name)
    if not changed and not missing:
        print("  identical (DLL not compared)")
    errors += [f"{name} is in the published zip but not in data/mod" for name in missing]
    if errors:
        print("\nFAILED:")
        for e in errors:
            print("  -", e)
        return 1
    print("\nOK")
    return 0


if __name__ == "__main__":
    if len(sys.argv) != 4:
        sys.exit(__doc__)
    sys.exit(main(*sys.argv[1:]))
