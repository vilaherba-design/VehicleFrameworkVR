#!/usr/bin/env python3
"""Writes VehicleFrameworkVR_Vehicles.esp: one movable static (MSTT) per drivable vehicle model, the trunk container (CONT), and the
driver's cover (a global and a perk, see src/health/OccupantShield.cpp).

Why a plugin at all: the vanilla pre-war sedan (STAT 0x000A9774) has static collision that Havok cannot move, and the
rusty one (MSTT 0x00194634) is a wreck with open doors. This plugin's MSTT uses the mod's own nif (the clean sedan
geometry with the dynamic collision blocks of the rusty one), and none of the vanilla record's destruction data or
keywords ('ExplodableCar'), so nothing a player throws at it makes it explode.

The trunk container has no model: the plugin creates one persistent reference of it per vehicle profile, out of sight, and opens it from
there (see src/vehicle/Trunk.cpp).

The driver's cover: while the player sits in a vehicle whose cabin covers the driver, the plugin sets the global VFVR_DriverCovered to 1
and the perk VFVR_DriverCover (given to the player once, hidden) cuts, under the condition GetGlobalValue(VFVR_DriverCovered) == 1,
the incoming weapon and explosion damage to a tenth, and takes away the limb (crippling) damage, the falling damage, and the magnitude of
incoming poison, acid, fire, cryo, electrical and energy effects (a creature's poison, a flamer's burn): the hits still land and raise the
engine's hit events, which the plugin turns into vehicle damage, and the damage over time of those hits does not reach the driver. A tenth
and not nothing: the combat AI weighs the damage its weapons would do to the driver, and with the multiplier at 0 (0.1.84-0.1.105) no enemy
ever fired at the seated player (they fled or hid; a turret stayed silent) while everybody fired seconds after the player got out. Radiation is not in the
perk: the cabin's shielding lends radiation resistance instead (src/health/RadProtection.cpp). The record layouts follow xEdit's Fallout 4
definitions and the game's own perks: entry point 36 'Mod Incoming Weapon Damage' with three condition tabs and 124 'Mod Incoming
Explosion Damage' with one (the DLCs' incoming-damage perks), 4 'Mod Incoming Limb Damage' with three (CompanionPreventLimbDamage), 58
'Mod Falling Damage' with one (crNoFallDamage), 41 'Mod Incoming Spell Magnitude' with two, 'Set Value' 0 and the effect's keyword on
tab 1 (ImmuneToPoison, ImmuneToAcid; HC_SustenanceEffectsTurnOffFood has a global on tab 0 as well). Tab 0 is the perk owner; condition
function 74 is GetGlobalValue, 693 HasMagicEffectKeyword.

The plugin is a regular (non light) ESP with Fallout4.esm as its only master; FO4VR has no ESL support.
Usage: make_vehicle_esp.py <output.esp>
       make_vehicle_esp.py --perks <plugin.esp>   replace only the PERK group of an existing plugin (keeps the records other tools added)
"""
import struct
import sys

PLUGIN_AUTHOR = "Vehicle Framework VR"
DESCRIPTION = "Vehicle Framework VR: drivable vehicle base objects."
FORM_VERSION = 131

# (local form id, editor id, display name, model path relative to Meshes\, OBND min xyz, OBND max xyz) in game units
VEHICLES = [
    (0x800, "VFVR_Sedan01", "Sedan", "VehicleFrameworkVR\\Vehicles\\Sedan01.nif", (-233, -98, 0), (318, 98, 126)),
    (0x803, "VFVR_Rockboy", "Rockboy", "VehicleFrameworkVR\\Vehicles\\Rockboy.nif", (-82, -182, 24), (82, 186, 208)),
    (0x801, "VFVR_PickUpTruck01", "Pick-R-Up truck", "VehicleFrameworkVR\\Vehicles\\PickUpTruck01.nif", (-214, -104, 0), (212, 104, 166)),
    (0x804, "VFVR_Rusianmother", "Rusianmother", "VehicleFrameworkVR\\Vehicles\\Rusianmother.nif", (-33, -78, 0), (33, 78, 81)),
]
# Effect objects used by the plugin (not vehicles): the exhaust smoke puff.
EFFECTS = [
    (0x802, "VFVR_ExhaustPuff", "Exhaust puff", "VehicleFrameworkVR\\Effects\\ExhaustPuff.nif", (-4, -4, -4), (4, 4, 4)),
]
# Containers: (local form id, editor id, display name). No model, no items, does not respawn.
CONTAINERS = [
    (0x805, "VFVR_Trunk", "Trunk"),
]
# Globals: (local form id, editor id, initial value). Not constant: the plugin writes the value in memory while the game runs.
GLOBALS = [
    (0x806, "VFVR_DriverCovered", 0.0),
]
SET_VALUE = 1  # perk entry functions (EPFT 1: one float)
MULTIPLY_VALUE = 3

# Fallout4.esm keywords of the effects the cover takes away (master 0: the form id as it is).
DAMAGE_TYPE_POISON = 0x0004B25B
DAMAGE_TYPE_ACID = 0x001B9A3F
DAMAGE_TYPE_FIRE = 0x0004A0C8
DAMAGE_TYPE_CRYO = 0x0004A0C9  # 'DamageTypeCyro' in the game
DAMAGE_TYPE_ELECTRICAL = 0x0004A0CA  # 'DamageTypeElectricty' in the game
DAMAGE_TYPE_ENERGY = 0x0004A0CB
POISON_GAS_EFFECT = 0x000449B9  # DN102_PoisonGasEffectKeyword (the gas traps)

# Perks: (local form id, editor id, display name, global whose value 1 enables every entry, entries). An entry is (entry point, condition
# tab count, function, value, effect keyword on tab 1 or None).
PERKS = [
    (0x807, "VFVR_DriverCover", "Vehicle cover", 0x806, [
        (36, 3, MULTIPLY_VALUE, 0.1, None),  # Mod Incoming Weapon Damage (not 0: see above)
        (124, 1, MULTIPLY_VALUE, 0.1, None),  # Mod Incoming Explosion Damage (not 0: see above)
        (4, 3, MULTIPLY_VALUE, 0.0, None),  # Mod Incoming Limb Damage
        (58, 1, MULTIPLY_VALUE, 0.0, None),  # Mod Falling Damage
    ] + [(41, 2, SET_VALUE, 0.0, keyword) for keyword in (  # Mod Incoming Spell Magnitude of the hostile effects
        DAMAGE_TYPE_POISON, DAMAGE_TYPE_ACID, DAMAGE_TYPE_FIRE, DAMAGE_TYPE_CRYO, DAMAGE_TYPE_ELECTRICAL, DAMAGE_TYPE_ENERGY, POISON_GAS_EFFECT)]),
]

NON_PLAYABLE = 0x4  # PERK record flag
ENTRY_POINT_EFFECT = 2  # PRKE type
GET_GLOBAL_VALUE = 74  # condition function indices
HAS_MAGIC_EFFECT_KEYWORD = 693
RUN_ON_SUBJECT = 0


def z(s):
    return s.encode("cp1252") + b"\0"


def sub(tag, data):
    assert len(data) < 65536
    return tag + struct.pack("<H", len(data)) + data


def record(tag, flags, form_id, payload):
    return tag + struct.pack("<IIIIHH", len(payload), flags, form_id, 0, FORM_VERSION, 0) + payload


def group(label, records):
    return b"GRUP" + struct.pack("<I", 24 + len(records)) + label + struct.pack("<IHHHH", 0, 0, 0, 0, 0) + records


def local_form(local):
    # master index 1 = this plugin (master 0 is Fallout4.esm)
    return 0x01000000 | local


def condition_equals(function, parameter, value):
    # One CTDA (32 bytes): type 0 (equal, float comparison), 3 unused, the comparison value, the function, 2 unused, parameter 1 (a form),
    # parameter 2, run on (the subject), reference, parameter 3 (-1 when unused).
    return sub(b"CTDA", struct.pack("<B3xfHHIIIIi", 0, value, function, 0, parameter, 0, RUN_ON_SUBJECT, 0, -1))


def perk_record(local, edid, name, global_local, entries):
    payload = sub(b"EDID", z(edid))
    payload += sub(b"FULL", z(name))
    payload += sub(b"DESC", b"\0")
    payload += sub(b"DATA", struct.pack("<5B", 0, 0, 1, 0, 1))  # trait, level, ranks, playable, hidden
    for index, (entry_point, tabs, function, value, keyword) in enumerate(entries):
        payload += sub(b"PRKE", struct.pack("<3B", ENTRY_POINT_EFFECT, 0, 0))  # type, rank, priority
        payload += sub(b"DATA", struct.pack("<3B", entry_point, function, tabs))
        payload += sub(b"PRKC", struct.pack("<b", 0)) + condition_equals(GET_GLOBAL_VALUE, local_form(global_local), 1.0)  # tab 0: the perk owner
        if keyword is not None:
            payload += sub(b"PRKC", struct.pack("<b", 1)) + condition_equals(HAS_MAGIC_EFFECT_KEYWORD, keyword, 1.0)  # tab 1: the effect
        payload += sub(b"EPFT", b"\x01") + sub(b"EPFB", struct.pack("<H", index)) + sub(b"EPFD", struct.pack("<f", value))
        payload += sub(b"PRKF", b"")
    return record(b"PERK", NON_PLAYABLE, local_form(local), payload)


def perk_group():
    perk = b""
    for local, edid, name, global_local, entries in sorted(PERKS):
        perk += perk_record(local, edid, name, global_local, entries)
    return group(b"PERK", perk)


def replace_perks(path):
    """Put this script's PERK group in place of the plugin's own; every other group and the header's counts stay as they are."""
    data = open(path, "rb").read()
    assert data[:4] == b"TES4", "not a plugin"
    i = 24 + struct.unpack_from("<I", data, 4)[0]
    out = data[:i]
    replaced = False
    while i < len(data):
        assert data[i:i + 4] == b"GRUP", f"top level record at {i} is not a group"
        size = struct.unpack_from("<I", data, i + 4)[0]
        label = data[i + 8:i + 12]
        if label == b"PERK":
            old_count, j = 0, i + 24
            while j < i + size:
                assert data[j:j + 4] == b"PERK", f"record at {j} in the PERK group is not a perk"
                old_count += 1
                j += 24 + struct.unpack_from("<I", data, j + 4)[0]
            assert old_count == len(PERKS), f"the plugin has {old_count} perk record(s), this script {len(PERKS)}: the header's counts would change"
            out += perk_group()
            replaced = True
        else:
            out += data[i:i + size]
        i += size
    assert replaced, "the plugin has no PERK group"
    with open(path, "wb") as f:
        f.write(out)
    print("replaced the PERK group of", path, ":", len(data), "->", len(out), "bytes,", sum(len(p[4]) for p in PERKS), "perk entries")


def main(out):
    mstt = b""
    for local, edid, name, model, lo, hi in sorted(VEHICLES + EFFECTS):
        payload = sub(b"EDID", z(edid))
        payload += sub(b"OBND", struct.pack("<6h", *lo, *hi))
        payload += sub(b"FULL", z(name))
        payload += sub(b"MODL", z(model))
        payload += sub(b"DATA", b"\x01")
        mstt += record(b"MSTT", 0, local_form(local), payload)

    cont = b""
    for local, edid, name in sorted(CONTAINERS):
        payload = sub(b"EDID", z(edid))
        payload += sub(b"OBND", struct.pack("<6h", 0, 0, 0, 0, 0, 0))
        payload += sub(b"FULL", z(name))
        payload += sub(b"DATA", struct.pack("<Bf", 0, 0.0))  # flags (no respawn), weight
        cont += record(b"CONT", 0, local_form(local), payload)

    glob = b""
    for local, edid, value in sorted(GLOBALS):
        payload = sub(b"EDID", z(edid)) + sub(b"FNAM", b"f") + sub(b"FLTV", struct.pack("<f", value))
        glob += record(b"GLOB", 0, local_form(local), payload)

    # Top groups in the engine's order: GLOB, CONT, MSTT, PERK.
    groups = group(b"GLOB", glob) + group(b"CONT", cont) + group(b"MSTT", mstt) + perk_group()

    records = VEHICLES + EFFECTS + CONTAINERS + GLOBALS + PERKS
    next_id = max(v[0] for v in records) + 1
    header = sub(b"HEDR", struct.pack("<fII", 0.95, 1 + len(records), next_id))
    header += sub(b"CNAM", z(PLUGIN_AUTHOR))
    header += sub(b"SNAM", z(DESCRIPTION))
    header += sub(b"MAST", z("Fallout4.esm")) + sub(b"DATA", b"\0" * 8)
    tes4 = record(b"TES4", 0, 0, header)
    with open(out, "wb") as f:
        f.write(tes4 + groups)
    print("wrote", out, len(tes4 + groups), "bytes,", len(VEHICLES), "vehicle(s),", len(CONTAINERS), "container(s),", len(GLOBALS), "global(s),", len(PERKS), "perk(s)")


if __name__ == "__main__":
    if len(sys.argv) == 3 and sys.argv[1] == "--perks":
        replace_perks(sys.argv[2])
    else:
        main(sys.argv[1])
