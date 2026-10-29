#!/usr/bin/env python3
"""Add (or replace) turret kits in VehicleFrameworkVR_Vehicles.esp: each kit item and its chemistry workbench recipe, and keep the TES4
header in step (record count, next object id), as esp_add_charging_station.py does for the charging station.

Each kit is what a Turrets/<type>.json asks for ("kit": "VehicleFrameworkVR_Vehicles.esp|0x00080F" for mg_vanilla, 0x811 twin_mg, 0x813
cannon, 0x815 rockets; the recipes follow each kit): the plugin fits the turret from the vehicle's menu when the player carries one.
A vanilla weapons workbench only makes weapon mods, so the kit is made at the chemistry workbench, as the vanilla utility items (Fallout4.esm co_food_CuttingFluid 000657FB, BoS_co_ReactorCoolant):
  MISC VFVR_TurretKit_<type>: EDID, OBND, FULL, MODL, DATA (value, weight). The model is the vanilla weapon mod crate (Props\\ModsPartBox\\
       ModCrate.nif, the box of RRR09CarePackage and the misc mod items) with its bounds.
  COBJ VFVR_co_TurretKit_<type>: EDID, FVPA (the components in KITS), CNAM (the kit), BNAM WorkbenchChemlab,
       FNAM RecipeUtility (the chemistry workbench's Utility page, as co_food_CuttingFluid), INTV 1.
The MISC group is made when the plugin has none (between CONT and MSTT, the order of Fallout4.esm).

Usage: esp_add_turret_kit.py <plugin.esp> [mg] [twin_mg] [cannon] [rockets]     (no type = mg)
"""
import struct
import sys

import esp_add_charging_station as esp

MODEL = "Props\\ModsPartBox\\ModCrate.nif"
BOUNDS = (-14, -4, 0, 14, 5, 7)                 # the crate's OBND in Fallout4.esm (RRR09CarePackage 00146133)

# Fallout4.esm (master 0 of the plugin)
WORKBENCH_CHEMLAB = 0x00102158                  # KYWD WorkbenchChemlab
FILTER_UTILITY = 0x0006980C                     # KYWD RecipeUtility
STEEL, GEARS, SCREWS, SPRINGS = 0x0001FABD, 0x0001FAB0, 0x0003D294, 0x0001FABC           # CMPO c_Steel, c_Gears, c_Screws, c_Springs
CIRCUITRY, OIL, NUCLEAR = 0x0001FA9B, 0x0001FAB4, 0x0001FAB3                             # CMPO c_Circuitry, c_Oil, c_NuclearMaterial

# One kit per turret type (Turrets/<type>.json "kit"): editor ids, name, value, weight, components, and the form ids the JSON expects.
KITS = {
    "mg": dict(kit="VFVR_TurretKit_MG", recipe="VFVR_co_TurretKit_MG", name="Machine gun turret kit", value=250, weight=10.0,
               ids=(0x80F, 0x810), components=((STEEL, 12), (GEARS, 4), (SCREWS, 4), (SPRINGS, 2), (CIRCUITRY, 2), (OIL, 2))),
    "twin_mg": dict(kit="VFVR_TurretKit_TwinMG", recipe="VFVR_co_TurretKit_TwinMG", name="Twin machine gun turret kit", value=300, weight=12.0,
                    ids=(0x811, 0x812), components=((STEEL, 14), (GEARS, 4), (SCREWS, 4), (SPRINGS, 2), (CIRCUITRY, 2), (OIL, 2))),
    "cannon": dict(kit="VFVR_TurretKit_Cannon", recipe="VFVR_co_TurretKit_Cannon", name="Cannon turret kit", value=400, weight=20.0,
                   ids=(0x813, 0x814), components=((STEEL, 20), (GEARS, 6), (SPRINGS, 4), (OIL, 2))),
    "rockets": dict(kit="VFVR_TurretKit_Rockets", recipe="VFVR_co_TurretKit_Rockets", name="Rocket turret kit", value=450, weight=15.0,
                    ids=(0x815, 0x816), components=((STEEL, 14), (CIRCUITRY, 4), (SCREWS, 4), (NUCLEAR, 2))),
}

esp.GROUP_ORDER = ["GMST", "KYWD", "GLOB", "ACTI", "CONT", "MISC", "MSTT", "PERK", "COBJ"]     # Fallout4.esm order, the types this plugin uses


def kit_body(k):
    return (esp.sub('EDID', esp.zstr(k["kit"])) + esp.sub('OBND', struct.pack('<6h', *BOUNDS)) + esp.sub('FULL', esp.zstr(k["name"]))
            + esp.sub('MODL', esp.zstr(MODEL)) + esp.sub('DATA', struct.pack('<If', k["value"], k["weight"])))


def recipe_body(k, kit_fid):
    fvpa = b''.join(struct.pack('<II', f, n) for f, n in k["components"])
    return (esp.sub('EDID', esp.zstr(k["recipe"])) + esp.sub('FVPA', fvpa) + esp.sub('CNAM', struct.pack('<I', kit_fid))
            + esp.sub('BNAM', struct.pack('<I', WORKBENCH_CHEMLAB)) + esp.sub('FNAM', struct.pack('<I', FILTER_UTILITY))
            + esp.sub('INTV', struct.pack('<HH', 1, 0)))


def main(path, *types):
    p = esp.Plugin(path)
    for name in types or ("mg",):
        k = KITS[name]
        kit_low, recipe_low = k["ids"]
        p.next_id = max(p.next_id, kit_low + 1, recipe_low + 1)
        kit_fid, recipe_fid = 0x01000000 | kit_low, 0x01000000 | recipe_low    # one master: the plugin's own records are 01xxxxxx
        a = p.put('MISC', kit_fid, kit_body(k))
        b = p.put('COBJ', recipe_fid, recipe_body(k, kit_fid))
        print('%s %s 0x%06X, %s %s 0x%06X' % ('added' if a else 'replaced', k["kit"], kit_low, 'added' if b else 'replaced', k["recipe"], recipe_low))
    p.save(path)
    print('%s: %d records, next id 0x%X' % (path, p.nrec, p.next_id))


if __name__ == '__main__':
    if len(sys.argv) < 2 or any(t not in KITS for t in sys.argv[2:]):
        sys.exit(__doc__)
    main(*sys.argv[1:])
