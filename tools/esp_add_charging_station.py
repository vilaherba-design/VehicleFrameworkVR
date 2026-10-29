#!/usr/bin/env python3
"""Add (or replace) the vehicle charging station in VehicleFrameworkVR_Vehicles.esp: its base object and its settlement workshop recipe,
and keep the TES4 header in step (record count, next object id), as esp_add_mstt.py does for the vehicles.

The layout is the one of a vanilla powered workshop object with no activation of its own, WorkshopWaterPurifier (Fallout4.esm 0002058C,
an ACTI) and its recipe workshop_co_WaterPurifier (0005A0CD, a COBJ):
  ACTI VFVR_ChargingStation: EDID, VMAD (workshopobjectscript with its WorkshopParent property, as the purifier, then
       VehicleFrameworkVR:ChargingStation), OBND, FULL, MODL, KSIZ/KWDA (WorkshopCanBePowered, WorkshopPowerConnection), PRPS (PowerRequired
       0x330 = 10), PNAM (the purifier's marker colour), FNAM 0. The purifier's water rating, resource keyword and destruction data are left out.
  COBJ workshop_co_VFVR_ChargingStation: EDID, YNAM/ZNAM (the purifier's pick up / put down sounds), FVPA (8 steel, 6 copper, 3 circuitry,
       2 nuclear material), DESC, CNAM (the station), BNAM WorkshopWorkbenchTypePower and FNAM WorkshopRecipeFilterResource02Power03Misc
       (the settlement build menu, Power > Misc, as the vanilla powered miscellany), INTV 1.
The groups are made when the plugin has none (ACTI before CONT, COBJ after PERK, the order of Fallout4.esm).

Usage: esp_add_charging_station.py <plugin.esp> [station form id, hex] [recipe form id, hex]
Without form ids the plugin's next object ids are used (an existing VFVR_ChargingStation / workshop_co_VFVR_ChargingStation keeps its id).
"""
import struct
import sys

STATION_EDID = "VFVR_ChargingStation"
RECIPE_EDID = "workshop_co_VFVR_ChargingStation"
NAME = "Charging station"
DESCRIPTION = "Charges the battery of a vehicle parked within 6 m while it has power."
MODEL = "VehicleFrameworkVR\\Props\\ChargingStation.nif"
SCRIPT = "VehicleFrameworkVR:ChargingStation"
BOUNDS = (-29, -23, 0, 29, 23, 134)            # Meshes/VehicleFrameworkVR/Props/ChargingStation.nif, game units
POWER_REQUIRED = 10.0

# Fallout4.esm (master 0 of the plugin)
WORKSHOP_PARENT = 0x0002058E                    # QUST WorkshopParent
KW_CAN_BE_POWERED = 0x0003037E                  # WorkshopCanBePowered
KW_POWER_CONNECTION = 0x00054BA4                # WorkshopPowerConnection
AV_POWER_REQUIRED = 0x00000330                  # PowerRequired
WORKBENCH_POWER = 0x0005A0CA                    # WorkshopWorkbenchTypePower
FILTER_POWER_MISC = 0x0016AC69                  # WorkshopRecipeFilterResource02Power03Misc
SOUND_PICK_UP = 0x000196B1                      # the purifier recipe's YNAM
SOUND_PUT_DOWN = 0x000196BB                     # the purifier recipe's ZNAM
COMPONENTS = ((0x0001FABD, 8),                  # c_Steel
              (0x0001FA9C, 6),                  # c_Copper
              (0x0001FA9B, 3),                  # c_Circuitry
              (0x0001FAB3, 2))                  # c_NuclearMaterial
MARKER_COLOUR = bytes.fromhex("cc4c3300")       # the purifier's PNAM

GROUP_ORDER = ["GMST", "KYWD", "GLOB", "ACTI", "CONT", "MSTT", "PERK", "COBJ"]     # Fallout4.esm order, the types this plugin uses


def sub(t, data):
    return struct.pack('<4sH', t.encode(), len(data)) + data


def zstr(s):
    return s.encode('cp1252') + b'\0'


def wstr(s):
    b = s.encode('cp1252')
    return struct.pack('<H', len(b)) + b


def subrecords(body):
    out, p = [], 0
    while p < len(body):
        t, n = struct.unpack_from('<4sH', body, p)
        out.append((t.decode(), body[p + 6:p + 6 + n]))
        p += 6 + n
    return out


def station_body():
    # VMAD v6, object format 2: workshopobjectscript (WorkshopParent: object, status 1, unused 0, alias -1, form) and our script
    vmad = struct.pack('<hhH', 6, 2, 2)
    vmad += wstr("workshopobjectscript") + b'\x00' + struct.pack('<H', 1)
    vmad += wstr("WorkshopParent") + b'\x01\x01' + struct.pack('<HhI', 0, -1, WORKSHOP_PARENT)
    vmad += wstr(SCRIPT) + b'\x00' + struct.pack('<H', 0)
    kw = (KW_CAN_BE_POWERED, KW_POWER_CONNECTION)
    return (sub('EDID', zstr(STATION_EDID)) + sub('VMAD', vmad) + sub('OBND', struct.pack('<6h', *BOUNDS)) + sub('FULL', zstr(NAME))
            + sub('MODL', zstr(MODEL)) + sub('KSIZ', struct.pack('<I', len(kw))) + sub('KWDA', struct.pack('<%dI' % len(kw), *kw))
            + sub('PRPS', struct.pack('<If', AV_POWER_REQUIRED, POWER_REQUIRED)) + sub('PNAM', MARKER_COLOUR) + sub('FNAM', b'\0\0'))


def recipe_body(station_fid):
    fvpa = b''.join(struct.pack('<II', f, n) for f, n in COMPONENTS)
    return (sub('EDID', zstr(RECIPE_EDID)) + sub('YNAM', struct.pack('<I', SOUND_PICK_UP)) + sub('ZNAM', struct.pack('<I', SOUND_PUT_DOWN))
            + sub('FVPA', fvpa) + sub('DESC', zstr(DESCRIPTION)) + sub('CNAM', struct.pack('<I', station_fid))
            + sub('BNAM', struct.pack('<I', WORKBENCH_POWER)) + sub('FNAM', struct.pack('<I', FILTER_POWER_MISC))
            + sub('INTV', struct.pack('<HH', 1, 0)))


def record(t, fid, body):
    return struct.pack('<4sIIIIHH', t.encode(), len(body), 0, fid, 0, 131, 0) + body


class Plugin:
    def __init__(self, path):
        self.d = bytearray(open(path, 'rb').read())
        assert self.d[:4] == b'TES4'
        size, = struct.unpack_from('<I', self.d, 4)
        self.tes_end = 24 + size
        self.subs = subrecords(bytes(self.d[24:self.tes_end]))
        self.version, self.nrec, self.next_id = struct.unpack('<fiI', next(v for t, v in self.subs if t == 'HEDR'))

    def groups(self):
        out, p = {}, self.tes_end
        while p < len(self.d):
            assert self.d[p:p + 4] == b'GRUP'
            size, label = struct.unpack_from('<I4s', self.d, p + 4)
            out[label.decode()] = p
            p += size
        return out

    def find(self, label, edid):
        """The form id of the record of `label` with this editor id, or None."""
        g = self.groups().get(label)
        if g is None:
            return None
        gsize, = struct.unpack_from('<I', self.d, g + 4)
        q = g + 24
        while q < g + gsize:
            rsize, rfid = struct.unpack_from('<I4xI', self.d, q + 4)
            if dict(subrecords(bytes(self.d[q + 24:q + 24 + rsize]))).get('EDID') == zstr(edid):
                return rfid
            q += 24 + rsize
        return None

    def put(self, label, fid, body):
        """Add or replace (same form id) a record of type `label`, making its top group when there is none. True when added."""
        rec = record(label, fid, body)
        groups = self.groups()
        if label not in groups:
            later = [groups[l] for l in GROUP_ORDER[GROUP_ORDER.index(label) + 1:] if l in groups]
            at = min(later) if later else len(self.d)
            self.d[at:at] = struct.pack('<4sI4si8x', b'GRUP', 24, label.encode(), 0)      # as the plugin's other groups
            groups = self.groups()
        g = groups[label]
        gsize, = struct.unpack_from('<I', self.d, g + 4)
        q = g + 24
        while q < g + gsize:
            rsize, rfid = struct.unpack_from('<I4xI', self.d, q + 4)
            if rfid == fid:
                self.d[q:q + 24 + rsize] = rec
                struct.pack_into('<I', self.d, g + 4, gsize + len(rec) - (24 + rsize))
                return False
            q += 24 + rsize
        self.d[g + gsize:g + gsize] = rec
        struct.pack_into('<I', self.d, g + 4, gsize + len(rec))
        self.nrec += 1
        return True

    def new_id(self):
        fid = self.next_id
        self.next_id += 1
        return fid

    def save(self, path):
        hedr = struct.pack('<fiI', self.version, self.nrec, self.next_id)
        tes_body = b''.join(sub(t, hedr if t == 'HEDR' else v) for t, v in self.subs)
        assert len(tes_body) == self.tes_end - 24
        self.d[24:self.tes_end] = tes_body
        open(path, 'wb').write(self.d)


def main(path, station_hex=None, recipe_hex=None):
    p = Plugin(path)
    station = p.find('ACTI', STATION_EDID)
    station_low = int(station_hex, 16) if station_hex else (station & 0xFFFFFF if station else p.new_id())
    recipe = p.find('COBJ', RECIPE_EDID)
    recipe_low = int(recipe_hex, 16) if recipe_hex else (recipe & 0xFFFFFF if recipe else p.new_id())
    p.next_id = max(p.next_id, station_low + 1, recipe_low + 1)
    station_fid, recipe_fid = 0x01000000 | station_low, 0x01000000 | recipe_low    # one master: the plugin's own records are 01xxxxxx
    a = p.put('ACTI', station_fid, station_body())
    b = p.put('COBJ', recipe_fid, recipe_body(station_fid))
    p.save(path)
    print('%s %s 0x%06X, %s %s 0x%06X (%s, %d records, next id 0x%X)' % ('added' if a else 'replaced', STATION_EDID, station_low,
          'added' if b else 'replaced', RECIPE_EDID, recipe_low, path, p.nrec, p.next_id))


if __name__ == '__main__':
    if len(sys.argv) not in (2, 3, 4):
        sys.exit(__doc__)
    main(*sys.argv[1:])
