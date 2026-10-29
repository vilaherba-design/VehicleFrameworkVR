#!/usr/bin/env python3
"""Add (or replace) a movable static (MSTT) vehicle base object in VehicleFrameworkVR_Vehicles.esp, the same shape as the existing ones
(EDID, OBND, FULL, MODL, DATA = 1), and keep the TES4 header in step (record count, next object id).

Usage: esp_add_mstt.py <plugin.esp> <EditorID> <"Name"> <model path under Meshes> <x1,y1,z1,x2,y2,z2> [form id, hex, e.g. 808]
Without a form id the plugin's next object id is used. Prints the form id.
"""
import struct
import sys


def sub(t, data):
    return struct.pack('<4sH', t.encode(), len(data)) + data


def zstr(s):
    return s.encode('cp1252') + b'\0'


def subrecords(body):
    out, p = [], 0
    while p < len(body):
        t, n = struct.unpack_from('<4sH', body, p)
        out.append((t.decode(), body[p + 6:p + 6 + n]))
        p += 6 + n
    return out


def main(path, edid, name, model, bounds, fid_hex=None):
    d = bytearray(open(path, 'rb').read())
    assert d[:4] == b'TES4'
    tes_size, = struct.unpack_from('<I', d, 4)
    tes_end = 24 + tes_size
    subs = subrecords(bytes(d[24:tes_end]))
    hedr = next(v for t, v in subs if t == 'HEDR')
    version, nrec, next_id = struct.unpack('<fiI', hedr)
    fid_low = int(fid_hex, 16) if fid_hex else next_id
    fid = 0x01000000 | fid_low                      # one master: the plugin's own records are 01xxxxxx
    body = (sub('EDID', zstr(edid)) + sub('OBND', struct.pack('<6h', *bounds)) + sub('FULL', zstr(name)) + sub('MODL', zstr(model))
            + sub('DATA', b'\x01'))
    rec = struct.pack('<4sIIIIHH', b'MSTT', len(body), 0, fid, 0, 131, 0) + body
    # find the MSTT group (top level)
    p = tes_end
    grp = None
    while p < len(d):
        assert d[p:p + 4] == b'GRUP'
        size, label = struct.unpack_from('<I4s', d, p + 4)
        if label == b'MSTT':
            grp = p
            break
        p += size
    assert grp is not None, 'no MSTT group'
    gsize, = struct.unpack_from('<I', d, grp + 4)
    # replace a record with the same form id, if any
    q, added = grp + 24, True
    while q < grp + gsize:
        rsize, = struct.unpack_from('<I', d, q + 4)
        rfid, = struct.unpack_from('<I', d, q + 12)
        if rfid == fid:
            d[q:q + 24 + rsize] = rec
            gsize += len(rec) - (24 + rsize)
            added = False
            break
        q += 24 + rsize
    if added:
        d[grp + gsize:grp + gsize] = rec
        gsize += len(rec)
    struct.pack_into('<I', d, grp + 4, gsize)
    nrec += 1 if added else 0
    next_id = max(next_id, fid_low + 1)
    new_hedr = struct.pack('<fiI', version, nrec, next_id)
    tes_body = b''.join(sub(t, new_hedr if t == 'HEDR' else v) for t, v in subs)
    assert len(tes_body) == tes_size
    d[24:tes_end] = tes_body
    open(path, 'wb').write(d)
    print('%s %s 0x%06X (%s, %d records, next id 0x%X)' % ('added' if added else 'replaced', edid, fid_low, path, nrec, next_id))


if __name__ == '__main__':
    if len(sys.argv) not in (6, 7):
        sys.exit(__doc__)
    main(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4], [int(v) for v in sys.argv[5].split(',')], sys.argv[6] if len(sys.argv) > 6 else None)
