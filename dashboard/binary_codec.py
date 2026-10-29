"""Binary v2 decoding for the optional Python dashboard; no network side effects."""
import json
from pathlib import Path

FIELDS = json.loads((Path(__file__).resolve().parents[1] / 'docs/protocol/telemetry-fields.json').read_text())

def decode_telemetry(payload):
    data = bytes(payload)
    v3 = len(data)>1 and data[1]==3
    header, base = (19,15) if v3 else (18,14)
    if len(data) < header or len(data) > 512 or (data[0]!=0xb1 or data[1] not in (2,3) or data[2]!=1) or data[3] & (0xc0 if v3 else 0xe0) or (v3 and data[14]&0xf0):
        raise ValueError('Invalid binary telemetry')
    result = dict(schema=data[1], demo=bool(data[3] & 1), base_valid=bool(data[3] & 2),
                  seq=int.from_bytes(data[4:8], 'little'), uptime_ms=int.from_bytes(data[8:12], 'little'),
                  fault_mask=int.from_bytes(data[12:15 if v3 else 14], 'little'), speed_percent=data[base],
                  distance_cm=int.from_bytes(data[base+1:base+4], 'little'), buddies={})
    if result['speed_percent'] > 100 or result['distance_cm'] > 100000:
        raise ValueError('Invalid binary measurement')
    at, seen, mask = header, 0, (data[3] >> 2) & (15 if v3 else 7)
    while at < len(data):
        if at + 6 > len(data):
            raise ValueError('Truncated section')
        buddy, length = data[at:at+2]
        flag = {2: 8 if v3 else 0, 3: 1, 4: 2, 5: 4}.get(buddy, 0)
        if not flag or not mask & flag or seen & flag or at + 6 + length > len(data):
            raise ValueError('Invalid section')
        raw = data[at+6:at+6+length]
        section = dict(updated_ms=int.from_bytes(data[at+2:at+6], 'little'))
        if buddy == 4:
            if length != 17 or raw[0] != 0x21:
                raise ValueError('Invalid Buddy 4 frame')
            section['raw'] = list(raw)
        else:
            fields = FIELDS[str(buddy)]
            bits = sum(field[1] for field in fields)
            if length != (bits+7)//8:
                raise ValueError('Invalid section length')
            word, shift = int.from_bytes(raw, 'little'), 0
            for name, width, minimum, maximum, _ in fields:
                value = (word >> shift) & ((1 << width)-1)
                if minimum < 0 and value >= 1 << (width-1):
                    value -= 1 << width
                if not minimum <= value <= maximum:
                    raise ValueError('Invalid field: ' + name)
                section[name] = value
                shift += width
            if word >> shift:
                raise ValueError('Nonzero padding')
        result['buddies'][buddy] = section
        seen |= flag
        at += 6 + length
    if seen != mask:
        raise ValueError('Missing section')
    return result
