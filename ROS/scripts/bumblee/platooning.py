"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class platooning(object):
    __slots__ = ["myId", "platoonId", "role", "startStop", "laneId", "laneChange", "mySpeed", "currentLeader"]

    __typenames__ = ["int8_t", "int8_t", "int8_t", "int8_t", "int8_t", "int8_t", "int8_t", "int8_t"]

    __dimensions__ = [None, None, None, None, None, None, None, None]

    def __init__(self):
        self.myId = 0
        self.platoonId = 0
        self.role = 0
        self.startStop = 0
        self.laneId = 0
        self.laneChange = 0
        self.mySpeed = 0
        self.currentLeader = 0

    def encode(self):
        buf = BytesIO()
        buf.write(platooning._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">bbbbbbbb", self.myId, self.platoonId, self.role, self.startStop, self.laneId, self.laneChange, self.mySpeed, self.currentLeader))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != platooning._get_packed_fingerprint():
            raise ValueError("Decode error")
        return platooning._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = platooning()
        self.myId, self.platoonId, self.role, self.startStop, self.laneId, self.laneChange, self.mySpeed, self.currentLeader = struct.unpack(">bbbbbbbb", buf.read(8))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if platooning in parents: return 0
        tmphash = (0x9dda450eb9576b51) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff) + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if platooning._packed_fingerprint is None:
            platooning._packed_fingerprint = struct.pack(">Q", platooning._get_hash_recursive([]))
        return platooning._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

