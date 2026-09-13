"""Structural validator for the .glb scene_export writes: chunk layout, accessor bounds, index range."""
import json
import struct
import sys

path = sys.argv[1]
raw = open(path, "rb").read()

magic, version, length = struct.unpack_from("<III", raw, 0)
assert magic == 0x46546C67, f"bad magic {magic:#x}"
assert version == 2, f"bad version {version}"
assert length == len(raw), f"header length {length} != file size {len(raw)}"

off, chunks = 12, {}
while off < len(raw):
    clen, ctype = struct.unpack_from("<II", raw, off)
    chunks[ctype] = raw[off + 8 : off + 8 + clen]
    assert clen % 4 == 0, f"chunk {ctype:#x} length {clen} not 4-aligned"
    off += 8 + clen
assert off == len(raw), "trailing bytes after last chunk"

gltf = json.loads(chunks[0x4E4F534A])
bin_chunk = chunks.get(0x004E4942, b"")
assert gltf["buffers"][0]["byteLength"] == len(bin_chunk), "buffer length mismatch"

for i, bv in enumerate(gltf["bufferViews"]):
    end = bv["byteOffset"] + bv["byteLength"]
    assert end <= len(bin_chunk), f"bufferView {i} overruns BIN chunk"
    assert bv["byteOffset"] % 4 == 0, f"bufferView {i} byteOffset not 4-aligned"

SIZES = {5125: 4, 5126: 4}
COMPONENTS = {"SCALAR": 1, "VEC3": 3}
for i, acc in enumerate(gltf["accessors"]):
    bv = gltf["bufferViews"][acc["bufferView"]]
    need = acc["count"] * SIZES[acc["componentType"]] * COMPONENTS[acc["type"]]
    assert need <= bv["byteLength"], f"accessor {i} needs {need} > bufferView {bv['byteLength']}"
    if acc["type"] == "VEC3" and "min" in acc:
        assert len(acc["min"]) == 3 and len(acc["max"]) == 3, f"accessor {i} bad min/max"

tris = verts = 0
for mesh in gltf["meshes"]:
    for prim in mesh["primitives"]:
        pos = gltf["accessors"][prim["attributes"]["POSITION"]]
        nrm = gltf["accessors"][prim["attributes"]["NORMAL"]]
        idx = gltf["accessors"][prim["indices"]]
        assert pos["count"] == nrm["count"], "POSITION/NORMAL count mismatch"
        assert idx["count"] % 3 == 0, "index count not a multiple of 3"
        bv = gltf["bufferViews"][idx["bufferView"]]
        data = bin_chunk[bv["byteOffset"] : bv["byteOffset"] + bv["byteLength"]]
        hi = max(struct.unpack(f"<{idx['count']}I", data))
        assert hi < pos["count"], f"index {hi} out of range for {pos['count']} vertices"
        assert prim["material"] < len(gltf["materials"]), "material index out of range"
        tris += idx["count"] // 3
        verts += pos["count"]

names = [n["name"] for n in gltf["nodes"]]
print(f"OK  {path}")
print(f"    nodes={len(gltf['nodes'])} meshes={len(gltf['meshes'])} "
      f"materials={len(gltf['materials'])} tris={tris} verts={verts}")
print(f"    node names: {names}")
