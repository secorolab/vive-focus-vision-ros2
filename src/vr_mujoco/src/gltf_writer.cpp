/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Vamsi Kalagaturu
 * See LICENSE for details. */

#include "vr_mujoco/gltf_writer.hpp"

#include <cstdio>
#include <cstring>
#include <fstream>
#include <limits>
#include <sstream>

namespace vr_mujoco {
namespace {

constexpr uint32_t kGlbMagic    = 0x46546C67; // "glTF"
constexpr uint32_t kChunkJson   = 0x4E4F534A; // "JSON"
constexpr uint32_t kChunkBin    = 0x004E4942; // "BIN\0"
constexpr int      kFloat       = 5126;
constexpr int      kUnsignedInt = 5125;
constexpr int      kArrayBuffer = 34962;
constexpr int      kIndexBuffer = 34963;

void append_bytes(std::vector<uint8_t> &buf, const void *src, size_t n)
{
    const auto *p = static_cast<const uint8_t *>(src);
    buf.insert(buf.end(), p, p + n);
}

/* glTF requires every accessor's byteOffset to be a multiple of its component size; the writer
 * only emits 4-byte components, so padding to 4 is enough. */
void pad_to_4(std::vector<uint8_t> &buf, uint8_t fill = 0)
{
    while (buf.size() % 4 != 0) buf.push_back(fill);
}

std::string fmt_float(float v)
{
    char b[32];
    std::snprintf(b, sizeof(b), "%.7g", static_cast<double>(v));
    return b;
}

} // namespace

int GlbBuilder::add_material(const std::array<float, 4> &rgba)
{
    for (size_t i = 0; i < materials_.size(); ++i) {
        if (materials_[i] == rgba) return static_cast<int>(i);
    }
    materials_.push_back(rgba);
    return static_cast<int>(materials_.size()) - 1;
}

int GlbBuilder::add_mesh_node(MeshGroup mesh)
{
    meshes_.push_back(std::move(mesh));
    return static_cast<int>(meshes_.size()) - 1;
}

bool GlbBuilder::write(const std::string &path) const
{
    std::vector<uint8_t> bin;
    std::ostringstream   accessors, buffer_views, meshes_json;

    int accessor_count = 0;
    int view_count     = 0;

    meshes_json << "\"meshes\":[";
    for (size_t m = 0; m < meshes_.size(); ++m) {
        const MeshGroup &mesh = meshes_[m];
        if (m) meshes_json << ",";
        meshes_json << "{\"name\":\"" << mesh.name << "\",\"primitives\":[";

        for (size_t p = 0; p < mesh.primitives.size(); ++p) {
            const Primitive &prim = mesh.primitives[p];
            if (prim.indices.empty() || prim.positions.empty()) continue;
            const auto vertex_count = static_cast<int>(prim.positions.size() / 3);

            /* min/max are mandatory on POSITION accessors; viewers use them for culling. */
            float lo[3] = { std::numeric_limits<float>::max(), std::numeric_limits<float>::max(),
                            std::numeric_limits<float>::max() };
            float hi[3] = { std::numeric_limits<float>::lowest(),
                            std::numeric_limits<float>::lowest(),
                            std::numeric_limits<float>::lowest() };
            for (int v = 0; v < vertex_count; ++v) {
                for (int c = 0; c < 3; ++c) {
                    const float x = prim.positions[3 * v + c];
                    lo[c]         = std::min(lo[c], x);
                    hi[c]         = std::max(hi[c], x);
                }
            }

            const size_t idx_offset = bin.size();
            append_bytes(bin, prim.indices.data(), prim.indices.size() * sizeof(uint32_t));
            pad_to_4(bin);
            const size_t pos_offset = bin.size();
            append_bytes(bin, prim.positions.data(), prim.positions.size() * sizeof(float));
            pad_to_4(bin);
            const size_t nrm_offset = bin.size();
            append_bytes(bin, prim.normals.data(), prim.normals.size() * sizeof(float));
            pad_to_4(bin);

            const int idx_view = view_count++;
            const int pos_view = view_count++;
            const int nrm_view = view_count++;
            if (idx_view) buffer_views << ",";
            buffer_views << "{\"buffer\":0,\"byteOffset\":" << idx_offset
                         << ",\"byteLength\":" << prim.indices.size() * sizeof(uint32_t)
                         << ",\"target\":" << kIndexBuffer << "},"
                         << "{\"buffer\":0,\"byteOffset\":" << pos_offset
                         << ",\"byteLength\":" << prim.positions.size() * sizeof(float)
                         << ",\"target\":" << kArrayBuffer << "},"
                         << "{\"buffer\":0,\"byteOffset\":" << nrm_offset
                         << ",\"byteLength\":" << prim.normals.size() * sizeof(float)
                         << ",\"target\":" << kArrayBuffer << "}";

            const int idx_acc = accessor_count++;
            const int pos_acc = accessor_count++;
            const int nrm_acc = accessor_count++;
            if (idx_acc) accessors << ",";
            accessors << "{\"bufferView\":" << idx_view << ",\"componentType\":" << kUnsignedInt
                      << ",\"count\":" << prim.indices.size() << ",\"type\":\"SCALAR\"},"
                      << "{\"bufferView\":" << pos_view << ",\"componentType\":" << kFloat
                      << ",\"count\":" << vertex_count << ",\"type\":\"VEC3\",\"min\":["
                      << fmt_float(lo[0]) << "," << fmt_float(lo[1]) << "," << fmt_float(lo[2])
                      << "],\"max\":[" << fmt_float(hi[0]) << "," << fmt_float(hi[1]) << ","
                      << fmt_float(hi[2]) << "]},"
                      << "{\"bufferView\":" << nrm_view << ",\"componentType\":" << kFloat
                      << ",\"count\":" << vertex_count << ",\"type\":\"VEC3\"}";

            if (p) meshes_json << ",";
            meshes_json << "{\"attributes\":{\"POSITION\":" << pos_acc << ",\"NORMAL\":" << nrm_acc
                        << "},\"indices\":" << idx_acc;
            if (prim.material >= 0) meshes_json << ",\"material\":" << prim.material;
            meshes_json << "}";
        }
        meshes_json << "]}";
    }
    meshes_json << "]";

    std::ostringstream json;
    json << "{\"asset\":{\"version\":\"2.0\",\"generator\":\"vr_mujoco scene_export\"},"
         << "\"scene\":0,\"scenes\":[{\"nodes\":[";
    for (size_t m = 0; m < meshes_.size(); ++m) {
        if (m) json << ",";
        json << m;
    }
    json << "]}],\"nodes\":[";
    for (size_t m = 0; m < meshes_.size(); ++m) {
        if (m) json << ",";
        json << "{\"name\":\"" << meshes_[m].name << "\",\"mesh\":" << m << "}";
    }
    json << "],";

    json << "\"materials\":[";
    for (size_t i = 0; i < materials_.size(); ++i) {
        const auto &c = materials_[i];
        if (i) json << ",";
        json << "{\"pbrMetallicRoughness\":{\"baseColorFactor\":[" << fmt_float(c[0]) << ","
             << fmt_float(c[1]) << "," << fmt_float(c[2]) << "," << fmt_float(c[3])
             << "],\"metallicFactor\":0,\"roughnessFactor\":0.8}";
        if (c[3] < 1.0f) json << ",\"alphaMode\":\"BLEND\"";
        json << ",\"doubleSided\":true}";
    }
    json << "],";

    json << meshes_json.str() << ",\"accessors\":[" << accessors.str() << "],\"bufferViews\":["
         << buffer_views.str() << "],\"buffers\":[{\"byteLength\":" << bin.size() << "}]}";

    std::string json_chunk = json.str();
    while (json_chunk.size() % 4 != 0) json_chunk.push_back(' ');
    pad_to_4(const_cast<std::vector<uint8_t> &>(bin));

    const uint32_t json_len  = static_cast<uint32_t>(json_chunk.size());
    const uint32_t bin_len   = static_cast<uint32_t>(bin.size());
    const uint32_t total_len = 12 + 8 + json_len + (bin_len ? 8 + bin_len : 0);

    std::ofstream out(path, std::ios::binary);
    if (!out) return false;

    const uint32_t header[3] = { kGlbMagic, 2, total_len };
    out.write(reinterpret_cast<const char *>(header), sizeof(header));
    const uint32_t json_hdr[2] = { json_len, kChunkJson };
    out.write(reinterpret_cast<const char *>(json_hdr), sizeof(json_hdr));
    out.write(json_chunk.data(), json_len);
    if (bin_len) {
        const uint32_t bin_hdr[2] = { bin_len, kChunkBin };
        out.write(reinterpret_cast<const char *>(bin_hdr), sizeof(bin_hdr));
        out.write(reinterpret_cast<const char *>(bin.data()), bin_len);
    }
    return out.good();
}

} // namespace vr_mujoco
