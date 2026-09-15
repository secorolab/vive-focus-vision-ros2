/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Vamsi Kalagaturu
 * See LICENSE for details. */

#include "vr/gltf_writer.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <limits>
#include <sstream>

#include <zlib.h>

namespace vr {
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

/** Quaternion (xyzw) rotating -Z, which is where a glTF light points, onto `dir`. */
std::array<float, 4> rotation_from_minus_z(const std::array<float, 3> &dir)
{
    const float len = std::sqrt(dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2]);
    if (len <= 0.0f) return { 0, 0, 0, 1 };
    const float d[3] = { dir[0] / len, dir[1] / len, dir[2] / len };
    const float from[3] = { 0, 0, -1 };

    const float dot = from[0] * d[0] + from[1] * d[1] + from[2] * d[2];
    if (dot > 0.999999f) return { 0, 0, 0, 1 };
    if (dot < -0.999999f) return { 0, 1, 0, 0 }; // half turn about any perpendicular axis

    const float axis[3] = { from[1] * d[2] - from[2] * d[1], from[2] * d[0] - from[0] * d[2],
                            from[0] * d[1] - from[1] * d[0] };
    const float s       = std::sqrt((1.0f + dot) * 2.0f);
    return { axis[0] / s, axis[1] / s, axis[2] / s, s * 0.5f };
}

uint32_t crc32_of(const uint8_t *data, size_t n)
{
    static uint32_t table[256];
    static bool     ready = false;
    if (!ready) {
        for (uint32_t i = 0; i < 256; ++i) {
            uint32_t c = i;
            for (int k = 0; k < 8; ++k) c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
            table[i] = c;
        }
        ready = true;
    }
    uint32_t c = 0xFFFFFFFFu;
    for (size_t i = 0; i < n; ++i) c = table[(c ^ data[i]) & 0xFF] ^ (c >> 8);
    return c ^ 0xFFFFFFFFu;
}

void png_chunk(std::vector<uint8_t> &out, const char tag[4], const std::vector<uint8_t> &payload)
{
    const uint32_t len = static_cast<uint32_t>(payload.size());
    for (int i = 3; i >= 0; --i) out.push_back(static_cast<uint8_t>(len >> (8 * i)));
    std::vector<uint8_t> tagged(tag, tag + 4);
    tagged.insert(tagged.end(), payload.begin(), payload.end());
    out.insert(out.end(), tagged.begin(), tagged.end());
    const uint32_t crc = crc32_of(tagged.data(), tagged.size());
    for (int i = 3; i >= 0; --i) out.push_back(static_cast<uint8_t>(crc >> (8 * i)));
}

/**
 * Encodes RGBA8 as PNG. glTF permits only PNG and JPEG, so raw pixels cannot be embedded
 * directly. zlib does the compression; a texture atlas is megabytes uncompressed and the stored
 * form would bloat the .glb the headset has to download.
 */
std::vector<uint8_t> encode_png(const Texture &tex)
{
    std::vector<uint8_t> raw;
    raw.reserve(static_cast<size_t>(tex.height) * (1 + tex.width * 4));
    for (int y = 0; y < tex.height; ++y) {
        raw.push_back(0); // filter type: none
        const uint8_t *row = tex.rgba.data() + static_cast<size_t>(y) * tex.width * 4;
        raw.insert(raw.end(), row, row + static_cast<size_t>(tex.width) * 4);
    }

    uLongf               bound = compressBound(static_cast<uLong>(raw.size()));
    std::vector<uint8_t> deflated(bound);
    if (compress2(deflated.data(), &bound, raw.data(), static_cast<uLong>(raw.size()), 6) != Z_OK) {
        return {};
    }
    deflated.resize(bound);

    std::vector<uint8_t> png = { 0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A };
    std::vector<uint8_t> ihdr;
    for (int i = 3; i >= 0; --i) ihdr.push_back(static_cast<uint8_t>(tex.width >> (8 * i)));
    for (int i = 3; i >= 0; --i) ihdr.push_back(static_cast<uint8_t>(tex.height >> (8 * i)));
    ihdr.insert(ihdr.end(), { 8, 6, 0, 0, 0 }); // 8-bit, RGBA, deflate, no filter, no interlace
    png_chunk(png, "IHDR", ihdr);
    png_chunk(png, "IDAT", deflated);
    png_chunk(png, "IEND", {});
    return png;
}

} // namespace

int GlbBuilder::add_material(const std::array<float, 4> &rgba, int texture, float metallic,
                             float roughness)
{
    for (size_t i = 0; i < materials_.size(); ++i) {
        const Material &m = materials_[i];
        if (m.rgba == rgba && m.texture == texture && m.metallic == metallic
            && m.roughness == roughness) {
            return static_cast<int>(i);
        }
    }
    materials_.push_back({ rgba, texture, metallic, roughness });
    return static_cast<int>(materials_.size()) - 1;
}

int GlbBuilder::add_texture(const std::string &key, Texture texture)
{
    for (size_t i = 0; i < texture_keys_.size(); ++i) {
        if (texture_keys_[i] == key) return static_cast<int>(i);
    }
    textures_.push_back(std::move(texture));
    texture_keys_.push_back(key);
    return static_cast<int>(textures_.size()) - 1;
}

int GlbBuilder::add_mesh_node(MeshGroup mesh)
{
    meshes_.push_back(std::move(mesh));
    return static_cast<int>(meshes_.size()) - 1;
}

void GlbBuilder::add_light(Light light) { lights_.push_back(std::move(light)); }

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

            const bool   has_uv    = prim.uvs.size() == static_cast<size_t>(vertex_count) * 2;
            const size_t uv_offset = bin.size();
            if (has_uv) {
                append_bytes(bin, prim.uvs.data(), prim.uvs.size() * sizeof(float));
                pad_to_4(bin);
            }

            const int idx_view = view_count++;
            const int pos_view = view_count++;
            const int nrm_view = view_count++;
            const int uv_view  = has_uv ? view_count++ : -1;
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
            if (has_uv) {
                buffer_views << ",{\"buffer\":0,\"byteOffset\":" << uv_offset
                             << ",\"byteLength\":" << prim.uvs.size() * sizeof(float)
                             << ",\"target\":" << kArrayBuffer << "}";
            }

            const int idx_acc = accessor_count++;
            const int pos_acc = accessor_count++;
            const int nrm_acc = accessor_count++;
            const int uv_acc  = has_uv ? accessor_count++ : -1;
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
            if (has_uv) {
                accessors << ",{\"bufferView\":" << uv_view << ",\"componentType\":" << kFloat
                          << ",\"count\":" << vertex_count << ",\"type\":\"VEC2\"}";
            }

            if (p) meshes_json << ",";
            meshes_json << "{\"attributes\":{\"POSITION\":" << pos_acc << ",\"NORMAL\":" << nrm_acc;
            if (has_uv) meshes_json << ",\"TEXCOORD_0\":" << uv_acc;
            meshes_json << "},\"indices\":" << idx_acc;
            if (prim.material >= 0) meshes_json << ",\"material\":" << prim.material;
            meshes_json << "}";
        }
        meshes_json << "]}";
    }
    meshes_json << "]";

    std::ostringstream json;
    json << "{\"asset\":{\"version\":\"2.0\",\"generator\":\"vr scene_export\"},";
    if (!lights_.empty()) {
        json << "\"extensionsUsed\":[\"KHR_lights_punctual\"],"
             << "\"extensions\":{\"KHR_lights_punctual\":{\"lights\":[";
        for (size_t l = 0; l < lights_.size(); ++l) {
            const Light &li = lights_[l];
            if (l) json << ",";
            json << "{\"type\":\"" << li.type << "\",\"color\":[" << fmt_float(li.colour[0]) << ","
                 << fmt_float(li.colour[1]) << "," << fmt_float(li.colour[2])
                 << "],\"intensity\":" << fmt_float(li.intensity) << "}";
        }
        json << "]}},";
    }

    json << "\"scene\":0,\"scenes\":[{\"nodes\":[";
    for (size_t m = 0; m < meshes_.size() + lights_.size(); ++m) {
        if (m) json << ",";
        json << m;
    }
    json << "]}],\"nodes\":[";
    for (size_t m = 0; m < meshes_.size(); ++m) {
        const MeshGroup &mesh = meshes_[m];
        if (m) json << ",";
        json << "{\"name\":\"" << mesh.name << "\",\"mesh\":" << m << ",\"translation\":["
             << fmt_float(mesh.translation[0]) << "," << fmt_float(mesh.translation[1]) << ","
             << fmt_float(mesh.translation[2]) << "],\"rotation\":["
             << fmt_float(mesh.rotation[0]) << "," << fmt_float(mesh.rotation[1]) << ","
             << fmt_float(mesh.rotation[2]) << "," << fmt_float(mesh.rotation[3]) << "]}";
    }

    /* A punctual light points down its node's -Z, so the node carries a rotation taking -Z onto
     * the light's direction. */
    for (size_t l = 0; l < lights_.size(); ++l) {
        const Light &li = lights_[l];
        json << ",{\"name\":\"light_" << l << "\",\"translation\":["
             << fmt_float(li.position[0]) << "," << fmt_float(li.position[1]) << ","
             << fmt_float(li.position[2]) << "],\"rotation\":[";
        const auto q = rotation_from_minus_z(li.direction);
        json << fmt_float(q[0]) << "," << fmt_float(q[1]) << "," << fmt_float(q[2]) << ","
             << fmt_float(q[3]) << "],\"extensions\":{\"KHR_lights_punctual\":{\"light\":" << l
             << "}}}";
    }
    json << "],";

    json << "\"materials\":[";
    for (size_t i = 0; i < materials_.size(); ++i) {
        const auto &c = materials_[i].rgba;
        if (i) json << ",";
        json << "{\"pbrMetallicRoughness\":{\"baseColorFactor\":[" << fmt_float(c[0]) << ","
             << fmt_float(c[1]) << "," << fmt_float(c[2]) << "," << fmt_float(c[3]) << "]";
        if (materials_[i].texture >= 0) {
            json << ",\"baseColorTexture\":{\"index\":" << materials_[i].texture << "}";
        }
        json << ",\"metallicFactor\":" << fmt_float(materials_[i].metallic)
             << ",\"roughnessFactor\":" << fmt_float(materials_[i].roughness) << "}";
        if (c[3] < 1.0f) json << ",\"alphaMode\":\"BLEND\"";
        json << ",\"doubleSided\":true}";
    }
    json << "],";

    if (!textures_.empty()) {
        /* Images live in the buffer rather than as URIs, so the .glb stays one file the headset
         * fetches in one request. */
        std::ostringstream images, textures_json;
        images << "\"images\":[";
        textures_json << "\"textures\":[";
        for (size_t t = 0; t < textures_.size(); ++t) {
            const std::vector<uint8_t> png = encode_png(textures_[t]);
            pad_to_4(bin);
            const size_t offset = bin.size();
            append_bytes(bin, png.data(), png.size());

            const int view = view_count++;
            buffer_views << ",{\"buffer\":0,\"byteOffset\":" << offset
                         << ",\"byteLength\":" << png.size() << "}";
            if (t) { images << ","; textures_json << ","; }
            images << "{\"bufferView\":" << view << ",\"mimeType\":\"image/png\"}";
            textures_json << "{\"sampler\":0,\"source\":" << t << "}";
        }
        images << "],";
        textures_json << "],";
        // One sampler: repeat in both directions, which is what tiled MuJoCo textures expect.
        json << images.str() << textures_json.str()
             << "\"samplers\":[{\"wrapS\":10497,\"wrapT\":10497}],";
    }

    /* Pad before the length is recorded, not after: an embedded PNG is any number of bytes, so
     * the chunk needs padding that the declared buffer length must already account for. Every
     * attribute was 4-byte aligned by construction, which is why this only appeared with images. */
    pad_to_4(bin);

    json << meshes_json.str() << ",\"accessors\":[" << accessors.str() << "],\"bufferViews\":["
         << buffer_views.str() << "],\"buffers\":[{\"byteLength\":" << bin.size() << "}]}";

    std::string json_chunk = json.str();
    while (json_chunk.size() % 4 != 0) json_chunk.push_back(' ');

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

} // namespace vr
