/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Vamsi Kalagaturu
 * See LICENSE for details. */

#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace vr {

/** One draw call: a triangle list with a single material. */
struct Primitive
{
    std::vector<float>    positions; // xyz per vertex
    std::vector<float>    normals;   // xyz per vertex, same count as positions
    std::vector<uint32_t> indices;   // triangle list
    int                   material = -1;
};

/** All geometry of one MuJoCo body, in body-local glTF axes, split by material. */
struct MeshGroup
{
    std::string            name;
    std::vector<Primitive> primitives;
};

/**
 * Minimal binary glTF (.glb) writer: triangle meshes, flat colours, one node per mesh at the
 * scene root with an identity transform. Node transforms stay identity because the viewer sets
 * each body's pose from the live pose stream; only body-local geometry is baked in here.
 */
class GlbBuilder
{
  public:
    /** Returns a material index, reusing one that already has this colour. */
    int add_material(const std::array<float, 4> &rgba);

    /** Adds a mesh plus the root node that references it; returns the node index. */
    int add_mesh_node(MeshGroup mesh);

    bool write(const std::string &path) const;

  private:
    std::vector<std::array<float, 4>> materials_;
    std::vector<MeshGroup>            meshes_;
};

} // namespace vr
