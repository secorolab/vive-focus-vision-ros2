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

    /* The body's pose when the model is loaded, in glTF axes. Written into the node so the file
     * shows an assembled scene on its own; the client overwrites it from the pose stream. */
    std::array<float, 3> translation = { 0, 0, 0 };
    std::array<float, 4> rotation    = { 0, 0, 0, 1 }; // xyzw
};

/**
 * Minimal binary glTF (.glb) writer: triangle meshes, flat colours, one node per mesh at the
 * scene root with an identity transform. Node transforms stay identity because the viewer sets
 * each body's pose from the live pose stream; only body-local geometry is baked in here.
 */
/** A KHR_lights_punctual light, so a viewer lights the scene the way MuJoCo does. */
struct Light
{
    std::string          type = "directional"; // directional | point | spot
    std::array<float, 3> colour    = { 1, 1, 1 };
    float                intensity = 1.0f;
    std::array<float, 3> position  = { 0, 0, 0 }; // glTF axes
    std::array<float, 3> direction = { 0, -1, 0 };
};

class GlbBuilder
{
  public:
    /** Returns a material index, reusing one that already has this colour. */
    int add_material(const std::array<float, 4> &rgba);

    /** Adds a mesh plus the root node that references it; returns the node index. */
    int add_mesh_node(MeshGroup mesh);

    void add_light(Light light);

    bool write(const std::string &path) const;

  private:
    std::vector<std::array<float, 4>> materials_;
    std::vector<MeshGroup>            meshes_;
    std::vector<Light>                lights_;
};

} // namespace vr
