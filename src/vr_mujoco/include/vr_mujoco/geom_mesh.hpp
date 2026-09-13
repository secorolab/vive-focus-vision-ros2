/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Vamsi Kalagaturu
 * See LICENSE for details. */

#pragma once

#include <cstdint>
#include <vector>

#include <mujoco/mujoco.h>

namespace vr_mujoco {

/** Triangle list in the geom's own frame, MuJoCo axes (Z up, right-handed). */
struct TriMesh
{
    std::vector<float>    positions;
    std::vector<float>    normals;
    std::vector<uint32_t> indices;
};

/** Tessellation density for the round primitives. */
struct TessOptions
{
    int   segments = 24;   // around the axis
    int   rings    = 12;   // pole to pole, sphere and hemisphere caps
    float plane_extent =
      10.0f; // half-size substituted for an infinite plane (MuJoCo size 0 means unbounded)
};

/**
 * Tessellates one geom. Returns false for geom types with no static triangle representation
 * (height fields, SDFs, rendering-only decor), which the caller should skip.
 */
bool build_geom_mesh(const mjModel *model, int geom_id, const TessOptions &opts, TriMesh *out);

} // namespace vr_mujoco
