/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Vamsi Kalagaturu
 * See LICENSE for details. */

#include "vr/geom_mesh.hpp"

#include <cmath>

namespace vr {
namespace {

constexpr float kPi = 3.14159265358979323846f;

void push_vertex(TriMesh *m, float px, float py, float pz, float nx, float ny, float nz)
{
    m->positions.insert(m->positions.end(), { px, py, pz });
    m->normals.insert(m->normals.end(), { nx, ny, nz });
}

void push_tri(TriMesh *m, uint32_t a, uint32_t b, uint32_t c)
{
    m->indices.insert(m->indices.end(), { a, b, c });
}

uint32_t vertex_count(const TriMesh *m) { return static_cast<uint32_t>(m->positions.size() / 3); }

void build_box(TriMesh *m, float sx, float sy, float sz)
{
    /* Six independent quads: a cube needs per-face normals, so corners are not shared. */
    const float n[6][3]  = { { 1, 0, 0 }, { -1, 0, 0 }, { 0, 1, 0 },
                             { 0, -1, 0 }, { 0, 0, 1 }, { 0, 0, -1 } };
    const float u[6][3]  = { { 0, 1, 0 }, { 0, -1, 0 }, { -1, 0, 0 },
                             { 1, 0, 0 }, { 1, 0, 0 }, { -1, 0, 0 } };
    const float v[6][3]  = { { 0, 0, 1 }, { 0, 0, 1 }, { 0, 0, 1 },
                             { 0, 0, 1 }, { 0, 1, 0 }, { 0, 1, 0 } };
    const float half[3]  = { sx, sy, sz };

    for (int f = 0; f < 6; ++f) {
        const uint32_t base = vertex_count(m);
        for (int corner = 0; corner < 4; ++corner) {
            const float su = (corner == 0 || corner == 3) ? -1.0f : 1.0f;
            const float sv = (corner < 2) ? -1.0f : 1.0f;
            float       p[3];
            for (int c = 0; c < 3; ++c) {
                p[c] = (n[f][c] + su * u[f][c] + sv * v[f][c]) * half[c];
            }
            push_vertex(m, p[0], p[1], p[2], n[f][0], n[f][1], n[f][2]);
        }
        push_tri(m, base, base + 1, base + 2);
        push_tri(m, base, base + 2, base + 3);
    }
}

void build_plane(TriMesh *m, float hx, float hy)
{
    const uint32_t base = vertex_count(m);
    push_vertex(m, -hx, -hy, 0, 0, 0, 1);
    push_vertex(m, hx, -hy, 0, 0, 0, 1);
    push_vertex(m, hx, hy, 0, 0, 0, 1);
    push_vertex(m, -hx, hy, 0, 0, 0, 1);
    push_tri(m, base, base + 1, base + 2);
    push_tri(m, base, base + 2, base + 3);
}

/** UV sphere scaled per axis; equal radii give a sphere, unequal an ellipsoid. */
void build_ellipsoid(TriMesh *m, float rx, float ry, float rz, const TessOptions &opts)
{
    const uint32_t base = vertex_count(m);
    for (int i = 0; i <= opts.rings; ++i) {
        const float phi = kPi * static_cast<float>(i) / static_cast<float>(opts.rings);
        for (int j = 0; j <= opts.segments; ++j) {
            const float theta = 2.0f * kPi * static_cast<float>(j) /
                                static_cast<float>(opts.segments);
            const float ux = std::sin(phi) * std::cos(theta);
            const float uy = std::sin(phi) * std::sin(theta);
            const float uz = std::cos(phi);
            /* Correct normal for a scaled sphere is the inverse-scaled direction, not the
             * position: n = (x/rx^2, y/ry^2, z/rz^2), normalised. */
            float nx = ux / rx, ny = uy / ry, nz = uz / rz;
            const float len = std::sqrt(nx * nx + ny * ny + nz * nz);
            if (len > 0.0f) { nx /= len; ny /= len; nz /= len; }
            push_vertex(m, ux * rx, uy * ry, uz * rz, nx, ny, nz);
        }
    }
    const uint32_t row = static_cast<uint32_t>(opts.segments + 1);
    for (int i = 0; i < opts.rings; ++i) {
        for (int j = 0; j < opts.segments; ++j) {
            const uint32_t a = base + static_cast<uint32_t>(i) * row + static_cast<uint32_t>(j);
            push_tri(m, a, a + row, a + 1);
            push_tri(m, a + 1, a + row, a + row + 1);
        }
    }
}

/** Tube wall around the z axis between z = -half and z = +half. */
void build_tube(TriMesh *m, float radius, float half, const TessOptions &opts)
{
    const uint32_t base = vertex_count(m);
    for (int j = 0; j <= opts.segments; ++j) {
        const float theta = 2.0f * kPi * static_cast<float>(j) / static_cast<float>(opts.segments);
        const float cx = std::cos(theta), sy = std::sin(theta);
        push_vertex(m, radius * cx, radius * sy, -half, cx, sy, 0);
        push_vertex(m, radius * cx, radius * sy, half, cx, sy, 0);
    }
    for (int j = 0; j < opts.segments; ++j) {
        const uint32_t a = base + 2u * static_cast<uint32_t>(j);
        push_tri(m, a, a + 2, a + 1);
        push_tri(m, a + 1, a + 2, a + 3);
    }
}

/** Flat disc cap facing dir (+1 or -1) along z. */
void build_disc(TriMesh *m, float radius, float z, float dir, const TessOptions &opts)
{
    const uint32_t centre = vertex_count(m);
    push_vertex(m, 0, 0, z, 0, 0, dir);
    for (int j = 0; j <= opts.segments; ++j) {
        const float theta = 2.0f * kPi * static_cast<float>(j) / static_cast<float>(opts.segments);
        push_vertex(m, radius * std::cos(theta), radius * std::sin(theta), z, 0, 0, dir);
    }
    for (int j = 0; j < opts.segments; ++j) {
        const uint32_t a = centre + 1u + static_cast<uint32_t>(j);
        if (dir > 0) {
            push_tri(m, centre, a, a + 1);
        } else {
            push_tri(m, centre, a + 1, a);
        }
    }
}

/** Hemisphere cap centred at z_centre, bulging towards dir. */
void build_hemisphere(TriMesh *m, float radius, float z_centre, float dir, const TessOptions &opts)
{
    const int      rings = std::max(2, opts.rings / 2);
    const uint32_t base  = vertex_count(m);
    for (int i = 0; i <= rings; ++i) {
        const float phi = 0.5f * kPi * static_cast<float>(i) / static_cast<float>(rings);
        for (int j = 0; j <= opts.segments; ++j) {
            const float theta = 2.0f * kPi * static_cast<float>(j) /
                                static_cast<float>(opts.segments);
            const float nx = std::sin(phi) * std::cos(theta);
            const float ny = std::sin(phi) * std::sin(theta);
            const float nz = dir * std::cos(phi);
            push_vertex(m, radius * nx, radius * ny, z_centre + radius * nz, nx, ny, nz);
        }
    }
    const uint32_t row = static_cast<uint32_t>(opts.segments + 1);
    for (int i = 0; i < rings; ++i) {
        for (int j = 0; j < opts.segments; ++j) {
            const uint32_t a = base + static_cast<uint32_t>(i) * row + static_cast<uint32_t>(j);
            if (dir > 0) {
                push_tri(m, a, a + 1, a + row);
                push_tri(m, a + 1, a + row + 1, a + row);
            } else {
                push_tri(m, a, a + row, a + 1);
                push_tri(m, a + 1, a + row, a + row + 1);
            }
        }
    }
}

/** MuJoCo mesh asset, flat-shaded: face normals need one vertex per face corner. */
void build_asset_mesh(TriMesh *m, const mjModel *model, int mesh_id)
{
    const int vert_adr = model->mesh_vertadr[mesh_id];
    const int face_adr = model->mesh_faceadr[mesh_id];
    const int face_num = model->mesh_facenum[mesh_id];

    for (int f = 0; f < face_num; ++f) {
        const int *face = model->mesh_face + 3 * (face_adr + f);
        float      p[3][3];
        for (int c = 0; c < 3; ++c) {
            const float *v = model->mesh_vert + 3 * (vert_adr + face[c]);
            p[c][0] = v[0];
            p[c][1] = v[1];
            p[c][2] = v[2];
        }
        const float e1[3] = { p[1][0] - p[0][0], p[1][1] - p[0][1], p[1][2] - p[0][2] };
        const float e2[3] = { p[2][0] - p[0][0], p[2][1] - p[0][1], p[2][2] - p[0][2] };
        float       n[3]  = { e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2],
                              e1[0] * e2[1] - e1[1] * e2[0] };
        const float len   = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
        if (len > 0.0f) { n[0] /= len; n[1] /= len; n[2] /= len; }

        const uint32_t base = vertex_count(m);
        for (int c = 0; c < 3; ++c) push_vertex(m, p[c][0], p[c][1], p[c][2], n[0], n[1], n[2]);
        push_tri(m, base, base + 1, base + 2);
    }
}

} // namespace

bool build_geom_mesh(const mjModel *model, int geom_id, const TessOptions &opts, TriMesh *out)
{
    const int    type = model->geom_type[geom_id];
    const mjtNum *s   = model->geom_size + 3 * geom_id;
    const float  s0   = static_cast<float>(s[0]);
    const float  s1   = static_cast<float>(s[1]);
    const float  s2   = static_cast<float>(s[2]);

    switch (type) {
    case mjGEOM_PLANE:
        build_plane(out, s0 > 0 ? s0 : opts.plane_extent, s1 > 0 ? s1 : opts.plane_extent);
        return true;
    case mjGEOM_SPHERE:
        build_ellipsoid(out, s0, s0, s0, opts);
        return true;
    case mjGEOM_ELLIPSOID:
        build_ellipsoid(out, s0, s1, s2, opts);
        return true;
    case mjGEOM_CAPSULE:
        build_tube(out, s0, s1, opts);
        build_hemisphere(out, s0, s1, 1.0f, opts);
        build_hemisphere(out, s0, -s1, -1.0f, opts);
        return true;
    case mjGEOM_CYLINDER:
        build_tube(out, s0, s1, opts);
        build_disc(out, s0, s1, 1.0f, opts);
        build_disc(out, s0, -s1, -1.0f, opts);
        return true;
    case mjGEOM_BOX:
        build_box(out, s0, s1, s2);
        return true;
    case mjGEOM_MESH:
    case mjGEOM_SDF: {
        const int mesh_id = model->geom_dataid[geom_id];
        if (mesh_id < 0) return false;
        build_asset_mesh(out, model, mesh_id);
        return true;
    }
    default:
        return false; // hfield and the decor-only types
    }
}

} // namespace vr
