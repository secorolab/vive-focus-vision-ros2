/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Vamsi Kalagaturu
 * See LICENSE for details. */

/* Converts an MJCF model into one .glb plus a manifest the headset client loads once.
 *
 * Geometry is baked per body in body-local coordinates: the client creates one object per body
 * and drives its pose from /vive_vr/body_poses, so nothing about the body transforms is stored here.
 *
 * Axes: MuJoCo is Z-up right-handed, glTF is Y-up right-handed, so vertices are rotated by
 * -90 degrees about X on the way out ((x,y,z) -> (x,z,-y)). That keeps the .glb correct in any
 * standard glTF viewer, which is how this step gets verified. The client applies one further
 * fixed rotation to reconcile its importer's handedness flip with the live pose stream; see
 * GltfCorrection in the Unity project. */

#include <algorithm>
#include <array>
#include <cstdio>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <set>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>

#include "vive_vr_ros2/geom_mesh.hpp"
#include "vive_vr_ros2/gltf_writer.hpp"

namespace {

struct Options
{
    std::string              mjcf;
    // Empty means ~/.cache/vive_vr_ros2/scenes/<model stem>: writing a 7 MB glb into whatever
    // directory the command was run from is never what was wanted.
    std::string              out_dir;
    std::vector<int>         groups  = { 0, 1, 2 };
    float                    sky_radius = 30.0f;
    bool                     export_ground = false;
    bool                     export_sky    = false;
    /* A headset renders this, and a scene composed from someone else's assets carries things it
     * does not need: a robot that is not being teleoperated, and textures authored for a desktop
     * renderer. Both cost frame rate. */
    std::vector<std::string> exclude;
    int                      max_texture = 0; // 0 keeps the source resolution
    vive_vr_ros2::TessOptions   tess;
};

void usage(const char *argv0)
{
    std::fprintf(stderr,
                 "usage: %s <model.xml> [-o OUT_DIR] [--groups 0,1,2] [--segments N]\n"
                 "          [--rings N] [--plane-extent M] [--sky-radius M]\n"
                 "          [--export-ground] [--export-sky]\n\n"
                 "Writes OUT_DIR/scene.glb and OUT_DIR/manifest.json.\n"
                 "OUT_DIR defaults to ~/.cache/vive_vr_ros2/scenes/<model>.\n",
                 argv0);
}

bool parse_args(int argc, char **argv, Options *opt)
{
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        const bool        has_next = (i + 1 < argc);
        if (a == "-h" || a == "--help") return false;
        if (a == "-o" && has_next) {
            opt->out_dir = argv[++i];
        } else if (a == "--groups" && has_next) {
            opt->groups.clear();
            std::string spec = argv[++i];
            size_t      pos  = 0;
            while (!spec.empty()) {
                pos                    = spec.find(',');
                const std::string head = spec.substr(0, pos);
                if (!head.empty()) opt->groups.push_back(std::stoi(head));
                if (pos == std::string::npos) break;
                spec = spec.substr(pos + 1);
            }
        } else if (a == "--segments" && has_next) {
            opt->tess.segments = std::stoi(argv[++i]);
        } else if (a == "--rings" && has_next) {
            opt->tess.rings = std::stoi(argv[++i]);
        } else if (a == "--exclude" && has_next) {
            std::string spec = argv[++i];
            size_t      pos;
            while (!spec.empty()) {
                pos                    = spec.find(',');
                const std::string head = spec.substr(0, pos);
                if (!head.empty()) opt->exclude.push_back(head);
                if (pos == std::string::npos) break;
                spec = spec.substr(pos + 1);
            }
        } else if (a == "--max-texture" && has_next) {
            opt->max_texture = std::stoi(argv[++i]);
        } else if (a == "--export-sky") {
            opt->export_sky = true;
        } else if (a == "--export-ground") {
            opt->export_ground = true;
        } else if (a == "--sky-radius" && has_next) {
            opt->sky_radius = std::stof(argv[++i]);
        } else if (a == "--plane-extent" && has_next) {
            opt->tess.plane_extent = std::stof(argv[++i]);
        } else if (!a.empty() && a[0] != '-' && opt->mjcf.empty()) {
            opt->mjcf = a;
        } else {
            std::fprintf(stderr, "unrecognised argument: %s\n", a.c_str());
            return false;
        }
    }
    return !opt->mjcf.empty();
}

std::string body_name(const mjModel *m, int body_id)
{
    const char *n = mj_id2name(m, mjOBJ_BODY, body_id);
    return n ? n : ("body_" + std::to_string(body_id));
}

std::array<float, 4> geom_colour(const mjModel *m, int geom_id)
{
    const int matid = m->geom_matid[geom_id];
    const float *src = (matid >= 0) ? (m->mat_rgba + 4 * matid) : (m->geom_rgba + 4 * geom_id);
    return { src[0], src[1], src[2], src[3] };
}

std::array<float, 3> texel(const mjModel *m, int tex, int row, int col)
{
    const int  w  = m->tex_width[tex];
    const int  nc = m->tex_nchannel[tex];
    const auto a  = m->tex_adr[tex] + static_cast<mjtSize>(row) * w * nc + col * nc;
    return { m->tex_data[a] / 255.0f, m->tex_data[a + 1] / 255.0f, m->tex_data[a + 2] / 255.0f };
}

/**
 * The two colours of a checker texture, sampled at the centres of adjacent squares.
 *
 * Not the darkest and lightest pixels: MuJoCo's builtin checker draws a light `mark` along the
 * square edges, and picking extremes returns that border instead of either square.
 */
void checker_colours(const mjModel *m, int tex, std::array<float, 3> *a, std::array<float, 3> *b)
{
    const int h = m->tex_height[tex], w = m->tex_width[tex];
    *a = texel(m, tex, h / 4, w / 4);
    *b = texel(m, tex, h / 4, (3 * w) / 4);

    // Adjacent squares should differ; if they do not, the sample landed inside one square.
    const float diff = std::fabs((*a)[0] - (*b)[0]) + std::fabs((*a)[1] - (*b)[1]) +
                       std::fabs((*a)[2] - (*b)[2]);
    if (diff < 0.02f) *b = texel(m, tex, (3 * h) / 4, w / 4);
}

/** Texture id in the base-colour role, or -1. */
/**
 * Copies one of MuJoCo's textures into the .glb, widening to RGBA. MuJoCo stores pixels with one,
 * three or four channels; glTF wants a PNG, and the writer encodes from RGBA.
 */
int add_mj_texture(const mjModel *m, int texid, int max_size, vive_vr_ros2::GlbBuilder *builder)
{
    const int w = m->tex_width[texid];
    const int h = m->tex_height[texid];
    const int c = m->tex_nchannel[texid];
    if (w <= 0 || h <= 0 || c <= 0) return -1;

    /* Halve until it fits. Powers of two keep the box filter exact and keep the result
     * mipmappable, which matters more on a headset than the lost detail does. */
    int step = 1;
    while (max_size > 0 && (w / step > max_size || h / step > max_size)) step *= 2;
    const int ow = std::max(1, w / step);
    const int oh = std::max(1, h / step);

    vive_vr_ros2::Texture tex;
    tex.width  = ow;
    tex.height = oh;
    tex.rgba.resize(static_cast<size_t>(ow) * oh * 4);

    const mjtByte *src = m->tex_data + m->tex_adr[texid];
    for (int y = 0; y < oh; ++y) {
        for (int x = 0; x < ow; ++x) {
            int sum[4] = { 0, 0, 0, 0 }, n = 0;
            for (int dy = 0; dy < step; ++dy) {
                for (int dx = 0; dx < step; ++dx) {
                    const int sx = x * step + dx, sy = y * step + dy;
                    if (sx >= w || sy >= h) continue;
                    const mjtByte *p = src + (static_cast<size_t>(sy) * w + sx) * c;
                    sum[0] += p[0];
                    sum[1] += c >= 3 ? p[1] : p[0]; // grey textures replicate their one channel
                    sum[2] += c >= 3 ? p[2] : p[0];
                    sum[3] += c == 4 ? p[3] : 255;
                    ++n;
                }
            }
            const size_t o = (static_cast<size_t>(y) * ow + x) * 4;
            for (int k = 0; k < 4; ++k) {
                tex.rgba[o + k] = static_cast<uint8_t>(n ? sum[k] / n : 0);
            }
        }
    }

    const char *name = mj_id2name(m, mjOBJ_TEXTURE, texid);
    return builder->add_texture(name ? name : ("tex_" + std::to_string(texid)), std::move(tex));
}

int base_texture(const mjModel *m, int matid)
{
    if (matid < 0) return -1;
    const int rgb  = m->mat_texid[matid * mjNTEXROLE + mjTEXROLE_RGB];
    const int rgba = m->mat_texid[matid * mjNTEXROLE + mjTEXROLE_RGBA];
    return rgb >= 0 ? rgb : rgba;
}

void push_quad(vive_vr_ros2::Primitive *prim, const float corners[4][3])
{
    const auto base = static_cast<uint32_t>(prim->positions.size() / 3);
    for (int i = 0; i < 4; ++i) {
        prim->positions.insert(prim->positions.end(),
                               { corners[i][0], corners[i][1], corners[i][2] });
        prim->normals.insert(prim->normals.end(), { 0.0f, 1.0f, 0.0f }); // glTF Y is up
    }
    prim->indices.insert(prim->indices.end(),
                         { base, base + 1, base + 2, base, base + 2, base + 3 });
}

/** Tiles a plane geom into alternating squares, in glTF axes, body-local. */
void add_checker_plane(const mjModel *m, int geom_id, const Options &opt, vive_vr_ros2::GlbBuilder *builder,
                       std::map<int, vive_vr_ros2::Primitive> *by_material,
                       const std::array<float, 4> &dark, const std::array<float, 4> &light)
{
    const mjtNum *s  = m->geom_size + 3 * geom_id;
    const float   hx = s[0] > 0 ? static_cast<float>(s[0]) : opt.tess.plane_extent;
    const float   hy = s[1] > 0 ? static_cast<float>(s[1]) : opt.tess.plane_extent;
    /* Square size. MuJoCo renders the checker from a mipmapped texture, so it can afford tiny
     * squares; flat quads cannot, and at a grazing angle a fine grid turns into moire stripes.
     * Keep the count low enough that the floor still reads as a checkerboard from across the
     * scene. */
    const int   max_cells = 48;
    const float step      = std::max({ s[2] > 0 ? static_cast<float>(s[2]) : 0.5f,
                                       2.0f * hx / max_cells, 2.0f * hy / max_cells });

    const int nx = std::max(2, static_cast<int>(std::round(2.0f * hx / step)));
    const int ny = std::max(2, static_cast<int>(std::round(2.0f * hy / step)));
    const float dx = 2.0f * hx / nx, dy = 2.0f * hy / ny;

    const int dark_mat  = builder->add_material(dark);
    const int light_mat = builder->add_material(light);
    (*by_material)[dark_mat].material  = dark_mat;
    (*by_material)[light_mat].material = light_mat;

    const mjtNum *gp = m->geom_pos + 3 * geom_id;
    for (int i = 0; i < nx; ++i) {
        for (int j = 0; j < ny; ++j) {
            const float x0 = -hx + i * dx, x1 = x0 + dx;
            const float y0 = -hy + j * dy, y1 = y0 + dy;
            /* MuJoCo plane lies in local xy with +z up; in glTF axes that is xz with +y up. */
            const float z  = static_cast<float>(gp[2]);
            const float corners[4][3] = { { x0 + (float)gp[0], z, -(y0 + (float)gp[1]) },
                                          { x1 + (float)gp[0], z, -(y0 + (float)gp[1]) },
                                          { x1 + (float)gp[0], z, -(y1 + (float)gp[1]) },
                                          { x0 + (float)gp[0], z, -(y1 + (float)gp[1]) } };
            push_quad(&(*by_material)[((i + j) % 2) ? light_mat : dark_mat], corners);
        }
    }
}

/** A banded dome carrying the skybox's vertical gradient, so the world is not floating in void. */
void add_sky_dome(const mjModel *m, int tex, vive_vr_ros2::GlbBuilder *builder, float radius)
{
    /* A full sphere, not a hemisphere: stopping at the horizon leaves a hard seam wherever the
     * ground plane ends. More bands than the eye can pick out as steps. */
    const int   bands = 32, segments = 32;
    const int   rows  = m->tex_height[tex];
    vive_vr_ros2::MeshGroup sky;
    sky.name = "sky";

    for (int b = 0; b < bands; ++b) {
        /* Sample the skybox top-to-bottom and give each band a flat colour: a stepped gradient
         * rather than a texture, which needs no sampler or UVs. */
        const int   row = std::min(rows - 1, b * rows / bands);
        const auto  c   = texel(m, tex, row, m->tex_width[tex] / 2);
        const int   mat = builder->add_material({ c[0], c[1], c[2], 1.0f });

        vive_vr_ros2::Primitive prim;
        prim.material = mat;

        /* Sweep the full pole-to-pole range so the dome closes underneath the ground. */
        const float t0 = static_cast<float>(b) / bands, t1 = static_cast<float>(b + 1) / bands;
        const float phi0 = (0.5f - t0) * static_cast<float>(mjPI);
        const float phi1 = (0.5f - t1) * static_cast<float>(mjPI);
        for (int s = 0; s < segments; ++s) {
            const float a0 = 2.0f * static_cast<float>(mjPI) * s / segments;
            const float a1 = 2.0f * static_cast<float>(mjPI) * (s + 1) / segments;
            const float ring[4][3] = {
                { radius * std::cos(phi0) * std::cos(a0), radius * std::sin(phi0),
                  radius * std::cos(phi0) * std::sin(a0) },
                { radius * std::cos(phi0) * std::cos(a1), radius * std::sin(phi0),
                  radius * std::cos(phi0) * std::sin(a1) },
                { radius * std::cos(phi1) * std::cos(a1), radius * std::sin(phi1),
                  radius * std::cos(phi1) * std::sin(a1) },
                { radius * std::cos(phi1) * std::cos(a0), radius * std::sin(phi1),
                  radius * std::cos(phi1) * std::sin(a0) }
            };
            const auto base = static_cast<uint32_t>(prim.positions.size() / 3);
            for (int i = 0; i < 4; ++i) {
                prim.positions.insert(prim.positions.end(),
                                      { ring[i][0], ring[i][1], ring[i][2] });
                // Normals point inward: the viewer is inside the dome.
                const float n = std::sqrt(ring[i][0] * ring[i][0] + ring[i][1] * ring[i][1] +
                                          ring[i][2] * ring[i][2]);
                prim.normals.insert(prim.normals.end(),
                                    { -ring[i][0] / n, -ring[i][1] / n, -ring[i][2] / n });
            }
            prim.indices.insert(prim.indices.end(),
                                { base, base + 2, base + 1, base, base + 3, base + 2 });
        }
        sky.primitives.push_back(std::move(prim));
    }
    builder->add_mesh_node(std::move(sky));
}

} // namespace

int main(int argc, char **argv)
{
    Options opt;
    if (!parse_args(argc, argv, &opt)) {
        usage(argv[0]);
        return 2;
    }

    if (opt.out_dir.empty()) {
        const char *home = std::getenv("HOME");
        if (!home) {
            std::fprintf(stderr, "no HOME for the default output directory; pass -o\n");
            return 2;
        }
        // Menagerie names every world <robot>/scene.xml, so the stem alone collides.
        const std::filesystem::path mjcf(opt.mjcf);
        const std::string           stem = mjcf.stem().string();
        const std::string name = stem == "scene" ? mjcf.parent_path().filename().string() : stem;
        opt.out_dir = std::string(home) + "/.cache/vive_vr_ros2/scenes/" + name;
        std::fprintf(stderr, "writing to %s\n", opt.out_dir.c_str());
    }

    char     error[1024] = "";
    mjModel *model       = mj_loadXML(opt.mjcf.c_str(), nullptr, error, sizeof(error));
    if (!model) {
        std::fprintf(stderr, "failed to load %s: %s\n", opt.mjcf.c_str(), error);
        return 1;
    }

    /* Poses at the loaded state: written into both the glb nodes and the manifest. */
    mjData *pose_data = mj_makeData(model);
    mj_forward(model, pose_data);

    vive_vr_ros2::GlbBuilder builder;
    std::vector<int>      body_node(model->nbody, -1);
    int                   exported_geoms  = 0;
    int                   invisible_geoms = 0;
    int                   ground_planes   = 0;
    std::set<int>         textured;       // MuJoCo texture ids that made it into the file
    std::set<int>         not_2d;         // referenced, but a cube or skybox: not a surface map
    std::set<int>         no_uvs;         // referenced by geometry that carries no coordinates
    std::map<int, int>    skipped_types; // geom type -> count

    int excluded_bodies = 0;

    for (int b = 0; b < model->nbody; ++b) {
        /* Whole bodies the scene does not need. The manifest still lists them, so the index a
         * pose carries keeps matching; they simply have no geometry to draw. */
        const std::string bname = body_name(model, b);
        if (std::any_of(opt.exclude.begin(), opt.exclude.end(), [&](const std::string &p) {
                return bname.rfind(p, 0) == 0;
            })) {
            ++excluded_bodies;
            continue;
        }

        /* One primitive per distinct colour so a body is a single mesh with few draw calls. */
        std::map<int, vive_vr_ros2::Primitive> by_material;

        const int geom_begin = model->body_geomadr[b];
        const int geom_end   = geom_begin + model->body_geomnum[b];
        for (int g = geom_begin; g < geom_end; ++g) {
            if (std::find(opt.groups.begin(), opt.groups.end(), model->geom_group[g]) ==
                opt.groups.end()) {
                continue;
            }

            const auto colour = geom_colour(model, g);
            if (colour[3] <= 0.0f) {
                /* MuJoCo hides fully transparent geoms; they are collision shapes that would
                 * otherwise cost triangles and alpha-blending work on the headset GPU. */
                ++invisible_geoms;
                continue;
            }

            /* An unbounded plane is MuJoCo's ground, which the client draws better than a baked
             * mesh can: a real material with mipmaps, no moire at grazing angles, and it can
             * receive shadows. Finite planes are geometry someone modelled, so they stay. */
            const mjtNum *psz = model->geom_size + 3 * g;
            if (!opt.export_ground && model->geom_type[g] == mjGEOM_PLANE &&
                (psz[0] <= 0 || psz[1] <= 0)) {
                ++ground_planes;
                continue;
            }

            /* A textured finite plane is still a checkerboard; tile it into squares of its two
             * colours rather than exporting the texture and the UVs it would need. */
            const int tex = base_texture(model, model->geom_matid[g]);
            if (model->geom_type[g] == mjGEOM_PLANE && tex >= 0 &&
                model->tex_type[tex] == mjTEXTURE_2D) {
                std::array<float, 3> dark{ 0.2f, 0.2f, 0.2f }, light{ 0.8f, 0.8f, 0.8f };
                checker_colours(model, tex, &dark, &light);
                add_checker_plane(model, g, opt, &builder, &by_material,
                                  { dark[0], dark[1], dark[2], 1.0f },
                                  { light[0], light[1], light[2], 1.0f });
                ++exported_geoms;
                continue;
            }

            vive_vr_ros2::TriMesh tri;
            if (!vive_vr_ros2::build_geom_mesh(model, g, opt.tess, &tri)) {
                ++skipped_types[model->geom_type[g]];
                continue;
            }

            /* A 2D texture on a mesh that carries texture coordinates. Anything else - a skybox,
             * a procedural checker on a primitive, a mesh with no UVs - stays a flat colour,
             * because without coordinates there is nowhere to put the image. */
            int   tex_index   = -1;
            float uv_scale[2] = { 1.0f, 1.0f };
            if (tex >= 0 && model->tex_type[tex] != mjTEXTURE_2D) {
                not_2d.insert(tex);
            } else if (tex >= 0 && tri.uvs.empty()) {
                no_uvs.insert(tex);
            }
            if (!tri.uvs.empty() && tex >= 0 && model->tex_type[tex] == mjTEXTURE_2D) {
                tex_index = add_mj_texture(model, tex, opt.max_texture, &builder);
                if (tex_index >= 0) {
                    textured.insert(tex);
                    const int matid = model->geom_matid[g];
                    if (matid >= 0 && model->geom_type[g] != mjGEOM_MESH) {
                        uv_scale[0] = model->mat_texrepeat[2 * matid];
                        uv_scale[1] = model->mat_texrepeat[2 * matid + 1];
                    }
                }
            }

            /* Texturing replaces the colour rather than tinting it: MuJoCo's rgba on a textured
             * material is the fallback, and multiplying by it darkens every texture we export. */
            const std::array<float, 4> factor =
              tex_index >= 0 ? std::array<float, 4>{ 1.0f, 1.0f, 1.0f, colour[3] } : colour;

            /* MuJoCo's own surface response rather than one matte constant for everything, which
             * made metal, ceramic and glass indistinguishable. mat_metallic/mat_roughness are the
             * direct PBR fields; where a model predates them they are -1 and the older
             * shininess/specular pair stands in. */
            float metallic = 0.0f, roughness = 0.8f;
            if (const int matid = model->geom_matid[g]; matid >= 0) {
                metallic  = model->mat_metallic[matid] >= 0.0f ? model->mat_metallic[matid]
                                                               : 0.0f;
                roughness = model->mat_roughness[matid] >= 0.0f
                              ? model->mat_roughness[matid]
                              : std::clamp(1.0f - model->mat_shininess[matid], 0.05f, 1.0f);
            }

            const int material = builder.add_material(factor, tex_index, metallic, roughness);
            auto     &prim     = by_material[material];
            prim.material      = material;

            const mjtNum *gp = model->geom_pos + 3 * g;
            const mjtNum *gq = model->geom_quat + 4 * g;
            const auto    base = static_cast<uint32_t>(prim.positions.size() / 3);

            const size_t vertices = tri.positions.size() / 3;
            for (size_t v = 0; v < vertices; ++v) {
                mjtNum p[3] = { tri.positions[3 * v], tri.positions[3 * v + 1],
                                tri.positions[3 * v + 2] };
                mjtNum n[3] = { tri.normals[3 * v], tri.normals[3 * v + 1],
                                tri.normals[3 * v + 2] };
                mjtNum pr[3], nr[3];
                mju_rotVecQuat(pr, p, gq);
                mju_rotVecQuat(nr, n, gq);
                mju_addTo3(pr, gp);

                // MuJoCo Z-up -> glTF Y-up.
                prim.positions.insert(prim.positions.end(),
                                      { static_cast<float>(pr[0]), static_cast<float>(pr[2]),
                                        static_cast<float>(-pr[1]) });
                prim.normals.insert(prim.normals.end(),
                                    { static_cast<float>(nr[0]), static_cast<float>(nr[2]),
                                      static_cast<float>(-nr[1]) });

                /* Every vertex of a primitive needs a coordinate or none does, so an untextured
                 * geom sharing a material with a textured one still contributes a pair. */
                if (tex_index >= 0 && 2 * v + 1 < tri.uvs.size()) {
                    /* glTF has no texrepeat, so the tiling is baked into the coordinates. For a
                     * primitive those arrive in metres, which is what makes texuniform's
                     * repeats-per-metre meaningful; a mesh asset's own coordinates are already
                     * normalised and must not be scaled. */
                    prim.uvs.insert(prim.uvs.end(),
                                    { tri.uvs[2 * v] * uv_scale[0], tri.uvs[2 * v + 1] * uv_scale[1] });
                } else if (!prim.uvs.empty()) {
                    prim.uvs.insert(prim.uvs.end(), { 0.0f, 0.0f });
                }
            }
            for (uint32_t idx : tri.indices) prim.indices.push_back(base + idx);
            ++exported_geoms;
        }

        if (by_material.empty()) continue;

        vive_vr_ros2::MeshGroup mesh;
        mesh.name = body_name(model, b);

        /* The body's loaded pose, rotated into glTF axes the same way its vertices were, so the
         * file renders assembled without needing the manifest or a pose stream. */
        const mjtNum *bp = pose_data->xpos + 3 * b;
        const mjtNum *bq = pose_data->xquat + 4 * b; // (w, x, y, z)
        mesh.translation  = { static_cast<float>(bp[0]), static_cast<float>(bp[2]),
                              static_cast<float>(-bp[1]) };
        /* Rotating the pose by the same -90 degrees about X: for a quaternion that is
         * q_gltf = r * q * r^-1 with r the half-turn's quaternion, which for this axis reduces
         * to the same component shuffle used for the vertices. */
        mjtNum rq[4] = { std::cos(-mjPI / 4), std::sin(-mjPI / 4), 0, 0 };
        mjtNum rq_inv[4], tmp[4], out[4];
        mju_negQuat(rq_inv, rq);
        mju_mulQuat(tmp, rq, bq);
        mju_mulQuat(out, tmp, rq_inv);
        mesh.rotation = { static_cast<float>(out[1]), static_cast<float>(out[2]),
                          static_cast<float>(out[3]), static_cast<float>(out[0]) };
        for (auto &[material, prim] : by_material) {
            (void)material;
            mesh.primitives.push_back(std::move(prim));
        }
        body_node[b] = builder.add_mesh_node(std::move(mesh));
    }


    std::error_code ec;
    std::filesystem::create_directories(opt.out_dir, ec);
    if (ec) {
        std::fprintf(stderr, "cannot create %s: %s\n", opt.out_dir.c_str(),
                     ec.message().c_str());
        mj_deleteModel(model);
        return 1;
    }

    /* MuJoCo's skybox is a texture on the renderer, not geometry, so nothing of it survives into
     * glTF on its own; a dome carrying its gradient is the equivalent a viewer can show. */
    int sky_tex = -1;
    for (int t = 0; opt.export_sky && t < model->ntex; ++t) {
        if (model->tex_type[t] == mjTEXTURE_SKYBOX) { sky_tex = t; break; }
    }
    if (sky_tex >= 0) add_sky_dome(model, sky_tex, &builder, opt.sky_radius);

    /* MuJoCo lights, so a viewer lights the scene the way the simulator does rather than
     * inventing its own. Positions and directions go through the same Z-up to Y-up rotation as
     * the geometry. */
    int exported_lights = 0;
    for (int l = 0; l < model->nlight; ++l) {
        if (!model->light_active[l]) continue;
        vive_vr_ros2::Light light;
        light.type = model->light_type[l] == mjLIGHT_DIRECTIONAL  ? "directional"
                     : model->light_type[l] == mjLIGHT_SPOT       ? "spot"
                                                                  : "point";
        const mjtNum *p = pose_data->light_xpos + 3 * l;
        const mjtNum *d = pose_data->light_xdir + 3 * l;
        light.position  = { static_cast<float>(p[0]), static_cast<float>(p[2]),
                            static_cast<float>(-p[1]) };
        light.direction = { static_cast<float>(d[0]), static_cast<float>(d[2]),
                            static_cast<float>(-d[1]) };
        /* glTF measures directional lights in lux and the others in candela, while MuJoCo's
         * diffuse is a plain 0..1 weight. Split it: the largest channel becomes the strength and
         * the rest becomes hue, so a light at full diffuse is one ordinary sun rather than three
         * and a dim light stays dim. Ignoring the weight blew out every scene with more than one
         * light in it. */
        const float *diff   = model->light_diffuse + 3 * l;
        const float  weight = std::max({ diff[0], diff[1], diff[2] });
        light.colour        = weight > 0.0f
                                ? std::array<float, 3>{ diff[0] / weight, diff[1] / weight,
                                                        diff[2] / weight }
                                : std::array<float, 3>{ 1.0f, 1.0f, 1.0f };
        light.intensity     = (light.type == "directional" ? 1.0f : 15.0f) * weight;
        builder.add_light(light);
        ++exported_lights;
    }

    const std::string glb_path      = opt.out_dir + "/scene.glb";
    const std::string manifest_path = opt.out_dir + "/manifest.json";

    if (!builder.write(glb_path)) {
        std::fprintf(stderr, "failed to write %s\n", glb_path.c_str());
        mj_deleteModel(model);
        return 1;
    }

    std::ofstream manifest(manifest_path);
    if (!manifest) {
        std::fprintf(stderr, "failed to write %s\n", manifest_path.c_str());
        mj_deleteModel(model);
        return 1;
    }
    manifest << "{\n  \"generator\": \"vr scene_export\",\n"
             << "  \"model\": \"" << opt.mjcf << "\",\n"
             << "  \"mesh\": \"scene.glb\",\n"
             << "  \"axis_convention\": \"gltf_y_up_from_mujoco_z_up\",\n"
             << "  \"nbody\": " << model->nbody << ",\n"
             << "  \"bodies\": [\n";
    for (int b = 0; b < model->nbody; ++b) {
        manifest << "    {\"name\": \"" << body_name(model, b) << "\", \"node\": " << body_node[b]
                 << "}" << (b + 1 < model->nbody ? "," : "") << "\n";
    }
    manifest << "  ],\n";

    /* The pose each body sits at before anything streams. Without it a client shows every body
     * stacked at the origin until the first update, which for a robot looks like a pile of
     * disconnected parts rather than an arm. */
    manifest << "  \"initial_poses\": [\n";
    for (int b = 0; b < model->nbody; ++b) {
        const mjtNum *p = pose_data->xpos + 3 * b;
        const mjtNum *q = pose_data->xquat + 4 * b; // MuJoCo order is (w, x, y, z)
        manifest << "    {\"p\": [" << p[0] << ", " << p[1] << ", " << p[2] << "], \"q\": ["
                 << q[1] << ", " << q[2] << ", " << q[3] << ", " << q[0] << "]}"
                 << (b + 1 < model->nbody ? "," : "") << "\n";
    }
    manifest << "  ]\n}\n";
    manifest.close();
    mj_deleteData(pose_data);

    std::printf("%s: %ld bodies, %ld geoms exported\n", opt.mjcf.c_str(),
                static_cast<long>(model->nbody), static_cast<long>(exported_geoms));
    if (ground_planes) {
        std::printf("  skipped %d unbounded ground plane(s); the client draws the floor\n",
                    ground_planes);
    }
    if (invisible_geoms) {
        std::printf("  skipped %d fully transparent geom(s)\n", invisible_geoms);
    }
    for (const auto &[type, count] : skipped_types) {
        std::printf("  skipped %d geom(s) of type %d (no static triangle form)\n", count, type);
    }
    if (model->ntex > 0) {
        /* Against what the exported geometry actually asked for. Counting against every texture
         * in the model reported a shortfall made of skyboxes and textures nothing here draws,
         * which reads as missing texture on a visible surface when none is. */
        const size_t wanted = textured.size() + not_2d.size() + no_uvs.size();
        std::printf("  %d of %ld texture(s) the exported geometry uses\n",
                    static_cast<int>(textured.size()), static_cast<long>(wanted));
        if (!no_uvs.empty()) {
            std::printf("  %d fell back to flat colour: no texture coordinates on the geometry\n",
                        static_cast<int>(no_uvs.size()));
        }
        if (!not_2d.empty()) {
            std::printf("  %d skipped: a cube or skybox texture is not a surface map\n",
                        static_cast<int>(not_2d.size()));
        }
    }
    std::printf("wrote %s and %s\n", glb_path.c_str(), manifest_path.c_str());

    mj_deleteModel(model);
    return 0;
}
