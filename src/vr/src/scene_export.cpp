/* SPDX-License-Identifier: MIT
 * Copyright (c) 2026 Vamsi Kalagaturu
 * See LICENSE for details. */

/* Converts an MJCF model into one .glb plus a manifest the headset client loads once.
 *
 * Geometry is baked per body in body-local coordinates: the client creates one object per body
 * and drives its pose from /vr/body_poses, so nothing about the body transforms is stored here.
 *
 * Axes: MuJoCo is Z-up right-handed, glTF is Y-up right-handed, so vertices are rotated by
 * -90 degrees about X on the way out ((x,y,z) -> (x,z,-y)). That keeps the .glb correct in any
 * standard glTF viewer, which is how this step gets verified. The client applies one further
 * fixed rotation to reconcile its importer's handedness flip with the live pose stream; see
 * GltfCorrection in the Unity project. */

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>

#include "vr/geom_mesh.hpp"
#include "vr/gltf_writer.hpp"

namespace {

struct Options
{
    std::string              mjcf;
    std::string              out_dir = ".";
    std::vector<int>         groups  = { 0, 1, 2 };
    vr::TessOptions   tess;
};

void usage(const char *argv0)
{
    std::fprintf(stderr,
                 "usage: %s <model.xml> [-o OUT_DIR] [--groups 0,1,2] [--segments N]\n"
                 "          [--rings N] [--plane-extent M]\n\n"
                 "Writes OUT_DIR/scene.glb and OUT_DIR/manifest.json.\n",
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

} // namespace

int main(int argc, char **argv)
{
    Options opt;
    if (!parse_args(argc, argv, &opt)) {
        usage(argv[0]);
        return 2;
    }

    char     error[1024] = "";
    mjModel *model       = mj_loadXML(opt.mjcf.c_str(), nullptr, error, sizeof(error));
    if (!model) {
        std::fprintf(stderr, "failed to load %s: %s\n", opt.mjcf.c_str(), error);
        return 1;
    }

    vr::GlbBuilder builder;
    std::vector<int>      body_node(model->nbody, -1);
    int                   exported_geoms  = 0;
    int                   invisible_geoms = 0;
    std::map<int, int>    skipped_types; // geom type -> count

    for (int b = 0; b < model->nbody; ++b) {
        /* One primitive per distinct colour so a body is a single mesh with few draw calls. */
        std::map<int, vr::Primitive> by_material;

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

            vr::TriMesh tri;
            if (!vr::build_geom_mesh(model, g, opt.tess, &tri)) {
                ++skipped_types[model->geom_type[g]];
                continue;
            }

            const int material = builder.add_material(colour);
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
            }
            for (uint32_t idx : tri.indices) prim.indices.push_back(base + idx);
            ++exported_geoms;
        }

        if (by_material.empty()) continue;

        vr::MeshGroup mesh;
        mesh.name = body_name(model, b);
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
    manifest << "  ]\n}\n";
    manifest.close();

    std::printf("%s: %d bodies, %d geoms exported\n", opt.mjcf.c_str(), model->nbody,
                exported_geoms);
    if (invisible_geoms) {
        std::printf("  skipped %d fully transparent geom(s)\n", invisible_geoms);
    }
    for (const auto &[type, count] : skipped_types) {
        std::printf("  skipped %d geom(s) of type %d (no static triangle form)\n", count, type);
    }
    if (model->ntex > 0) {
        std::printf("  note: %d texture(s) in the model; only flat material colours are "
                    "exported\n",
                    model->ntex);
    }
    std::printf("wrote %s and %s\n", glb_path.c_str(), manifest_path.c_str());

    mj_deleteModel(model);
    return 0;
}
