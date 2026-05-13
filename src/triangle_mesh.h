#ifndef TRIANGLE_MESH_H
#define TRIANGLE_MESH_H

#include "hittable.h"
#include "material.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

/*
Triangle mesh primitive backed by an internal BVH.
- Loads Wavefront OBJ vertex/face data.
- Optionally normalizes model size for predictable camera framing.
- Builds per-mesh BVH so triangle-heavy assets remain traceable on CPU.
*/

class triangle_mesh : public hittable {
  public:
    explicit triangle_mesh(shared_ptr<material> mat) : mat(std::move(mat)) {}

    bool load_wavefront_obj(const char* path, bool normalize_to_unit = true,
                            double target_extent = 2.0) {
        vertices.clear();
        indices.clear();
        tri_order.clear();
        nodes.clear();

        std::vector<point3> file_vertices;
        std::ifstream in(path);
        if (!in) {
            std::cerr << "ERROR: Could not open OBJ file: " << path << '\n';
            return false;
        }

        std::string line;
        while (std::getline(in, line)) {
            if (line.empty() || line[0] == '#') continue;
            std::istringstream ls(line);
            std::string tok;
            ls >> tok;
            if (tok == "v") {
                // Vertex position.
                double x, y, z;
                ls >> x >> y >> z;
                file_vertices.emplace_back(x, y, z);
            } else if (tok == "f") {
                // Face indices; polygon faces are triangulated as fan (v0, vk, vk+1).
                std::vector<int> corner;
                std::string corner_token;
                while (ls >> corner_token) {
                    std::istringstream ct(corner_token);
                    int vi = 0;
                    ct >> vi;
                    if (vi < 0) vi = int(file_vertices.size()) + vi + 1;
                    corner.push_back(vi - 1);
                }
                if (corner.size() < 3) continue;
                for (size_t k = 1; k + 1 < corner.size(); ++k) {
                    int i0 = corner[0];
                    int i1 = corner[k];
                    int i2 = corner[k + 1];
                    if (i0 < 0 || i1 < 0 || i2 < 0 || static_cast<size_t>(i0) >= file_vertices.size()
                        || static_cast<size_t>(i1) >= file_vertices.size()
                        || static_cast<size_t>(i2) >= file_vertices.size())
                        continue;
                    indices.push_back(static_cast<uint32_t>(i0));
                    indices.push_back(static_cast<uint32_t>(i1));
                    indices.push_back(static_cast<uint32_t>(i2));
                }
            }
        }

        if (indices.empty()) {
            std::cerr << "ERROR: No triangles found in OBJ (need faces).\n";
            return false;
        }

        vertices = std::move(file_vertices);

        if (normalize_to_unit) {
            // Center and uniformly scale mesh to roughly target_extent in its widest axis.
            aabb bb = compute_scene_bounds();
            point3 center((bb.x.min + bb.x.max) / 2, (bb.y.min + bb.y.max) / 2,
                          (bb.z.min + bb.z.max) / 2);
            auto ext_x = bb.x.size();
            auto ext_y = bb.y.size();
            auto ext_z = bb.z.size();
            auto mx = ext_x > ext_y ? ext_x : ext_y;
            mx = mx > ext_z ? mx : ext_z;
            if (mx <= 1e-12) mx = 1.0;
            auto s = target_extent / mx;
            for (auto& v : vertices) {
                v = (v - center) * s;
            }
        }

        scene_bounds = compute_scene_bounds();

        const size_t num_tris = indices.size() / 3;
        tri_order.resize(num_tris);
        std::iota(tri_order.begin(), tri_order.end(), 0);

        // Build acceleration structure over triangle index order.
        root_index = build_bvh(0, static_cast<int>(num_tris), 0);
        return true;
    }

    bool hit(const ray& r, interval ray_t, hit_record& rec) const override {
        if (nodes.empty())
            return false;
        return hit_bvh(root_index, r, ray_t, rec);
    }

    aabb bounding_box() const override { return scene_bounds; }

    size_t triangle_count() const { return indices.size() / 3; }

  private:
    std::vector<point3> vertices;
    std::vector<uint32_t> indices;
    shared_ptr<material> mat;
    aabb scene_bounds;

    struct BVHNode {
        aabb box;
        int left = -1;
        int right = -1;
        int first_tri = 0;
        int tri_count = 0;
    };

    std::vector<BVHNode> nodes;
    std::vector<size_t> tri_order;
    int root_index = 0;

    static constexpr int max_leaf_tris = 4;
    static constexpr int max_depth = 64;

    aabb compute_scene_bounds() const {
        aabb b = aabb::empty;
        const size_t nt = indices.size() / 3;
        for (size_t t = 0; t < nt; ++t)
            b = aabb(b, tri_bounds(t));
        return b;
    }

    aabb tri_bounds(size_t tri_id) const {
        uint32_t i0 = indices[3 * tri_id];
        uint32_t i1 = indices[3 * tri_id + 1];
        uint32_t i2 = indices[3 * tri_id + 2];
        const auto& v0 = vertices[i0];
        const auto& v1 = vertices[i1];
        const auto& v2 = vertices[i2];
        auto xmin = std::fmin(v0.x(), std::fmin(v1.x(), v2.x()));
        auto ymin = std::fmin(v0.y(), std::fmin(v1.y(), v2.y()));
        auto zmin = std::fmin(v0.z(), std::fmin(v1.z(), v2.z()));
        auto xmax = std::fmax(v0.x(), std::fmax(v1.x(), v2.x()));
        auto ymax = std::fmax(v0.y(), std::fmax(v1.y(), v2.y()));
        auto zmax = std::fmax(v0.z(), std::fmax(v1.z(), v2.z()));
        return aabb(point3(xmin, ymin, zmin), point3(xmax, ymax, zmax));
    }

    point3 tri_centroid(size_t tri_id) const {
        uint32_t i0 = indices[3 * tri_id];
        uint32_t i1 = indices[3 * tri_id + 1];
        uint32_t i2 = indices[3 * tri_id + 2];
        return (vertices[i0] + vertices[i1] + vertices[i2]) / 3.0;
    }

    int build_bvh(int start, int end, int depth) {
        int node_idx = static_cast<int>(nodes.size());
        nodes.push_back({});

        aabb box = aabb::empty;
        for (int i = start; i < end; ++i)
            box = aabb(box, tri_bounds(tri_order[static_cast<size_t>(i)]));

        nodes[node_idx].box = box;

        int count = end - start;
        if (count <= max_leaf_tris || depth >= max_depth) {
            // Leaf stores contiguous span in tri_order.
            nodes[node_idx].first_tri = start;
            nodes[node_idx].tri_count = count;
            return node_idx;
        }

        // Split triangles around median centroid along longest axis.
        int axis = box.longest_axis();
        auto begin = tri_order.begin() + start;
        auto mid_it = begin + count / 2;
        auto eend = tri_order.begin() + end;
        std::nth_element(begin, mid_it, eend, [&](size_t A, size_t B) {
            return tri_centroid(A)[axis] < tri_centroid(B)[axis];
        });
        int mid = start + count / 2;
        if (mid <= start) mid = start + 1;
        if (mid >= end) mid = end - 1;

        int left = build_bvh(start, mid, depth + 1);
        int right = build_bvh(mid, end, depth + 1);
        nodes[node_idx].left = left;
        nodes[node_idx].right = right;
        return node_idx;
    }

    bool ray_triangle(const ray& r, size_t tri_id, interval ray_t, hit_record& rec) const {
        uint32_t i0 = indices[3 * tri_id];
        uint32_t i1 = indices[3 * tri_id + 1];
        uint32_t i2 = indices[3 * tri_id + 2];
        const point3& a = vertices[i0];
        const point3& b = vertices[i1];
        const point3& c = vertices[i2];

        // Moller-Trumbore ray/triangle intersection.
        vec3 edge1 = b - a;
        vec3 edge2 = c - a;
        vec3 pvec = cross(r.direction(), edge2);
        double det = dot(edge1, pvec);
        if (std::fabs(det) < 1e-12) return false;
        double inv_det = 1.0 / det;

        vec3 tvec = r.origin() - a;
        double u = dot(tvec, pvec) * inv_det;
        if (u < 0.0 || u > 1.0) return false;

        vec3 qvec = cross(tvec, edge1);
        double v = dot(r.direction(), qvec) * inv_det;
        if (v < 0.0 || u + v > 1.0) return false;

        double t = dot(edge2, qvec) * inv_det;
        if (!ray_t.contains(t)) return false;

        vec3 outward_normal = unit_vector(cross(edge1, edge2));
        rec.t = t;
        rec.p = r.at(t);
        rec.mat = mat;
        rec.set_face_normal(r, outward_normal);
        rec.u = u;
        rec.v = v;
        return true;
    }

    bool hit_bvh(int node_idx, const ray& r, interval ray_t, hit_record& rec) const {
        const BVHNode& node = nodes[node_idx];
        if (!node.box.hit(r, ray_t)) return false;

        if (node.tri_count > 0) {
            hit_record temp;
            bool hit_any = false;
            auto closest = ray_t.max;
            for (int i = 0; i < node.tri_count; ++i) {
                size_t tid = tri_order[static_cast<size_t>(node.first_tri + i)];
                if (ray_triangle(r, tid, interval(ray_t.min, closest), temp)) {
                    hit_any = true;
                    closest = temp.t;
                    rec = temp;
                }
            }
            return hit_any;
        }

        bool hit_left = hit_bvh(node.left, r, ray_t, rec);
        // Tighten max-t so the right branch only searches for closer hits.
        bool hit_right =
            hit_bvh(node.right, r, interval(ray_t.min, hit_left ? rec.t : ray_t.max), rec);
        return hit_left || hit_right;
    }
};

#endif
