#ifndef QUAD_H
#define QUAD_H

#include "hittable.h"

#include <cmath>

/*
Planar geometry primitives.
- quad is a finite plane patch parameterized by origin Q and edge vectors (u, v).
- box(...) composes 6 quads into an axis-aligned rectangular solid.
- Used heavily in Cornell-box style scenes and area-light setups.
*/

class quad : public hittable {
  public:
    quad(const point3& Q, const vec3& u, const vec3& v, shared_ptr<material> mat)
        : Q(Q), u(u), v(v), mat(std::move(mat)) {
        auto n = cross(u, v);
        normal = unit_vector(n);
        D = dot(normal, Q);
        // Precompute helper for fast barycentric-like interior tests.
        w = n / dot(n, n);

        set_bounding_box();
    }

    bool hit(const ray& r, interval ray_t, hit_record& rec) const override {
        // Plane intersection first, then interior check in local (u,v) coordinates.
        auto denom = dot(normal, r.direction());

        if (std::fabs(denom) < 1e-8)
            return false;

        auto t = (D - dot(normal, r.origin())) / denom;
        if (!ray_t.contains(t))
            return false;

        auto intersection = r.at(t);
        vec3 planar_hitpt_vector = intersection - Q;
        auto alpha = dot(w, cross(planar_hitpt_vector, v));
        auto beta = dot(w, cross(u, planar_hitpt_vector));

        if (!is_interior(alpha, beta, rec))
            return false;

        rec.t = t;
        rec.p = intersection;
        rec.mat = mat;
        rec.set_face_normal(r, normal);

        return true;
    }

    virtual bool is_interior(double a, double b, hit_record& rec) const {
        // Base quad is a unit square in (a,b): [0,1] x [0,1].
        interval unit_interval = interval(0, 1);

        if (!unit_interval.contains(a) || !unit_interval.contains(b))
            return false;

        rec.u = a;
        rec.v = b;
        return true;
    }

    aabb bounding_box() const override { return bbox; }

  protected:
    virtual void set_bounding_box() {
        // Merge two diagonals to robustly bound all winding/order variants.
        auto bbox_diagonal1 = aabb(Q, Q + u + v);
        auto bbox_diagonal2 = aabb(Q + u, Q + v);
        bbox = aabb(bbox_diagonal1, bbox_diagonal2);
    }

  private:
    point3 Q;
    vec3 u, v;
    vec3 w;
    shared_ptr<material> mat;
    aabb bbox;
    vec3 normal;
    double D;
};


inline shared_ptr<hittable_list> box(const point3& a, const point3& b, shared_ptr<material> mat) {
    // Build an axis-aligned box from 6 quads with outward-facing windings.
    auto sides = make_shared<hittable_list>();

    auto min = point3(std::fmin(a.x(), b.x()), std::fmin(a.y(), b.y()), std::fmin(a.z(), b.z()));
    auto max = point3(std::fmax(a.x(), b.x()), std::fmax(a.y(), b.y()), std::fmax(a.z(), b.z()));

    auto dx = vec3(max.x() - min.x(), 0, 0);
    auto dy = vec3(0, max.y() - min.y(), 0);
    auto dz = vec3(0, 0, max.z() - min.z());

    sides->add(make_shared<quad>(point3(min.x(), min.y(), max.z()), dx, dy, mat));
    sides->add(make_shared<quad>(point3(max.x(), min.y(), max.z()), -dz, dy, mat));
    sides->add(make_shared<quad>(point3(max.x(), min.y(), min.z()), -dx, dy, mat));
    sides->add(make_shared<quad>(point3(min.x(), min.y(), min.z()), dz, dy, mat));
    sides->add(make_shared<quad>(point3(min.x(), max.y(), max.z()), dx, -dz, mat));
    sides->add(make_shared<quad>(point3(min.x(), min.y(), min.z()), dx, dz, mat));

    return sides;
}


#endif
